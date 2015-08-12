/*
 *  dir.c
 *
 *  Copyright (C) 1995, 1996 by Paal-Kr. Engstad and Volker Lendecke
 *  Copyright (C) 1997 by Volker Lendecke
 *
 *  Copyright (C) 2008-2009 ARM Limited
 *
 *  Please add a note about your changes to vmfs_ in the ChangeLog file.
 */

#include <linux/time.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/net.h>
#include <linux/sched.h>
#include <linux/version.h>

#include "vmfs_fs.h"
#include "vmfs_mount.h"
#include "vmfsno.h"

#include "vmfs_debug.h"
#include "proto.h"

static int vmfs_readdir(struct file *, struct dir_context *);
static int vmfs_dir_open(struct inode *, struct file *);

static struct dentry *vmfs_lookup(struct inode *, struct dentry *,
				  unsigned int flags);
static int vmfs_create(struct inode *, struct dentry *, umode_t,
		       bool excl);
static int vmfs_mkdir(struct inode *, struct dentry *, umode_t);
static int vmfs_rmdir(struct inode *, struct dentry *);
static int vmfs_unlink(struct inode *, struct dentry *);
static int vmfs_rename(struct inode *, struct dentry *,
		       struct inode *, struct dentry *);
static int vmfs_make_node(struct inode *, struct dentry *, umode_t, dev_t);
static int vmfs_link(struct dentry *, struct inode *, struct dentry *);

const struct file_operations vmfs_dir_operations = {
	.read = generic_read_dir,
	.iterate = vmfs_readdir,
	.unlocked_ioctl = vmfs_unlocked_ioctl,
	.open = vmfs_dir_open,
};

const struct inode_operations vmfs_dir_inode_operations = {
	.create = vmfs_create,
	.lookup = vmfs_lookup,
	.unlink = vmfs_unlink,
	.mkdir = vmfs_mkdir,
	.rmdir = vmfs_rmdir,
	.rename = vmfs_rename,
	.getattr = vmfs_getattr,
	.setattr = vmfs_notify_change,
};

const struct inode_operations vmfs_dir_inode_operations_unix = {
	.create = vmfs_create,
	.lookup = vmfs_lookup,
	.unlink = vmfs_unlink,
	.mkdir = vmfs_mkdir,
	.rmdir = vmfs_rmdir,
	.rename = vmfs_rename,
	.getattr = vmfs_getattr,
	.setattr = vmfs_notify_change,
	.symlink = vmfs_symlink,
	.mknod = vmfs_make_node,
	.link = vmfs_link,
};

/*
 * Read a directory, using filldir to fill the dirent memory.
 * vmfs_proc_readdir does the actual reading from the vmfs server.
 *
 * The cache code is almost directly taken from ncpfs
 */
static int vmfs_readdir(struct file *filp, struct dir_context *ctx)
{
	struct dentry *dentry = filp->f_path.dentry;
	struct inode *dir = dentry->d_inode;
	struct vmfs_sb_info *server = server_from_dentry(dentry);
	union vmfs_dir_cache *cache = NULL;
	struct vmfs_cache_control ctl;
	struct page *page = NULL;
	int result;

	ctl.page = NULL;
	ctl.cache = NULL;

	VERBOSE("reading %s/%s, f_pos=%d\n",
		DENTRY_PATH(dentry), (int)filp->f_pos);

	result = 0;

	mutex_lock(&vmfs_mutex);

	if (!dir_emit_dots(filp, ctx))
		return 0;

	/*
	 * Make sure our inode is up-to-date.
	 */
	result = vmfs_revalidate_inode(dentry);
	if (result)
		goto out;

	page = grab_cache_page(&dir->i_data, 0);
	if (!page)
		goto read_really;

	ctl.cache = cache = kmap(page);
	ctl.head = cache->head;

	if (!PageUptodate(page) || !ctl.head.eof) {
		VERBOSE("%s/%s, page uptodate=%d, eof=%d\n",
			DENTRY_PATH(dentry), PageUptodate(page), ctl.head.eof);
		goto init_cache;
	}

	if (ctx->pos == 2) {
		if (jiffies - ctl.head.time >= VMFS_MAX_AGE(server))
			goto init_cache;

		/*
		 * N.B. ncpfs checks mtime of dentry too here, we don't.
		 *   1. common vmfs servers do not update mtime on dir changes
		 *   2. it requires an extra vmfs request
		 *      (revalidate has the same timeout as ctl.head.time)
		 *
		 * Instead vmfs_ invalidates its own cache on local changes
		 * and remote changes are not seen until timeout.
		 */
	}

	if (ctx->pos > ctl.head.end)
		goto finished;

	ctl.fpos = ctx->pos + (VMFS_DIRCACHE_START - 2);
	ctl.ofs = ctl.fpos / VMFS_DIRCACHE_SIZE;
	ctl.idx = ctl.fpos % VMFS_DIRCACHE_SIZE;

	for (;;) {
		if (ctl.ofs != 0) {
			ctl.page = find_lock_page(&dir->i_data, ctl.ofs);
			if (!ctl.page)
				goto invalid_cache;
			ctl.cache = kmap(ctl.page);
			if (!PageUptodate(ctl.page))
				goto invalid_cache;
		}
		while (ctl.idx < VMFS_DIRCACHE_SIZE) {
			struct dentry *dent;
			int res;

			dent = vmfs_dget_fpos(ctl.cache->dentry[ctl.idx],
					      dentry, filp->f_pos);
			if (!dent)
				goto invalid_cache;

			res = !dir_emit(ctx, dent->d_name.name,
					dent->d_name.len,
					dent->d_inode->i_ino, DT_UNKNOWN);
			dput(dent);
			if (res)
				goto finished;
			ctx->pos += 1;
			ctl.idx += 1;
			if (ctx->pos > ctl.head.end)
				goto finished;
		}
		if (ctl.page) {
			kunmap(ctl.page);
			SetPageUptodate(ctl.page);
			unlock_page(ctl.page);
			page_cache_release(ctl.page);
			ctl.page = NULL;
		}
		ctl.idx = 0;
		ctl.ofs += 1;
	}
invalid_cache:
	if (ctl.page) {
		kunmap(ctl.page);
		unlock_page(ctl.page);
		page_cache_release(ctl.page);
		ctl.page = NULL;
	}
	ctl.cache = cache;
init_cache:
	vmfs_invalidate_dircache_entries(dentry);
	ctl.head.time = jiffies;
	ctl.head.eof = 0;
	ctl.fpos = 2;
	ctl.ofs = 0;
	ctl.idx = VMFS_DIRCACHE_START;
	ctl.filled = 0;
	ctl.valid = 1;
read_really:
	result = server->ops->readdir(filp, ctx, &ctl);
	if (result == -ERESTARTSYS && page)
		ClearPageUptodate(page);
	if (ctl.idx == -1)
		goto invalid_cache;	/* retry */
	ctl.head.end = ctl.fpos - 1;
	ctl.head.eof = ctl.valid;
finished:
	if (page) {
		cache->head = ctl.head;
		kunmap(page);
		if (result != -ERESTARTSYS)
			SetPageUptodate(page);
		unlock_page(page);
		page_cache_release(page);
	}
	if (ctl.page) {
		kunmap(ctl.page);
		SetPageUptodate(ctl.page);
		unlock_page(ctl.page);
		page_cache_release(ctl.page);
	}
out:
	mutex_unlock(&vmfs_mutex);
	return result;
}

static int vmfs_dir_open(struct inode *dir, struct file *file)
{
	struct dentry *dentry = file->f_path.dentry;
	int error = 0;

	VERBOSE("(%s/%s)\n", dentry->d_parent->d_name.name,
		file->f_path.dentry->d_name.name);

	mutex_lock(&vmfs_mutex);


	if (!IS_ROOT(dentry))
		error = vmfs_revalidate_inode(dentry);

	mutex_unlock(&vmfs_mutex);
	return error;
}

/*
 * Dentry operations routines
 */
static int vmfs_lookup_validate(struct dentry *, unsigned int flags);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38)
static int vmfs_delete_dentry(struct dentry *);
#else
static int vmfs_delete_dentry(const struct dentry *);
#endif


static const struct dentry_operations vmfs__dentry_operations_case = {
	.d_revalidate = vmfs_lookup_validate,
	.d_delete = vmfs_delete_dentry,
};

/*
 * This is the callback when the dcache has a lookup hit.
 */
static int vmfs_lookup_validate(struct dentry *dentry, unsigned int flags)
{
	struct vmfs_sb_info *server = server_from_dentry(dentry);
	struct inode *inode = dentry->d_inode;
	unsigned long age = jiffies - dentry->d_time;
	int valid;

	/*
	 * The default validation is based on dentry age:
	 * we believe in dentries for a few seconds.  (But each
	 * successful server lookup renews the timestamp.)
	 */
	valid = (age <= VMFS_MAX_AGE(server));
#ifdef VMFSFS_DEBUG_VERBOSE
	if (!valid)
		VERBOSE("%s/%s not valid, age=%lu\n", DENTRY_PATH(dentry), age);
#endif

	if (inode) {
		mutex_lock(&vmfs_mutex);
		if (is_bad_inode(inode)) {
			PARANOIA("%s/%s has dud inode\n", DENTRY_PATH(dentry));
			valid = 0;
		} else if (!valid)
			valid = (vmfs_revalidate_inode(dentry) == 0);
		mutex_unlock(&vmfs_mutex);
	} else {
		/*
		 * What should we do for negative dentries?
		 */
	}
	return valid;
}


/*
 * This is the callback from dput() when d_count is going to 0.
 * We use this to unhash dentries with bad inodes.
 */
static int
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38)
vmfs_delete_dentry(struct dentry *dentry)
#else
vmfs_delete_dentry(const struct dentry *dentry)
#endif
{
	if (dentry->d_inode) {
		if (is_bad_inode(dentry->d_inode)) {
			PARANOIA("bad inode, unhashing %s/%s\n",
				 DENTRY_PATH(dentry));
			return 1;
		}
	} else {
		/* N.B. Unhash negative dentries? */
	}
	return 0;
}

/*
 * Initialize a new dentry
 */
void vmfs_new_dentry(struct dentry *dentry)
{
	dentry->d_op = &vmfs__dentry_operations_case;
}

/*
 * Whenever a lookup succeeds, we know the parent directories
 * are all valid, so we want to update the dentry timestamps.
 * N.B. Move this to dcache?
 */
void vmfs_renew_times(struct dentry *dentry)
{
	dget(dentry);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
	spin_lock(&dentry->d_lock);
	for (;;) {
		struct dentry *parent;

		dentry->d_time = jiffies;
		if (IS_ROOT(dentry))
			break;
		parent = dentry->d_parent;
		dget(parent);
		spin_unlock(&dentry->d_lock);
		dput(dentry);
		dentry = parent;
		spin_lock(&dentry->d_lock);
	}
	spin_unlock(&dentry->d_lock);
#else
	dentry->d_time = jiffies;

	while (!IS_ROOT(dentry)) {
		struct dentry *parent = dget_parent(dentry);

		dput(dentry);
		dentry = parent;

		dentry->d_time = jiffies;
	}
#endif
	dput(dentry);
}

static struct dentry *vmfs_lookup(struct inode *dir, struct dentry *dentry,
				  unsigned int flags)
{
	struct vmfs_fattr finfo;
	struct inode *inode;
	int error;
	struct vmfs_sb_info *server;

	VERBOSE("%s\n", dentry->d_name.name);

	error = -ENAMETOOLONG;
	if (dentry->d_name.len > VMFS_MAXNAMELEN)
		goto out;

	/* Do not allow lookup of names with backslashes in */
	error = -EINVAL;

	mutex_lock(&vmfs_mutex);
	error = vmfs_proc_getattr(dentry, &finfo);
#ifdef VMFSFS_PARANOIA
	if (error && error != -ENOENT)
		PARANOIA("find %s/%s failed, error=%d\n",
			 DENTRY_PATH(dentry), error);
#endif

	inode = NULL;
	if (error == -ENOENT)
		goto add_entry;
	if (!error) {
		error = -EACCES;
		finfo.f_ino = iunique(dentry->d_sb, 2);
		inode = vmfs_iget(dir->i_sb, &finfo);
		if (inode) {
add_entry:
			server = server_from_dentry(dentry);
			dentry->d_op = &vmfs__dentry_operations_case;
			d_add(dentry, inode);
			vmfs_renew_times(dentry);
			error = 0;
		}
	}
	mutex_unlock(&vmfs_mutex);
out:
	return ERR_PTR(error);
}

/*
 * This code is common to all routines creating a new inode.
 */
static int vmfs_instantiate(struct dentry *dentry, int32_t vhandle,
			    int have_id)
{
	struct inode *inode;
	int error;
	struct vmfs_fattr fattr;

	VERBOSE("file %s/%s, fileid=%u\n", DENTRY_PATH(dentry), vhandle);

	error = vmfs_proc_getattr(dentry, &fattr);
	if (error)
		goto out_close;

	vmfs_renew_times(dentry);
	fattr.f_ino = iunique(dentry->d_sb, 2);
	inode = vmfs_iget(dentry->d_sb, &fattr);
	if (!inode)
		goto out_no_inode;

	if (have_id) {
		/* this is really only for create, where there is a
		 * catch-22 between creating the file and inode */
		struct vmfs_inode_info *ei = VMFS_I(inode);

		ei->vhandle = vhandle;
		ei->vopen = 1;
		ei->vaccess = VFS_OPEN_RDWR;
	}
	d_instantiate(dentry, inode);
out:
	return error;

out_no_inode:
	error = -EACCES;
out_close:
	if (have_id) {
		PARANOIA("%s/%s failed, error=%d, closing %u\n",
			 DENTRY_PATH(dentry), error, vhandle);
		vmfs_close_fileid(dentry, vhandle);
	}
	goto out;
}

/* N.B. How should the mode argument be used? */
static int
vmfs_create(struct inode *dir, struct dentry *dentry, umode_t mode,
	    bool excl)
{
	int32_t fileid;
	int error;

	VERBOSE("creating %s/%s, mode=%d\n", DENTRY_PATH(dentry), mode);

	mutex_lock(&vmfs_mutex);

	vmfs_invalid_dir_cache(dir);
	error = vmfs_proc_create(dentry, mode, &fileid);
	if (!error) {
		error = vmfs_instantiate(dentry, fileid, 1);
	} else {
		PARANOIA("%s/%s failed, error=%d\n",
			 DENTRY_PATH(dentry), error);
	}
	mutex_unlock(&vmfs_mutex);
	return error;
}

/* N.B. How should the mode argument be used? */
static int vmfs_mkdir(struct inode *dir, struct dentry *dentry, umode_t mode)
{
	int error;

	VERBOSE("\n");

	mutex_lock(&vmfs_mutex);
	vmfs_invalid_dir_cache(dir);
	error = vmfs_proc_mkdir(dentry);
	if (!error)
		error = vmfs_instantiate(dentry, 0, 0);
	mutex_unlock(&vmfs_mutex);
	return error;
}

static int vmfs_rmdir(struct inode *dir, struct dentry *dentry)
{
	struct inode *inode = dentry->d_inode;
	int error;

	VERBOSE("\n");
	/*
	 * Close the directory if it's open.
	 */
	mutex_lock(&vmfs_mutex);
	vmfs_close(inode);

	/*
	 * Check that nobody else is using the directory..
	 */
	error = -EBUSY;
	if (!d_unhashed(dentry))
		goto out;

	vmfs_invalid_dir_cache(dir);
	error = vmfs_proc_rmdir(dentry);

out:
	mutex_unlock(&vmfs_mutex);
	return error;
}

static int vmfs_unlink(struct inode *dir, struct dentry *dentry)
{
	int error;

	/*
	 * Close the file if it's open.
	 */
	mutex_lock(&vmfs_mutex);
	vmfs_close(dentry->d_inode);

	vmfs_invalid_dir_cache(dir);
	error = vmfs_proc_unlink(dentry);
	if (!error)
		vmfs_renew_times(dentry);
	mutex_unlock(&vmfs_mutex);
	return error;
}

static int
vmfs_rename(struct inode *old_dir, struct dentry *old_dentry,
	    struct inode *new_dir, struct dentry *new_dentry)
{
	int error;

	VERBOSE("\n");

	/*
	 * Close any open files, and check whether to delete the
	 * target before attempting the rename.
	 */
	mutex_lock(&vmfs_mutex);
	if (old_dentry->d_inode)
		vmfs_close(old_dentry->d_inode);
	if (new_dentry->d_inode) {
		vmfs_close(new_dentry->d_inode);
		error = vmfs_proc_unlink(new_dentry);
		if (error) {
			VERBOSE("unlink %s/%s, error=%d\n",
				DENTRY_PATH(new_dentry), error);
			goto out;
		}
		/* FIXME */
		d_delete(new_dentry);
	}

	vmfs_invalid_dir_cache(old_dir);
	vmfs_invalid_dir_cache(new_dir);
	error = vmfs_proc_mv(old_dentry, new_dentry);
	if (!error) {
		vmfs_renew_times(old_dentry);
		vmfs_renew_times(new_dentry);
	}
out:
	mutex_unlock(&vmfs_mutex);
	return error;
}

/*
 * FIXME: samba servers won't let you create device nodes unless uid/gid
 * matches the connection credentials (and we don't know which those are ...)
 */
static int
vmfs_make_node(struct inode *dir, struct dentry *dentry, umode_t mode, dev_t dev)
{
	return -EINVAL;
}

/*
 * dentry = existing file
 * new_dentry = new file
 */
static int
vmfs_link(struct dentry *dentry, struct inode *dir, struct dentry *new_dentry)
{
	int error;

	DEBUG1("vmfs_link old=%s/%s new=%s/%s\n",
	       DENTRY_PATH(dentry), DENTRY_PATH(new_dentry));
	vmfs_invalid_dir_cache(dir);
	error = vmfs_proc_link(server_from_dentry(dentry), dentry, new_dentry);
	if (!error) {
		vmfs_renew_times(dentry);
		error = vmfs_instantiate(new_dentry, 0, 0);
	}
	return error;
}
