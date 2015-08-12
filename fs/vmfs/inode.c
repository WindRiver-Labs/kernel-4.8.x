/*
 *  inode.c
 *
 *  Copyright (C) 1995, 1996 by Paal-Kr. Engstad and Volker Lendecke
 *  Copyright (C) 1997 by Volker Lendecke
 *
 *  Copyright (C) 2008-2009 ARM Limited
 *
 *  Please add a note about your changes to vmfs_ in the ChangeLog file.
 */

#include <linux/module.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/stat.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/file.h>
#include <linux/dcache.h>
#include <linux/seq_file.h>
#include <linux/mount.h>
#include <linux/vfs.h>
#include <linux/highuid.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include "vmfs_fs.h"
#include "vmfsno.h"
#include "vmfs_mount.h"

#include <linux/uaccess.h>
#include <linux/version.h>

#include "vmfs_debug.h"
#include "getopt.h"
#include "proto.h"

#include "messagebox.h"
#include "vmfs.h"

/* gpb: no reason this has to be in linux/magic.h */

#define VMFS_SUPER_MAGIC 0x564D4653	/* VMFS */

/* gpb: this needs to be made configurable */

#define VMFS_TTL_DEFAULT 1000

DEFINE_MUTEX(vmfs_mutex);

static void vmfs_delete_inode(struct inode *);
static void vmfs_put_super(struct super_block *);
static int vmfs_statfs(struct dentry *, struct kstatfs *);
static int vmfs_show_options(struct seq_file *, struct dentry *);

static struct kmem_cache *vmfs_inode_cachep;

static MessageBox *mbox;
static VFS *vfs;

static struct inode *vmfs_alloc_inode(struct super_block *sb)
{
	struct vmfs_inode_info *ei;

	ei = (struct vmfs_inode_info *)kmem_cache_alloc(vmfs_inode_cachep,
							GFP_KERNEL);
	if (!ei)
		return NULL;
	return &ei->vfs_inode;
}

static void vmfs_destroy_inode(struct inode *inode)
{
	kmem_cache_free(vmfs_inode_cachep, VMFS_I(inode));
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
static void init_once(void *foo)
#else
static void init_once(struct kmem_cache *cachep, void *foo)
#endif
{
	struct vmfs_inode_info *ei = (struct vmfs_inode_info *)foo;

	inode_init_once(&ei->vfs_inode);
}

static int init_inodecache(void)
{
	vmfs_inode_cachep = kmem_cache_create("vmfs_inode_cache",
					      sizeof(struct vmfs_inode_info),
					      0, (SLAB_RECLAIM_ACCOUNT |
						  SLAB_MEM_SPREAD), init_once);
	if (vmfs_inode_cachep == NULL)
		return -ENOMEM;
	return 0;
}

static void destroy_inodecache(void)
{
	kmem_cache_destroy(vmfs_inode_cachep);
}

static int vmfs_remount(struct super_block *sb, int *flags, char *data)
{
	*flags |= MS_NODIRATIME;
	return 0;
}

static const struct super_operations vmfs_sops = {
	.alloc_inode = vmfs_alloc_inode,
	.destroy_inode = vmfs_destroy_inode,
	.drop_inode = generic_delete_inode,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
	.delete_inode = vmfs_delete_inode,
#else
	.evict_inode = vmfs_delete_inode,
#endif
	.put_super = vmfs_put_super,
	.statfs = vmfs_statfs,
	.show_options = vmfs_show_options,
	.remount_fs = vmfs_remount,
};

/* We are always generating a new inode here */
struct inode *vmfs_iget(struct super_block *sb, struct vmfs_fattr *fattr)
{
	struct inode *result;

	DEBUG1("vmfs_iget: %p\n", fattr);

	result = new_inode(sb);
	if (!result)
		return result;
	result->i_ino = fattr->f_ino;
	VMFS_I(result)->open = 0;
	VMFS_I(result)->closed = 0;
	VMFS_I(result)->openers = 0;
	VMFS_I(result)->vhandle = -1;
	VMFS_I(result)->vaccess = 0;
	VMFS_I(result)->vopen = 0;

	vmfs_set_inode_attr(result, fattr);
	if (S_ISREG(result->i_mode)) {
		result->i_op = &vmfs_file_inode_operations;
		result->i_fop = &vmfs_file_operations;
		result->i_data.a_ops = &vmfs_file_aops;
	} else if (S_ISDIR(result->i_mode)) {
		result->i_op = &vmfs_dir_inode_operations_unix;
		result->i_fop = &vmfs_dir_operations;
	} else if (S_ISLNK(result->i_mode)) {
		result->i_op = &vmfs_link_inode_operations;
	} else {
		init_special_inode(result, result->i_mode, fattr->f_rdev);
	}
	insert_inode_hash(result);
	return result;
}

/*
 * Copy the inode data to a vmfs_fattr structure.
 */
void vmfs_get_inode_attr(struct inode *inode, struct vmfs_fattr *fattr)
{
	memset(fattr, 0, sizeof(struct vmfs_fattr));
	fattr->f_mode = inode->i_mode;
	fattr->f_nlink = inode->i_nlink;
	fattr->f_ino = inode->i_ino;
	fattr->f_uid = inode->i_uid;
	fattr->f_gid = inode->i_gid;
	fattr->f_size = inode->i_size;
	fattr->f_mtime = inode->i_mtime;
	fattr->f_ctime = inode->i_ctime;
	fattr->f_atime = inode->i_atime;
	fattr->f_blocks = inode->i_blocks;

}

/*
 * Update the inode, possibly causing it to invalidate its pages if mtime/size
 * is different from last time.
 */
void vmfs_set_inode_attr(struct inode *inode, struct vmfs_fattr *fattr)
{
	struct vmfs_inode_info *ei = VMFS_I(inode);

	/*
	 * A size change should have a different mtime, or same mtime
	 * but different size.
	 */
	time_t last_time = inode->i_mtime.tv_sec;
	loff_t last_sz = inode->i_size;

	inode->i_mode = fattr->f_mode;
	set_nlink(inode, fattr->f_nlink);
	inode->i_uid = fattr->f_uid;
	inode->i_gid = fattr->f_gid;
	inode->i_ctime = fattr->f_ctime;
	inode->i_blocks = fattr->f_blocks;
	inode->i_size = fattr->f_size;
	inode->i_mtime = fattr->f_mtime;
	inode->i_atime = fattr->f_atime;


	/*
	 * Update the "last time refreshed" field for revalidation.
	 */
	ei->oldmtime = jiffies;

	if (inode->i_mtime.tv_sec != last_time || inode->i_size != last_sz) {
		VERBOSE("%ld changed, old=%ld, new=%ld, oz=%ld, nz=%ld\n",
			inode->i_ino,
			(long)last_time, (long)inode->i_mtime.tv_sec,
			(long)last_sz, (long)inode->i_size);

		if (!S_ISDIR(inode->i_mode))
			invalidate_remote_inode(inode);
	}
}

/*
 * This is called if the connection has gone bad ...
 * try to kill off all the current inodes.
 */
void vmfs_invalidate_inodes(struct vmfs_sb_info *server)
{
	VERBOSE("\n");
	shrink_dcache_sb(SB_of(server));
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
	invalidate_inodes(SB_of(server));
#endif
}

/*
 * This is called to update the inode attributes after
 * we've made changes to a file or directory.
 */
static int vmfs_refresh_inode(struct dentry *dentry)
{
	struct inode *inode = dentry->d_inode;
	int error;
	struct vmfs_fattr fattr;

	DEBUG1("");

	error = vmfs_proc_getattr(dentry, &fattr);
	if (!error) {
		vmfs_renew_times(dentry);
		/*
		 * Check whether the type part of the mode changed,
		 * and don't update the attributes if it did.
		 *
		 * And don't dick with the root inode
		 */
		if (inode->i_ino == 2)
			return error;
		if (S_ISLNK(inode->i_mode))
			return error;	/* VFS will deal with it */

		if ((inode->i_mode & S_IFMT) == (fattr.f_mode & S_IFMT)) {
			vmfs_set_inode_attr(inode, &fattr);
		} else {
			/*
			 * Big trouble! The inode has become a new object,
			 * so any operations attempted on it are invalid.
			 *
			 * To limit damage, mark the inode as bad so that
			 * subsequent lookup validations will fail.
			 */
			PARANOIA("%s/%s changed mode, %07o to %07o\n",
				 DENTRY_PATH(dentry),
				 inode->i_mode, fattr.f_mode);

			fattr.f_mode = inode->i_mode;	/* save mode */
			make_bad_inode(inode);
			inode->i_mode = fattr.f_mode;	/* restore mode */
			/*
			 * No need to worry about unhashing the dentry: the
			 * lookup validation will see that the inode is bad.
			 * But we do want to invalidate the caches ...
			 */
			if (!S_ISDIR(inode->i_mode))
				invalidate_remote_inode(inode);
			else
				vmfs_invalid_dir_cache(inode);
			error = -EIO;
		}
	}
	return error;
}

/*
 * This is called when we want to check whether the inode
 * has changed on the server.  If it has changed, we must
 * invalidate our local caches.
 */
int vmfs_revalidate_inode(struct dentry *dentry)
{
	struct vmfs_sb_info *s = server_from_dentry(dentry);
	struct inode *inode = dentry->d_inode;
	int error = 0;

	DEBUG1("vmfs_revalidate_inode\n");

	/*
	 * Check whether we've recently refreshed the inode.
	 */
	if (time_before(jiffies, VMFS_I(inode)->oldmtime + VMFS_MAX_AGE(s))) {
		VERBOSE("up-to-date, ino=%ld, jiffies=%lu, oldtime=%lu\n",
			inode->i_ino, jiffies, VMFS_I(inode)->oldmtime);
		goto out;
	}

	error = vmfs_refresh_inode(dentry);
out:
	return error;
}

/*
 * This routine is called when i_nlink == 0 and i_count goes to 0.
 * All blocking cleanup operations need to go here to avoid races.
 */
static void vmfs_delete_inode(struct inode *ino)
{
	DEBUG1("ino=%ld\n", ino->i_ino);
	truncate_inode_pages(&ino->i_data, 0);
	clear_inode(ino);
	mutex_lock(&vmfs_mutex);
	if (vmfs_close(ino))
		PARANOIA("could not close inode %ld\n", ino->i_ino);
	mutex_unlock(&vmfs_mutex);
}

/*
 * vmfs_show_options() is for displaying mount options in /proc/mounts.
 * It tries to avoid showing settings that were not changed from their
 * defaults.
 */
static int vmfs_show_options(struct seq_file *s, struct dentry *m)
{

	return 0;
}

static void vmfs_put_super(struct super_block *sb)
{
	struct vmfs_sb_info *server = VMFS_SB(sb);

	mutex_lock(&vmfs_mutex);

	kfree(server->ops);

	sb->s_fs_info = NULL;

	mutex_unlock(&vmfs_mutex);
	kfree(server);
}

static int vmfs_fill_super(struct super_block *sb, void *raw_data_unused,
			   int silent)
{
	struct vmfs_sb_info *server;
	struct vmfs_mount_data_kernel *mnt;
	struct inode *root_inode;
	struct vmfs_fattr root;
	void *mem;

	sb->s_flags |= MS_NODIRATIME;
	sb->s_blocksize = 1024;
	sb->s_blocksize_bits = 10;
	sb->s_magic = VMFS_SUPER_MAGIC;
	sb->s_op = &vmfs_sops;
	sb->s_time_gran = 100;

	server = kzalloc(sizeof(struct vmfs_sb_info), GFP_KERNEL);
	if (!server)
		goto out_no_server;
	sb->s_fs_info = server;

	server->super_block = sb;
	server->mnt = NULL;
	server->vfs = vfs;

	sema_init(&server->sem, 1);
	INIT_LIST_HEAD(&server->entry);

	/* Allocate global temp buffer and some superblock helper structs */
	/* FIXME: move these to the vmfs_sb_info struct */
	VERBOSE("alloc chunk = %zu\n", sizeof(struct vmfs_ops) +
		sizeof(struct vmfs_mount_data_kernel));
	mem = kmalloc(sizeof(struct vmfs_ops) +
		      sizeof(struct vmfs_mount_data_kernel), GFP_KERNEL);
	if (!mem)
		goto out_no_mem;

	server->ops = mem;
	vmfs_install_ops(server->ops);
	server->mnt = mem + sizeof(struct vmfs_ops);

	mnt = server->mnt;

	memset(mnt, 0, sizeof(struct vmfs_mount_data_kernel));

	mnt->ttl = VMFS_TTL_DEFAULT;
	mnt->file_mode = S_IRWXU | S_IRGRP | S_IXGRP |
	    S_IROTH | S_IXOTH | S_IFREG;
	mnt->dir_mode = S_IRWXU | S_IRGRP | S_IXGRP |
	    S_IROTH | S_IXOTH | S_IFDIR;

	mnt->mounted_uid = current->real_cred->uid;

	/*
	 * Keep the super block locked while we get the root inode.
	 */
	vmfs_init_root_dirent(server, &root, sb);
	root_inode = vmfs_iget(sb, &root);
	if (!root_inode)
		goto out_no_root;

	sb->s_root = d_make_root(root_inode);
	if (!sb->s_root)
		goto out_no_root;

	vmfs_new_dentry(sb->s_root);

	return 0;

out_no_root:
	iput(root_inode);
	kfree(mem);
out_no_mem:
	if (!server->mnt)
		pr_err("vmfs_fill_super: allocation failure\n");
	sb->s_fs_info = NULL;
	kfree(server);
	return -EINVAL;
out_no_server:
	pr_err("vmfs_fill_super: cannot allocate struct vmfs_sb_info\n");
	return -ENOMEM;
}

static int vmfs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	int result;

	mutex_lock(&vmfs_mutex);

	result = vmfs_proc_dskattr(dentry, buf);

	mutex_unlock(&vmfs_mutex);

	buf->f_type = VMFS_SUPER_MAGIC;
	buf->f_namelen = VMFS_MAXPATHLEN;
	return result;
}

int vmfs_getattr(struct vfsmount *mnt, struct dentry *dentry,
		 struct kstat *stat)
{
	int err = vmfs_revalidate_inode(dentry);

	if (!err)
		generic_fillattr(dentry->d_inode, stat);
	return err;
}

int vmfs_notify_change(struct dentry *dentry, struct iattr *attr)
{
	struct inode *inode = dentry->d_inode;
	struct vmfs_sb_info *server = server_from_dentry(dentry);
	unsigned int mask = (S_IFREG | S_IFDIR | S_IRWXUGO);
	int error, changed, refresh = 0;
	struct vmfs_fattr fattr;

	DEBUG1("\n");

	mutex_lock(&vmfs_mutex);

	error = vmfs_revalidate_inode(dentry);
	if (error)
		goto out;

	error = inode_change_ok(inode, attr);
	if (error < 0)
		goto out;

	error = -EPERM;

	if ((attr->ia_valid & ATTR_UID) &&
	    (attr->ia_uid.val != (server->mnt->uid).val))
		goto out;

	if ((attr->ia_valid & ATTR_GID) &&
	    ((attr->ia_gid).val != (server->mnt->gid).val))
		goto out;

	if ((attr->ia_valid & ATTR_MODE) && (attr->ia_mode & ~mask))
		goto out;

	if ((attr->ia_valid & ATTR_SIZE) != 0) {
		VERBOSE("changing %s/%s, old size=%ld, new size=%ld\n",
			DENTRY_PATH(dentry),
			(long)inode->i_size, (long)attr->ia_size);

		filemap_write_and_wait(inode->i_mapping);

		error = vmfs_open(dentry, 0, O_WRONLY);
		if (error)
			goto out;
		error = server->ops->truncate(inode, attr->ia_size);
		if (error)
			goto out;
		/*error = vmtruncate(inode, attr->ia_size);
		if (error)
			goto out;*/
		refresh = 1;
	}

	/*
	 * Initialize the fattr and check for changed fields.
	 * Note: CTIME under VMFS is creation time rather than
	 * change time, so we don't attempt to change it.
	 */
	vmfs_get_inode_attr(inode, &fattr);

	changed = 0;
	if ((attr->ia_valid & ATTR_MTIME) != 0) {
		fattr.f_mtime = attr->ia_mtime;
		changed = 1;
	}
	if ((attr->ia_valid & ATTR_ATIME) != 0) {
		fattr.f_atime = attr->ia_atime;
		/* Earlier protocols don't have an access time */
	}
	if (changed) {
		error = vmfs_proc_settime(dentry, &fattr);
		if (error)
			goto out;
		refresh = 1;
	}

	/*
	 * Check for mode changes ... we're extremely limited in
	 * what can be set for VMFS servers: just the read-only bit.
	 */
	if ((attr->ia_valid & ATTR_MODE) != 0) {
		VERBOSE("%s/%s mode change, old=%x, new=%x\n",
			DENTRY_PATH(dentry), fattr.f_mode, attr->ia_mode);
		changed = 0;
		if (attr->ia_mode & S_IWUSR) {
			if (fattr.attr & aRONLY) {
				fattr.attr &= ~aRONLY;
				changed = 1;
			}
		} else {
			if (!(fattr.attr & aRONLY)) {
				fattr.attr |= aRONLY;
				changed = 1;
			}
		}
		if (changed) {
			error = vmfs_proc_setattr(dentry, &fattr);
			if (error)
				goto out;
			refresh = 1;
		}
	}
	error = 0;

out:
	if (refresh)
		vmfs_refresh_inode(dentry);
	mutex_unlock(&vmfs_mutex);
	return error;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
static int vmfs_get_sb(struct file_system_type *fs_type,
		       int flags, const char *dev_name, void *data,
		       struct vfsmount *mnt)
{
	return get_sb_nodev(fs_type, flags, data, vmfs_fill_super, mnt);
}
#else
static struct dentry *vmfs_do_mount(struct file_system_type *fs_type,
				    int flags, const char *dev_name,
				    void *data)
{
	return mount_nodev(fs_type, flags, data, vmfs_fill_super);
}
#endif

static struct file_system_type vmfs_fs_type = {
	.owner = THIS_MODULE,
	.name = "vmfs",
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
	.get_sb = vmfs_get_sb,
#else
	.mount = vmfs_do_mount,
#endif
	.kill_sb = kill_anon_super,
	.fs_flags = FS_BINARY_MOUNTDATA,
};

static int __init init_vmfs_fs(void)
{
	phys_addr_t dev_base = CONFIG_VMFS_DEV_BASE;
	uint32_t dev_irq = -1;
	int err;

	DEBUG1("registering ...\n");

	err = init_inodecache();
	if (err)
		goto out_inode;

	if (IS_ENABLED(CONFIG_OF)) {
		struct device_node *np;

		np = of_find_compatible_node(NULL, NULL, "arm,messagebox");
		if (np) {
			struct resource res;

			if (of_address_to_resource(np, 0, &res) == 0)
				dev_base = res.start;
		}
	}

	/* map the message box device into memory */

	mbox = mb_new(dev_base, dev_irq);

	if (mbox == NULL) {
		err = -1;
		goto out_inode;
	}

	vfs = vfsop_new(mbox);
	if (vfs == NULL) {
		err = -1;
		goto out_mbox;
	}

	err = register_filesystem(&vmfs_fs_type);
	if (err)
		goto out;
	return 0;
out:
	destroy_inodecache();
	vfsop_delete(vfs);
out_mbox:
	mb_delete(mbox);
out_inode:
	return err;
}

static void __exit exit_vmfs_fs(void)
{
	DEBUG1("unregistering ...\n");
	unregister_filesystem(&vmfs_fs_type);

	vfsop_delete(vfs);
	mb_delete(mbox);

	destroy_inodecache();
}

module_init(init_vmfs_fs)
	module_exit(exit_vmfs_fs)
	MODULE_LICENSE("GPL");
