/*
 * Driver for /dev/crypto device (aka CryptoDev)
 *
 * Copyright (c) 2004 Michal Ludvig <mludvig@logix.net.nz>, SuSE Labs
 * Copyright (c) 2009,2010 Nikos Mavrogiannopoulos <nmav@gnutls.org>
 *
 * This file is part of linux cryptodev.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 * Device /dev/crypto provides an interface for
 * accessing kernel CryptoAPI algorithms (ciphers,
 * hashes) from userspace programs.
 *
 * /dev/crypto interface was originally introduced in
 * OpenBSD and this module attempts to keep the API.
 *
 */

#include <linux/crypto.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/ioctl.h>
#include <linux/random.h>
#include <linux/syscalls.h>
#include <linux/pagemap.h>
#include <linux/uaccess.h>
#include "cryptodev.h"
#include <linux/scatterlist.h>
#include "cryptodev_int.h"
#include "version.h"

MODULE_AUTHOR("Nikos Mavrogiannopoulos <nmav@gnutls.org>");
MODULE_DESCRIPTION("CryptoDev driver");
MODULE_LICENSE("GPL");

/* ====== Compile-time config ====== */

#define CRYPTODEV_STATS

/* ====== Module parameters ====== */

int cryptodev_verbosity;
module_param(cryptodev_verbosity, int, 0644);
MODULE_PARM_DESC(cryptodev_verbosity, "0: normal, 1: verbose, 2: debug");

#ifdef CRYPTODEV_STATS
static int enable_stats;
module_param(enable_stats, int, 0644);
MODULE_PARM_DESC(enable_stats, "collect statictics about cryptodev usage");
#endif

/* ====== CryptoAPI ====== */
struct fcrypt {
	struct list_head list;
	struct mutex sem;
};

struct crypt_priv {
	struct fcrypt fcrypt;
};

#define FILL_SG(sg, ptr, len)					\
	do {							\
		(sg)->page = virt_to_page(ptr);			\
		(sg)->offset = offset_in_page(ptr);		\
		(sg)->length = len;				\
		(sg)->dma_address = 0;				\
	} while (0)

struct csession {
	struct list_head entry;
	struct mutex sem;
	struct cipher_data cdata;
	struct hash_data hdata;
	uint32_t sid;
#ifdef CRYPTODEV_STATS
#if !((COP_ENCRYPT < 2) && (COP_DECRYPT < 2))
#error Struct csession.stat uses COP_{ENCRYPT,DECRYPT} as indices. Do something!
#endif
	unsigned long long stat[2];
	size_t stat_max_size, stat_count;
#endif
	int array_size;
	struct page **pages;
	struct scatterlist *sg;
};

/* Prepare session for future use. */
static int
crypto_create_session(struct fcrypt *fcr, struct session_op *sop)
{
	struct csession	*ses_new = NULL, *ses_ptr;
	int ret = 0;
	const char *alg_name = NULL;
	const char *hash_name = NULL;
	int hmac_mode = 1;

	/* Does the request make sense? */
	if (unlikely(!sop->cipher && !sop->mac)) {
		dprintk(1, KERN_DEBUG, "Both 'cipher' and 'mac' unset.\n");
		return -EINVAL;
	}

	switch (sop->cipher) {
	case 0:
		break;
	case CRYPTO_DES_CBC:
		alg_name = "cbc(des)";
		break;
	case CRYPTO_3DES_CBC:
		alg_name = "cbc(des3_ede)";
		break;
	case CRYPTO_BLF_CBC:
		alg_name = "cbc(blowfish)";
		break;
	case CRYPTO_AES_CBC:
		alg_name = "cbc(aes)";
		break;
	case CRYPTO_CAMELLIA_CBC:
		alg_name = "cbc(camelia)";
		break;
	case CRYPTO_AES_CTR:
		alg_name = "ctr(aes)";
		break;
	case CRYPTO_NULL:
		alg_name = "ecb(cipher_null)";
		break;
	default:
		dprintk(1, KERN_DEBUG, "%s: bad cipher: %d\n", __func__,
			sop->cipher);
		return -EINVAL;
	}

	switch (sop->mac) {
	case 0:
		break;
	case CRYPTO_MD5_HMAC:
		hash_name = "hmac(md5)";
		break;
	case CRYPTO_RIPEMD160_HMAC:
		hash_name = "hmac(rmd160)";
		break;
	case CRYPTO_SHA1_HMAC:
		hash_name = "hmac(sha1)";
		break;
	case CRYPTO_SHA2_256_HMAC:
		hash_name = "hmac(sha256)";
		break;
	case CRYPTO_SHA2_384_HMAC:
		hash_name = "hmac(sha384)";
		break;
	case CRYPTO_SHA2_512_HMAC:
		hash_name = "hmac(sha512)";
		break;

	/* non-hmac cases */
	case CRYPTO_MD5:
		hash_name = "md5";
		hmac_mode = 0;
		break;
	case CRYPTO_RIPEMD160:
		hash_name = "rmd160";
		hmac_mode = 0;
		break;
	case CRYPTO_SHA1:
		hash_name = "sha1";
		hmac_mode = 0;
		break;
	case CRYPTO_SHA2_256:
		hash_name = "sha256";
		hmac_mode = 0;
		break;
	case CRYPTO_SHA2_384:
		hash_name = "sha384";
		hmac_mode = 0;
		break;
	case CRYPTO_SHA2_512:
		hash_name = "sha512";
		hmac_mode = 0;
		break;

	default:
		dprintk(1, KERN_DEBUG, "%s: bad mac: %d\n", __func__,
			sop->mac);
		return -EINVAL;
	}

	/* Create a session and put it to the list. */
	ses_new = kzalloc(sizeof(*ses_new), GFP_KERNEL);
	if (!ses_new)
		return -ENOMEM;

	/* Set-up crypto transform. */
	if (alg_name) {
		uint8_t keyp[CRYPTO_CIPHER_MAX_KEY_LEN];

		if (unlikely(sop->keylen > CRYPTO_CIPHER_MAX_KEY_LEN)) {
			dprintk(1, KERN_DEBUG,
				"Setting key failed for %s-%zu.\n",
				alg_name, (size_t)sop->keylen*8);
			ret = -EINVAL;
			goto error_cipher;
		}

		if (unlikely(copy_from_user(keyp, sop->key, sop->keylen))) {
			ret = -EFAULT;
			goto error_cipher;
		}

		ret = cryptodev_cipher_init(&ses_new->cdata, alg_name, keyp,
								sop->keylen);
		if (ret < 0) {
			dprintk(1, KERN_DEBUG,
				"%s: Failed to load cipher for %s\n",
				__func__, alg_name);
			ret = -EINVAL;
			goto error_cipher;
		}
	}

	if (hash_name) {
		uint8_t keyp[CRYPTO_HMAC_MAX_KEY_LEN];

		if (unlikely(sop->mackeylen > CRYPTO_HMAC_MAX_KEY_LEN)) {
			dprintk(1, KERN_DEBUG,
				"Setting key failed for %s-%zu.\n",
				alg_name, (size_t)sop->mackeylen*8);
			ret = -EINVAL;
			goto error_hash;
		}

		if (unlikely(copy_from_user(keyp, sop->mackey,
					    sop->mackeylen))) {
			ret = -EFAULT;
			goto error_hash;
		}

		ret = cryptodev_hash_init(&ses_new->hdata, hash_name, hmac_mode,
							keyp, sop->mackeylen);
		if (ret != 0) {
			dprintk(1, KERN_DEBUG,
			"%s: Failed to load hash for %s\n",
			__func__, hash_name);
			ret = -EINVAL;
			goto error_hash;
		}
	}

	ses_new->array_size = DEFAULT_PREALLOC_PAGES;
	dprintk(2, KERN_DEBUG, "%s: preallocating for %d user pages\n",
			__func__, ses_new->array_size);
	ses_new->pages = kzalloc(ses_new->array_size *
			sizeof(struct page *), GFP_KERNEL);
	ses_new->sg = kzalloc(ses_new->array_size *
			sizeof(struct scatterlist), GFP_KERNEL);
	if (ses_new->sg == NULL || ses_new->pages == NULL) {
		dprintk(0, KERN_DEBUG, "Memory error\n");
		ret = -ENOMEM;
		goto error_hash;
	}

	/* put the new session to the list */
	get_random_bytes(&ses_new->sid, sizeof(ses_new->sid));
	mutex_init(&ses_new->sem);

	mutex_lock(&fcr->sem);
restart:
	list_for_each_entry(ses_ptr, &fcr->list, entry) {
		/* Check for duplicate SID */
		if (unlikely(ses_new->sid == ses_ptr->sid)) {
			get_random_bytes(&ses_new->sid, sizeof(ses_new->sid));
			/* Unless we have a broken RNG this
			   shouldn't loop forever... ;-) */
			goto restart;
		}
	}

	list_add(&ses_new->entry, &fcr->list);
	mutex_unlock(&fcr->sem);

	/* Fill in some values for the user. */
	sop->ses = ses_new->sid;

	return 0;

error_hash:
	cryptodev_cipher_deinit(&ses_new->cdata);
	kfree(ses_new->sg);
	kfree(ses_new->pages);
error_cipher:
	kfree(ses_new);

	return ret;

}

/* Everything that needs to be done when remowing a session. */
static inline void
crypto_destroy_session(struct csession *ses_ptr)
{
	if (!mutex_trylock(&ses_ptr->sem)) {
		dprintk(2, KERN_DEBUG, "Waiting for semaphore of sid=0x%08X\n",
			ses_ptr->sid);
		mutex_lock(&ses_ptr->sem);
	}
	dprintk(2, KERN_DEBUG, "Removed session 0x%08X\n", ses_ptr->sid);
#if defined(CRYPTODEV_STATS)
	if (enable_stats)
		dprintk(2, KERN_DEBUG,
			"Usage in Bytes: enc=%llu, dec=%llu, \
			max=%zu, avg=%lu, cnt=%zu\n",
			ses_ptr->stat[COP_ENCRYPT], ses_ptr->stat[COP_DECRYPT],
			ses_ptr->stat_max_size, ses_ptr->stat_count > 0
				? ((unsigned long)(ses_ptr->stat[COP_ENCRYPT]+
						   ses_ptr->stat[COP_DECRYPT]) /
				   ses_ptr->stat_count) : 0,
			ses_ptr->stat_count);
#endif
	cryptodev_cipher_deinit(&ses_ptr->cdata);
	cryptodev_hash_deinit(&ses_ptr->hdata);
	dprintk(2, KERN_DEBUG, "%s: freeing space for %d user pages\n",
			__func__, ses_ptr->array_size);
	kfree(ses_ptr->pages);
	kfree(ses_ptr->sg);
	mutex_unlock(&ses_ptr->sem);
	kfree(ses_ptr);
}

/* Look up a session by ID and remove. */
static int
crypto_finish_session(struct fcrypt *fcr, uint32_t sid)
{
	struct csession *tmp, *ses_ptr;
	struct list_head *head;
	int ret = 0;

	mutex_lock(&fcr->sem);
	head = &fcr->list;
	list_for_each_entry_safe(ses_ptr, tmp, head, entry) {
		if (ses_ptr->sid == sid) {
			list_del(&ses_ptr->entry);
			crypto_destroy_session(ses_ptr);
			break;
		}
	}

	if (unlikely(!ses_ptr)) {
		dprintk(1, KERN_ERR, "Session with sid=0x%08X not found!\n",
			sid);
		ret = -ENOENT;
	}
	mutex_unlock(&fcr->sem);

	return ret;
}

/* Remove all sessions when closing the file */
static int
crypto_finish_all_sessions(struct fcrypt *fcr)
{
	struct csession *tmp, *ses_ptr;
	struct list_head *head;

	mutex_lock(&fcr->sem);

	head = &fcr->list;
	list_for_each_entry_safe(ses_ptr, tmp, head, entry) {
		list_del(&ses_ptr->entry);
		crypto_destroy_session(ses_ptr);
	}
	mutex_unlock(&fcr->sem);

	return 0;
}

/* Look up session by session ID. The returned session is locked. */
static struct csession *
crypto_get_session_by_sid(struct fcrypt *fcr, uint32_t sid)
{
	struct csession *ses_ptr;

	mutex_lock(&fcr->sem);
	list_for_each_entry(ses_ptr, &fcr->list, entry) {
		if (ses_ptr->sid == sid) {
			mutex_lock(&ses_ptr->sem);
			break;
		}
	}
	mutex_unlock(&fcr->sem);

	return ses_ptr;
}

static int
hash_n_crypt(struct csession *ses_ptr, struct crypt_op *cop,
		struct scatterlist *src_sg, struct scatterlist *dst_sg,
		uint32_t len)
{
	int ret;

	/* Always hash before encryption and after decryption. Maybe
	 * we should introduce a flag to switch... TBD later on.
	 */
	if (cop->op == COP_ENCRYPT) {
		if (ses_ptr->hdata.init != 0) {
			ret = cryptodev_hash_update(&ses_ptr->hdata,
							src_sg, len);
			if (unlikely(ret))
				goto out_err;
		}
		if (ses_ptr->cdata.init != 0) {
			ret = cryptodev_cipher_encrypt(&ses_ptr->cdata,
							src_sg, dst_sg, len);

			if (unlikely(ret))
				goto out_err;
		}
	} else {
		if (ses_ptr->cdata.init != 0) {
			ret = cryptodev_cipher_decrypt(&ses_ptr->cdata,
							src_sg, dst_sg, len);

			if (unlikely(ret))
				goto out_err;
		}

		if (ses_ptr->hdata.init != 0) {
			ret = cryptodev_hash_update(&ses_ptr->hdata,
								dst_sg, len);
			if (unlikely(ret))
				goto out_err;
		}
	}
	return 0;
out_err:
	dprintk(0, KERN_ERR, "CryptoAPI failure: %d\n", ret);
	return ret;
}


/* This is the main crypto function - feed it with plaintext
   and get a ciphertext (or vice versa :-) */
static int
__crypto_run_std(struct csession *ses_ptr, struct crypt_op *cop)
{
	char *data;
	char __user *src, *dst;
	struct scatterlist sg;
	size_t nbytes, bufsize;
	int ret = 0;

	nbytes = cop->len;
	data = (char *)__get_free_page(GFP_KERNEL);

	if (unlikely(!data))
		return -ENOMEM;

	bufsize = PAGE_SIZE < nbytes ? PAGE_SIZE : nbytes;

	src = cop->src;
	dst = cop->dst;

	while (nbytes > 0) {
		size_t current_len = nbytes > bufsize ? bufsize : nbytes;

		if (unlikely(copy_from_user(data, src, current_len))) {
			ret = -EFAULT;
			break;
		}

		sg_init_one(&sg, data, current_len);

		ret = hash_n_crypt(ses_ptr, cop, &sg, &sg, current_len);

		if (unlikely(ret))
			break;

		if (ses_ptr->cdata.init != 0) {
			if (unlikely(copy_to_user(dst, data, current_len))) {
				ret = -EFAULT;
				break;
			}
		}

		dst += current_len;
		nbytes -= current_len;
		src += current_len;
	}

	free_page((unsigned long)data);
	return ret;
}

void release_user_pages(struct page **pg, int pagecount)
{
	while (pagecount--) {
		if (!PageReserved(pg[pagecount]))
			SetPageDirty(pg[pagecount]);
		page_cache_release(pg[pagecount]);
	}
}

/* offset of buf in it's first page */
#define PAGEOFFSET(buf) ((unsigned long)buf & ~PAGE_MASK)

/* fetch the pages addr resides in into pg and initialise sg with them */
int __get_userbuf(uint8_t __user *addr, uint32_t len, int write,
		int pgcount, struct page **pg, struct scatterlist *sg)
{
	int ret, pglen, i = 0;
	struct scatterlist *sgp;

	down_write(&current->mm->mmap_sem);
	ret = get_user_pages(current, current->mm,
			(unsigned long)addr, pgcount, write, 0, pg, NULL);
	up_write(&current->mm->mmap_sem);
	if (ret != pgcount)
		return -EINVAL;

	sg_init_table(sg, pgcount);

	pglen = min((ptrdiff_t)(PAGE_SIZE - PAGEOFFSET(addr)), (ptrdiff_t)len);
	sg_set_page(sg, pg[i++], pglen, PAGEOFFSET(addr));

	len -= pglen;
	for (sgp = sg_next(sg); len; sgp = sg_next(sgp)) {
		pglen = min((uint32_t)PAGE_SIZE, len);
		sg_set_page(sgp, pg[i++], pglen, 0);
		len -= pglen;
	}
	sg_mark_end(sg_last(sg, pgcount));
	return 0;
}

/* make cop->src and cop->dst available in scatterlists */
static int get_userbuf(struct csession *ses, struct kernel_crypt_op *kcop,
                       struct scatterlist **src_sg, struct scatterlist **dst_sg,
                       int *tot_pages)
{
	int src_pagecount, dst_pagecount = 0, pagecount, write_src = 1;
	struct crypt_op *cop = &kcop->cop;

	if (cop->src == NULL)
		return -EINVAL;

	src_pagecount = PAGECOUNT(cop->src, cop->len);
	if (!ses->cdata.init) {		/* hashing only */
		write_src = 0;
	} else if (cop->src != cop->dst) {	/* non-in-situ transformation */
		if (cop->dst == NULL)
			return -EINVAL;

		dst_pagecount = PAGECOUNT(cop->dst, cop->len);
		write_src = 0;
	}
	(*tot_pages) = pagecount = src_pagecount + dst_pagecount;

	if (pagecount > ses->array_size) {
		struct scatterlist *sg;
		struct page **pages;
		int array_size;

		for (array_size = ses->array_size; array_size < pagecount;
		     array_size *= 2)
			;

		dprintk(2, KERN_DEBUG, "%s: reallocating to %d elements\n",
				__func__, array_size);
		pages = krealloc(ses->pages, array_size * sizeof(struct page *),
				 GFP_KERNEL);
		if (pages == NULL)
			return -ENOMEM;
		ses->pages = pages;
		sg = krealloc(ses->sg, array_size * sizeof(struct scatterlist),
			      GFP_KERNEL);
		if (sg == NULL)
			return -ENOMEM;
		ses->sg = sg;
		ses->array_size = array_size;
	}

	if (__get_userbuf(cop->src, cop->len, write_src,
			src_pagecount, ses->pages, ses->sg)) {
		dprintk(1, KERN_ERR,
			"failed to get user pages for data input\n");
		return -EINVAL;
	}
	(*src_sg) = (*dst_sg) = ses->sg;

	if (dst_pagecount) {
		(*dst_sg) = ses->sg + src_pagecount;

		if (__get_userbuf(cop->dst, cop->len, 1, dst_pagecount,
					ses->pages + src_pagecount, *dst_sg)) {
			dprintk(1, KERN_ERR,
				"failed to get user pages for data output\n");
			release_user_pages(ses->pages, src_pagecount);
			return -EINVAL;
		}
	}
	return 0;
}

/* This is the main crypto function - zero-copy edition */
static int
__crypto_run_zc(struct csession *ses_ptr, struct kernel_crypt_op *kcop)
{
	struct scatterlist *src_sg, *dst_sg;
	struct crypt_op *cop = &kcop->cop;
	int ret = 0, pagecount;

	ret = get_userbuf(ses_ptr, kcop, &src_sg, &dst_sg, &pagecount);
	if (unlikely(ret)) {
		dprintk(1, KERN_ERR, "Error getting user pages. \
					Falling back to non zero copy.\n");
		return __crypto_run_std(ses_ptr, cop);
	}

	ret = hash_n_crypt(ses_ptr, cop, src_sg, dst_sg, cop->len);

	release_user_pages(ses_ptr->pages, pagecount);
	return ret;
}

static int crypto_run(struct fcrypt *fcr, struct kernel_crypt_op *kcop)
{
	struct csession *ses_ptr;
	struct crypt_op *cop = &kcop->cop;
	int ret;

	if (unlikely(cop->op != COP_ENCRYPT && cop->op != COP_DECRYPT)) {
		dprintk(1, KERN_DEBUG, "invalid operation op=%u\n", cop->op);
		return -EINVAL;
	}

	/* this also enters ses_ptr->sem */
	ses_ptr = crypto_get_session_by_sid(fcr, cop->ses);
	if (unlikely(!ses_ptr)) {
		dprintk(1, KERN_ERR, "invalid session ID=0x%08X\n", cop->ses);
		return -EINVAL;
	}

	if (ses_ptr->hdata.init != 0 && !(cop->flags & COP_FLAG_UPDATE) &&
		!(cop->flags & COP_FLAG_FINAL)) {
		ret = cryptodev_hash_reset(&ses_ptr->hdata);
		if (unlikely(ret)) {
			dprintk(1, KERN_ERR,
				"error in cryptodev_hash_reset()\n");
			goto out_unlock;
		}
	}

	if (ses_ptr->cdata.init != 0) {
		int blocksize = ses_ptr->cdata.blocksize;

		if (unlikely(cop->len % blocksize)) {
			dprintk(1, KERN_ERR,
				"data size (%u) isn't a multiple \
				of block size (%u)\n",
				cop->len, blocksize);
			ret = -EINVAL;
			goto out_unlock;
		}

		cryptodev_cipher_set_iv(&ses_ptr->cdata, kcop->iv,
				min(ses_ptr->cdata.ivsize, kcop->ivlen));
	}

	if (cop->len != 0) {
		ret = __crypto_run_zc(ses_ptr, kcop);
		if (unlikely(ret))
			goto out_unlock;
	}

	if (ses_ptr->hdata.init != 0 &&
		((cop->flags & COP_FLAG_FINAL) ||
		   (!(cop->flags & COP_FLAG_UPDATE) || cop->len == 0))) {

		ret = cryptodev_hash_final(&ses_ptr->hdata, kcop->hash_output);
		if (unlikely(ret)) {
			dprintk(0, KERN_ERR, "CryptoAPI failure: %d\n", ret);
			goto out_unlock;
		}
	}

#if defined(CRYPTODEV_STATS)
	if (enable_stats) {
		/* this is safe - we check cop->op at the function entry */
		ses_ptr->stat[cop->op] += cop->len;
		if (ses_ptr->stat_max_size < cop->len)
			ses_ptr->stat_max_size = cop->len;
		ses_ptr->stat_count++;
	}
#endif

out_unlock:
	mutex_unlock(&ses_ptr->sem);
	return ret;
}

/* ====== /dev/crypto ====== */

static int
cryptodev_open(struct inode *inode, struct file *filp)
{
	struct crypt_priv *pcr;

	pcr = kmalloc(sizeof(*pcr), GFP_KERNEL);
	if (!pcr)
		return -ENOMEM;

	memset(pcr, 0, sizeof(*pcr));
	mutex_init(&pcr->fcrypt.sem);
	INIT_LIST_HEAD(&pcr->fcrypt.list);

	filp->private_data = pcr;
	return 0;
}

static int
cryptodev_release(struct inode *inode, struct file *filp)
{
	struct crypt_priv *pcr = filp->private_data;

	if (pcr) {
		crypto_finish_all_sessions(&pcr->fcrypt);
		kfree(pcr);
		filp->private_data = NULL;
	}

	return 0;
}

static int
clonefd(struct file *filp)
{
	int ret;
	ret = get_unused_fd();
	if (ret >= 0) {
			get_file(filp);
			fd_install(ret, filp);
	}

	return ret;
}

/* this function has to be called from process context */
static int fill_kcop_from_cop(struct kernel_crypt_op *kcop, struct fcrypt *fcr)
{
	struct crypt_op *cop = &kcop->cop;
	struct csession *ses_ptr;
	int rc;

	/* this also enters ses_ptr->sem */
	ses_ptr = crypto_get_session_by_sid(fcr, cop->ses);
	if (unlikely(!ses_ptr)) {
		dprintk(1, KERN_ERR, "invalid session ID=0x%08X\n", cop->ses);
		return -EINVAL;
	}
	kcop->ivlen = cop->iv ? ses_ptr->cdata.ivsize : 0;
	kcop->digestsize = ses_ptr->hdata.digestsize;
	mutex_unlock(&ses_ptr->sem);

	if (unlikely(rc = copy_from_user(kcop->iv, cop->iv, kcop->ivlen))) {
		dprintk(1, KERN_ERR,
		        "error copying IV (%d bytes), copy_from_user returned %d for address %lx\n",
		        kcop->ivlen, rc, (unsigned long)cop->iv);
		return -EFAULT;
	}

	return 0;
}

static int kcop_from_user(struct kernel_crypt_op *kcop,
                          struct fcrypt *fcr, void __user *arg)
{
	if (unlikely(copy_from_user(&kcop->cop, arg, sizeof(kcop->cop))))
		return -EFAULT;

	return fill_kcop_from_cop(kcop, fcr);
}

static long
cryptodev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg_)
{
	void __user *arg = (void __user *)arg_;
	int __user *p = arg;
	struct session_op sop;
	struct kernel_crypt_op kcop;
	struct crypt_priv *pcr = filp->private_data;
	struct fcrypt *fcr;
	uint32_t ses;
	int ret, fd;

	if (unlikely(!pcr))
		BUG();

	fcr = &pcr->fcrypt;

	switch (cmd) {
	case CIOCASYMFEAT:
		return put_user(0, p);
	case CRIOGET:
		fd = clonefd(filp);
		ret = put_user(fd, p);
		if (unlikely(ret)) {
			sys_close(fd);
			return ret;
		}
		return ret;
	case CIOCGSESSION:
		if (unlikely(copy_from_user(&sop, arg, sizeof(sop))))
			return -EFAULT;

		ret = crypto_create_session(fcr, &sop);
		if (unlikely(ret))
			return ret;
		ret = copy_to_user(arg, &sop, sizeof(sop));
		if (unlikely(ret)) {
			crypto_finish_session(fcr, sop.ses);
			return -EFAULT;
		}
		return ret;
	case CIOCFSESSION:
		ret = get_user(ses, (uint32_t __user *)arg);
		if (unlikely(ret))
			return ret;
		ret = crypto_finish_session(fcr, ses);
		return ret;
	case CIOCCRYPT:
		if (unlikely(ret = kcop_from_user(&kcop, fcr, arg)))
			return ret;

		ret = crypto_run(fcr, &kcop);
		if (unlikely(ret))
			return ret;
		ret = copy_to_user(kcop.cop.mac, kcop.hash_output, kcop.digestsize);
		if (unlikely(ret))
			return ret;
		if (unlikely(copy_to_user(arg, &kcop.cop, sizeof(kcop.cop))))
			return -EFAULT;
		return 0;

	default:
		return -EINVAL;
	}
}

/* compatibility code for 32bit userlands */
#ifdef CONFIG_COMPAT

static inline void
compat_to_session_op(struct compat_session_op *compat, struct session_op *sop)
{
	sop->cipher = compat->cipher;
	sop->mac = compat->mac;
	sop->keylen = compat->keylen;

	sop->key       = compat_ptr(compat->key);
	sop->mackeylen = compat->mackeylen;
	sop->mackey    = compat_ptr(compat->mackey);
	sop->ses       = compat->ses;
}

static inline void
session_op_to_compat(struct session_op *sop, struct compat_session_op *compat)
{
	compat->cipher = sop->cipher;
	compat->mac = sop->mac;
	compat->keylen = sop->keylen;

	compat->key       = ptr_to_compat(sop->key);
	compat->mackeylen = sop->mackeylen;
	compat->mackey    = ptr_to_compat(sop->mackey);
	compat->ses       = sop->ses;
}

static inline void
compat_to_crypt_op(struct compat_crypt_op *compat, struct crypt_op *cop)
{
	cop->ses = compat->ses;
	cop->op = compat->op;
	cop->flags = compat->flags;
	cop->len = compat->len;

	cop->src = compat_ptr(compat->src);
	cop->dst = compat_ptr(compat->dst);
	cop->mac = compat_ptr(compat->mac);
	cop->iv  = compat_ptr(compat->iv);
}

static inline void
crypt_op_to_compat(struct crypt_op *cop, struct compat_crypt_op *compat)
{
	compat->ses = cop->ses;
	compat->op = cop->op;
	compat->flags = cop->flags;
	compat->len = cop->len;

	compat->src = ptr_to_compat(cop->src);
	compat->dst = ptr_to_compat(cop->dst);
	compat->mac = ptr_to_compat(cop->mac);
	compat->iv  = ptr_to_compat(cop->iv);
}

static int compat_kcop_from_user(struct kernel_crypt_op *kcop,
                                 struct fcrypt *fcr, void __user *arg)
{
	struct compat_crypt_op compat_cop;

	if (unlikely(copy_from_user(&compat_cop, arg, sizeof(compat_cop))))
		return -EFAULT;
	compat_to_crypt_op(&compat_cop, &kcop->cop);

	return fill_kcop_from_cop(kcop, fcr);
}

static long
cryptodev_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg_)
{
	void __user *arg = (void __user *)arg_;
	struct fcrypt *fcr = file->private_data;
	struct session_op sop;
	struct compat_session_op compat_sop;
	struct kernel_crypt_op kcop;
	struct compat_crypt_op compat_cop;
	int ret;

	if (unlikely(!fcr))
		BUG();

	switch (cmd) {
	case CIOCASYMFEAT:
	case CRIOGET:
	case CIOCFSESSION:
		return cryptodev_ioctl(file, cmd, arg_);

	case COMPAT_CIOCGSESSION:
		if (unlikely(copy_from_user(&compat_sop, arg,
					    sizeof(compat_sop))))
			return -EFAULT;
		compat_to_session_op(&compat_sop, &sop);

		ret = crypto_create_session(fcr, &sop);
		if (unlikely(ret))
			return ret;

		session_op_to_compat(&sop, &compat_sop);
		ret = copy_to_user(arg, &compat_sop, sizeof(compat_sop));
		if (unlikely(ret)) {
			crypto_finish_session(fcr, sop.ses);
			return -EFAULT;
		}
		return ret;

	case COMPAT_CIOCCRYPT:
		ret = compat_kcop_from_user(&kcop, fcr, arg);
		if (unlikely(ret))
			return ret;

		ret = crypto_run(fcr, &kcop);
		if (unlikely(ret))
			return ret;

		ret = copy_to_user(kcop.cop.mac,
				kcop.hash_output, kcop.digestsize);
		if (unlikely(ret))
			return ret;

		crypt_op_to_compat(&kcop.cop, &compat_cop);
		if (unlikely(copy_to_user(arg, &compat_cop,
					  sizeof(compat_cop))))
			return -EFAULT;
		return 0;

	default:
		return -EINVAL;
	}
}

#endif /* CONFIG_COMPAT */

static const struct file_operations cryptodev_fops = {
	.owner = THIS_MODULE,
	.open = cryptodev_open,
	.release = cryptodev_release,
	.unlocked_ioctl = cryptodev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cryptodev_compat_ioctl,
#endif /* CONFIG_COMPAT */
};

static struct miscdevice cryptodev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "crypto",
	.fops = &cryptodev_fops,
};

static int __init
cryptodev_register(void)
{
	int rc;

	rc = misc_register(&cryptodev);
	if (unlikely(rc)) {
		printk(KERN_ERR PFX "registration of /dev/crypto failed\n");
		return rc;
	}

	return 0;
}

static void __exit
cryptodev_deregister(void)
{
	misc_deregister(&cryptodev);
}

/* ====== Module init/exit ====== */
static int __init init_cryptodev(void)
{
	int rc;

	rc = cryptodev_register();
	if (unlikely(rc))
		return rc;

	printk(KERN_INFO PFX "driver %s loaded.\n", VERSION);

	return 0;
}

static void __exit exit_cryptodev(void)
{
	cryptodev_deregister();
	printk(KERN_INFO PFX "driver unloaded.\n");
}

module_init(init_cryptodev);
module_exit(exit_cryptodev);

