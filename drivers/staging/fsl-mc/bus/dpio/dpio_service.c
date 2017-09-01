/* Copyright 2014 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <linux/types.h>
#include "fsl_qbman_portal.h"
#include "../../include/mc.h"
#include "../../include/fsl_dpaa_io.h"
#include "fsl_dpio.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include "dpio-drv.h"
#include "qbman_debug.h"

#define UNIMPLEMENTED() pr_err("FOO: %s unimplemented!\n", __func__)

#define MAGIC_SERVICE 0xabcd9876
#define MAGIC_OBJECT 0x1234fedc

struct dpaa_io {
	/* If MAGIC_SERVICE, this is a group of objects, use the 'service' part
	 * of the union. If MAGIC_OBJECT, use the 'object' part of the union. If
	 * it's neither, something got corrupted. This is mainly to satisfy
	 * dpaa_io_from_registration(), which dereferences a caller-instantiated
	 * struct and so warrants a bug-checking step - hence the magic rather
	 * than a boolean.
	 */
	unsigned int magic;
	atomic_t refs;
	union {
		struct dpaa_io_service {
			spinlock_t lock;
			struct list_head list;
			cpumask_t cpus_notifications;
			cpumask_t cpus_stashing;
			int has_nonaffine;
			/* slight hack. record the special case of the
			 * "default service", because that's the case where we
			 * need to avoid a kfree() ... */
			int is_defservice;
		} service;
		struct dpaa_io_object {
			struct dpaa_io_desc dpio_desc;
			struct qbman_swp_desc swp_desc;
			struct qbman_swp *swp;
			/* If the object is part of a service, this is it (and
			 * 'node' is linked into the service's list) */
			struct dpaa_io *service;
			struct list_head node;
			/* Interrupt mask, as used with
			 * qbman_swp_interrupt_[gs]et_vanish(). This isn't
			 * locked, because the higher layer is driving all
			 * "ingress" processing. */
			uint32_t irq_mask;
			/* As part of simplifying assumptions, we provide an
			 * irq-safe lock for each type of DPIO operation that
			 * isn't innately lockless. The selection algorithms
			 * (which are simplified) require this, whereas
			 * eventually adherence to cpu-affinity will presumably
			 * relax the locking requirements. */
			spinlock_t lock_mgmt_cmd;
			spinlock_t lock_notifications;
			struct list_head notifications;
		} object;
	};
};

struct dpaa_io_store {
	unsigned int max;
	dma_addr_t paddr;
	struct ldpaa_dq *vaddr;
	void *alloced_addr; /* the actual return from kmalloc as it may
			       be adjusted for alignment purposes */
	uint8_t token; /* current token if busy, otherwise next token */
	unsigned int idx; /* position of the next-to-be-returned entry */
	struct qbman_swp *swp; /* portal used to issue VDQCR */
	struct device *dev; /* device used for DMA mapping */
};

static struct dpaa_io def_serv;

/**********************/
/* Internal functions */
/**********************/

static void service_init(struct dpaa_io *d, int is_defservice)
{
	struct dpaa_io_service *s = &d->service;

	d->magic = MAGIC_SERVICE;
	atomic_set(&d->refs, 1);
	spin_lock_init(&s->lock);
	INIT_LIST_HEAD(&s->list);
	cpumask_clear(&s->cpus_notifications);
	cpumask_clear(&s->cpus_stashing);
	s->has_nonaffine = 0;
	s->is_defservice = is_defservice;
}

/* Selection algorithms, stupid ones at that. These are to handle the case where
 * the given dpaa_io is a service, by choosing the non-service dpaa_io within it
 * to use. Two variants for now, one for cpu-sensitive selection, and another
 * which doesn't care about cpu.
 */
static inline struct dpaa_io *service_select_by_cpu(struct dpaa_io *d, int cpu)
{
	struct dpaa_io_service *ss;
	struct dpaa_io *o;
	unsigned long irqflags;

	if (!d)
		d = &def_serv;
	else if (d->magic == MAGIC_OBJECT)
		return d;
	BUG_ON(d->magic != MAGIC_SERVICE);

	/* TODO: this is about the dumbest and slowest selection algorithm you
	 * could imagine. (We're looking for something working first, and
	 * something efficient second...)
	 *
	 * Lock the service, iterate the linked-list looking for the first DPIO
	 * object matching the required cpu, ignore everything else about that
	 * DPIO, and choose it to do the operation! As a post-selection step,
	 * move the DPIO to the end of the list. It should improve
	 * load-balancing a little, though will make the list iterations
	 * slower...
	 */
	ss = &d->service;
	spin_lock_irqsave(&ss->lock, irqflags);
	/* If cpu==-1, choose the current cpu, now we're in atomic context */
	if (cpu < 0)
		cpu = smp_processor_id();
	list_for_each_entry(o, &ss->list, object.node)
		if (o->object.dpio_desc.cpu == cpu)
			goto found;
	/* No joy. Try the first nonaffine portal (bleurgh) */
	if (ss->has_nonaffine)
		list_for_each_entry(o, &ss->list, object.node)
			if (!o->object.dpio_desc.stash_affinity)
				goto found;
	/* No joy. Try the first object. Told you it was horrible. */
	if (!list_empty(&ss->list))
		o = list_entry(ss->list.next, struct dpaa_io, object.node);
	else
		o = NULL;
found:
	if (o) {
		list_del(&o->object.node);
		list_add_tail(&o->object.node, &ss->list);
	}
	spin_unlock_irqrestore(&ss->lock, irqflags);
	return o;
}

static inline struct dpaa_io *service_select_any(struct dpaa_io *d)
{
	struct dpaa_io_service *ss;
	struct dpaa_io *o;
	unsigned long irqflags;

	if (!d)
		d = &def_serv;
	else if (d->magic == MAGIC_OBJECT)
		return d;
	BUG_ON(d->magic != MAGIC_SERVICE);

	ss = &d->service;
	spin_lock_irqsave(&ss->lock, irqflags);
	if (!list_empty(&ss->list)) {
		o = list_entry(ss->list.next, struct dpaa_io, object.node);
		list_del(&o->object.node);
		list_add_tail(&o->object.node, &ss->list);
	} else
		o = NULL;
	spin_unlock_irqrestore(&ss->lock, irqflags);
	return o;
}

/**********************/
/* Exported functions */
/**********************/

struct dpaa_io *dpaa_io_create(const struct dpaa_io_desc *desc)
{
	struct dpaa_io *ret = kmalloc(sizeof(*ret), GFP_KERNEL);
	struct dpaa_io_object *o = &ret->object;

	if (!ret)
		return NULL;
	ret->magic = MAGIC_OBJECT;
	atomic_set(&ret->refs, 1);
	o->dpio_desc = *desc;
	o->swp_desc.cena_bar = o->dpio_desc.regs_cena;
	o->swp_desc.cinh_bar = o->dpio_desc.regs_cinh;
	o->swp = qbman_swp_init(&o->swp_desc);
	o->service = NULL;
	if (!o->swp) {
		kfree(ret);
		return NULL;
	}
	INIT_LIST_HEAD(&o->node);
	spin_lock_init(&o->lock_mgmt_cmd);
	spin_lock_init(&o->lock_notifications);
	INIT_LIST_HEAD(&o->notifications);
	if (!o->dpio_desc.has_irq)
		qbman_swp_interrupt_set_vanish(o->swp, 0xffffffff);
	else {
		/* For now only enable DQRR interrupts */
		qbman_swp_interrupt_set_trigger(o->swp,
						QBMAN_SWP_INTERRUPT_DQRI);
	}
	qbman_swp_interrupt_clear_status(o->swp, 0xffffffff);
	if (o->dpio_desc.receives_notifications)
		qbman_swp_push_set(o->swp, 0, 1);
	return ret;
}
EXPORT_SYMBOL(dpaa_io_create);

struct dpaa_io *dpaa_io_create_service(void)
{
	struct dpaa_io *ret = kmalloc(sizeof(*ret), GFP_KERNEL);

	if (ret)
		service_init(ret, 0);
	return ret;
}
EXPORT_SYMBOL(dpaa_io_create_service);

struct dpaa_io *dpaa_io_default_service(void)
{
	atomic_inc(&def_serv.refs);
	return &def_serv;
}
EXPORT_SYMBOL(dpaa_io_default_service);

void dpaa_io_down(struct dpaa_io *d)
{
	if (!atomic_dec_and_test(&d->refs))
		return;
	if (d->magic == MAGIC_SERVICE) {
		BUG_ON(!list_empty(&d->service.list));
		if (d->service.is_defservice)
			/* avoid the kfree()! */
			return;
	} else {
		BUG_ON(d->magic != MAGIC_OBJECT);
		BUG_ON(d->object.service);
		BUG_ON(!list_empty(&d->object.notifications));
	}
	kfree(d);
}
EXPORT_SYMBOL(dpaa_io_down);

int dpaa_io_service_add(struct dpaa_io *s, struct dpaa_io *o)
{
	struct dpaa_io_service *ss = &s->service;
	struct dpaa_io_object *oo = &o->object;
	int res = -EINVAL;

	if ((s->magic != MAGIC_SERVICE) || (o->magic != MAGIC_OBJECT))
		return res;
	atomic_inc(&o->refs);
	atomic_inc(&s->refs);
	spin_lock(&ss->lock);
	/* 'obj' must not already be associated with a service */
	if (!oo->service) {
		oo->service = s;
		list_add(&oo->node, &ss->list);
		if (oo->dpio_desc.receives_notifications)
			cpumask_set_cpu(oo->dpio_desc.cpu,
					&ss->cpus_notifications);
		if (oo->dpio_desc.stash_affinity)
			cpumask_set_cpu(oo->dpio_desc.cpu,
					&ss->cpus_stashing);
		if (!oo->dpio_desc.stash_affinity)
			ss->has_nonaffine = 1;
		/* success */
		res = 0;
	}
	spin_unlock(&ss->lock);
	if (res) {
		dpaa_io_down(s);
		dpaa_io_down(o);
	}
	return res;
}
EXPORT_SYMBOL(dpaa_io_service_add);

int dpaa_io_get_descriptor(struct dpaa_io *obj, struct dpaa_io_desc *desc)
{
	if (obj->magic == MAGIC_SERVICE)
		return -EINVAL;
	BUG_ON(obj->magic != MAGIC_OBJECT);
	*desc = obj->object.dpio_desc;
	return 0;
}
EXPORT_SYMBOL(dpaa_io_get_descriptor);

int dpaa_io_poll(struct dpaa_io *obj)
{
	const struct ldpaa_dq *dq;
	struct qbman_swp *swp;

	if (obj->magic != MAGIC_OBJECT)
		return -EINVAL;
	swp = obj->object.swp;
	dq = qbman_swp_dqrr_next(swp);
	if (dq) {
		if (qbman_dq_entry_is_FQDAN(dq)) {
			struct dpaa_io_notification_ctx *ctx;
			uint64_t q64;

			q64 = qbman_dq_entry_SCN_ctx(dq);
			ctx = (void *)q64;
			ctx->cb(ctx);
		} else
			pr_crit("Unrecognised/ignored DQRR entry\n");
		qbman_swp_dqrr_consume(swp, dq);
	}
	return 0;
}
EXPORT_SYMBOL(dpaa_io_poll);

int dpaa_io_preirq(struct dpaa_io *obj)
{
	struct qbman_swp *swp;
	uint32_t status;

	if (obj->magic != MAGIC_OBJECT)
		return -EINVAL;
	swp = obj->object.swp;
	status = qbman_swp_interrupt_read_status(swp);
	if (!status)
		return IRQ_NONE;
	qbman_swp_interrupt_set_inhibit(swp, 1);
	return IRQ_WAKE_THREAD;
}

int dpaa_io_irq(struct dpaa_io *obj)
{
	struct qbman_swp *swp;
	uint32_t status;

	if (obj->magic != MAGIC_OBJECT)
		return -EINVAL;
	swp = obj->object.swp;
	status = qbman_swp_interrupt_read_status(swp);
	if (!status)
		return IRQ_NONE;
	dpaa_io_poll(obj);
	qbman_swp_interrupt_clear_status(swp, status);
	qbman_swp_interrupt_set_inhibit(swp, 0);
	return IRQ_HANDLED;
}
EXPORT_SYMBOL(dpaa_io_irq);

int dpaa_io_pause_poll(struct dpaa_io *obj)
{
	UNIMPLEMENTED();
	return -EINVAL;
}
EXPORT_SYMBOL(dpaa_io_pause_poll);

int dpaa_io_resume_poll(struct dpaa_io *obj)
{
	UNIMPLEMENTED();
	return -EINVAL;
}
EXPORT_SYMBOL(dpaa_io_resume_poll);

void dpaa_io_service_notifications(struct dpaa_io *s, cpumask_t *mask)
{
	struct dpaa_io_service *ss = &s->service;

	BUG_ON(s->magic != MAGIC_SERVICE);
	cpumask_copy(mask, &ss->cpus_notifications);
}
EXPORT_SYMBOL(dpaa_io_service_notifications);

void dpaa_io_service_stashing(struct dpaa_io *s, cpumask_t *mask)
{
	struct dpaa_io_service *ss = &s->service;

	BUG_ON(s->magic != MAGIC_SERVICE);
	cpumask_copy(mask, &ss->cpus_stashing);
}
EXPORT_SYMBOL(dpaa_io_service_stashing);

int dpaa_io_service_has_nonaffine(struct dpaa_io *s)
{
	struct dpaa_io_service *ss = &s->service;

	BUG_ON(s->magic != MAGIC_SERVICE);
	return ss->has_nonaffine;
}
EXPORT_SYMBOL(dpaa_io_service_has_nonaffine);

int dpaa_io_service_register(struct dpaa_io *d,
			     struct dpaa_io_notification_ctx *ctx)
{
	unsigned long irqflags;

	d = service_select_by_cpu(d, ctx->desired_cpu);
	ctx->dpio_id = d->object.dpio_desc.dpio_id;
	ctx->qman64 = (uint64_t)ctx;
	ctx->dpio_private = d;
	spin_lock_irqsave(&d->object.lock_notifications, irqflags);
	list_add(&ctx->node, &d->object.notifications);
	spin_unlock_irqrestore(&d->object.lock_notifications, irqflags);
	return 0;
}
EXPORT_SYMBOL(dpaa_io_service_register);

int dpaa_io_service_deregister(struct dpaa_io *service,
			       struct dpaa_io_notification_ctx *ctx)
{
	struct dpaa_io *d = ctx->dpio_private;

	if (!service)
		service = &def_serv;
	BUG_ON((service != d) && (service != d->object.service));
	/* TBD: lock! */
	list_del(&ctx->node);
	return 0;
}
EXPORT_SYMBOL(dpaa_io_service_deregister);

int dpaa_io_service_rearm(struct dpaa_io *d,
			  struct dpaa_io_notification_ctx *ctx)
{
	unsigned long irqflags;
	int err;

	BUG_ON(ctx->is_cdan);
	d = service_select_any(d);
	if (!d)
		return -ENODEV;
	spin_lock_irqsave(&d->object.lock_mgmt_cmd, irqflags);
	err = qbman_swp_fq_schedule(d->object.swp, ctx->id);
	spin_unlock_irqrestore(&d->object.lock_mgmt_cmd, irqflags);
	return err;
}
EXPORT_SYMBOL(dpaa_io_service_rearm);

int dpaa_io_from_registration(struct dpaa_io_notification_ctx *ctx,
			      struct dpaa_io **io)
{
	struct dpaa_io_notification_ctx *tmp;
	struct dpaa_io *d = ctx->dpio_private;
	unsigned long irqflags;
	int ret = 0;

	BUG_ON(d->magic != MAGIC_OBJECT);
	/* Iterate the notifications associated with 'd' looking for a match. If
	 * not, we've been passed an unregistered ctx! */
	spin_lock_irqsave(&d->object.lock_notifications, irqflags);
	list_for_each_entry(tmp, &d->object.notifications, node)
		if (tmp == ctx)
			goto found;
	ret = -EINVAL;
found:
	spin_unlock_irqrestore(&d->object.lock_notifications, irqflags);
	if (!ret) {
		atomic_inc(&d->refs);
		*io = d;
	}
	return ret;
}
EXPORT_SYMBOL(dpaa_io_from_registration);

int dpaa_io_service_get_persistent(struct dpaa_io *service, int cpu,
				   struct dpaa_io **ret)
{
	if (cpu == -1)
		*ret = service_select_any(service);
	else
		*ret = service_select_by_cpu(service, cpu);
	if (*ret) {
		atomic_inc(&(*ret)->refs);
		return 0;
	}
	return -ENODEV;
}
EXPORT_SYMBOL(dpaa_io_service_get_persistent);

int dpaa_io_service_pull_fq(struct dpaa_io *d, uint32_t fqid,
			    struct dpaa_io_store *s)
{
	struct qbman_pull_desc pd;
	int err;

	qbman_pull_desc_clear(&pd);
	qbman_pull_desc_set_storage(&pd, s->vaddr, s->paddr, 1);
	qbman_pull_desc_set_numframes(&pd, s->max);
	qbman_pull_desc_set_token(&pd, s->token);
	qbman_pull_desc_set_fq(&pd, fqid);
	d = service_select_by_cpu(d, -1);
	if (d) {
		s->swp = d->object.swp;
		err = qbman_swp_pull(d->object.swp, &pd);
	}
	if (!d)
		return -ENODEV;
	if (err)
		s->swp = NULL;
	return err;
}
EXPORT_SYMBOL(dpaa_io_service_pull_fq);

int dpaa_io_service_pull_channel(struct dpaa_io *d, uint32_t channelid,
				 struct dpaa_io_store *s)
{
	UNIMPLEMENTED();
	return -EINVAL;
}
EXPORT_SYMBOL(dpaa_io_service_pull_channel);

int dpaa_io_service_enqueue_fq(struct dpaa_io *d,
			       uint32_t fqid,
			       const struct dpaa_fd *fd)
{
	struct qbman_eq_desc ed;

	d = service_select_any(d);
	if (!d)
		return -ENODEV;
	qbman_eq_desc_clear(&ed);
	qbman_eq_desc_set_no_orp(&ed, 0);
	qbman_eq_desc_set_fq(&ed, fqid);
	return qbman_swp_enqueue(d->object.swp, &ed,
				 (const struct qbman_fd *)fd);
}
EXPORT_SYMBOL(dpaa_io_service_enqueue_fq);

int dpaa_io_service_enqueue_qd(struct dpaa_io *d,
			       uint32_t qdid, uint8_t prio, uint16_t qdbin,
			       const struct dpaa_fd *fd)
{
	struct qbman_eq_desc ed;

	d = service_select_any(d);
	if (!d)
		return -ENODEV;
	qbman_eq_desc_clear(&ed);
	qbman_eq_desc_set_no_orp(&ed, 0);
	qbman_eq_desc_set_qd(&ed, qdid, qdbin, prio);
	return qbman_swp_enqueue(d->object.swp, &ed,
				 (const struct qbman_fd *)fd);
}
EXPORT_SYMBOL(dpaa_io_service_enqueue_qd);

int dpaa_io_service_release(struct dpaa_io *d,
			    uint32_t bpid,
			    const uint64_t *buffers,
			    unsigned int num_buffers)
{
	struct qbman_release_desc rd;

	d = service_select_any(d);
	if (!d)
		return -ENODEV;
	qbman_release_desc_clear(&rd);
	qbman_release_desc_set_bpid(&rd, bpid);
	return qbman_swp_release(d->object.swp, &rd, buffers, num_buffers);
}
EXPORT_SYMBOL(dpaa_io_service_release);

int dpaa_io_service_acquire(struct dpaa_io *d,
			    uint32_t bpid,
			    uint64_t *buffers,
			    unsigned int num_buffers)
{
	unsigned long irqflags;
	int err;

	d = service_select_any(d);
	if (!d)
		return -ENODEV;
	spin_lock_irqsave(&d->object.lock_mgmt_cmd, irqflags);
	err = qbman_swp_acquire(d->object.swp, bpid, buffers, num_buffers);
	spin_unlock_irqrestore(&d->object.lock_mgmt_cmd, irqflags);
	return err;
}
EXPORT_SYMBOL(dpaa_io_service_acquire);

static void store_token(struct dpaa_io_store *store, unsigned int idx,
			unsigned num)
{
	struct ldpaa_dq *dq = &store->vaddr[idx];

	qbman_dq_entry_set_oldtoken(dq, num, store->token++);
}

struct dpaa_io_store *dpaa_io_store_create(unsigned int max_frames,
					   struct device *dev)
{
	struct dpaa_io_store *ret = kmalloc(sizeof(*ret), GFP_KERNEL);
	size_t size;

	BUG_ON(!max_frames || (max_frames > 16));
	if (!ret)
		return NULL;
	ret->max = max_frames;
	size = max_frames * sizeof(struct ldpaa_dq) + 64;
	ret->alloced_addr = kmalloc(size, GFP_KERNEL);
	if (!ret->alloced_addr) {
		kfree(ret);
		return NULL;
	}
	ret->vaddr =  PTR_ALIGN(ret->alloced_addr, 64);
	ret->paddr = dma_map_single(dev, ret->vaddr,
				    sizeof(struct ldpaa_dq) * max_frames,
				    DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, ret->paddr)) {
		kfree(ret->alloced_addr);
		kfree(ret);
		return NULL;
	}
	ret->token = 0x53;
	ret->idx = 0;
	ret->dev = dev;
	store_token(ret, 0, ret->max);
	return ret;
}
EXPORT_SYMBOL(dpaa_io_store_create);

void dpaa_io_store_destroy(struct dpaa_io_store *s)
{
	dma_unmap_single(s->dev, s->paddr, sizeof(struct ldpaa_dq) * s->max,
			 DMA_FROM_DEVICE);
	kfree(s->alloced_addr);
	kfree(s);
}
EXPORT_SYMBOL(dpaa_io_store_destroy);

struct ldpaa_dq *dpaa_io_store_next(struct dpaa_io_store *s, int *is_last)
{
	int match;
	struct ldpaa_dq *ret = &s->vaddr[s->idx];

	match = qbman_dq_entry_has_newtoken(s->swp, ret, s->token);
	if (!match) {
		*is_last = 0;
		return NULL;
	}
	BUG_ON(!qbman_dq_entry_is_DQ(ret));
	s->idx++;
	if (ldpaa_dq_is_pull_complete(ret)) {
		*is_last = 1;
		if (s->idx < s->max)
			store_token(s, s->idx, s->max - s->idx);
		else
			s->token++;
		s->idx = 0;
		/* If we get an empty dequeue result to terminate a zero-results
		 * vdqcr, return NULL to the caller rather than expecting him to
		 * check non-NULL results every time. */
		if (!(ldpaa_dq_flags(ret) & LDPAA_DQ_STAT_VALIDFRAME))
			ret = NULL;
	} else
		*is_last = 0;
	return ret;
}
EXPORT_SYMBOL(dpaa_io_store_next);

#ifdef CONFIG_FSL_QBMAN_DEBUG
int dpaa_io_query_fq_count(struct dpaa_io *d, uint32_t fqid,
			   uint32_t *fcnt, uint32_t *bcnt)
{
	struct qbman_attr state;
	struct qbman_swp *swp;
	unsigned long irqflags;
	int ret;

	d = service_select_any(d);
	if (!d)
		return -ENODEV;

	swp = d->object.swp;
	spin_lock_irqsave(&d->object.lock_mgmt_cmd, irqflags);
	ret = qbman_fq_query_state(swp, fqid, &state);
	spin_unlock_irqrestore(&d->object.lock_mgmt_cmd, irqflags);
	if (ret)
		return ret;
	*fcnt = qbman_fq_state_frame_count(&state);
	*bcnt = qbman_fq_state_byte_count(&state);

	return 0;
}
#endif

/* module init/exit hooks called from dpio-drv.c. These are declared in
 * dpio-drv.h.
 */
int dpaa_io_service_driver_init(void)
{
	service_init(&def_serv, 1);
	return 0;
}

void dpaa_io_service_driver_exit(void)
{
	if (atomic_read(&def_serv.refs) != 1)
		pr_err("default DPIO service leaves dangling DPIO objects!\n");
}
