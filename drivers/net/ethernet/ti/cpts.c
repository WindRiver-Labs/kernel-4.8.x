/*
 * TI Common Platform Time Sync
 *
 * Copyright (C) 2012 Richard Cochran <richardcochran@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <linux/err.h>
#include <linux/if.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/ptp_classify.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>

#include "cpts.h"

#define CPTS_TS_COMP_PULSE_LENGTH_DEF	3

#define cpts_read32(c, r)	readl_relaxed(&c->reg->r)
#define cpts_write32(c, v, r)	writel_relaxed(v, &c->reg->r)

static int cpts_report_ts_events(struct cpts *cpts, bool pps_reload);

static int cpts_event_port(struct cpts_event *event)
{
	return (event->high >> PORT_NUMBER_SHIFT) & PORT_NUMBER_MASK;
}

static int event_expired(struct cpts_event *event)
{
	return time_after(jiffies, event->tmo);
}

static int event_type(struct cpts_event *event)
{
	return (event->high >> EVENT_TYPE_SHIFT) & EVENT_TYPE_MASK;
}

static int cpts_fifo_pop(struct cpts *cpts, u32 *high, u32 *low)
{
	u32 r = cpts_read32(cpts, intstat_raw);

	if (r & TS_PEND_RAW) {
		*high = cpts_read32(cpts, event_high);
		*low  = cpts_read32(cpts, event_low);
		cpts_write32(cpts, EVENT_POP, event_pop);
		return 0;
	}
	return -1;
}

static int cpts_purge_events(struct cpts *cpts)
{
	struct list_head *this, *next;
	struct cpts_event *event;
	int removed = 0;

	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_expired(event)) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			++removed;
		}
	}

	if (removed)
		dev_dbg(cpts->dev, "cpts: event pool cleaned up %d\n", removed);
	return removed ? 0 : -1;
}

/*
 * Returns zero if matching event type was found.
 */
static int cpts_fifo_read(struct cpts *cpts, int match)
{
	int i, type = -1;
	u32 hi, lo;
	struct cpts_event *event;

	for (i = 0; i < CPTS_FIFO_DEPTH; i++) {
		if (cpts_fifo_pop(cpts, &hi, &lo))
			break;

		if (list_empty(&cpts->pool) && cpts_purge_events(cpts)) {
			dev_err(cpts->dev, "cpts: event pool empty\n");
			return -1;
		}

		event = list_first_entry(&cpts->pool, struct cpts_event, list);
		event->tmo = jiffies +
			     msecs_to_jiffies(CPTS_EVENT_RX_TX_TIMEOUT);
		event->high = hi;
		event->low = lo;
		type = event_type(event);
		switch (type) {
		case CPTS_EV_HW:
		case CPTS_EV_COMP:
			event->tmo +=
				msecs_to_jiffies(CPTS_EVENT_HWSTAMP_TIMEOUT);
		case CPTS_EV_PUSH:
		case CPTS_EV_RX:
		case CPTS_EV_TX:
			list_del_init(&event->list);
			list_add_tail(&event->list, &cpts->events);
			break;
		case CPTS_EV_ROLL:
		case CPTS_EV_HALF:
			break;
		default:
			pr_err("cpts: unknown event type\n");
			break;
		}
		if (type == match)
			break;
	}
	return type == match ? 0 : -1;
}

static cycle_t cpts_systim_read(const struct cyclecounter *cc)
{
	u64 val = 0;
	struct cpts_event *event;
	struct list_head *this, *next;
	struct cpts *cpts = container_of(cc, struct cpts, cc);

	cpts_write32(cpts, TS_PUSH, ts_push);
	if (cpts_fifo_read(cpts, CPTS_EV_PUSH))
		pr_err("cpts: unable to obtain a time stamp\n");

	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_type(event) == CPTS_EV_PUSH) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			val = event->low;
			break;
		}
	}

	return val;
}

static cycle_t cpts_cc_ns2cyc(struct cpts *cpts, u64 nsecs)
{
	cycle_t cyc = (nsecs << cpts->cc.shift) + nsecs;

	do_div(cyc, cpts->cc.mult);

	return cyc;
}

static void cpts_ts_comp_disable(struct cpts *cpts)
{
	cpts_write32(cpts, 0, ts_comp_length);
}

static void cpts_ts_comp_enable(struct cpts *cpts)
{
	/* TS_COMP_LENGTH should be 0 while the TS_COMP_VAL value is
	 * being written
	 */
	cpts_write32(cpts, 0, ts_comp_length);
	cpts_write32(cpts, cpts->ts_comp_next, ts_comp_val);
	cpts_write32(cpts, cpts->ts_comp_length, ts_comp_length);
}

static void cpts_ts_comp_add_ns(struct cpts *cpts, s64 add_ns)
{
	cycle_t cyc_next;

	if (add_ns == NSEC_PER_SEC)
		/* avoid calculation */
		cyc_next = cpts->ts_comp_one_sec_cycs;
	else
		cyc_next = cpts_cc_ns2cyc(cpts, add_ns);

	cyc_next += cpts->ts_comp_next;
	cpts->ts_comp_next = cyc_next & cpts->cc.mask;
	pr_debug("cpts comp ts_comp_next: %u\n", cpts->ts_comp_next);
}

static void cpts_ts_comp_settime(struct cpts *cpts, s64 now_ns)
{
	struct timespec64 ts;

	if (cpts->ts_comp_enabled) {
		ts = ns_to_timespec64(now_ns);

		/* align pulse to next sec boundary and add one sec */
		cpts_ts_comp_add_ns(cpts, NSEC_PER_SEC - ts.tv_nsec);

		/* enable ts_comp pulse */
		cpts_ts_comp_enable(cpts);
	}
}

/* PTP clock operations */

static int cpts_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u64 adj;
	u32 diff, mult;
	int neg_adj = 0;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);
	u64 ns;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}
	mult = cpts->cc_mult;
	adj = mult;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);

	mutex_lock(&cpts->ptp_clk_mutex);

	spin_lock_irqsave(&cpts->lock, flags);
	if (cpts->ts_comp_enabled) {
		cpts_ts_comp_disable(cpts);
		/* if any, report existing pulse before adj */
		cpts_fifo_read(cpts, CPTS_EV_COMP);
		/* if any, report existing pulse before adj */
		cpts_report_ts_events(cpts, false);
	}

	timecounter_read(&cpts->tc);

	cpts->cc.mult = neg_adj ? mult - diff : mult + diff;
	/* get updated time with adj */
	ns = timecounter_read(&cpts->tc);
	cpts->ts_comp_next = cpts->tc.cycle_last;
	spin_unlock_irqrestore(&cpts->lock, flags);

	if (cpts->ts_comp_enabled)
		cpts->ts_comp_one_sec_cycs = cpts_cc_ns2cyc(cpts, NSEC_PER_SEC);
	cpts_ts_comp_settime(cpts, ns);

	mutex_unlock(&cpts->ptp_clk_mutex);

	return 0;
}

static int cpts_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);
	u64 ns;

	mutex_lock(&cpts->ptp_clk_mutex);

	spin_lock_irqsave(&cpts->lock, flags);
	if (cpts->ts_comp_enabled) {
		cpts_ts_comp_disable(cpts);
		/* if any, report existing pulse before adj */
		cpts_fifo_read(cpts, CPTS_EV_COMP);
		/* if any, report existing pulse before adj */
		cpts_report_ts_events(cpts, false);
	}

	timecounter_adjtime(&cpts->tc, delta);
	ns = timecounter_read(&cpts->tc);
	cpts->ts_comp_next = cpts->tc.cycle_last;
	spin_unlock_irqrestore(&cpts->lock, flags);

	cpts_ts_comp_settime(cpts, ns);

	mutex_unlock(&cpts->ptp_clk_mutex);

	return 0;
}

static int cpts_ptp_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	u64 ns;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	spin_lock_irqsave(&cpts->lock, flags);
	ns = timecounter_read(&cpts->tc);
	spin_unlock_irqrestore(&cpts->lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

static int cpts_ptp_settime(struct ptp_clock_info *ptp,
			    const struct timespec64 *ts)
{
	struct cpts *cpts = container_of(ptp, struct cpts, info);
	unsigned long flags;
	u64 ns;

	ns = timespec64_to_ns(ts);

	mutex_lock(&cpts->ptp_clk_mutex);

	spin_lock_irqsave(&cpts->lock, flags);
	if (cpts->ts_comp_enabled) {
		cpts_ts_comp_disable(cpts);
		/* if any, get existing pulse event before adj */
		cpts_fifo_read(cpts, CPTS_EV_COMP);
		/* if any, report existing pulse before adj */
		cpts_report_ts_events(cpts, false);
	}

	timecounter_init(&cpts->tc, &cpts->cc, ns);
	cpts->ts_comp_next = cpts->tc.cycle_last;
	spin_unlock_irqrestore(&cpts->lock, flags);

	cpts_ts_comp_settime(cpts, ns);

	mutex_unlock(&cpts->ptp_clk_mutex);

	return 0;
}

static int cpts_pps_enable(struct cpts *cpts, int on)
{
	struct timespec64 ts;
	unsigned long flags;
	u64 ns;

	if (cpts->ts_comp_enabled == on)
		return 0;

	mutex_lock(&cpts->ptp_clk_mutex);
	cpts->ts_comp_enabled = on;

	if (!on) {
		cpts_ts_comp_disable(cpts);
		mutex_unlock(&cpts->ptp_clk_mutex);
		return 0;
	}

	/* get current counter value */
	spin_lock_irqsave(&cpts->lock, flags);
	ns = timecounter_read(&cpts->tc);
	cpts->ts_comp_next = cpts->tc.cycle_last;
	spin_unlock_irqrestore(&cpts->lock, flags);

	ts = ns_to_timespec64(ns);
	/* align to next sec boundary and add one sec to avoid the situation
	 * when the current time is very close to the next second point and
	 * it might be possible that ts_comp_val will be configured to
	 * the time in the past.
	 */
	cpts_ts_comp_add_ns(cpts, 2 * NSEC_PER_SEC - ts.tv_nsec);

	/* enable ts_comp pulse */
	cpts_ts_comp_enable(cpts);

	if (cpts->ts_comp_enabled)
		/* poll for events faster - evry 200 ms */
		cpts->ov_check_period =
			msecs_to_jiffies(CPTS_EVENT_HWSTAMP_TIMEOUT);
	else if (!cpts->hw_ts_enable)
		cpts->ov_check_period = cpts->ov_check_period_slow;

	mod_delayed_work(system_wq, &cpts->overflow_work,
			 cpts->ov_check_period);

	mutex_unlock(&cpts->ptp_clk_mutex);

	return 0;
}

static int cpts_report_ts_events(struct cpts *cpts, bool pps_reload)
{
	struct list_head *this, *next;
	struct ptp_clock_event pevent;
	struct cpts_event *event;
	int reported = 0, ev;
	u64 ns;

	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		ev = event_type(event);
		if (ev == CPTS_EV_HW) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			/* report the event */
			pevent.timestamp =
				timecounter_cyc2time(&cpts->tc, event->low);
			pevent.type = PTP_CLOCK_EXTTS;
			pevent.index = cpts_event_port(event) - 1;
			ptp_clock_event(cpts->clock, &pevent);
			++reported;
			continue;
		}

		if (event_type(event) == CPTS_EV_COMP) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			if (cpts->ts_comp_next != event->low) {
				pr_err("cpts ts_comp mismatch: %08x %08x\n",
				       cpts->ts_comp_next, event->low);
				continue;
			} else
				pr_debug("cpts comp ev tstamp: %u\n",
					 event->low);

			/* report the event */
			ns = timecounter_cyc2time(&cpts->tc, event->low);
			pevent.type = PTP_CLOCK_PPSUSR;
			pevent.pps_times.ts_real = ns_to_timespec64(ns);
			ptp_clock_event(cpts->clock, &pevent);

			if (pps_reload) {
				/* reload: add ns to ts_comp */
				cpts_ts_comp_add_ns(cpts, NSEC_PER_SEC);
				/* enable ts_comp pulse with new val */
				cpts_ts_comp_enable(cpts);
			}
			++reported;
			continue;
		}
	}
	return reported;
}

/* HW TS */
static int cpts_extts_enable(struct cpts *cpts, u32 index, int on)
{
	unsigned long flags;
	u32 v;

	if (index >= cpts->info.n_ext_ts)
		return -ENXIO;

	if (((cpts->hw_ts_enable & BIT(index)) >> index) == on)
		return 0;

	mutex_lock(&cpts->ptp_clk_mutex);

	spin_lock_irqsave(&cpts->lock, flags);

	v = cpts_read32(cpts, control);
	if (on) {
		v |= BIT(8 + index);
		cpts->hw_ts_enable |= BIT(index);
	} else {
		v &= ~BIT(8 + index);
		cpts->hw_ts_enable &= ~BIT(index);
	}
	cpts_write32(cpts, v, control);

	spin_unlock_irqrestore(&cpts->lock, flags);

	if (cpts->hw_ts_enable)
		/* poll for events faster - evry 200 ms */
		cpts->ov_check_period =
 			msecs_to_jiffies(CPTS_EVENT_HWSTAMP_TIMEOUT);
	else if (!cpts->ts_comp_enabled)
		cpts->ov_check_period = cpts->ov_check_period_slow;

	mod_delayed_work(system_wq, &cpts->overflow_work,
			 cpts->ov_check_period);
	mutex_unlock(&cpts->ptp_clk_mutex);
	return 0;
}

static int cpts_ptp_enable(struct ptp_clock_info *ptp,
			   struct ptp_clock_request *rq, int on)
{
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		return cpts_extts_enable(cpts, rq->extts.index, on);
	case PTP_CLK_REQ_PPS:
		return cpts_pps_enable(cpts, on);
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static struct ptp_clock_info cpts_info = {
	.owner		= THIS_MODULE,
	.name		= "CTPS timer",
	.max_adj	= 1000000,
	.n_ext_ts	= 0,
	.n_pins		= 0,
	.pps		= 0,
	.adjfreq	= cpts_ptp_adjfreq,
	.adjtime	= cpts_ptp_adjtime,
	.gettime64	= cpts_ptp_gettime,
	.settime64	= cpts_ptp_settime,
	.enable		= cpts_ptp_enable,
};

static void cpts_overflow_check(struct work_struct *work)
{
	struct cpts *cpts = container_of(work, struct cpts, overflow_work.work);
	struct timespec64 ts;
	unsigned long flags;

	mutex_lock(&cpts->ptp_clk_mutex);
	spin_lock_irqsave(&cpts->lock, flags);
	ts = ns_to_timespec64(timecounter_read(&cpts->tc));
	spin_unlock_irqrestore(&cpts->lock, flags);

	if (cpts->hw_ts_enable || cpts->ts_comp_enabled)
		cpts_report_ts_events(cpts, true);
	mutex_unlock(&cpts->ptp_clk_mutex);

	pr_debug("cpts overflow check at %lld.%09lu\n", ts.tv_sec, ts.tv_nsec);
	schedule_delayed_work(&cpts->overflow_work, cpts->ov_check_period);
}

static int cpts_match(struct sk_buff *skb, unsigned int ptp_class,
		      u16 ts_seqid, u8 ts_msgtype)
{
	u16 *seqid;
	unsigned int offset = 0;
	u8 *msgtype, *data = skb->data;

	if (ptp_class & PTP_CLASS_VLAN)
		offset += VLAN_HLEN;

	switch (ptp_class & PTP_CLASS_PMASK) {
	case PTP_CLASS_IPV4:
		offset += ETH_HLEN + IPV4_HLEN(data + offset) + UDP_HLEN;
		break;
	case PTP_CLASS_IPV6:
		offset += ETH_HLEN + IP6_HLEN + UDP_HLEN;
		break;
	case PTP_CLASS_L2:
		offset += ETH_HLEN;
		break;
	default:
		return 0;
	}

	if (skb->len + ETH_HLEN < offset + OFF_PTP_SEQUENCE_ID + sizeof(*seqid))
		return 0;

	if (unlikely(ptp_class & PTP_CLASS_V1))
		msgtype = data + offset + OFF_PTP_CONTROL;
	else
		msgtype = data + offset;

	seqid = (u16 *)(data + offset + OFF_PTP_SEQUENCE_ID);

	return (ts_msgtype == (*msgtype & 0xf) && ts_seqid == ntohs(*seqid));
}

static u64 cpts_find_ts(struct cpts *cpts, struct sk_buff *skb, int ev_type)
{
	u64 ns = 0;
	struct cpts_event *event;
	struct list_head *this, *next;
	unsigned int class = ptp_classify_raw(skb);
	unsigned long flags;
	u16 seqid;
	u8 mtype;

	if (class == PTP_CLASS_NONE)
		return 0;

	spin_lock_irqsave(&cpts->lock, flags);
	cpts_fifo_read(cpts, CPTS_EV_PUSH);
	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_expired(event)) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			continue;
		}
		mtype = (event->high >> MESSAGE_TYPE_SHIFT) & MESSAGE_TYPE_MASK;
		seqid = (event->high >> SEQUENCE_ID_SHIFT) & SEQUENCE_ID_MASK;
		if (ev_type == event_type(event) &&
		    cpts_match(skb, class, seqid, mtype)) {
			ns = timecounter_cyc2time(&cpts->tc, event->low);
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			break;
		}
	}
	spin_unlock_irqrestore(&cpts->lock, flags);

	return ns;
}

int cpts_rx_timestamp(struct cpts *cpts, struct sk_buff *skb)
{
	u64 ns;
	struct skb_shared_hwtstamps *ssh;

	if (!cpts->rx_enable)
		return -EPERM;
	ns = cpts_find_ts(cpts, skb, CPTS_EV_RX);
	if (!ns)
		return -ENOENT;
	ssh = skb_hwtstamps(skb);
	memset(ssh, 0, sizeof(*ssh));
	ssh->hwtstamp = ns_to_ktime(ns);

	return 0;
}
EXPORT_SYMBOL_GPL(cpts_rx_timestamp);

int cpts_tx_timestamp(struct cpts *cpts, struct sk_buff *skb)
{
	u64 ns;
	struct skb_shared_hwtstamps ssh;

	if (!(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS))
		return -EPERM;
	ns = cpts_find_ts(cpts, skb, CPTS_EV_TX);
	if (!ns)
		return -ENOENT;
	memset(&ssh, 0, sizeof(ssh));
	ssh.hwtstamp = ns_to_ktime(ns);
	skb_tstamp_tx(skb, &ssh);

	return 0;
}
EXPORT_SYMBOL_GPL(cpts_tx_timestamp);

int cpts_register(struct cpts *cpts)
{
	int err, i;
	u32 control;

	INIT_LIST_HEAD(&cpts->events);
	INIT_LIST_HEAD(&cpts->pool);
	for (i = 0; i < CPTS_MAX_EVENTS; i++)
		list_add(&cpts->pool_data[i].list, &cpts->pool);

	clk_enable(cpts->refclk);

	control = CPTS_EN;
	if (cpts->caps & CPTS_CAP_TS_COMP_EN) {
		if (cpts->caps & CPTS_CAP_TS_COMP_POL_LOW_SEL)
			control &= ~TS_COMP_POL;
		else
			control |= TS_COMP_POL;
	}
	cpts_write32(cpts, control, control);
	cpts_write32(cpts, TS_PEND_EN, int_enable);

	cpts->cc.mult = cpts->cc_mult;
	timecounter_init(&cpts->tc, &cpts->cc, ktime_to_ns(ktime_get_real()));

	cpts->clock = ptp_clock_register(&cpts->info, cpts->dev);
	if (IS_ERR(cpts->clock)) {
		err = PTR_ERR(cpts->clock);
		cpts->clock = NULL;
		goto err_ptp;
	}
	cpts->phc_index = ptp_clock_index(cpts->clock);

	schedule_delayed_work(&cpts->overflow_work, cpts->ov_check_period);
	return 0;

err_ptp:
	clk_enable(cpts->refclk);
	return err;
}
EXPORT_SYMBOL_GPL(cpts_register);

void cpts_unregister(struct cpts *cpts)
{
	if (WARN_ON(!cpts->clock))
		return;

	cancel_delayed_work_sync(&cpts->overflow_work);

	ptp_clock_unregister(cpts->clock);
	cpts->clock = NULL;

	cpts_write32(cpts, 0, int_enable);
	cpts_write32(cpts, 0, control);

	clk_disable(cpts->refclk);
}

static void cpts_calc_mult_shift(struct cpts *cpts)
{
	u64 frac, maxsec, ns;
	u32 freq, mult, shift;

	freq = clk_get_rate(cpts->refclk);

	/* Calc the maximum number of seconds which we can run before
	 * wrapping around.
	 */
	maxsec = cpts->cc.mask;
	do_div(maxsec, freq);
	if (maxsec > 600 && cpts->cc.mask > UINT_MAX)
		maxsec = 600;

	/* Calc overflow check period (maxsec / 2) */
	cpts->ov_check_period = (HZ * maxsec) / 2;
	cpts->ov_check_period_slow = cpts->ov_check_period;

	dev_info(cpts->dev, "cpts: overflow check period %lu\n",
		 cpts->ov_check_period);

	if (cpts->cc_mult || cpts->cc.shift)
		return;

	clocks_calc_mult_shift(&mult, &shift, freq, NSEC_PER_SEC, maxsec);

	cpts->cc_mult = mult;
	cpts->cc.mult = mult;
	cpts->cc.shift = shift;

	frac = 0;
	ns = cyclecounter_cyc2ns(&cpts->cc, freq, cpts->cc.mask, &frac);

	dev_info(cpts->dev,
		 "CPTS: ref_clk_freq:%u calc_mult:%u calc_shift:%u error:%lld nsec/sec\n",
		 freq, cpts->cc_mult, cpts->cc.shift, (ns - NSEC_PER_SEC));
}

static int cpts_of_parse(struct cpts *cpts, struct device_node *node)
{
	int ret = -EINVAL;
	u32 prop;

	cpts->cc_mult = 0;
	if (!of_property_read_u32(node, "cpts_clock_mult", &prop))
		cpts->cc_mult = prop;

	cpts->cc.shift = 0;
	if (!of_property_read_u32(node, "cpts_clock_shift", &prop))
		cpts->cc.shift = prop;

	if ((cpts->cc_mult && !cpts->cc.shift) ||
	    (!cpts->cc_mult && cpts->cc.shift))
		goto of_error;

	if (!of_property_read_u32(node, "cpts-rftclk-sel", &prop)) {
		if (prop & ~CPTS_RFTCLK_SEL_MASK) {
			dev_err(cpts->dev, "cpts: invalid cpts_rftclk_sel.\n");
			goto of_error;
		}
		cpts->caps |= CPTS_CAP_RFTCLK_SEL;
		cpts->rftclk_sel = prop & CPTS_RFTCLK_SEL_MASK;
	}

	if (of_property_read_bool(node, "cpts-ts-comp-length")) {
		cpts->caps |= CPTS_CAP_TS_COMP_EN;
		cpts->ts_comp_length = CPTS_TS_COMP_PULSE_LENGTH_DEF;
	}

	if (cpts->caps & CPTS_CAP_TS_COMP_EN) {
		ret = of_property_read_u32(node, "cpts-ts-comp-length", &prop);
		if (!ret)
			cpts->ts_comp_length = prop;

		if (of_property_read_bool(node, "cpts-ts-comp-polarity-low"))
			cpts->caps |= CPTS_CAP_TS_COMP_POL_LOW_SEL;
	}

	if (!of_property_read_u32(node, "cpts-ext-ts-inputs", &prop))
		cpts->ext_ts_inputs = prop;

	return 0;

of_error:
	dev_err(cpts->dev, "CPTS: Missing property in the DT.\n");
	return ret;
}

struct cpts *cpts_create(struct device *dev, void __iomem *regs,
			 struct device_node *node)
{
	struct cpts *cpts;
	int ret;

	if (!regs || !dev)
		return ERR_PTR(-EINVAL);

	cpts = devm_kzalloc(dev, sizeof(*cpts), GFP_KERNEL);
	if (!cpts)
		return ERR_PTR(-ENOMEM);

	cpts->dev = dev;
	cpts->reg = (struct cpsw_cpts __iomem *)regs;
	spin_lock_init(&cpts->lock);
	mutex_init(&cpts->ptp_clk_mutex);
	INIT_DELAYED_WORK(&cpts->overflow_work, cpts_overflow_check);

	ret = cpts_of_parse(cpts, node);
	if (ret)
		return ERR_PTR(ret);

	cpts->refclk = devm_clk_get(dev, "cpts");
	if (IS_ERR(cpts->refclk)) {
		dev_err(dev, "Failed to get cpts refclk\n");
		return ERR_PTR(PTR_ERR(cpts->refclk));
	}

	clk_prepare(cpts->refclk);

	if (cpts->caps & CPTS_CAP_RFTCLK_SEL)
		cpts_write32(cpts, cpts->rftclk_sel, rftclk_sel);

	cpts->cc.read = cpts_systim_read;
	cpts->cc.mask = CLOCKSOURCE_MASK(32);
	cpts->info = cpts_info;

	if (cpts->ext_ts_inputs)
		cpts->info.n_ext_ts = cpts->ext_ts_inputs;

	cpts_calc_mult_shift(cpts);

	if (cpts->caps & CPTS_CAP_TS_COMP_EN) {
		cpts->info.pps = 1;
		cpts->ts_comp_one_sec_cycs = clk_get_rate(cpts->refclk);
	}

	return cpts;
}

void cpts_release(struct cpts *cpts)
{
	if (!cpts)
		return;

	if (WARN_ON(!cpts->clock))
		return;

	clk_unprepare(cpts->refclk);
}
EXPORT_SYMBOL_GPL(cpts_unregister);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI CPTS ALE driver");
