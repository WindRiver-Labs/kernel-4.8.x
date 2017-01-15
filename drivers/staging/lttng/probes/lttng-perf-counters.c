/*
 * (C) Copyright	2009-2011 -
 * 		Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
 *
 * LTTng performance monitoring counters (perf-counters) integration module.
 *
 * Dual LGPL v2.1/GPL v2 license.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/perf_event.h>
#include <linux/list.h>
#include "../ltt-events.h"
#include "../wrapper/ringbuffer/frontend_types.h"
#include "../wrapper/vmalloc.h"
#include "../ltt-tracer.h"

/*
 * TODO: Add CPU hotplug support.
 */

static DEFINE_MUTEX(perf_counter_mutex);
static LIST_HEAD(perf_counter_contexts);

static
void perf_counter_record(struct lttng_ctx_field *field,
			 struct lib_ring_buffer_ctx *ctx,
			 struct ltt_channel *chan)
{
	struct perf_event *event;
	uint64_t value;

	event = field->u.perf_counter.e[ctx->cpu];
	event->pmu->read(event);
	value = local64_read(&event->count);
	lib_ring_buffer_align_ctx(ctx,
		ltt_alignof(field->type.u.basic.integer.alignment / CHAR_BIT));
	chan->ops->event_write(ctx, &value, sizeof(value));
}

static
void overflow_callback(struct perf_event *event, int nmi,
		       struct perf_sample_data *data,
		       struct pt_regs *regs)
{
}

static
void lttng_destroy_perf_counter_field(struct lttng_ctx_field *field)
{
	struct perf_event **events = field->u.perf_counter.e;
	int cpu;

	mutex_lock(&perf_counter_mutex);
	list_del(&field->u.perf_counter.head);
	for_each_online_cpu(cpu)
		perf_event_release_kernel(events[cpu]);
	mutex_unlock(&perf_counter_mutex);
	kfree(field->u.perf_counter.attr);
	kfree(events);
}

int lttng_add_perf_counter_to_ctx(uint32_t type,
				  uint64_t config,
				  struct lttng_ctx **ctx)
{
	struct lttng_ctx_field *field;
	struct perf_event **events;
	struct perf_event_attr *attr;
	int ret;
	int cpu;

	events = kzalloc(num_possible_cpus() * sizeof(*events), GFP_KERNEL);
	if (!events)
		return -ENOMEM;

	attr = kzalloc(sizeof(*field->u.perf_counter.attr), GFP_KERNEL);
	if (!attr) {
		ret = -ENOMEM;
		goto error_attr;
	}

	attr->type = type;
	attr->config = config;
	attr->size = sizeof(struct perf_event_attr);
	attr->pinned = 1;
	attr->disabled = 0;

	mutex_lock(&perf_counter_mutex);

	for_each_online_cpu(cpu) {
		events[cpu] = perf_event_create_kernel_counter(attr,
					cpu, NULL, overflow_callback);
		if (!events[cpu]) {
			ret = -EINVAL;
			goto error;
		}
	}

	field = lttng_append_context(ctx);
	if (!field) {
		ret = -ENOMEM;
		goto error;
	}
	field->destroy = lttng_destroy_perf_counter_field;

	field->name = "dummyname";//TODO: lookup_counter_name(type, config);
	field->type.atype = atype_integer;
	field->type.u.basic.integer.size = sizeof(unsigned long) * CHAR_BIT;
	field->type.u.basic.integer.alignment = ltt_alignof(unsigned long) * CHAR_BIT;
	field->type.u.basic.integer.signedness = 0;
	field->type.u.basic.integer.reverse_byte_order = 0;
	field->type.u.basic.integer.base = 10;
	field->type.u.basic.integer.encoding = lttng_encode_none;
	field->callback = perf_counter_record;
	field->u.perf_counter.e = events;
	field->u.perf_counter.attr = attr;

	list_add(&field->u.perf_counter.head, &perf_counter_contexts);
	mutex_unlock(&perf_counter_mutex);

	wrapper_vmalloc_sync_all();
	return 0;

error:
	for_each_online_cpu(cpu) {
		if (events[cpu])
			perf_event_release_kernel(events[cpu]);
	}
	mutex_unlock(&perf_counter_mutex);
	kfree(attr);
error_attr:
	kfree(events);
	return ret;
}

MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("Linux Trace Toolkit Perf Support");
