/*
 * ltt-probes.c
 *
 * Copyright 2010 (c) - Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
 *
 * Holds LTTng probes registry.
 */

#include <linux/module.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include "ltt-events.h"

static LIST_HEAD(probe_list);
static DEFINE_MUTEX(probe_mutex);

static
const struct lttng_event_desc *find_event(const char *name)
{
	struct lttng_probe_desc *probe_desc;
	int i;

	list_for_each_entry(probe_desc, &probe_list, head) {
		for (i = 0; i < probe_desc->nr_events; i++) {
			if (!strcmp(probe_desc->event_desc[i].name, name))
				return &probe_desc->event_desc[i];
		}
	}
	return NULL;
}

/*
 * TODO: registration of probe descriptions in dynamically allocated memory (not
 * directly in a module memory) will require some care for refcounting: it's
 * currently done by just refcounting the module in event_get/put.
 */
int ltt_probe_register(struct lttng_probe_desc *desc)
{
	int ret = 0;
	int i;

	mutex_lock(&probe_mutex);
	/*
	 * TODO: This is O(N^2). Turn into a hash table when probe registration
	 * overhead becomes an issue.
	 */
	for (i = 0; i < desc->nr_events; i++) {
		if (find_event(desc->event_desc[i].name)) {
			ret = -EEXIST;
			goto end;
		}
	}
	list_add(&desc->head, &probe_list);
end:
	mutex_unlock(&probe_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(ltt_probe_register);

void ltt_probe_unregister(struct lttng_probe_desc *desc)
{
	mutex_lock(&probe_mutex);
	list_del(&desc->head);
	mutex_unlock(&probe_mutex);
}
EXPORT_SYMBOL_GPL(ltt_probe_unregister);

const struct lttng_event_desc *ltt_event_get(const char *name)
{
	const struct lttng_event_desc *event;
	int ret;

	mutex_lock(&probe_mutex);
	event = find_event(name);
	mutex_unlock(&probe_mutex);
	if (!event)
		return NULL;
	ret = try_module_get(__module_text_address((unsigned long) event));
	WARN_ON_ONCE(!ret);
	return event;
}
EXPORT_SYMBOL_GPL(ltt_event_get);

void ltt_event_put(const struct lttng_event_desc *event)
{
	module_put(__module_text_address((unsigned long) event));
}
EXPORT_SYMBOL_GPL(ltt_event_put);
