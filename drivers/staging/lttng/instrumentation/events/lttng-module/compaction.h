#undef TRACE_SYSTEM
#define TRACE_SYSTEM compaction

#if !defined(LTTNG_TRACE_COMPACTION_H) || defined(TRACE_HEADER_MULTI_READ)
#define LTTNG_TRACE_COMPACTION_H

#include "../../../probes/lttng-tracepoint-event.h"
#include <linux/types.h>
#include <linux/version.h>
#include <trace/events/gfpflags.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0)

LTTNG_TRACEPOINT_EVENT_CLASS(compaction_isolate_template,

	TP_PROTO(unsigned long start_pfn,
		unsigned long end_pfn,
		unsigned long nr_scanned,
		unsigned long nr_taken),

	TP_ARGS(start_pfn, end_pfn, nr_scanned, nr_taken),

	TP_STRUCT__entry(
		__field(unsigned long, start_pfn)
		__field(unsigned long, end_pfn)
		__field(unsigned long, nr_scanned)
		__field(unsigned long, nr_taken)
	),

	TP_fast_assign(
		tp_assign(start_pfn, start_pfn)
		tp_assign(end_pfn, end_pfn)
		tp_assign(nr_scanned, nr_scanned)
		tp_assign(nr_taken, nr_taken)
	),

	TP_printk("range=(0x%lx ~ 0x%lx) nr_scanned=%lu nr_taken=%lu",
		__entry->start_pfn,
		__entry->end_pfn,
		__entry->nr_scanned,
		__entry->nr_taken)
)

LTTNG_TRACEPOINT_EVENT_INSTANCE(compaction_isolate_template, mm_compaction_isolate_migratepages,

	TP_PROTO(unsigned long start_pfn,
		unsigned long end_pfn,
		unsigned long nr_scanned,
		unsigned long nr_taken),

	TP_ARGS(start_pfn, end_pfn, nr_scanned, nr_taken)
)

LTTNG_TRACEPOINT_EVENT_INSTANCE(compaction_isolate_template, mm_compaction_isolate_freepages,

	TP_PROTO(unsigned long start_pfn,
		unsigned long end_pfn,
		unsigned long nr_scanned,
		unsigned long nr_taken),

	TP_ARGS(start_pfn, end_pfn, nr_scanned, nr_taken)
)

#else /* #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0) */

LTTNG_TRACEPOINT_EVENT_CLASS(compaction_isolate_template,

	TP_PROTO(unsigned long nr_scanned,
		unsigned long nr_taken),

	TP_ARGS(nr_scanned, nr_taken),

	TP_STRUCT__entry(
		__field(unsigned long, nr_scanned)
		__field(unsigned long, nr_taken)
	),

	TP_fast_assign(
		tp_assign(nr_scanned, nr_scanned)
		tp_assign(nr_taken, nr_taken)
	),

	TP_printk("nr_scanned=%lu nr_taken=%lu",
		__entry->nr_scanned,
		__entry->nr_taken)
)

LTTNG_TRACEPOINT_EVENT_INSTANCE_MAP(compaction_isolate_template,

	mm_compaction_isolate_migratepages,

	compaction_isolate_migratepages,

	TP_PROTO(unsigned long nr_scanned,
		unsigned long nr_taken),

	TP_ARGS(nr_scanned, nr_taken)
)

LTTNG_TRACEPOINT_EVENT_INSTANCE_MAP(compaction_isolate_template,

	mm_compaction_isolate_freepages,

	compaction_isolate_freepages,

	TP_PROTO(unsigned long nr_scanned,
		unsigned long nr_taken),

	TP_ARGS(nr_scanned, nr_taken)
)

#endif /* #else #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0) */

#if LTTNG_KERNEL_RANGE(3,12,30, 3,13,0) || \
	LTTNG_KERNEL_RANGE(3,14,25, 3,15,0) || \
	(LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0))
LTTNG_TRACEPOINT_EVENT_MAP(mm_compaction_migratepages,

	compaction_migratepages,

	TP_PROTO(unsigned long nr_all,
		int migrate_rc,
		struct list_head *migratepages),

	TP_ARGS(nr_all, migrate_rc, migratepages),

	TP_STRUCT__entry(
		__field(unsigned long, nr_migrated)
		__field(unsigned long, nr_failed)
	),

	TP_fast_assign(
		tp_assign(nr_migrated,
			nr_all -
			(migrate_rc >= 0 ? migrate_rc :
				({
					unsigned long nr_failed = 0;
					struct list_head *page_lru;

					list_for_each(page_lru, migratepages)
						nr_failed++;
					nr_failed;
				})))
		tp_assign(nr_failed,
				({
					unsigned long nr_failed = 0;
					struct list_head *page_lru;

					list_for_each(page_lru, migratepages)
						nr_failed++;
					nr_failed;
				}))
	),

	TP_printk("nr_migrated=%lu nr_failed=%lu",
		__entry->nr_migrated,
		__entry->nr_failed)
)
#else /* #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)) */
LTTNG_TRACEPOINT_EVENT_MAP(mm_compaction_migratepages,

	compaction_migratepages,

	TP_PROTO(unsigned long nr_migrated,
		unsigned long nr_failed),

	TP_ARGS(nr_migrated, nr_failed),

	TP_STRUCT__entry(
		__field(unsigned long, nr_migrated)
		__field(unsigned long, nr_failed)
	),

	TP_fast_assign(
		tp_assign(nr_migrated, nr_migrated)
		tp_assign(nr_failed, nr_failed)
	),

	TP_printk("nr_migrated=%lu nr_failed=%lu",
		__entry->nr_migrated,
		__entry->nr_failed)
)
#endif /* #else #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)) */

#endif /* LTTNG_TRACE_COMPACTION_H */

/* This part must be outside protection */
#include "../../../probes/define_trace.h"
