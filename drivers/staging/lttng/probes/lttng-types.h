/*
 * Protect against multiple inclusion of structure declarations, but run the
 * stages below each time.
 */
#ifndef _LTTNG_PROBES_LTTNG_TYPES_H
#define _LTTNG_PROBES_LTTNG_TYPES_H

#include <linux/seq_file.h>
#include "lttng.h"
#include "../ltt-events.h"

#ifdef __KERNEL__
# include <asm/byteorder.h>
# ifdef __BIG_ENDIAN
#  define __BYTE_ORDER __BIG_ENDIAN
# elif defined(__LITTLE_ENDIAN)
#  define __BYTE_ORDER __LITTLE_ENDIAN
# else
#  error "unknown endianness"
# endif
#ifndef __BIG_ENDIAN
# define __BIG_ENDIAN 4321
#endif
#ifndef __LITTLE_ENDIAN
# define __LITTLE_ENDIAN 1234
#endif
#else
# include <endian.h>
#endif

#endif /* _LTTNG_PROBES_LTTNG_TYPES_H */


/* Export enumerations */

#ifdef STAGE_EXPORT_ENUMS

#undef TRACE_EVENT_TYPE
#define TRACE_EVENT_TYPE(_name, _abstract_type, args...)

#undef TRACE_EVENT_ENUM
#define TRACE_EVENT_ENUM(_name, _entries...)				\
	const struct lttng_enum_entry __trace_event_enum_##_name[] = {	\
		PARAMS(_entries)					\
	};

/* Enumeration entry (single value) */
#undef V
#define V(_string)		{ _string, _string, #_string}

/* Enumeration entry (range) */
#undef R
#define R(_string, _range_start, _range_end)				\
	{ _range_start, _range_end, #_string }

#endif /* STAGE_EXPORT_ENUMS */


/* Export named types */

#ifdef STAGE_EXPORT_TYPES

#undef TRACE_EVENT_TYPE___enum
#define TRACE_EVENT_TYPE___enum(_name, _container_type)			\
		{							\
		  .name = #_name,					\
		  .container_type = __type_integer(_container_type, __BYTE_ORDER), \
		  .entries = __trace_event_enum_##_name, \
		  .len = ARRAY_SIZE(__trace_event_enum_##_name), \
		},

/* Local declaration */
#undef TRACE_EVENT_TYPE
#define TRACE_EVENT_TYPE(_name, _abstract_type, args...)	\
		TRACE_EVENT_TYPE___##_abstract_type(_name, args)

#undef TRACE_EVENT_ENUM
#define TRACE_EVENT_ENUM(_name, _entries...)

#endif /* STAGE_EXPORT_TYPES */
