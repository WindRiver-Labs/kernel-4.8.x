/*
 * lttng-events-reset.h
 *
 * Copyright (C) 2010-2012 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; only
 * version 2.1 of the License.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/* Reset macros used within TRACE_EVENT to "nothing" */

#undef __field_full
#define __field_full(_type, _item, _order, _base)

#undef __array_enc_ext
#define __array_enc_ext(_type, _item, _length, _order, _base, _encoding)

#undef __dynamic_array_enc_ext
#define __dynamic_array_enc_ext(_type, _item, _length, _order, _base, _encoding)

#undef __dynamic_array_enc_ext_2
#define __dynamic_array_enc_ext_2(_type, _item, _length1, _length2, _order, _base, _encoding)

#undef __dynamic_array_len
#define __dynamic_array_len(_type, _item, _length)

#undef __string
#define __string(_item, _src)

#undef tp_assign
#define tp_assign(dest, src)

#undef tp_memcpy
#define tp_memcpy(dest, src, len)

#undef tp_memcpy_dyn
#define tp_memcpy_dyn(dest, src, len)

#undef tp_strcpy
#define tp_strcpy(dest, src)

#undef __get_str
#define __get_str(field)

#undef __get_dynamic_array
#define __get_dynamic_array(field)

#undef __get_dynamic_array_len
#define __get_dynamic_array_len(field)

#undef TP_PROTO
#define TP_PROTO(args...)

#undef TP_ARGS
#define TP_ARGS(args...)

#undef TP_locvar
#define TP_locvar(...)

#undef TP_code
#define TP_code(...)

#undef TP_STRUCT__entry
#define TP_STRUCT__entry(args...)

#undef TP_fast_assign
#define TP_fast_assign(args...)

#undef __perf_count
#define __perf_count(args...)

#undef __perf_addr
#define __perf_addr(args...)

#undef TP_perf_assign
#define TP_perf_assign(args...)

#undef TP_printk
#define TP_printk(args...)

#undef LTTNG_TRACEPOINT_EVENT_CLASS_CODE
#define LTTNG_TRACEPOINT_EVENT_CLASS_CODE(_name, _proto, _args, _locvar, _code, _tstruct, _assign, _print)

#undef LTTNG_TRACEPOINT_EVENT_CLASS_CODE_NOARGS
#define LTTNG_TRACEPOINT_EVENT_CLASS_CODE_NOARGS(_name, _locvar, _code, _tstruct, _assign, _print)

#undef LTTNG_TRACEPOINT_EVENT_INSTANCE_MAP
#define LTTNG_TRACEPOINT_EVENT_INSTANCE_MAP(_template, _name, _map, _proto, _args)

#undef LTTNG_TRACEPOINT_EVENT_INSTANCE_MAP_NOARGS
#define LTTNG_TRACEPOINT_EVENT_INSTANCE_MAP_NOARGS(_template, _name, _map)
