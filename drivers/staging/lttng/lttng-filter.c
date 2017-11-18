/*
 * lttng-filter.c
 *
 * LTTng modules filter code.
 *
 * Copyright (C) 2010-2014 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
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

#include <linux/list.h>
#include <linux/slab.h>

#include "lttng-filter.h"

static const char *opnames[] = {
	[ FILTER_OP_UNKNOWN ] = "UNKNOWN",

	[ FILTER_OP_RETURN ] = "RETURN",

	/* binary */
	[ FILTER_OP_MUL ] = "MUL",
	[ FILTER_OP_DIV ] = "DIV",
	[ FILTER_OP_MOD ] = "MOD",
	[ FILTER_OP_PLUS ] = "PLUS",
	[ FILTER_OP_MINUS ] = "MINUS",
	[ FILTER_OP_RSHIFT ] = "RSHIFT",
	[ FILTER_OP_LSHIFT ] = "LSHIFT",
	[ FILTER_OP_BIN_AND ] = "BIN_AND",
	[ FILTER_OP_BIN_OR ] = "BIN_OR",
	[ FILTER_OP_BIN_XOR ] = "BIN_XOR",

	/* binary comparators */
	[ FILTER_OP_EQ ] = "EQ",
	[ FILTER_OP_NE ] = "NE",
	[ FILTER_OP_GT ] = "GT",
	[ FILTER_OP_LT ] = "LT",
	[ FILTER_OP_GE ] = "GE",
	[ FILTER_OP_LE ] = "LE",

	/* string binary comparators */
	[ FILTER_OP_EQ_STRING ] = "EQ_STRING",
	[ FILTER_OP_NE_STRING ] = "NE_STRING",
	[ FILTER_OP_GT_STRING ] = "GT_STRING",
	[ FILTER_OP_LT_STRING ] = "LT_STRING",
	[ FILTER_OP_GE_STRING ] = "GE_STRING",
	[ FILTER_OP_LE_STRING ] = "LE_STRING",

	/* s64 binary comparators */
	[ FILTER_OP_EQ_S64 ] = "EQ_S64",
	[ FILTER_OP_NE_S64 ] = "NE_S64",
	[ FILTER_OP_GT_S64 ] = "GT_S64",
	[ FILTER_OP_LT_S64 ] = "LT_S64",
	[ FILTER_OP_GE_S64 ] = "GE_S64",
	[ FILTER_OP_LE_S64 ] = "LE_S64",

	/* double binary comparators */
	[ FILTER_OP_EQ_DOUBLE ] = "EQ_DOUBLE",
	[ FILTER_OP_NE_DOUBLE ] = "NE_DOUBLE",
	[ FILTER_OP_GT_DOUBLE ] = "GT_DOUBLE",
	[ FILTER_OP_LT_DOUBLE ] = "LT_DOUBLE",
	[ FILTER_OP_GE_DOUBLE ] = "GE_DOUBLE",
	[ FILTER_OP_LE_DOUBLE ] = "LE_DOUBLE",

	/* Mixed S64-double binary comparators */
	[ FILTER_OP_EQ_DOUBLE_S64 ] = "EQ_DOUBLE_S64",
	[ FILTER_OP_NE_DOUBLE_S64 ] = "NE_DOUBLE_S64",
	[ FILTER_OP_GT_DOUBLE_S64 ] = "GT_DOUBLE_S64",
	[ FILTER_OP_LT_DOUBLE_S64 ] = "LT_DOUBLE_S64",
	[ FILTER_OP_GE_DOUBLE_S64 ] = "GE_DOUBLE_S64",
	[ FILTER_OP_LE_DOUBLE_S64 ] = "LE_DOUBLE_S64",

	[ FILTER_OP_EQ_S64_DOUBLE ] = "EQ_S64_DOUBLE",
	[ FILTER_OP_NE_S64_DOUBLE ] = "NE_S64_DOUBLE",
	[ FILTER_OP_GT_S64_DOUBLE ] = "GT_S64_DOUBLE",
	[ FILTER_OP_LT_S64_DOUBLE ] = "LT_S64_DOUBLE",
	[ FILTER_OP_GE_S64_DOUBLE ] = "GE_S64_DOUBLE",
	[ FILTER_OP_LE_S64_DOUBLE ] = "LE_S64_DOUBLE",

	/* unary */
	[ FILTER_OP_UNARY_PLUS ] = "UNARY_PLUS",
	[ FILTER_OP_UNARY_MINUS ] = "UNARY_MINUS",
	[ FILTER_OP_UNARY_NOT ] = "UNARY_NOT",
	[ FILTER_OP_UNARY_PLUS_S64 ] = "UNARY_PLUS_S64",
	[ FILTER_OP_UNARY_MINUS_S64 ] = "UNARY_MINUS_S64",
	[ FILTER_OP_UNARY_NOT_S64 ] = "UNARY_NOT_S64",
	[ FILTER_OP_UNARY_PLUS_DOUBLE ] = "UNARY_PLUS_DOUBLE",
	[ FILTER_OP_UNARY_MINUS_DOUBLE ] = "UNARY_MINUS_DOUBLE",
	[ FILTER_OP_UNARY_NOT_DOUBLE ] = "UNARY_NOT_DOUBLE",

	/* logical */
	[ FILTER_OP_AND ] = "AND",
	[ FILTER_OP_OR ] = "OR",

	/* load field ref */
	[ FILTER_OP_LOAD_FIELD_REF ] = "LOAD_FIELD_REF",
	[ FILTER_OP_LOAD_FIELD_REF_STRING ] = "LOAD_FIELD_REF_STRING",
	[ FILTER_OP_LOAD_FIELD_REF_SEQUENCE ] = "LOAD_FIELD_REF_SEQUENCE",
	[ FILTER_OP_LOAD_FIELD_REF_S64 ] = "LOAD_FIELD_REF_S64",
	[ FILTER_OP_LOAD_FIELD_REF_DOUBLE ] = "LOAD_FIELD_REF_DOUBLE",

	/* load from immediate operand */
	[ FILTER_OP_LOAD_STRING ] = "LOAD_STRING",
	[ FILTER_OP_LOAD_S64 ] = "LOAD_S64",
	[ FILTER_OP_LOAD_DOUBLE ] = "LOAD_DOUBLE",

	/* cast */
	[ FILTER_OP_CAST_TO_S64 ] = "CAST_TO_S64",
	[ FILTER_OP_CAST_DOUBLE_TO_S64 ] = "CAST_DOUBLE_TO_S64",
	[ FILTER_OP_CAST_NOP ] = "CAST_NOP",

	/* get context ref */
	[ FILTER_OP_GET_CONTEXT_REF ] = "GET_CONTEXT_REF",
	[ FILTER_OP_GET_CONTEXT_REF_STRING ] = "GET_CONTEXT_REF_STRING",
	[ FILTER_OP_GET_CONTEXT_REF_S64 ] = "GET_CONTEXT_REF_S64",
	[ FILTER_OP_GET_CONTEXT_REF_DOUBLE ] = "GET_CONTEXT_REF_DOUBLE",

	/* load userspace field ref */
	[ FILTER_OP_LOAD_FIELD_REF_USER_STRING ] = "LOAD_FIELD_REF_USER_STRING",
	[ FILTER_OP_LOAD_FIELD_REF_USER_SEQUENCE ] = "LOAD_FIELD_REF_USER_SEQUENCE",
};

const char *lttng_filter_print_op(enum filter_op op)
{
	if (op >= NR_FILTER_OPS)
		return "UNKNOWN";
	else
		return opnames[op];
}

static
int apply_field_reloc(struct lttng_event *event,
		struct bytecode_runtime *runtime,
		uint32_t runtime_len,
		uint32_t reloc_offset,
		const char *field_name)
{
	const struct lttng_event_desc *desc;
	const struct lttng_event_field *fields, *field = NULL;
	unsigned int nr_fields, i;
	struct field_ref *field_ref;
	struct load_op *op;
	uint32_t field_offset = 0;

	dbg_printk("Apply field reloc: %u %s\n", reloc_offset, field_name);

	/* Lookup event by name */
	desc = event->desc;
	if (!desc)
		return -EINVAL;
	fields = desc->fields;
	if (!fields)
		return -EINVAL;
	nr_fields = desc->nr_fields;
	for (i = 0; i < nr_fields; i++) {
		if (!strcmp(fields[i].name, field_name)) {
			field = &fields[i];
			break;
		}
		/* compute field offset */
		switch (fields[i].type.atype) {
		case atype_integer:
		case atype_enum:
			field_offset += sizeof(int64_t);
			break;
		case atype_array:
		case atype_sequence:
			field_offset += sizeof(unsigned long);
			field_offset += sizeof(void *);
			break;
		case atype_string:
			field_offset += sizeof(void *);
			break;
		default:
			return -EINVAL;
		}
	}
	if (!field)
		return -EINVAL;

	/* Check if field offset is too large for 16-bit offset */
	if (field_offset > LTTNG_KERNEL_FILTER_BYTECODE_MAX_LEN - 1)
		return -EINVAL;

	/* set type */
	op = (struct load_op *) &runtime->data[reloc_offset];
	field_ref = (struct field_ref *) op->data;
	switch (field->type.atype) {
	case atype_integer:
	case atype_enum:
		op->op = FILTER_OP_LOAD_FIELD_REF_S64;
		break;
	case atype_array:
	case atype_sequence:
		if (field->user)
			op->op = FILTER_OP_LOAD_FIELD_REF_USER_SEQUENCE;
		else
			op->op = FILTER_OP_LOAD_FIELD_REF_SEQUENCE;
		break;
	case atype_string:
		if (field->user)
			op->op = FILTER_OP_LOAD_FIELD_REF_USER_STRING;
		else
			op->op = FILTER_OP_LOAD_FIELD_REF_STRING;
		break;
	default:
		return -EINVAL;
	}
	/* set offset */
	field_ref->offset = (uint16_t) field_offset;
	return 0;
}

static
int apply_context_reloc(struct lttng_event *event,
		struct bytecode_runtime *runtime,
		uint32_t runtime_len,
		uint32_t reloc_offset,
		const char *context_name)
{
	struct field_ref *field_ref;
	struct load_op *op;
	struct lttng_ctx_field *ctx_field;
	int idx;

	dbg_printk("Apply context reloc: %u %s\n", reloc_offset, context_name);

	/* Get context index */
	idx = lttng_get_context_index(lttng_static_ctx, context_name);
	if (idx < 0)
		return -ENOENT;

	/* Check if idx is too large for 16-bit offset */
	if (idx > LTTNG_KERNEL_FILTER_BYTECODE_MAX_LEN - 1)
		return -EINVAL;

	/* Get context return type */
	ctx_field = &lttng_static_ctx->fields[idx];
	op = (struct load_op *) &runtime->data[reloc_offset];
	field_ref = (struct field_ref *) op->data;
	switch (ctx_field->event_field.type.atype) {
	case atype_integer:
	case atype_enum:
		op->op = FILTER_OP_GET_CONTEXT_REF_S64;
		break;
		/* Sequence and array supported as string */
	case atype_string:
	case atype_array:
	case atype_sequence:
		BUG_ON(ctx_field->event_field.user);
		op->op = FILTER_OP_GET_CONTEXT_REF_STRING;
		break;
	default:
		return -EINVAL;
	}
	/* set offset to context index within channel contexts */
	field_ref->offset = (uint16_t) idx;
	return 0;
}

static
int apply_reloc(struct lttng_event *event,
		struct bytecode_runtime *runtime,
		uint32_t runtime_len,
		uint32_t reloc_offset,
		const char *name)
{
	struct load_op *op;

	dbg_printk("Apply reloc: %u %s\n", reloc_offset, name);

	/* Ensure that the reloc is within the code */
	if (runtime_len - reloc_offset < sizeof(uint16_t))
		return -EINVAL;

	op = (struct load_op *) &runtime->data[reloc_offset];
	switch (op->op) {
	case FILTER_OP_LOAD_FIELD_REF:
		return apply_field_reloc(event, runtime, runtime_len,
			reloc_offset, name);
	case FILTER_OP_GET_CONTEXT_REF:
		return apply_context_reloc(event, runtime, runtime_len,
			reloc_offset, name);
	default:
		printk(KERN_WARNING "Unknown reloc op type %u\n", op->op);
		return -EINVAL;
	}
	return 0;
}

static
int bytecode_is_linked(struct lttng_filter_bytecode_node *filter_bytecode,
		struct lttng_event *event)
{
	struct lttng_bytecode_runtime *bc_runtime;

	list_for_each_entry(bc_runtime,
			&event->bytecode_runtime_head, node) {
		if (bc_runtime->bc == filter_bytecode)
			return 1;
	}
	return 0;
}

/*
 * Take a bytecode with reloc table and link it to an event to create a
 * bytecode runtime.
 */
static
int _lttng_filter_event_link_bytecode(struct lttng_event *event,
		struct lttng_filter_bytecode_node *filter_bytecode,
		struct list_head *insert_loc)
{
	int ret, offset, next_offset;
	struct bytecode_runtime *runtime = NULL;
	size_t runtime_alloc_len;

	if (!filter_bytecode)
		return 0;
	/* Bytecode already linked */
	if (bytecode_is_linked(filter_bytecode, event))
		return 0;

	dbg_printk("Linking...\n");

	/* We don't need the reloc table in the runtime */
	runtime_alloc_len = sizeof(*runtime) + filter_bytecode->bc.reloc_offset;
	runtime = kzalloc(runtime_alloc_len, GFP_KERNEL);
	if (!runtime) {
		ret = -ENOMEM;
		goto alloc_error;
	}
	runtime->p.bc = filter_bytecode;
	runtime->len = filter_bytecode->bc.reloc_offset;
	/* copy original bytecode */
	memcpy(runtime->data, filter_bytecode->bc.data, runtime->len);
	/*
	 * apply relocs. Those are a uint16_t (offset in bytecode)
	 * followed by a string (field name).
	 */
	for (offset = filter_bytecode->bc.reloc_offset;
			offset < filter_bytecode->bc.len;
			offset = next_offset) {
		uint16_t reloc_offset =
			*(uint16_t *) &filter_bytecode->bc.data[offset];
		const char *name =
			(const char *) &filter_bytecode->bc.data[offset + sizeof(uint16_t)];

		ret = apply_reloc(event, runtime, runtime->len, reloc_offset, name);
		if (ret) {
			goto link_error;
		}
		next_offset = offset + sizeof(uint16_t) + strlen(name) + 1;
	}
	/* Validate bytecode */
	ret = lttng_filter_validate_bytecode(runtime);
	if (ret) {
		goto link_error;
	}
	/* Specialize bytecode */
	ret = lttng_filter_specialize_bytecode(runtime);
	if (ret) {
		goto link_error;
	}
	runtime->p.filter = lttng_filter_interpret_bytecode;
	runtime->p.link_failed = 0;
	list_add_rcu(&runtime->p.node, insert_loc);
	dbg_printk("Linking successful.\n");
	return 0;

link_error:
	runtime->p.filter = lttng_filter_false;
	runtime->p.link_failed = 1;
	list_add_rcu(&runtime->p.node, insert_loc);
alloc_error:
	dbg_printk("Linking failed.\n");
	return ret;
}

void lttng_filter_sync_state(struct lttng_bytecode_runtime *runtime)
{
	struct lttng_filter_bytecode_node *bc = runtime->bc;

	if (!bc->enabler->enabled || runtime->link_failed)
		runtime->filter = lttng_filter_false;
	else
		runtime->filter = lttng_filter_interpret_bytecode;
}

/*
 * Link bytecode for all enablers referenced by an event.
 */
void lttng_enabler_event_link_bytecode(struct lttng_event *event,
		struct lttng_enabler *enabler)
{
	struct lttng_filter_bytecode_node *bc;
	struct lttng_bytecode_runtime *runtime;

	/* Can only be called for events with desc attached */
	WARN_ON_ONCE(!event->desc);

	/* Link each bytecode. */
	list_for_each_entry(bc, &enabler->filter_bytecode_head, node) {
		int found = 0, ret;
		struct list_head *insert_loc;

		list_for_each_entry(runtime,
				&event->bytecode_runtime_head, node) {
			if (runtime->bc == bc) {
				found = 1;
				break;
			}
		}
		/* Skip bytecode already linked */
		if (found)
			continue;

		/*
		 * Insert at specified priority (seqnum) in increasing
		 * order.
		 */
		list_for_each_entry_reverse(runtime,
				&event->bytecode_runtime_head, node) {
			if (runtime->bc->bc.seqnum < bc->bc.seqnum) {
				/* insert here */
				insert_loc = &runtime->node;
				goto add_within;
			}
		}
		/* Add to head to list */
		insert_loc = &event->bytecode_runtime_head;
	add_within:
		dbg_printk("linking bytecode\n");
		ret = _lttng_filter_event_link_bytecode(event, bc,
				insert_loc);
		if (ret) {
			dbg_printk("[lttng filter] warning: cannot link event bytecode\n");
		}
	}
}

/*
 * We own the filter_bytecode if we return success.
 */
int lttng_filter_enabler_attach_bytecode(struct lttng_enabler *enabler,
		struct lttng_filter_bytecode_node *filter_bytecode)
{
	list_add(&filter_bytecode->node, &enabler->filter_bytecode_head);
	return 0;
}

void lttng_free_enabler_filter_bytecode(struct lttng_enabler *enabler)
{
	struct lttng_filter_bytecode_node *filter_bytecode, *tmp;

	list_for_each_entry_safe(filter_bytecode, tmp,
			&enabler->filter_bytecode_head, node) {
		kfree(filter_bytecode);
	}
}

void lttng_free_event_filter_runtime(struct lttng_event *event)
{
	struct bytecode_runtime *runtime, *tmp;

	list_for_each_entry_safe(runtime, tmp,
			&event->bytecode_runtime_head, p.node) {
		kfree(runtime);
	}
}
