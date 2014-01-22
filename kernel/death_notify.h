/*
 * kernel/death_notify.h, Process death notification support
 *
 * Copyright (c) 2006-2014 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#ifndef _KERNEL_DEATH_NOTIFY_H
#define _KERNEL_DEATH_NOTIFY_H

#ifdef CONFIG_SIGEXIT

struct signotifier {
	struct task_struct *notify_tsk;
	struct list_head notify_list;
	struct list_head monitor_list;
	int sig;
	unsigned int events;
};

extern int do_notify_task_state(unsigned long arg);
extern void do_notify_others(struct task_struct *tsk,
					struct siginfo *info);
extern void release_notify_others(struct task_struct *p);

#else /* !CONFIG_SIGEXIT */

static inline void do_notify_others(struct task_struct *tsk,
					struct siginfo *info) {}
static inline void release_notify_others(struct task_struct *p) {}

#endif /* CONFIG_SIGEXIT */
#endif
