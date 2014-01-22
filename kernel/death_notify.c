/*
 * kernel/death_notify.c, Process death notification support
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

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/prctl.h>
#include <linux/uaccess.h>

#include "death_notify.h"

static void unlink_status_notifier(struct signotifier *n)
{
	list_del(&n->monitor_list);
	list_del(&n->notify_list);
	kfree(n);
}

static void handle_already_monitoring(struct signotifier *node,
       struct task_state_notify_info *args,
       struct task_state_notify_info *oldargs)
{
	/* Store the old values */
	oldargs->sig = node->sig;
	oldargs->events = node->events;

	/* We know that args->sig is 0 or a valid signal. */
	if (args->sig > 0) {
		/* Update the new values */
		node->sig = args->sig;
		node->events = args->events;
	} else if (!args->sig) {
		/* args->sig of 0 means to deregister */
		unlink_status_notifier(node);
	}
}

static void setup_new_node(struct task_struct *p,
	struct signotifier *node,
	struct task_state_notify_info *args)
{
	node->notify_tsk = current;
	node->sig = args->sig;
	node->events = args->events;

	/* Add this node to the list of notification requests
	 * for the specified process.
	 */
	list_add_tail(&node->notify_list, &p->notify);

	/* Also add this node to the list of monitor requests
	 * for the current process.
	 */
	list_add_tail(&node->monitor_list, &current->monitor);
}


/* Returns 0 if arguments are valid, 1 if they are not. */
static int invalid_args(struct task_state_notify_info *args)
{
	int ret = 1;

	if (args->pid <= 0)
		goto out;

	/* Sig of -1 implies query, sig of 0 implies deregistration.
	 * Otherwise sig must be positive and within range.
	 */
	if ((args->sig < -1) || (args->sig > _NSIG))
		goto out;

	/* If positive sig, must have valid events. */
	if (args->sig > 0) {
		if (!args->events || (args->events >= (1 << (NSIGCHLD+1))))
			goto out;
	}

	ret = 0;
out:
	return ret;
}

/* Notify those registered for process state updates via do_notify_task_state().
 *
 * Note: we only notify processes for events in which they have registered
 * interest.
 *
 * Must be called holding a lock on tasklist_lock.
 */
void do_notify_others(struct task_struct *tsk, struct siginfo *info)
{
	struct signotifier *node;
	unsigned int events;

	/* This method of generating the event bit must be
	 * matched in the userspace library.
	 */
	events = 1 << (info->si_code & 0xFF);

	list_for_each_entry(node, &tsk->notify, notify_list) {
		if (events & node->events) {
			info->si_signo = node->sig;
			group_send_sig_info(node->sig, info, node->notify_tsk);
		}
	}
}

void release_notify_others(struct task_struct *p)
{
	struct signotifier *n, *t;

	/* Need to clean up any outstanding requests where we
	 * wanted to be notified when others died.
	 */
	list_for_each_entry_safe(n, t, &p->monitor, monitor_list) {
		unlink_status_notifier(n);
	}

	/* Also need to clean up any outstanding requests where others
	 * wanted to be notified when we died.
	 */
	list_for_each_entry_safe(n, t, &p->notify, notify_list) {
		unlink_status_notifier(n);
	}
}

/* If the config is defined, then processes can call this routine
 * to request notification when the specified task's state changes.
 * On the death (or other state change) of the specified process,
 * we will send them the specified signal if the event is listed
 * in their event bitfield.
 *
 * A sig of 0 means that we want to deregister.
 *
 * The sig/events fields are value/result.  On success we update them
 * to reflect what they were before the call.
 *
 * Returns error code on error, on success we return 0.
 */
int do_notify_task_state(unsigned long arg)
{
	int err;
	struct task_struct *p;
	struct signotifier *node, *tmp;
	struct task_state_notify_info args, oldargs;

	if (copy_from_user(&args, (struct task_state_notify_info __user *)arg,
			sizeof(args)))
		return -EFAULT;
	oldargs.pid = args.pid;

	/* Validate the arguments passed in. */
	err = -EINVAL;
	if (invalid_args(&args))
		goto out;

	/* We must hold a write lock on tasklist_lock to add the notification
	 * later on, and we need some lock on tasklist_lock for
	 * find_task_by_pid(), so may as well take the write lock now.
	 * Must use write_lock_irq().
	 */
	write_lock_irq(&tasklist_lock);

	err = -ESRCH;
	p = find_task_by_vpid(args.pid);
	if (!p)
		goto unlock_out;

	/* Now we know pid exists, unlikely to fail. */
	err = 0;

	/* Check if we're already monitoring the specified pid. If so, update
	 * the monitoring parameters and return the old ones.
	 */
	list_for_each_entry(tmp, &p->notify, notify_list) {
		if (tmp->notify_tsk == current) {
			handle_already_monitoring(tmp, &args, &oldargs);
			goto unlock_out;
		}
	}

	/* If we get here, we're not currently monitoring the process. */
	oldargs.sig = 0;
	oldargs.events = 0;

	/* If we wanted to set up a new monitor, do it now. If we didn't
	 * manage to allocate memory for the new node, then we return
	 * an appropriate error.
	 */
	if (args.sig > 0) {
		node = kmalloc(sizeof(*node), GFP_ATOMIC);
		if (node)
			setup_new_node(p, node, &args);
		else
			err = -ENOMEM;
	}

unlock_out:
	write_unlock_irq(&tasklist_lock);

	/* Copy the old values back to caller. */
	if (copy_to_user((struct task_state_notify_info __user *)arg,
			&oldargs, sizeof(oldargs)))
		err = -EFAULT;

out:
	return err;
}
