/*
 *  Notify processes whenever the time is modified via settimeofday.
 *  Copyright (C) 2006-2007 Wind River Systems, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/stat.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

/*
 * Called when the time is modified below with settimeofday in order
 * to notify all of the tasks that requested notification.
 */
void do_notify_timechange(void)
{
	struct task_struct *p;

	read_lock(&tasklist_lock);
	for_each_process(p) {
		if (p->settime_sig)
			group_send_sig_info(p->settime_sig, SEND_SIG_PRIV, p);
	}
	read_unlock(&tasklist_lock);
}

/*
 * Set a signal to be delivered to the calling process whenever
 * time of day is changed. A signal value of zero indicates that
 * no signal should be sent.  On success, the call returns the
 * old value of the signal.  A return value of -1 indicates an error.
 */
static int do_notify_settime(int sig)
{
	int ret = -EINVAL;

	if ((sig > 0) && (sig <= _NSIG)) {
		ret = current->settime_sig;
		current->settime_sig = sig;
	}

	return ret;
}

#ifdef CONFIG_PROC_FS
#define NOTIFY_SETTIME_PROCFS_NAME "notify_settime_signal"

static int notify_settime_signal_show(struct seq_file *m, void *v )
{
	seq_printf(m, "%d\n", current->settime_sig);
	return 0;
}

static int notify_settime_signal_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, notify_settime_signal_show, PDE_DATA(inode));
}

static ssize_t notify_settime_signal_proc_write(struct file *file,
					const char __user *buffer,
				       size_t count, loff_t *pos)
{
	char str[16];
	int sig = 0;

	if (count > sizeof(str)-1)
		return -EINVAL;

	memset(str, 0, sizeof(str));
	if (copy_from_user(str, buffer, count))
		return -EFAULT;

	sscanf(str, "%d", &sig);
	return do_notify_settime(sig);
}

static const struct file_operations notify_settime_signal_proc_fops = {
	.open		= notify_settime_signal_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= notify_settime_signal_proc_write,
};

static int __init notify_settime_init(void)
{
	int ret = 0;

	proc_create(NOTIFY_SETTIME_PROCFS_NAME,
		    S_IFREG | S_IRUGO | S_IWUSR, NULL,
		    &notify_settime_signal_proc_fops);

	return ret;
}
device_initcall(notify_settime_init);
#endif
