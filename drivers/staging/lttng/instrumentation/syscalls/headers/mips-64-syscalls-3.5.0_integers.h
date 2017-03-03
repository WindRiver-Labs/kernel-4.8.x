/* THIS FILE IS AUTO-GENERATED. DO NOT EDIT */
#ifndef CREATE_SYSCALL_TABLE

#if !defined(_TRACE_SYSCALLS_INTEGERS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SYSCALLS_INTEGERS_H

#include <linux/tracepoint.h>
#include <linux/syscalls.h>
#include "mips-64-syscalls-3.5.0_integers_override.h"
#include "syscalls_integers_override.h"

SC_DECLARE_EVENT_CLASS_NOARGS(syscalls_noargs,
	TP_STRUCT__entry(),
	TP_fast_assign(),
	TP_printk()
)
#ifndef OVERRIDE_64_sys_sgetmask
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_sgetmask)
#endif
#ifndef OVERRIDE_64_sys_sched_yield
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_sched_yield)
#endif
#ifndef OVERRIDE_64_sys_pause
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_pause)
#endif
#ifndef OVERRIDE_64_sys_getpid
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_getpid)
#endif
#ifndef OVERRIDE_64_sys_getuid
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_getuid)
#endif
#ifndef OVERRIDE_64_sys_getgid
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_getgid)
#endif
#ifndef OVERRIDE_64_sys_geteuid
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_geteuid)
#endif
#ifndef OVERRIDE_64_sys_getegid
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_getegid)
#endif
#ifndef OVERRIDE_64_sys_getppid
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_getppid)
#endif
#ifndef OVERRIDE_64_sys_getpgrp
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_getpgrp)
#endif
#ifndef OVERRIDE_64_sys_setsid
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_setsid)
#endif
#ifndef OVERRIDE_64_sys_munlockall
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_munlockall)
#endif
#ifndef OVERRIDE_64_sys_vhangup
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_vhangup)
#endif
#ifndef OVERRIDE_64_sys_sync
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_sync)
#endif
#ifndef OVERRIDE_64_sys_gettid
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_gettid)
#endif
#ifndef OVERRIDE_64_sys_restart_syscall
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_restart_syscall)
#endif
#ifndef OVERRIDE_64_sys_inotify_init
SC_DEFINE_EVENT_NOARGS(syscalls_noargs, sys_inotify_init)
#endif
#ifndef OVERRIDE_64_sys_nice
SC_TRACE_EVENT(sys_nice,
	TP_PROTO(int increment),
	TP_ARGS(increment),
	TP_STRUCT__entry(__field(int, increment)),
	TP_fast_assign(tp_assign(increment, increment)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_ssetmask
SC_TRACE_EVENT(sys_ssetmask,
	TP_PROTO(int newmask),
	TP_ARGS(newmask),
	TP_STRUCT__entry(__field(int, newmask)),
	TP_fast_assign(tp_assign(newmask, newmask)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_personality
SC_TRACE_EVENT(sys_personality,
	TP_PROTO(unsigned int personality),
	TP_ARGS(personality),
	TP_STRUCT__entry(__field(unsigned int, personality)),
	TP_fast_assign(tp_assign(personality, personality)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_close
SC_TRACE_EVENT(sys_close,
	TP_PROTO(unsigned int fd),
	TP_ARGS(fd),
	TP_STRUCT__entry(__field(unsigned int, fd)),
	TP_fast_assign(tp_assign(fd, fd)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_brk
SC_TRACE_EVENT(sys_brk,
	TP_PROTO(unsigned long brk),
	TP_ARGS(brk),
	TP_STRUCT__entry(__field(unsigned long, brk)),
	TP_fast_assign(tp_assign(brk, brk)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_dup
SC_TRACE_EVENT(sys_dup,
	TP_PROTO(unsigned int fildes),
	TP_ARGS(fildes),
	TP_STRUCT__entry(__field(unsigned int, fildes)),
	TP_fast_assign(tp_assign(fildes, fildes)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_alarm
SC_TRACE_EVENT(sys_alarm,
	TP_PROTO(unsigned int seconds),
	TP_ARGS(seconds),
	TP_STRUCT__entry(__field(unsigned int, seconds)),
	TP_fast_assign(tp_assign(seconds, seconds)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_exit
SC_TRACE_EVENT(sys_exit,
	TP_PROTO(int error_code),
	TP_ARGS(error_code),
	TP_STRUCT__entry(__field(int, error_code)),
	TP_fast_assign(tp_assign(error_code, error_code)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_fsync
SC_TRACE_EVENT(sys_fsync,
	TP_PROTO(unsigned int fd),
	TP_ARGS(fd),
	TP_STRUCT__entry(__field(unsigned int, fd)),
	TP_fast_assign(tp_assign(fd, fd)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_fdatasync
SC_TRACE_EVENT(sys_fdatasync,
	TP_PROTO(unsigned int fd),
	TP_ARGS(fd),
	TP_STRUCT__entry(__field(unsigned int, fd)),
	TP_fast_assign(tp_assign(fd, fd)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_fchdir
SC_TRACE_EVENT(sys_fchdir,
	TP_PROTO(unsigned int fd),
	TP_ARGS(fd),
	TP_STRUCT__entry(__field(unsigned int, fd)),
	TP_fast_assign(tp_assign(fd, fd)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_umask
SC_TRACE_EVENT(sys_umask,
	TP_PROTO(int mask),
	TP_ARGS(mask),
	TP_STRUCT__entry(__field(int, mask)),
	TP_fast_assign(tp_assign(mask, mask)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_setuid
SC_TRACE_EVENT(sys_setuid,
	TP_PROTO(uid_t uid),
	TP_ARGS(uid),
	TP_STRUCT__entry(__field(uid_t, uid)),
	TP_fast_assign(tp_assign(uid, uid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_setgid
SC_TRACE_EVENT(sys_setgid,
	TP_PROTO(gid_t gid),
	TP_ARGS(gid),
	TP_STRUCT__entry(__field(gid_t, gid)),
	TP_fast_assign(tp_assign(gid, gid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_getpgid
SC_TRACE_EVENT(sys_getpgid,
	TP_PROTO(pid_t pid),
	TP_ARGS(pid),
	TP_STRUCT__entry(__field(pid_t, pid)),
	TP_fast_assign(tp_assign(pid, pid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_setfsuid
SC_TRACE_EVENT(sys_setfsuid,
	TP_PROTO(uid_t uid),
	TP_ARGS(uid),
	TP_STRUCT__entry(__field(uid_t, uid)),
	TP_fast_assign(tp_assign(uid, uid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_setfsgid
SC_TRACE_EVENT(sys_setfsgid,
	TP_PROTO(gid_t gid),
	TP_ARGS(gid),
	TP_STRUCT__entry(__field(gid_t, gid)),
	TP_fast_assign(tp_assign(gid, gid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_getsid
SC_TRACE_EVENT(sys_getsid,
	TP_PROTO(pid_t pid),
	TP_ARGS(pid),
	TP_STRUCT__entry(__field(pid_t, pid)),
	TP_fast_assign(tp_assign(pid, pid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_32_personality
SC_TRACE_EVENT(sys_32_personality,
	TP_PROTO(unsigned long personality),
	TP_ARGS(personality),
	TP_STRUCT__entry(__field(unsigned long, personality)),
	TP_fast_assign(tp_assign(personality, personality)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_sched_getscheduler
SC_TRACE_EVENT(sys_sched_getscheduler,
	TP_PROTO(pid_t pid),
	TP_ARGS(pid),
	TP_STRUCT__entry(__field(pid_t, pid)),
	TP_fast_assign(tp_assign(pid, pid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_sched_get_priority_max
SC_TRACE_EVENT(sys_sched_get_priority_max,
	TP_PROTO(int policy),
	TP_ARGS(policy),
	TP_STRUCT__entry(__field(int, policy)),
	TP_fast_assign(tp_assign(policy, policy)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_sched_get_priority_min
SC_TRACE_EVENT(sys_sched_get_priority_min,
	TP_PROTO(int policy),
	TP_ARGS(policy),
	TP_STRUCT__entry(__field(int, policy)),
	TP_fast_assign(tp_assign(policy, policy)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_mlockall
SC_TRACE_EVENT(sys_mlockall,
	TP_PROTO(int flags),
	TP_ARGS(flags),
	TP_STRUCT__entry(__field(int, flags)),
	TP_fast_assign(tp_assign(flags, flags)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_io_destroy
SC_TRACE_EVENT(sys_io_destroy,
	TP_PROTO(aio_context_t ctx),
	TP_ARGS(ctx),
	TP_STRUCT__entry(__field(aio_context_t, ctx)),
	TP_fast_assign(tp_assign(ctx, ctx)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_exit_group
SC_TRACE_EVENT(sys_exit_group,
	TP_PROTO(int error_code),
	TP_ARGS(error_code),
	TP_STRUCT__entry(__field(int, error_code)),
	TP_fast_assign(tp_assign(error_code, error_code)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_epoll_create
SC_TRACE_EVENT(sys_epoll_create,
	TP_PROTO(int size),
	TP_ARGS(size),
	TP_STRUCT__entry(__field(int, size)),
	TP_fast_assign(tp_assign(size, size)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_timer_getoverrun
SC_TRACE_EVENT(sys_timer_getoverrun,
	TP_PROTO(timer_t timer_id),
	TP_ARGS(timer_id),
	TP_STRUCT__entry(__field(timer_t, timer_id)),
	TP_fast_assign(tp_assign(timer_id, timer_id)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_timer_delete
SC_TRACE_EVENT(sys_timer_delete,
	TP_PROTO(timer_t timer_id),
	TP_ARGS(timer_id),
	TP_STRUCT__entry(__field(timer_t, timer_id)),
	TP_fast_assign(tp_assign(timer_id, timer_id)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_set_thread_area
SC_TRACE_EVENT(sys_set_thread_area,
	TP_PROTO(unsigned long addr),
	TP_ARGS(addr),
	TP_STRUCT__entry(__field_hex(unsigned long, addr)),
	TP_fast_assign(tp_assign(addr, addr)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_unshare
SC_TRACE_EVENT(sys_unshare,
	TP_PROTO(unsigned long unshare_flags),
	TP_ARGS(unshare_flags),
	TP_STRUCT__entry(__field(unsigned long, unshare_flags)),
	TP_fast_assign(tp_assign(unshare_flags, unshare_flags)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_eventfd
SC_TRACE_EVENT(sys_eventfd,
	TP_PROTO(unsigned int count),
	TP_ARGS(count),
	TP_STRUCT__entry(__field(unsigned int, count)),
	TP_fast_assign(tp_assign(count, count)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_epoll_create1
SC_TRACE_EVENT(sys_epoll_create1,
	TP_PROTO(int flags),
	TP_ARGS(flags),
	TP_STRUCT__entry(__field(int, flags)),
	TP_fast_assign(tp_assign(flags, flags)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_inotify_init1
SC_TRACE_EVENT(sys_inotify_init1,
	TP_PROTO(int flags),
	TP_ARGS(flags),
	TP_STRUCT__entry(__field(int, flags)),
	TP_fast_assign(tp_assign(flags, flags)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_syncfs
SC_TRACE_EVENT(sys_syncfs,
	TP_PROTO(int fd),
	TP_ARGS(fd),
	TP_STRUCT__entry(__field(int, fd)),
	TP_fast_assign(tp_assign(fd, fd)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_bdflush
SC_TRACE_EVENT(sys_bdflush,
	TP_PROTO(int func, long data),
	TP_ARGS(func, data),
	TP_STRUCT__entry(__field(int, func) __field(long, data)),
	TP_fast_assign(tp_assign(func, func) tp_assign(data, data)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_munmap
SC_TRACE_EVENT(sys_munmap,
	TP_PROTO(unsigned long addr, size_t len),
	TP_ARGS(addr, len),
	TP_STRUCT__entry(__field_hex(unsigned long, addr) __field(size_t, len)),
	TP_fast_assign(tp_assign(addr, addr) tp_assign(len, len)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_dup2
SC_TRACE_EVENT(sys_dup2,
	TP_PROTO(unsigned int oldfd, unsigned int newfd),
	TP_ARGS(oldfd, newfd),
	TP_STRUCT__entry(__field(unsigned int, oldfd) __field(unsigned int, newfd)),
	TP_fast_assign(tp_assign(oldfd, oldfd) tp_assign(newfd, newfd)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_shutdown
SC_TRACE_EVENT(sys_shutdown,
	TP_PROTO(int fd, int how),
	TP_ARGS(fd, how),
	TP_STRUCT__entry(__field(int, fd) __field(int, how)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(how, how)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_listen
SC_TRACE_EVENT(sys_listen,
	TP_PROTO(int fd, int backlog),
	TP_ARGS(fd, backlog),
	TP_STRUCT__entry(__field(int, fd) __field(int, backlog)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(backlog, backlog)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_kill
SC_TRACE_EVENT(sys_kill,
	TP_PROTO(pid_t pid, int sig),
	TP_ARGS(pid, sig),
	TP_STRUCT__entry(__field(pid_t, pid) __field(int, sig)),
	TP_fast_assign(tp_assign(pid, pid) tp_assign(sig, sig)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_msgget
SC_TRACE_EVENT(sys_msgget,
	TP_PROTO(key_t key, int msgflg),
	TP_ARGS(key, msgflg),
	TP_STRUCT__entry(__field(key_t, key) __field(int, msgflg)),
	TP_fast_assign(tp_assign(key, key) tp_assign(msgflg, msgflg)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_flock
SC_TRACE_EVENT(sys_flock,
	TP_PROTO(unsigned int fd, unsigned int cmd),
	TP_ARGS(fd, cmd),
	TP_STRUCT__entry(__field(unsigned int, fd) __field(unsigned int, cmd)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(cmd, cmd)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_ftruncate
SC_TRACE_EVENT(sys_ftruncate,
	TP_PROTO(unsigned int fd, unsigned long length),
	TP_ARGS(fd, length),
	TP_STRUCT__entry(__field(unsigned int, fd) __field(unsigned long, length)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(length, length)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_fchmod
SC_TRACE_EVENT(sys_fchmod,
	TP_PROTO(unsigned int fd, umode_t mode),
	TP_ARGS(fd, mode),
	TP_STRUCT__entry(__field(unsigned int, fd) __field(umode_t, mode)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(mode, mode)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_setpgid
SC_TRACE_EVENT(sys_setpgid,
	TP_PROTO(pid_t pid, pid_t pgid),
	TP_ARGS(pid, pgid),
	TP_STRUCT__entry(__field(pid_t, pid) __field(pid_t, pgid)),
	TP_fast_assign(tp_assign(pid, pid) tp_assign(pgid, pgid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_setreuid
SC_TRACE_EVENT(sys_setreuid,
	TP_PROTO(uid_t ruid, uid_t euid),
	TP_ARGS(ruid, euid),
	TP_STRUCT__entry(__field(uid_t, ruid) __field(uid_t, euid)),
	TP_fast_assign(tp_assign(ruid, ruid) tp_assign(euid, euid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_setregid
SC_TRACE_EVENT(sys_setregid,
	TP_PROTO(gid_t rgid, gid_t egid),
	TP_ARGS(rgid, egid),
	TP_STRUCT__entry(__field(gid_t, rgid) __field(gid_t, egid)),
	TP_fast_assign(tp_assign(rgid, rgid) tp_assign(egid, egid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_getpriority
SC_TRACE_EVENT(sys_getpriority,
	TP_PROTO(int which, int who),
	TP_ARGS(which, who),
	TP_STRUCT__entry(__field(int, which) __field(int, who)),
	TP_fast_assign(tp_assign(which, which) tp_assign(who, who)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_mlock
SC_TRACE_EVENT(sys_mlock,
	TP_PROTO(unsigned long start, size_t len),
	TP_ARGS(start, len),
	TP_STRUCT__entry(__field(unsigned long, start) __field(size_t, len)),
	TP_fast_assign(tp_assign(start, start) tp_assign(len, len)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_munlock
SC_TRACE_EVENT(sys_munlock,
	TP_PROTO(unsigned long start, size_t len),
	TP_ARGS(start, len),
	TP_STRUCT__entry(__field(unsigned long, start) __field(size_t, len)),
	TP_fast_assign(tp_assign(start, start) tp_assign(len, len)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_tkill
SC_TRACE_EVENT(sys_tkill,
	TP_PROTO(pid_t pid, int sig),
	TP_ARGS(pid, sig),
	TP_STRUCT__entry(__field(pid_t, pid) __field(int, sig)),
	TP_fast_assign(tp_assign(pid, pid) tp_assign(sig, sig)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_inotify_rm_watch
SC_TRACE_EVENT(sys_inotify_rm_watch,
	TP_PROTO(int fd, __s32 wd),
	TP_ARGS(fd, wd),
	TP_STRUCT__entry(__field(int, fd) __field(__s32, wd)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(wd, wd)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_ioprio_get
SC_TRACE_EVENT(sys_ioprio_get,
	TP_PROTO(int which, int who),
	TP_ARGS(which, who),
	TP_STRUCT__entry(__field(int, which) __field(int, who)),
	TP_fast_assign(tp_assign(which, which) tp_assign(who, who)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_timerfd_create
SC_TRACE_EVENT(sys_timerfd_create,
	TP_PROTO(int clockid, int flags),
	TP_ARGS(clockid, flags),
	TP_STRUCT__entry(__field(int, clockid) __field(int, flags)),
	TP_fast_assign(tp_assign(clockid, clockid) tp_assign(flags, flags)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_eventfd2
SC_TRACE_EVENT(sys_eventfd2,
	TP_PROTO(unsigned int count, int flags),
	TP_ARGS(count, flags),
	TP_STRUCT__entry(__field(unsigned int, count) __field(int, flags)),
	TP_fast_assign(tp_assign(count, count) tp_assign(flags, flags)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_setns
SC_TRACE_EVENT(sys_setns,
	TP_PROTO(int fd, int nstype),
	TP_ARGS(fd, nstype),
	TP_STRUCT__entry(__field(int, fd) __field(int, nstype)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(nstype, nstype)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_ioctl
SC_TRACE_EVENT(sys_ioctl,
	TP_PROTO(unsigned int fd, unsigned int cmd, unsigned long arg),
	TP_ARGS(fd, cmd, arg),
	TP_STRUCT__entry(__field(unsigned int, fd) __field(unsigned int, cmd) __field(unsigned long, arg)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(cmd, cmd) tp_assign(arg, arg)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_fcntl
SC_TRACE_EVENT(sys_fcntl,
	TP_PROTO(unsigned int fd, unsigned int cmd, unsigned long arg),
	TP_ARGS(fd, cmd, arg),
	TP_STRUCT__entry(__field(unsigned int, fd) __field(unsigned int, cmd) __field(unsigned long, arg)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(cmd, cmd) tp_assign(arg, arg)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_lseek
SC_TRACE_EVENT(sys_lseek,
	TP_PROTO(unsigned int fd, off_t offset, unsigned int origin),
	TP_ARGS(fd, offset, origin),
	TP_STRUCT__entry(__field(unsigned int, fd) __field(off_t, offset) __field(unsigned int, origin)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(offset, offset) tp_assign(origin, origin)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_mprotect
SC_TRACE_EVENT(sys_mprotect,
	TP_PROTO(unsigned long start, size_t len, unsigned long prot),
	TP_ARGS(start, len, prot),
	TP_STRUCT__entry(__field(unsigned long, start) __field(size_t, len) __field(unsigned long, prot)),
	TP_fast_assign(tp_assign(start, start) tp_assign(len, len) tp_assign(prot, prot)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_msync
SC_TRACE_EVENT(sys_msync,
	TP_PROTO(unsigned long start, size_t len, int flags),
	TP_ARGS(start, len, flags),
	TP_STRUCT__entry(__field(unsigned long, start) __field(size_t, len) __field(int, flags)),
	TP_fast_assign(tp_assign(start, start) tp_assign(len, len) tp_assign(flags, flags)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_madvise
SC_TRACE_EVENT(sys_madvise,
	TP_PROTO(unsigned long start, size_t len_in, int behavior),
	TP_ARGS(start, len_in, behavior),
	TP_STRUCT__entry(__field(unsigned long, start) __field(size_t, len_in) __field(int, behavior)),
	TP_fast_assign(tp_assign(start, start) tp_assign(len_in, len_in) tp_assign(behavior, behavior)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_shmget
SC_TRACE_EVENT(sys_shmget,
	TP_PROTO(key_t key, size_t size, int shmflg),
	TP_ARGS(key, size, shmflg),
	TP_STRUCT__entry(__field(key_t, key) __field(size_t, size) __field(int, shmflg)),
	TP_fast_assign(tp_assign(key, key) tp_assign(size, size) tp_assign(shmflg, shmflg)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_socket
SC_TRACE_EVENT(sys_socket,
	TP_PROTO(int family, int type, int protocol),
	TP_ARGS(family, type, protocol),
	TP_STRUCT__entry(__field(int, family) __field(int, type) __field(int, protocol)),
	TP_fast_assign(tp_assign(family, family) tp_assign(type, type) tp_assign(protocol, protocol)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_semget
SC_TRACE_EVENT(sys_semget,
	TP_PROTO(key_t key, int nsems, int semflg),
	TP_ARGS(key, nsems, semflg),
	TP_STRUCT__entry(__field(key_t, key) __field(int, nsems) __field(int, semflg)),
	TP_fast_assign(tp_assign(key, key) tp_assign(nsems, nsems) tp_assign(semflg, semflg)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_fchown
SC_TRACE_EVENT(sys_fchown,
	TP_PROTO(unsigned int fd, uid_t user, gid_t group),
	TP_ARGS(fd, user, group),
	TP_STRUCT__entry(__field(unsigned int, fd) __field(uid_t, user) __field(gid_t, group)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(user, user) tp_assign(group, group)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_setresuid
SC_TRACE_EVENT(sys_setresuid,
	TP_PROTO(uid_t ruid, uid_t euid, uid_t suid),
	TP_ARGS(ruid, euid, suid),
	TP_STRUCT__entry(__field(uid_t, ruid) __field(uid_t, euid) __field(uid_t, suid)),
	TP_fast_assign(tp_assign(ruid, ruid) tp_assign(euid, euid) tp_assign(suid, suid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_setresgid
SC_TRACE_EVENT(sys_setresgid,
	TP_PROTO(gid_t rgid, gid_t egid, gid_t sgid),
	TP_ARGS(rgid, egid, sgid),
	TP_STRUCT__entry(__field(gid_t, rgid) __field(gid_t, egid) __field(gid_t, sgid)),
	TP_fast_assign(tp_assign(rgid, rgid) tp_assign(egid, egid) tp_assign(sgid, sgid)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_sysfs
SC_TRACE_EVENT(sys_sysfs,
	TP_PROTO(int option, unsigned long arg1, unsigned long arg2),
	TP_ARGS(option, arg1, arg2),
	TP_STRUCT__entry(__field(int, option) __field(unsigned long, arg1) __field(unsigned long, arg2)),
	TP_fast_assign(tp_assign(option, option) tp_assign(arg1, arg1) tp_assign(arg2, arg2)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_setpriority
SC_TRACE_EVENT(sys_setpriority,
	TP_PROTO(int which, int who, int niceval),
	TP_ARGS(which, who, niceval),
	TP_STRUCT__entry(__field(int, which) __field(int, who) __field(int, niceval)),
	TP_fast_assign(tp_assign(which, which) tp_assign(who, who) tp_assign(niceval, niceval)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_cacheflush
SC_TRACE_EVENT(sys_cacheflush,
	TP_PROTO(unsigned long addr, unsigned long bytes, unsigned int cache),
	TP_ARGS(addr, bytes, cache),
	TP_STRUCT__entry(__field_hex(unsigned long, addr) __field(unsigned long, bytes) __field(unsigned int, cache)),
	TP_fast_assign(tp_assign(addr, addr) tp_assign(bytes, bytes) tp_assign(cache, cache)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_tgkill
SC_TRACE_EVENT(sys_tgkill,
	TP_PROTO(pid_t tgid, pid_t pid, int sig),
	TP_ARGS(tgid, pid, sig),
	TP_STRUCT__entry(__field(pid_t, tgid) __field(pid_t, pid) __field(int, sig)),
	TP_fast_assign(tp_assign(tgid, tgid) tp_assign(pid, pid) tp_assign(sig, sig)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_ioprio_set
SC_TRACE_EVENT(sys_ioprio_set,
	TP_PROTO(int which, int who, int ioprio),
	TP_ARGS(which, who, ioprio),
	TP_STRUCT__entry(__field(int, which) __field(int, who) __field(int, ioprio)),
	TP_fast_assign(tp_assign(which, which) tp_assign(who, who) tp_assign(ioprio, ioprio)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_dup3
SC_TRACE_EVENT(sys_dup3,
	TP_PROTO(unsigned int oldfd, unsigned int newfd, int flags),
	TP_ARGS(oldfd, newfd, flags),
	TP_STRUCT__entry(__field(unsigned int, oldfd) __field(unsigned int, newfd) __field(int, flags)),
	TP_fast_assign(tp_assign(oldfd, oldfd) tp_assign(newfd, newfd) tp_assign(flags, flags)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_32_ftruncate64
SC_TRACE_EVENT(sys_32_ftruncate64,
	TP_PROTO(unsigned long fd, unsigned long __dummy, unsigned long a2, unsigned long a3),
	TP_ARGS(fd, __dummy, a2, a3),
	TP_STRUCT__entry(__field(unsigned long, fd) __field(unsigned long, __dummy) __field(unsigned long, a2) __field(unsigned long, a3)),
	TP_fast_assign(tp_assign(fd, fd) tp_assign(__dummy, __dummy) tp_assign(a2, a2) tp_assign(a3, a3)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_ptrace
SC_TRACE_EVENT(sys_ptrace,
	TP_PROTO(long request, long pid, unsigned long addr, unsigned long data),
	TP_ARGS(request, pid, addr, data),
	TP_STRUCT__entry(__field(long, request) __field(long, pid) __field_hex(unsigned long, addr) __field(unsigned long, data)),
	TP_fast_assign(tp_assign(request, request) tp_assign(pid, pid) tp_assign(addr, addr) tp_assign(data, data)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_n32_semctl
SC_TRACE_EVENT(sys_n32_semctl,
	TP_PROTO(int semid, int semnum, int cmd, u32 arg),
	TP_ARGS(semid, semnum, cmd, arg),
	TP_STRUCT__entry(__field(int, semid) __field(int, semnum) __field(int, cmd) __field(u32, arg)),
	TP_fast_assign(tp_assign(semid, semid) tp_assign(semnum, semnum) tp_assign(cmd, cmd) tp_assign(arg, arg)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_n32_msgsnd
SC_TRACE_EVENT(sys_n32_msgsnd,
	TP_PROTO(int msqid, u32 msgp, unsigned int msgsz, int msgflg),
	TP_ARGS(msqid, msgp, msgsz, msgflg),
	TP_STRUCT__entry(__field(int, msqid) __field(u32, msgp) __field(unsigned int, msgsz) __field(int, msgflg)),
	TP_fast_assign(tp_assign(msqid, msqid) tp_assign(msgp, msgp) tp_assign(msgsz, msgsz) tp_assign(msgflg, msgflg)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_tee
SC_TRACE_EVENT(sys_tee,
	TP_PROTO(int fdin, int fdout, size_t len, unsigned int flags),
	TP_ARGS(fdin, fdout, len, flags),
	TP_STRUCT__entry(__field(int, fdin) __field(int, fdout) __field(size_t, len) __field(unsigned int, flags)),
	TP_fast_assign(tp_assign(fdin, fdin) tp_assign(fdout, fdout) tp_assign(len, len) tp_assign(flags, flags)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_mremap
SC_TRACE_EVENT(sys_mremap,
	TP_PROTO(unsigned long addr, unsigned long old_len, unsigned long new_len, unsigned long flags, unsigned long new_addr),
	TP_ARGS(addr, old_len, new_len, flags, new_addr),
	TP_STRUCT__entry(__field_hex(unsigned long, addr) __field(unsigned long, old_len) __field(unsigned long, new_len) __field(unsigned long, flags) __field_hex(unsigned long, new_addr)),
	TP_fast_assign(tp_assign(addr, addr) tp_assign(old_len, old_len) tp_assign(new_len, new_len) tp_assign(flags, flags) tp_assign(new_addr, new_addr)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_n32_msgrcv
SC_TRACE_EVENT(sys_n32_msgrcv,
	TP_PROTO(int msqid, u32 msgp, size_t msgsz, int msgtyp, int msgflg),
	TP_ARGS(msqid, msgp, msgsz, msgtyp, msgflg),
	TP_STRUCT__entry(__field(int, msqid) __field(u32, msgp) __field(size_t, msgsz) __field(int, msgtyp) __field(int, msgflg)),
	TP_fast_assign(tp_assign(msqid, msqid) tp_assign(msgp, msgp) tp_assign(msgsz, msgsz) tp_assign(msgtyp, msgtyp) tp_assign(msgflg, msgflg)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_prctl
SC_TRACE_EVENT(sys_prctl,
	TP_PROTO(int option, unsigned long arg2, unsigned long arg3, unsigned long arg4, unsigned long arg5),
	TP_ARGS(option, arg2, arg3, arg4, arg5),
	TP_STRUCT__entry(__field(int, option) __field(unsigned long, arg2) __field(unsigned long, arg3) __field(unsigned long, arg4) __field(unsigned long, arg5)),
	TP_fast_assign(tp_assign(option, option) tp_assign(arg2, arg2) tp_assign(arg3, arg3) tp_assign(arg4, arg4) tp_assign(arg5, arg5)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_remap_file_pages
SC_TRACE_EVENT(sys_remap_file_pages,
	TP_PROTO(unsigned long start, unsigned long size, unsigned long prot, unsigned long pgoff, unsigned long flags),
	TP_ARGS(start, size, prot, pgoff, flags),
	TP_STRUCT__entry(__field(unsigned long, start) __field(unsigned long, size) __field(unsigned long, prot) __field(unsigned long, pgoff) __field(unsigned long, flags)),
	TP_fast_assign(tp_assign(start, start) tp_assign(size, size) tp_assign(prot, prot) tp_assign(pgoff, pgoff) tp_assign(flags, flags)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_32_ipc
SC_TRACE_EVENT(sys_32_ipc,
	TP_PROTO(u32 call, long first, long second, long third, unsigned long ptr, unsigned long fifth),
	TP_ARGS(call, first, second, third, ptr, fifth),
	TP_STRUCT__entry(__field(u32, call) __field(long, first) __field(long, second) __field(long, third) __field_hex(unsigned long, ptr) __field(unsigned long, fifth)),
	TP_fast_assign(tp_assign(call, call) tp_assign(first, first) tp_assign(second, second) tp_assign(third, third) tp_assign(ptr, ptr) tp_assign(fifth, fifth)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_mips_mmap2
SC_TRACE_EVENT(sys_mips_mmap2,
	TP_PROTO(unsigned long addr, unsigned long len, unsigned long prot, unsigned long flags, unsigned long fd, unsigned long pgoff),
	TP_ARGS(addr, len, prot, flags, fd, pgoff),
	TP_STRUCT__entry(__field_hex(unsigned long, addr) __field(unsigned long, len) __field(unsigned long, prot) __field(unsigned long, flags) __field(unsigned long, fd) __field(unsigned long, pgoff)),
	TP_fast_assign(tp_assign(addr, addr) tp_assign(len, len) tp_assign(prot, prot) tp_assign(flags, flags) tp_assign(fd, fd) tp_assign(pgoff, pgoff)),
	TP_printk()
)
#endif
#ifndef OVERRIDE_64_sys_mips_mmap
SC_TRACE_EVENT(sys_mips_mmap,
	TP_PROTO(unsigned long addr, unsigned long len, unsigned long prot, unsigned long flags, unsigned long fd, off_t offset),
	TP_ARGS(addr, len, prot, flags, fd, offset),
	TP_STRUCT__entry(__field_hex(unsigned long, addr) __field(unsigned long, len) __field(unsigned long, prot) __field(unsigned long, flags) __field(unsigned long, fd) __field(off_t, offset)),
	TP_fast_assign(tp_assign(addr, addr) tp_assign(len, len) tp_assign(prot, prot) tp_assign(flags, flags) tp_assign(fd, fd) tp_assign(offset, offset)),
	TP_printk()
)
#endif

#endif /*  _TRACE_SYSCALLS_INTEGERS_H */

/* This part must be outside protection */
#include "../../../probes/define_trace.h"

#else /* CREATE_SYSCALL_TABLE */

#include "mips-64-syscalls-3.5.0_integers_override.h"
#include "syscalls_integers_override.h"

#ifndef OVERRIDE_TABLE_64_sys_sgetmask
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_sgetmask, 4068, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_sched_yield
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_sched_yield, 6023, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_pause
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_pause, 6033, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_getpid
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_getpid, 6038, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_getuid
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_getuid, 6100, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_getgid
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_getgid, 6102, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_geteuid
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_geteuid, 6105, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_getegid
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_getegid, 6106, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_getppid
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_getppid, 6108, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_getpgrp
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_getpgrp, 6109, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setsid
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_setsid, 6110, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_munlockall
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_munlockall, 6149, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_vhangup
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_vhangup, 6150, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_sync
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_sync, 6157, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_gettid
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_gettid, 6178, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_restart_syscall
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_restart_syscall, 6214, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_inotify_init
TRACE_SYSCALL_TABLE(syscalls_noargs, sys_inotify_init, 6247, 0)
#endif
#ifndef OVERRIDE_TABLE_64_sys_nice
TRACE_SYSCALL_TABLE(sys_nice, sys_nice, 4034, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_ssetmask
TRACE_SYSCALL_TABLE(sys_ssetmask, sys_ssetmask, 4069, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_32_ipc
TRACE_SYSCALL_TABLE(sys_32_ipc, sys_32_ipc, 4117, 6)
#endif
#ifndef OVERRIDE_TABLE_64_sys_bdflush
TRACE_SYSCALL_TABLE(sys_bdflush, sys_bdflush, 4134, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_mips_mmap2
TRACE_SYSCALL_TABLE(sys_mips_mmap2, sys_mips_mmap2, 4210, 6)
#endif
#ifndef OVERRIDE_TABLE_64_sys_32_ftruncate64
TRACE_SYSCALL_TABLE(sys_32_ftruncate64, sys_32_ftruncate64, 4212, 4)
#endif
#ifndef OVERRIDE_TABLE_64_sys_ioctl
TRACE_SYSCALL_TABLE(sys_ioctl, sys_ioctl, 5015, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_fcntl
TRACE_SYSCALL_TABLE(sys_fcntl, sys_fcntl, 5070, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_ptrace
TRACE_SYSCALL_TABLE(sys_ptrace, sys_ptrace, 5099, 4)
#endif
#ifndef OVERRIDE_TABLE_64_sys_personality
TRACE_SYSCALL_TABLE(sys_personality, sys_personality, 5132, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_close
TRACE_SYSCALL_TABLE(sys_close, sys_close, 6003, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_lseek
TRACE_SYSCALL_TABLE(sys_lseek, sys_lseek, 6008, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_mips_mmap
TRACE_SYSCALL_TABLE(sys_mips_mmap, sys_mips_mmap, 6009, 6)
#endif
#ifndef OVERRIDE_TABLE_64_sys_mprotect
TRACE_SYSCALL_TABLE(sys_mprotect, sys_mprotect, 6010, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_munmap
TRACE_SYSCALL_TABLE(sys_munmap, sys_munmap, 6011, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_brk
TRACE_SYSCALL_TABLE(sys_brk, sys_brk, 6012, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_mremap
TRACE_SYSCALL_TABLE(sys_mremap, sys_mremap, 6024, 5)
#endif
#ifndef OVERRIDE_TABLE_64_sys_msync
TRACE_SYSCALL_TABLE(sys_msync, sys_msync, 6025, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_madvise
TRACE_SYSCALL_TABLE(sys_madvise, sys_madvise, 6027, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_shmget
TRACE_SYSCALL_TABLE(sys_shmget, sys_shmget, 6028, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_dup
TRACE_SYSCALL_TABLE(sys_dup, sys_dup, 6031, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_dup2
TRACE_SYSCALL_TABLE(sys_dup2, sys_dup2, 6032, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_alarm
TRACE_SYSCALL_TABLE(sys_alarm, sys_alarm, 6037, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_socket
TRACE_SYSCALL_TABLE(sys_socket, sys_socket, 6040, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_shutdown
TRACE_SYSCALL_TABLE(sys_shutdown, sys_shutdown, 6047, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_listen
TRACE_SYSCALL_TABLE(sys_listen, sys_listen, 6049, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_exit
TRACE_SYSCALL_TABLE(sys_exit, sys_exit, 6058, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_kill
TRACE_SYSCALL_TABLE(sys_kill, sys_kill, 6060, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_semget
TRACE_SYSCALL_TABLE(sys_semget, sys_semget, 6062, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_n32_semctl
TRACE_SYSCALL_TABLE(sys_n32_semctl, sys_n32_semctl, 6064, 4)
#endif
#ifndef OVERRIDE_TABLE_64_sys_msgget
TRACE_SYSCALL_TABLE(sys_msgget, sys_msgget, 6066, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_n32_msgsnd
TRACE_SYSCALL_TABLE(sys_n32_msgsnd, sys_n32_msgsnd, 6067, 4)
#endif
#ifndef OVERRIDE_TABLE_64_sys_n32_msgrcv
TRACE_SYSCALL_TABLE(sys_n32_msgrcv, sys_n32_msgrcv, 6068, 5)
#endif
#ifndef OVERRIDE_TABLE_64_sys_flock
TRACE_SYSCALL_TABLE(sys_flock, sys_flock, 6071, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_fsync
TRACE_SYSCALL_TABLE(sys_fsync, sys_fsync, 6072, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_fdatasync
TRACE_SYSCALL_TABLE(sys_fdatasync, sys_fdatasync, 6073, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_ftruncate
TRACE_SYSCALL_TABLE(sys_ftruncate, sys_ftruncate, 6075, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_fchdir
TRACE_SYSCALL_TABLE(sys_fchdir, sys_fchdir, 6079, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_fchmod
TRACE_SYSCALL_TABLE(sys_fchmod, sys_fchmod, 6089, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_fchown
TRACE_SYSCALL_TABLE(sys_fchown, sys_fchown, 6091, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_umask
TRACE_SYSCALL_TABLE(sys_umask, sys_umask, 6093, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setuid
TRACE_SYSCALL_TABLE(sys_setuid, sys_setuid, 6103, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setgid
TRACE_SYSCALL_TABLE(sys_setgid, sys_setgid, 6104, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setpgid
TRACE_SYSCALL_TABLE(sys_setpgid, sys_setpgid, 6107, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setreuid
TRACE_SYSCALL_TABLE(sys_setreuid, sys_setreuid, 6111, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setregid
TRACE_SYSCALL_TABLE(sys_setregid, sys_setregid, 6112, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setresuid
TRACE_SYSCALL_TABLE(sys_setresuid, sys_setresuid, 6115, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setresgid
TRACE_SYSCALL_TABLE(sys_setresgid, sys_setresgid, 6117, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_getpgid
TRACE_SYSCALL_TABLE(sys_getpgid, sys_getpgid, 6119, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setfsuid
TRACE_SYSCALL_TABLE(sys_setfsuid, sys_setfsuid, 6120, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setfsgid
TRACE_SYSCALL_TABLE(sys_setfsgid, sys_setfsgid, 6121, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_getsid
TRACE_SYSCALL_TABLE(sys_getsid, sys_getsid, 6122, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_32_personality
TRACE_SYSCALL_TABLE(sys_32_personality, sys_32_personality, 6132, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_sysfs
TRACE_SYSCALL_TABLE(sys_sysfs, sys_sysfs, 6136, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_getpriority
TRACE_SYSCALL_TABLE(sys_getpriority, sys_getpriority, 6137, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setpriority
TRACE_SYSCALL_TABLE(sys_setpriority, sys_setpriority, 6138, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_sched_getscheduler
TRACE_SYSCALL_TABLE(sys_sched_getscheduler, sys_sched_getscheduler, 6142, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_sched_get_priority_max
TRACE_SYSCALL_TABLE(sys_sched_get_priority_max, sys_sched_get_priority_max, 6143, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_sched_get_priority_min
TRACE_SYSCALL_TABLE(sys_sched_get_priority_min, sys_sched_get_priority_min, 6144, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_mlock
TRACE_SYSCALL_TABLE(sys_mlock, sys_mlock, 6146, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_munlock
TRACE_SYSCALL_TABLE(sys_munlock, sys_munlock, 6147, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_mlockall
TRACE_SYSCALL_TABLE(sys_mlockall, sys_mlockall, 6148, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_prctl
TRACE_SYSCALL_TABLE(sys_prctl, sys_prctl, 6153, 5)
#endif
#ifndef OVERRIDE_TABLE_64_sys_tkill
TRACE_SYSCALL_TABLE(sys_tkill, sys_tkill, 6192, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_cacheflush
TRACE_SYSCALL_TABLE(sys_cacheflush, sys_cacheflush, 6197, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_io_destroy
TRACE_SYSCALL_TABLE(sys_io_destroy, sys_io_destroy, 6201, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_exit_group
TRACE_SYSCALL_TABLE(sys_exit_group, sys_exit_group, 6205, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_epoll_create
TRACE_SYSCALL_TABLE(sys_epoll_create, sys_epoll_create, 6207, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_remap_file_pages
TRACE_SYSCALL_TABLE(sys_remap_file_pages, sys_remap_file_pages, 6210, 5)
#endif
#ifndef OVERRIDE_TABLE_64_sys_timer_getoverrun
TRACE_SYSCALL_TABLE(sys_timer_getoverrun, sys_timer_getoverrun, 6223, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_timer_delete
TRACE_SYSCALL_TABLE(sys_timer_delete, sys_timer_delete, 6224, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_tgkill
TRACE_SYSCALL_TABLE(sys_tgkill, sys_tgkill, 6229, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_set_thread_area
TRACE_SYSCALL_TABLE(sys_set_thread_area, sys_set_thread_area, 6246, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_inotify_rm_watch
TRACE_SYSCALL_TABLE(sys_inotify_rm_watch, sys_inotify_rm_watch, 6249, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_unshare
TRACE_SYSCALL_TABLE(sys_unshare, sys_unshare, 6266, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_tee
TRACE_SYSCALL_TABLE(sys_tee, sys_tee, 6269, 4)
#endif
#ifndef OVERRIDE_TABLE_64_sys_ioprio_set
TRACE_SYSCALL_TABLE(sys_ioprio_set, sys_ioprio_set, 6277, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_ioprio_get
TRACE_SYSCALL_TABLE(sys_ioprio_get, sys_ioprio_get, 6278, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_eventfd
TRACE_SYSCALL_TABLE(sys_eventfd, sys_eventfd, 6282, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_timerfd_create
TRACE_SYSCALL_TABLE(sys_timerfd_create, sys_timerfd_create, 6284, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_eventfd2
TRACE_SYSCALL_TABLE(sys_eventfd2, sys_eventfd2, 6288, 2)
#endif
#ifndef OVERRIDE_TABLE_64_sys_epoll_create1
TRACE_SYSCALL_TABLE(sys_epoll_create1, sys_epoll_create1, 6289, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_dup3
TRACE_SYSCALL_TABLE(sys_dup3, sys_dup3, 6290, 3)
#endif
#ifndef OVERRIDE_TABLE_64_sys_inotify_init1
TRACE_SYSCALL_TABLE(sys_inotify_init1, sys_inotify_init1, 6292, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_syncfs
TRACE_SYSCALL_TABLE(sys_syncfs, sys_syncfs, 6306, 1)
#endif
#ifndef OVERRIDE_TABLE_64_sys_setns
TRACE_SYSCALL_TABLE(sys_setns, sys_setns, 6308, 2)
#endif

#endif /* CREATE_SYSCALL_TABLE */
