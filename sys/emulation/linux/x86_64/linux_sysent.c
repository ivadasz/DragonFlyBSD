/*
 * System call switch table.
 *
 * DO NOT EDIT-- To regenerate this file, edit syscalls.master followed
 *               by running make sysent in the same directory.
 */

#include "opt_compat.h"
#include <sys/param.h>
#include <sys/sysent.h>
#include <sys/sysproto.h>
#include <emulation/linux/linux_sysproto.h>
#include "linux.h"
#include "linux_proto.h"
#include <emulation/43bsd/stat.h>

#define AS(name) ((sizeof(struct name) - sizeof(struct sysmsg)) / sizeof(register_t))

/* The casts are bogus but will do for now. */
struct sysent linux_sysent[] = {
#define	nosys	linux_nosys
	{ 0, (sy_call_t *)sys_nosys },			/* 0 = read */
	{ 0, (sy_call_t *)sys_nosys },			/* 1 = write */
	{ AS(linux_open_args), (sy_call_t *)sys_linux_open },	/* 2 = linux_open */
	{ 0, (sy_call_t *)sys_nosys },			/* 3 = close */
	{ 0, (sy_call_t *)sys_nosys },			/* 4 = stat */
	{ 0, (sy_call_t *)sys_nosys },			/* 5 = fstat */
	{ 0, (sy_call_t *)sys_nosys },			/* 6 = lstat */
	{ 0, (sy_call_t *)sys_nosys },			/* 7 = poll */
	{ 0, (sy_call_t *)sys_nosys },			/* 8 = lseek */
	{ 0, (sy_call_t *)sys_nosys },			/* 9 = mmap */
	{ 0, (sy_call_t *)sys_nosys },			/* 10 = mprotect */
	{ 0, (sy_call_t *)sys_nosys },			/* 11 = munmap */
	{ 0, (sy_call_t *)sys_nosys },			/* 12 = brk */
	{ 0, (sy_call_t *)sys_nosys },			/* 13 = rt_sigaction */
	{ 0, (sy_call_t *)sys_nosys },			/* 14 = rt_sigprocmask */
	{ 0, (sy_call_t *)sys_nosys },			/* 15 = rt_sigreturn */
	{ 0, (sy_call_t *)sys_nosys },			/* 16 = ioctl */
	{ 0, (sy_call_t *)sys_nosys },			/* 17 = pread64 */
	{ 0, (sy_call_t *)sys_nosys },			/* 18 = pwrite64 */
	{ AS(exit_args), (sy_call_t *)sys_exit },	/* 19 = exit */
	{ 0, (sy_call_t *)sys_nosys },			/* 20 = writev */
	{ AS(linux_execve_args), (sy_call_t *)sys_linux_execve },	/* 21 = linux_execve */
	{ AS(linux_sigreturn_args), (sy_call_t *)sys_linux_sigreturn },	/* 22 = linux_sigreturn */
	{ AS(linux_rt_sigreturn_args), (sy_call_t *)sys_linux_rt_sigreturn },	/* 23 = linux_rt_sigreturn */
	{ AS(linux_signal_args), (sy_call_t *)sys_linux_signal },	/* 24 = linux_signal */
	{ AS(linux_rt_sigaction_args), (sy_call_t *)sys_linux_rt_sigaction },	/* 25 = linux_rt_sigaction */
	{ AS(linux_sigprocmask_args), (sy_call_t *)sys_linux_sigprocmask },	/* 26 = linux_sigprocmask */
	{ AS(linux_rt_sigprocmask_args), (sy_call_t *)sys_linux_rt_sigprocmask },	/* 27 = linux_rt_sigprocmask */
	{ 0, (sy_call_t *)sys_linux_sgetmask },		/* 28 = linux_sgetmask */
	{ AS(linux_ssetmask_args), (sy_call_t *)sys_linux_ssetmask },	/* 29 = linux_ssetmask */
	{ AS(linux_sigpending_args), (sy_call_t *)sys_linux_sigpending },	/* 30 = linux_sigpending */
	{ AS(linux_kill_args), (sy_call_t *)sys_linux_kill },	/* 31 = linux_kill */
	{ AS(linux_tgkill_args), (sy_call_t *)sys_linux_tgkill },	/* 32 = linux_tgkill */
	{ AS(linux_tkill_args), (sy_call_t *)sys_linux_tkill },	/* 33 = linux_tkill */
};
