/*
 * Union of syscall args for messaging.
 *
 * DO NOT EDIT-- To regenerate this file, edit syscalls.master followed
 *               by running make sysent in the same directory.
 */

union sysunion {
#ifdef _KERNEL /* header only applies in kernel */
	struct	lwkt_msg lmsg;
	struct	sysmsg sysmsg;
#endif
#define	nosys	linux_nosys
	struct	linux_open_args linux_open;
	struct	linux_execve_args linux_execve;
	struct	linux_sigreturn_args linux_sigreturn;
	struct	linux_rt_sigreturn_args linux_rt_sigreturn;
	struct	linux_signal_args linux_signal;
	struct	linux_rt_sigaction_args linux_rt_sigaction;
	struct	linux_sigprocmask_args linux_sigprocmask;
	struct	linux_rt_sigprocmask_args linux_rt_sigprocmask;
	struct	linux_sgetmask_args linux_sgetmask;
	struct	linux_ssetmask_args linux_ssetmask;
	struct	linux_sigpending_args linux_sigpending;
	struct	linux_kill_args linux_kill;
	struct	linux_tgkill_args linux_tgkill;
	struct	linux_tkill_args linux_tkill;
};
