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
	struct	linux_stat64_args linux_stat64;
	struct	linux_lseek_args linux_lseek;
	struct	linux_brk_args linux_brk;
	struct	linux_rt_sigaction_args linux_rt_sigaction;
	struct	linux_rt_sigprocmask_args linux_rt_sigprocmask;
	struct	linux_rt_sigreturn_args linux_rt_sigreturn;
	struct	linux_getpid_args linux_getpid;
	struct	linux_execve_args linux_execve;
	struct	linux_kill_args linux_kill;
	struct	linux_newuname_args linux_newuname;
	struct	linux_chdir_args linux_chdir;
	struct	linux_rt_sigsuspend_args linux_rt_sigsuspend;
	struct	linux_tkill_args linux_tkill;
	struct	linux_tgkill_args linux_tgkill;
};
