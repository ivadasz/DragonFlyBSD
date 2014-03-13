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
	struct	linux_fstat64_args linux_fstat64;
	struct	linux_lstat64_args linux_lstat64;
	struct	linux_lseek_args linux_lseek;
	struct	linux_mmap2_args linux_mmap2;
	struct	linux_brk_args linux_brk;
	struct	linux_rt_sigaction_args linux_rt_sigaction;
	struct	linux_rt_sigprocmask_args linux_rt_sigprocmask;
	struct	linux_rt_sigreturn_args linux_rt_sigreturn;
	struct	linux_ioctl_args linux_ioctl;
	struct	linux_access_args linux_access;
	struct	linux_getpid_args linux_getpid;
	struct	linux_execve_args linux_execve;
	struct	linux_kill_args linux_kill;
	struct	linux_newuname_args linux_newuname;
	struct	linux_getcwd_args linux_getcwd;
	struct	linux_chdir_args linux_chdir;
	struct	linux_getuid_args linux_getuid;
	struct	linux_getgid_args linux_getgid;
	struct	linux_getppid_args linux_getppid;
	struct	linux_rt_sigsuspend_args linux_rt_sigsuspend;
	struct	linux_arch_prctl_args linux_arch_prctl;
	struct	linux_tkill_args linux_tkill;
	struct	linux_exit_group_args linux_exit_group;
	struct	linux_tgkill_args linux_tgkill;
};
