/*
 * System call numbers.
 *
 * DO NOT EDIT-- To regenerate this file, edit syscalls.master followed
 *               by running make sysent in the same directory.
 */

#define	LINUX_SYS_read	0
#define	LINUX_SYS_write	1
#define	LINUX_SYS_linux_open	2
#define	LINUX_SYS_close	3
#define	LINUX_SYS_linux_stat64	4
#define	LINUX_SYS_linux_fstat64	5
#define	LINUX_SYS_linux_lstat64	6
#define	LINUX_SYS_poll	7
#define	LINUX_SYS_linux_lseek	8
#define	LINUX_SYS_linux_mmap2	9
#define	LINUX_SYS_mprotect	10
#define	LINUX_SYS_munmap	11
#define	LINUX_SYS_linux_brk	12
#define	LINUX_SYS_linux_rt_sigaction	13
#define	LINUX_SYS_linux_rt_sigprocmask	14
#define	LINUX_SYS_linux_rt_sigreturn	15
#define	LINUX_SYS_linux_ioctl	16
#define	LINUX_SYS_readv	19
#define	LINUX_SYS_writev	20
#define	LINUX_SYS_linux_access	21
#define	LINUX_SYS_linux_getpid	39
#define	LINUX_SYS_linux_clone	56
#define	LINUX_SYS_linux_execve	59
#define	LINUX_SYS_exit	60
#define	LINUX_SYS_linux_wait4	61
#define	LINUX_SYS_linux_kill	62
#define	LINUX_SYS_linux_newuname	63
#define	LINUX_SYS_linux_fcntl64	72
#define	LINUX_SYS_linux_getcwd	79
#define	LINUX_SYS_linux_chdir	80
#define	LINUX_SYS_linux_getuid	102
#define	LINUX_SYS_linux_getgid	104
#define	LINUX_SYS_geteuid	107
#define	LINUX_SYS_getegid	108
#define	LINUX_SYS_setpgid	109
#define	LINUX_SYS_linux_getppid	110
#define	LINUX_SYS_getpgrp	111
#define	LINUX_SYS_setsid	112
#define	LINUX_SYS_linux_rt_sigsuspend	130
#define	LINUX_SYS_linux_arch_prctl	158
#define	LINUX_SYS_linux_tkill	200
#define	LINUX_SYS_linux_sys_futex	202
#define	LINUX_SYS_linux_exit_group	231
#define	LINUX_SYS_linux_tgkill	234
#define	LINUX_SYS_linux_set_robust_list	273
#define	LINUX_SYS_linux_get_robust_list	274
#define	LINUX_SYS_MAXSYSCALL	312
