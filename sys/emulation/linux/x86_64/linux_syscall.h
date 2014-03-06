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
#define	LINUX_SYS_linux_lseek	8
#define	LINUX_SYS_linux_brk	12
#define	LINUX_SYS_linux_rt_sigaction	13
#define	LINUX_SYS_linux_rt_sigprocmask	14
#define	LINUX_SYS_linux_rt_sigreturn	15
#define	LINUX_SYS_linux_getpid	39
#define	LINUX_SYS_linux_execve	59
#define	LINUX_SYS_exit	60
#define	LINUX_SYS_linux_kill	62
#define	LINUX_SYS_linux_newuname	63
#define	LINUX_SYS_linux_chdir	80
#define	LINUX_SYS_linux_rt_sigsuspend	130
#define	LINUX_SYS_linux_tkill	200
#define	LINUX_SYS_linux_tgkill	234
#define	LINUX_SYS_MAXSYSCALL	312
