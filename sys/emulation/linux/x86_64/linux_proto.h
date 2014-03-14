/*
 * System call prototypes.
 *
 * DO NOT EDIT-- To regenerate this file, edit syscalls.master followed
 *               by running make sysent in the same directory.
 */

#ifndef _LINUX_SYSPROTO_H_
#define	_LINUX_SYSPROTO_H_

#include <sys/select.h>

#include <sys/signal.h>

#include <sys/acl.h>

#include <sys/msgport.h>

#include <sys/sysmsg.h>

#include <sys/syslink.h>

#define	PAD_(t)	(sizeof(register_t) <= sizeof(t) ? \
		0 : sizeof(register_t) - sizeof(t))

#define	nosys	linux_nosys
struct	linux_open_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	char *	path;	char path_[PAD_(char *)];
	l_int	flags;	char flags_[PAD_(l_int)];
	l_int	mode;	char mode_[PAD_(l_int)];
};
struct	linux_stat64_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	char *	filename;	char filename_[PAD_(char *)];
	struct l_stat64 *	statbuf;	char statbuf_[PAD_(struct l_stat64 *)];
	l_long	flags;	char flags_[PAD_(l_long)];
};
struct	linux_fstat64_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_ulong	fd;	char fd_[PAD_(l_ulong)];
	struct l_stat64 *	statbuf;	char statbuf_[PAD_(struct l_stat64 *)];
	l_long	flags;	char flags_[PAD_(l_long)];
};
struct	linux_lstat64_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	char *	filename;	char filename_[PAD_(char *)];
	struct l_stat64 *	statbuf;	char statbuf_[PAD_(struct l_stat64 *)];
	l_long	flags;	char flags_[PAD_(l_long)];
};
struct	linux_lseek_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_uint	fdes;	char fdes_[PAD_(l_uint)];
	l_off_t	off;	char off_[PAD_(l_off_t)];
	l_int	whence;	char whence_[PAD_(l_int)];
};
struct	linux_mmap2_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_ulong	addr;	char addr_[PAD_(l_ulong)];
	l_ulong	len;	char len_[PAD_(l_ulong)];
	l_int	prot;	char prot_[PAD_(l_int)];
	l_int	flags;	char flags_[PAD_(l_int)];
	l_int	fd;	char fd_[PAD_(l_int)];
	l_ulong	pgoff;	char pgoff_[PAD_(l_ulong)];
};
struct	linux_brk_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_ulong	dsend;	char dsend_[PAD_(l_ulong)];
};
struct	linux_rt_sigaction_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_int	sig;	char sig_[PAD_(l_int)];
	l_sigaction_t *	act;	char act_[PAD_(l_sigaction_t *)];
	l_sigaction_t *	oact;	char oact_[PAD_(l_sigaction_t *)];
	l_size_t	sigsetsize;	char sigsetsize_[PAD_(l_size_t)];
};
struct	linux_rt_sigprocmask_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_int	how;	char how_[PAD_(l_int)];
	l_sigset_t *	mask;	char mask_[PAD_(l_sigset_t *)];
	l_sigset_t *	omask;	char omask_[PAD_(l_sigset_t *)];
	l_size_t	sigsetsize;	char sigsetsize_[PAD_(l_size_t)];
};
struct	linux_rt_sigreturn_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	struct l_ucontext *	ucp;	char ucp_[PAD_(struct l_ucontext *)];
};
struct	linux_ioctl_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_int	fd;	char fd_[PAD_(l_int)];
	l_ulong	cmd;	char cmd_[PAD_(l_ulong)];
	l_ulong	arg;	char arg_[PAD_(l_ulong)];
};
struct	linux_access_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	char *	path;	char path_[PAD_(char *)];
	l_int	flags;	char flags_[PAD_(l_int)];
};
struct	linux_getpid_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	register_t dummy;
};
struct	linux_clone_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_int	flags;	char flags_[PAD_(l_int)];
	void *	stack;	char stack_[PAD_(void *)];
	void *	parent_tidptr;	char parent_tidptr_[PAD_(void *)];
	int	dummy;	char dummy_[PAD_(int)];
	void *	child_tidptr;	char child_tidptr_[PAD_(void *)];
};
struct	linux_execve_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	char *	path;	char path_[PAD_(char *)];
	char **	argp;	char argp_[PAD_(char **)];
	char **	envp;	char envp_[PAD_(char **)];
};
struct	linux_wait4_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_pid_t	pid;	char pid_[PAD_(l_pid_t)];
	l_uint *	status;	char status_[PAD_(l_uint *)];
	l_int	options;	char options_[PAD_(l_int)];
	struct l_rusage *	rusage;	char rusage_[PAD_(struct l_rusage *)];
};
struct	linux_kill_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_int	pid;	char pid_[PAD_(l_int)];
	l_int	signum;	char signum_[PAD_(l_int)];
};
struct	linux_newuname_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	struct l_new_utsname *	buf;	char buf_[PAD_(struct l_new_utsname *)];
};
struct	linux_fcntl64_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_uint	fd;	char fd_[PAD_(l_uint)];
	l_uint	cmd;	char cmd_[PAD_(l_uint)];
	l_ulong	arg;	char arg_[PAD_(l_ulong)];
};
struct	linux_getcwd_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	char *	buf;	char buf_[PAD_(char *)];
	l_ulong	bufsize;	char bufsize_[PAD_(l_ulong)];
};
struct	linux_chdir_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	char *	path;	char path_[PAD_(char *)];
};
struct	linux_getuid_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	register_t dummy;
};
struct	linux_getgid_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	register_t dummy;
};
struct	linux_getppid_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	register_t dummy;
};
struct	linux_rt_sigsuspend_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_sigset_t *	newset;	char newset_[PAD_(l_sigset_t *)];
	l_size_t	sigsetsize;	char sigsetsize_[PAD_(l_size_t)];
};
struct	linux_arch_prctl_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_int	code;	char code_[PAD_(l_int)];
	l_ulong	addr;	char addr_[PAD_(l_ulong)];
};
struct	linux_tkill_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	int	tid;	char tid_[PAD_(int)];
	int	sig;	char sig_[PAD_(int)];
};
struct	linux_sys_futex_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	void *	uaddr;	char uaddr_[PAD_(void *)];
	int	op;	char op_[PAD_(int)];
	int	val;	char val_[PAD_(int)];
	struct l_timespec *	timeout;	char timeout_[PAD_(struct l_timespec *)];
	void *	uaddr2;	char uaddr2_[PAD_(void *)];
	int	val3;	char val3_[PAD_(int)];
};
struct	linux_exit_group_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	int	rval;	char rval_[PAD_(int)];
};
struct	linux_tgkill_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	int	tgid;	char tgid_[PAD_(int)];
	int	pid;	char pid_[PAD_(int)];
	int	sig;	char sig_[PAD_(int)];
};
struct	linux_set_robust_list_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	struct linux_robust_list_head *	head;	char head_[PAD_(struct linux_robust_list_head *)];
	l_size_t	len;	char len_[PAD_(l_size_t)];
};
struct	linux_get_robust_list_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_int	pid;	char pid_[PAD_(l_int)];
	struct linux_robust_list_head **	head;	char head_[PAD_(struct linux_robust_list_head **)];
	l_size_t *	len;	char len_[PAD_(l_size_t *)];
};

#ifdef COMPAT_43

#define	nosys	linux_nosys

#ifdef _KERNEL


#endif /* _KERNEL */

#endif /* COMPAT_43 */


#ifdef COMPAT_DF12

#define	nosys	linux_nosys

#ifdef _KERNEL


#endif /* _KERNEL */

#endif /* COMPAT_DF12 */


#ifdef _KERNEL

#define	nosys	linux_nosys
int	sys_linux_open (struct linux_open_args *);
int	sys_linux_stat64 (struct linux_stat64_args *);
int	sys_linux_fstat64 (struct linux_fstat64_args *);
int	sys_linux_lstat64 (struct linux_lstat64_args *);
int	sys_linux_lseek (struct linux_lseek_args *);
int	sys_linux_mmap2 (struct linux_mmap2_args *);
int	sys_linux_brk (struct linux_brk_args *);
int	sys_linux_rt_sigaction (struct linux_rt_sigaction_args *);
int	sys_linux_rt_sigprocmask (struct linux_rt_sigprocmask_args *);
int	sys_linux_rt_sigreturn (struct linux_rt_sigreturn_args *);
int	sys_linux_ioctl (struct linux_ioctl_args *);
int	sys_linux_access (struct linux_access_args *);
int	sys_linux_getpid (struct linux_getpid_args *);
int	sys_linux_clone (struct linux_clone_args *);
int	sys_linux_execve (struct linux_execve_args *);
int	sys_linux_wait4 (struct linux_wait4_args *);
int	sys_linux_kill (struct linux_kill_args *);
int	sys_linux_newuname (struct linux_newuname_args *);
int	sys_linux_fcntl64 (struct linux_fcntl64_args *);
int	sys_linux_getcwd (struct linux_getcwd_args *);
int	sys_linux_chdir (struct linux_chdir_args *);
int	sys_linux_getuid (struct linux_getuid_args *);
int	sys_linux_getgid (struct linux_getgid_args *);
int	sys_linux_getppid (struct linux_getppid_args *);
int	sys_linux_rt_sigsuspend (struct linux_rt_sigsuspend_args *);
int	sys_linux_arch_prctl (struct linux_arch_prctl_args *);
int	sys_linux_tkill (struct linux_tkill_args *);
int	sys_linux_sys_futex (struct linux_sys_futex_args *);
int	sys_linux_exit_group (struct linux_exit_group_args *);
int	sys_linux_tgkill (struct linux_tgkill_args *);
int	sys_linux_set_robust_list (struct linux_set_robust_list_args *);
int	sys_linux_get_robust_list (struct linux_get_robust_list_args *);

#endif /* !_LINUX_SYSPROTO_H_ */
#undef PAD_

#endif /* _KERNEL */
