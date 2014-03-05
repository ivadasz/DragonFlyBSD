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
struct	linux_execve_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	char *	path;	char path_[PAD_(char *)];
	char **	argp;	char argp_[PAD_(char **)];
	char **	envp;	char envp_[PAD_(char **)];
};
struct	linux_kill_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_int	pid;	char pid_[PAD_(l_int)];
	l_int	signum;	char signum_[PAD_(l_int)];
};
struct	linux_rt_sigsuspend_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	l_sigset_t *	newset;	char newset_[PAD_(l_sigset_t *)];
	l_size_t	sigsetsize;	char sigsetsize_[PAD_(l_size_t)];
};
struct	linux_tkill_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	int	tid;	char tid_[PAD_(int)];
	int	sig;	char sig_[PAD_(int)];
};
struct	linux_tgkill_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	int	tgid;	char tgid_[PAD_(int)];
	int	pid;	char pid_[PAD_(int)];
	int	sig;	char sig_[PAD_(int)];
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
int	sys_linux_rt_sigaction (struct linux_rt_sigaction_args *);
int	sys_linux_rt_sigprocmask (struct linux_rt_sigprocmask_args *);
int	sys_linux_rt_sigreturn (struct linux_rt_sigreturn_args *);
int	sys_linux_execve (struct linux_execve_args *);
int	sys_linux_kill (struct linux_kill_args *);
int	sys_linux_rt_sigsuspend (struct linux_rt_sigsuspend_args *);
int	sys_linux_tkill (struct linux_tkill_args *);
int	sys_linux_tgkill (struct linux_tgkill_args *);

#endif /* !_LINUX_SYSPROTO_H_ */
#undef PAD_

#endif /* _KERNEL */
