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
struct	linux_execve_args {
#ifdef _KERNEL
	struct sysmsg sysmsg;
#endif
	char *	path;	char path_[PAD_(char *)];
	char **	argp;	char argp_[PAD_(char **)];
	char **	envp;	char envp_[PAD_(char **)];
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
int	sys_linux_execve (struct linux_execve_args *);

#endif /* !_LINUX_SYSPROTO_H_ */
#undef PAD_

#endif /* _KERNEL */
