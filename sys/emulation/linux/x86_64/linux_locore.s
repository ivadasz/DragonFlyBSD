/* $FreeBSD: src/sys/i386/linux/linux_locore.s,v 1.5.2.3 2001/11/05 19:08:23 marcel Exp $ */
/* $DragonFly: src/sys/emulation/linux/i386/linux_locore.s,v 1.4 2007/04/13 12:12:27 corecode Exp $ */

#include "linux_assym.h"			/* system definitions */
#include <machine/asmacros.h>			/* miscellaneous asm macros */
#include "linux_syscall.h"			/* system call numbers */

NON_GPROF_ENTRY(linux_sigcode)
	call	*LINUX_SIGF_HANDLER(%rsp)
	leaq	LINUX_SIGF_SC(%rsp),%rbx	/* linux scp */
	mov	LINUX_SC_GS(%rbx),%gs
	movq	%rsp, %rbx			/* pass sigframe */
	push	%rax				/* fake ret addr */
#if 0
	movq	$LINUX_SYS_linux_sigreturn,%rax	/* linux_sigreturn() */
#endif
	syscall
0:	jmp	0b
	ALIGN_TEXT
/* XXXXX */
linux_rt_sigcode:
	call	*LINUX_RT_SIGF_HANDLER(%rsp)
	leaq	LINUX_RT_SIGF_UC(%rsp),%rbx	/* linux ucp */
	leaq	LINUX_RT_SIGF_SC(%rbx),%rcx	/* linux sigcontext */
	mov	LINUX_SC_GS(%rcx),%gs
	push	%rax				/* fake ret addr */
#if 0
	movq	$LINUX_SYS_linux_rt_sigreturn,%rax   /* linux_rt_sigreturn() */
#endif
	syscall
0:	jmp	0b
	ALIGN_TEXT
/* XXXXX */
linux_esigcode:

	.data
	.globl	linux_szsigcode, linux_sznonrtsigcode
linux_szsigcode:
	.long	linux_esigcode-linux_sigcode
linux_sznonrtsigcode:
	.long	linux_rt_sigcode-linux_sigcode
