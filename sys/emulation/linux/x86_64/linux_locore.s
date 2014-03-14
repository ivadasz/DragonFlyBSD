/* $FreeBSD: src/sys/i386/linux/linux_locore.s,v 1.5.2.3 2001/11/05 19:08:23 marcel Exp $ */
/* $DragonFly: src/sys/emulation/linux/i386/linux_locore.s,v 1.4 2007/04/13 12:12:27 corecode Exp $ */

#include "linux_assym.h"			/* system definitions */
#include <machine/asmacros.h>			/* miscellaneous asm macros */
#include "linux_syscall.h"			/* system call numbers */

/*
 * The signal trampoline is not used on Linux/amd64: a
 * libc provided trampoline is always used.
 * We just provide the symbol so that the kernel builds.
 */

NON_GPROF_ENTRY(linux_sigcode)
	ALIGN_TEXT
linux_rt_sigcode:
	ALIGN_TEXT
linux_esigcode:

	.data
	.globl	linux_szsigcode, linux_sznonrtsigcode
linux_szsigcode:
/*	.long	linux_esigcode-linux_sigcode */
linux_sznonrtsigcode:
/*	.long	linux_rt_sigcode-linux_sigcode */
