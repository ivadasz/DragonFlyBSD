/*
 * Copyright (c) 2007 Matthew Dillon <dillon@backplane.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Neither the name of the author nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $DragonFly: src/lib/libc/i386/gen/mcontext.S,v 1.2 2007/04/13 12:12:27 corecode Exp $
 */
/*
 * swapcontext_quick() and setcontext_quick().  There is *NO*
 * getcontext_quick() routine on purpose.
 *
 * Quick routines are not required to save or resetore scratch registers,
 * FP regs (which are scratch), or flags.
 */
#include <machine/asm.h>
#include <cpu/specialreg.h>
#include <asmcontext.h>

	/*
	 * void setcontext_quick(ucontext_t *ucp)
	 *
	 * Load the register context, effectively switching to the
	 * new context.
	 */
        .weak   setcontext_quick
        .set    setcontext_quick,_setcontext_quick
ENTRY(_setcontext_quick)
	movl	4(%esp),%eax
	addl	$UC_MCONTEXT,%eax

	/*	0*4(%eax),mc_onstack	*/
	movw	1*4(%eax),%gs
	movw	2*4(%eax),%fs
	movw	3*4(%eax),%es
	/*	4*4(%eax),%ds	*/
	movl	5*4(%eax),%edi
	movl	6*4(%eax),%esi
	movl	7*4(%eax),%ebp
	/*	8*4(%eax),%isp	*/
	movl	9*4(%eax),%ebx
	movl	10*4(%eax),%edx
	movl	11*4(%eax),%ecx
	/*	mc_eax:		12*4(%eax)	*/
	/*	mc_xflags:	13*4(%eax)	*/
	/*	mc_trapno:	14*4(%eax)	*/
	/*	mc_err:		15*4(%eax)	*/
	/*	mc_eip:		16*4(%eax)	*/
	/*	mc_cs:		17*4(%eax)	*/
	/*	mc_eflags:	18*4(%eax)	*/
	/*	mc_esp:		19*4(%eax)	*/

	/*
	 * Restore eflags
	 */
	pushl	18*4(%eax)
	popfl

	/*
	 * Load the new stack pointer
	 */
	movl	19*4(%eax),%esp

	/*
	 * Use %eax to hold the return pc.  function returns void.  Do
	 * not mess with the target stack (particularly not the red-zone!).
	 */
	movl	16*4(%eax),%eax
	jmp	*%eax
END(_setcontext_quick)


	/*
	 * void swapcontext_quick(ucontex_t *oucp, ucontext_t *nucp)
	 *
	 * Save the current state in oucp and switch to nucp.
	 */
	.weak	swapcontext_quick
	.set	swapcontext_quick,_swapcontext_quick
ENTRY(_swapcontext_quick)
	/*
	 * Save
	 */
	movl	1*4(%esp),%eax
	addl	$UC_MCONTEXT,%eax

	/* Pop the return PC, we'll return by jmp. */
	popl	%edx	/* return pc */

	movl	%edx,16*4(%eax)
	/*	mc_onstack,0*4(%eax)	*/
	movw	%gs,1*4(%eax)
	movw	%fs,2*4(%eax)
	movw	%es,3*4(%eax)
	/*	%ds,4*4(%eax)	*/
	movl	%edi,5*4(%eax)
	movl	%esi,6*4(%eax)
	movl	%ebp,7*4(%eax)
	/*	%isp,8*4(%eax)	*/
	movl	%ebx,9*4(%eax)
	movl	%edx,10*4(%eax)
	movl	%ecx,11*4(%eax)
	movl	%esp,19*4(%eax)

	/*
	 * Restore (copy of setcontext above)
	 */
	movl	2*4(%esp),%eax
	addl	$UC_MCONTEXT,%eax

	/*	0*4(%eax),mc_onstack	*/
	movw	1*4(%eax),%gs
	movw	2*4(%eax),%fs
	movw	3*4(%eax),%es
	/*	4*4(%eax),%ds	*/
	movl	5*4(%eax),%edi
	movl	6*4(%eax),%esi
	movl	7*4(%eax),%ebp
	/*	8*4(%eax),%isp	*/
	movl	9*4(%eax),%ebx
	movl	10*4(%eax),%edx
	movl	11*4(%eax),%ecx
	movl	19*4(%eax),%esp

	/*
	 * Use %eax to hold the return pc.  function returns void.  Do
	 * not mess with the target stack (particularly not the red-zone!).
	 */
	movl	16*4(%eax),%eax
	jmp	*%eax
END(_swapcontext_quick)

	.section .note.GNU-stack,"",%progbits
