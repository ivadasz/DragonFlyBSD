/*
 * Copyright (c) 2006 The DragonFly Project.  All rights reserved.
 * 
 * This code is derived from software contributed to The DragonFly Project
 * by Matthew Dillon <dillon@backplane.com>
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of The DragonFly Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific, prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 * 
 * $DragonFly: src/lib/libc/i386/sys/getcontext.S,v 1.2 2007/02/04 20:28:21 corecode Exp $
 */

#include <machine/asm.h>
#include <cpu/specialreg.h>
#include <asmcontext.h>

	/*
	 * This function is special-cased because the context it saves
	 * includes a stale stack context (because it returns before the
	 * caller presumably makes the call to setcontext()).  
	 */
	.weak	getcontext
	.set	getcontext,_getcontext
ENTRY(_getcontext)
	/*
	 * Retrieve the current signal mask and save it in &ucp->uc_sigmask.
	 */
	movl	4(%esp),%eax		/* ucontext_t pointer */
	addl	$UC_SIGMASK,%eax	/* pointer to signal mask */
	PIC_PROLOGUE
	pushl	%eax
	pushl	$0
	pushl	$SIG_BLOCK
	call	PIC_PLT(_sigprocmask)	/* retrieve & save signal mask */
	addl	$12,%esp
	PIC_EPILOGUE

	/*
	 * Save what we need because our stack context is going stale.
	 */
	movl	4(%esp),%eax
	movl	%eax,%ecx		/* save ucontext_t in %ecx */
	movl	$0,UC_LINK(%eax)	/* make sure UC_LINK is clear */
	movl	(%esp),%edx		/* save return PC in %edx */
	addl	$UC_MCONTEXT,%eax

	movl	$0,0*4(%eax)
	movl	%edx,16*4(%eax)
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
	/* movl	%cs,17*4(%eax) */

	pushfl
	popl	18*4(%eax)

	/* movl	%ss,20*4(%eax) */

	/*
	 * FP registers are scratch, do not save or restore, but make
	 * sure the context can be restored by a signal handler (where
	 * they might not be) by properly initializing the FP control
	 * fields.
	 */
	movl	$_MC_FPFMT_NODEV,22*4(%eax)
	movl	$_MC_FPOWNED_NONE,23*4(%eax)
/*
	movq	%rdi,%rsi
	movq	$CPU_XFEATURE_X87 | CPU_XFEATURE_SSE,%eax
	movq	$0,%rdx
	xsave	MC_FPREGS(%rsi)
	movq	%rsi,%rdi
*/

	/*
	 * Saved stack pointer as if we had returned from this
	 * procedure.
	 */
	movl	%esp,19*4(%eax)
	addl	$4,19*4(%eax)

	/*
	 * On restore as if procedure returned the value 1
	 */
	movl	$1,12*4(%eax)
	/*
	 * Set MC_LEN
	 */
	movl	$SIZEOF_MCONTEXT_T,21*4(%eax)

	/*
	 * Return 0
	 */
	xorl	%eax,%eax
	ret

#if 0
1:
	PIC_PROLOGUE
	jmp	PIC_PLT(HIDENAME(cerror))
#endif
END(_getcontext)

	.section .note.GNU-stack,"",%progbits
