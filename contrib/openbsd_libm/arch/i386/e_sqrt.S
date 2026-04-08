/*	$OpenBSD: e_sqrt.S,v 1.4 2016/09/12 19:47:02 guenther Exp $ */
/*
 * Written by J.T. Conklin <jtc@netbsd.org>.
 * Public domain.
 */

#include "DEFS.h"

ENTRY(sqrt)
	fldl	4(%esp)
	fsqrt
	ret
#if 0
END_STD(sqrt)
#else
END(sqrt)
#endif

.section .note.GNU-stack,"",%progbits
