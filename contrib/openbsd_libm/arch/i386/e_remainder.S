/*	$OpenBSD: e_remainder.S,v 1.5 2016/09/12 19:47:02 guenther Exp $ */
/*
 * Written by J.T. Conklin <jtc@netbsd.org>.
 * Public domain.
 */

#include "DEFS.h"

ENTRY(remainder)
	fldl	12(%esp)
	fldl	4(%esp)
1:	fprem1
	fstsw	%ax
	sahf
	jp	1b
	fstp	%st(1)
	ret
#if 0
END_STD(remainder)
#else
END(remainder)
#endif

.section .note.GNU-stack,"",%progbits
