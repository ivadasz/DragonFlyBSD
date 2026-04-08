/*	$OpenBSD: s_rint.S,v 1.3 2016/09/12 19:47:02 guenther Exp $ */
/*
 * Written by J.T. Conklin <jtc@netbsd.org>.
 * Public domain.
 */

#include "DEFS.h"

ENTRY(rint)
	fldl	4(%esp)
	frndint
	ret
#if 0
END_STD(rint)
#else
END(rint)
#endif

.section .note.GNU-stack,"",%progbits
