/*	$OpenBSD: e_log.S,v 1.4 2016/09/12 19:47:02 guenther Exp $ */
/*
 * Written by J.T. Conklin <jtc@netbsd.org>.
 * Public domain.
 */

#include "DEFS.h"

ENTRY(log)
	fldln2
	fldl	4(%esp)
	fyl2x
	ret
#if 0
END_STD(log)
#else
END(log)
#endif

.section .note.GNU-stack,"",%progbits
