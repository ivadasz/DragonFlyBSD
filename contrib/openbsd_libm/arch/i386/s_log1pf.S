/*	$OpenBSD: s_log1pf.S,v 1.3 2016/09/12 19:47:02 guenther Exp $ */
/*
 * Written by J.T. Conklin <jtc@netbsd.org>.
 * Public domain.
 */

#include "DEFS.h"

/*
 * Since the fyl2xp1 instruction has such a limited range:
 *	-(1 - (sqrt(2) / 2)) <= x <= sqrt(2) - 1
 * it's not worth trying to use it.  
 */

ENTRY(log1pf)
	fldln2
	flds 4(%esp)
	fld1
	faddp
	fyl2x
	ret
#if 0
END_STD(log1pf)
#else
END(log1pf)
#endif

.section .note.GNU-stack,"",%progbits
