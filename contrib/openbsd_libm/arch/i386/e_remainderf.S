/*	$OpenBSD: e_remainderf.S,v 1.5 2016/09/12 19:47:02 guenther Exp $ */
/*
 * Written by J.T. Conklin <jtc@netbsd.org>.
 * Public domain.
 */

#include "DEFS.h"

ENTRY(remainderf)
	flds	8(%esp)
	flds	4(%esp)
1:	fprem1
	fstsw	%ax
	sahf
	jp	1b
	fstp	%st(1)
	ret
#if 0
END_STD(remainderf)
#else
END(remainderf)
#endif

.section .note.GNU-stack,"",%progbits
