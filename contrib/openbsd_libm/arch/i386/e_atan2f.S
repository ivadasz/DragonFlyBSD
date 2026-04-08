/*	$OpenBSD: e_atan2f.S,v 1.4 2016/09/12 19:47:02 guenther Exp $ */
/*
 * Written by J.T. Conklin <jtc@netbsd.org>.
 * Public domain.
 */

#include "DEFS.h"

ENTRY(atan2f)
	flds	4(%esp)
	flds	8(%esp)
	fpatan
	ret
#if 0
END_STD(atan2f)
#else
END(atan2f)
#endif

.section .note.GNU-stack,"",%progbits
