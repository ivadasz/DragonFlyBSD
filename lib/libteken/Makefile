# $FreeBSD$

LIB=	private_teken${LIB_SUFFIX}
PRIVATELIB=
SHLIB_MAJOR= 0
INCSDIR=	${INCLUDEDIR}/priv

SHLIBDIR?=      /lib

WARNS?=	6

SRCDIR=	${.CURDIR}

SRCS=	teken.c
INCS=	teken.h
CLEANFILES=teken_state.h

CFLAGS+=	-I${.OBJDIR}

MAN=
#MAN=	teken.3
#MLINKS=	teken.3 teken_256to8.3 \
#	teken.3 teken_get_curattr.3 \
#	teken.3 teken_get_cursor.3 \
#	teken.3 teken_get_defattr.3 \
#	teken.3 teken_get_defattr_cons25.3 \
#	teken.3 teken_get_sequence.3 \
#	teken.3 teken_get_winsize.3 \
#	teken.3 teken_init.3 \
#	teken.3 teken_input.3 \
#	teken.3 teken_set_8bit.3 \
#	teken.3 teken_set_cons25.3 \
#	teken.3 teken_set_curattr.3 \
#	teken.3 teken_set_cursor.3 \
#	teken.3 teken_set_defattr.3 \
#	teken.3 teken_set_winsize.3

teken_state.h: ${SRCDIR}/gensequences ${SRCDIR}/sequences
	awk -f ${SRCDIR}/gensequences ${SRCDIR}/sequences > ${.TARGET}

teken.c:	teken_state.h

.include <bsd.lib.mk>
