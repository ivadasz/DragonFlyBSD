.include "../platform/${MACHINE_PLATFORM}/Makefile.inc"

SUBDIR=	\
	acpica \
	agp \
	crypto \
	disk \
	drm \
	misc \
	netif \
	pccard \
	powermng \
	raid \
	serial \
	sound \
	smbus \
	video \
	virtual

.for dir in ${SUBDIR}
.if empty(DEV_SUPPORT:M${dir}) && empty(DEV_SUPPORT:M${dir}/*)
SUBDIR:= ${SUBDIR:N${dir}}
.endif
.endfor

.include <bsd.subdir.mk>
