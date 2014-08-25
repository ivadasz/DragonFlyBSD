#include "txtdev.h"
#include "dumbfb.h"
#include "fbinfo.h"

#if 0
static void
dumbfb_blit_char_1to32(uint32_t *fb, uint8_t *glyph, uint16_t width,
    uint16_t height, uint16_t glyphstride, uint32_t fbstride,
    uint32_t bg, uint32_t fg)
{
	/* XXX */
}
#endif

static int
dumbfb_getmode(void *cookie, struct txtmode *mode)
{
	mode->txt_columns = 80;
	mode->txt_rows = 25;

	return 0;
}

static int
dumbfb_setmode(void *cookie, struct txtmode *mode)
{
	if (mode->txt_columns != 80 ||
	    mode->txt_rows != 25)
		return 1;

	return 0;
}

static int
dumbfb_putchars(void *cookie, int col, int row, uint16_t *buf, int len)
{
	/* XXX */

	return 0;
}

static int
dumbfb_getchars(void *cookie, int col, int row, uint16_t *buf, int len)
{
	return 1;
}

static int
dumbfb_setcursor(void *cookie, int pos)
{
	return 0;
}

static int
dumbfb_getcursor(void *cookie, int *pos)
{
	*pos = -1;

	return 0;
}

static int
dumbfb_setcurmode(void *cookie, int mode)
{
	/* only supporting block mode for now */

	return 0;
}

static char *
dumbfb_getname(void *cookie)
{
	return "kms";
}

static void
dumbfb_restore(void *cookie)
{
	struct fb_info *fb = (struct fb_info *)cookie;

	fb->restore(fb->cookie);
}

struct txtdev_sw dumbfbsw = {
	dumbfb_getmode,
	dumbfb_setmode,
	dumbfb_putchars,
	dumbfb_getchars,
	dumbfb_setcursor,
	dumbfb_getcursor,
	dumbfb_setcurmode,
	dumbfb_getname,
	dumbfb_restore
};

int
register_dumbfb_txtdev(struct fb_info *info)
{
	int how;

	if (info->is_vga_boot_display)
		how = TXTDEV_REPLACE_VGA;
	else
		how = 0;

	return register_txtdev((void *)info, &dumbfbsw, how);
}
