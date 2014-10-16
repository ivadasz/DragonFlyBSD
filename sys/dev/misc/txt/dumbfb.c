#include "txtdev.h"
#include "dumbfb.h"
#include "fbinfo.h"

#include "gallant12x22.h"

struct dumbfb_state {
	struct fb_info *fbi;
	int cursorpos;
	int columns;
	int rows;
	uint16_t saveunder;
	uint16_t *buffer;
};

#if notyet
static uint16_t early_buffer[80*25];
static struct dumbfb_state early_txtdev = {
	.fbi = NULL,
	.cursorpos = 0,
	.columns = 80,
	.rows = 25,
	.saveunder = 0x0000,
	.buffer = early_buffer
};
#endif

static int dumbfb_setcursor(void *cookie, int pos);

static uint32_t colormap[16] = {
	0x00000000,	/* BLACK */
	0x00000080,	/* BLUE */
	0x00008000,	/* GREEN */
	0x00008080,	/* CYAN */
	0x00800000,	/* RED */
	0x00800080,	/* MAGENTA */
	0x00808000,	/* BROWN */
	0x00c0c0c0,	/* WHITE */
	0x00808080,	/* HIGHLIGHT BLACK */
	0x000000ff,	/* HIGHLIGHT BLUE */
	0x0000ff00,	/* HIGHLIGHT GREEN */
	0x0000ffff,	/* HIGHLIGHT CYAN */
	0x00ff0000,	/* HIGHLIGHT RED */
	0x00ff00ff,	/* HIGHLIGHT MAGENTA */
	0x00ffff00,	/* HIGHLIGHT BROWN */
	0x00ffffff,	/* HIGHLIGHT WHITE */
};

static void
dumbfb_blit_char_1to32(uint32_t *fb, uint8_t *glyph, uint16_t glyphstride,
    uint16_t fbstride, uint16_t glyphwidth, uint16_t glyphheight,
    uint32_t bg, uint32_t fg)
{
	int j, k;

	for (j = 0; j < glyphheight; j++) {
		for (k = 0; k < glyphwidth; k++) {
			fb[j*(fbstride/4) + k] =
			    ((glyph[j * glyphstride + k/8] >>
			    (7 - (k%8))) & 1) ? fg : bg;
		}
	}
}

static int
dumbfb_getmode(void *cookie, struct txtmode *mode)
{
	struct dumbfb_state *st = (struct dumbfb_state *)cookie;

	mode->txt_columns = st->columns;
	mode->txt_rows = st->rows;

	return 0;
}

static int
dumbfb_setmode(void *cookie, struct txtmode *mode)
{
	struct dumbfb_state *st = (struct dumbfb_state *)cookie;

	/* XXX disable mode changing for now */
	if (mode->txt_columns != st->columns ||
	    mode->txt_rows != st->rows)
		return 1;

	return 0;
}

static int
dumbfb_putchars(void *cookie, int col, int row, uint16_t *buf, int len)
{
	struct dumbfb_state *st = (struct dumbfb_state *)cookie;
	struct fb_info *info = st->fbi;
	int tw, th, i, pos;
	uint32_t bg, fg;
	uint16_t *tobuf;
	uint8_t attr, index;
	uint16_t glyphstride = 2;
	uint16_t glyphheight = 22;
	uint16_t glyphwidth = 12;
	uint8_t *font = gallant12x22_data;
	uint8_t *fb, *myfb, *glyph;

	tw = st->columns;
	th = st->rows;
	fb = (uint8_t *)info->vaddr;
	pos = tw * row + col;
	if (pos >= tw * th)
		return 1;
	if (pos + len > tw * st->rows)
		len = tw * th - pos;
	tobuf = &st->buffer[pos];

	for (i = 0; i < len; i++) {
		if (st->cursorpos == pos + i) {
			if (st->saveunder != buf[i]) {
				tobuf[i] = buf[i];
				st->cursorpos = -1;
				dumbfb_setcursor(cookie, pos + i);
			}
		} else if (tobuf[i] != buf[i]) {
			tobuf[i] = buf[i];
			attr = (tobuf[i] >> 8) & 0xff;
			index = tobuf[i] & 0xff;
			fg = colormap[attr & 0xf];
			bg = colormap[(attr >> 4) & 0xf];
			glyph = &font[glyphheight * glyphstride * index];
			myfb = &fb[((pos + i)/tw)*glyphheight*info->stride +
				   ((pos + i)%tw)*glyphwidth*4];
			dumbfb_blit_char_1to32((uint32_t *)myfb, glyph,
			    glyphstride, info->stride, glyphwidth, glyphheight,
			    bg, fg);
		}
	}

	return 0;
}

static int
dumbfb_getchars(void *cookie, int col, int row, uint16_t *buf, int len)
{
#if notyet
	struct dumbfb_state *st = (struct dumbfb_state *)cookie;
#endif

	return 1;
}

static int
dumbfb_setcursor(void *cookie, int pos)
{
	struct dumbfb_state *st = (struct dumbfb_state *)cookie;
	int v;
	uint16_t val;
	uint16_t a, c;

	if (pos < -1)
		return 1;

	if (st->cursorpos == pos)
		return 0;

	if (st->cursorpos >= 0) {
		v = st->cursorpos;
		st->cursorpos = -1;
		dumbfb_putchars(cookie, v%st->columns, v/st->columns,
		    &st->saveunder, 1);
	}

	if (pos >= 0) {
		st->saveunder = st->buffer[pos];
		a = st->saveunder & 0xff00;
		c = st->saveunder & 0x00ff;
		if ((a & 0x7000) == 0x7000) {
			a &= 0x8f00;
			if ((a & 0x0700) == 0)
				a |= 0x0700;
		} else {
			a |= 0x7000;
			if ((a & 0x0700) == 0x0700)
				a &= 0xf000;
		}
		val = a | c;
		dumbfb_putchars(cookie, pos%st->columns, pos/st->columns,
		    &val, 1);
	}
	st->cursorpos = pos;

	return 0;
}

static int
dumbfb_getcursor(void *cookie, int *pos)
{
	struct dumbfb_state *st = (struct dumbfb_state *)cookie;

	*pos = st->cursorpos;

	return 0;
}

static int
dumbfb_setcurmode(void *cookie, int mode)
{
#if notyet
	struct dumbfb_state *st = (struct dumbfb_state *)cookie;
#endif

	/* only supporting block mode for now */

	return 0;
}

static char *
dumbfb_getname(void *cookie)
{
#if notyet
	struct dumbfb_state *st = (struct dumbfb_state *)cookie;
#endif

	/* XXX retrieve device name from struct dumbfb_state */

	return "dumbfb";
}

static void
dumbfb_restore(void *cookie, int emergency)
{
	struct dumbfb_state *st = (struct dumbfb_state *)cookie;
	struct fb_info *fb = st->fbi;

	fb->restore(fb->cookie, emergency);
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
	struct dumbfb_state *st;
	int how;
	int error, i, j;

	st = kmalloc(sizeof(struct dumbfb_state), M_DEVBUF, M_ZERO | M_WAITOK);
	st->buffer = kmalloc(80*25*sizeof(*st->buffer), M_DEVBUF,
	    M_ZERO | M_WAITOK);
	st->fbi = info;
	st->columns = 80;
	st->rows = 25;
	st->cursorpos = -1;
	st->saveunder = 0x0000;

	if (info->is_vga_boot_display)
		how = TXTDEV_REPLACE_VGA;
	else
		how = 0;

	for (i = 0; i < info->height; i++) {
		for (j = 0; j < info->width; j++) {
			*(uint32_t *)&((uint8_t *)info->vaddr)[i*info->stride+j*4] = ((i * info->width + j) % 3 == 0) ? 0xffffffff : 0x00000000;
		}
	}

	error = register_txtdev((void *)st, &dumbfbsw, how);
	if (error != 0) {
		kfree(st->buffer, M_DEVBUF);
		kfree(st, M_DEVBUF);
	}

	return error;
}
