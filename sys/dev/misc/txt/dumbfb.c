#include "dumbfb.h"
#include "fbinfo.h"

void
dumbfb_blit_char_1to32(uint32_t *fb, uint8_t *glyph, uint16_t width,
    uint16_t height, uint16_t glyphstride, uint32_t fbstride,
    uint32_t bg, uint32_t fg)
{
	/* XXX */
}

int
register_dumbfb_txtdev(struct fb_info *info)
{
	/* XXX */

//	return register_txtdev((void *)info, dumfbsw, how);
	return 0;
}
