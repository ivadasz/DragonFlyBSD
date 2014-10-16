#ifndef _DEV_SYSCONS_FBINFO_H_
#define _DEV_SYSCONS_FBINFO_H_

struct fb_info {
	vm_offset_t vaddr;
	vm_paddr_t paddr;
	uint16_t width;
	uint16_t height;
	uint16_t stride;
	uint16_t depth;
	int is_vga_boot_display;
	void *cookie;
	void(*restore)(void *, int);
};

/* XXX */

#endif /* !_DEV_SYSCONS_FBINFO_H_ */
