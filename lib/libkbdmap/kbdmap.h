#ifndef _KBDMAP_H
#define _KBDMAP_H

char	*kbdmap_parse(char *name, keymap_t *keymap, accentmap_t *accentmap);
uint8_t	kbdmap_translate(keymap_t *keymap, uint8_t code);
void	kbdmap_dump(FILE *fp, keymap_t *keymap, accentmap_t *accentmap,
	    int hex);
void	kbdmap_dump_key_definition(char *name, keymap_t *keymap);
void	kbdmap_dump_accent_definition(char *name, accentmap_t *accentmap);

#endif
