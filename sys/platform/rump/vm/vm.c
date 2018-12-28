#include <sys/param.h>
#include <vm/vm_extern.h>

#include <sys/mman.h>
#include <assert.h>

int kernel_map; /* dummy */

vm_offset_t
kmem_alloc (vm_map_t map, vm_size_t size, vm_subsys_t id)
{
	void *addr;

	addr = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_ANON, -1, 0);
	assert(addr != MAP_FAILED);
	return (vm_offset_t)addr;
}

void
kmem_free (vm_map_t map, vm_offset_t off, vm_size_t size)
{
	munmap((void *)off, size);
}
