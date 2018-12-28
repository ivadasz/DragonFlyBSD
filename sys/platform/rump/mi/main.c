#include <stdio.h>
#include <stdlib.h>

__thread int mycpuid;
__thread struct globaldata *mycpu;

void
main(int argc, char *argv[])
{
	/* XXX Initialize BSP's mycpuid and mycpu values. */

	printf("Hello, World!\n");

	exit(0);
}
