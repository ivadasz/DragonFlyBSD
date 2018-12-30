#include <sys/msgbuf.h>

#include <stdio.h>
#include <stdlib.h>

__thread int mycpuid;
__thread struct globaldata *mycpu;

char buf_data[sizeof(struct msgbuf) + MSGBUF_SIZE];
struct msgbuf *msgbufp = (void *)buf_data;

void
main(int argc, char *argv[])
{
	/* XXX Initialize BSP's mycpuid and mycpu values. */

	printf("Hello, World!\n");

	exit(0);
}
