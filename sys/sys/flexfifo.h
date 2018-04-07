#ifndef _SYS_FLEXFIFO_H_
#define _SYS_FLEXFIFO_H_

struct flexfifo_ops {
	u_int(*pktlen)(void *);
	u_int(*evtopkt)(void *arg, u_int8_t *ev, u_int8_t *pkt);
	void(*sendev)(void *arg, u_int8_t *pkt);
	int(*pkttoev)(void *arg, u_int8_t *ev, u_int8_t *pkt, u_int len);
	int(*ioctl)(void *arg, caddr_t data, u_long cmd, int fflags);
	void(*open)(void *arg);
	void(*close)(void *arg);
};

/* Only ever read/write a single packet. */
#define FLEXFIFO_FLAG_SINGLEPKT	0x01

struct flexfifo	*flexfifo_create(u_int chunk, u_int count,
		    struct flexfifo_ops *ops, int minor, const char *name,
		    void *arg, u_int max_pktlen);
void		flexfifo_setflags(struct flexfifo *fifo, int flags);
void		flexfifo_destroy(struct flexfifo *fifo);
cdev_t		flexfifo_get_cdev(struct flexfifo *fifo);
void		flexfifo_enqueue_ring(struct flexfifo *fifo, uint8_t *chunk);
void		flexfifo_write_done(struct flexfifo *fifo);

/* Can only be safely used from flexfifo_ops callbacks. */
uint8_t		*flexfifo_peek_ring(struct flexfifo *fifo);

#endif /* !_SYS_FLEXFIFO_H_ */
