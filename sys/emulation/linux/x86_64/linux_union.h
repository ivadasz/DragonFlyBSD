/*
 * Union of syscall args for messaging.
 *
 * DO NOT EDIT-- To regenerate this file, edit syscalls.master followed
 *               by running make sysent in the same directory.
 */

union sysunion {
#ifdef _KERNEL /* header only applies in kernel */
	struct	lwkt_msg lmsg;
	struct	sysmsg sysmsg;
#endif
#define	nosys	linux_nosys
	struct	linux_open_args linux_open;
	struct	linux_execve_args linux_execve;
};
