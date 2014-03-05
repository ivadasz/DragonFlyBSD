/*
 * System call switch table.
 *
 * DO NOT EDIT-- To regenerate this file, edit syscalls.master followed
 *               by running make sysent in the same directory.
 */

#include "opt_compat.h"
#include <sys/param.h>
#include <sys/sysent.h>
#include <sys/sysproto.h>
#include <emulation/linux/linux_sysproto.h>
#include "linux.h"
#include "linux_proto.h"
#include <emulation/43bsd/stat.h>

#define AS(name) ((sizeof(struct name) - sizeof(struct sysmsg)) / sizeof(register_t))

/* The casts are bogus but will do for now. */
struct sysent linux_sysent[] = {
#define	nosys	linux_nosys
	{ 0, (sy_call_t *)sys_nosys },			/* 0 = read */
	{ 0, (sy_call_t *)sys_nosys },			/* 1 = write */
	{ AS(linux_open_args), (sy_call_t *)sys_linux_open },	/* 2 = linux_open */
	{ 0, (sy_call_t *)sys_nosys },			/* 3 = close */
	{ 0, (sy_call_t *)sys_nosys },			/* 4 = stat */
	{ 0, (sy_call_t *)sys_nosys },			/* 5 = fstat */
	{ 0, (sy_call_t *)sys_nosys },			/* 6 = lstat */
	{ 0, (sy_call_t *)sys_nosys },			/* 7 = poll */
	{ 0, (sy_call_t *)sys_nosys },			/* 8 = lseek */
	{ 0, (sy_call_t *)sys_nosys },			/* 9 = mmap */
	{ 0, (sy_call_t *)sys_nosys },			/* 10 = mprotect */
	{ 0, (sy_call_t *)sys_nosys },			/* 11 = munmap */
	{ 0, (sy_call_t *)sys_nosys },			/* 12 = brk */
	{ AS(linux_rt_sigaction_args), (sy_call_t *)sys_linux_rt_sigaction },	/* 13 = linux_rt_sigaction */
	{ AS(linux_rt_sigprocmask_args), (sy_call_t *)sys_linux_rt_sigprocmask },	/* 14 = linux_rt_sigprocmask */
	{ AS(linux_rt_sigreturn_args), (sy_call_t *)sys_linux_rt_sigreturn },	/* 15 = linux_rt_sigreturn */
	{ 0, (sy_call_t *)sys_nosys },			/* 16 = ioctl */
	{ 0, (sy_call_t *)sys_nosys },			/* 17 = pread64 */
	{ 0, (sy_call_t *)sys_nosys },			/* 18 = pwrite64 */
	{ 0, (sy_call_t *)sys_nosys },			/* 19 = readv */
	{ 0, (sy_call_t *)sys_nosys },			/* 20 = writev */
	{ 0, (sy_call_t *)sys_nosys },			/* 21 = access */
	{ 0, (sy_call_t *)sys_nosys },			/* 22 = pipe */
	{ 0, (sy_call_t *)sys_nosys },			/* 23 = select */
	{ 0, (sy_call_t *)sys_nosys },			/* 24 = sched_yield */
	{ 0, (sy_call_t *)sys_nosys },			/* 25 = mremap */
	{ 0, (sy_call_t *)sys_nosys },			/* 26 = msync */
	{ 0, (sy_call_t *)sys_nosys },			/* 27 = mincore */
	{ 0, (sy_call_t *)sys_nosys },			/* 28 = madvise */
	{ 0, (sy_call_t *)sys_nosys },			/* 29 = shmget */
	{ 0, (sy_call_t *)sys_nosys },			/* 30 = shmat */
	{ 0, (sy_call_t *)sys_nosys },			/* 31 = shmctl */
	{ 0, (sy_call_t *)sys_nosys },			/* 32 = dup */
	{ 0, (sy_call_t *)sys_nosys },			/* 33 = dup2 */
	{ 0, (sy_call_t *)sys_nosys },			/* 34 = pause */
	{ 0, (sy_call_t *)sys_nosys },			/* 35 = nanosleep */
	{ 0, (sy_call_t *)sys_nosys },			/* 36 = getitimer */
	{ 0, (sy_call_t *)sys_nosys },			/* 37 = alarm */
	{ 0, (sy_call_t *)sys_nosys },			/* 38 = setitimer */
	{ 0, (sy_call_t *)sys_nosys },			/* 39 = getpid */
	{ 0, (sy_call_t *)sys_nosys },			/* 40 = sendfile */
	{ 0, (sy_call_t *)sys_nosys },			/* 41 = socket */
	{ 0, (sy_call_t *)sys_nosys },			/* 42 = connect */
	{ 0, (sy_call_t *)sys_nosys },			/* 43 = accept */
	{ 0, (sy_call_t *)sys_nosys },			/* 44 = sendto */
	{ 0, (sy_call_t *)sys_nosys },			/* 45 = recvfrom */
	{ 0, (sy_call_t *)sys_nosys },			/* 46 = sendmsg */
	{ 0, (sy_call_t *)sys_nosys },			/* 47 = recvmsg */
	{ 0, (sy_call_t *)sys_nosys },			/* 48 = shutdown */
	{ 0, (sy_call_t *)sys_nosys },			/* 49 = bind */
	{ 0, (sy_call_t *)sys_nosys },			/* 50 = listen */
	{ 0, (sy_call_t *)sys_nosys },			/* 51 = getsockname */
	{ 0, (sy_call_t *)sys_nosys },			/* 52 = getpeername */
	{ 0, (sy_call_t *)sys_nosys },			/* 53 = socketpair */
	{ 0, (sy_call_t *)sys_nosys },			/* 54 = setsockopt */
	{ 0, (sy_call_t *)sys_nosys },			/* 55 = getsockopt */
	{ 0, (sy_call_t *)sys_nosys },			/* 56 = clone */
	{ 0, (sy_call_t *)sys_nosys },			/* 57 = fork */
	{ 0, (sy_call_t *)sys_nosys },			/* 58 = vfork */
	{ AS(linux_execve_args), (sy_call_t *)sys_linux_execve },	/* 59 = linux_execve */
	{ AS(exit_args), (sy_call_t *)sys_exit },	/* 60 = exit */
	{ 0, (sy_call_t *)sys_nosys },			/* 61 = wait4 */
	{ AS(linux_kill_args), (sy_call_t *)sys_linux_kill },	/* 62 = linux_kill */
	{ 0, (sy_call_t *)sys_nosys },			/* 63 = uname */
	{ 0, (sy_call_t *)sys_nosys },			/* 64 = semget */
	{ 0, (sy_call_t *)sys_nosys },			/* 65 = semop */
	{ 0, (sy_call_t *)sys_nosys },			/* 66 = semctl */
	{ 0, (sy_call_t *)sys_nosys },			/* 67 = shmdt */
	{ 0, (sy_call_t *)sys_nosys },			/* 68 = msgget */
	{ 0, (sy_call_t *)sys_nosys },			/* 69 = msgsnd */
	{ 0, (sy_call_t *)sys_nosys },			/* 70 = msgrcv */
	{ 0, (sy_call_t *)sys_nosys },			/* 71 = msgctl */
	{ 0, (sy_call_t *)sys_nosys },			/* 72 = fcntl */
	{ 0, (sy_call_t *)sys_nosys },			/* 73 = flock */
	{ 0, (sy_call_t *)sys_nosys },			/* 74 = fsync */
	{ 0, (sy_call_t *)sys_nosys },			/* 75 = fdatasync */
	{ 0, (sy_call_t *)sys_nosys },			/* 76 = truncate */
	{ 0, (sy_call_t *)sys_nosys },			/* 77 = ftruncate */
	{ 0, (sy_call_t *)sys_nosys },			/* 78 = getdents */
	{ 0, (sy_call_t *)sys_nosys },			/* 79 = getcwd */
	{ 0, (sy_call_t *)sys_nosys },			/* 80 = chdir */
	{ 0, (sy_call_t *)sys_nosys },			/* 81 = fchdir */
	{ 0, (sy_call_t *)sys_nosys },			/* 82 = rename */
	{ 0, (sy_call_t *)sys_nosys },			/* 83 = mkdir */
	{ 0, (sy_call_t *)sys_nosys },			/* 84 = rmdir */
	{ 0, (sy_call_t *)sys_nosys },			/* 85 = creat */
	{ 0, (sy_call_t *)sys_nosys },			/* 86 = link */
	{ 0, (sy_call_t *)sys_nosys },			/* 87 = unlink */
	{ 0, (sy_call_t *)sys_nosys },			/* 88 = symlink */
	{ 0, (sy_call_t *)sys_nosys },			/* 89 = readlink */
	{ 0, (sy_call_t *)sys_nosys },			/* 90 = chmod */
	{ 0, (sy_call_t *)sys_nosys },			/* 91 = fchmod */
	{ 0, (sy_call_t *)sys_nosys },			/* 92 = chown */
	{ 0, (sy_call_t *)sys_nosys },			/* 93 = fchown */
	{ 0, (sy_call_t *)sys_nosys },			/* 94 = lchown */
	{ 0, (sy_call_t *)sys_nosys },			/* 95 = umask */
	{ 0, (sy_call_t *)sys_nosys },			/* 96 = gettimeofday */
	{ 0, (sy_call_t *)sys_nosys },			/* 97 = getrlimit */
	{ 0, (sy_call_t *)sys_nosys },			/* 98 = getrusage */
	{ 0, (sy_call_t *)sys_nosys },			/* 99 = sysinfo */
	{ 0, (sy_call_t *)sys_nosys },			/* 100 = times */
	{ 0, (sy_call_t *)sys_nosys },			/* 101 = ptrace */
	{ 0, (sy_call_t *)sys_nosys },			/* 102 = getuid */
	{ 0, (sy_call_t *)sys_nosys },			/* 103 = syslog */
	{ 0, (sy_call_t *)sys_nosys },			/* 104 = getgid */
	{ 0, (sy_call_t *)sys_nosys },			/* 105 = setuid */
	{ 0, (sy_call_t *)sys_nosys },			/* 106 = setgid */
	{ 0, (sy_call_t *)sys_nosys },			/* 107 = geteuid */
	{ 0, (sy_call_t *)sys_nosys },			/* 108 = getegid */
	{ 0, (sy_call_t *)sys_nosys },			/* 109 = setpgid */
	{ 0, (sy_call_t *)sys_nosys },			/* 110 = getppid */
	{ 0, (sy_call_t *)sys_nosys },			/* 111 = getpgrp */
	{ 0, (sy_call_t *)sys_nosys },			/* 112 = setsid */
	{ 0, (sy_call_t *)sys_nosys },			/* 113 = setreuid */
	{ 0, (sy_call_t *)sys_nosys },			/* 114 = setregid */
	{ 0, (sy_call_t *)sys_nosys },			/* 115 = getgroups */
	{ 0, (sy_call_t *)sys_nosys },			/* 116 = setgroups */
	{ 0, (sy_call_t *)sys_nosys },			/* 117 = setresuid */
	{ 0, (sy_call_t *)sys_nosys },			/* 118 = getresuid */
	{ 0, (sy_call_t *)sys_nosys },			/* 119 = setresgid */
	{ 0, (sy_call_t *)sys_nosys },			/* 120 = getresgid */
	{ 0, (sy_call_t *)sys_nosys },			/* 121 = getpgid */
	{ 0, (sy_call_t *)sys_nosys },			/* 122 = setfsuid */
	{ 0, (sy_call_t *)sys_nosys },			/* 123 = setfsgid */
	{ 0, (sy_call_t *)sys_nosys },			/* 124 = getsid */
	{ 0, (sy_call_t *)sys_nosys },			/* 125 = capget */
	{ 0, (sy_call_t *)sys_nosys },			/* 126 = capset */
	{ 0, (sy_call_t *)sys_nosys },			/* 127 = rt_sigpending */
	{ 0, (sy_call_t *)sys_nosys },			/* 128 = rt_sigptimedwait */
	{ 0, (sy_call_t *)sys_nosys },			/* 129 = rt_sigqueueinfo */
	{ AS(linux_rt_sigsuspend_args), (sy_call_t *)sys_linux_rt_sigsuspend },	/* 130 = linux_rt_sigsuspend */
	{ 0, (sy_call_t *)sys_nosys },			/* 131 = sigaltstack */
	{ 0, (sy_call_t *)sys_nosys },			/* 132 = utime */
	{ 0, (sy_call_t *)sys_nosys },			/* 133 = mknod */
	{ 0, (sy_call_t *)sys_nosys },			/* 134 = uselib */
	{ 0, (sy_call_t *)sys_nosys },			/* 135 = personality */
	{ 0, (sy_call_t *)sys_nosys },			/* 136 = ustat */
	{ 0, (sy_call_t *)sys_nosys },			/* 137 = statfs */
	{ 0, (sy_call_t *)sys_nosys },			/* 138 = fstatfs */
	{ 0, (sy_call_t *)sys_nosys },			/* 139 = sysfs */
	{ 0, (sy_call_t *)sys_nosys },			/* 140 = getpriority */
	{ 0, (sy_call_t *)sys_nosys },			/* 141 = setpriority */
	{ 0, (sy_call_t *)sys_nosys },			/* 142 = sched_setparam */
	{ 0, (sy_call_t *)sys_nosys },			/* 143 = sched_getparam */
	{ 0, (sy_call_t *)sys_nosys },			/* 144 = sched_setscheduler */
	{ 0, (sy_call_t *)sys_nosys },			/* 145 = sched_getscheduler */
	{ 0, (sy_call_t *)sys_nosys },			/* 146 = sched_get_priority_max */
	{ 0, (sy_call_t *)sys_nosys },			/* 147 = sched_get_priority_min */
	{ 0, (sy_call_t *)sys_nosys },			/* 148 = sched_rr_get_interval */
	{ 0, (sy_call_t *)sys_nosys },			/* 149 = mlock */
	{ 0, (sy_call_t *)sys_nosys },			/* 150 = munlock */
	{ 0, (sy_call_t *)sys_nosys },			/* 151 = mlockall */
	{ 0, (sy_call_t *)sys_nosys },			/* 152 = munlockall */
	{ 0, (sy_call_t *)sys_nosys },			/* 153 = vhangup */
	{ 0, (sy_call_t *)sys_nosys },			/* 154 = modify_ldt */
	{ 0, (sy_call_t *)sys_nosys },			/* 155 = pivot_root */
	{ 0, (sy_call_t *)sys_nosys },			/* 156 = sysctl */
	{ 0, (sy_call_t *)sys_nosys },			/* 157 = prctl */
	{ 0, (sy_call_t *)sys_nosys },			/* 158 = arch_prctl */
	{ 0, (sy_call_t *)sys_nosys },			/* 159 = adjtimex */
	{ 0, (sy_call_t *)sys_nosys },			/* 160 = setrlimit */
	{ 0, (sy_call_t *)sys_nosys },			/* 161 = chroot */
	{ 0, (sy_call_t *)sys_nosys },			/* 162 = sync */
	{ 0, (sy_call_t *)sys_nosys },			/* 163 = acct */
	{ 0, (sy_call_t *)sys_nosys },			/* 164 = settimeofday */
	{ 0, (sy_call_t *)sys_nosys },			/* 165 = mount */
	{ 0, (sy_call_t *)sys_nosys },			/* 166 = umount2 */
	{ 0, (sy_call_t *)sys_nosys },			/* 167 = swapon */
	{ 0, (sy_call_t *)sys_nosys },			/* 168 = swapoff */
	{ 0, (sy_call_t *)sys_nosys },			/* 169 = reboot */
	{ 0, (sy_call_t *)sys_nosys },			/* 170 = sethostname */
	{ 0, (sy_call_t *)sys_nosys },			/* 171 = setdomainname */
	{ 0, (sy_call_t *)sys_nosys },			/* 172 = iopl */
	{ 0, (sy_call_t *)sys_nosys },			/* 173 = ioperm */
	{ 0, (sy_call_t *)sys_nosys },			/* 174 = create_module */
	{ 0, (sy_call_t *)sys_nosys },			/* 175 = init_module */
	{ 0, (sy_call_t *)sys_nosys },			/* 176 = delete_module */
	{ 0, (sy_call_t *)sys_nosys },			/* 177 = get_kernel_syms */
	{ 0, (sy_call_t *)sys_nosys },			/* 178 = query_module */
	{ 0, (sy_call_t *)sys_nosys },			/* 179 = quotactl */
	{ 0, (sy_call_t *)sys_nosys },			/* 180 = nfsservctl */
	{ 0, (sy_call_t *)sys_nosys },			/* 181 = getpmsg */
	{ 0, (sy_call_t *)sys_nosys },			/* 182 = putpmsg */
	{ 0, (sy_call_t *)sys_nosys },			/* 183 = afs_syscall */
	{ 0, (sy_call_t *)sys_nosys },			/* 184 = tuxcall */
	{ 0, (sy_call_t *)sys_nosys },			/* 185 = security */
	{ 0, (sy_call_t *)sys_nosys },			/* 186 = gettid */
	{ 0, (sy_call_t *)sys_nosys },			/* 187 = linux_readahead */
	{ 0, (sy_call_t *)sys_nosys },			/* 188 = setxattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 189 = lsetxattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 190 = fsetxattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 191 = getxattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 192 = lgetxattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 193 = fgetxattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 194 = listxattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 195 = llistxattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 196 = flistxattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 197 = removexattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 198 = lremovexattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 199 = fremovexattr */
	{ AS(linux_tkill_args), (sy_call_t *)sys_linux_tkill },	/* 200 = linux_tkill */
	{ 0, (sy_call_t *)sys_nosys },			/* 201 = time */
	{ 0, (sy_call_t *)sys_nosys },			/* 202 = futex */
	{ 0, (sy_call_t *)sys_nosys },			/* 203 = sched_setaffinity */
	{ 0, (sy_call_t *)sys_nosys },			/* 204 = sched_getaffinity */
	{ 0, (sy_call_t *)sys_nosys },			/* 205 = set_thread_area */
	{ 0, (sy_call_t *)sys_nosys },			/* 206 = io_setup */
	{ 0, (sy_call_t *)sys_nosys },			/* 207 = io_destroy */
	{ 0, (sy_call_t *)sys_nosys },			/* 208 = io_getevents */
	{ 0, (sy_call_t *)sys_nosys },			/* 209 = io_submit */
	{ 0, (sy_call_t *)sys_nosys },			/* 210 = io_cancel */
	{ 0, (sy_call_t *)sys_nosys },			/* 211 = get_thread_area */
	{ 0, (sy_call_t *)sys_nosys },			/* 212 = lookup_dcookie */
	{ 0, (sy_call_t *)sys_nosys },			/* 213 = epoll_create */
	{ 0, (sy_call_t *)sys_nosys },			/* 214 = epoll_ctl_old */
	{ 0, (sy_call_t *)sys_nosys },			/* 215 = epoll_wait_old */
	{ 0, (sy_call_t *)sys_nosys },			/* 216 = remap_file_pages */
	{ 0, (sy_call_t *)sys_nosys },			/* 217 = getdents64 */
	{ 0, (sy_call_t *)sys_nosys },			/* 218 = set_tid_address */
	{ 0, (sy_call_t *)sys_nosys },			/* 219 = restart_syscall */
	{ 0, (sy_call_t *)sys_nosys },			/* 220 = semtimedop */
	{ 0, (sy_call_t *)sys_nosys },			/* 221 = fadvise64 */
	{ 0, (sy_call_t *)sys_nosys },			/* 222 = linux_timer_create */
	{ 0, (sy_call_t *)sys_nosys },			/* 223 = timer_settime */
	{ 0, (sy_call_t *)sys_nosys },			/* 224 = timer_gettime */
	{ 0, (sy_call_t *)sys_nosys },			/* 225 = timer_getoverrun */
	{ 0, (sy_call_t *)sys_nosys },			/* 226 = timer_delete */
	{ 0, (sy_call_t *)sys_nosys },			/* 227 = clock_settime */
	{ 0, (sy_call_t *)sys_nosys },			/* 228 = clock_gettime */
	{ 0, (sy_call_t *)sys_nosys },			/* 229 = clock_getres */
	{ 0, (sy_call_t *)sys_nosys },			/* 230 = clock_nanosleep */
	{ 0, (sy_call_t *)sys_nosys },			/* 231 = exit_group */
	{ 0, (sy_call_t *)sys_nosys },			/* 232 = epoll_wait */
	{ 0, (sy_call_t *)sys_nosys },			/* 233 = epoll_ctl */
	{ AS(linux_tgkill_args), (sy_call_t *)sys_linux_tgkill },	/* 234 = linux_tgkill */
	{ 0, (sy_call_t *)sys_nosys },			/* 235 = utimes */
	{ 0, (sy_call_t *)sys_nosys },			/* 236 = vserver */
	{ 0, (sy_call_t *)sys_nosys },			/* 237 = mbind */
	{ 0, (sy_call_t *)sys_nosys },			/* 238 = set_mempolicy */
	{ 0, (sy_call_t *)sys_nosys },			/* 239 = get_mempolicy */
	{ 0, (sy_call_t *)sys_nosys },			/* 240 = mq_open */
	{ 0, (sy_call_t *)sys_nosys },			/* 241 = mq_unlink */
	{ 0, (sy_call_t *)sys_nosys },			/* 242 = mq_timedsend */
	{ 0, (sy_call_t *)sys_nosys },			/* 243 = mq_timedreceive */
	{ 0, (sy_call_t *)sys_nosys },			/* 244 = mq_notify */
	{ 0, (sy_call_t *)sys_nosys },			/* 245 = mq_getsetattr */
	{ 0, (sy_call_t *)sys_nosys },			/* 246 = kexec_load */
	{ 0, (sy_call_t *)sys_nosys },			/* 247 = waitid */
	{ 0, (sy_call_t *)sys_nosys },			/* 248 = add_key */
	{ 0, (sy_call_t *)sys_nosys },			/* 249 = request_key */
	{ 0, (sy_call_t *)sys_nosys },			/* 250 = keyctl */
	{ 0, (sy_call_t *)sys_nosys },			/* 251 = ioprio_set */
	{ 0, (sy_call_t *)sys_nosys },			/* 252 = ioprio_get */
	{ 0, (sy_call_t *)sys_nosys },			/* 253 = inotify_init */
	{ 0, (sy_call_t *)sys_nosys },			/* 254 = inotify_add_watch */
	{ 0, (sy_call_t *)sys_nosys },			/* 255 = inotify_rm_watch */
	{ 0, (sy_call_t *)sys_nosys },			/* 256 = migrate_pages */
	{ 0, (sy_call_t *)sys_nosys },			/* 257 = openat */
	{ 0, (sy_call_t *)sys_nosys },			/* 258 = mkdirat */
	{ 0, (sy_call_t *)sys_nosys },			/* 259 = mknodat */
	{ 0, (sy_call_t *)sys_nosys },			/* 260 = fchownat */
	{ 0, (sy_call_t *)sys_nosys },			/* 261 = futimesat */
	{ 0, (sy_call_t *)sys_nosys },			/* 262 = newfstatat */
	{ 0, (sy_call_t *)sys_nosys },			/* 263 = unlinkat */
	{ 0, (sy_call_t *)sys_nosys },			/* 264 = renameat */
	{ 0, (sy_call_t *)sys_nosys },			/* 265 = linkat */
	{ 0, (sy_call_t *)sys_nosys },			/* 266 = symlinkat */
	{ 0, (sy_call_t *)sys_nosys },			/* 267 = readlinkat */
	{ 0, (sy_call_t *)sys_nosys },			/* 268 = fchmodat */
	{ 0, (sy_call_t *)sys_nosys },			/* 269 = faccessat */
	{ 0, (sy_call_t *)sys_nosys },			/* 270 = pselect6 */
	{ 0, (sy_call_t *)sys_nosys },			/* 271 = ppoll */
	{ 0, (sy_call_t *)sys_nosys },			/* 272 = unshare */
	{ 0, (sy_call_t *)sys_nosys },			/* 273 = set_robust_list */
	{ 0, (sy_call_t *)sys_nosys },			/* 274 = get_robust_list */
	{ 0, (sy_call_t *)sys_nosys },			/* 275 = splice */
	{ 0, (sy_call_t *)sys_nosys },			/* 276 = tee */
	{ 0, (sy_call_t *)sys_nosys },			/* 277 = sync_file_range */
	{ 0, (sy_call_t *)sys_nosys },			/* 278 = vmsplice */
	{ 0, (sy_call_t *)sys_nosys },			/* 279 = move_pages */
	{ 0, (sy_call_t *)sys_nosys },			/* 280 = utimensat */
	{ 0, (sy_call_t *)sys_nosys },			/* 281 = epoll_pwait */
	{ 0, (sy_call_t *)sys_nosys },			/* 282 = signalfd */
	{ 0, (sy_call_t *)sys_nosys },			/* 283 = timerfd_create */
	{ 0, (sy_call_t *)sys_nosys },			/* 284 = eventfd */
	{ 0, (sy_call_t *)sys_nosys },			/* 285 = fallocate */
	{ 0, (sy_call_t *)sys_nosys },			/* 286 = timerfd_settime */
	{ 0, (sy_call_t *)sys_nosys },			/* 287 = timerfd_gettime */
	{ 0, (sy_call_t *)sys_nosys },			/* 288 = accept4 */
	{ 0, (sy_call_t *)sys_nosys },			/* 289 = signalfd4 */
	{ 0, (sy_call_t *)sys_nosys },			/* 290 = eventfd2 */
	{ 0, (sy_call_t *)sys_nosys },			/* 291 = epoll_create1 */
	{ 0, (sy_call_t *)sys_nosys },			/* 292 = dup3 */
	{ 0, (sy_call_t *)sys_nosys },			/* 293 = pipe2 */
	{ 0, (sy_call_t *)sys_nosys },			/* 294 = inotify_init1 */
	{ 0, (sy_call_t *)sys_nosys },			/* 295 = linux_preadv */
	{ 0, (sy_call_t *)sys_nosys },			/* 296 = linux_pwritev */
	{ 0, (sy_call_t *)sys_nosys },			/* 297 = rt_tgsigqueueinfo */
	{ 0, (sy_call_t *)sys_nosys },			/* 298 = perf_event_open */
	{ 0, (sy_call_t *)sys_nosys },			/* 299 = recvmmsg */
	{ 0, (sy_call_t *)sys_nosys },			/* 300 = fanotify_init */
	{ 0, (sy_call_t *)sys_nosys },			/* 301 = fanotify_mark */
	{ 0, (sy_call_t *)sys_nosys },			/* 302 = prlimit64 */
	{ 0, (sy_call_t *)sys_nosys },			/* 303 = name_to_handle_at */
	{ 0, (sy_call_t *)sys_nosys },			/* 304 = open_by_handle_at */
	{ 0, (sy_call_t *)sys_nosys },			/* 305 = clock_adjtime */
	{ 0, (sy_call_t *)sys_nosys },			/* 306 = syncfs */
	{ 0, (sy_call_t *)sys_nosys },			/* 307 = sendmmsg */
	{ 0, (sy_call_t *)sys_nosys },			/* 308 = setns */
	{ 0, (sy_call_t *)sys_nosys },			/* 309 = getcpu */
	{ 0, (sy_call_t *)sys_nosys },			/* 310 = process_vm_readv */
	{ 0, (sy_call_t *)sys_nosys },			/* 311 = process_vm_writev */
};
