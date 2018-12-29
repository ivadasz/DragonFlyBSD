load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kernel_object")
load("//sys:build_defs.bzl", "dfly_kobj_code")
load("//sys:build_defs.bzl", "dfly_kobj_hdr")

exports_files([
    "genassym.sh",
])

dfly_kobj_hdr(name = "device_if_hdr", kobj = "device_if.m")
dfly_kobj_hdr(name = "bus_if_hdr", kobj = "bus_if.m")
dfly_kobj_hdr(name = "cpu_if_hdr", kobj = "cpu_if.m")
dfly_kernel_headers(
    name = "core_ifs",
    hdrs = [
        ":device_if_hdr",
        ":bus_if_hdr",
        #":cpu_if_hdr",
    ],
    visibility = ["//visibility:public"],
)

dfly_kobj_code(name = "device_if_src", kobj = "device_if.m")
dfly_kobj_code(name = "bus_if_src", kobj = "bus_if.m")
dfly_kobj_code(name = "cpu_if_src", kobj = "cpu_if.m")
filegroup(
    name = "core_ifs_srcs",
    srcs = [
        ":device_if_src",
        ":bus_if_src",
        #":cpu_if_src",
    ],
)

dfly_kernel_lib(
    name = "ioconf",
    srcs = ["ioconf.c"],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        ":core_ifs",
        "//sys/bus/isa",
    ],
)

always_excludes = [
    "subr_input.c",
    "kern_sched.c",
    "uipc_accf.c",
    "kern_ktr.c",
    "imgact_gzip.c",
    "inflate.c",

    # built separately
    "ioconf.c",

    # Not used by core kernel parts
    "subr_sglist.c",

    # Not used by core kernel parts
    "subr_cpuhelper.c",

    # Optional cpu topology support
    "subr_cpu_topology.c",

    # Only include when dynamic linking is enabled
    "link_elf.c",
    "link_elf_obj.c",

    # Firmware handling
    "subr_firmware.c",

    # pty driver
    "tty_pty.c",

    # watchdog support
    "kern_wdog.c",

    # kcollect
    "kern_collect.c",

    # sensors
    "kern_sensors.c",

    # /dev/{null,mem,kmem,io,random,urandom,upmap,kpmap,...}
    "kern_memio.c",

    # /dev/klog support
    "subr_log.c",
]

rump_excludes = always_excludes + [
    "imgact_aout.c",
    "imgact_elf.c",
    "imgact_shell.c",
    "imgact_resident.c",
    "init_sysent.c",
    "kern_acct.c",
    "kern_checkpoint.c",
    "kern_exec.c",
    "kern_exit.c",
    "kern_fork.c",
    "kern_jail.c",
    "kern_kinfo.c",
    "kern_linker.c",
    "kern_module.c",
    "kern_plimit.c",
    "kern_proc.c",
    "kern_prot.c",
    "kern_resource.c",
    "kern_sfbuf.c",
    "kern_sig.c",
    "kern_umtx.c",
    "kern_usched.c",
    "kern_xio.c",
    "subr_alist.c",
    "subr_busdma.c",
    "subr_module.c",
    "sysv_ipc.c",
    "sysv_msg.c",
    "sysv_sem.c",
    "sysv_shm.c",
    "sys_process.c",
    "sys_vmm.c",
    "usched_bsd4.c",
    "usched_dfly.c",
    "usched_dummy.c",
    "vfs_aio.c",
    "vfs_cluster.c",
    "vfs_lock.c",
    "vfs_mount.c",
    "vfs_vm.c",
]

dfly_kernel_lib(
    name = "kern",
    # TODO(ivadasz): kern_sched.c depends on _kposix_priority_scheduling feature
    srcs = select({
        "//sys:metal": glob(["*.c"], exclude = always_excludes) + [
            ":core_ifs_srcs",
        ],
        "//sys:userspace": glob(["*.c"], exclude = rump_excludes) + [
            ":core_ifs_srcs",
        ]
    }),
    deps = select({
        "//sys:metal": [
            "//sys/sys:options",
            "//sys/sys",
            "//sys/ddb",
            "//sys/vm",
            "//sys/crypto/sha2",
            "//sys/crypto/chacha",
            "//sys/net",
            "//sys/netinet",
            "//sys/netinet6",
            "//sys/net/altq:net_altq",
            # kobj headers
            ":core_ifs",
            # very specific includes
            "//sys/net:radix",
            "//sys/vfs/ufs:dinode",
            "//sys/vfs/ufs:fs",
            "//sys/vfs/procfs",
            "//crypto/libressl/include/openssl:md5",
            "//sys/dev/acpica/acpi_pvpanic",

            # ioconf
            ":ioconf",
        ],
        "//sys:userspace": [
            "//sys/sys:options",
            "//sys/sys",
            "//sys/ddb",
            "//sys/platform/rump/vm",
            "//sys/crypto/sha2",
            "//sys/crypto/chacha",
            "//sys/net",
            "//sys/netinet",
            "//sys/netinet6",
            "//sys/net/altq:net_altq",
            # kobj headers
            ":core_ifs",
            # very specific includes
            "//sys/net:radix",
            "//sys/vfs/ufs:dinode",
            "//sys/vfs/ufs:fs",
            "//sys/vfs/procfs",
            "//crypto/libressl/include/openssl:md5",

            # ioconf
            ":ioconf",
        ]
    }),
    visibility = ["//visibility:public"],
)
