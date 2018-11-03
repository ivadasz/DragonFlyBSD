load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kernel_object")

dfly_kernel_object(
    name = "kern_rbtree",
    srcs = ["subr_rbtree.c"],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//include:std",
        "//sys/sys:machine",
        "//sys/sys:machine_cpu",
        "//sys/sys:platform",
        "//sys/vm",
    ],
    visibility = ["//visibility:public"],
)

dfly_kernel_lib(
    name = "kern_clock",
    srcs = [
        "kern_clock.c",
        "kern_time.c",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//include:std",
        "//sys/sys:machine",
        "//sys/sys:machine_cpu",
        "//sys/sys:platform",
        "//sys/vm",
    ],
)

dfly_kernel_lib(
    name = "kern_vfs",
    srcs = [
        "vfs_aio.c",
        "vfs_bio.c",
        "vfs_cache.c",
        "vfs_cluster.c",
        "vfs_conf.c",
        "vfs_default.c",
        "vfs_helper.c",
        "vfs_init.c",
        "vfs_jops.c",
        "vfs_journal.c",
        "vfs_lock.c",
        "vfs_lookup.c",
        "vfs_mount.c",
        "vfs_nlookup.c",
        "vfs_quota.c",
        "vfs_subr.c",
        "vfs_sync.c",
        "vfs_synth.c",
        "vfs_syscalls.c",
        "vfs_vfsops.c",
        "vfs_vm.c",
        "vfs_vnops.c",
        "vfs_vopops.c",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//include:std",
        "//sys/sys:machine",
        "//sys/sys:machine_cpu",
        "//sys/sys:platform",
        "//sys/ddb",
        "//sys/vm",
        "//sys/libprop",
        "//sys/net",
        "//sys/net:radix",
        "//sys/netinet",
        "//sys/netinet6",
    ],
)
