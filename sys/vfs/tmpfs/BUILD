load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_lib(
    name = "tmpfs",
    srcs = glob(["*.c"]) + [
        "tmpfs.h",
    ],
    deps = [
        "//sys/sys",
        "//sys/vm",
        "//sys/net:radix",
        "//sys/vfs/fifofs",
    ],
    hdrs = [
        "tmpfs.h",
        "tmpfs_mount.h",
        "tmpfs_vnops.h",
    ],
    include_prefix = "vfs/tmpfs",
    visibility = ["//visibility:public"],
)
