load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")

filegroup(
    name = "procfs_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "procfs",
    hdrs = [":procfs_headers"],
    include_prefix = "vfs/procfs",
    visibility = ["//visibility:public"],
)

dfly_kernel_lib(
    name = "core",
    srcs = [
        "procfs_mem.c",
        "procfs_regs.c",
        "procfs_dbregs.c",
        "procfs_fpregs.c",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/vm",
        ":procfs",
    ],
    visibility = ["//visibility:public"],
)
