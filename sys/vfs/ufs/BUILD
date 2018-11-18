load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "dinode_hdr",
    srcs = glob(["dinode.h", "ufs_types.h"]),
)

dfly_kernel_headers(
    name = "dinode",
    hdrs = [":dinode_hdr"],
    include_prefix = "vfs/ufs",
    visibility = ["//visibility:public"],
)

filegroup(
    name = "fs_hdr",
    srcs = glob(["fs.h"]),
)

dfly_kernel_headers(
    name = "fs",
    hdrs = [":fs_hdr"],
    include_prefix = "vfs/ufs",
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "readwrite_c",
    hdrs = ["ufs_readwrite.c"],
)

dfly_kernel_lib(
    name = "ufs",
    srcs = glob(["*.c"], exclude = [
        "ffs_softdep.c",
        "ufs_readwrite.c",
    ]) + glob(["*.h"]),
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/vm",
        "//sys/net",
        "//sys/vfs/fifofs",
        ":readwrite_c",
    ],
    visibility = ["//visibility:public"],
)
