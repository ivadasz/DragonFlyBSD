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
