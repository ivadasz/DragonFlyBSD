load("//sys:build_defs.bzl", "dfly_kernel_headers")

dfly_kernel_headers(
    name = "nfs",
    hdrs = [
        "nfsv2.h",
        "nfsproto.h",
        "rpcv2.h",
        "nfs.h",
        "nfsdiskless.h",
    ],
    include_prefix = "vfs/nfs",
    visibility = ["//sys/platform/pc64/x86_64:__pkg__"],
)
