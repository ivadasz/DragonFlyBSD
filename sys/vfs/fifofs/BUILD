load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_lib(
    name = "fifofs",
    srcs = ["fifo_vnops.c", "fifo.h"],
    deps = [
        "//sys/sys",
        "//sys/net:netisr",
    ],
    hdrs = ["fifo.h"],
    include_prefix = "vfs/fifofs",
    visibility = ["//visibility:public"],
)
