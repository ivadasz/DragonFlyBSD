load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")

filegroup(
    name = "private_incs",
    srcs = [
        "qsort.c",
    ],
)

dfly_kernel_headers(
    name = "private",
    hdrs = [":private_incs"],
    include_prefix = "libkern",
)

dfly_kernel_lib(
    name = "libkern",
    srcs = glob(["*.c"], exclude = [
        "ffsl.c", "flsl.c", "flsll.c",
        "linux_idr.c",
        "mcount.c",
    ]),
    deps = [
        "//sys/sys",
        "//sys/vm",
        "//include/xlocale",
        "//sys/netinet",
        "//sys/netinet6",
        ":private",
    ],
    visibility = ["//visibility:public"],
)
