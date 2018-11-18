load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_lib(
    name = "devfs",
    srcs = glob(["*.c"]),
    deps = [
        "//sys/sys",
        "//sys/kern:core_ifs",
        "//sys/net:radix",
        "//sys/vm",
        "//sys/net",
        "//sys/vfs/fifofs",
    ],
    visibility = ["//visibility:public"],
)
