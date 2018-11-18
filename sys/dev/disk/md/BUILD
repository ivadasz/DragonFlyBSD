load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_lib(
    name = "md",
    srcs = glob(["*.c"]),
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/vm",
        "//sys/net:radix",
    ],
    visibility = ["//visibility:public"],
)
