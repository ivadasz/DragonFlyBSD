load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_lib(
    name = "mi",
    srcs = [
        "copyio.c",
        "main.c",
        "systimer.c",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
    ],
    visibility = ["//visibility:public"],
)
