load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kernel_headers")

dfly_kernel_lib(
    name = "mi",
    srcs = [
        "main.c",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
    ],
    visibility = ["//visibility:public"],
)
