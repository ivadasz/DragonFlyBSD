load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_lib(
    name = "chacha",
    srcs = ["chacha.c", "chacha.h"],
    deps = ["//sys/sys"],
    hdrs = ["chacha.h"],
    include_prefix = "crypto/chacha",
    visibility = ["//visibility:public"],
)
