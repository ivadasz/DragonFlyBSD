load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_lib(
    name = "deadfs",
    srcs = ["dead_vnops.c"],
    deps = [
        "//sys/sys",
    ],
    visibility = ["//visibility:public"],
)
