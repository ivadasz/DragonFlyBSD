load("//sys:build_defs.bzl", "dfly_kernel_driver")

dfly_kernel_driver(
    name = "sio",
    srcs = [
        "sio.c",
        "sioreg.h",
        "sio_private.h",
    ],
    deps = [
        "//sys/ddb",
        "//sys/bus/isa",
        "//sys/bus/pci",
        "//sys/dev/serial/ic_layer",
    ],
    visibility = ["//visibility:public"],
)
