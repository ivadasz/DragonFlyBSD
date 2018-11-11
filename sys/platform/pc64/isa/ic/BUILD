load("//sys:build_defs.bzl", "dfly_kernel_headers")

dfly_kernel_headers(
    name = "ic",
    hdrs = ["i8237.h"],
    include_prefix = "machine_base/isa/ic",
    visibility = [
        "//sys/bus/isa/x86_64:__pkg__",
    ]
)
