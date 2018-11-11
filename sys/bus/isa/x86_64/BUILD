load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kernel_headers")

dfly_kernel_headers(
    name = "priv",
    hdrs = ["isa.h"],
    include_prefix = "x86_64",
    visibility = ["//sys/bus/isa:__pkg__"],
)

dfly_kernel_lib(
    name = "x86_64",
    srcs = glob(["*.c"]) + ["isa.h"],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/kern:core_ifs",
        "//sys/bus/isa",
        "//sys/platform/pc64/isa/ic",
        "//sys/vm",
    ],
    visibility = ["//sys/bus/isa:__pkg__"],
)
