load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "net_headers",
    srcs = glob(["*.h"], exclude = ["radix.h"]),
)

dfly_kernel_headers(
    name = "net",
    hdrs = [":net_headers"],
    include_prefix = "net",
    visibility = ["//visibility:public"],
)

filegroup(
    name = "radix_headers",
    srcs = ["radix.h"],
)

dfly_kernel_headers(
    name = "radix",
    hdrs = [":radix_headers"],
    include_prefix = "net",
    visibility = ["//visibility:public"],
)
