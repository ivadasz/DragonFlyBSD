load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "libprop_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "libprop",
    hdrs = [":libprop_headers"],
    include_prefix = "libprop",
    visibility = ["//visibility:public"],
)
