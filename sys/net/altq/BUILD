load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "net_altq_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "net_altq",
    hdrs = [":net_altq_headers"],
    include_prefix = "net/altq",
    visibility = ["//visibility:public"],
)
