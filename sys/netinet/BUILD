load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "netinet_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "netinet",
    hdrs = [":netinet_headers"],
    include_prefix = "netinet",
    visibility = ["//visibility:public"],
)
