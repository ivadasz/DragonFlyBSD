load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "netinet6_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "netinet6",
    hdrs = [":netinet6_headers"],
    include_prefix = "netinet6",
    visibility = ["//visibility:public"],
)
