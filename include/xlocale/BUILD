load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "xlocale_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "xlocale",
    hdrs = [":xlocale_headers"],
    include_prefix = "xlocale",
    visibility = ["//visibility:public"],
)
