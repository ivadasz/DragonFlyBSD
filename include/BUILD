load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "std_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "std",
    hdrs = [":std_headers"],
    visibility = ["//visibility:public"],
)
