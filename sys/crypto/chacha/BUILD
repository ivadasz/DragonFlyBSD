load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "chacha_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "chacha",
    hdrs = [":chacha_headers"],
    include_prefix = "crypto/chacha",
    visibility = ["//visibility:public"],
)
