load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "sha2_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "sha2",
    hdrs = [":sha2_headers"],
    include_prefix = "crypto/sha2",
    visibility = ["//visibility:public"],
)
