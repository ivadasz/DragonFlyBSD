load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "md5_headers",
    srcs = glob(["md5.h"]),
)

dfly_kernel_headers(
    name = "md5",
    hdrs = [":md5_headers"],
    include_prefix = "openssl",
    visibility = ["//visibility:public"],
)
