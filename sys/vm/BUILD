load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "vm_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "vm",
    hdrs = [":vm_headers"],
    include_prefix = "vm",
    visibility = ["//visibility:public"],
)
