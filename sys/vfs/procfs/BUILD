load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "procfs_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "procfs",
    hdrs = [":procfs_headers"],
    include_prefix = "vfs/procfs",
    visibility = ["//visibility:public"],
)
