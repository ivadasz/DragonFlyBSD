load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")

filegroup(
    name = "ip_mroute_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "ip_mroute",
    hdrs = [":ip_mroute_headers"],
    include_prefix = "net/ip_mroute",
    visibility = ["//visibility:public"],
)

dfly_kernel_lib(
    name = "ip_mroute_obj",
    srcs = [
        "ip_mroute.c",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        ":ip_mroute",
    ],
    visibility = ["//visibility:public"],
)
