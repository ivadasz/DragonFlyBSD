load("//sys:build_defs.bzl", "dfly_kernel_lib")

filegroup(
    name = "vm_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_lib(
    name = "vm",
    srcs = ["vm.c"],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
    ],
    hdrs = [":vm_headers"],
    include_prefix = "vm",
    visibility = ["//visibility:public"],
)
