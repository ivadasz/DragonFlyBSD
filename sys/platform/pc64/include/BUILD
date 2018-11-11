load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "platform_headers",
    srcs = glob(["*.h"]),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "platform_pc_headers",
    srcs = glob(["pc/*.h"]),
)

dfly_kernel_headers(
    name = "pc",
    hdrs = [":platform_pc_headers"],
    include_prefix = "machine/pc",
    visibility = [
        "//sys/bus/pci/x86_64:__pkg__",
        "//sys/bus/isa:__pkg__",
        "//sys/platform/pc64/acpica:__pkg__",
    ],
)
