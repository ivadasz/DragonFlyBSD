load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "acpica_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "acpica",
    hdrs = [":acpica_headers"],
    include_prefix = "platform/pc64/acpica",
    visibility = ["//sys/dev/acpica:__pkg__"],
)
