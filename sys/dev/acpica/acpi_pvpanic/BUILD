load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "pvpanic_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "acpi_pvpanic",
    hdrs = [":pvpanic_headers"],
    include_prefix = "dev/acpica/acpi_pvpanic",
    visibility = ["//visibility:public"],
)
