load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "sys_headers",
    srcs = glob(["*.h"]),
)

dfly_kernel_headers(
    name = "sys",
    hdrs = [":sys_headers"],
    include_prefix = "sys",
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "machine",
    # TODO(ivadasz): Do a select on the cpu architecture
    hdrs = ["//sys/cpu/x86_64/include:machine_headers"],
    include_prefix = "machine",
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "machine_cpu",
    # TODO(ivadasz): Do a select on the cpu architecture
    hdrs = ["//sys/cpu/x86_64/include:machine_headers"],
    include_prefix = "cpu",
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "platform",
    # TODO(ivadasz): Do a select on the dragonfly platform
    hdrs = ["//sys/platform/pc64/include:platform_headers"],
    include_prefix = "machine",
    deps = [
        ":apic",
    ],
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "platform_cpu",
    # TODO(ivadasz): Do a select on the dragonfly platform
    hdrs = ["//sys/platform/pc64/include:platform_headers"],
    include_prefix = "cpu",
    deps = [
        ":apic",
    ],
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "apic",
    hdrs = ["//sys/platform/pc64:apic_headers"],
    include_prefix = "machine_base/apic",
    visibility = ["//visibility:public"],
)
