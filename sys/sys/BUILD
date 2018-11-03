load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_opt_header")
load("//sys/sys:opt_headers.bzl", "dfly_opt_dummies")

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
    hdrs = ["//sys/cpu/x86_64/include:cpu_headers"],
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
    name = "apic",
    hdrs = ["//sys/platform/pc64:apic_headers"],
    include_prefix = "machine_base/apic",
    visibility = ["//visibility:public"],
)


### Deal with the opt_* header files, currently just generate stub headers.
opt_dummy_headers = [
    "ddb",
    "debug_cluster",
    "global",
    "inet",
    "inet6",
    "ktrace",
    "ntp",
    "quota",
    "pctrack",
    "rootdevname",
    "suiddir",
]

dfly_opt_dummies("opt_dummy_targets", opt_dummy_headers)

dfly_kernel_headers(
    name = "options",
    hdrs = [":opt_dummy_targets"],
    visibility = ["//visibility:public"],
)
