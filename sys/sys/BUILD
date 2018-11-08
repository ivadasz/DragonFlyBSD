load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_opt_header")
load("//sys/sys:opt_headers.bzl", "dfly_opt_dummies")
load("//sys/sys:opt_headers.bzl", "dfly_use_dummies")

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
    name = "machine_cpu",
    # TODO(ivadasz): Do a select on the cpu architecture
    hdrs = ["//sys/cpu/x86_64/include:cpu_headers"],
    include_prefix = "cpu",
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "platform",
    # TODO(ivadasz): Do a select on the dragonfly platform
    hdrs = [
        "//sys/platform/pc64/include:platform_headers",
        "//sys/cpu/x86_64/include:machine_headers",
    ],
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
    "bus",
    "comconsole",
    "ddb",
    "ddb_trace",
    "debug_cluster",
    "debug_lockf",
    "global",
    "inet",
    "inet6",
    "init_path",
    "ktr",
    "ktrace",
    "lint",
    "mbuf_stress_test",
    "ntp",
    "quota",
    "panic",
    "param",
    "pctrack",
    "posix",
    "rootdevname",
    "suiddir",
    "sysvipc",
    "uconsole",
    "vm",
]

#opt_basic_headers = [
#    XXX opt_global.h should have a MAXUSERS setting
#},

dfly_opt_dummies("opt_dummy_targets", opt_dummy_headers)

use_dummy_headers = [
    "gpio",
]
dfly_use_dummies("use_dummy_targets", use_dummy_headers)

dfly_opt_header("maxusers_options", "maxusers", ["'MAXUSERS 64'"])

dfly_kernel_headers(
    name = "options",
    hdrs = [":opt_dummy_targets", ":use_dummy_targets", ":maxusers_options"],
    visibility = ["//visibility:public"],
)
