load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_opt_header")
load("//sys:build_defs.bzl", "dfly_use_header")
load("//sys/sys:opt_headers.bzl", "dfly_opt_dummies")
load("//sys/sys:opt_headers.bzl", "dfly_use_dummies")

filegroup(
    name = "sys_headers",
    srcs = glob(["*.h"], exclude = ["idr.h"]),
)

dfly_kernel_headers(
    name = "sys",
    hdrs = [":sys_headers"],
    include_prefix = "sys",
    deps = [
        "//sys/libprop",
        "//include:std",
        ":platform",
    ],
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "machine_cpu_core",
    # TODO(ivadasz): Do a select on the cpu architecture
    hdrs = ["//sys/cpu/x86_64/include:cpu_headers_core"],
    include_prefix = "cpu",
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "machine_cpu",
    # TODO(ivadasz): Do a select on the cpu architecture
    hdrs = ["//sys/cpu/x86_64/include:cpu_headers"],
    deps = [":machine_cpu_core"],
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
        "//sys/platform/pc64/apic:apicreg",
        ":machine_cpu_core",
    ],
    visibility = ["//visibility:public"],
)


### Deal with the opt_* header files, currently just generate stub headers.
opt_dummy_headers = [
    "acpi",
    "auto_eoi",
    "bootp",
    "bus",
    "cam",
    "cd9660",
    "comconsole",
    "ddb",
    "ddb_trace",
    "debug_cluster",
    "debug_lockf",
    "fb",
    "inet",
    "inet6",
    "init_path",
    "ktr",
    "ktrace",
    "lint",
    "mbuf_stress_test",
    "msgbuf",
    "nfs",
    "nfsroot",
    "ntp",
    "quota",
    "panic",
    "param",
    "pctrack",
    "posix",
    "rootdevname",
    "suiddir",
    "swap",
    "syscons",
    "sysvipc",
    "uconsole",
    "vga",
    "vm",
]

dfly_opt_dummies("opt_dummy_targets", opt_dummy_headers)

use_dummy_headers = [
    "gpio",
]
dfly_use_dummies("use_dummy_targets", use_dummy_headers)

dfly_opt_header("maxusers_options", "maxusers", ["'MAXUSERS 64'"])
dfly_opt_header("global_options", "global", ["'INVARIANTS 1'"])
dfly_opt_header("cpu_options", "cpu", ["'HAMMER_CPU 1'"])

dfly_use_header("use_isa", "isa", ["'NISA 1'"])
dfly_use_header("use_pci", "pci", ["'NPCI 1'"])

filegroup(
    name = "use_options",
    srcs = [
        ":use_isa",
        ":use_pci",
    ],
)

dfly_kernel_headers(
    name = "options",
    hdrs = [
        ":opt_dummy_targets",
        ":use_dummy_targets",
        ":maxusers_options",
        ":global_options",
        ":cpu_options",
        ":use_options",
    ],
    visibility = ["//visibility:public"],
)
