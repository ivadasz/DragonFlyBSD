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
    hdrs = select({
        "//sys:metal": ["//sys/cpu/x86_64/include:cpu_headers_core"],
        "//sys:userspace": ["//sys/cpu/x86_64/include:cpu_headers_core"],
    }),
    include_prefix = "cpu",
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "machine_cpu",
    hdrs = select({
        "//sys:metal": ["//sys/cpu/x86_64/include:cpu_headers_core"],
        "//sys:userspace": ["//sys/cpu/x86_64/include:cpu_headers_core"],
    }),
    deps = [":machine_cpu_core"],
    include_prefix = "cpu",
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "platform",
    hdrs = select({
        "//sys:metal": [
            "//sys/platform/pc64/include:platform_headers",
            "//sys/cpu/x86_64/include:machine_headers",
        ],
        "//sys:userspace": [
            "//sys/platform/rump/include:platform_headers",
            "//sys/cpu/x86_64/include:machine_headers",
        ],
    }),
    include_prefix = "machine",
    deps = select({
        "//sys:metal": [
            "//sys/platform/pc64/apic:apicreg",
            ":machine_cpu_core",
        ],
        "//sys:userspace": [
            ":machine_cpu_core",
        ],
    }),
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
    "ddb_trace",
    "debug_cluster",
    "debug_lockf",
    "fb",
    "ffs",
    "ifpoll",
    "inet",
    "inet6",
    "init_path",
    "ktr",
    "ktrace",
    "lint",
    "mbuf_stress_test",
    "mpls",
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
    "sio",
    "suiddir",
    "swap",
    "syscons",
    "sysvipc",
    "uconsole",
    "ufs",
    "vga",
    "vm",
]

dfly_opt_dummies("opt_dummy_targets", opt_dummy_headers)

use_dummy_headers = [
    "gpio",
    "puc",
]
dfly_use_dummies("use_dummy_targets", use_dummy_headers)

# TODO(ivadasz): Implement a macro for definining these.
dfly_opt_header("maxusers_options", "maxusers", ["'MAXUSERS 64'"])
dfly_opt_header("global_options", "global", [
    "'INVARIANTS 1'",
])
dfly_opt_header("cpu_options", "cpu", ["'HAMMER_CPU 1'"])
dfly_opt_header("ddb_options", "ddb", [
    #"'DDB 1'",
])
dfly_opt_header("md_options", "md", [
    "'MD_ROOT 1'",
])
dfly_opt_header("kcollect_options", "kcollect", [
    #"'ENABLE_KCOLLECT 1'",
])
dfly_opt_header("sensors_options", "sensors", [
    #"'ENABLE_SENSORS 1'",
])
dfly_opt_header("constty_options", "constty", [
    #"'ENABLE_CONSTTY 1'",
])
dfly_opt_header("topology_options", "topology", [
    #"'ENABLE_TOPOLOGY 1'",
])
dfly_opt_header("upmap_options", "upmap", [
    #"'ENABLE_UPMAP 1'",
])

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
    hdrs = select({
        "//sys:metal": [
            ":opt_dummy_targets",
            ":use_dummy_targets",
            ":maxusers_options",
            ":global_options",
            ":cpu_options",
            ":use_options",
            ":ddb_options",
            ":md_options",
            ":kcollect_options",
            ":sensors_options",
            ":constty_options",
            ":topology_options",
            ":upmap_options",
        ],
        "//sys:userspace": [
            ":cpu_options",
            ":global_options",
            ":kcollect_options",
            ":maxusers_options",
            ":opt_dummy_targets",
            ":sensors_options",
            ":topology_options",
        ],
    }),
    visibility = ["//visibility:public"],
)
