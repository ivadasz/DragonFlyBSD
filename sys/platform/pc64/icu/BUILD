load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_headers(
    name = "icu",
    hdrs = ["icu.h", "icu_var.h", "icu_ipl.h", "elcr_var.h"],
    include_prefix = "machine_base/icu",
    visibility = [
        "//sys/platform/pc64/acpica:__pkg__",
        "//sys/platform/pc64/isa:__pkg__",
        "//sys/platform/pc64/apic:__pkg__",
    ]
)

dfly_kernel_lib(
    name = "icu_obj",
    srcs = glob(["*.c"]) + glob(["*.s"]),
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/kern:core_ifs",
        "//sys/sys:machine_cpu",
        "//sys/bus/isa",
        "//sys/platform/pc64/apic",
        "//sys/platform/pc64/icu",
        "//sys/platform/pc64/isa",
        "//sys/platform/pc64/x86_64:assym",
    ],
    visibility = ["//visibility:public"],
)
