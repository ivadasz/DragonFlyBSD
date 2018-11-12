load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_headers(
    name = "isa",
    hdrs = ["isa_intr.h", "timerreg.h"],
    include_prefix = "machine_base/isa",
    visibility = [
        "//sys/platform/pc64/acpica:__pkg__",
        "//sys/platform/pc64/icu:__pkg__",
        "//sys/platform/pc64/apic:__pkg__",
        "//sys/platform/pc64/x86_64:__pkg__",
    ]
)

dfly_kernel_lib(
    name = "isa_obj",
    srcs = [
        "clock.c",
        "isa_intr.c",
        # TODO(ivadasz): pmtimer is a separate driver, that is used with acpi.
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/kern:core_ifs",
        "//sys/bus/isa",
        "//sys/platform/pc64/apic",
        "//sys/platform/pc64/icu",
        ":isa",
    ],
    visibility = ["//visibility:public"],
)
