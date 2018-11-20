load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_headers(
    name = "acpica_sdt",
    hdrs = ["acpi_sdt_var.h"],
    include_prefix = "platform/pc64/acpica",
    visibility = ["//sys/dev/acpica:__pkg__"],
)

dfly_kernel_headers(
    name = "acpica_cpu",
    hdrs = ["acpi_md_cpu.h"],
    include_prefix = "machine_base/acpica",
    visibility = ["//sys/platform/pc64/x86_64:__pkg__"],
)

dfly_kernel_lib(
    name = "acpica_obj",
    srcs = glob(["*.c"], exclude = [
        "acpi_cstate_machdep.c",
        "acpi_pstate_machdep.c",
    ]) + ["acpi_sdt_var.h"],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/kern:core_ifs",
        "//sys/platform/pc64/apic",
        "//sys/platform/pc64/icu",
        "//sys/platform/pc64/isa",
        "//sys/platform/pc64/include:pc",
        "//sys/vm",
        "//sys/dev/acpica:acpi_machine",
        "//sys/dev/acpica:acpi_priv",
        "//sys/dev/acpica",
        "//sys/contrib/dev/acpica/source/include:acpica",
        "//sys/contrib/dev/acpica/source/include:acpica_priv",
        "//sys/contrib/dev/acpica/source/include:acpica_platform",
        ":acpica_cpu",
    ],
    visibility = ["//visibility:public"],
)
