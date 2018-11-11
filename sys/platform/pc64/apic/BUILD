load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_headers(
    name = "apicreg",
    hdrs = [
        "apicreg.h",
        "lapic.h",
    ],
    include_prefix = "machine_base/apic",
    visibility = [
        "//sys/sys:__pkg__",
    ],
)

dfly_kernel_headers(
    name = "apic",
    hdrs = [
        "apicvar.h",
        "ioapic_abi.h",
        "ioapic_ipl.h",
        "ioapic.h",
    ],
    include_prefix = "machine_base/apic",
    deps = [":apicreg"],
    visibility = [
        "//sys/platform/pc64/acpica:__pkg__",
        "//sys/platform/pc64/icu:__pkg__",
        "//sys/platform/pc64/isa:__pkg__",
        "//sys/platform/pc64/x86_64:__pkg__",
        "//sys/bus/pci:__pkg__",
        "//sys/bus/pci/x86_64:__pkg__",
    ],
)

dfly_kernel_lib(
    name = "apic_obj",
    srcs = glob(["*.c"]) + glob(["*.s"]) + glob(["*.h"]),
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/sys:machine_cpu",
        "//sys/kern:core_ifs",
        "//sys/bus/isa",
        "//sys/dev/acpica",
        "//sys/platform/pc64/icu",
        "//sys/platform/pc64/isa",
        "//sys/platform/pc64/x86_64:assym",
        ":apic",
    ],
    visibility = ["//visibility:public"],
)
