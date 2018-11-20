load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kobj_code")
load("//sys:build_defs.bzl", "dfly_kobj_hdr")

dfly_kobj_hdr(name = "pci_if_hdr", kobj = "pci_if.m")
dfly_kobj_hdr(name = "pcib_if_hdr", kobj = "pcib_if.m")
dfly_kernel_headers(
    name = "ifs",
    hdrs = [
        ":pci_if_hdr",
        ":pcib_if_hdr",
    ],
    visibility = ["//visibility:public"],
)

dfly_kobj_code(name = "pci_if_src", kobj = "pci_if.m")
dfly_kobj_code(name = "pcib_if_src", kobj = "pcib_if.m")

filegroup(
    name = "pci_headers",
    srcs = [
        "pcireg.h",
        "pcivar.h",
        "pci_cfgreg.h",
    ],
)

filegroup(
    name = "pci_private_headers",
    srcs = [
        "pci_private.h",
        "pcib_private.h",
    ],
)

dfly_kernel_headers(
    name = "pci_priv",
    hdrs = [":pci_private_headers"],
    include_prefix = "bus/pci",
    visibility = [
        "//sys/bus/pci/x86_64:__pkg__",
        "//sys/dev/acpica:__pkg__",
    ],
)

dfly_kernel_headers(
    name = "pci",
    hdrs = [
        ":pci_headers",
    ],
    include_prefix = "bus/pci",
    deps = [
        ":ifs",
        "//sys/bus/pci/x86_64:priv",
    ],
    visibility = ["//visibility:public"],
)

dfly_kernel_lib(
    name = "pci_obj",
    srcs = glob(["*.c"], exclude = [
        "vga_pci.c",
    ]) + [":pci_if_src", ":pcib_if_src"],
    deps  = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/kern:core_ifs",
        "//sys/vm",
        "//sys/bus/pci/x86_64",
        "//sys/bus/isa",
        "//sys/bus/u4b/controller:usb_regs",
        "//sys/platform/pc64/apic",
        ":pci_priv",
        ":pci",
    ],
    visibility = ["//visibility:public"],
)
