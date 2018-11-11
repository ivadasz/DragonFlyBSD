load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kernel_headers")

dfly_kernel_headers(
    name = "priv",
    hdrs = ["pci_cfgreg.h"],
    include_prefix = "x86_64",
    visibility = ["//sys/bus/pci:__pkg__"],
)

dfly_kernel_lib(
    name = "x86_64",
    srcs = glob(["*.c"]) + ["legacyvar.h", "pci_cfgreg.h"],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/sys:machine_cpu",
        "//sys/kern:core_ifs",
        "//sys/platform/pc64/include:pc",
        "//sys/platform/pc64/apic",
        "//sys/vm",
        "//sys/bus/isa",
        "//sys/bus/pci",
        "//sys/bus/pci:pci_priv",
    ],
    visibility = ["//sys/bus/pci:__pkg__"],
)
