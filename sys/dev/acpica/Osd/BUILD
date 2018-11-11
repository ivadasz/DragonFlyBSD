load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_lib(
    name = "Osd",
    srcs = glob(["*.c"]),
    deps  = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/kern:core_ifs",
        "//sys/contrib/dev/acpica/source/include:acpica",
        "//sys/contrib/dev/acpica/source/include:acpica_priv",
        "//sys/dev/acpica",
        "//sys/dev/acpica:acpi_priv",
        "//sys/bus/pci",
        "//sys/ddb",
        "//sys/ddb:ddb_misc",
        "//sys/vm",
    ],
    visibility = ["//visibility:public"],
)
