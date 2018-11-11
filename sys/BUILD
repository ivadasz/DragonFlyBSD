load("//sys:build_defs.bzl", "dfly_toolchain")
load("//sys:build_defs.bzl", "dfly_kernel_binary")

toolchain_type(name = "toolchain_type")

constraint_setting(name = "execmode")

constraint_value(
    name = "generic",
    constraint_setting = ":execmode",
)

constraint_value(
    name = "vkernel",
    constraint_setting = ":execmode",
)

platform(
    name = "dragonfly_kernel_x86_64",
    constraint_values = [
        "@bazel_tools//platforms:x86_64",
        ":generic",
    ],
)

dfly_toolchain(
    name = "dragonfly_kernel",
    arch_flags = [
        "-c",
        "-fno-builtin-log",
        "-pipe",
        "-O2",
        "-D_KERNEL",
    ],
    compiler_path = "/usr/bin/gcc",
    linker_path = "/usr/bin/ld",
    assembler_path = "/usr/bin/as",
    visibility = ["//visibility:public"],
)

toolchain(
    name = "dragonfly_kernel_toolchain",
    exec_compatible_with = [],
    target_compatible_with = [
        #":dragonfly_kernel_x86_64",
        "@bazel_tools//platforms:x86_64",
    ],
    toolchain = ":dragonfly_kernel",
    toolchain_type = ":toolchain_type",
)

########

dfly_kernel_binary(
    name = "kernel_generic",
    deps = [
        "//sys/kern",
        "//sys/vm",
        "//sys/libkern",
        "//sys/libprop",
        "//sys/dev/acpica:acpica_obj",
        "//sys/bus/pci:pci_obj",
        "//sys/bus/isa:isa_obj",
        "//sys/contrib/dev/acpica/source:acpica",
        "//sys/platform/pc64/acpica:acpica_obj",
        "//sys/platform/pc64/isa:isa_obj",
        "//sys/platform/pc64/icu:icu_obj",
        "//sys/platform/pc64/apic:apic_obj",
    ],
    ldscript = "//sys/platform/pc64/conf:ldscript.x86_64",
)
