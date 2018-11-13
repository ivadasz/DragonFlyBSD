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
        "-fno-common",
        "-fno-stack-protector",
        "-fno-strict-aliasing",
        "-fno-strict-overflow",
        "-mcmodel=kernel",
        "-mno-red-zone",
        "-mfpmath=387",
        "-mno-mmx",
        "-mno-3dnow",
        "-mno-sse",
        "-mno-sse2",
        "-mno-sse3",
        # if gcc
        "-mpreferred-stack-boundary=4",
        "-mno-ssse3",
        "-mno-sse4.1",
        "-mno-sse4.2",
        "-mno-sse4",
        "-mno-sse4a",
        "-mno-sse5",
        "-mno-abm",
        "-mno-aes",
        "-mno-avx",
        "-mno-pclmul",
        "-mno-popcnt",
        # if gcc4 or gcc5
        #"-mno-avx2",
        #"-mno-fsgsbase",
        #"-mno-rdrnd",
        #"-mno-f16c",
        #"-mno-fma",
        #"-mno-fma4",
        #"-mno-bmi",
        #"-mno-bmi2",
        #"-mno-xop",
        #"-mno-lwp",
        #"-mno-lzcnt",
        #"-mno-tbm",

        "-msoft-float",
        "-fno-asynchronous-unwind-tables",
        "-fno-omit-frame-pointer",

        "-pipe",
        "-O2",
        "-D_KERNEL",
    ],
    asm_flags = [
        "-DLOCORE",
        "-x",
        "assembler-with-cpp",
    ],
    conly_flags = [
        "-x",
        "c",
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
        "//sys/net:netisr_obj",
        "//sys/vfs/procfs:core",
        "//sys/libprop:libprop_obj",
        "//sys/dev/acpica:acpica_obj",
        "//sys/bus/pci:pci_obj",
        "//sys/bus/isa:isa_obj",
        "//sys/contrib/dev/acpica/source:acpica",
        "//sys/platform/pc64/acpica:acpica_obj",
        "//sys/platform/pc64/isa:isa_obj",
        "//sys/platform/pc64/icu:icu_obj",
        "//sys/platform/pc64/apic:apic_obj",
        "//sys/platform/pc64/x86_64",
        "//sys/platform/pc64/vmm",
        "//sys/cpu/x86_64/misc",
    ],
    ldscript = "//sys/platform/pc64/conf:ldscript.x86_64",
)
