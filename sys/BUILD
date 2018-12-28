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

constraint_value(
    name = "rump",
    constraint_setting = ":execmode",
)

config_setting(
    name = "userspace",
    constraint_values = [
        ":rump",
    ],
    visibility = ["//visibility:public"],
)

config_setting(
    name = "metal",
    constraint_values = [
        ":generic",
    ],
    visibility = ["//visibility:public"],
)

platform(
    name = "dragonfly_kernel_x86_64",
    constraint_values = [
        "@bazel_tools//platforms:x86_64",
        ":generic",
    ],
)

platform(
    name = "dragonfly_kernel_rump",
    constraint_values = [
        "@bazel_tools//platforms:x86_64",
        ":rump",
    ],
)

dfly_toolchain(
    name = "dragonfly_kernel",
    arch_flags = [
        "-c",
        # TODO(ivadasz): Only set -g flag when compiling for debugging.
        #"-g",

        #"-flto",
        "-ffreestanding",
        "-fno-builtin-log",
        "-fno-builtin-bcmp",
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
        "-D_KERNEL_BAZEL",
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
    linker_flags = [
        "-nostdlib",
        # -ffreestanding causes linking to fail
        #"-ffreestanding",
        "-Wl,--hash-style=sysv",
        "-Wl,-Bdynamic",
        #"-flto",
        #"-fwhole-program",
        "-Wl,--export-dynamic",
        "-Wl,--dynamic-linker=/red/herring",
    ],
    compiler_path = "/usr/bin/gcc",
    linker_path = "/usr/bin/ld",
    assembler_path = "/usr/bin/as",
    visibility = ["//visibility:public"],
)

toolchain(
    name = "dragonfly_kernel_x86_64_toolchain",
    exec_compatible_with = [],
    target_compatible_with = [
        ":generic",
        "@bazel_tools//platforms:x86_64",
    ],
    toolchain = ":dragonfly_kernel",
    toolchain_type = ":toolchain_type",
)

dfly_toolchain(
    name = "dragonfly_rump",
    arch_flags = [
        "-c",
        # TODO(ivadasz): Only set -g flag when compiling for debugging.
        #"-g",

        #"-flto",
        "-fno-builtin-log",
        #"-fno-builtin-bcmp",
        "-fno-common",
        "-fno-stack-protector",
        "-fno-strict-aliasing",
        "-fno-strict-overflow",
        #"-mcmodel=kernel",
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
        "-D_RUMPKERNEL",
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
    linker_flags = [
        "-Wl,--hash-style=sysv",
        "-Wl,-Bdynamic",
        #"-flto",
        #"-fwhole-program",
    ],
    compiler_path = "/usr/bin/gcc",
    linker_path = "/usr/bin/ld",
    assembler_path = "/usr/bin/as",
    visibility = ["//visibility:public"],
)

toolchain(
    name = "dragonfly_rump_toolchain",
    exec_compatible_with = [
        "@bazel_tools//platforms:x86_64",
        "@bazel_tools//platforms:dragonfly",
    ],
    target_compatible_with = [
        ":rump",
        "@bazel_tools//platforms:x86_64",
    ],
    toolchain = ":dragonfly_rump",
    toolchain_type = ":toolchain_type",
)

########

dfly_kernel_binary(
    name = "kernel_generic",
    deps = [
        "//sys/kern",
        "//sys/vm",
        "//sys/net:radix",
        "//sys/net:netisr_obj",
        "//sys/net:net_obj",
        "//sys/vfs/procfs:core",
        "//sys/libprop:libprop_obj",
        "//sys/dev/acpica:acpica_obj",
        "//sys/bus/pci:pci_obj",
        "//sys/bus/isa:isa_obj",
        "//sys/crypto/chacha",
        "//sys/crypto/sha2",
        "//sys/contrib/dev/acpica/source:acpica",
        "//sys/platform/pc64/acpica:acpica_obj",
        "//sys/platform/pc64/isa:isa_obj",
        "//sys/platform/pc64/icu:icu_obj",
        "//sys/platform/pc64/apic:apic_obj",
        "//sys/platform/pc64/x86_64",
        "//sys/platform/pc64/vmm",
        "//sys/cpu/x86_64/misc",
        "//sys/libkern",

        # Filesystems
        "//sys/vfs/deadfs",
        "//sys/vfs/fifofs",
        "//sys/vfs/devfs",
        "//sys/vfs/tmpfs",
        "//sys/vfs/ufs",

        # Drivers
        "//sys/dev/serial/sio",
        "//sys/dev/disk/md",

        # DDB support
        #"//sys/ddb:ddb_obj",
    ],
    ldscript = "//sys/platform/pc64/conf:ldscript.x86_64",
    #toolchains = [":dragonfly_kernel_x86_64_toolchain"],
)

dfly_kernel_binary(
    name = "kernel_rump",
    deps = [
        "//sys/kern",
        "//sys/vm",
        "//sys/net:radix",
        "//sys/net:netisr_obj",
        "//sys/net:net_obj",
        "//sys/libprop:libprop_obj",
        "//sys/crypto/chacha",
        "//sys/crypto/sha2",
        "//sys/platform/rump/mi",
        "//sys/platform/rump/x86_64",
        "//sys/cpu/x86_64/misc",
        #"//sys/libkern",

        # Filesystems
        "//sys/vfs/deadfs",
        "//sys/vfs/fifofs",
        "//sys/vfs/devfs",
        #"//sys/vfs/tmpfs",
        #"//sys/vfs/ufs",

        # Drivers
        #"//sys/dev/disk/md",
    ],
    #ldscript = "//sys/platform/pc64/conf:ldscript.x86_64",
    #toolchains = [":dragonfly_kernel_rump_toolchain"],
)
