load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kernel_object")
load("//sys:build_defs.bzl", "dfly_kobj_code")
load("//sys:build_defs.bzl", "dfly_kobj_hdr")

exports_files([
    "genassym.sh",
])

dfly_kobj_hdr(name = "device_if_hdr", kobj = "device_if.m")
dfly_kobj_hdr(name = "bus_if_hdr", kobj = "bus_if.m")
dfly_kobj_hdr(name = "cpu_if_hdr", kobj = "cpu_if.m")
dfly_kernel_headers(
    name = "core_ifs",
    hdrs = [
        ":device_if_hdr",
        ":bus_if_hdr",
        #":cpu_if_hdr",
    ],
    visibility = ["//visibility:public"],
)

dfly_kobj_code(name = "device_if_src", kobj = "device_if.m")
dfly_kobj_code(name = "bus_if_src", kobj = "bus_if.m")
dfly_kobj_code(name = "cpu_if_src", kobj = "cpu_if.m")
filegroup(
    name = "core_ifs_srcs",
    srcs = [
        ":device_if_src",
        ":bus_if_src",
        #":cpu_if_src",
    ],
)

dfly_kernel_lib(
    name = "ioconf",
    srcs = ["ioconf.c"],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        ":core_ifs",
        "//sys/bus/isa",
    ],
)

dfly_kernel_lib(
    name = "kern",
    # TODO(ivadasz): kern_sched.c depends on _kposix_priority_scheduling feature
    srcs = glob(["*.c"], exclude = [
        "subr_input.c",
        "kern_sched.c",
        "uipc_accf.c",
        "kern_ktr.c",
        "imgact_gzip.c",
        "inflate.c",

        # built separately
        "ioconf.c",

        # Not used by core kernel parts
        "subr_cpuhelper.c",

        # Optional cpu topology support
        "subr_cpu_topology.c",

        # Only include when dynamic linking is enabled
        "link_elf.c",
        "link_elf_obj.c",

        # Firmware handling
        "subr_firmware.c",

        # pty driver
        "tty_pty.c",

        # kcollect
        "kern_collect.c",

        # sensors
        "kern_sensors.c",
    ]) + [
        ":core_ifs_srcs",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/ddb",
        "//sys/vm",
        "//sys/crypto/sha2",
        "//sys/crypto/chacha",
        "//sys/net",
        "//sys/netinet",
        "//sys/netinet6",
        "//sys/net/altq:net_altq",
        # kobj headers
        ":core_ifs",
        # very specific includes
        "//sys/net:radix",
        "//sys/vfs/ufs:dinode",
        "//sys/vfs/ufs:fs",
        "//sys/vfs/procfs",
        "//crypto/libressl/include/openssl:md5",
        "//sys/dev/acpica/acpi_pvpanic",

        # ioconf
        ":ioconf",
    ],
    visibility = ["//visibility:public"],
)
