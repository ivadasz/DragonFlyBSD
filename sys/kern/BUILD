load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kernel_object")
load("//sys:build_defs.bzl", "dfly_kobj_hdr")

dfly_kobj_hdr(name = "device_if_hdr", kobj = "device_if.m")
dfly_kobj_hdr(name = "bus_if_hdr", kobj = "bus_if.m")
dfly_kernel_headers(
    name = "core_ifs",
    hdrs = [
        ":device_if_hdr",
        ":bus_if_hdr",
    ],
)

dfly_kernel_lib(
    name = "kern",
    # TODO(ivadasz): kern_sched.c depends on _kposix_priority_scheduling feature
    srcs = glob(["*.c"], exclude = ["subr_input.c", "kern_sched.c"]),
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//include:std",
        "//sys/sys:machine_cpu",
        "//sys/sys:platform",
        "//sys/ddb",
        "//sys/vm",
        "//sys/libprop",
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
    ],
)
