load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kernel_headers")

dfly_kernel_lib(
    name = "assym_o",
    srcs = ["genassym.c"],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/kern:core_ifs",
        "//sys/net:radix",
        "//sys/net",
        "//sys/netinet",
        "//sys/netinet6",
        "//sys/platform/pc64/apic",
        "//sys/vfs/nfs",
        "//sys/vm",
    ],
)

genrule(
    name = "assym_s",
    srcs = [":assym_o"],
    outs = ["assym.s"],
    cmd = "sh $(location //sys/kern:genassym.sh) -o $@ $<",
    tools = ["//sys/kern:genassym.sh"],
)

dfly_kernel_headers(
    name = "assym",
    hdrs = [":assym_s"],
    visibility = [
        "//sys/platform/pc64/apic:__pkg__",
        "//sys/platform/pc64/icu:__pkg__",
        "//sys/cpu/x86_64/misc:__pkg__",
    ],
)

dfly_kernel_lib(
    name = "x86_64",
    srcs = [
        "amd64_mem.c",
        "exception.S",
        "mp_machdep.c",
        "sigtramp.s",
        "autoconf.c",
        "mpboot.S",
        "spinlock.s",
        "bios.c",
        "global.s",
        "mptable.c",
        "support.s",
        "identcpu.c",
        "msi_vector.s",
        "swtch.s",
        "busdma_machdep.c",
        "initcpu.c",
        "msi.c",
        "sysarch.c",
        "cpufreq_machdep.c",
        "ipl_funcs.c",
        "nexus.c",
        "tls.c",
        "ipl.s",
        "npx.c",
        "trap.c",
        "locore.s",
        "pmap_inval.c",
        "userldt.c",
        "dump_machdep.c",
        "machdep.c",
        "pmap.c",
        "uwrapper.c",
        "minidump_machdep.c",
        "procfs_machdep.c",
        "vm_machdep.c",

        # DDB support
        #"db_interface.c",
        #"db_trace.c",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        #"//sys/ddb",
        "//sys/vm",
        "//sys/kern:core_ifs",
        "//sys/net",
        "//sys/net:radix",
        "//sys/bus/cam",
        "//sys/bus/isa",
        "//sys/bus/pci",
        "//sys/platform/pc64/apic",
        "//sys/platform/pc64/isa",
        "//sys/platform/pc64/icu",
        "//sys/platform/pc64/include:pc",
        "//sys/platform/pc64/acpica:acpica_cpu",
        # TODO(ivadasz): For procfs_machdep.c
        "//sys/vfs/procfs",
        ":assym",

        # For db_trace.c
        #"//sys/ddb:ddb_misc",
    ],
    visibility = ["//visibility:public"],
)
