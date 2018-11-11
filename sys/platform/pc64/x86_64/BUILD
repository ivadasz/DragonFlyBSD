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
    ],
)
