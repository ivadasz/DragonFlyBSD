load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")

filegroup(
    name = "net_headers",
    srcs = glob(["*.h"], exclude = [
        "radix.h",
        "netisr.h", "netisr2.h", "netmsg.h", "netmsg2.h",
        "ethernet.h",
    ]),
)

dfly_kernel_headers(
    name = "net",
    hdrs = [":net_headers"],
    include_prefix = "net",
    deps = [
        ":netisr",
        ":radix",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "radix_headers",
    srcs = ["radix.h"],
)

dfly_kernel_lib(
    name = "radix",
    srcs = ["radix.c"],
    deps = [
        "//sys/sys",
        ":netisr",
    ],
    hdrs = [":radix_headers"],
    include_prefix = "net",
    visibility = ["//visibility:public"],
)

dfly_kernel_lib(
    name = "netisr",
    hdrs = [
        "netisr.h",
        "netisr2.h",
        "netmsg.h",
        "netmsg2.h",
        "ethernet.h",
    ],
    include_prefix = "net",
    visibility = ["//visibility:public"],
)

dfly_kernel_lib(
    name = "netisr_obj",
    srcs = [
        "netisr.c",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/vm",
        ":netisr",
    ],
    visibility = ["//visibility:public"],
)

dfly_kernel_lib(
    name = "net_obj",
    srcs = [
        "if.c",
        "if_clone.c",
        "route.c",
        "rtsock.c",
        "raw_cb.c",
        "raw_usrreq.c",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/kern:core_ifs",
        "//sys/net",
        "//sys/net/altq:net_altq",
        "//sys/netinet",
        "//sys/netinet6",
        "//sys/net/ip_mroute",
        "//sys/vm",
        ":netisr",
    ],
    visibility = ["//visibility:public"],
)
