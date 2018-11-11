load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kobj_code")
load("//sys:build_defs.bzl", "dfly_kobj_hdr")

dfly_kobj_hdr(name = "isa_if_hdr", kobj = "isa_if.m")
dfly_kernel_headers(
    name = "ifs",
    hdrs = [
        ":isa_if_hdr",
    ],
    visibility = ["//visibility:public"],
)

dfly_kobj_code(name = "isa_if_src", kobj = "isa_if.m")

filegroup(
    name = "isa_priv_headers",
    srcs = [
    ],
)

filegroup(
    name = "isa_headers",
    srcs = [
        "isavar.h",
        "pnpvar.h",
    ],
)

dfly_kernel_headers(
    name = "isa_priv",
    hdrs = [":isa_priv_headers"],
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "isa",
    hdrs = [
        ":isa_headers",
    ],
    include_prefix = "bus/isa",
    deps = [
        ":ifs",
    ],
    visibility = ["//visibility:public"],
)

dfly_kernel_lib(
    name = "isa_obj",
    srcs = [
    ],
    deps  = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/kern:core_ifs",
        "//include:std",
        "//sys/sys:platform",
        ":isa_priv",
        ":isa",
    ],
    visibility = ["//visibility:public"],
)
