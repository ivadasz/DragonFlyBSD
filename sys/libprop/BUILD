load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")

filegroup(
    name = "libprop_headers",
    srcs = glob(["*.h"], exclude = [
        "prop_object_impl.h",
        "prop_rb_impl.h",
        "prop_stack.h",
    ]),
)

dfly_kernel_headers(
    name = "libprop",
    hdrs = [":libprop_headers"],
    include_prefix = "libprop",
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "private_headers",
    hdrs = ["prop_object_impl.h", "prop_rb_impl.h", "prop_stack.h"],
)

dfly_kernel_lib(
    name = "libprop_obj",
    srcs = glob(["*.c"]),
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/sys:platform",
        "//sys/sys:machine_cpu",
        "//sys/vm",
        "//include:std",
        ":libprop",
        ":private_headers",
    ],
    visibility = ["//visibility:public"],
)
