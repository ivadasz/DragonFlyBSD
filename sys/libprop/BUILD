load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "private_headers",
    srcs = ["prop_object_impl.h", "prop_rb_impl.h", "prop_stack.h"],
)

filegroup(
    name = "libprop_headers",
    srcs = glob(["*.h"], exclude = [
        ":private_headers",
    ]),
)

dfly_kernel_headers(
    name = "libprop",
    hdrs = [":libprop_headers"],
    include_prefix = "libprop",
    visibility = ["//visibility:public"],
)

dfly_kernel_lib(
    name = "libprop_obj",
    srcs = glob(["*.c"]) + [":private_headers"],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/sys:machine_cpu",
        "//sys/vm",
        ":libprop",
    ],
    visibility = ["//visibility:public"],
)
