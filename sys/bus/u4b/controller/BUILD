load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "usb_regs_headers",
    srcs = [
        "ehcireg.h",
        "ohcireg.h",
        "uhcireg.h",
        "xhcireg.h",
    ],
)

dfly_kernel_headers(
    name = "usb_regs",
    hdrs = [":usb_regs_headers"],
    include_prefix = "bus/u4b/controller",
    visibility = ["//visibility:public"],
)
