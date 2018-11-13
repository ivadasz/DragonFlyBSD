load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kernel_headers")

dfly_kernel_lib(
    name = "vmx_genassym_o",
    srcs = [
        "vmx_genassym.c",
        "ept.h",
        "vmx.h",
        "vmx_instr.h",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/vm",
    ],
)

genrule(
    name = "vmx_assym_h",
    srcs = [":vmx_genassym_o"],
    outs = ["vmx_assym.h"],
    cmd = "sh $(location //sys/kern:genassym.sh) -o $@ $<",
    tools = ["//sys/kern:genassym.sh"],
)

dfly_kernel_headers(
    name = "assym",
    hdrs = [":vmx_assym_h"],
)

dfly_kernel_lib(
    name = "vmm",
    srcs = [
        "ept.c",
        "svm.h",
        "vmm.c",
        "vmx_instr.h",
        "vmx.c",
        "ept.h",
        "vmm_utils.c",
        "vmm.h",
        "vmx_trap.s",
        "vmx.h",
        "svm.c",
        "vmm_utils.h",
        "vmx_vmcs.h",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/vm",
        ":assym",
    ],
    visibility = ["//visibility:public"],
)
