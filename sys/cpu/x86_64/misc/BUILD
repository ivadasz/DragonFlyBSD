load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_lib(
    name = "misc",
    srcs = [
        # TODO(ivadasz): Compile atomic with -fomit-frame-pointer (except when
        # building a profiling kernel, apparently).
        "atomic.c",
        "lwbuf.c",
        "elf_machdep.c",
        "bzeront.s",
        "monitor.s",
    ],
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/vm",
        "//sys/platform/pc64/x86_64:assym",
    ],
    visibility = ["//visibility:public"],
)
