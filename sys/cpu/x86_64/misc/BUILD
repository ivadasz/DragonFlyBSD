load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_lib(
    name = "misc",
    srcs = select({
        "//sys:userspace": [
            # TODO(ivadasz): Compile atomic with -fomit-frame-pointer (except
            # when building a profiling kernel, apparently).
            "atomic.c",
            "lwbuf.c",
            "bzeront.s",
        ],
        "//sys:metal": [
            # TODO(ivadasz): Compile atomic with -fomit-frame-pointer (except when
            # building a profiling kernel, apparently).
            "atomic.c",
            "lwbuf.c",
            "elf_machdep.c",
            "bzeront.s",
            "monitor.s",

            # DDB support
            #"db_disasm.c",
            #"x86_64-gdbstub.c",
        ],
    }),
    deps = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/vm",
        "//sys/platform/pc64/x86_64:assym",
        # For db_disasm.c
        #"//sys/ddb:ddb_all",
    ],
    visibility = ["//visibility:public"],
)
