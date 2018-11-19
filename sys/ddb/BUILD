load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")

dfly_kernel_headers(
    name = "ddb",
    hdrs = ["ddb.h"],
    include_prefix = "ddb",
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "ddb_misc",
    hdrs = ["db_sym.h", "db_variables.h", "db_access.h", "db_output.h"],
    include_prefix = "ddb",
    visibility = [
        "//sys/dev/acpica/Osd:__pkg__",
        "//sys/platform/pc64/x86_64:__pkg__",
    ],
)

filegroup(
    name = "ddb_all_headers",
    srcs = glob(["*.h"]),
)

# For usage for //sys/ddb/*.c sources
dfly_kernel_headers(
    name = "ddb_all",
    hdrs = [":ddb_all_headers"],
    include_prefix = "ddb",
    visibility = [
        "//sys/cpu/x86_64/misc:__pkg__",
    ],
)

dfly_kernel_lib(
    name = "ddb_obj",
    srcs = glob(["*.c"], exclude = ["db_kld.c"]),
    deps = [
        "//sys/sys",
        "//sys/vm",
        ":ddb_all",
    ],
    visibility = ["//visibility:public"],
)
