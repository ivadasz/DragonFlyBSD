load("//sys:build_defs.bzl", "dfly_kernel_headers")

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
    # TODO(ivadasz): Limit visibility to the few places that need this.
    visibility = ["//visibility:public"],
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
)
