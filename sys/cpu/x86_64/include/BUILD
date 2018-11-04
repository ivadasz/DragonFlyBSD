filegroup(
    name = "cpu_headers",
    srcs = glob(["*.h"]),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "machine_headers",
    srcs = glob(["*.h"], exclude = ["cpu.h", "param.h", "pmap.h", "types.h"]),
    visibility = ["//visibility:public"],
)
