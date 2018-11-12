load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "cam_headers",
    srcs = [
        "cam.h",
        "cam_ccb.h",
        "cam_debug.h",
    ],
)

filegroup(
    name = "scsi_headers",
    srcs = [
        "scsi/scsi_all.h",
    ],
)

dfly_kernel_headers(
    name = "scsi",
    hdrs = [":scsi_headers"],
    include_prefix = "scsi",
)

dfly_kernel_headers(
    name = "cam",
    hdrs = [":cam_headers"],
    include_prefix = "bus/cam",
    deps = [":scsi"],
    visibility = ["//visibility:public"],
)
