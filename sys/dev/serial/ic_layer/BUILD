load("//sys:build_defs.bzl", "dfly_kernel_headers")

dfly_kernel_headers(
    name = "ic_layer",
    hdrs = [
        "esp.h",
        "ns16550.h",
    ],
    include_prefix = "dev/serial/ic_layer",
    visibility = [
        "//sys/dev/serial/sio:__pkg__",
    ],
)
