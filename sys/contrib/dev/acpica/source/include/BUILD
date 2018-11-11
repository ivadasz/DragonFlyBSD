load("//sys:build_defs.bzl", "dfly_kernel_headers")

filegroup(
    name = "acpica_headers",
    srcs = glob(["*.h"]),
)

filegroup(
    name = "acpica_platform_headers",
    srcs = [
        "platform/acdragonfly.h",
        "platform/acenv.h",
        "platform/acenvex.h",
        "platform/acdragonflyex.h",
        "platform/acgcc.h",
        "platform/acgccex.h",
    ],
)

dfly_kernel_headers(
    name = "acpica_platform",
    hdrs = [":acpica_platform_headers"],
    include_prefix = "platform",
)

dfly_kernel_headers(
    name = "acpica",
    hdrs = [":acpica_headers"],
    include_prefix = "contrib/dev/acpica/source/include",
    deps = [":acpica_platform"],
    visibility = ["//visibility:public"],
)

dfly_kernel_headers(
    name = "acpica_priv",
    hdrs = [
        "acbuffer.h",
        "accommon.h",
        "acconfig.h",
        "acdebug.h",
        "acexcep.h",
        "acglobal.h",
        "achware.h",
        "aclocal.h",
        "acmacros.h",
        "acnames.h",
        "acnamesp.h",
        "acobject.h",
        "acoutput.h",
        "acpi.h",
        "acpiosxf.h",
        "acpixf.h",
        "acrestyp.h",
        "acstruct.h",
        "actbl.h",
        "actbl1.h",
        "actbl2.h",
        "actbl3.h",
        "actypes.h",
        "acutils.h",
    ],
    visibility = [
        "//sys/dev/acpica:__pkg__",
        "//sys/dev/acpica/Osd:__pkg__",
    ],
)
