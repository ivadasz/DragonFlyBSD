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
    visibility = [
        "//sys/contrib/dev/acpica/source:__pkg__",
        "//sys/platform/pc64/acpica:__pkg__",
    ],
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
        "acconvert.h",
        "acdebug.h",
        "acdispat.h",
        "acdisasm.h",
        "acevents.h",
        "acexcep.h",
        "acglobal.h",
        "achware.h",
        "acinterp.h",
        "aclocal.h",
        "acmacros.h",
        "acnames.h",
        "acnamesp.h",
        "acobject.h",
        "acopcode.h",
        "acoutput.h",
        "acparser.h",
        "acpi.h",
        "acpiosxf.h",
        "acpixf.h",
        "acpredef.h",
        "acresrc.h",
        "acrestyp.h",
        "acstruct.h",
        "actables.h",
        "actbl.h",
        "actbl1.h",
        "actbl2.h",
        "actbl3.h",
        "actypes.h",
        "acutils.h",
        "amlcode.h",
        "amlresrc.h",
    ],
    visibility = [
        "//sys/contrib/dev/acpica/source:__pkg__",
        "//sys/dev/acpica:__pkg__",
        "//sys/dev/acpica/Osd:__pkg__",
        "//sys/platform/pc64/acpica:__pkg__",
    ],
)
