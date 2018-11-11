# TODO(ivadasz): lwbuf.h is actually needed by "sys" includes.
core_cpu_headers = [
    "atomic.h",
    "cpu.h",
    "frame.h",
    "lwbuf.h",
    "param.h",
    "pmap.h",
    "trap.h",
    "types.h",
]

filegroup(
    name = "cpu_headers",
    srcs = glob(["*.h"], exclude = core_cpu_headers),
    visibility = ["//sys/sys:__pkg__"],
)

filegroup(
    name = "cpu_headers_core",
    srcs = core_cpu_headers,
    visibility = ["//sys/sys:__pkg__"],
)

# Headers from here that are needed in machine/ on pc64.
filegroup(
    name = "machine_headers",
    srcs = [
        "atomic.h",
        "bus_dma.h",
        "cpufunc.h",
        "cpumask.h",
        "cputypes.h",
        "db_machdep.h",
        "elf.h",
        "endian.h",
        "exec.h",
        "frame.h",
        "inttypes.h",
        "int_const.h",
        "int_limits.h",
        "limits.h",
        "npx.h",
        "profile.h",
        "psl.h",
        "reg.h",
        "segments.h",
        "sigframe.h",
        "signal.h",
        "specialreg.h",
        "stdarg.h",
        "stdint.h",
        "trap.h",
        "tss.h",
        "ucontext.h",
        "vframe.h",
    ],
    visibility = ["//sys/sys:__pkg__"],
)
