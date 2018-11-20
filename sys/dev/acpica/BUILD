load("//sys:build_defs.bzl", "dfly_kernel_headers")
load("//sys:build_defs.bzl", "dfly_kernel_lib")
load("//sys:build_defs.bzl", "dfly_kobj_code")
load("//sys:build_defs.bzl", "dfly_kobj_hdr")

dfly_kobj_hdr(name = "acpi_if_hdr", kobj = "acpi_if.m")
dfly_kobj_hdr(name = "acpi_wmi_if_hdr", kobj = "acpi_wmi_if.m")

dfly_kobj_code(name = "acpi_if_src", kobj = "acpi_if.m")
dfly_kobj_code(name = "acpi_wmi_if_src", kobj = "acpi_wmi_if.m")

filegroup(
    name = "acpi_priv_headers",
    srcs = [
        ":acpi_if_hdr",
        ":acpi_wmi_if_hdr",
        ":quirks",
    ],
)

filegroup(
    name = "acpi_headers",
    srcs = [
        "acpiio.h",
        "acpiio_mcall.h",
        "acpivar.h",
        "acpi_pcibvar.h",
        "acpi_sci_var.h",
    ],
)

dfly_kernel_headers(
    name = "acpi_machine",
    hdrs = [
        "acpi_cpu.h",
        "acpi_cpu_cstate.h",
        "acpi_cpu_pstate.h",
        "acpi_sci_var.h",
    ],
    visibility = ["//sys/platform/pc64/acpica:__pkg__"],
)

dfly_kernel_headers(
    name = "acpi_priv",
    hdrs = [":acpi_priv_headers"],
    visibility = ["//visibility:public"],
)

genrule(
    name = "quirks",
    srcs = ["acpi_quirks"],
    outs = ["acpi_quirks.h"],
    cmd = "/usr/bin/awk -f $(location //sys/tools:acpi_quirks2h.awk) $< && cp acpi_quirks.h $@",
    tools = ["//sys/tools:acpi_quirks2h.awk"]
)

dfly_kernel_lib(
    name = "acpica",
    srcs = [
        "acpi.c",
        "acpi_cpu.c",
        "acpi_ec.c",
        "acpi_hpet.c",
        "acpi_isab.c",
        "acpi_package.c",
        "acpi_pci.c",
        "acpi_pci_link.c",
        "acpi_pcib.c",
        "acpi_pcib_acpi.c",
        "acpi_pcib_pci.c",
        "acpi_powerres.c",
        "acpi_quirk.c",
        "acpi_resource.c",
        "acpi_timer.c",
        ":acpi_if_src",
        ":acpi_wmi_if_src",

        # Only relevant for real hardware
        #"acpi_cpu_cstate.c",
        #"acpi_cpu_pstate.c",
        #"acpi_acad.c",
        #"acpi_battery.c",
        #"acpi_button.c",
        #"acpi_cmbat.c",
        #"acpi_lid.c",
        #"acpi_smbat.c",
        #"acpi_thermal.c",

	# Headers
        "acpi_cpu.h",
        #"acpi_cpu_cstate.h",
        #"acpi_cpu_pstate.h",
        "acpi_hpet.h",
        "acpi_smbus.h",
    ],
    deps  = [
        "//sys/sys:options",
        "//sys/sys",
        "//sys/kern:core_ifs",
        "//sys/vm",
        "//sys/contrib/dev/acpica/source/include:acpica",
        "//sys/contrib/dev/acpica/source/include:acpica_priv",
        "//sys/bus/isa",
        "//sys/bus/pci",
        "//sys/bus/pci:pci_priv",
        "//sys/platform/pc64/acpica:acpica_sdt",
        ":acpi_priv",
    ],
    hdrs = [":acpi_headers"],
    include_prefix = "dev/acpica",
    visibility = ["//visibility:public"],
)

dfly_kernel_lib(
    name = "acpica_obj",
    deps = [
        "//sys/dev/acpica",
        "//sys/dev/acpica/Osd",
    ],
    visibility = ["//visibility:public"],
)
