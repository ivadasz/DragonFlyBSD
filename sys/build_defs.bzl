CompilerInfo = provider(
    doc = "Information about how to invoke gcc for the DragonFly kernel.",
    fields = ["compiler_path", "arch_flags"],
)

def _dfly_toolchain_impl(ctx):
    toolchain_info = platform_common.ToolchainInfo(
        compilerinfo = CompilerInfo(
            compiler_path = ctx.attr.compiler_path,
            arch_flags = ctx.attr.arch_flags,
        ),
    )
    return [toolchain_info]

dfly_toolchain = rule(
    implementation = _dfly_toolchain_impl,
    attrs = {
        "compiler_path": attr.string(),
        "arch_flags": attr.string_list(),
    },
)

####### Rules for building kernel

IncludeDirs = provider("transitive_includes")
TransitiveDeps = provider("transitive_deps")

def get_transitive_includes(incs, deps):
    """Obtain the includes for a target and its transitive dependencies."""
    return depset(
        incs,
        transitive = [dep[IncludeDirs].transitive_includes for dep in deps],
    )

def get_transitive_deps(deps):
    """Obtain the transitive dependencies for a target."""
    return depset(
        deps,
        transitive = [dep[TransitiveDeps].transitive_deps for dep in deps],
    )

def _dfly_kernel_object_impl(ctx):
    trans_incs = get_transitive_includes([], ctx.attr.deps)
    trans_deps = get_transitive_deps(ctx.attr.deps)
    info = ctx.toolchains["//sys:toolchain_type"].compilerinfo
    cflags = [
        "-nostdinc",
    ] + info.arch_flags
    for i in trans_incs.to_list():
        cflags += ["-I%s" % i]
    myobj = None
    set = depset()
    depfiles = []
    for i in trans_deps:
        depfiles += i.files.to_list()
    for s in ctx.attr.srcs:
        for i in s.files.to_list():
            if i.extension == "c":
                objname = i.basename.rstrip("c") + "o"
            else:
                objname = i.basename + ".o"
            obj = ctx.actions.declare_file(objname)
            set = depset(direct = [obj], transitive = [set])
            ctx.actions.run(outputs = [obj], inputs = [i] + depfiles,
                            executable = info.compiler_path,
                            arguments = cflags + ["-o", obj.path] + [i.path], mnemonic = "CCompile",
                            progress_message = "Compiling %s to %s" % (i.basename, obj.basename))
            myobj = obj
    return [
        IncludeDirs(transitive_includes = trans_incs),
        DefaultInfo(files = set)
    ]

dfly_kernel_object = rule(
    implementation = _dfly_kernel_object_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = True),
        "deps": attr.label_list(allow_files = True),
    },
    toolchains = ["//sys:toolchain_type"],
)

def _dfly_kernel_headers_impl(ctx):
    loc = ctx.label.name + "/_include"
    trans_incs = get_transitive_includes([ctx.bin_dir.path + "/" + ctx.label.package + "/" + loc], ctx.attr.deps)
    if ctx.attr.include_prefix:
        loc += "/" + ctx.attr.include_prefix
    set = depset(direct = [])
    for h in ctx.attr.hdrs:
        for i in h.files.to_list():
            hdr = ctx.actions.declare_file(loc + "/" + i.basename)
            set = depset(direct = [hdr], transitive = [set])
            ctx.actions.run_shell(outputs = [hdr], inputs = [i],
                                  arguments = [],
                                  command = "/bin/ln -s $(realpath %s) %s" % (i.path, hdr.path),
                                  mnemonic = "LinkHeader",
                                  progress_message = "Linking header %s into virtual include directory %s" % (i.path, loc))
    return [
        IncludeDirs(transitive_includes = trans_incs),
        TransitiveDeps(transitive_deps = get_transitive_deps(ctx.attr.deps)),
        DefaultInfo(files = set)
    ]

dfly_kernel_headers = rule(
    implementation = _dfly_kernel_headers_impl,
    attrs = {
        "hdrs": attr.label_list(allow_files = True),
        "include_prefix": attr.string(),
        "deps": attr.label_list(),
    },
)

def dfly_kernel_lib(name, srcs=[], deps=[], hdrs=[], include_prefix="", visibility=None):
  if len(hdrs) > 0:
    dfly_kernel_headers(name = name + "_hdrs", hdrs = hdrs, include_prefix = include_prefix, visibility = visibility)
    deps += [name + "_hdrs"]
  if len(srcs) > 0:
    dfly_kernel_object(name = name, srcs = srcs, deps = deps, visibility = visibility)

def dfly_kobj_hdr(name, kobj):
    if kobj.endswith(".m"):
        hdr = kobj.rstrip("m") + "h"
        native.genrule(name = name, srcs = [kobj], outs = [hdr],
                       cmd = "/usr/bin/awk -f $(location //sys/tools:makeobjops.awk) $< -h && cp %s $@" % hdr,
                       tools = ["//sys/tools:makeobjops.awk"])

def dfly_kobj_code(name, kobj):
    if kobj.endswith(".m"):
        code = kobj.rstrip("m") + "c"
        native.genrule(name = name, srcs = [kobj], outs = [code],
                       cmd = "/usr/bin/awk -f $(location //sys/tools:makeobjops.awk) $< -c && cp %s $@" % code,
                       tools = ["//sys/tools:makeobjops.awk"])

def dfly_opt_header(name, opt, vals=[]):
  if len(vals) > 0:
    native.genrule(name = name,
                   srcs = [],
                   outs = ["opt_" + opt + ".h"],
                   cmd = "for i in %s; do echo \#define $$i > \"$@\"; done" % ' '.join(vals))
  else:
    native.genrule(name = name,
                   srcs = [],
                   outs = ["opt_" + opt + ".h"],
                   cmd = "touch \"$@\"")

def dfly_use_header(name, use):
  native.genrule(name = name,
                 srcs = [],
                 outs = ["use_" + use + ".h"],
                 cmd = "touch \"$@\"")

# XXX
def dfly_kernel_binary(name, deps=[], visibility=None):
  dfly_kernel_object(name = name, deps = deps, visibility = visibility)
