CompilerInfo = provider(
    doc = "Information about how to invoke gcc for the DragonFly kernel.",
    fields = ["compiler_path", "linker_path", "assembler_path", "arch_flags"],
)

def _dfly_toolchain_impl(ctx):
    toolchain_info = platform_common.ToolchainInfo(
        compilerinfo = CompilerInfo(
            compiler_path = ctx.attr.compiler_path,
            linker_path = ctx.attr.linker_path,
            assembler_path = ctx.attr.assembler_path,
            arch_flags = ctx.attr.arch_flags,
        ),
    )
    return [toolchain_info]

dfly_toolchain = rule(
    implementation = _dfly_toolchain_impl,
    attrs = {
        "compiler_path": attr.string(),
        "linker_path": attr.string(),
        "assembler_path": attr.string(),
        "arch_flags": attr.string_list(),
    },
)

####### Rules for building kernel

IncludeDirs = provider("transitive_includes")
TransitiveDeps = provider("transitive_deps")
TransitiveObjects = provider("transitive_objects")

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

def get_transitive_objects(objs):
    """Obtain the transitive objects for a target."""
    return depset(
        objs,
        transitive = [obj[TransitiveObjects].transitive_objects for obj in objs],
    )

def _dfly_kernel_object_impl(ctx):
    trans_incs = get_transitive_includes([], ctx.attr.deps)
    trans_hdr_incs = get_transitive_includes([], ctx.attr.hdeps)
    trans_deps = get_transitive_deps(ctx.attr.deps)
    trans_hdeps = get_transitive_deps(ctx.attr.hdeps)
    trans_objs = get_transitive_objects(ctx.attr.deps)
    info = ctx.toolchains["//sys:toolchain_type"].compilerinfo
    cflags = [
        "-nostdinc",
    ] + info.arch_flags
    for i in trans_incs:
        cflags += ["-I%s" % i]
    for i in trans_hdr_incs:
        cflags += ["-I%s" % i]
    set = depset()
    depfiles = depset()
    for s in ctx.attr.srcs:
        for i in s.files:
            if i.extension == "h":
                depfiles = depset(direct = [i], transitive = [depfiles])
    for i in trans_deps:
        for h in i.files:
            # TODO(ivadasz): Solve the problem of including .c files nicer.
            if h.extension in ["h", "c", "s"]:
                depfiles = depset(direct = [h], transitive = [depfiles])
    for i in trans_hdeps:
        for h in i.files:
            if h.extension == "h":
                depfiles = depset(direct = [h], transitive = [depfiles])
    for s in ctx.attr.srcs:
        for i in s.files:
            if i.extension == "c":
                objname = i.basename.rstrip("c") + "o"
                obj = ctx.actions.declare_file(objname)
                set = depset(direct = [obj], transitive = [set])
                ctx.actions.run(outputs = [obj], inputs = depset(direct = [i], transitive = [depfiles]),
                                executable = info.compiler_path,
                                arguments = cflags + ["-x", "c", "-o", obj.path, i.path], mnemonic = "CCompile",
                                progress_message = "Compiling %s to %s" % (i.basename, obj.basename))
            elif i.extension in ["s", "S"]:
                objname = i.basename.rstrip("s") + "o"
                obj = ctx.actions.declare_file(objname)
                set = depset(direct = [obj], transitive = [set])
                ctx.actions.run(outputs = [obj], inputs = depset(direct = [i], transitive = [depfiles]),
                                executable = info.compiler_path,
                                arguments = cflags + ["-DLOCORE", "-x", "assembler-with-cpp", "-o", obj.path, i.path], mnemonic = "Assembler",
                                progress_message = "Compiling %s to %s" % (i.basename, obj.basename))
    return [
        IncludeDirs(transitive_includes = trans_hdr_incs),
        TransitiveDeps(transitive_deps = trans_hdeps),
        TransitiveObjects(transitive_objects = trans_objs),
        DefaultInfo(files = depset(transitive = [set])),
    ]

dfly_kernel_object = rule(
    implementation = _dfly_kernel_object_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = True),
        "deps": attr.label_list(),
        "hdeps": attr.label_list(),
    },
    toolchains = ["//sys:toolchain_type"],
)

# TODO(ivadasz): Allow declaring a hierarchy of header file directories, to
# make double-quoted relative includes work as expected.
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
        TransitiveObjects(transitive_objects = depset()),
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
  hdeps = []
  if len(hdrs) > 0:
    dfly_kernel_headers(name = name + "_hdrs", hdrs = hdrs, include_prefix = include_prefix)
    hdeps = [name + "_hdrs"]
  dfly_kernel_object(name = name, srcs = srcs, deps = deps, hdeps = hdeps, visibility = visibility)

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

def map_depfile(itm):
  if itm.extension == "o":
    return itm.path
  else:
    return None

def _dfly_kernel_binary_impl(ctx):
    trans_objs = get_transitive_objects(ctx.attr.deps)
    info = ctx.toolchains["//sys:toolchain_type"].compilerinfo
    ldscript = ctx.attr.ldscript.files.to_list()
    if len(ldscript) != 1:
        fail("Exactly one ldscript needed")
    ldflags = [
        "-nostdlib",
        "-T",
        ldscript[0].path,
    ]
    bin = ctx.actions.declare_file("kernel")
    depfiles = depset(transitive = [ctx.attr.ldscript.files])
    args = ctx.actions.args()
    for f in ldflags:
        args.add(f)
    for i in trans_objs:
        for h in i.files:
            if h.extension == "o":
                args.add(h.path)
                depfiles = depset(direct = [h], transitive = [depfiles])
    args.add("-o", bin.path)
    ctx.actions.run(outputs = [bin], inputs = depset(transitive = [depfiles]),
                    executable = info.linker_path,
                    arguments = [args], mnemonic = "CLink",
                    progress_message = "Linking %s" % bin.basename)
    return [
        IncludeDirs(transitive_includes = depset()),
        TransitiveDeps(transitive_deps = depset()),
        TransitiveObjects(transitive_objects = depset()),
        DefaultInfo(files = depset(direct = [bin])),
    ]

dfly_kernel_binary = rule(
    implementation = _dfly_kernel_binary_impl,
    attrs = {
        "deps": attr.label_list(),
        "ldscript": attr.label(allow_single_file = True),
    },
    toolchains = ["//sys:toolchain_type"],
)
