load("//sys:build_defs.bzl", "dfly_opt_header")

def dfly_opt_dummies(name, lst):
    out = []
    for h in lst:
        n = "opt_" + h
        out += [":%s" % n]
        dfly_opt_header(name = n, opt = h)
    native.filegroup(
        name = name,
        srcs = out,
    )
