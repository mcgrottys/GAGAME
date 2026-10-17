# ================================================================================================
#  tools/astindex/astindex.py -- the AST index behind the Scriptorium's refs / callers / callees /
#  layout / cbuffer_reads / includes tools.
#
#  The Scriptorium's own indexer is a regex scan: it finds where a name is DECLARED. It cannot say
#  who calls a function, who writes a field, what offset a member sits at, or which shader reads
#  gWavesB.z -- those need the compiler's resolution of every name. This walks two compilers' ASTs:
#
#    C++   libclang (pip wheel `libclang`), driven by build/compile_commands.json with the MSVC
#          flags translated, the MSVC + SDK include paths, and shims for the builtins the 14.51
#          STL expects of clang 19 (the wheel is clang 18).
#    HLSL  dxc.exe -ast-dump, one dump per .hlsl (its .hlsli come along), with the defines its
#          C++ Compile() calls pass.
#
#  and writes the ast_* tables of schema.sql into tools/Scriptorium/scriptorium.db.
#
#  Incremental: a TU is re-parsed only when its arguments or any file it read (system headers
#  included) changed mtime or size; the HLSL pass reruns whole when any shader file moved (it is
#  about a second). Deterministic: rows are keyed by where they are in the source, and a header's
#  rows are rewritten from whichever TU re-parses it -- the same rows from any of them.
#
#  Usage:  py -3 tools/astindex/astindex.py [--full] [--jobs N] [--cc build/compile_commands.json]
#                                           [--hlsl-only] [--only <substring of a TU path>]
# ================================================================================================
import hashlib
import json
import os
import re
import sqlite3
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normcase(os.path.abspath(os.path.join(HERE, "..", "..")))
DB = os.path.join(ROOT, "tools", "Scriptorium", "scriptorium.db")
SCHEMA = os.path.join(HERE, "schema.sql")
with open(os.path.abspath(__file__), "rb") as _fh:
    SELF_HASH = hashlib.sha1(_fh.read()).hexdigest()
DXC_DEFAULT = r"C:\Program Files (x86)\Windows Kits\10\bin\10.0.26100.0\x64\dxc.exe"


def log(msg):
    print(f"[astindex] {time.strftime('%H:%M:%S')} {msg}", flush=True)


def rel(path):
    """Repo-relative '/' path for a file inside the repo, else None."""
    a = os.path.abspath(path)
    if os.path.normcase(a).startswith(ROOT + os.sep):
        r = a[len(ROOT) + 1:].replace("\\", "/")
        # build/ holds generated files only; the index is of the source.
        return None if r.startswith("build/") else r
    return None


def stamp(path):
    try:
        st = os.stat(path)
        return [st.st_mtime_ns, st.st_size]
    except OSError:
        return [0, -1]


# ---------------------------------------------------------------------------------- C++ (libclang)

# MSVC's 14.51 STL uses builtins clang gained in 19; the wheel is clang 18. Defining them away only
# changes the answers of type traits the engine never asks (is_layout_compatible and kin), and
# __builtin_verbose_trap is the body of a failed _STL_VERIFY.
CLANG_FIXED = [
    "--target=x86_64-pc-windows-msvc", "-x", "c++", "-std=c++20",
    # No -fdelayed-template-parsing, though MSVC behaves that way: delayed bodies are never
    # walked, and the references inside every template would be missing.
    "-fms-compatibility", "-fms-extensions",
    "-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH",
    "-D__builtin_verbose_trap(a,b)=__debugbreak()",
    "-D__is_layout_compatible(a,b)=false",
    "-D__is_pointer_interconvertible_base_of(a,b)=false",
    "-Wno-everything", "-ferror-limit=0",
]


def msvc_include(db):
    """The MSVC + Windows SDK include path. From INCLUDE when the build's vcvars set it (the
    post-build hook runs inside build.bat), else from vcvars64 once, remembered in ast_meta."""
    inc = os.environ.get("INCLUDE", "")
    if "MSVC" in inc:
        db.execute("INSERT OR REPLACE INTO ast_meta VALUES('msvc_include', ?)", (inc,))
        return inc
    row = db.execute("SELECT value FROM ast_meta WHERE key='msvc_include'").fetchone()
    if row:
        return row[0]
    vswhere = os.path.join(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"),
                           "Microsoft Visual Studio", "Installer", "vswhere.exe")
    vs = subprocess.run([vswhere, "-latest", "-products", "*", "-property", "installationPath"],
                        capture_output=True, text=True).stdout.strip()
    bat = os.path.join(vs, "VC", "Auxiliary", "Build", "vcvars64.bat")
    out = subprocess.run(f'cmd /c ""{bat}" >nul && set INCLUDE"', capture_output=True, text=True,
                         shell=True).stdout
    for line in out.splitlines():
        if line.startswith("INCLUDE="):
            inc = line[len("INCLUDE="):]
            db.execute("INSERT OR REPLACE INTO ast_meta VALUES('msvc_include', ?)", (inc,))
            return inc
    return ""


def clang_args(command, include):
    """Translate one cl.exe command line: keep -D/-I (either spelling), add the fixed clang flags,
    the MSVC include dirs as system dirs, and the shim dir first."""
    args = list(CLANG_FIXED)
    m = re.search(r"MSVC[\\/]14\.(\d+)", include)
    args.append(f"-fms-compatibility-version=19.{m.group(1) if m else '40'}")
    args.append("-I" + os.path.join(HERE, "shim"))
    for tok in command.split():
        if tok[:2] in ("-D", "/D"):
            args.append("-D" + tok[2:].replace('\\"', '"'))   # -DGA_GIT_REV=\"2a96c5f\" from Ninja
        elif tok[:2] in ("-I", "/I"):
            args.append("-I" + tok[2:])
    for p in include.split(";"):
        if p:
            args += ["-isystem", p]
    return args


FN_KINDS = {"FUNCTION_DECL", "CXX_METHOD", "CONSTRUCTOR", "DESTRUCTOR", "CONVERSION_FUNCTION",
            "FUNCTION_TEMPLATE"}
DECL_KINDS = FN_KINDS | {"STRUCT_DECL", "CLASS_DECL", "UNION_DECL", "ENUM_DECL", "ENUM_CONSTANT_DECL",
                         "FIELD_DECL", "VAR_DECL", "TYPEDEF_DECL", "TYPE_ALIAS_DECL",
                         "CLASS_TEMPLATE", "NAMESPACE"}
REF_EXPRS = {"DECL_REF_EXPR", "MEMBER_REF_EXPR"}
PASS_THROUGH = {"UNEXPOSED_EXPR", "PAREN_EXPR"}
RECORD_KINDS = {"STRUCT_DECL", "CLASS_DECL", "UNION_DECL"}
# tools/codemap's regions and values (parse_tu's guards / ex)
CASTS = {"UNEXPOSED_EXPR", "PAREN_EXPR", "CSTYLE_CAST_EXPR", "CXX_STATIC_CAST_EXPR",
         "CXX_FUNCTIONAL_CAST_EXPR", "CXX_CONST_CAST_EXPR"}
TYPEISH = {"TYPE_REF", "TEMPLATE_REF", "NAMESPACE_REF"}
OPS = {"==", "!=", "<", "<=", ">", ">=", "&&", "||", "+", "-", "*", "/", "%", "&", "|", "^"}
EXIT_KINDS = {"RETURN_STMT", "BREAK_STMT", "CONTINUE_STMT", "GOTO_STMT", "CXX_THROW_EXPR"}
LOOP_KINDS = {"WHILE_STMT", "FOR_STMT", "CXX_FOR_RANGE_STMT", "DO_STMT"}
# clang's CXBinaryOperatorKind / CXUnaryOperatorKind (clang-c/Index.h)
BIN_OPS = {3: "*", 4: "/", 5: "%", 6: "+", 7: "-", 8: "<<", 9: ">>", 11: "<", 12: ">", 13: "<=", 14: ">=",
           15: "==", 16: "!=", 17: "&", 18: "^", 19: "|", 20: "&&", 21: "||", 22: "=", 23: "*=", 24: "/=",
           25: "%=", 26: "+=", 27: "-=", 28: "<<=", 29: ">>=", 30: "&=", 31: "^=", 32: "|=", 33: ","}
STL_BUILDS = {"emplace_back", "emplace", "emplace_front", "emplace_hint", "try_emplace", "make_unique",
              "make_shared", "make_optional", "construct_at"}
UN_OPS = {1: "++", 2: "--", 3: "++", 4: "--", 5: "&", 6: "*", 7: "+", 8: "-", 9: "~", 10: "!"}
CPP_TABLES = ("ast_decls", "ast_refs", "ast_layouts", "ast_fields", "ast_includes", "ast_regions",
              "ast_values", "ast_litcalls")
GUARD_KINDS = {"IF_STMT", "CONDITIONAL_OPERATOR", "BINARY_OPERATOR", "SWITCH_STMT", "COMPOUND_STMT"} | LOOP_KINDS


def parse_tu(job):
    """Worker: parse one TU and return its rows for every repo file it sees. Runs in a pool
    process, so it imports libclang itself and shares nothing."""
    import clang.cindex as ci
    sys.setrecursionlimit(20000)
    path, args = job
    t0 = time.perf_counter()
    index = ci.Index.create()
    # 0x200 = CXTranslationUnit_KeepGoing: an error in a system header must not end the parse.
    tu = index.parse(path, args, options=0x200)
    t_parse = time.perf_counter() - t0
    # Read every diagnostic's fields at once: a Diagnostic outlives its iterator badly in the
    # clang 18 bindings (access violation on a second pass).
    diags = [(d.severity, d.location.file.name if d.location.file else "", d.location.line, d.spelling)
             for d in tu.diagnostics]
    K = ci.CursorKind

    relcache = {}

    def frel(f):
        if f is None:
            return None
        n = f.name
        r = relcache.get(n, 0)
        if r == 0:
            r = relcache[n] = rel(n)
        return r

    srccache = {}

    def src(r):
        b = srccache.get(r)
        if b is None:
            try:
                with open(os.path.join(ROOT, r), "rb") as fh:
                    b = fh.read()
            except OSError:
                b = b""
            srccache[r] = b
        return b

    def text_between(r, a, b):
        """The source between two offsets (a fallback; bin_op / un_op ask clang first)."""
        s = src(r)
        if 0 <= a <= b <= len(s) and b - a < 64:
            return s[a:b].decode("utf-8", "replace").strip()
        return ""

    # An operator's kind from clang itself (the C API has had it since clang 17; the Python
    # bindings of 18 do not wrap it). Reading the source between the operands, as this did before,
    # fails inside a macro's arguments: GA_CHECK(q->GetTimestampFrequency(&freq)) read `&freq` as a
    # plain read, so the write through the pointer was invisible.
    def bin_op(c, r, ch):
        k = lib.clang_getCursorBinaryOperatorKind(c)
        return BIN_OPS.get(k) or text_between(r, ch[0].extent.end.offset, ch[1].extent.start.offset)

    def un_op(c, r, ch):
        k = lib.clang_getCursorUnaryOperatorKind(c)
        return UN_OPS.get(k) or (text_between(r, c.extent.start.offset, ch.extent.start.offset)
                                 or text_between(r, ch.extent.end.offset, c.extent.end.offset))

    decls, refs, layouts, fields = [], [], [], []
    files_seen = set()

    def qual(c):
        parts = []
        p = c
        while p is not None and p.kind != K.TRANSLATION_UNIT:
            if p.spelling:
                parts.append(p.spelling)
            p = p.semantic_parent
        return "::".join(reversed(parts))

    def add_ref(c, target, kind, container, r, detail=""):
        if frel(target.location.file) is None:     # into std:: or the SDK: not the engine's
            return
        loc = c.location
        refs.append((target.get_usr(), target.spelling, r, loc.line, loc.column, kind, container, detail, "cpp"))

    # A call in a template whose arguments depend on its parameters is not resolved until it is
    # instantiated, and the cursor tree holds the pattern: clang keeps the candidate set instead
    # (an OVERLOADED_DECL_REF). The clang 18 bindings do not expose it; the C API does. Each
    # candidate is recorded with detail 'candidate' -- without this, SweCycle.h's
    # LogWestBoundary, called only from a template, read as never called.
    import ctypes
    lib = ci.conf.lib
    lib.clang_getNumOverloadedDecls.argtypes = [ci.Cursor]
    lib.clang_getNumOverloadedDecls.restype = ctypes.c_uint
    lib.clang_getOverloadedDecl.argtypes = [ci.Cursor, ctypes.c_uint]
    lib.clang_getOverloadedDecl.restype = ci.Cursor
    lib.clang_getOverloadedDecl.errcheck = ci.Cursor.from_result

    lib.clang_getCursorBinaryOperatorKind.argtypes = [ci.Cursor]
    lib.clang_getCursorBinaryOperatorKind.restype = ctypes.c_int
    lib.clang_getCursorUnaryOperatorKind.argtypes = [ci.Cursor]
    lib.clang_getCursorUnaryOperatorKind.restype = ctypes.c_int

    def candidates(c):
        return [lib.clang_getOverloadedDecl(c, i) for i in range(lib.clang_getNumOverloadedDecls(c))]

    def unwrap(c):
        while c.kind.name in PASS_THROUGH:
            ch = list(c.get_children())
            if len(ch) != 1:
                return c
            c = ch[0]
        return c

    def is_def(c, kn):
        # A '= default' member is a definition in the TUs that use it and a declaration in the
        # rest; asked of the syntax instead, the answer is the same from every TU.
        return c.is_definition() or (kn in FN_KINDS and kn != "FUNCTION_TEMPLATE" and c.is_default_method())

    # ---- tools/codemap: the conditions code runs under, the values declarations take, and the calls
    # that name something by a string (schema.sql: ast_regions, ast_values, ast_litcalls).
    regions, values, litcalls = [], [], []

    def span(r, a, b):
        s = src(r)
        return s[a:b].decode("utf-8", "replace") if 0 <= a <= b <= len(s) else ""

    def peel(c):
        """Through parentheses, casts and the implicit wrappers clang 18 leaves unexposed, as long
        as each holds exactly one expression."""
        while c.kind.name in CASTS:
            ch = [x for x in c.get_children() if x.kind.name not in TYPEISH]
            if len(ch) != 1:
                return c
            c = ch[0]
        return c

    def first_token(c):
        for t in c.get_tokens():
            return t.spelling
        return None

    def ex(c, d=0):
        """An expression as JSON-ready data (the grammar in schema.sql)."""
        c = peel(c)
        kn = c.kind.name
        if d > 8:
            return ["?"]
        if kn == "INTEGER_LITERAL":
            t = (first_token(c) or "").rstrip("uUlL").replace("'", "")
            try:
                return int(t, 0)
            except ValueError:
                return ["?"]
        if kn == "FLOATING_LITERAL":
            try:
                return float((first_token(c) or "").rstrip("fFlL"))
            except ValueError:
                return ["?"]
        if kn == "CXX_BOOL_LITERAL_EXPR":
            return first_token(c) == "true"
        if kn == "CXX_NULL_PTR_LITERAL_EXPR":
            return None
        if kn == "STRING_LITERAL":
            parts = [t.spelling for t in c.get_tokens()]
            out = []
            for p in parts:
                m = re.match(r'^(?:L|u8|u|U)?"(.*)"$', p, re.S)
                if m:
                    out.append(m.group(1).replace('\\"', '"').replace("\\\\", "\\"))
            return "".join(out)
        if kn in REF_EXPRS:
            t = c.referenced
            if t is None:
                return ["?"]
            tk = t.kind.name
            if tk == "FIELD_DECL":
                return ["f", t.get_usr()]
            if tk == "VAR_DECL":
                p = t.semantic_parent
                return ["l" if p is not None and p.kind.name in FN_KINDS else "g", t.get_usr()]
            if tk == "PARM_DECL":
                return ["p", t.get_usr()]
            if tk == "ENUM_CONSTANT_DECL":
                return t.enum_value
            if tk in FN_KINDS:
                return ["fn", t.get_usr()]
            return ["?"]
        ch = list(c.get_children())
        if kn == "BINARY_OPERATOR" and len(ch) == 2:
            op = bin_op(c, r_cur[0], ch)
            return [op, ex(ch[0], d + 1), ex(ch[1], d + 1)] if op in OPS else ["?"]
        if kn == "UNARY_OPERATOR" and len(ch) == 1:
            op = {"!": "!", "-": "neg", "&": "&", "*": "*"}.get(un_op(c, r_cur[0], ch[0]))
            return [op, ex(ch[0], d + 1)] if op else ["?"]
        if kn == "CONDITIONAL_OPERATOR" and len(ch) == 3:
            return ["?:"] + [ex(x, d + 1) for x in ch]
        if kn == "ARRAY_SUBSCRIPT_EXPR" and len(ch) == 2:
            return ["[]", ex(ch[0], d + 1), ex(ch[1], d + 1)]
        if kn == "INIT_LIST_EXPR" and len(ch) <= 32:      # BindEnum's names: {"chart", "world", "gulf"}
            return ["{}"] + [ex(x, d + 1) for x in ch]
        if kn == "CALL_EXPR":
            t = c.referenced
            if t is None:
                return ["?"]
            obj, args = call_parts(c, t, ch)
            name = t.spelling
            if t.kind.name == "CONSTRUCTOR" and len(args) == 1:
                return ex(args[0], d + 1)                    # a conversion: std::string("mars")
            if name.startswith("operator"):
                op = name[8:].strip()
                xs = ([obj] if obj is not None else []) + args
                if op in OPS and len(xs) == 2:
                    return [op, ex(xs[0], d + 1), ex(xs[1], d + 1)]
                if op == "!" and len(xs) == 1:
                    return ["!", ex(xs[0], d + 1)]
                if op == "bool" and len(xs) == 1:
                    return ["bool", ex(xs[0], d + 1)]
                if op in ("->", "*") and len(xs) == 1:
                    return ex(xs[0], d + 1)
            return ["call", t.get_usr(), ex(obj, d + 1) if obj is not None else None] + [ex(a, d + 1) for a in args]
        return ["?"]

    def call_parts(c, t, ch):
        """(object, arguments) of a call: the callee expression is dropped (a member call's object
        is the child of its MEMBER_REF_EXPR), an operator call's operands are its arguments."""
        tusr = t.get_usr()
        obj, args, seen, before = None, [], False, 0
        for x in ch:
            u = peel(x)
            if not seen and u.kind.name in REF_EXPRS and u.referenced is not None and u.referenced.get_usr() == tusr:
                seen, before = True, len(args)
                sub = [y for y in u.get_children() if y.kind.name not in TYPEISH]
                if u.kind.name == "MEMBER_REF_EXPR" and sub:
                    obj = sub[0]
                continue
            args.append(x)
        # An operator called on an object -- `find(a, b)` with find a lambda, `m[k]`, `a == b` as a
        # member -- lists the object FIRST, before the operator's own reference (or with no such
        # reference at all for operator()). Set aside, the arguments line up with the parameters
        # (a by-reference out-parameter was read one place off before).
        if obj is None and args and t.kind.name == "CXX_METHOD" and not t.is_static_method() \
                and t.spelling.startswith("operator") and (before > 0 or not seen):
            obj = args.pop(0)
        return obj, args

    def targets(c):
        """Every (usr, detail) an lvalue may name: `(side == 0 ? aP : aS) = a` writes either arm."""
        c = peel(c)
        if c.kind.name == "CONDITIONAL_OPERATOR":
            ch = list(c.get_children())
            if len(ch) == 3:
                return targets(ch[1]) + targets(ch[2])
        return [target(c)]

    def target(c):
        """(usr, detail) of the declaration an lvalue names: the field at the end of a.b.c, a
        variable, either through a subscript (detail '[i]')."""
        c = peel(c)
        kn = c.kind.name
        if kn == "ARRAY_SUBSCRIPT_EXPR":
            ch = list(c.get_children())
            if len(ch) == 2:
                u, _ = target(ch[0])
                i = ex(ch[1])
                return u, f"[{i}]" if isinstance(i, int) else "[?]"
        if kn == "CALL_EXPR" and c.referenced is not None and c.referenced.spelling in ("operator[]", "operator*", "operator->"):
            obj, args = call_parts(c, c.referenced, list(c.get_children()))
            if obj is not None:
                return target(obj)[0], "[?]"
        if kn in REF_EXPRS and c.referenced is not None and c.referenced.kind.name in ("FIELD_DECL", "VAR_DECL"):
            return c.referenced.get_usr(), ""
        return None, ""

    def value(usr, c, kind, expr, container, r, detail=""):
        if isinstance(usr, list):          # targets(): every arm
            for u, det in usr:
                value(u, c, kind, expr, container, r, det or detail)
            return
        if usr:
            values.append((usr, r, c.location.line, kind, json.dumps(expr, separators=(",", ":")),
                           container, detail, "cpp"))

    def init_expr(c):
        """A declaration's initializer, or None. The children alone cannot say (`float a[4];` has the
        4 as a child): the source after the name must open one with '=', '{' or '('."""
        tail = span(r_cur[0], c.location.offset, c.extent.end.offset)
        if not re.match(r"^[\w:~]*\s*(?:\[[^\]]*\]\s*)*[={(]", tail):
            return None
        ch = [x for x in c.get_children() if x.kind.name not in TYPEISH and x.kind.name != "PARM_DECL"]
        return ch[-1] if ch else None

    def region(r, a, b, cond, pol, container):
        regions.append((r, a.line, a.column, b.line, b.column,
                        json.dumps(cond, separators=(",", ":")), pol, container, "cpp"))

    def exits(c):
        kn = c.kind.name
        if kn in EXIT_KINDS:
            return True
        if kn == "COMPOUND_STMT":
            ch = list(c.get_children())
            return bool(ch) and exits(ch[-1])
        if kn == "IF_STMT":
            p = if_parts(c)
            return p is not None and p[2] is not None and exits(p[1]) and exits(p[2])
        return False

    def if_parts(c):
        """(cond, then, else-or-None) of an if, with or without an init statement. The children
        alone are ambiguous at three (init, cond, then | cond, then, else): an `else` between the
        last two decides."""
        ch = list(c.get_children())
        if len(ch) < 2:
            return None
        r = r_cur[0]
        has_else = len(ch) >= 3 and re.search(r"\belse\b", span(r, ch[-2].extent.end.offset, ch[-1].extent.start.offset))
        if has_else:
            return ch[-3], ch[-2], ch[-1]
        return ch[-2], ch[-1], None

    def cond_of(c):
        """A condition: an expression, or a declared condition variable (`if (auto* x = f())`)."""
        if c.kind.name == "VAR_DECL":
            return ["l", c.get_usr()]
        return ex(c)

    def guards(c, kn, container, r):
        """The regions a statement or expression opens (schema.sql ast_regions)."""
        if kn == "IF_STMT":
            p = if_parts(c)
            if p:
                cond = cond_of(p[0])
                region(r, p[1].extent.start, p[1].extent.end, cond, 1, container)
                if p[2] is not None:
                    region(r, p[2].extent.start, p[2].extent.end, cond, -1, container)
        elif kn == "CONDITIONAL_OPERATOR":
            ch = list(c.get_children())
            if len(ch) == 3:
                cond = ex(ch[0])
                region(r, ch[1].extent.start, ch[1].extent.end, cond, 1, container)
                region(r, ch[2].extent.start, ch[2].extent.end, cond, -1, container)
        elif kn == "BINARY_OPERATOR":
            ch = list(c.get_children())
            if len(ch) == 2:
                op = bin_op(c, r, ch)
                if op in ("&&", "||"):
                    region(r, ch[1].extent.start, ch[1].extent.end, ex(ch[0]), 1 if op == "&&" else -1, container)
        elif kn in LOOP_KINDS:
            ch = list(c.get_children())
            if ch:
                body = ch[0] if kn == "DO_STMT" else ch[-1]
                region(r, body.extent.start, body.extent.end, "loop", 0, container)
                if kn == "WHILE_STMT" and len(ch) >= 2:
                    region(r, body.extent.start, body.extent.end, cond_of(ch[-2]), 1, container)
        elif kn == "SWITCH_STMT":
            ch = list(c.get_children())
            if len(ch) >= 2 and ch[-1].kind.name == "COMPOUND_STMT":
                sel = cond_of(ch[-2])
                vals, live, dflt = [], False, False
                for s in ch[-1].get_children():
                    stmt = s
                    if s.kind.name in ("CASE_STMT", "DEFAULT_STMT"):
                        if not live:
                            vals, dflt = [], False
                        live = True
                        while stmt is not None and stmt.kind.name in ("CASE_STMT", "DEFAULT_STMT"):
                            sc = list(stmt.get_children())
                            if stmt.kind.name == "CASE_STMT":
                                vals.append(ex(sc[0]) if sc else ["?"])
                            else:
                                dflt = True
                            stmt = sc[-1] if len(sc) >= (2 if stmt.kind.name == "CASE_STMT" else 1) else None
                    if stmt is None or not live:
                        continue
                    if not dflt:           # `default:` runs for what no case named: left unguarded
                        region(r, stmt.extent.start, stmt.extent.end, ["in", sel] + vals, 1, container)
                    if exits(stmt):
                        live = False
        elif kn == "COMPOUND_STMT":
            ch = list(c.get_children())
            for i in range(len(ch) - 1):
                if ch[i].kind.name != "IF_STMT":
                    continue
                p = if_parts(ch[i])
                if not p:
                    continue
                te, ee = exits(p[1]), p[2] is not None and exits(p[2])
                if te != ee:       # one arm leaves the block: the rest runs only on the other
                    region(r, ch[i + 1].extent.start, c.extent.end, cond_of(p[0]), -1 if te else 1, container)

    r_cur = [None]       # the file being walked, for ex()'s operator spelling

    def visit(c, ctx, container, r):
        kn = c.kind.name
        children = None
        r_cur[0] = r
        if container and kn in GUARD_KINDS:
            guards(c, kn, container, r)
        if kn == "VAR_DECL":
            e = init_expr(c)
            p = c.semantic_parent
            if e is not None:
                local = p is not None and p.kind.name in FN_KINDS
                # 'ref': `auto& mode = m_mode;` -- a write through it is a write to what it names
                tk = c.type.kind
                ref = tk in (ci.TypeKind.LVALUEREFERENCE, ci.TypeKind.RVALUEREFERENCE)
                value(c.get_usr(), c, "local" if local else "init", ex(e), container, r,
                      "ref" if ref else "const" if c.type.is_const_qualified() else "")
        elif kn == "FIELD_DECL" and not c.is_bitfield():
            e = init_expr(c)
            if e is not None:
                value(c.get_usr(), c, "default", ex(e), container, r)
        elif kn == "CONSTRUCTOR" and c.is_definition():
            ch = list(c.get_children())
            for i, x in enumerate(ch[:-1]):
                if x.kind.name == "MEMBER_REF" and x.referenced is not None:
                    value(x.referenced.get_usr(), x, "assign", ex(ch[i + 1]), c.get_usr(), r)
        elif kn == "INIT_LIST_EXPR":
            # canonical: push_back({...}) types the list as the container's value_type, a typedef
            d = c.type.get_canonical().get_declaration()
            if d is not None and d.kind.name in RECORD_KINDS and any(True for _ in c.get_children()):
                value(d.get_usr(), c, "aggregate", ["?"], container, r)
        elif kn == "CALL_EXPR":
            t = c.referenced
            if t is not None and t.kind.name in FN_KINDS:
                ch = list(c.get_children())
                obj, args = call_parts(c, t, ch)
                name = t.spelling
                if name == "operator=" or (name.startswith("operator") and name.endswith("=") and name[8:] not in ("==", "!=", "<=", ">=")):
                    xs = ([obj] if obj is not None else []) + args
                    if len(xs) == 2:
                        value(targets(xs[0]), c, "assign", ex(xs[1]) if name == "operator=" else ["?"], container, r)
                else:
                    # a non-const member call may change its object (ptr.reset(), name.clear())
                    if obj is not None and t.kind.name != "CONSTRUCTOR" and not t.is_const_method() \
                            and not t.is_static_method():
                        value(targets(obj), c, "byref", ["?"], container, r, name)
                    try:
                        params = list(t.get_arguments())
                    except Exception:
                        params = []
                    for a, p in zip(args, params):
                        pt = p.type
                        if pt.kind == ci.TypeKind.LVALUEREFERENCE and not pt.get_pointee().is_const_qualified():
                            value(targets(a), a, "byref", ["?"], container, r, name)
                # A record built INSIDE the STL from arguments -- v.emplace_back(a, b),
                # make_unique<T>(a), m.try_emplace(k, a) -- gets its fields where no repo code
                # writes them: every record among the template arguments counts as built.
                if name in STL_BUILDS and args:
                    ty = (c.type if name.startswith("make_") or obj is None else obj.type).get_canonical()
                    for k_ in range(max(0, ty.get_num_template_arguments())):
                        d = ty.get_template_argument_type(k_).get_canonical().get_declaration()
                        if d is not None and d.kind.name in RECORD_KINDS:
                            value(d.get_usr(), c, "aggregate", ["?"], container, r, name)
                if any(peel(a).kind.name == "STRING_LITERAL" for a in args):
                    loc = c.location
                    litcalls.append((t.get_usr(), name, r, loc.line, loc.column, container,
                                     json.dumps([ex(a) for a in args], separators=(",", ":")), "cpp"))
        elif kn == "COMPOUND_ASSIGNMENT_OPERATOR":
            ch = list(c.get_children())
            if len(ch) == 2:
                value(targets(ch[0]), c, "assign", ["?"], container, r)
        elif kn == "BINARY_OPERATOR":
            ch = list(c.get_children())
            if len(ch) == 2 and bin_op(c, r, ch) == "=":
                value(targets(ch[0]), c, "assign", ex(ch[1]), container, r)
        elif kn == "UNARY_OPERATOR":
            ch = list(c.get_children())
            if len(ch) == 1:
                op = un_op(c, r, ch[0])
                if op in ("++", "--"):
                    value(targets(ch[0]), c, "assign", ["?"], container, r)
                elif op == "&":       # its address taken: it may be written through the pointer
                    value(targets(ch[0]), c, "byref", ["?"], container, r, "&")
        if kn in DECL_KINDS:
            loc = c.location
            parent = c.semantic_parent
            local = parent is not None and parent.kind.name in FN_KINDS
            # A struct declared inside a function IS kept: cbuffer mirrors are often local
            # (TileAtlas.cpp's 'struct Cb', SkyLayer.cpp's LutCb); local variables are not.
            if (not local or kn in RECORD_KINDS or kn == "FIELD_DECL") and kn != "NAMESPACE":
                usr = c.get_usr()
                flags = []
                if kn in FN_KINDS:
                    if c.is_virtual_method():
                        flags.append("virtual")
                    if c.is_static_method():
                        flags.append("static")
                    if c.is_pure_virtual_method():
                        flags.append("pure")
                acc = c.access_specifier.name
                if acc in ("PRIVATE", "PROTECTED"):
                    flags.append(acc.lower())
                decls.append((usr, kn.lower(), c.spelling, qual(c), r, loc.line, loc.column,
                              c.extent.end.line, c.type.spelling,
                              parent.get_usr() if parent is not None else "",
                              1 if is_def(c, kn) else 0, ",".join(flags), "cpp"))
                if kn in RECORD_KINDS and c.is_definition():
                    t = c.type
                    size = t.get_size()
                    if size > 0:      # negative: dependent or incomplete (a template pattern)
                        layouts.append((usr, c.spelling, qual(c), r, loc.line, size, t.get_align(), "cpp"))
                        ordn = 0
                        for f in c.get_children():
                            if f.kind == K.FIELD_DECL:
                                off = t.get_offset(f.spelling) if f.spelling else -1
                                fields.append((usr, ordn, f.spelling, f.type.spelling,
                                               off // 8 if off >= 0 else -1, f.type.get_size(),
                                               r, f.location.line, "cpp"))
                                ordn += 1
            if kn in FN_KINDS and c.is_definition():
                container = c.get_usr()
        elif kn in REF_EXPRS and c.referenced is not None and c.referenced.kind.name == "OVERLOADED_DECL_REF":
            for t in candidates(c.referenced):
                if t is not None and t.kind.name in FN_KINDS:
                    add_ref(c, t, "call" if ctx == "callee" else "addr", container, r, "candidate")
            return
        elif kn in REF_EXPRS:
            t = c.referenced
            if t is not None:
                tk = t.kind.name
                if ctx == "callee" and tk in FN_KINDS:
                    kind = "call"
                elif tk in FN_KINDS:
                    kind = "addr"
                elif ctx in ("write", "addr"):
                    kind = ctx
                else:
                    kind = "read"
                add_ref(c, t, kind, container, r)
        elif kn == "OVERLOADED_DECL_REF":
            for t in candidates(c):
                if t is not None and t.kind.name in FN_KINDS:
                    add_ref(c, t, "call" if ctx == "callee" else "addr", container, r, "candidate")
        elif kn == "TYPE_REF" or kn == "TEMPLATE_REF":
            t = c.referenced
            if t is not None:
                add_ref(c, t, "type", container, r)
        elif kn == "MEMBER_REF":     # a member named in a constructor's initializer list
            t = c.referenced
            if t is not None:
                add_ref(c, t, "write", container, r)
        elif kn == "CALL_EXPR":
            t = c.referenced
            children = list(c.get_children())
            tusr = t.get_usr() if t is not None else None
            callee_seen = False
            for ch in children:
                u = unwrap(ch)
                is_callee = not callee_seen and (
                    (tusr is not None and u.kind.name in REF_EXPRS
                     and u.referenced is not None and u.referenced.get_usr() == tusr)
                    or (tusr is None and (u.kind.name == "OVERLOADED_DECL_REF" or (
                        u.kind.name in REF_EXPRS and u.referenced is not None
                        and u.referenced.kind.name == "OVERLOADED_DECL_REF"))))
                callee_seen = callee_seen or is_callee
                visit(ch, "callee" if is_callee else None, container, r)
            # A constructor call names no function in the source (T{...}, T(...), implicit):
            # the CALL_EXPR itself is the only place it is called from.
            if t is not None and not callee_seen and t.kind.name in FN_KINDS:
                add_ref(c, t, "call", container, r)
            return
        elif kn == "BINARY_OPERATOR" or kn == "COMPOUND_ASSIGNMENT_OPERATOR":
            children = list(c.get_children())
            if len(children) == 2:
                lhs, rhs = children
                write = kn == "COMPOUND_ASSIGNMENT_OPERATOR"
                if not write:
                    write = bin_op(c, r, children) == "="
                visit(lhs, "write" if write else None, container, r)
                visit(rhs, None, container, r)
                return
        elif kn == "UNARY_OPERATOR":
            children = list(c.get_children())
            if len(children) == 1:
                ch = children[0]
                op = un_op(c, r, ch)
                sub = "write" if op in ("++", "--") else "addr" if op == "&" else None
                visit(ch, sub, container, r)
                return
        elif kn == "CXX_BASE_SPECIFIER":
            t = c.referenced
            if t is not None:
                add_ref(c, t, "base", container, r)
        if children is None:
            children = c.get_children()
        sub = ctx if kn in PASS_THROUGH else None
        for ch in children:
            visit(ch, sub, container, r)

    for top in tu.cursor.get_children():
        r = frel(top.location.file)
        if r is None:
            continue
        files_seen.add(r)
        visit(top, None, "", r)

    deps = {os.path.normcase(os.path.abspath(path))}
    for inc in tu.get_includes():
        inc_abs = os.path.abspath(inc.include.name)
        deps.add(os.path.normcase(inc_abs))
        for f in (inc.source, inc.include):
            fr = frel(f)
            if fr is not None:
                files_seen.add(fr)
    # The include graph is read from each repo file's #include lines, resolved the way the
    # compiler searches (the includer's directory, then -I, then the system dirs). libclang's
    # inclusion list is NOT used for edges: it omits a header's includes when #pragma once already
    # took them in, so the edges a header got depended on which TU reached it first.
    search = [a[2:] for a in args if a.startswith("-I")] + [args[i + 1] for i, a in enumerate(args) if a == "-isystem"]
    includes = []
    for fr in sorted(files_seen):
        for m in re.finditer(rb'^[ \t]*#[ \t]*include[ \t]*[<"]([^>"]+)[>"]', src(fr), re.M):
            name = m.group(1).decode("utf-8", "replace")
            dirs = ([os.path.dirname(os.path.join(ROOT, fr))] if m.group(0).rstrip().endswith(b'"') else []) + search
            hit = next((os.path.abspath(os.path.join(d, name)) for d in dirs if os.path.isfile(os.path.join(d, name))), None)
            target = (rel(hit) or hit.replace("\\", "/")) if hit else "<" + name + ">"
            includes.append((fr, target, "cpp"))
    main_r = rel(path)
    if main_r:
        files_seen.add(main_r)
    return {
        "tu": main_r, "path": path, "parse_s": t_parse, "total_s": time.perf_counter() - t0,
        "errors": sum(1 for d in diags if d[0] >= 3),
        "errors_in_repo": [d for d in diags if d[0] >= 3 and d[1] and rel(d[1])],
        "diags": len(diags), "deps": sorted(deps), "files": sorted(files_seen),
        "decls": decls, "refs": refs, "layouts": layouts, "fields": fields, "includes": includes,
        "regions": regions, "values": values, "litcalls": litcalls,
    }


# ---------------------------------------------------------------------------------- HLSL (dxc)

COMPILE_RE = re.compile(r'Compile\(([^;]*?)L"([a-z]s_\d_\d)"\s*(?:,\s*\{([^}]*)\})?')
LIT_RE = re.compile(r'L"([A-Za-z_]\w*)"')
STAGE_PREFIX = {"Vs": "vs", "Ps": "ps", "Cs": "cs", "Ms": "ms", "As": "as", "Gs": "gs", "Hs": "hs", "Ds": "ds"}


def hlsl_entries_from_cpp():
    """Every wide string literal in src/ that names an identifier, with where it appears, and the
    defines each shader file is compiled with (the {L"X=1"} argument of a Compile call that names
    the file). The entry list is not taken from Compile() alone because entries also travel
    through tables ({L"CsFoo", ...}) and variables."""
    literals = {}
    defines = {}
    for dp, _, fs in os.walk(os.path.join(ROOT, "src")):
        for f in sorted(fs):
            if not f.endswith((".cpp", ".h")):
                continue
            p = os.path.join(dp, f)
            with open(p, encoding="utf-8", errors="replace") as fh:
                text = fh.read()
            r = rel(p)
            for m in LIT_RE.finditer(text):
                line = text.count("\n", 0, m.start()) + 1
                literals.setdefault(m.group(1), []).append(f"{r}:{line}")
            for m in COMPILE_RE.finditer(text):
                if m.group(3):
                    fm = re.search(r'(\w+\.hlsl)', m.group(1))
                    if fm:
                        for d in re.findall(r'L"([^"]+)"', m.group(3)):
                            defines.setdefault(fm.group(1), set()).add(d)
    return literals, defines


LOC_RE = re.compile(r"(line|col):(\d+)(?::(\d+))?|([^\s<>,']+\.hlsli?):(\d+):(\d+)")
NODE_RE = re.compile(r"^([A-Za-z]+) (0x[0-9a-f]+)")


def strip_qual(t):
    return re.sub(r"\s+", " ", re.sub(r"\b(const|row_major|column_major|static|uniform|volatile)\b", "", t)).strip()


def hlsl_type_size(t, structs, cbuffer):
    """Bytes an HLSL type takes under cbuffer packing (cbuffer=True) or StructuredBuffer packing.
    None for a type this does not model (resources, templates)."""
    row_major = "row_major" in t
    t = strip_qual(t)
    arr = re.match(r"^(.*?)\s*\[(\d+)\]$", t)
    if arr:
        inner = hlsl_type_size(("row_major " if row_major else "") + arr.group(1), structs, cbuffer)
        if inner is None:
            return None
        n = int(arr.group(2))
        if cbuffer:   # every element starts a 16-byte row
            return (inner + 15) // 16 * 16 * (n - 1) + inner
        return inner * n
    base = {"float": 4, "int": 4, "uint": 4, "bool": 4, "half": 4, "double": 8, "dword": 4,
            "min16float": 4, "min16int": 4, "min16uint": 4, "int64_t": 8, "uint64_t": 8,
            "unsigned int": 4}
    m = re.match(r"^(float|int|uint|bool|half|double|min16float|min16int|min16uint)(\d)?(?:x(\d))?$", t)
    if m:
        s = base[m.group(1)]
        if m.group(3):    # matrix RxC: row_major packs R rows of C; the default packs C columns of R
            rows, cols = int(m.group(2)), int(m.group(3))
            if not row_major:
                rows, cols = cols, rows
            return (16 * (rows - 1) + s * cols) if cbuffer else rows * cols * s
        return s * int(m.group(2) or 1)
    m = re.match(r"^vector<([\w ]+), (\d)>$", t)
    if m:
        return base.get(m.group(1), 4) * int(m.group(2))
    if t in base:
        return base[t]
    if t in structs:
        return struct_layout(structs[t], structs, cbuffer)[0]
    return None


def struct_layout(fields, structs, cbuffer):
    """Pack (name, type) fields; returns (size, [(name, type, offset, size)]). cbuffer rules:
    16-byte rows, a member may not straddle a row, arrays / structs / matrices start a row.
    StructuredBuffer rules: 4-byte alignment (8 for double), no rows."""
    off, out = 0, []
    for name, t in fields:
        s = hlsl_type_size(t, structs, cbuffer)
        if s is None:
            out.append((name, t, -1, -1))
            continue
        tt = strip_qual(t)
        aggregate = bool(re.search(r"\[\d+\]$", tt)) or tt in structs or bool(re.search(r"\d+x\d+$", tt))
        if cbuffer:
            if aggregate or off // 16 != (off + min(s, 16) - 1) // 16:
                off = (off + 15) // 16 * 16
        else:
            al = 8 if "double" in tt else 4
            off = (off + al - 1) // al * al
        out.append((name, t, off, s))
        off += s
    return off, out


def dump_hlsl(job):
    dxc, f, defines = job
    args = [dxc, "-ast-dump", "-T", "lib_6_6", "-HV", "2021", "-I", "shaders", f]
    for d in defines:
        args += ["-D", d]
    t0 = time.perf_counter()
    p = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace")
    return f, p.stdout, p.stderr, p.returncode, time.perf_counter() - t0


def parse_hlsl_dump(text, out):
    """Walk dxc's text AST. clang prints a location's file only when it changes and its line only
    when that changes, in dump order, so the walk carries the last file and line forward."""
    cur_file, cur_line = None, 0
    stack = []        # stack[depth] = (kind, rest)
    addr = {}         # decl address -> (usr, name[, cbuffer])
    fn_stack = []     # (depth, usr)
    cb_name, cb_depth = None, -1
    struct_name, struct_depth = None, -1
    for raw in text.splitlines():
        i = 0
        while i < len(raw) and raw[i] in "|`- ":
            i += 1
        depth = i // 2
        body = raw[i:]
        m = NODE_RE.match(body)
        if not m:
            continue
        kind, a = m.group(1), m.group(2)
        rest = body[m.end():]
        q = rest.find("'")
        locpart = rest if q < 0 else rest[:q]
        gt = locpart.find(">")
        own, end_line, in_range = None, 0, []
        for lm in LOC_RE.finditer(locpart):
            if lm.group(4):
                cur_file, cur_line = lm.group(4).lstrip("./").replace("\\", "/"), int(lm.group(5))
                loc = (cur_file, cur_line, int(lm.group(6)))
            elif lm.group(1) == "line":
                cur_line = int(lm.group(2))
                loc = (cur_file, cur_line, int(lm.group(3) or 0))
            else:
                loc = (cur_file, cur_line, int(lm.group(2)))
            if gt >= 0 and lm.start() < gt:
                in_range.append(loc)
            elif own is None:
                own = loc     # the node's own location follows its <range>
        if own is None and in_range:
            own = in_range[0]
        end_line = in_range[-1][1] if in_range else (own[1] if own else 0)
        tail = rest[gt + 1:] if gt >= 0 else rest

        del stack[depth:]
        stack.append((kind, rest))
        while fn_stack and fn_stack[-1][0] >= depth:
            fn_stack.pop()
        if cb_depth >= depth:
            cb_name, cb_depth = None, -1
        if struct_depth >= depth:
            struct_name, struct_depth = None, -1

        if kind in ("FunctionDecl", "CXXMethodDecl") and own and own[0]:
            nm = re.search(r"(\w+) '([^']*)'", tail)
            if not nm or " implicit " in rest:
                continue
            name, ftype = nm.group(1), nm.group(2)
            usr = f"hlsl:{name} {ftype}"
            # A function defined in a .hlsl belongs to that shader alone, and eleven shaders each
            # define PsMain(VsOut): without the file in its usr they were one row, the last dumped.
            # A .hlsli's functions are shared by name, so they keep the plain usr.
            if own[0].endswith(".hlsl"):
                usr = f"hlsl:{os.path.basename(own[0])}#{name} {ftype}"
            if kind == "CXXMethodDecl" and struct_name:
                usr = f"hlsl:{struct_name}::{name} {ftype}"
            addr[a] = (usr, name)
            fn_stack.append((depth, usr))
            out["fn_pending"][usr] = (name, ftype, own, end_line)
        elif kind == "CompoundStmt" and fn_stack and fn_stack[-1][0] == depth - 1:
            usr = fn_stack[-1][1]
            name, ftype, own_, end_ = out["fn_pending"][usr]
            out["fns"][usr] = (name, ftype, own_[0], own_[1], own_[2], end_)
        elif kind == "HLSLBufferDecl" and own:
            nm = re.search(r"cbuffer (\w+)", rest)
            if nm:
                cb_name, cb_depth = nm.group(1), depth
                if cb_name not in out["cbuffers"]:
                    out["cbuffers"][cb_name] = {"file": own[0], "line": own[1], "fields": []}
                    out["cb_new"].add(cb_name)
        elif kind == "CXXRecordDecl" and " definition" in rest and depth == 1 and own:
            nm = re.search(r"struct (\w+) definition", rest)
            if nm:
                struct_name, struct_depth = nm.group(1), depth
                if struct_name not in out["structs"]:
                    out["structs"][struct_name] = {"file": own[0], "line": own[1], "fields": []}
                    out["struct_new"].add(struct_name)
        elif kind == "FieldDecl" and struct_name and depth == struct_depth + 1 and own:
            nm = re.search(r"(\w+) '([^']*)'", tail)
            if nm and struct_name in out["struct_new"]:
                out["structs"][struct_name]["fields"].append((nm.group(1), nm.group(2), own[1]))
        elif kind == "VarDecl" and own:
            nm = re.search(r"(\w+) '([^']*)'", tail)
            if not nm:
                continue
            name, vtype = nm.group(1), nm.group(2)
            if cb_name and "parent cbuffer" in rest:
                usr = f"hlsl:cb:{cb_name}.{name}"
                addr[a] = (usr, name, cb_name)
                if cb_name in out["cb_new"]:
                    out["cbuffers"][cb_name]["fields"].append((name, vtype, own[1]))
                out["globals"][usr] = (name, vtype, own[0], own[1], own[2], cb_name)
            elif depth == 1:
                usr = f"hlsl:var:{name}"
                addr[a] = (usr, name, "")
                out["globals"][usr] = (name, vtype, own[0], own[1], own[2], "")
        elif kind == "DeclRefExpr" and fn_stack and own:
            fm = re.search(r"(Function|Var) (0x[0-9a-f]+) '(\w+)'", rest)
            if not fm:
                continue
            caller = fn_stack[-1][1]
            tgt = addr.get(fm.group(2))
            if fm.group(1) == "Function":
                if tgt:
                    out["calls"].add((caller, tgt[0], own[0], own[1], own[2]))
            elif tgt and len(tgt) == 3:
                # the component read: the swizzle / member / subscript wrapped around the name
                comp = ""
                up = depth - 1
                while up >= 0 and stack[up][0] == "ParenExpr":
                    up -= 1
                if up >= 0 and stack[up][0] == "ArraySubscriptExpr":
                    comp = "[]"
                    up -= 1
                    while up >= 0 and stack[up][0] in ("ImplicitCastExpr", "ParenExpr"):
                        up -= 1
                if up >= 0:
                    pk, pr = stack[up]
                    if pk in ("HLSLVectorElementExpr", "ExtMatrixElementExpr"):
                        comp += pr.rsplit(" ", 1)[-1]
                    elif pk == "MemberExpr":
                        mm = re.search(r"\.(\w+) 0x", pr)
                        if mm:
                            comp += "." + mm.group(1)
                out["reads"].add((tgt[0], tgt[1], own[0], own[1], own[2], caller, comp))
        elif kind == "MemberExpr" and fn_stack and own:
            mm = re.search(r"\.(\w+) (0x[0-9a-f]+)", rest)
            if mm and mm.group(2) in addr and len(addr[mm.group(2)]) == 2:
                out["calls"].add((fn_stack[-1][1], addr[mm.group(2)][0], own[0], own[1], own[2]))


def index_hlsl(db, dxc, jobs, full):
    t0 = time.perf_counter()
    shader_dir = os.path.join(ROOT, "shaders")
    files = sorted(f for f in os.listdir(shader_dir) if f.endswith((".hlsl", ".hlsli")))
    literals, defines = hlsl_entries_from_cpp()
    stamps = {f: stamp(os.path.join(shader_dir, f)) for f in files}
    stamps["#defines"] = sorted((k, sorted(v)) for k, v in defines.items())
    stamps["#literals"] = hashlib.sha1(json.dumps(sorted(literals)).encode()).hexdigest()
    stamps["#self"] = SELF_HASH
    key = json.dumps(stamps, sort_keys=True)
    row = db.execute("SELECT value FROM ast_meta WHERE key='hlsl_stamps'").fetchone()
    if row and row[0] == key and not full:
        log("hlsl: no shader changed")
        return
    roots = [f for f in files if f.endswith(".hlsl")]
    # threads, not processes: each worker only waits on a dxc.exe, and spawning Python workers
    # cost more than the dumps (9.0 s vs 3.6 s serial, measured)
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        dumps = list(ex.map(dump_hlsl, [(dxc, "shaders/" + f, sorted(defines.get(f, ()))) for f in roots]))
    t_dump = time.perf_counter() - t0
    for t in ("ast_decls", "ast_refs", "ast_includes"):
        db.execute(f"DELETE FROM {t} WHERE lang='hlsl'")
    for t in ("ast_layouts", "ast_fields"):
        db.execute(f"DELETE FROM {t} WHERE lang LIKE 'hlsl%'")
    db.execute("DELETE FROM ast_hlsl_entries")
    db.execute("DELETE FROM ast_hlsl_reach")
    fns, globals_, cbuffers, structs = {}, {}, {}, {}
    all_calls, all_reads = set(), set()
    failed = []
    for f, text, err, rc, secs in dumps:
        if rc != 0:
            failed.append((f, [l for l in err.splitlines() if "error" in l][:2]))
        out = {"fns": {}, "fn_pending": {}, "globals": {}, "cbuffers": cbuffers, "cb_new": set(),
               "structs": structs, "struct_new": set(), "calls": set(), "reads": set()}
        parse_hlsl_dump(text, out)
        fns.update(out["fns"])
        globals_.update(out["globals"])
        all_calls |= out["calls"]
        all_reads |= out["reads"]
        edges = {}
        for c in out["calls"]:
            edges.setdefault(c[0], set()).add(c[1])
        # entries: functions defined in THIS .hlsl that src/ names by a string literal
        for usr, (name, ftype, ff, line, col, end) in sorted(out["fns"].items()):
            if ff != f or name not in literals or "::" in usr:
                continue
            db.execute("INSERT INTO ast_hlsl_entries VALUES(?,?,?,?,?,?)",
                       (f, name, STAGE_PREFIX.get(name[:2], ""), usr,
                        ",".join(sorted(defines.get(os.path.basename(f), ()))), "; ".join(literals[name][:4])))
            seen, todo = {usr}, [usr]
            while todo:
                u = todo.pop()
                for v in edges.get(u, ()):
                    if v not in seen:
                        seen.add(v)
                        todo.append(v)
            db.executemany("INSERT INTO ast_hlsl_reach VALUES(?,?,?)", [(f, name, u) for u in sorted(seen)])
    for f in files:
        with open(os.path.join(shader_dir, f), encoding="utf-8", errors="replace") as fh:
            for m in re.finditer(r'^\s*#include\s+"([^"]+)"', fh.read(), re.M):
                db.execute("INSERT OR IGNORE INTO ast_includes VALUES(?,?,?)",
                           ("shaders/" + f, "shaders/" + m.group(1), "hlsl"))
    db.executemany("INSERT OR IGNORE INTO ast_decls VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)",
                   [(u, "hlsl-fn", n, u[5:].split(" ", 1)[0], ff, l, c, e, t, "", 1, "", "hlsl")
                    for u, (n, t, ff, l, c, e) in fns.items()])
    db.executemany("INSERT OR IGNORE INTO ast_decls VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)",
                   [(u, "hlsl-cbfield" if cb else "hlsl-global", n, (cb + "." if cb else "") + n, ff, l, c, l, t,
                     "hlsl:cbuffer:" + cb if cb else "", 1, "", "hlsl")
                    for u, (n, t, ff, l, c, cb) in globals_.items()])
    db.executemany("INSERT OR IGNORE INTO ast_refs VALUES(?,?,?,?,?,?,?,?,?)",
                   [(callee, callee[5:].split(" ", 1)[0].split("::")[-1], ff, l, c, "call", caller, "", "hlsl")
                    for caller, callee, ff, l, c in all_calls])
    db.executemany("INSERT OR IGNORE INTO ast_refs VALUES(?,?,?,?,?,?,?,?,?)",
                   [(usr, name, ff, l, c, "read", caller, comp, "hlsl")
                    for usr, name, ff, l, c, caller, comp in all_reads])
    struct_fields = {n: [(fn_, ft) for fn_, ft, _ in s["fields"]] for n, s in structs.items()}
    for n, cb in cbuffers.items():
        size, packed = struct_layout([(fn_, ft) for fn_, ft, _ in cb["fields"]], struct_fields, True)
        usr = "hlsl:cbuffer:" + n
        db.execute("INSERT OR IGNORE INTO ast_layouts VALUES(?,?,?,?,?,?,?,?)",
                   (usr, n, n, cb["file"], cb["line"], (size + 15) // 16 * 16, 16, "hlsl-cbuffer"))
        db.executemany("INSERT OR IGNORE INTO ast_fields VALUES(?,?,?,?,?,?,?,?,?)",
                       [(usr, i, fn_, ft, off, sz, cb["file"], cb["fields"][i][2], "hlsl-cbuffer")
                        for i, (fn_, ft, off, sz) in enumerate(packed)])
    for n, s in structs.items():
        if not s["fields"]:
            continue
        size, packed = struct_layout(struct_fields[n], struct_fields, False)
        usr = "hlsl:struct:" + n
        db.execute("INSERT OR IGNORE INTO ast_layouts VALUES(?,?,?,?,?,?,?,?)",
                   (usr, n, n, s["file"], s["line"], size, 4, "hlsl-struct"))
        db.executemany("INSERT OR IGNORE INTO ast_fields VALUES(?,?,?,?,?,?,?,?,?)",
                       [(usr, i, fn_, ft, off, sz, s["file"], s["fields"][i][2], "hlsl-struct")
                        for i, (fn_, ft, off, sz) in enumerate(packed)])
    db.execute("INSERT OR REPLACE INTO ast_meta VALUES('hlsl_stamps', ?)", (key,))
    for f, why in failed:
        log(f"hlsl: dxc failed on {f}: {' | '.join(why)}")
    log(f"hlsl: {len(roots)} files dumped in {t_dump:.2f}s, {len(fns)} functions, {len(all_calls)} call sites, "
        f"{len(all_reads)} global reads, {len(cbuffers)} cbuffers, {time.perf_counter() - t0:.2f}s total")


# ---------------------------------------------------------------------------------- driver

def main():
    """One run at a time: a build that finishes while an index is running leaves a rerun mark
    instead of starting a second writer, and the running one goes round again when it ends, so
    the last build's sources are always the ones indexed."""
    argv = sys.argv[1:]
    if "--lock" not in argv:
        run(argv)
        return
    import msvcrt
    lock_path = argv[argv.index("--lock") + 1]
    mark = lock_path + ".rerun"
    fh = open(lock_path, "a+")
    try:
        msvcrt.locking(fh.fileno(), msvcrt.LK_NBLCK, 1)
    except OSError:
        open(mark, "w").close()
        log("another index run holds the lock; it will run again when it ends")
        return
    while True:
        if os.path.exists(mark):
            os.remove(mark)
        log(f"run: {' '.join(argv)}")
        run(argv)
        if not os.path.exists(mark):
            break


def run(argv):
    full = "--full" in argv
    jobs = int(argv[argv.index("--jobs") + 1]) if "--jobs" in argv else max(1, (os.cpu_count() or 2) - 1)
    cc_path = argv[argv.index("--cc") + 1] if "--cc" in argv else os.path.join(ROOT, "build", "compile_commands.json")
    only = argv[argv.index("--only") + 1] if "--only" in argv else None
    dxc = os.environ.get("GA_DXC", DXC_DEFAULT)
    t_start = time.perf_counter()

    db = sqlite3.connect(DB, timeout=60)
    # WAL: the Scriptorium keeps answering from the last committed index while a run writes the
    # next one (with the default rollback journal its readers got "database is locked" for the
    # length of a run). The mode is stored in the file, so the server needs no change.
    db.execute("PRAGMA journal_mode=WAL")
    with open(SCHEMA, encoding="utf-8") as fh:
        db.executescript(fh.read())
    if full:
        for t in ("ast_tus", "ast_hlsl_entries", "ast_hlsl_reach") + CPP_TABLES:
            db.execute(f"DELETE FROM {t}")
        db.execute("DELETE FROM ast_meta WHERE key='hlsl_stamps'")
    db.commit()

    if os.path.exists(dxc):
        index_hlsl(db, dxc, jobs, full)
        db.commit()
    else:
        log(f"hlsl: skipped, no dxc at {dxc} (set GA_DXC)")

    if "--hlsl-only" in argv:
        return
    try:
        import clang.cindex  # noqa: F401
    except ImportError:
        log("c++: skipped, libclang is not installed (py -3 -m pip install libclang)")
        return
    if not os.path.exists(cc_path):
        log(f"c++: skipped, no {cc_path} (configure with CMAKE_EXPORT_COMPILE_COMMANDS)")
        return
    include = msvc_include(db)
    with open(cc_path, encoding="utf-8") as fh:
        cc = json.load(fh)
    old = {r[0]: (r[1], json.loads(r[2])) for r in db.execute("SELECT tu, args_hash, deps FROM ast_tus")}
    stat_cache = {}

    def st(p):
        s = stat_cache.get(p)
        if s is None:
            s = stat_cache[p] = stamp(p)
        return s

    jobs_list, hashes, dirty_files, cur_tus = [], {}, set(), set()
    for e in sorted(cc, key=lambda e: e["file"]):
        path = os.path.abspath(os.path.join(e["directory"], e["file"]))
        r = rel(path)
        if r is None or (only and only not in r):
            continue
        cur_tus.add(r)
        args = clang_args(e["command"], include)
        # the indexer's own source is part of the key: a changed walk re-reads every TU, so a new
        # table never holds only the TUs that happened to change since
        h = hashlib.sha1(("\0".join(args) + "\0" + SELF_HASH).encode()).hexdigest()
        prev = old.get(r)
        if prev and prev[0] == h and all(st(p) == s for p, s in prev[1]):
            continue
        jobs_list.append((path, args))
        hashes[path] = h
        if prev:
            dirty_files.update(x for x in (rel(p) for p, _ in prev[1]) if x)
        dirty_files.add(r)
    gone = [t for t in old if t not in cur_tus and not only]
    for t in gone:
        db.execute("DELETE FROM ast_tus WHERE tu=?", (t,))
        dirty_files.add(t)
    if not jobs_list and not gone:
        log(f"c++: no TU changed ({len(cur_tus)} checked) in {time.perf_counter() - t_start:.2f}s")
        db.commit()
        return
    log(f"c++: {len(jobs_list)} of {len(cur_tus)} TUs to parse on {jobs} processes")

    cleared = set()

    def clear(files):
        for f in sorted(files - cleared):
            for t in CPP_TABLES:
                db.execute(f"DELETE FROM {t} WHERE file=? AND lang='cpp'", (f,))
            cleared.add(f)

    clear(dirty_files)
    n_err, n_diag, per_tu = 0, 0, []
    ref_seen = set()
    with ProcessPoolExecutor(max_workers=jobs) as ex:
        futs = [ex.submit(parse_tu, j) for j in jobs_list]
        for fut in as_completed(futs):
            res = fut.result()
            clear(set(res["files"]))
            db.executemany("INSERT OR IGNORE INTO ast_decls VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)", res["decls"])
            fresh = []
            for x in res["refs"]:
                k = (x[0], x[2], x[3], x[4], x[5])
                if k not in ref_seen:
                    ref_seen.add(k)
                    fresh.append(x)
            db.executemany("INSERT INTO ast_refs VALUES(?,?,?,?,?,?,?,?,?)", fresh)
            db.executemany("INSERT OR IGNORE INTO ast_layouts VALUES(?,?,?,?,?,?,?,?)", res["layouts"])
            db.executemany("INSERT OR IGNORE INTO ast_fields VALUES(?,?,?,?,?,?,?,?,?)", res["fields"])
            db.executemany("INSERT OR IGNORE INTO ast_includes VALUES(?,?,?)", res["includes"])
            # a header's rows come from every TU that includes it: kept once (they are identical)
            for t, n in (("regions", 9), ("values", 8), ("litcalls", 8)):
                fresh = [x for x in res[t] if (t, x) not in ref_seen]
                ref_seen.update((t, x) for x in fresh)
                db.executemany(f"INSERT INTO ast_{t} VALUES({','.join('?' * n)})", fresh)
            db.execute("INSERT OR REPLACE INTO ast_tus VALUES(?,?,?,?,?,?)",
                       (res["tu"], hashes[res["path"]], json.dumps([[p, st(p)] for p in res["deps"]]),
                        round(res["total_s"], 3), res["errors"], res["diags"]))
            n_err += res["errors"]
            n_diag += res["diags"]
            per_tu.append((res["total_s"], res["parse_s"], res["tu"]))
            for d in res["errors_in_repo"][:3]:
                log(f"c++: error in {res['tu']}: {rel(d[1])}:{d[2]}: {d[3]}")
    db.commit()
    per_tu.sort(reverse=True)
    n = len(per_tu)
    log(f"c++: {n} TUs in {time.perf_counter() - t_start:.1f}s wall; per TU mean "
        f"{sum(t for t, _, _ in per_tu) / n:.2f}s (parse {sum(p for _, p, _ in per_tu) / n:.2f}s), "
        f"slowest {per_tu[0][0]:.2f}s {per_tu[0][2]}; {n_err} errors / {n_diag} diagnostics, system headers included")
    counts = {t: db.execute(f"SELECT COUNT(*) FROM {t}").fetchone()[0]
              for t in CPP_TABLES}
    log("rows: " + ", ".join(f"{k} {v}" for k, v in counts.items()))


if __name__ == "__main__":
    main()
