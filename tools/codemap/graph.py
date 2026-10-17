# ================================================================================================
#  tools/codemap/graph.py -- the code as one graph, and what a scene makes of it.
#
#  NODES are clangd's symbols (C++) and dxc's functions (HLSL). One id for both C++ indexers:
#  clangd's SymbolID is the first 8 bytes of SHA-1 over the USR, the same USR libclang reports
#  (measured: 12,828 of libclang's 13,356 declarations land on a clangd symbol; the rest are members
#  of function-local structs, which clangd does not index).
#  EDGES: call (clangd, a ref with the Call bit, from the function it sits in), ref (a function named
#  without being called: a callback, a table entry -- reachable as if called), shader (a C++
#  function that names an HLSL entry point), hlsl (HLSL calls), and, per scene, the overrides a
#  virtual call may reach.
#  GUARDS: every site carries the regions (astindex ast_regions) that contain it: the conditions it
#  runs under.
#
#  WHAT A SCENE MAKES OF IT is abstract interpretation, sound by construction:
#    - a value is a set of constants, or TOP (unknown). A field's set is the union of everything that
#      may be written to it (ast_values): its default, every assignment, its scene key's value; a
#      write we cannot model (x += e, an out-parameter, a braced init of its record) makes it TOP.
#    - a condition is decided three ways (true / false / unknown). An edge is cut only when one of
#      its regions is DEFINITELY not taken, so whatever runs for the scene is in what is reached.
#    - reachability, live classes (rapid type analysis: a virtual call reaches an override only in a
#      class some reached code constructs or names) and the writes that count (only those in reached
#      code, under conditions not decided false) are one fixpoint: each round can only cut more, and
#      each cut is justified by the round before (the first round counts every write).
#  What is NOT modelled (and so left as TOP, never as a wrong constant): values through pointers or
#  references to a field, parameters, return values (but for the scene's own lookups below and
#  std::string::empty / size), aliasing of locals.
# ================================================================================================
import hashlib
import json
import os
import re
import sqlite3
import subprocess
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
DB = os.path.join(ROOT, "tools", "Scriptorium", "scriptorium.db")

# ---- the few names the analysis is told (everything else is found)
ROOTS = ["main"]                                # where the process starts
FRAME = ["ga::app::FrameLoop::Frame"]           # called once per frame by FrameLoop::Run
ARGV = ["ga::app::ParseArgs"]                   # writes there happen only for a command-line flag
SCHEMA = "ga::scene::Schema::"                  # the scene file's vocabulary (scene/Props.h)
SCENE = "ga::app::Scene::"                      # the scene's own lookups, decided from the file


def strip_parens(t):
    """'(a && b)' -> 'a && b' when the outer pair encloses all of it."""
    while t.startswith("(") and t.endswith(")"):
        d = 0
        for k, ch in enumerate(t):
            d += ch == "("
            d -= ch == ")"
            if d == 0 and k < len(t) - 1:
                return t
        t = t[1:-1]
    return t


def atomic(t):
    """True when t has no operator outside parentheses or quotes: a name, a call, a literal."""
    d, q = 0, False
    for ch in t:
        if ch == '"':
            q = not q
        elif not q:
            d += (ch in "([{") - (ch in ")]}")
            if d == 0 and ch in " &|<>=+-*/?%^":
                return False
    return True


def negate(t):
    """!t without stacking negations: !(!x) is x; a single term is negated bare."""
    if t.startswith("!") and atomic(t[1:]):
        return strip_parens(t[1:])
    return "!" + t if atomic(t) else f"!({t})"


def _first(e, pred, depth=0):
    """The first sub-expression of e (depth first) that pred accepts."""
    if pred(e):
        return e
    if isinstance(e, list) and depth < 6:
        for x in e[1:]:
            r = _first(x, pred, depth + 1)
            if r is not None:
                return r
    return None


def sid(usr):
    """clangd's SymbolID for a USR."""
    return hashlib.sha1(usr.encode()).digest()[:8].hex().upper()


FN_KINDS = {"Function", "InstanceMethod", "StaticMethod", "ClassMethod", "Constructor", "Destructor",
            "ConversionFunction"}
CLASS_KINDS = {"Struct", "Class", "Union"}
NULL_ID = "0" * 16
TOP = None              # the unknown value
MAX_SET = 24


class Edge:
    __slots__ = ("src", "dst", "kind", "file", "line", "col", "guard", "loop")

    def __init__(self, src, dst, kind, file, line, col):
        self.src, self.dst, self.kind, self.file, self.line, self.col = src, dst, kind, file, line, col
        self.guard, self.loop = (), False


class Graph:
    def __init__(self, db_path=DB):
        db = sqlite3.connect(db_path)
        self.db = db
        self.sym = {}
        self.parent_of = {}
        self.same = {}                 # libclang id -> clangd id of the same function (see lid)
        for r in db.execute("SELECT id, kind, name, scope, file, line, def_file, def_line, def_end, signature, "
                            "type, ret, in_repo FROM cd_symbols"):
            self.sym[r[0]] = {"kind": r[1], "name": r[2], "scope": r[3], "file": r[6] or r[4],
                              "line": r[7] or r[5], "end": r[8] or r[7] or r[5], "sig": r[9],
                              "type": r[10], "ret": r[11], "repo": r[12], "lang": "cpp"}
        self.by_qual = defaultdict(list)
        for i, s in self.sym.items():
            self.by_qual[s["scope"] + s["name"]].append(i)
        self._hlsl(db)
        self.out = defaultdict(list)
        self.inc = defaultdict(list)
        self.class_refs = defaultdict(set)       # container -> classes it constructs or names
        # A declaration or definition is not a use (kind & 3); clangd sets the Call bit on a function
        # named as a value too (Register("car.baja", MakeCarBaja) is kind 28), so a callback is an edge.
        for i, f, l, c, k, cont in db.execute("SELECT id, file, line, col, kind, container FROM cd_refs "
                                              "WHERE container != ? AND (kind & 3) = 0", (NULL_ID,)):
            t = self.sym.get(i)
            if t is None or cont not in self.sym:
                continue
            tk = t["kind"]
            if tk in FN_KINDS:
                if tk == "Constructor":
                    cl = self.class_of(i)
                    if cl:
                        self.class_refs[cont].add(cl)
                self._edge(cont, i, "call" if k & 16 else "ref", f, l, c)
            elif tk in CLASS_KINDS:
                self.class_refs[cont].add(i)
        self._libclang_calls(db)
        self.overrides = defaultdict(list)
        for s, p, o in db.execute("SELECT subject, predicate, object FROM cd_rel WHERE predicate = 1"):
            self.overrides[s].append(o)
        self._libclang_overrides(db)
        self._libclang_class_refs(db)
        self._libclang_members(db)
        self._values(db)
        self._shader_edges(db)        # reads values: a Compile path held in a local
        self._guards(db)
        self._scene_keys(db)

    # ------------------------------------------------------------------------------ construction
    def _libclang_calls(self, db):
        """What clangd-indexer leaves out, from libclang: calls to a function seen only in its own
        .cpp (an anonymous namespace, a static -- clangd keeps the symbol but drops refs to it) and
        the functions of local structs (not indexed at all). Those become nodes from ast_decls, and
        every libclang call site clangd did not record becomes an edge. The split is kept for INDEX."""
        # A member of a function-local struct has one USR in clang 18 and another in clangd's 23: the
        # same function at the same place, merged by (file, line, name) so it is one node.
        at = {(s_["file"], s_["line"], s_["name"]): i for i, s_ in self.sym.items() if s_["kind"] in FN_KINDS}
        for usr, kind, name, qual, f, l, e, parent in db.execute(
                "SELECT usr, kind, name, qual, file, line, end_line, parent_usr FROM ast_decls WHERE lang='cpp' AND is_def=1 "
                "AND kind IN ('function_decl','cxx_method','constructor','destructor','conversion_function',"
                "'function_template')"):
            i = sid(usr)
            if i not in self.sym and (f, l, name) in at:
                self.same[i] = i = at[(f, l, name)]
            if kind in ("cxx_method", "constructor", "destructor") and parent:
                self.parent_of[i] = sid(parent)
            if i not in self.sym:
                scope = qual[:-len(name)] if qual.endswith(name) else ""
                self.sym[i] = {"kind": {"cxx_method": "InstanceMethod", "constructor": "Constructor",
                                        "destructor": "Destructor"}.get(kind, "Function"),
                               "name": name, "scope": scope, "file": f, "line": l, "end": e, "sig": "",
                               "type": "", "ret": "", "repo": 1, "lang": "cpp", "src": "libclang"}
                self.by_qual[qual].append(i)
        # A function named outside any function -- in a global table's initializer,
        # `const Law kPlantSquare = {..., GlobalSquare, ...}` -- is attributed to that variable,
        # which runs before main (a root).
        var_spans = defaultdict(list)
        for usr, name, qual, f, l, e in db.execute(
                "SELECT d.usr, d.name, d.qual, d.file, d.line, d.end_line FROM ast_decls d WHERE d.lang='cpp' "
                "AND d.kind='var_decl'"):
            var_spans[f].append((l, e, usr, name, qual))
        # libclang names a call to a function template by the INSTANTIATION (`...@F@Reconstruct<#...`),
        # not the template (`...@FT@>1#TReconstruct...`): the template is found by its scope prefix and
        # name, or a call into a file-local template is lost (SweSolver's Reconstruct read as dead).
        templates = {}
        for usr, name in db.execute("SELECT usr, name FROM ast_decls WHERE lang='cpp' AND kind='function_template'"):
            k = usr.find("@FT@")
            if k >= 0:
                templates[(usr[:k + 1], name)] = sid(usr)
        seen = {(e.dst, e.file, e.line) for lst in self.out.values() for e in lst}
        self.split = {"clangd": len(seen), "libclang_only": 0, "unknown_callee": 0}
        for usr, cont, f, l, c, kind in db.execute(
                "SELECT usr, container, file, line, col, kind FROM ast_refs WHERE lang='cpp' AND kind IN ('call','addr')"):
            dst = self.lid(usr)
            if dst not in self.sym:
                m = re.match(r"^(.*?@)F@([~\w]+)<", usr)
                if m and (m.group(1), m.group(2)) in templates:
                    dst = templates[(m.group(1), m.group(2))]
            if (dst, f, l) in seen:
                continue
            if dst not in self.sym or self.sym[dst]["kind"] not in FN_KINDS:
                if kind == "call":
                    self.split["unknown_callee"] += 1  # an implicit member (operator=), a lambda's ()
                continue
            if not cont:
                v = min((x for x in var_spans.get(f, ()) if x[0] <= l <= x[1]), key=lambda x: x[1] - x[0], default=None)
                if v is None:
                    continue
                cont = v[2]
                if sid(cont) not in self.sym:
                    self.sym[sid(cont)] = {"kind": "Variable", "name": v[3], "scope": v[4][:-len(v[3])], "file": f,
                                           "line": v[0], "end": v[1], "sig": "", "type": "", "ret": "", "repo": 1,
                                           "lang": "cpp", "src": "libclang"}
            src = self.lid(cont)
            if src not in self.sym:
                continue
            seen.add((dst, f, l))
            self.split["libclang_only"] += 1
            self._edge(src, dst, "call" if kind == "call" else "ref", f, l, c)

    def _libclang_class_refs(self, db):
        """Classes a function names, from libclang too: a file-local class built by value
        (`OverlayColor over;`) is named only there, since clangd drops refs to file-local symbols."""
        for usr, cont in db.execute("SELECT r.usr, r.container FROM ast_refs r JOIN ast_decls d ON d.usr = r.usr "
                                    "WHERE r.lang='cpp' AND r.kind='type' AND r.container != '' AND "
                                    "d.kind IN ('struct_decl','class_decl') AND d.is_def=1"):
            c, f = sid(usr), self.lid(cont)
            if f in self.sym:
                self.class_refs[f].add(c)
                if c not in self.sym:
                    self.sym[c] = {"kind": "Class", "name": usr.rsplit("@", 1)[-1], "scope": "", "file": "", "line": 0,
                                   "end": 0, "sig": "", "type": "", "ret": "", "repo": 1, "lang": "cpp", "src": "libclang"}

    def _libclang_members(self, db):
        """A class held BY VALUE in another (a field of its type) is built with it: when the outer
        class is live, so is the member's (Assembly's BankMaskClose and its destructor)."""
        classes = defaultdict(list)
        for usr, f, l, e in db.execute("SELECT usr, file, line, end_line FROM ast_decls WHERE lang='cpp' AND "
                                       "kind IN ('struct_decl','class_decl') AND is_def=1"):
            classes[f].append((l, e, usr))
        fields = {(f, l) for f, l in db.execute("SELECT file, line FROM ast_decls WHERE lang='cpp' AND kind='field_decl'")}
        self.members = defaultdict(set)
        for usr, f, l in db.execute("SELECT r.usr, r.file, r.line FROM ast_refs r JOIN ast_decls d ON d.usr = r.usr "
                                    "WHERE r.lang='cpp' AND r.kind='type' AND r.container = '' AND "
                                    "d.kind IN ('struct_decl','class_decl')"):
            if (f, l) not in fields:
                continue
            c = min((x for x in classes.get(f, ()) if x[0] <= l <= x[1]), key=lambda x: x[1] - x[0], default=None)
            if c and c[2] != usr:
                self.members[sid(c[2])].add(sid(usr))
        # A struct defined INSIDE a function can only be built there (`struct Leave {...} leave{this};`
        # names no type either): it is live when that function runs.
        for usr, parent in db.execute("SELECT c.usr, c.parent_usr FROM ast_decls c JOIN ast_decls f ON f.usr = c.parent_usr "
                                      "WHERE c.lang='cpp' AND c.kind IN ('struct_decl','class_decl') AND f.kind IN "
                                      "('function_decl','cxx_method','constructor','destructor','function_template')"):
            if sid(parent) in self.sym:
                self.class_refs[sid(parent)].add(sid(usr))
        # `struct BankMaskClose {...} bankMaskClose;` names no type (it is defined right there): a
        # by-value field whose declared type is a struct nested in the same owner links it too
        nested = defaultdict(list)
        for usr, name, parent in db.execute("SELECT usr, name, parent_usr FROM ast_decls WHERE lang='cpp' AND "
                                            "kind IN ('struct_decl','class_decl') AND is_def=1"):
            nested[(parent, name)].append(usr)
        for parent, ftype in db.execute("SELECT parent_usr, type FROM ast_decls WHERE lang='cpp' AND kind='field_decl'"):
            if "*" in ftype or "&" in ftype or "<" in ftype:
                continue
            base = re.sub(r"^(const |volatile |struct |class )+", "", ftype).split("[")[0].strip().rsplit("::", 1)[-1]
            for u in nested.get((parent, base), ()):
                self.members[sid(parent)].add(sid(u))

    def _libclang_overrides(self, db):
        """Overrides clangd does not relate: a class seen only in its own .cpp (an anonymous
        namespace, a test's local class). A virtual method there overrides the same-named method of
        a base, the base read from the class's own `: public Base` (ast_refs kind base)."""
        classes = defaultdict(list)
        for usr, f, l, e in db.execute("SELECT usr, file, line, end_line FROM ast_decls WHERE lang='cpp' AND "
                                       "kind IN ('struct_decl','class_decl') AND is_def=1"):
            classes[f].append((l, e, usr))
        bases = defaultdict(set)
        for usr, f, l in db.execute("SELECT usr, file, line FROM ast_refs WHERE lang='cpp' AND kind='base'"):
            c = min((x for x in classes.get(f, ()) if x[0] <= l <= x[1]), key=lambda x: x[1] - x[0], default=None)
            if c and c[2] != usr:
                bases[c[2]].add(usr)
        methods = defaultdict(lambda: defaultdict(list))
        virt = []
        for usr, name, parent, flags in db.execute("SELECT usr, name, parent_usr, flags FROM ast_decls WHERE lang='cpp' "
                                                   "AND kind='cxx_method'"):
            methods[parent][name].append(usr)
            if "virtual" in flags:
                virt.append((usr, name, parent))
        have = {(s, o) for s, lst in self.overrides.items() for o in lst}
        for usr, name, parent in virt:
            for b in bases.get(parent, ()):
                for bm in methods[b].get(name, ()):
                    pair = (self.lid(bm), self.lid(usr))
                    if pair not in have and pair[0] != pair[1]:
                        have.add(pair)
                        self.overrides[pair[0]].append(pair[1])

    def lid(self, usr):
        """The node id of a libclang USR: clangd's SymbolID, or the clangd node it was merged into."""
        i = sid(usr)
        return self.same.get(i, i)

    def _edge(self, src, dst, kind, f, l, c):
        e = Edge(src, dst, kind, f, l, c)
        self.out[src].append(e)
        self.inc[dst].append(e)

    def qual(self, i):
        s = self.sym.get(i)
        return (s["scope"] + s["name"]) if s else i

    def all_overrides(self, i):
        out, stack = [], list(self.overrides.get(i, ()))
        while stack:
            o = stack.pop()
            out.append(o)
            stack.extend(self.overrides.get(o, ()))
        return out

    def nrepo(self, ids):
        """How many of ids are the repo's own functions (reach also holds the std and SDK callees)."""
        return sum(1 for i in ids if self.sym.get(i, {}).get("repo") and self.sym[i]["file"])

    def subsystem(self, i):
        f = self.sym.get(i, {}).get("file") or ""
        return f.rsplit("/", 1)[0] if "/" in f else f

    def find(self, name):
        """Ids for a name as a person types it: 'Assemble', 'FrameLoop::Frame', 'ga::app::Assemble',
        'Globe.hlsl#PsSurface' or 'PsSurface'. Functions first."""
        hits = list(self.by_qual.get(name, ()))
        if not hits:
            tail = "::" + name
            hits = [i for q, ids in self.by_qual.items() if q.endswith(tail) or q == name for i in ids]
        if not hits:
            hits = [i for i, s in self.sym.items() if s["lang"] == "hlsl" and (
                s["name"] == name or i.split(" ", 1)[0].endswith(name))]
        fn = [i for i in hits if self.sym[i]["kind"] in FN_KINDS or self.sym[i]["lang"] == "hlsl"]
        return sorted(fn or hits, key=lambda i: (self.sym[i]["file"] or "~", self.sym[i]["line"] or 0))

    def class_of(self, i):
        if i in self.parent_of:
            return self.parent_of[i]
        s = self.sym.get(i)
        if not s or not s["scope"].endswith("::"):
            return None
        for c in self.by_qual.get(s["scope"][:-2], ()):
            if self.sym[c]["kind"] in CLASS_KINDS:
                return c
        return None

    def _hlsl(self, db):
        for usr, name, f, l, e in db.execute("SELECT usr, name, file, line, end_line FROM ast_decls "
                                             "WHERE lang='hlsl' AND kind='hlsl-fn'"):
            self.sym[usr] = {"kind": "HlslFunction", "name": name, "scope": "", "file": f, "line": l,
                             "end": e, "sig": "", "type": "", "ret": "", "repo": 1, "lang": "hlsl"}
            self.by_qual[name].append(usr)
        self.entries = {}
        for f, entry, prof, usr, defines, site in db.execute("SELECT file, entry, profile, usr, defines, cpp_site "
                                                             "FROM ast_hlsl_entries"):
            self.entries[usr] = {"file": f, "entry": entry, "profile": prof, "defines": defines, "sites": site}

    def enclosing(self, f, line):
        """The innermost C++ function whose definition holds file:line."""
        if not hasattr(self, "_fn_spans"):
            self._fn_spans = defaultdict(list)
            for i, s in self.sym.items():
                if s["kind"] in FN_KINDS and s["file"] and s["end"] > s["line"]:
                    self._fn_spans[s["file"]].append((s["line"], s["end"], i))
        best = None
        for a, b, i in self._fn_spans.get(f, ()):
            if a <= line <= b and (best is None or b - a < best[1] - best[0]):
                best = (a, b, i)
        return best[2] if best else None

    def _hlsl_file(self, e, depth=0):
        """The .hlsl a Compile() path argument names: a literal in it, or in the initializer of a
        local or field it reads (`path = dir + L"/Globe.hlsl"`)."""
        if depth > 4:
            return None
        if isinstance(e, str):
            return e.replace("\\", "/").rsplit("/", 1)[-1] if e.endswith(".hlsl") else None
        if not isinstance(e, list) or not e:
            return None
        if e[0] in ("l", "f", "g") and len(e) == 2:
            for kind, expr, *_ in self.values.get(sid(e[1]), ()):
                if kind in ("local", "default", "init", "assign"):
                    r = self._hlsl_file(json.loads(expr), depth + 1)
                    if r:
                        return r
            return None
        for x in e[1:]:
            r = self._hlsl_file(x, depth + 1)
            if r:
                return r
        return None

    def _shader_edges(self, db):
        """C++ -> HLSL entry. From each Compile(path, L"Entry", ...) call: the file its path names
        and the entry, so PsMain (defined by eleven shaders) links only the one compiled. An entry
        no Compile call names with its file falls back to the function holding its literal, when
        the entry's name is unique; the rest are listed unlinked in their shader's map."""
        by_name, by_pair = defaultdict(list), {}
        for usr, e in self.entries.items():
            by_name[e["entry"]].append(usr)
            by_pair[(e["file"].rsplit("/", 1)[-1], e["entry"])] = usr
            e["named_by"] = []
        rows = db.execute("SELECT callee, name, file, line, col, container, args FROM ast_litcalls "
                          "WHERE lang='cpp'").fetchall()
        # every .hlsl a function names in any call: where a Compile path is a parameter (a local
        # lambda, makePso(L"TideRibbon.hlsl", ...)), these are the files it can be
        named_in = defaultdict(set)
        for callee, name, f, l, c, cont, args in rows:
            if ".hlsl" in args:
                for x in re.findall(r'"([^"]*\.hlsl)"', args):
                    named_in[cont].add(x.replace("\\", "/").rsplit("/", 1)[-1])
        for callee, name, f, l, c, cont, args in rows:
            if name != "Compile":
                continue
            a = json.loads(args)
            entry = a[1] if len(a) > 1 and isinstance(a[1], str) else None
            if not entry:
                continue
            hf = self._hlsl_file(a[0]) if a else None
            usr = by_pair.get((hf, entry)) if hf else None
            if usr:
                targets = [usr]
            elif by_name[entry] and len(by_name[entry]) == 1:
                targets = by_name[entry]
            else:
                targets = [by_pair[(h, entry)] for h in sorted(named_in.get(cont, ())) if (h, entry) in by_pair]
            fn = self.lid(cont) if cont and self.lid(cont) in self.sym else self.enclosing(f, l)
            for t in targets:
                self.entries[t]["named_by"].append((fn, f, l))
                if fn:
                    self._edge(fn, t, "shader", f, l, c)
        for usr, e in self.entries.items():
            if e["named_by"] or len(by_name[e["entry"]]) != 1:
                continue
            for site in (e["sites"] or "").split("; "):
                if ":" in site:
                    f, l = site.rsplit(":", 1)
                    fn = self.enclosing(f, int(l))
                    e["named_by"].append((fn, f, int(l)))
                    if fn:
                        self._edge(fn, usr, "shader", f, int(l), 1)
        for callee, f, l, c, caller in db.execute("SELECT usr, file, line, col, container FROM ast_refs "
                                                  "WHERE lang='hlsl' AND kind='call'"):
            if caller in self.sym and callee in self.sym:
                self._edge(caller, callee, "hlsl", f, l, c)

    def _guards(self, db):
        """Every edge gets the regions that hold its site (a sweep per file over the nested spans)."""
        self.regions = []
        by_file = defaultdict(list)
        for f, l0, c0, l1, c1, cond, pol in db.execute("SELECT file, l0, c0, l1, c1, cond, pol FROM ast_regions "
                                                       "WHERE lang='cpp'"):
            by_file[f].append(len(self.regions))
            self.regions.append((f, (l0, c0), (l1, c1), cond, pol))
        sites = defaultdict(list)
        for lst in self.out.values():
            for e in lst:
                if e.kind in ("call", "ref", "shader"):
                    sites[e.file].append(e)
        self._site_cache = {}
        for f, es in sites.items():
            regs = sorted(by_file.get(f, ()), key=lambda k: (self.regions[k][1], (-self.regions[k][2][0], -self.regions[k][2][1])))
            es.sort(key=lambda e: (e.line, e.col))
            i, stack = 0, []
            for e in es:
                p = (e.line, e.col)
                while i < len(regs) and self.regions[regs[i]][1] <= p:
                    stack.append(regs[i])
                    i += 1
                g = tuple(k for k in stack if self.regions[k][2] >= p)
                e.loop = any(self.regions[k][4] == 0 for k in g)
                e.guard = tuple(k for k in g if self.regions[k][4] != 0)

    def guard_at(self, f, line):
        """The regions holding a line (for a value row, whose site is known to the line). A region
        that starts or ends inside the line is not counted: fewer conditions, never a wrong one."""
        key = (f, line)
        g = self._site_cache.get(key)
        if g is None:
            if not hasattr(self, "_regs_by_file"):
                self._regs_by_file = defaultdict(list)
                for k, r in enumerate(self.regions):
                    if r[4] != 0:
                        self._regs_by_file[r[0]].append(k)
            g = self._site_cache[key] = tuple(k for k in self._regs_by_file.get(f, ())
                                              if self.regions[k][1][0] < line < self.regions[k][2][0])
        return g

    def _values(self, db):
        self.values = defaultdict(list)
        self.aggregate = set()
        self.field_record = {sid(u): sid(p) for u, p in db.execute(
            "SELECT usr, parent_usr FROM ast_decls WHERE lang='cpp' AND kind='field_decl' AND parent_usr != ''")}
        for usr, f, l, kind, expr, cont, detail in db.execute("SELECT usr, file, line, kind, expr, container, detail "
                                                              "FROM ast_values WHERE lang='cpp'"):
            i = sid(usr)
            if kind == "aggregate":
                self.aggregate.add(i)
                continue
            self.values[i].append((kind, expr, f, l, self.lid(cont) if cont else "", detail))
        # A reference local is its referent: `auto& mode = m_mode; mode = 1;` writes m_mode. Its
        # writes move to what it names (through chains of references), its own value is then read
        # from its initializer, which is that name.
        alias = {}
        for i, rows in self.values.items():
            for kind, expr, f, l, cont, detail in rows:
                if kind == "local" and detail == "ref":
                    e = json.loads(expr)
                    if isinstance(e, list) and e[:1] and e[0] in ("f", "g", "l"):
                        alias[i] = sid(e[1])
        for i in list(alias):
            t, hops = alias[i], 0
            while t in alias and hops < 16:
                t, hops = alias[t], hops + 1
            moved = [r for r in self.values[i] if r[0] in ("assign", "byref")]
            self.values[i] = [r for r in self.values[i] if r[0] not in ("assign", "byref")]
            self.values[t].extend(moved)
        self.argv = set()
        for q in ARGV:
            self.argv.update(self.by_qual.get(q, ()))

    def _scene_keys(self, db):
        """The scene file's keys, each to the field it lands in: Schema::Bind / BindEnum calls, nested
        by Schema::Nest from the root schema down (scene/SceneSchema.cpp), read from ast_litcalls."""
        binds = defaultdict(list)      # schema fn -> [(key, field id, enum names)]
        nests = defaultdict(list)      # schema fn -> [(key, child schema fn, list?)]
        names = {}                     # schema fn -> the name its Schema(...) constructor gives
        for callee, name, f, l, cont, args in db.execute("SELECT callee, name, file, line, container, args FROM "
                                                         "ast_litcalls WHERE lang='cpp'"):
            q = self.qual(sid(callee))
            if not q.startswith(SCHEMA):
                continue
            a = json.loads(args)
            fn = self.lid(cont) if cont else ""
            if name == "Schema" and a and isinstance(a[0], str):
                names[fn] = a[0]
            elif name in ("Bind", "BindEnum") and len(a) >= 2 and isinstance(a[0], str):
                fld = a[1]
                if isinstance(fld, list) and fld[:1] == ["f"]:
                    en = None
                    if name == "BindEnum" and len(a) >= 3:
                        # the names arrive as a braced list, inside std::vector's constructor
                        lst = _first(a[2], lambda x: isinstance(x, list) and x[:1] == ["{}"])
                        if lst:
                            en = [x if isinstance(x, str) else None for x in lst[1:]]
                    binds[fn].append((a[0], sid(fld[1]), en))
            elif name in ("Nest", "List") and len(a) >= 2 and isinstance(a[0], str):
                sub = a[1]
                if isinstance(sub, list) and sub[:1] == ["&"]:
                    sub = sub[1]
                if isinstance(sub, list) and sub[:1] == ["call"]:
                    nests[fn].append((a[0], sid(sub[1]), name == "List"))
        self.keys = {}                 # key path -> {field, enum}
        self.field_keys = defaultdict(list)
        roots = [fn for fn, n in names.items() if n == "scene-file"]
        seen = set()

        def walk(fn, prefix):
            if (fn, prefix) in seen:
                return
            seen.add((fn, prefix))
            for k, fld, en in binds.get(fn, ()):
                path = prefix + k
                self.keys[path] = {"field": fld, "enum": en}
                self.field_keys[fld].append(path)
            for k, child, is_list in nests.get(fn, ()):
                walk(child, prefix + k + ("[]." if is_list else "."))
        for r in roots:
            walk(r, "")
        self.schema_fns = set(binds) | set(nests) | set(names)

    # ------------------------------------------------------------------------------ rendering
    def short(self, i, usr=None):
        s = self.sym.get(i)
        if not s:
            # outside clangd's symbols (a std:: member whose USR clang 18 spells differently)
            m = re.search(r"@F@([~\w]+)", usr or "")
            return m.group(1) if m else i
        scope = s["scope"]
        cls = scope[:-2].rsplit("::", 1)[-1] if scope.endswith("::") else ""
        q = s["scope"] + s["name"]
        # a class member is spelled Class::name; a free function by its name alone
        if cls and self.class_of(i):
            return f"{cls}::{s['name']}"
        return s["name"] if s["lang"] == "cpp" else q

    def show(self, e, depth=0):
        """A decoded expression (astindex's JSON) as compact source-like text."""
        if depth > 6:
            return "..."
        if e is None:
            return "null"
        if isinstance(e, bool):
            return "true" if e else "false"
        if isinstance(e, (int, float)):
            return repr(e)
        if isinstance(e, str):
            return json.dumps(e)
        if e == "loop":
            return "loop"
        h, a = e[0], e[1:]
        S = lambda x: self.show(x, depth + 1)       # noqa: E731
        if h == "?":
            return "?"
        if h in ("f", "g"):
            i = sid(a[0])
            keys = self.field_keys.get(i)
            return keys[0] if keys else self.short(i)
        if h in ("l", "p"):
            return a[0].rsplit("@", 1)[-1]
        if h == "fn":
            return self.short(sid(a[0]))
        if h == "!":
            return "!" + S(a[0])
        if h == "neg":
            return "-" + S(a[0])
        if h in ("&", "*"):
            return h + S(a[0])
        if h == "bool":
            return S(a[0])
        if h == "?:":
            return f"({S(a[0])} ? {S(a[1])} : {S(a[2])})"
        if h == "[]":
            return f"{S(a[0])}[{S(a[1])}]"
        if h == "{}":
            return "{" + ", ".join(S(x) for x in a) + "}"
        if h == "in":
            return f"{S(a[0])} in {{{', '.join(S(x) for x in a[1:])}}}"
        if h == "call":
            name = self.short(sid(a[0]), a[0])
            obj = (S(a[1]) + ".") if a[1] is not None else ""
            return f"{obj}{name.rsplit('::', 1)[-1] if obj else name}({', '.join(S(x) for x in a[2:])})"
        if len(a) == 2:
            return f"({S(a[0])} {h} {S(a[1])})"
        return "?"

    def show_region(self, k):
        """Region k's condition as it must hold for code inside it to run."""
        r = self.regions[k]
        t = strip_parens(self.show(json.loads(r[3])))
        if t == "?":
            return "(not modelled)"
        return t if r[4] == 1 else negate(t)

    def show_guard(self, guard, ev=None):
        return " && ".join(self.show_region(k) for k in guard
                           if ev is None or ev.region_truth(k) is not True)   # decided true: says nothing


# ================================================================================================
#  A scene: the folded document, the values it pins, and what it reaches.
# ================================================================================================
def fold_scene(path, exe=None):
    """The scene as the engine resolves it (base, includes, defaults): gagame.exe <scene> --print-scene."""
    exe = exe or os.path.join(ROOT, "build", "bin", "gagame.exe")
    p = subprocess.run([exe, path, "--print-scene"], cwd=ROOT, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=60)
    if p.returncode != 0:
        raise RuntimeError(f"--print-scene {path}: exit {p.returncode}: {p.stderr.strip()[-300:]}")
    return json.loads(p.stdout)


def key_values(doc, path):
    """Every value a key path takes in the folded document ('views[].fovY': one per element)."""
    cur = [doc]
    for part in path.split("."):
        lst = part.endswith("[]")
        k = part[:-2] if lst else part
        nxt = []
        for c in cur:
            if isinstance(c, dict) and k in c:
                v = c[k]
                if lst:
                    nxt.extend(v if isinstance(v, list) else [])
                else:
                    nxt.append(v)
        cur = nxt
    return cur


class Eval:
    """Values and conditions under one scene (doc) -- or under none (doc None: every scene key TOP),
    which is what holds for EVERY scene."""

    def __init__(self, g, doc=None, argv=False):
        self.g, self.doc, self.argv = g, doc, argv
        self.reach = None              # functions reached (None: every write counts)
        self.memo, self.rmemo = {}, {}
        self.scene = {}
        if doc is not None:
            for path, k in g.keys.items():
                vals = key_values(doc, path)
                if "[]" in path and not vals:
                    continue           # an empty list: no element, no value
                out = set()
                for v in vals:
                    if k["enum"] is not None and isinstance(v, str):
                        v = k["enum"].index(v) if v in k["enum"] else None
                        if v is None:
                            out = TOP
                            break
                    if isinstance(v, (dict, list)):
                        out = TOP
                        break
                    out.add(v)
                if out is TOP or not vals:
                    self.scene[k["field"]] = TOP
                else:
                    prev = self.scene.get(k["field"], set())
                    self.scene[k["field"]] = TOP if prev is TOP else prev | out

    def reset(self, reach):
        self.reach = reach
        self.memo, self.rmemo = {}, {}

    # -- values
    def var(self, i):
        if i in self.memo:
            return self.memo[i]
        self.memo[i] = TOP                       # a cycle reads as unknown
        g = self.g
        out = set()
        bound = i in g.field_keys
        if bound:
            if self.doc is None:
                return TOP
            sv = self.scene.get(i, TOP)
            if sv is TOP:
                return TOP
            out |= sv
        rows = g.values.get(i, ())
        if g.class_of(i) in g.aggregate or g.field_record.get(i) in g.aggregate:
            return TOP                           # its record is built from a braced list or in the STL
        if not rows and not bound:
            return TOP
        has_start = bound
        for kind, expr, f, l, cont, detail in rows:
            if kind in ("default", "init", "local"):
                if bound:
                    continue                     # the scene's value is the field's start
                has_start = True
            if not self.live_row(cont, f, l):
                continue
            if cont in g.schema_fns:
                continue                         # the scene loader's own binding
            if kind == "byref":
                if not self.argv and cont in g.argv:
                    continue
                return self._set(i, TOP)
            if kind == "assign" and not self.argv and cont in g.argv:
                continue
            v = self.expr(json.loads(expr))
            if v is TOP:
                return self._set(i, TOP)
            out |= v
            if len(out) > MAX_SET:
                return self._set(i, TOP)
        if not has_start:
            return self._set(i, TOP)            # no initializer: its first value is not known
        return self._set(i, frozenset(out))

    def _set(self, i, v):
        self.memo[i] = v
        return v

    def live_row(self, cont, f, l):
        if self.reach is None or not cont or cont not in self.g.sym:
            return True                # a container clangd does not know cannot be judged unreached
        if cont not in self.reach:
            return False
        return not self.guard_false(self.g.guard_at(f, l))

    def expr(self, e):
        """e is decoded JSON: a str here is a string literal, never an encoded expression."""
        if e is None or isinstance(e, (bool, int, float, str)):
            return frozenset([e])
        h, a = e[0], e[1:]
        if h in ("f", "g", "l"):
            return self.var(sid(a[0]))
        if h in ("?", "p", "fn", "&", "*", "{}", "[]"):
            return TOP
        if h == "bool":
            v = self.expr(a[0])
            return TOP if v is TOP else frozenset(bool(x) for x in v)
        if h == "!":
            v = self.expr(a[0])
            return TOP if v is TOP else frozenset(not x for x in v)
        if h == "neg":
            v = self.expr(a[0])
            try:
                return TOP if v is TOP else frozenset(-x for x in v)
            except TypeError:
                return TOP
        if h in ("&&", "||"):
            x = self.truth_of(self.expr(a[0]))
            if h == "&&" and x is False:
                return frozenset([False])
            if h == "||" and x is True:
                return frozenset([True])
            y = self.truth_of(self.expr(a[1]))
            if h == "&&":
                r = False if y is False else (True if x is True and y is True else None)
            else:
                r = True if y is True else (False if x is False and y is False else None)
            return TOP if r is None else frozenset([r])
        if h == "?:":
            c = self.truth_of(self.expr(a[0]))
            if c is True:
                return self.expr(a[1])
            if c is False:
                return self.expr(a[2])
            x, y = self.expr(a[1]), self.expr(a[2])
            return TOP if x is TOP or y is TOP else x | y
        if h == "in":
            v = self.expr(a[0])
            opts = [self.expr(x) for x in a[1:]]
            if v is TOP or any(o is TOP for o in opts):
                return TOP
            allv = set().union(*opts) if opts else set()
            return frozenset(x in allv for x in v)
        if h == "call":
            return self.call(a)
        if len(a) == 2:
            x, y = self.expr(a[0]), self.expr(a[1])
            if x is TOP or y is TOP or len(x) * len(y) > 64:
                return TOP
            out = set()
            for p in x:
                for q in y:
                    r = binop(h, p, q)
                    if r is TOP:
                        return TOP
                    out.add(r)
            return frozenset(out)
        return TOP

    def call(self, a):
        g = self.g
        q = g.qual(sid(a[0]))
        args = a[2:]
        if q.startswith(SCENE) and self.doc is not None and args and isinstance(args[0], str):
            name = q[len(SCENE):]
            r = scene_lookup(self.doc, name, args[0])
            if r is not None:
                return frozenset([r])
        if q.startswith("std::basic_string::") and a[1] is not None:
            v = self.expr(a[1])
            if v is not TOP and all(isinstance(x, str) for x in v):
                m = q.rsplit("::", 1)[1]
                if m == "empty":
                    return frozenset(len(x) == 0 for x in v)
                if m in ("size", "length"):
                    return frozenset(len(x) for x in v)
        return TOP

    @staticmethod
    def truth_of(v):
        if v is TOP or not v:
            return None
        t = {bool(x) for x in v}
        return t.pop() if len(t) == 1 else None

    def region_truth(self, k):
        """Whether region k is taken: True / False / None (unknown)."""
        r = self.rmemo.get(k, 0)
        if r == 0:
            reg = self.g.regions[k]
            t = self.truth_of(self.expr(json.loads(reg[3])))
            r = self.rmemo[k] = None if t is None else (t if reg[4] == 1 else not t)
        return r

    def guard_false(self, guard):
        return any(self.region_truth(k) is False for k in guard)


def binop(op, p, q):
    try:
        if op == "==":
            return p == q
        if op == "!=":
            return p != q
        if p is None or q is None or isinstance(p, str) != isinstance(q, str):
            return TOP
        if op == "<":
            return p < q
        if op == "<=":
            return p <= q
        if op == ">":
            return p > q
        if op == ">=":
            return p >= q
        if op == "+":
            return p + q
        if op == "-":
            return p - q
        if op == "*":
            return p * q
        if op == "/":
            return TOP if q == 0 else (p // q if isinstance(p, int) and isinstance(q, int) else p / q)
        if op == "&" and isinstance(p, int):
            return p & q
        if op == "|" and isinstance(p, int):
            return p | q
    except TypeError:
        return TOP
    return TOP


def scene_lookup(doc, name, arg):
    """The scene's own lookups (app/Scene.h), decided from the folded file: True/False, or None
    when the lookup is not one of these."""
    def named(lst, key="name", on=False):
        for x in doc.get(lst, []) or []:
            if isinstance(x, dict) and x.get(key) == arg and (not on or x.get("enabled", True)):
                return True
        return False
    if name == "Tool":
        return named("tools")
    if name == "LayerOn":
        return named("layers", on=True)
    if name == "Layer":
        return named("layers")
    if name == "Portal":
        return named("portals")
    if name == "Entity":
        return named("entities")
    if name == "Effect":
        return named("effects")
    if name == "EffectOfType":
        return named("effects", key="type", on=True)
    if name == "View":
        return named("views")
    return None


# ================================================================================================
#  Reachability
# ================================================================================================
def roots(g):
    out = []
    for q in ROOTS:
        out += [i for i in g.by_qual.get(q, ()) if g.sym[i]["kind"] in FN_KINDS]
    # what runs before main: the initializers of namespace-scope variables (their refs' container)
    out += sorted({e.src for lst in g.out.values() for e in lst
                   if g.sym.get(e.src, {}).get("kind") == "Variable"})
    return out


def closure(g, ev, start, live):
    """What `start` reaches under a scene's evaluation and live classes (no new classes become live:
    the per-frame set inside a whole-program reach)."""
    seen, todo = set(start), list(start)
    while todo:
        f = todo.pop()
        for e in g.out.get(f, ()):
            if e.guard and ev.guard_false(e.guard):
                continue
            for t in [e.dst] + [o for o in g.all_overrides(e.dst) if g.class_of(o) in live]:
                if t not in seen:
                    seen.add(t)
                    todo.append(t)
    return seen


def reach(g, ev, start=None, rounds=8):
    """The fixpoint described at the top: (reached functions, live classes, rounds taken)."""
    start = start if start is not None else roots(g)
    # What runs when a class is built: its fields' initializers, its constructors and destructor --
    # all of them, since make_unique<T>(...) calls the one it picks from inside the STL, where no
    # reference of the repo's records it.
    fields_of = defaultdict(list)
    for i, s in g.sym.items():
        if (s["kind"] == "Field" and i in g.out) or s["kind"] in ("Constructor", "Destructor"):
            c = g.class_of(i)
            if c:
                fields_of[c].append(i)
    prev = None
    for n in range(rounds):
        seen, live, todo = set(start), set(), list(start)
        pending_virtual = defaultdict(list)    # class -> overrides waiting for it to be live

        def add(x):
            if x not in seen:
                seen.add(x)
                todo.append(x)

        def make_live(c):
            if c in live:
                return
            live.add(c)
            for m in g.members.get(c, ()):
                make_live(m)
            for f in fields_of.get(c, ()):
                add(f)
            for o in pending_virtual.pop(c, ()):
                add(o)
        while todo:
            f = todo.pop()
            for c in g.class_refs.get(f, ()):
                make_live(c)
            for e in g.out.get(f, ()):
                if e.guard and ev.guard_false(e.guard):
                    continue
                add(e.dst)
                stack = list(g.overrides.get(e.dst, ()))
                while stack:
                    o = stack.pop()
                    stack.extend(g.overrides.get(o, ()))
                    c = g.class_of(o)
                    if c is None or c in live:
                        add(o)
                    else:
                        pending_virtual[c].append(o)
        if seen == prev:
            return seen, live, n + 1
        prev = seen
        ev.reset(seen)
    return prev, live, rounds


def successors(g, n, nodes, edge_ok, live=None):
    """Where n's calls go inside `nodes`: each callee, and the overrides a virtual call may reach
    (in live classes, when `live` is given)."""
    out = []
    for e in g.out.get(n, ()):
        if not edge_ok(e):
            continue
        for t in [e.dst] + [o for o in g.all_overrides(e.dst) if live is None or g.class_of(o) in live]:
            if t in nodes:
                out.append(t)
    return out


def dominators(g, start, nodes, edge_ok=lambda e: True, live=None):
    """Immediate dominators over the call graph restricted to `nodes`, from `start`, virtual calls
    included (Cooper, Harvey, Kennedy: 'A Simple, Fast Dominance Algorithm', 2001)."""
    succ = {}

    def nxt(n):
        if n not in succ:
            succ[n] = successors(g, n, nodes, edge_ok, live)
        return succ[n]
    order, seen, stack = [], {start}, [(start, iter(nxt(start)))]
    while stack:
        n, it = stack[-1]
        m = next((x for x in it if x not in seen), None)
        if m is None:
            order.append(n)
            stack.pop()
        else:
            seen.add(m)
            stack.append((m, iter(nxt(m))))
    order.reverse()
    idx = {n: k for k, n in enumerate(order)}
    preds = defaultdict(list)
    for n in order:
        for m in nxt(n):
            if m in idx:
                preds[m].append(n)
    idom = {start: start}
    changed = True
    while changed:
        changed = False
        for n in order[1:]:
            new = None
            for p in preds[n]:
                if p in idom:
                    if new is None:
                        new = p
                    else:
                        a, b = p, new
                        while a != b:
                            while idx[a] > idx[b]:
                                a = idom[a]
                            while idx[b] > idx[a]:
                                b = idom[b]
                        new = a
            if new is not None and idom.get(n) != new:
                idom[n] = new
                changed = True
    return idom


def sccs(g, nodes):
    """Strongly connected components of the call graph on `nodes` (Tarjan, iterative): recursion
    collapses to one node."""
    index, low, on, st, out, k = {}, {}, set(), [], [], 0
    for root in sorted(nodes):
        if root in index:
            continue
        work = [(root, iter(g.out.get(root, ())))]
        index[root] = low[root] = k
        k += 1
        st.append(root)
        on.add(root)
        while work:
            n, it = work[-1]
            pushed = False
            for e in it:
                m = e.dst
                if m not in nodes:
                    continue
                if m not in index:
                    index[m] = low[m] = k
                    k += 1
                    st.append(m)
                    on.add(m)
                    work.append((m, iter(g.out.get(m, ()))))
                    pushed = True
                    break
                if m in on:
                    low[n] = min(low[n], index[m])
            if pushed:
                continue
            work.pop()
            if work:
                low[work[-1][0]] = min(low[work[-1][0]], low[n])
            if low[n] == index[n]:
                comp = []
                while True:
                    m = st.pop()
                    on.discard(m)
                    comp.append(m)
                    if m == n:
                        break
                out.append(sorted(comp))
    return out
