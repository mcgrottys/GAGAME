# ================================================================================================
#  tools/codemap/build.py -- writes build/codemap/, the map of the code, laid out like the code:
#
#    INDEX.md               one page: what is where, how to ask, the counts, every scene's reach
#    subsystems.md          one row per source directory: size, reach, who it calls and is called by
#    src/<path>.map         one per C++ file: each function, its callers, its calls grouped by the
#    shaders/<file>.map     condition they run under, the scenes that reach it, per frame or not
#    scenes/<path>.map      one per scene file: what it reaches, the switches it decides and what
#                           each cuts, its shader entries, what it leaves out, file by file
#    keys.tsv               every scene key: the field it lands in, the conditions that read it
#    dominators.txt         who owns what: the dominator tree from main and from the frame
#    graph/nodes.tsv, graph/edges.tsv   everything, one fact a line, for grep
#    architecture.html      the picture (viz.py)
#
#  Every file is sorted and carries no clock time, so two builds of the same source are the same
#  bytes and a diff of build/codemap between two commits is a diff of the code's structure.
#  Usage: py -3 tools/codemap/build.py [--out build/codemap] [--scenes scenes]
# ================================================================================================
import json
import os
import subprocess
import sys
import time
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import graph as G  # noqa: E402

WRAP = 112


def log(msg):
    print(f"[codemap] {time.strftime('%H:%M:%S')} {msg}", flush=True)


def wrap(prefix, items, indent="      "):
    """`prefix` then the items comma-separated, wrapped at WRAP columns."""
    lines, cur = [], prefix
    for k, it in enumerate(items):
        piece = it + ("," if k < len(items) - 1 else "")
        if len(cur) + len(piece) + 1 > WRAP and cur.strip():
            lines.append(cur.rstrip())
            cur = indent
        cur += piece + " "
    lines.append(cur.rstrip())
    return lines


class Scenes:
    """Every scene file folded and evaluated once: reach, per-frame set, live classes."""

    def __init__(self, g, scene_dir):
        self.g = g
        self.any_ev = G.Eval(g, None)
        self.any_reach, self.any_live, _ = G.reach(g, self.any_ev)
        self.frame_roots = [i for q in G.FRAME for i in g.by_qual.get(q, ())]
        self.any_frame = G.closure(g, self.any_ev, self.frame_roots, self.any_live)
        self.items = []
        files = []
        for dp, dn, fs in os.walk(os.path.join(G.ROOT, scene_dir)):
            dn.sort()
            files += [os.path.relpath(os.path.join(dp, f), G.ROOT).replace("\\", "/") for f in sorted(fs) if f.endswith(".json")]
        for path in files:
            try:
                doc = G.fold_scene(path)
            except Exception as ex:       # a fragment (an include, a rail) is not a scene
                self.items.append({"path": path, "error": str(ex).splitlines()[0][:160]})
                continue
            ev = G.Eval(g, doc)
            r, live, rounds = G.reach(g, ev)
            fr = G.closure(g, ev, self.frame_roots, live)
            self.items.append({"path": path, "doc": doc, "ev": ev, "reach": r, "live": live, "frame": fr,
                               "rounds": rounds})
        self.ok = [s for s in self.items if "reach" in s]
        self.count = Counter()
        self.frame_count = Counter()
        for s in self.ok:
            self.count.update(s["reach"])
            self.frame_count.update(s["frame"])


def fn_ids(g, f):
    return sorted((i for i, s in g.sym.items() if s["file"] == f and (s["kind"] in G.FN_KINDS or s["lang"] == "hlsl")
                   and s["line"]), key=lambda i: (g.sym[i]["line"], i))


def card(g, i, sc=None, ev=None, max_callers=12):
    """One function's block: header, callers, calls by condition, std calls, shaders, scenes."""
    s = g.sym[i]
    sig = s["sig"] or ""
    ret = s["ret"] or ""
    head = f"{g.short(i)}  L{s['line']}-{s['end']}"
    if ret or sig:
        head += f"  {ret}{(' ' if ret else '')}{sig}".rstrip()
    if s["kind"] in ("Constructor", "Destructor"):
        head += f"  [{s['kind'].lower()}]"
    out = [head]
    if i in g.entries:
        e = g.entries[i]
        out.append(f"  ENTRY {e['profile'] or '?'} {e['entry']}" + (f"  defines {e['defines']}" if e["defines"] else ""))
    callers = sorted({(g.short(e.src), e.file, e.line) for e in g.inc.get(i, ()) if e.src in g.sym},
                     key=lambda t: (t[0], t[2], t[1]))
    if callers:
        shown = [f"{n}@{f.rsplit('/', 1)[-1]}:{l}" for n, f, l in callers[:max_callers]]
        if len(callers) > max_callers:
            shown.append(f"+{len(callers) - max_callers} more")
        out += wrap("  callers: ", shown)
    groups, std, shaders = defaultdict(list), Counter(), []
    for e in g.out.get(i, ()):
        if e.kind == "shader":
            en = g.entries.get(e.dst, {})
            shaders.append(f"{en.get('file', '?').rsplit('/', 1)[-1]}:{en.get('entry', '?')}")
            continue
        t = g.sym.get(e.dst)
        if t is None:
            continue
        if not t["repo"]:
            std[g.short(e.dst)] += 1
            continue
        cond = ""
        if e.guard:             # the innermost condition; '...' = more conditions hold it
            cond = g.show_region(e.guard[-1])
            if len(e.guard) > 1:
                cond = "... " + cond
            if len(cond) > 90:
                cond = cond[:87] + "..."
        name = g.short(e.dst) + ("*" if e.loop else "")
        if name not in groups[cond]:
            groups[cond].append(name)
    if groups.get(""):
        out += wrap("  calls: ", groups.pop(""))
    for cond in sorted(groups):
        out += wrap(f"    if {cond}: ", groups[cond], indent="        ")
    if std:
        out += wrap("  std/sdk: ", [f"{n}x{c}" if c > 1 else n for n, c in sorted(std.items())])
    if shaders:
        out += wrap("  compiles: ", sorted(set(shaders)))
    if sc is not None:
        n, nf = sc.count.get(i, 0), sc.frame_count.get(i, 0)
        tag = "UNREACHED by any scene" if i not in sc.any_reach else f"scenes {n}/{len(sc.ok)}"
        if nf:
            tag += f" · per frame in {nf}"
        out.append("  " + tag)
    return out


def write(path, lines):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(lines).rstrip() + "\n")


def file_maps(g, sc, out):
    files = sorted({s["file"] for s in g.sym.values() if s["repo"] and s["file"]
                    and (s["file"].startswith("src/") or s["file"].startswith("shaders/"))})
    inc = defaultdict(set)
    incby = defaultdict(set)
    for f, t in g.db.execute("SELECT file, included FROM ast_includes"):
        if not t.startswith("<") and ":" not in t:
            inc[f].add(t)
            incby[t].add(f)
    for f in files:
        ids = fn_ids(g, f)
        try:
            with open(os.path.join(G.ROOT, f), encoding="utf-8", errors="replace") as fh:
                nlines = sum(1 for _ in fh)
        except OSError:
            nlines = 0
        reached = sum(1 for i in ids if i in sc.any_reach)
        head = [f"# {f}", f"{nlines} lines · {len(ids)} functions ({reached} reachable) · subsystem {f.rsplit('/', 1)[0]}"]
        if inc.get(f):
            head += wrap("includes: ", sorted(x.split("/", 1)[-1] for x in inc[f]))
        if incby.get(f):
            head += wrap(f"included by ({len(incby[f])}): ", sorted(x.split("/", 1)[-1] for x in incby[f])[:24]
                         + ([f"+{len(incby[f]) - 24} more"] if len(incby[f]) > 24 else []))
        if f.startswith("shaders/"):
            ent = [e for e in g.entries.values() if e["file"] == f]
            for e in sorted(ent, key=lambda e: e["entry"]):
                by = sorted({g.short(n[0]) if n[0] else f"(table at {n[1]}:{n[2]})" for n in e["named_by"]})
                head.append(f"entry {e['entry']} ({e['profile'] or '?'}) compiled by: {', '.join(by) or 'NOTHING FOUND'}")
        body = []
        for i in ids:
            body.append("")
            body += card(g, i, sc)
        write(os.path.join(out, f + ".map"), head + body)
    return files


def scene_maps(g, sc, out):
    rows = []
    for s in sc.items:
        p = s["path"]
        if "error" in s:
            rows.append(f"| {p} | not a scene: {s['error']} | | |")
            continue
        ev, r, fr = s["ev"], s["reach"], s["frame"]
        doc = s["doc"]
        sh = sorted({(g.entries[i]["file"].rsplit("/", 1)[-1], g.entries[i]["entry"]) for i in r if i in g.entries})
        lost = sc.any_reach - r
        lines = [f"# {p}",
                 f"scene {doc.get('scene', {}).get('name', '?')} · mode {doc.get('scene', {}).get('mode', '?')} · "
                 f"planet {doc.get('scene', {}).get('planet', '?')} · base {doc.get('base') or '-'}",
                 f"reached {g.nrepo(r)} of {g.nrepo(sc.any_reach)} reachable functions ({g.nrepo(lost)} cut) · per frame {g.nrepo(fr)} · "
                 f"live classes {len(s['live'])} · shader entries {len(sh)}/{len(g.entries)} · fixpoint rounds {s['rounds']}",
                 "", "## The switches this scene decides (outermost decided-false condition -> calls cut, functions lost)"]
        # group every cut edge out of reached code by its outermost region decided false
        sw = defaultdict(lambda: [0, set()])
        for f in r:
            for e in g.out.get(f, ()):
                if not e.guard:
                    continue
                k = next((k for k in e.guard if ev.region_truth(k) is False), None)
                if k is None:
                    continue
                key = g.show_region(k) + "  ==> false"
                sw[key][0] += 1
                if e.dst in lost:
                    sw[key][1].add(e.dst)
        for key, (n, fns) in sorted(sw.items(), key=lambda kv: (-len(kv[1][1]), -kv[1][0], kv[0])):
            if not fns and n < 3:
                continue
            names = sorted(g.short(x) for x in fns if g.sym.get(x, {}).get("repo"))
            lines += wrap(f"- {key[:150]} : {n} calls, {len(fns)} lost: ", names[:14] + ([f"+{len(names) - 14}"] if len(names) > 14 else []), "    ")
        lines += ["", "## Shader entries reached"]
        byf = defaultdict(list)
        for f_, en in sh:
            byf[f_].append(en)
        lines += [f"- {f_}: {', '.join(v)}" for f_, v in sorted(byf.items())]
        lines += ["", "## Not reached here (reachable in some scene), by file"]
        lostf = Counter(g.sym[i]["file"] for i in lost if g.sym.get(i, {}).get("repo") and g.sym[i]["file"])
        lines += [f"- {f_}: {n}" for f_, n in sorted(lostf.items(), key=lambda kv: (-kv[1], kv[0]))]
        lines += ["", "## Per frame, by subsystem (functions)"]
        perf = Counter(g.subsystem(i) for i in fr if g.sym.get(i, {}).get("repo"))
        lines += [f"- {k}: {v}" for k, v in sorted(perf.items(), key=lambda kv: (-kv[1], kv[0]))]
        write(os.path.join(out, "scenes", p[len("scenes/"):].rsplit(".", 1)[0] + ".map"), lines)
        rows.append(f"| {p} | {g.nrepo(r)} | {g.nrepo(fr)} | {len(sh)} |")
    return rows


def subsystems(g, sc, out):
    stat = defaultdict(lambda: {"fns": 0, "reach": 0, "files": set(), "out": Counter(), "in": Counter()})
    for i, s in g.sym.items():
        if s["repo"] and s["file"] and (s["kind"] in G.FN_KINDS or s["lang"] == "hlsl"):
            d = g.subsystem(i)
            stat[d]["fns"] += 1
            stat[d]["files"].add(s["file"])
            stat[d]["reach"] += i in sc.any_reach
    for lst in g.out.values():
        for e in lst:
            a, b = g.subsystem(e.src), g.subsystem(e.dst)
            if a and b and a != b and g.sym.get(e.dst, {}).get("repo"):
                stat[a]["out"][b] += 1
                stat[b]["in"][a] += 1
    lines = ["# Subsystems (one row per source directory)", "",
             "| dir | files | functions | reachable | calls out to (calls) | called from (calls) |", "|---|---|---|---|---|---|"]
    for d in sorted(stat):
        st = stat[d]
        o = ", ".join(f"{k.split('/', 1)[-1]} {v}" for k, v in st["out"].most_common(5))
        n = ", ".join(f"{k.split('/', 1)[-1]} {v}" for k, v in st["in"].most_common(5))
        lines.append(f"| {d} | {len(st['files'])} | {st['fns']} | {st['reach']} | {o} | {n} |")
    write(os.path.join(out, "subsystems.md"), lines)
    return stat


def dominator_text(g, sc, out):
    lines = ["# Dominators: every path from the root to a function passes through its parent here.",
             "# A subtree is what its root owns. [n] = functions in the subtree; only subtrees of 8+ are opened.", ""]
    for title, start, nodes in (("main (any scene)", [i for i in G.roots(g) if g.sym[i]["name"] == "main"], sc.any_reach),
                                ("FrameLoop::Frame (per frame, any scene)", sc.frame_roots, sc.any_frame)):
        for s0 in start:
            idom = G.dominators(g, s0, nodes, lambda e: not (e.guard and sc.any_ev.guard_false(e.guard)), sc.any_live)
            kids = defaultdict(list)
            for n, p in idom.items():
                if n != p:
                    kids[p].append(n)
            size = {}

            def sz(n):
                stack, order = [n], []
                while stack:
                    x = stack.pop()
                    order.append(x)
                    stack.extend(kids.get(x, ()))
                for x in reversed(order):
                    size[x] = 1 + sum(size[k] for k in kids.get(x, ()))
                return size[n]
            sz(s0)
            lines.append(f"## {title}: {size[s0]} functions")

            def emit(n, depth):
                lines.append(f"{'  ' * depth}{g.short(n)} [{size[n]}]  {g.sym[n]['file']}:{g.sym[n]['line']}")
                if depth >= 7:
                    return
                big = sorted((k for k in kids.get(n, ()) if size[k] >= 8), key=lambda k: (-size[k], g.short(k)))
                for k in big:
                    emit(k, depth + 1)
                small = len(kids.get(n, ())) - len(big)
                if small:
                    lines.append(f"{'  ' * (depth + 1)}... {small} smaller")
            emit(s0, 0)
            lines.append("")
    cyc = [c for c in G.sccs(g, sc.any_reach) if len(c) > 1]
    lines.append(f"## Recursion: {len(cyc)} cycles of mutually recursive functions (each one node above)")
    for c in sorted(cyc, key=len, reverse=True):
        lines += wrap(f"- [{len(c)}] ", sorted(g.short(x) for x in c), "    ")
    write(os.path.join(out, "dominators.txt"), lines)


def keys_tsv(g, sc, out):
    users = defaultdict(set)
    for k, r in enumerate(g.regions):
        if r[4] == 0:
            continue
        for usr in set(_usrs(r[3])):
            users[G.sid(usr)].add(k)
    lines = ["key\tfield\tconditions reading it\tfunctions they sit in\tvalues across scenes"]
    for path in sorted(g.keys):
        fld = g.keys[path]["field"]
        ks = users.get(fld, ())
        fns = sorted({g.short(g.enclosing(g.regions[k][0], g.regions[k][1][0]) or "") for k in ks} - {""})
        vals = Counter()
        for s in sc.ok:
            v = s["ev"].scene.get(fld)
            vals["?" if v is None else ",".join(sorted(str(x) for x in v))[:24]] += 1
        lines.append(f"{path}\t{g.short(fld)}\t{len(ks)}\t{', '.join(fns[:8])}{' …' if len(fns) > 8 else ''}\t"
                     + " ".join(f"{k}x{n}" for k, n in vals.most_common(4)))
    write(os.path.join(out, "keys.tsv"), lines)


def _usrs(cond):
    import re
    return re.findall(r'\["(?:f|g)","([^"]+)"\]', cond)


def graph_tsv(g, sc, out):
    nodes = ["id\tkind\tname\tfile:line-end\treachable\tscenes\tper-frame-scenes"]
    for i in sorted(g.sym, key=lambda i: (g.sym[i]["file"] or "~", g.sym[i]["line"] or 0, i)):
        s = g.sym[i]
        if not s["repo"] or not (s["kind"] in G.FN_KINDS or s["lang"] == "hlsl"):
            continue
        nodes.append(f"{i}\t{s['kind']}\t{g.qual(i)}\t{s['file']}:{s['line']}-{s['end']}\t{int(i in sc.any_reach)}"
                     f"\t{sc.count.get(i, 0)}\t{sc.frame_count.get(i, 0)}")
    write(os.path.join(out, "graph", "nodes.tsv"), nodes)
    edges = ["from\tto\tkind\tsite\tin-loop\tcondition"]
    rows = []
    for lst in g.out.values():
        for e in lst:
            if e.src not in g.sym or e.dst not in g.sym:
                continue
            rows.append((g.qual(e.src), g.qual(e.dst), e.kind, f"{e.file}:{e.line}", "loop" if e.loop else "",
                         g.show_guard(e.guard)[:240]))
    rows.sort()
    edges += ["\t".join(r) for r in rows]
    write(os.path.join(out, "graph", "edges.tsv"), edges)


def index(g, sc, out, scene_rows, nfiles, rev):
    n_fn = sum(1 for s in g.sym.values() if s["repo"] and s["kind"] in G.FN_KINDS and (s["file"] or "").startswith("src/"))
    lines = [
        "# The code map (generated by tools/codemap after every build -- do not edit)",
        "",
        f"Source {rev}. {n_fn} C++ functions, {sum(1 for s in g.sym.values() if s['lang'] == 'hlsl')} HLSL functions, "
        f"{len(g.entries)} shader entries, {sum(len(v) for v in g.out.values())} edges, {len(g.regions)} conditions, "
        f"{len(g.keys)} scene keys. Of the repo's functions, reachable from main in some scene: "
        f"{g.nrepo(sc.any_reach)}; per frame: {g.nrepo(sc.any_frame)}.",
        f"Call sites: {g.split['clangd']} from clangd, {g.split['libclang_only']} more from libclang (calls clangd drops: "
        f"functions seen only in their own .cpp, local structs, template instantiations); "
        f"{g.split['unknown_callee']} libclang sites name no node (implicit members, a lambda's call operator).",
        "",
        "## Read the smallest thing that answers",
        "- a function: `py -3 tools/codemap/q.py fn NAME` (its card), or the file's map: `src/<path>.map`",
        "- who calls / what it calls, as a tree: `q.py callers NAME -d 3`, `q.py callees NAME -d 2`",
        "- why a scene runs it (or what cut it): `q.py why NAME --scene scenes/x.json`",
        "- a path between two functions: `q.py path A B [--scene S]`",
        "- a scene: `scenes/<path>.map` (its switches, shaders, what it leaves out), `q.py scene S [--diff T]`",
        "- a scene key: `q.py key render.shadowSteps` (field, conditions, values per scene); all in `keys.tsv`",
        "- by meaning, not name: `q.py ask \"terrain shadow march\"` (full-text over code, keys, docs)",
        "- the shape: `subsystems.md`, `dominators.txt`; the picture: `architecture.html`",
        "- what the code does not need or says twice: `redundancy.md` (dead, islands, clones, unread keys, per frame)",
        "- everything, for grep: `graph/nodes.tsv`, `graph/edges.tsv` (one edge a line, with its condition)",
        "",
        "## How it is made",
        "clangd (symbols, every call with the function it sits in, overrides) + libclang (the condition each",
        "line runs under, the values each field can take, the scene key each field is bound to) + dxc (HLSL",
        "calls; C++ Compile() calls link the entries). A scene is folded by `gagame.exe <scene> --print-scene`;",
        "its keys pin field values; a call is cut only when a condition holding it is DEFINITELY false",
        "(three-valued, sound). Virtual calls reach only overrides in classes the reached code builds (RTA).",
        "tools/codemap/README.md has the details and what is not modelled.",
        "",
        "## Scenes (reached functions / per frame / shader entries)",
        "| scene | reached | per frame | shaders |", "|---|---|---|---|",
    ] + scene_rows
    write(os.path.join(out, "INDEX.md"), lines)


def git_rev():
    try:
        r = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=G.ROOT, capture_output=True, text=True).stdout.strip()
        d = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no", "--", "src", "shaders"],
                           cwd=G.ROOT, capture_output=True, text=True).stdout.strip()
        return r + ("-dirty" if d else "")
    except OSError:
        return "unknown"


def main(argv):
    out = os.path.abspath(argv[argv.index("--out") + 1]) if "--out" in argv else os.path.join(G.ROOT, "build", "codemap")
    scene_dir = argv[argv.index("--scenes") + 1] if "--scenes" in argv else "scenes"
    t0 = time.perf_counter()
    g = G.Graph()
    t1 = time.perf_counter()
    sc = Scenes(g, scene_dir)
    t2 = time.perf_counter()
    for sub in ("src", "shaders", "scenes", "graph"):     # stale maps of deleted files must go
        d = os.path.join(out, sub)
        if os.path.isdir(d):
            for dp, _, fs in os.walk(d):
                for f in fs:
                    if f.endswith((".map", ".tsv")):
                        os.remove(os.path.join(dp, f))
    files = file_maps(g, sc, out)
    rows = scene_maps(g, sc, out)
    subsystems(g, sc, out)
    dominator_text(g, sc, out)
    keys_tsv(g, sc, out)
    graph_tsv(g, sc, out)
    index(g, sc, out, rows, len(files), git_rev())
    import audit
    audit.write(g, sc, os.path.join(out, "redundancy.md"))
    try:
        import viz
        viz.write(g, sc, os.path.join(out, "architecture.html"))
    except ImportError:
        pass
    try:
        import rag
        rag.index(g, sc)
    except ImportError:
        pass
    log(f"map: {len(files)} file maps, {len(sc.ok)} scenes ({len(sc.items) - len(sc.ok)} fragments skipped) in {out}; "
        f"load {t1 - t0:.1f}s, scenes {t2 - t1:.1f}s, write {time.perf_counter() - t2:.1f}s")


if __name__ == "__main__":
    main(sys.argv[1:])
