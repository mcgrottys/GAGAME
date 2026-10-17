# ================================================================================================
#  tools/codemap/q.py -- ask the code map one question, get a short answer.
#
#    q.py fn NAME [--scene S]                 the function's card (callers, calls by condition, ...)
#    q.py callers NAME [-d 3] [--scene S]     who calls it, as a tree (cut edges left out under S)
#    q.py callees NAME [-d 2] [--scene S]     what it calls, as a tree (repo functions only)
#    q.py path A B [--scene S]                the shortest call path from A to B, with its conditions
#    q.py why NAME --scene S                  the chain main -> NAME under S, or what cuts it
#    q.py scene S [--diff T]                  what S reaches; with --diff, what differs from T
#    q.py key KEY [--scene S]                 a scene key: its field, the conditions that read it
#    q.py ask TEXT [-k 12]                    full-text search over code, keys and docs (rag.py)
#    q.py dom NAME                            its dominator chain from main: what every path passes
#    q.py file PATH                           the file's map (build/codemap/<PATH>.map)
#
#  NAME: 'Assemble', 'FrameLoop::Frame', 'ga::app::Assemble', 'PsSurface', 'Globe.hlsl#PsSurface'.
#  Output is capped (--limit, default 40 lines a list); a cap says how much it held back.
# ================================================================================================
import json
import os
import sys
from collections import Counter, deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import graph as G  # noqa: E402


def opt(argv, name, default=None):
    if name in argv:
        k = argv.index(name)
        v = argv[k + 1]
        del argv[k:k + 2]
        return v
    return default


class Ctx:
    def __init__(self, scene):
        self.g = G.Graph()
        self.scene = scene
        self.ev = G.Eval(self.g, G.fold_scene(scene) if scene else None)
        self.reach, self.live, _ = G.reach(self.g, self.ev)

    def one(self, name):
        hits = self.g.find(name)
        if not hits:
            sys.exit(f"no function named {name!r}")
        if len(hits) > 1:
            print(f"({len(hits)} match {name!r}; using the first -- name it fully to choose: "
                  + ", ".join(f"{self.g.qual(h)}@{self.g.sym[h]['file']}:{self.g.sym[h]['line']}" for h in hits[:5]) + ")")
        return hits[0]

    def cut(self, e):
        return bool(e.guard) and self.ev.guard_false(e.guard)

    def cond(self, e):
        return self.g.show_guard(e.guard, self.ev if self.scene else None)

    def targets(self, e):
        out = [e.dst]
        for o in self.g.all_overrides(e.dst):
            c = self.g.class_of(o)
            if c is None or c in self.live:
                out.append(o)
        return out

    def where(self, i):
        s = self.g.sym.get(i, {})
        return f"{s.get('file')}:{s.get('line')}"


def tree(c, root, down, depth, limit, repo_only=True):
    g, lines, seen = c.g, [], {root}

    def walk(n, d):
        if len(lines) >= limit:
            return
        edges = g.out.get(n, ()) if down else g.inc.get(n, ())
        rows = {}
        for e in edges:
            other = e.dst if down else e.src
            if other not in g.sym or (repo_only and not g.sym[other]["repo"]) or c.cut(e):
                continue
            if other not in rows:
                rows[other] = e
        for other, e in sorted(rows.items(), key=lambda kv: c.g.short(kv[0])):
            if len(lines) >= limit:
                lines.append(f"{'  ' * d}... (limit; raise --limit)")
                return
            cond = c.cond(e)
            mark = "" if other not in seen else "  (above)"
            lines.append(f"{'  ' * d}{g.short(other)}  {e.file.rsplit('/', 1)[-1]}:{e.line}"
                         f"{'  [' + cond[:100] + ']' if cond else ''}{'  loop' if e.loop else ''}{mark}")
            if other not in seen and d < depth:
                seen.add(other)
                walk(other, d + 1)
    walk(root, 1)
    return lines


def bfs(c, start, goal=None):
    """Parents of a breadth-first walk from start over the edges the scene does not cut."""
    par, todo = {s: None for s in start}, deque(start)
    while todo:
        n = todo.popleft()
        if n == goal:
            break
        for e in sorted(c.g.out.get(n, ()), key=lambda e: (e.file, e.line, e.col)):
            if c.cut(e):
                continue
            for t in c.targets(e):
                if t not in par:
                    par[t] = (e, t)       # t: the callee, or the override a virtual call reaches
                    todo.append(t)
    return par


def chain(c, par, goal):
    out, n = [], goal
    while par.get(n) is not None:
        out.append(par[n])
        n = par[n][0].src
    return list(reversed(out))


def show_chain(c, steps):
    g = c.g
    if not steps:
        return []
    lines = [f"{g.short(steps[0][0].src)}  {c.where(steps[0][0].src)}"]
    for e, t in steps:
        cond = c.cond(e)
        via = f" (virtual {g.short(e.dst)})" if t != e.dst else ""
        lines.append(f"  -> {g.short(t)}{via}   at {e.file}:{e.line}{'  if ' + cond[:140] if cond else ''}")
    return lines


def explain_cut(c, e):
    """Why an edge is cut: its regions decided false, and the values that decided them."""
    import re
    g, out = c.g, []
    for k in e.guard:
        if c.ev.region_truth(k) is not False:
            continue
        r = g.regions[k]
        t = g.show(json.loads(r[3]))
        out.append(f"      cut: needs {g.show_region(k)[:120]}  [{r[0]}:{r[1][0]}]")
        # the parts that decided it: every name and call in the condition whose value is known
        seen = set()

        def walk(x):
            if not isinstance(x, list) or not x:
                return
            if x[0] in ("f", "g", "l", "call"):
                t_ = g.show(x)
                v = c.ev.expr(x)
                if v is not None and t_ not in seen:
                    seen.add(t_)
                    out.append(f"        {t_[:80]} = {', '.join(map(str, sorted(v, key=str)))}")
                if x[0] != "call":
                    return
            for y in x[1:]:
                walk(y)
        walk(json.loads(r[3]))
    return out


def main(argv):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")   # source comments are UTF-8
    if not argv:
        with open(__file__, encoding="utf-8") as fh:      # the usage is the header above
            print("".join(l[3:] if l.startswith("#  ") else "\n" for l in fh.readlines()[1:19]))
        return
    scene = opt(argv, "--scene")
    depth = int(opt(argv, "-d", "2"))
    limit = int(opt(argv, "--limit", "40"))
    k = int(opt(argv, "-k", "12"))
    diff = opt(argv, "--diff")
    cmd, args = argv[0], argv[1:]
    if cmd == "scene" and not scene and args:   # `scene S [--diff T]`: S is positional
        scene = args[0]

    if cmd == "file":
        p = os.path.join(G.ROOT, "build", "codemap", args[0] + ".map")
        print(open(p, encoding="utf-8").read() if os.path.exists(p) else f"no map at {p} (build first)")
        return
    if cmd == "ask":
        import rag
        rag.ask(" ".join(args), k)
        return
    c = Ctx(scene)
    g = c.g
    if cmd == "fn":
        import build
        i = c.one(args[0])
        print("\n".join(build.card(g, i)))
        if scene:
            print(f"  under {scene}: {'REACHED' if i in c.reach else 'NOT reached'}")
    elif cmd in ("callers", "callees"):
        i = c.one(args[0])
        print(f"{g.short(i)}  {c.where(i)}")
        print("\n".join(tree(c, i, cmd == "callees", depth, limit)) or
              f"  (no {cmd} in the repo{' under ' + scene if scene else ''})")
    elif cmd == "path":
        a, b = c.one(args[0]), c.one(args[1])
        par = bfs(c, [a], b)
        print("\n".join(show_chain(c, chain(c, par, b))) if b in par else f"no call path from {g.short(a)} to {g.short(b)}"
              + (f" under {scene}" if scene else ""))
    elif cmd == "why":
        i = c.one(args[0])
        roots = G.roots(g)
        par = bfs(c, roots, i)
        if i in par:
            print(f"REACHED under {scene or 'some scene'}:")
            print("\n".join(show_chain(c, chain(c, par, i))))
        else:
            print(f"NOT reached under {scene or 'any scene'}. Its callers:")
            for e in sorted(g.inc.get(i, ()), key=lambda e: (g.short(e.src), e.line))[:limit]:
                state = "caller not reached" if e.src not in par else ("CUT" if c.cut(e) else "?")
                print(f"  {g.short(e.src)}  {e.file}:{e.line}  {state}")
                if state == "CUT":
                    print("\n".join(explain_cut(c, e)))
    elif cmd == "scene":
        fr = G.closure(g, c.ev, [x for q in G.FRAME for x in g.by_qual.get(q, ())], c.live)
        print(f"{scene}: reached {g.nrepo(c.reach)} of the repo's functions, per frame {g.nrepo(fr)}, live classes {len(c.live)}")
        if diff:
            d = Ctx(diff)
            for title, a, b in ((f"only in {scene}", c.reach, d.reach), (f"only in {diff}", d.reach, c.reach)):
                files = Counter(g.sym[x]["file"] for x in a - b if g.sym.get(x, {}).get("repo"))
                print(f"{title}: {sum(files.values())} functions")
                for f, n in files.most_common(limit):
                    print(f"  {f}: {n}")
            flips = []
            for k_ in range(len(g.regions)):
                x, y = c.ev.region_truth(k_), d.ev.region_truth(k_)
                if x is not y and (x is not None and y is not None):
                    flips.append(k_)
            print(f"conditions decided differently: {len(flips)}")
            for k_ in flips[:limit]:
                r = g.regions[k_]
                print(f"  {r[0]}:{r[1][0]}  {g.show(json.loads(r[3]))[:110]}  {c.ev.region_truth(k_)} vs {d.ev.region_truth(k_)}")
        else:
            p = os.path.join(G.ROOT, "build", "codemap", "scenes", scene[len("scenes/"):].rsplit(".", 1)[0] + ".map")
            if os.path.exists(p):
                print(f"(the full summary: {os.path.relpath(p, G.ROOT)})")
    elif cmd == "key":
        key = args[0]
        if key not in g.keys:
            near = [x for x in g.keys if key in x][:20]
            sys.exit(f"no key {key!r}" + (f"; near: {', '.join(near)}" if near else ""))
        fld = g.keys[key]["field"]
        print(f"{key} -> {g.short(fld)} ({g.sym.get(fld, {}).get('file')}:{g.sym.get(fld, {}).get('line')})"
              + (f"  enum {g.keys[key]['enum']}" if g.keys[key]["enum"] else ""))
        if scene:
            v = c.ev.var(fld)
            print(f"value under {scene}: {'unknown' if v is None else sorted(v, key=str)}")
        n = 0
        for kk, r in enumerate(g.regions):
            if r[4] != 0 and fld in {G.sid(u) for u in _usrs(r[3])}:
                n += 1
                if n <= limit:
                    t = g.show(json.loads(r[3]))
                    st = c.ev.region_truth(kk) if scene else None
                    fn = g.enclosing(r[0], r[1][0])
                    print(f"  {r[0]}:{r[1][0]}  in {g.short(fn) if fn else '?'}  {'!' if r[4] == -1 else ''}({t[:100]})"
                          + (f"  -> {st}" if scene else ""))
        print(f"{n} conditions read it" + (" (capped)" if n > limit else ""))
    elif cmd == "dom":
        i = c.one(args[0])
        main_ = [x for x in G.roots(g) if g.sym[x]["name"] == "main"][0]
        idom = G.dominators(g, main_, c.reach, lambda e: not c.cut(e), c.live)
        if i not in idom:
            sys.exit(f"{g.short(i)} is not reached from main")
        n, out = i, []
        while idom[n] != n:
            n = idom[n]
            out.append(g.short(n))
        print(f"{g.short(i)} is dominated by (nearest first): " + " <- ".join(out))
    else:
        sys.exit(f"unknown command {cmd!r}; run with no arguments for the list")


def _usrs(cond):
    import re
    return re.findall(r'\["(?:f|g)","([^"]+)"\]', cond)


if __name__ == "__main__":
    main(sys.argv[1:])
