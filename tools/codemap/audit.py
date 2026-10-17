# ================================================================================================
#  tools/codemap/audit.py -- build/codemap/redundancy.md: what the map says the code does not need,
#  says twice, or does every frame. Each section names how it was found and what it cannot see.
#
#    1 functions no scene and no flag reaches -- named nowhere else (dead), called only from dead
#      code (a dead island), or overrides of a method nothing dispatches
#    2 bodies written twice: exact (type-1) and with names and numbers renamed (type-2), C++ and HLSL
#    3 scene keys whose field nothing reads but the scene loader
#    4 branches no scene and no flag can take (a condition decided false for every one)
#    5 per frame: what allocates or looks things up by string inside FrameLoop::Frame's reach
# ================================================================================================
import hashlib
import json
import os
import re
import sys
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import graph as G  # noqa: E402

TOKEN = re.compile(r'[A-Za-z_]\w*|\d[\w.]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|::|->|<<=|>>=|[-+*/%&|^!=<>]=|&&|\|\||<<|>>|\+\+|--|[^\s\w]')
KEYWORDS = set("""auto bool break case catch char class const constexpr continue default delete do double else enum
explicit false float for if inline int long namespace new noexcept nullptr operator private protected public return
short signed sizeof static static_cast struct switch template this throw true try typedef typename uint uint2 uint3
uint4 float2 float3 float4 float4x4 int2 int3 int4 unsigned using virtual void volatile while size_t uint32_t uint64_t
int32_t int64_t uint8_t uint16_t in out inout""".split())


def body_tokens(lines, a, b):
    text = "\n".join(lines[a - 1:b])
    text = re.sub(r"//[^\n]*", " ", text)
    text = re.sub(r"/\*[\s\S]*?\*/", " ", text)
    k = text.find("{")
    if k < 0:
        return []
    return TOKEN.findall(text[k:])


def normal(tokens):
    out = []
    for t in tokens:
        if t[0].isalpha() or t[0] == "_":
            out.append(t if t in KEYWORDS else "ID")
        elif t[0].isdigit():
            out.append("N")
        elif t[0] in "\"'":
            out.append("S")
        else:
            out.append(t)
    return out


def write(g, sc, path):
    src = {}

    def lines(f):
        if f not in src:
            try:
                with open(os.path.join(G.ROOT, f), encoding="utf-8", errors="replace") as fh:
                    src[f] = fh.read().splitlines()
            except OSError:
                src[f] = []
        return src[f]
    ev = G.Eval(g, None, argv=True)
    R, live, _ = G.reach(g, ev)
    fns = sorted((i for i, s in g.sym.items() if (s["kind"] in G.FN_KINDS or s["lang"] == "hlsl") and s["repo"]
                  and (s["file"] or "").startswith(("src/", "shaders/")) and s["end"] > s["line"]),
                 key=lambda i: (g.sym[i]["file"], g.sym[i]["line"], i))
    out = ["# Redundancy, read from the code map (tools/codemap/audit.py; regenerated after every build)", "",
           f"{len(fns)} functions with a body; reachable from main under some scene and some flag: "
           f"{sum(1 for i in fns if i in R)}. Every list is sorted and complete unless it says otherwise.", ""]

    # ---- 1 unreachable
    words = Counter()
    for d in ("src", "shaders"):
        for dp, _, fs in os.walk(os.path.join(G.ROOT, d)):
            for f in fs:
                rel = os.path.relpath(os.path.join(dp, f), G.ROOT).replace("\\", "/")
                words.update(re.findall(r"[A-Za-z_]\w*", "\n".join(lines(rel))))
    dead = [i for i in fns if i not in R]
    undispatched, island, named_nowhere, other = [], [], [], []
    called = {e.dst for lst in g.out.values() for e in lst}
    for i in dead:
        s = g.sym[i]
        bases = [b for b, lst in g.overrides.items() if i in lst]
        if bases and all(b not in called for b in bases):
            undispatched.append((i, bases))
        elif g.inc.get(i):
            island.append(i)
        elif s["kind"] == "Destructor" or s["name"].startswith(("~", "operator")):
            other.append(i)           # run implicitly (a scope's end, a hash table's hasher): never "dead"
        elif words[s["name"]] <= 2:
            named_nowhere.append(i)
        else:
            other.append(i)
    loc = lambda i: f"{g.sym[i]['file']}:{g.sym[i]['line']}-{g.sym[i]['end']}"   # noqa: E731
    out += ["## 1. Functions no scene and no flag reaches", "",
            "Reach: from main and the static initializers, every call that is not definitely cut for EVERY scene "
            "and with any command-line flag, virtual calls into the classes the reached code builds. "
            "Blind to: a call through a pointer the code computes (not named), an entry point the OS calls "
            "without the code naming it.", "",
            f"### 1a. Named nowhere else in src/ or shaders/ ({len(named_nowhere)}): dead", ""]
    out += [f"- `{g.short(i)}` {loc(i)} ({g.sym[i]['end'] - g.sym[i]['line'] + 1} lines)" for i in named_nowhere]
    out += ["", f"### 1b. Called only from code that is itself unreached ({len(island)}): dead islands", ""]
    out += [f"- `{g.short(i)}` {loc(i)} <- {', '.join(sorted({g.short(e.src) for e in g.inc[i]}))[:120]}" for i in island]
    out += ["", f"### 1c. Overrides of a method nothing calls through its base ({len(undispatched)}): an interface "
            "nobody dispatches", ""]
    byb = defaultdict(list)
    for i, bases in undispatched:
        byb[", ".join(sorted(g.short(b) for b in bases))].append(i)
    out += [f"- `{b}` (no caller): overridden by {', '.join(sorted(g.short(i) for i in v))}" for b, v in sorted(byb.items())]
    out += ["", f"### 1d. Unreached, but its name is spelled elsewhere ({len(other)}): check by hand (an overload, "
            "a name shared with live code, or a map gap)", ""]
    out += [f"- `{g.short(i)}` {loc(i)} (name appears {words[g.sym[i]['name']]}x)" for i in other]

    # ---- 2 clones
    t1, t2 = defaultdict(list), defaultdict(list)
    ntok = {}
    for i in fns:
        s = g.sym[i]
        toks = body_tokens(lines(s["file"]), s["line"], s["end"])
        ntok[i] = len(toks)
        if len(toks) >= 25:
            t1[hashlib.sha1(" ".join(toks).encode()).hexdigest()].append(i)
        if len(toks) >= 50:
            t2[hashlib.sha1(" ".join(normal(toks)).encode()).hexdigest()].append(i)
    groups = []
    exact = {frozenset(ids) for ids in t1.values() if len(ids) > 1}
    for kind, tbl in (("exact", t1), ("renamed", t2)):
        for ids in tbl.values():
            if len(ids) > 1 and not (kind == "renamed" and frozenset(ids) in exact):
                groups.append((ntok[ids[0]] * (len(ids) - 1), kind, ids))
    groups.sort(key=lambda x: (-x[0], x[1], [loc(i) for i in x[2]]))
    out += ["", "## 2. Bodies written twice", "",
            "Each function's body as tokens (comments and spacing dropped): EXACT = the same tokens; RENAMED = the "
            "same after every name, number and string is replaced by a placeholder (the same shape of code). Ranked "
            "by redundant tokens (tokens x copies beyond the first). Blind to: a copy with one statement changed.", ""]
    for red, kind, ids in groups[:60]:
        langs = {g.sym[i]["lang"] for i in ids}
        out.append(f"- {kind}, {ntok[ids[0]]} tokens x{len(ids)}{' (HLSL)' if langs == {'hlsl'} else ''}: "
                   + "; ".join(f"`{g.short(i)}` {loc(i)}" for i in sorted(ids, key=loc)))
    if len(groups) > 60:
        out.append(f"- ... {len(groups) - 60} smaller groups")

    # ---- 3 scene keys nobody reads
    reads = defaultdict(set)
    for usr, cont in g.db.execute("SELECT usr, container FROM ast_refs WHERE lang='cpp' AND kind='read'"):
        reads[G.sid(usr)].add(G.sid(cont) if cont else "")
    unread = []
    for key in sorted(g.keys):
        fld = g.keys[key]["field"]
        users = {c for c in reads.get(fld, ()) if c and c not in g.schema_fns}
        if not users:
            unread.append(key)
    out += ["", f"## 3. Scene keys nothing reads but the scene loader ({len(unread)})", "",
            "The key's field has no read outside the schema functions (a list element's key may be read by "
            "position through its parent; check before removing).", ""]
    out += [f"- `{p}` -> {g.short(g.keys[p]['field'])}" for p in unread]

    # ---- 4 branches no scene and no flag takes
    never = []
    for k, r in enumerate(g.regions):
        if r[4] == 0 or ev.region_truth(k) is not False:
            continue
        fn = g.enclosing(r[0], r[1][0])
        if fn in R:
            never.append((r[0], r[1][0], r[2][0], g.show_region(k), fn))
    never.sort()
    out += ["", f"## 4. Branches no scene and no flag can take ({len(never)}, in reached functions)", "",
            "The condition is decided false with every scene key unknown and any flag allowed: what pins it is a "
            "constant, a default nothing ever changes, or a field only ever given one value.", ""]
    out += [f"- {f}:{a}-{b} in `{g.short(fn)}`: needs `{t[:110]}`" for f, a, b, t, fn in never[:150]]
    if len(never) > 150:
        out.append(f"- ... {len(never) - 150} more")

    # ---- 5 per frame
    frame = G.closure(g, ev, [x for q in G.FRAME for x in g.by_qual.get(q, ())], live)
    hot = re.compile(r"^(make_unique|make_shared|operator new|basic_string|vector|map|unordered_map|push_back|emplace_back|"
                     r"insert|emplace|operator\[\]|function|resize|reserve|assign|append|find|Tool|LayerOn|Layer|Effect|"
                     r"EffectOfType|Portal|Entity|View|snprintf|Log)$")
    rows = []
    for i in sorted(frame, key=lambda i: (g.sym.get(i, {}).get("file") or "", g.sym.get(i, {}).get("line") or 0)):
        s = g.sym.get(i)
        if not s or not s["repo"] or s["lang"] != "cpp":
            continue
        c = Counter()
        for e in g.out.get(i, ()):
            t = g.sym.get(e.dst)
            # operator[] allocates only on a map (vector and array subscripts are loads)
            if t and hot.match(t["name"]) and not (t["name"] == "operator[]" and "map" not in t["scope"]) \
                    and not (e.guard and ev.guard_false(e.guard)):
                c[g.short(e.dst) + ("*" if e.loop else "")] += 1
        if c:
            rows.append((sum(c.values()), i, c))
    rows.sort(key=lambda x: (-x[0], loc(x[1])))
    out += ["", f"## 5. Per frame: allocations and lookups by name ({len(rows)} functions)", "",
            f"Inside what FrameLoop::Frame reaches ({len(frame)} functions, any scene): calls that allocate "
            "(make_unique, vector/string construction, push_back, map insert) or look up by string (the scene's "
            "Tool/Layer/Effect, map find). * = inside a loop. A list of where to measure, not a measurement.", ""]
    out += [f"- `{g.short(i)}` {loc(i)}: " + ", ".join(f"{k} x{v}" for k, v in sorted(c.items())) for n, i, c in rows[:80]]
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(out).rstrip() + "\n")
    return {"dead": len(named_nowhere), "islands": len(island), "undispatched": len(undispatched), "clones": len(groups),
            "unread_keys": len(unread), "never": len(never), "frame_alloc": len(rows)}
