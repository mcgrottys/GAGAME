# ================================================================================================
#  tools/codemap/rag.py -- find code by what it is about, not by its name.
#
#  One SQLite FTS5 table (map_fts in tools/Scriptorium/scriptorium.db), BM25-ranked, rebuilt after
#  every build: every function (its name split into words, its signature, the comment block above
#  its definition), every scene key (with the sentence its Bind gives it), every section of
#  docs/*.md. No model and no network, so a query answers the same today and tomorrow. Each hit
#  names the next, cheaper question: `q.py fn NAME`, `q.py key KEY`, or the doc section.
#
#    py -3 tools/codemap/q.py ask "terrain shadow march" [-k 12]
# ================================================================================================
import json
import os
import re
import sqlite3
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import graph as G  # noqa: E402


def words(name):
    """'TerrainRayCast' -> 'terrain ray cast'; 'm_shadowSteps' -> 'shadow steps'."""
    name = re.sub(r"^[mgk]_", "", name)
    return " ".join(w.lower() for w in re.findall(r"[A-Z]+(?=[A-Z][a-z])|[A-Z]?[a-z]+|[A-Z]+|\d+", name))


def comment_above(lines, line):
    """The // block (or /* */ tail) directly above a 1-based line, at most 30 lines."""
    out, k = [], line - 2
    while k >= 0 and len(out) < 30:
        t = lines[k].strip()
        if t.startswith("//") or t.startswith("*") or t.startswith("/*"):
            out.append(t.lstrip("/*! ").rstrip("*/ "))
            k -= 1
        elif not t and out:
            break
        elif not t:
            k -= 1
        else:
            break
    return " ".join(reversed(out))


def index(g, sc=None):
    db = sqlite3.connect(G.DB, timeout=60)
    db.execute("DROP TABLE IF EXISTS map_fts")
    db.execute("CREATE VIRTUAL TABLE map_fts USING fts5(kind, ref, title, body, tokenize='porter unicode61')")
    rows, cache = [], {}

    def src(f):
        if f not in cache:
            try:
                with open(os.path.join(G.ROOT, f), encoding="utf-8", errors="replace") as fh:
                    cache[f] = fh.read().splitlines()
            except OSError:
                cache[f] = []
        return cache[f]
    for i in sorted(g.sym, key=lambda i: (g.sym[i]["file"] or "~", g.sym[i]["line"] or 0, i)):
        s = g.sym[i]
        if not s["repo"] or not s["file"] or not (s["kind"] in G.FN_KINDS or s["lang"] == "hlsl"):
            continue
        name = g.short(i)
        body = " ".join([words(s["name"]), words(s["scope"].replace("::", " ")), s["file"], s.get("sig") or "",
                         comment_above(src(s["file"]), s["line"])])
        rows.append(("fn", f"{s['file']}:{s['line']}", name, body))
    docs = {}
    for callee, name, args in db.execute("SELECT callee, name, args FROM ast_litcalls WHERE name IN ('Bind','BindEnum','Nest','List')"):
        a = json.loads(args)
        strs = [x for x in a[1:] if isinstance(x, str)]
        if a and isinstance(a[0], str) and strs:
            docs.setdefault(a[0], max(strs, key=len))
    for path in sorted(g.keys):
        leaf = path.rsplit(".", 1)[-1]
        rows.append(("key", path, path, f"{words(leaf)} {path.replace('.', ' ')} {g.short(g.keys[path]['field'])} {docs.get(leaf, '')}"))
    ddir = os.path.join(G.ROOT, "docs")
    for f in sorted(os.listdir(ddir)):
        if not f.endswith(".md"):
            continue
        with open(os.path.join(ddir, f), encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        for m in re.finditer(r"^(#{1,3} .+)$([\s\S]*?)(?=^#{1,3} |\Z)", text, re.M):
            title = m.group(1).lstrip("# ").strip()
            line = text.count("\n", 0, m.start()) + 1
            rows.append(("doc", f"docs/{f}:{line}", f"{f}: {title}", m.group(2)[:4000]))
    db.executemany("INSERT INTO map_fts VALUES(?,?,?,?)", rows)
    db.commit()
    return len(rows)


def ask(text, k=12):
    db = sqlite3.connect(G.DB, timeout=60)
    terms = [t for t in re.findall(r"\w+", text.lower()) if len(t) > 1]
    if not terms:
        print("nothing to search for")
        return
    q = " OR ".join(f'"{t}"' for t in terms)
    try:
        hits = db.execute("SELECT kind, ref, title, snippet(map_fts, 3, '[', ']', ' ... ', 14), "
                          "bm25(map_fts, 0, 0, 6.0, 1.0) AS s FROM map_fts WHERE map_fts MATCH ? ORDER BY s, ref LIMIT ?",
                          (q, k)).fetchall()
    except sqlite3.OperationalError as ex:
        print(f"no index ({ex}); build first: py -3 tools/codemap/build.py")
        return
    for kind, ref, title, snip, score in hits:
        nxt = {"fn": f"q.py fn {title}", "key": f"q.py key {title}", "doc": ref}[kind]
        print(f"{kind:3} {title[:60]:60} {ref}\n      {snip[:150]}\n      -> {nxt}")


if __name__ == "__main__":
    ask(" ".join(sys.argv[1:]))
