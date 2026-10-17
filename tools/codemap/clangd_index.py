# ================================================================================================
#  tools/codemap/clangd_index.py -- clangd's view of the C++: symbols, references (each with the
#  function it sits in), and the base-of / overridden-by relations, for every TU of
#  build/compile_commands.json.
#
#  One `clangd-indexer --format=binary` per TU (riff.py reads its output), cut down to what the
#  repo touches -- refs located in the repo, the symbols they name, the relations between them --
#  and kept as a shard per TU under build/codemap/clangd/. A TU is re-indexed only when its
#  command or a file it read changed; the file list is the one tools/astindex recorded for the same
#  TU (ast_tus), so the two indexers agree on when a TU is stale. The shards are then merged into
#  the cd_* tables of tools/Scriptorium/scriptorium.db, sorted, so the tables are the same whatever
#  order the TUs finished in.
#
#  clangd is found as GA_CLANGD_DIR\bin, then on PATH, then the newest %USERPROFILE%\tools\clangd\*.
#  Not under %LOCALAPPDATA%: a packaged app (the Claude desktop app) sees that folder virtualized,
#  and a build run from a plain console would not find what was unpacked there.
# ================================================================================================
import glob
import hashlib
import json
import os
import pickle
import shutil
import sqlite3
import subprocess
import sys
import time
import urllib.parse
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import riff  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
ROOT_N = os.path.normcase(ROOT)
DB = os.path.join(ROOT, "tools", "Scriptorium", "scriptorium.db")

SCHEMA = """
CREATE TABLE IF NOT EXISTS cd_tus(tu TEXT PRIMARY KEY, key TEXT, seconds REAL);
-- Every symbol the repo declares or names. file/line/col: the canonical declaration (1-based, '' when
-- outside the repo); def_*: the definition. kind: clangd's SymbolKind name.
CREATE TABLE IF NOT EXISTS cd_symbols(id TEXT PRIMARY KEY, kind TEXT, name TEXT, scope TEXT,
    file TEXT, line INTEGER, col INTEGER, def_file TEXT, def_line INTEGER, def_col INTEGER,
    def_end INTEGER, signature TEXT, type TEXT, ret TEXT, in_repo INTEGER);
CREATE INDEX IF NOT EXISTS cd_symbols_name ON cd_symbols(name);
-- Every reference located in the repo. kind: clangd's RefKind bits (1 decl, 2 def, 4 ref, 8 spelled,
-- 16 call). container: the symbol the reference sits in (a function, or a variable's initializer).
CREATE TABLE IF NOT EXISTS cd_refs(id TEXT, file TEXT, line INTEGER, col INTEGER, kind INTEGER,
    container TEXT);
CREATE INDEX IF NOT EXISTS cd_refs_id ON cd_refs(id);
CREATE INDEX IF NOT EXISTS cd_refs_container ON cd_refs(container);
CREATE INDEX IF NOT EXISTS cd_refs_file ON cd_refs(file, line);
-- predicate: 0 BaseOf (subject is a base of object), 1 OverriddenBy (object overrides subject).
CREATE TABLE IF NOT EXISTS cd_rel(subject TEXT, predicate INTEGER, object TEXT);
"""


def log(msg):
    print(f"[codemap] {time.strftime('%H:%M:%S')} clangd: {msg}", flush=True)


def find_indexer():
    names = ("clangd-indexer.exe", "clangd-indexer")
    d = os.environ.get("GA_CLANGD_DIR")
    if d:
        for n in names:
            for p in (os.path.join(d, "bin", n), os.path.join(d, n)):
                if os.path.isfile(p):
                    return p
    p = shutil.which("clangd-indexer")
    if p:
        return p
    found = glob.glob(os.path.join(os.path.expanduser("~"), "tools", "clangd", "*", "clangd_*", "bin", names[0]))

    def ver(path):
        return [int(x) if x.isdigit() else 0 for x in os.path.basename(os.path.dirname(os.path.dirname(path)))[7:].split(".")]
    return max(found, key=ver) if found else None


_uri_cache = {}


def uri_rel(uri):
    """'file:///C:/x/y.h' -> repo-relative 'src/y.h', or None outside the repo (or in build/)."""
    if not uri:
        return None            # no location (a symbol never defined): not abspath(""), the cwd
    r = _uri_cache.get(uri, 0)
    if r == 0:
        p = os.path.abspath(urllib.parse.unquote(uri[8:] if uri.startswith("file:///") else uri))
        r = None
        if os.path.normcase(p).startswith(ROOT_N + os.sep):     # compared folded, kept in its own case
            r = p[len(ROOT_N) + 1:].replace("\\", "/")
            if r == "build" or r.startswith("build/"):
                r = None
        _uri_cache[uri] = r
    return r


def shard_of(data):
    """Cut one TU's index down to the repo: refs located in it, the symbols those refs and the repo's
    own declarations name, the relations among them. Lines and columns 1-based."""
    ix = riff.read(data)
    refs = []
    keep = set()
    for sid, lst in ix["refs"].items():
        for kind, loc, cont in lst:
            f = uri_rel(loc[0])
            if f is not None:
                refs.append((sid, f, loc[1] + 1, loc[2] + 1, kind, cont))
                keep.add(sid)
                keep.add(cont)
    syms = []
    for s in ix["symbols"]:
        df, cf = uri_rel(s["def"][0]), uri_rel(s["decl"][0])
        if df is None and cf is None and s["id"] not in keep:
            continue
        syms.append((s["id"], s["kind"], s["name"], s["scope"],
                     cf or "", s["decl"][1] + 1 if cf else 0, s["decl"][2] + 1 if cf else 0,
                     df or "", s["def"][1] + 1 if df else 0, s["def"][2] + 1 if df else 0, 0,
                     s["signature"], s["type"], s["ret"], 1 if (df or cf) else 0))
        keep.add(s["id"])
    rels = [r for r in ix["relations"] if r[0] in keep or r[2] in keep]
    return {"refs": refs, "symbols": syms, "relations": rels}


def index_tu(job):
    indexer, build, path = job
    t0 = time.perf_counter()
    p = subprocess.run([indexer, "-p", build, "--format=binary", path], capture_output=True,
                       creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    if p.returncode != 0 or not p.stdout:
        return path, None, p.stderr.decode("utf-8", "replace")[-400:], time.perf_counter() - t0
    return path, shard_of(p.stdout), "", time.perf_counter() - t0


def tu_keys(db, cc):
    """Per TU: a key over its command and the (mtime, size) of every file astindex saw it read."""
    deps = {}
    try:
        for tu, d in db.execute("SELECT tu, deps FROM ast_tus"):
            deps[tu] = json.loads(d)
    except sqlite3.OperationalError:
        pass
    keys = {}
    for e in cc:
        path = os.path.abspath(os.path.join(e["directory"], e["file"]))
        rel = os.path.relpath(path, ROOT).replace("\\", "/")
        if rel.startswith(".."):
            continue
        h = hashlib.sha1(e["command"].encode())
        dl = deps.get(rel)
        if dl is None:
            h.update(str(time.time_ns()).encode())     # nothing known of its reads: always stale
        else:
            for p, _ in dl:
                try:
                    st = os.stat(p)
                    h.update(f"{p}|{st.st_mtime_ns}|{st.st_size}".encode())
                except OSError:
                    h.update(f"{p}|gone".encode())
        keys[rel] = (path, h.hexdigest())
    return keys


def run(build, jobs, full=False):
    indexer = find_indexer()
    if not indexer:
        log("skipped: no clangd-indexer (set GA_CLANGD_DIR, or unpack clangd_indexing_tools to ~/tools/clangd/<ver>)")
        return False
    t0 = time.perf_counter()
    shard_dir = os.path.join(build, "codemap", "clangd")
    os.makedirs(shard_dir, exist_ok=True)
    with open(os.path.join(build, "compile_commands.json"), encoding="utf-8") as fh:
        cc = json.load(fh)
    db = sqlite3.connect(DB, timeout=60)
    db.execute("PRAGMA journal_mode=WAL")
    db.executescript(SCHEMA)
    keys = tu_keys(db, cc)
    old = dict(db.execute("SELECT tu, key FROM cd_tus"))

    def shard_path(rel):
        return os.path.join(shard_dir, hashlib.sha1(rel.encode()).hexdigest()[:16] + ".pkl")

    todo = sorted(rel for rel, (_, k) in keys.items()
                  if full or old.get(rel) != k or not os.path.exists(shard_path(rel)))
    if todo:
        log(f"{len(todo)} of {len(keys)} TUs to index on {jobs} processes ({indexer})")
        fails = 0
        # processes, not threads: reading a 9 MB index is Python work, and threads would hold the GIL
        with ProcessPoolExecutor(max_workers=jobs) as ex:
            for path, shard, err, secs in ex.map(index_tu, [(indexer, build, keys[r][0]) for r in todo]):
                rel = os.path.relpath(path, ROOT).replace("\\", "/")
                if shard is None:
                    fails += 1
                    log(f"FAILED {rel}: {err.strip()}")
                    db.execute("DELETE FROM cd_tus WHERE tu=?", (rel,))
                    continue
                with open(shard_path(rel), "wb") as fh:
                    pickle.dump(shard, fh, protocol=pickle.HIGHEST_PROTOCOL)
                db.execute("INSERT OR REPLACE INTO cd_tus VALUES(?,?,?)", (rel, keys[rel][1], round(secs, 2)))
        db.commit()
        log(f"indexed {len(todo) - fails} TUs in {time.perf_counter() - t0:.1f}s, {fails} failed")
    for rel in sorted(set(old) - set(keys)):
        db.execute("DELETE FROM cd_tus WHERE tu=?", (rel,))
        try:
            os.remove(shard_path(rel))
        except OSError:
            pass
    if not todo and db.execute("SELECT COUNT(*) FROM cd_refs").fetchone()[0]:
        log(f"no TU changed ({len(keys)} checked)")
        db.commit()
        return True

    # Merge every shard. A symbol seen from several TUs: the row with a definition wins, then the
    # first in TU order, so the merge does not depend on which TU finished first.
    t1 = time.perf_counter()
    syms, refs, rels = {}, set(), set()
    for rel in sorted(keys):
        sp = shard_path(rel)
        if not os.path.exists(sp):
            continue
        with open(sp, "rb") as fh:
            sh = pickle.load(fh)
        for s in sh["symbols"]:
            cur = syms.get(s[0])
            if cur is None or (not cur[7] and s[7]) or (not cur[4] and s[4]):
                syms[s[0]] = s
        refs.update(sh["refs"])
        rels.update(sh["relations"])
    # def_end: the definition's last line, from the next sibling clangd cannot give; astindex has it.
    ends = {}
    try:
        for f, l, e in db.execute("SELECT file, line, end_line FROM ast_decls WHERE lang='cpp' AND is_def=1"):
            ends[(f, l)] = max(e, ends.get((f, l), 0))
    except sqlite3.OperationalError:
        pass
    rows = []
    for s in sorted(syms.values()):
        s = list(s)
        s[10] = ends.get((s[7], s[8]), 0)
        rows.append(s)
    db.execute("DELETE FROM cd_symbols")
    db.execute("DELETE FROM cd_refs")
    db.execute("DELETE FROM cd_rel")
    db.executemany("INSERT INTO cd_symbols VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", rows)
    db.executemany("INSERT INTO cd_refs VALUES(?,?,?,?,?,?)", sorted(refs))
    db.executemany("INSERT INTO cd_rel VALUES(?,?,?)", sorted(rels))
    db.commit()
    log(f"merged: {len(rows)} symbols, {len(refs)} refs, {len(rels)} relations in {time.perf_counter() - t1:.1f}s; "
        f"total {time.perf_counter() - t0:.1f}s")
    return True


if __name__ == "__main__":
    a = sys.argv[1:]
    b = a[a.index("--build") + 1] if "--build" in a else os.path.join(ROOT, "build")
    j = int(a[a.index("--jobs") + 1]) if "--jobs" in a else min(8, os.cpu_count() or 1)
    run(os.path.abspath(b), j, "--full" in a)
