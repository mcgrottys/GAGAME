# ================================================================================================
#  tools/codemap/run.py -- the post-build pipeline, started detached by tools/astindex/launch.py:
#
#    1. tools/astindex/astindex.py   libclang + dxc -> ast_* (incremental: TUs whose reads changed)
#    2. clangd_index.py              clangd-indexer -> cd_* (incremental, the same TU keys)
#    3. build.py                     the map: build/codemap/ + map_fts (skipped when nothing it
#                                    reads changed: the two indexes, the scenes, the binary, itself)
#
#  One run at a time: a build that ends while a run is going leaves a mark, and the running one goes
#  round again when it ends, so the last build's sources are always the ones mapped.
#    py -3 tools/codemap/run.py <build dir> [--jobs N] [--full]
# ================================================================================================
import hashlib
import os
import sqlite3
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "tools", "astindex"))


def log(msg):
    print(f"[codemap] {time.strftime('%H:%M:%S')} {msg}", flush=True)


def map_key(build):
    """Everything build.py reads, as one hash."""
    h = hashlib.sha1()
    db = sqlite3.connect(os.path.join(ROOT, "tools", "Scriptorium", "scriptorium.db"), timeout=60)
    for q in ("SELECT tu, args_hash, deps FROM ast_tus ORDER BY tu", "SELECT value FROM ast_meta WHERE key='hlsl_stamps'",
              "SELECT tu, key FROM cd_tus ORDER BY tu"):
        try:
            for row in db.execute(q):
                h.update(repr(row).encode())
        except sqlite3.OperationalError:
            h.update(b"-")
    paths = [os.path.join(build, "bin", "gagame.exe")]
    for d in (os.path.join(ROOT, "scenes"), HERE, os.path.join(ROOT, "docs")):
        for dp, dn, fs in os.walk(d):
            dn.sort()
            paths += [os.path.join(dp, f) for f in sorted(fs) if f.endswith((".json", ".py", ".md"))]
    for p in paths:
        try:
            st = os.stat(p)
            h.update(f"{p}|{st.st_mtime_ns}|{st.st_size}".encode())
        except OSError:
            h.update(f"{p}|-".encode())
    return h.hexdigest()


def once(build, jobs, full):
    import astindex
    import clangd_index
    import build as mapbuild
    t0 = time.perf_counter()
    astindex.run(["--cc", os.path.join(build, "compile_commands.json"), "--jobs", str(jobs)] + (["--full"] if full else []))
    clangd_index.run(build, jobs, full)
    key = map_key(build)
    stamp = os.path.join(build, "codemap", ".key")
    old = open(stamp).read() if os.path.exists(stamp) else ""
    if key == old and not full:
        log(f"map: nothing changed ({time.perf_counter() - t0:.1f}s)")
        return
    mapbuild.main(["--out", os.path.join(build, "codemap")])
    os.makedirs(os.path.dirname(stamp), exist_ok=True)
    with open(stamp, "w") as fh:
        fh.write(key)
    log(f"done in {time.perf_counter() - t0:.1f}s")


def main(argv):
    build = os.path.abspath(argv[0] if argv and not argv[0].startswith("--") else os.path.join(ROOT, "build"))
    jobs = int(argv[argv.index("--jobs") + 1]) if "--jobs" in argv else min(8, os.cpu_count() or 1)
    full = "--full" in argv
    import msvcrt
    lock_path = os.path.join(build, "codemap.lock")
    mark = lock_path + ".rerun"
    fh = open(lock_path, "a+")
    try:
        msvcrt.locking(fh.fileno(), msvcrt.LK_NBLCK, 1)
    except OSError:
        open(mark, "w").close()
        log("another run holds the lock; it will go round again when it ends")
        return
    while True:
        if os.path.exists(mark):
            os.remove(mark)
        try:
            once(build, jobs, full)
        except Exception as ex:               # a failed map never fails anything else
            import traceback
            where = traceback.extract_tb(ex.__traceback__)[-1]
            log(f"FAILED: {type(ex).__name__}: {ex} at {os.path.basename(where.filename)}:{where.lineno} "
                f"({where.line})")
        if not os.path.exists(mark):
            break


if __name__ == "__main__":
    main(sys.argv[1:])
