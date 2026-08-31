# ==================================================================================================
#  snapshot.py - M8: dataset snapshots. "Separate files into folders so I can go back to a
#  dataset I viewed in the past" (the user, verbatim).
#
#  A snapshot captures the model-facing datasets -- the whole data/ tree (bathy planes, tide
#  fits, sea state, gis, the water registry, wave_scene.json) -- into a named folder under
#  D:\DataCache\GAGAME\snapshots\<name>\ with a manifest (utc, git rev, per-file FNV-1a).
#  cache/ is NOT snapshotted: composed tiles and wave solves carry content identity in their
#  filenames and regenerate from data/ -- the snapshot IS the identity.
#
#    py tools/snapshot.py save <name> [note...]
#    py tools/snapshot.py list
#    py tools/snapshot.py restore <name>     (auto-saves the current tree first)
#    py tools/snapshot.py diff <name>        (what changed since the snapshot)
# ==================================================================================================
import json
import os
import shutil
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(REPO, "data")
SNAPS = r"D:\DataCache\GAGAME\snapshots"
if not os.path.isdir(r"D:\DataCache\GAGAME"):
    SNAPS = os.path.join(REPO, "snapshots")   # fallback if the big drive is absent


def fnv1a(path):
    h = 0xcbf29ce484222325
    with open(path, "rb") as f:
        while True:
            b = f.read(1 << 20)
            if not b:
                break
            for byte in b:
                h = ((h ^ byte) * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return "%016x" % h


def fnv1a_fast(path):
    # big rasters hash their size + first/last MB (a full byte-hash of 250 MB planes is
    # minutes in python; size+ends catches every real regeneration)
    sz = os.path.getsize(path)
    if sz < (1 << 21):
        return fnv1a(path)
    h = 0xcbf29ce484222325
    with open(path, "rb") as f:
        head = f.read(1 << 20)
        f.seek(-(1 << 20), 2)
        tail = f.read(1 << 20)
    for b in (sz.to_bytes(8, "little") + head + tail):
        h = ((h ^ b) * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return "%016x*" % h


def manifest_of(root):
    out = {}
    for dirpath, _, files in os.walk(root):
        for fn in files:
            p = os.path.join(dirpath, fn)
            rel = os.path.relpath(p, root).replace("\\", "/")
            out[rel] = fnv1a_fast(p)
    return out


def git_rev():
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"],
                                       cwd=REPO, text=True).strip()
    except Exception:                                      # noqa: BLE001
        return "?"


def save(name, note):
    dst = os.path.join(SNAPS, name)
    if os.path.exists(dst):
        print("snapshot '%s' already exists -- pick a new name" % name)
        return 1
    os.makedirs(SNAPS, exist_ok=True)
    print("snapshotting data/ -> %s ..." % dst)
    shutil.copytree(DATA, os.path.join(dst, "data"))
    man = {
        "name": name,
        "note": note,
        "saved_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "git_rev": git_rev(),
        "files": manifest_of(os.path.join(dst, "data")),
    }
    json.dump(man, open(os.path.join(dst, "manifest.json"), "w"), indent=1)
    total = sum(os.path.getsize(os.path.join(dp, f))
                for dp, _, fs in os.walk(dst) for f in fs)
    print("saved '%s': %d files, %.1f MB (git %s)" % (
        name, len(man["files"]), total / 1e6, man["git_rev"]))
    return 0


def list_snaps():
    if not os.path.isdir(SNAPS):
        print("no snapshots yet (%s)" % SNAPS)
        return 0
    for name in sorted(os.listdir(SNAPS)):
        mp = os.path.join(SNAPS, name, "manifest.json")
        if not os.path.isfile(mp):
            continue
        m = json.load(open(mp))
        print("%-24s %s  git %s  %d files  %s" % (
            name, m.get("saved_utc", "?"), m.get("git_rev", "?"),
            len(m.get("files", {})), m.get("note", "")))
    return 0


def restore(name):
    src = os.path.join(SNAPS, name, "data")
    if not os.path.isdir(src):
        print("no snapshot '%s'" % name)
        return 1
    auto = "_before_restore_" + time.strftime("%Y%m%d_%H%M%S")
    print("auto-saving the current tree as '%s' first..." % auto)
    save(auto, "automatic safety snapshot before restoring '%s'" % name)
    print("restoring '%s' -> data/ ..." % name)
    shutil.rmtree(DATA)
    shutil.copytree(src, DATA)
    print("restored. cache/ entries keyed on the old content revive as cache hits;")
    print("anything new repaints on demand (the soak rule).")
    return 0


def diff(name):
    mp = os.path.join(SNAPS, name, "manifest.json")
    if not os.path.isfile(mp):
        print("no snapshot '%s'" % name)
        return 1
    old = json.load(open(mp))["files"]
    cur = manifest_of(DATA)
    for rel in sorted(set(old) | set(cur)):
        a, b = old.get(rel), cur.get(rel)
        if a == b:
            continue
        tag = "changed" if (a and b) else ("removed" if a else "added")
        print("%-8s %s" % (tag, rel))
    return 0


def main():
    if len(sys.argv) < 2:
        print(__doc__ or "save <name> | list | restore <name> | diff <name>")
        return 1
    cmd = sys.argv[1]
    if cmd == "save" and len(sys.argv) >= 3:
        return save(sys.argv[2], " ".join(sys.argv[3:]))
    if cmd == "list":
        return list_snaps()
    if cmd == "restore" and len(sys.argv) >= 3:
        return restore(sys.argv[2])
    if cmd == "diff" and len(sys.argv) >= 3:
        return diff(sys.argv[2])
    print("usage: py tools/snapshot.py save <name> [note] | list | restore <name> | diff <name>")
    return 1


if __name__ == "__main__":
    sys.exit(main())
