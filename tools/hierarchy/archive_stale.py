#!/usr/bin/env python3
"""Review finding 25, measured: does a tile tree's archive still hold what its loose files hold?

TileTree reads a packed archive BEFORE the loose file of the same address (TileTree.h
FromArchive, called first by LeafTile and Serve), while folds and drops read, write and delete
loose files only. So a tile folded or repainted after a pack is served from the archive as it
was when it was packed. This script opens every `cache/trees/<node>.<id>/<tag>.gaa`, and for
every record compares the archive's bytes with the loose file the record was packed from.

It reads and compares; it writes nothing and deletes nothing. Pure Python.

The archive's layout is TileArchive.h's: "GAAR", version, count, reserved; then count records of
{key64, offset64, size32, subset32}; then the payloads. key = face << 61 | mip << 56 | y << 28 | x.
"""
import os
import struct
import sys
import time

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.join("cache", "trees")


def audit(arc_path):
    folder = arc_path[:-4]                       # <tag>.gaa beside the folder <tag>
    out = {"records": 0, "same": 0, "differ": 0, "loose_missing": 0, "short": 0,
           "loose_newer": 0, "worst": 0, "first": None}
    arc_mtime = os.path.getmtime(arc_path)
    with open(arc_path, "rb") as f:
        hdr = f.read(16)
        if len(hdr) < 16:
            return None
        magic, version, n, _ = struct.unpack("<4I", hdr)
        if magic != 0x52414147:
            return None
        recs = [struct.unpack("<QQII", f.read(24)) for _ in range(n)]
        out["records"] = n
        out["version"] = version
        for key, offset, size, subset in recs:
            face, mip = key >> 61, (key >> 56) & 31
            y, x = (key >> 28) & 0xFFFFFFF, key & 0xFFFFFFF
            base = f"f{face}_m{mip}_x{x}_y{y}"
            name = base + (f"_{subset:08x}.bin" if subset else ".bin")
            loose = os.path.join(folder, name)
            if not os.path.exists(loose):
                out["loose_missing"] += 1
                continue
            if os.path.getmtime(loose) > arc_mtime:
                out["loose_newer"] += 1
            f.seek(offset)
            a = f.read(size)
            with open(loose, "rb") as g:
                b = g.read()
            if len(a) != size or len(b) != size:
                out["short"] += 1
                continue
            if a == b:
                out["same"] += 1
                continue
            out["differ"] += 1
            d = max(abs(p - q) for p, q in zip(a, b))
            out["worst"] = max(out["worst"], d)
            if out["first"] is None:
                out["first"] = (name, sum(1 for p, q in zip(a, b) if p != q), d)
    out["packed"] = time.strftime("%Y-%m-%d %H:%M", time.localtime(arc_mtime))
    return out


def main():
    total = {"records": 0, "differ": 0, "loose_missing": 0}
    print(f"{'archive':<62} {'packed':<17} {'records':>8} {'same':>7} {'DIFFER':>7} "
          f"{'no loose':>9} {'newer':>6} {'worst byte':>11}")
    for node in sorted(os.listdir(ROOT)):
        d = os.path.join(ROOT, node)
        if not os.path.isdir(d):
            continue
        for name in sorted(os.listdir(d)):
            if not name.endswith(".gaa"):
                continue
            r = audit(os.path.join(d, name))
            if r is None:
                print(f"{node + '/' + name:<62} unreadable")
                continue
            print(f"{node + '/' + name:<62} {r['packed']:<17} {r['records']:>8} {r['same']:>7} "
                  f"{r['differ']:>7} {r['loose_missing']:>9} {r['loose_newer']:>6} "
                  f"{r['worst']:>11}")
            if r["first"]:
                print(f"    first: {r['first'][0]}  {r['first'][1]} of its bytes differ, "
                      f"the largest by {r['first'][2]}")
            for k in total:
                total[k] += r[k]
    print(f"\n{total['records']} archived tiles; {total['differ']} hold bytes their loose file "
          f"no longer holds; {total['loose_missing']} have no loose file any more.")
    print("'newer' counts loose files written after the pack; 'DIFFER' is the bytes themselves.")


if __name__ == "__main__":
    main()
