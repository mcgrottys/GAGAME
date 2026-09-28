"""How sparse are the trees that exist? A census of the tile cache, by realization and by mip.

Reads file NAMES only (f<face>_m<mip>_x<x>_y<y>[_key].<ext>); opens nothing, writes nothing.
For each realization of each tree it prints, per mip, the tiles on disk against the tiles a full
level would hold, and what kind they are: bytes (.bin), a reference to a child tree (.ref-*), a
void (.void), or a fold marker (.fold).

usage: python tools/hierarchy/tree_census.py [cache/trees] [name-prefix ...]
"""
import os
import re
import sys
from collections import defaultdict

NAME = re.compile(r"^f(\d+)_m(\d+)_x(\d+)_y(\d+)(?:_[0-9a-f]+)?\.(.+)$")


def full_level(real, fmt_w, fmt_h, mip):
    """Tiles a complete level holds: 16384 texels a side, per face."""
    faces = 6 if real.startswith("cube") else 1
    w = max(1, (16384 >> mip) // fmt_w)
    h = max(1, (16384 >> mip) // fmt_h)
    return faces * w * h


def census(root, prefixes):
    for tree in sorted(os.listdir(root)):
        if prefixes and not any(tree.startswith(p) for p in prefixes):
            continue
        tdir = os.path.join(root, tree)
        if not os.path.isdir(tdir):
            continue
        # a height tile is 256 x 128 texels, every other tile here 128 x 128
        tw, th = (256, 128) if tree.startswith(("earth.height", "noaa.", "swell.")) else (128, 128)
        for real in sorted(os.listdir(tdir)):
            rdir = os.path.join(tdir, real)
            if not os.path.isdir(rdir):
                continue
            by_mip = defaultdict(lambda: defaultdict(int))
            seen = defaultdict(set)
            size = 0
            with os.scandir(rdir) as it:
                for e in it:
                    m = NAME.match(e.name)
                    if not m:
                        continue
                    face, mip, x, y, ext = m.groups()
                    kind = "ref" if ext.startswith("ref") else ext
                    by_mip[int(mip)][kind] += 1
                    seen[int(mip)].add((face, x, y))
                    if kind == "bin":
                        size += e.stat().st_size
            if not by_mip:
                continue
            total = sum(len(s) for s in seen.values())
            print(f"{tree}/{real}: {total} addresses, {size / 2**20:.1f} MiB of bytes")
            for mip in sorted(by_mip):
                full = full_level(real, tw, th, mip)
                k = by_mip[mip]
                n = len(seen[mip])
                kinds = ", ".join(f"{k[q]} {q}" for q in ("bin", "ref", "void", "fold") if k[q])
                print(f"    mip {mip}: {n:>6} of {full:>6} ({100.0 * n / full:5.1f} %)   {kinds}")


if __name__ == "__main__":
    root = sys.argv[1] if len(sys.argv) > 1 else "cache/trees"
    census(root, sys.argv[2:])
