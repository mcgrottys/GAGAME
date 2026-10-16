# tools/astindex/verify_layouts.py -- the gate on the index's HLSL cbuffer offsets.
#
# astindex.py COMPUTES a cbuffer's offsets from the packing rules (16-byte rows, no straddling,
# arrays / structs / matrices start a row); nothing in the AST dump states them. This compiles
# every entry point the index found with dxc -Fc, reads the offsets the compiler itself printed in
# the listing's "Buffer Definitions", and compares them field by field with ast_fields. A cbuffer
# the optimizer dropped from an entry is simply absent from that listing; one entry that keeps it
# is enough. Run after an index; exit 1 on any disagreement.
#
#   py -3 tools/astindex/verify_layouts.py
import os
import re
import sqlite3
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
DB = os.path.join(ROOT, "tools", "Scriptorium", "scriptorium.db")
DXC = os.environ.get("GA_DXC", r"C:\Program Files (x86)\Windows Kits\10\bin\10.0.26100.0\x64\dxc.exe")
PROFILE = {"vs": "vs_6_5", "ps": "ps_6_5", "cs": "cs_6_5", "ms": "ms_6_5", "as": "as_6_5"}


def listing(job):
    f, entry, stage, defines = job
    if stage not in PROFILE:
        return f, entry, ""
    out = os.path.join(os.environ.get("TEMP", "."), f"astverify_{os.path.basename(f)}_{entry}.lst")
    args = [DXC, "-T", PROFILE[stage], "-E", entry, "-HV", "2021", "-I", "shaders", f, "-Fc", out]
    for d in filter(None, defines.split(",")):
        args += ["-D", d]
    p = subprocess.run(args, cwd=ROOT, capture_output=True, text=True)
    text = ""
    if p.returncode == 0 and os.path.exists(out):
        with open(out, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        os.remove(out)
    return f, entry, text


def parse(text):
    """{cbuffer: {field: offset}} from the listing's Buffer Definitions: the fields of a cbuffer's
    hostlayout struct at its first indentation (members of nested structs are deeper)."""
    out = {}
    cur, indent = None, None
    for line in text.splitlines():
        m = re.match(r"^; cbuffer (\w+)$", line)
        if m:
            cur, indent = m.group(1), None
            out[cur] = {}
            continue
        if cur is None:
            continue
        if line.startswith(";   }"):
            cur = None
            continue
        m = re.match(r"^;(\s+)(.*?)\s(\w+)(\[\d+\])?;\s+; Offset:\s+(\d+)", line)
        if m:
            if indent is None:
                indent = len(m.group(1))
            if len(m.group(1)) == indent:
                out[cur][m.group(3)] = int(m.group(5))
    return out


def main():
    db = sqlite3.connect(DB)
    entries = db.execute("SELECT file, entry, profile, defines FROM ast_hlsl_entries ORDER BY file, entry").fetchall()
    t0 = time.perf_counter()
    with ThreadPoolExecutor(max_workers=8) as ex:
        results = list(ex.map(listing, entries))
    seen = {}
    for f, entry, text in results:
        for cb, fields in parse(text).items():
            seen.setdefault(cb, (f, entry, fields))
    bad = checked = 0
    for cb, (f, entry, fields) in sorted(seen.items()):
        mine = dict(db.execute("SELECT name, offset FROM ast_fields WHERE type_usr = ? AND lang = 'hlsl-cbuffer'",
                               ("hlsl:cbuffer:" + cb,)).fetchall())
        for name, off in fields.items():
            checked += 1
            if mine.get(name) != off:
                bad += 1
                print(f"MISMATCH {cb}.{name}: dxc {off}, index {mine.get(name)}  ({f} {entry})")
    all_cbs = {r[0] for r in db.execute("SELECT name FROM ast_layouts WHERE lang = 'hlsl-cbuffer'")}
    print(f"{len(entries)} entries compiled in {time.perf_counter() - t0:.1f}s; {len(seen)} of {len(all_cbs)} cbuffers "
          f"seen in a listing, {checked} fields checked, {bad} mismatches")
    unseen = sorted(all_cbs - set(seen))
    if unseen:
        print("not in any listing (dropped by every entry, or its entry did not compile): " + ", ".join(unseen))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
