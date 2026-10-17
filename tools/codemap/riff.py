# ================================================================================================
#  tools/codemap/riff.py -- clangd's binary index (RIFF 'CdIx', format version 20) read in Python.
#
#  clangd-indexer --format=binary writes what clangd itself loads: a RIFF file whose chunks are
#    meta  uint32 version
#    stri  uint32 raw size (0 = not compressed), then the strings, NUL-terminated, in index order
#    symb  per symbol: 8-byte id, kind, lang, then varints into the string table (layout below)
#    refs  per symbol: 8-byte id, varint count, per ref: kind byte, location, 8-byte container id
#    rela  12+? bytes: subject id, predicate byte (0 BaseOf, 1 OverriddenBy), object id
#  (clang-tools-extra/clangd/index/Serialization.cpp). The same file read as YAML is 15x larger
#  (137 MB vs 9 MB for one TU), which is why this reader exists; it is checked against the YAML
#  of the same TU by `py -3 tools/codemap/riff.py <x.idx> <x.yaml>`.
# ================================================================================================
import struct
import zlib

VERSION = 20

# clang::index::SymbolKind as clangd 23 numbers it: one more kind than clang 18 sits at 5, before
# Enum. Measured against the YAML of one TU, every kind that occurs: 2 Namespace, 6 Enum, 7 Struct,
# 8 Class, 11 Union, 12 TypeAlias, 13 Function, 14 Variable, 15 Field, 16 EnumConstant,
# 17 InstanceMethod, 19 StaticMethod, 22 StaticProperty, 23 Constructor, 24 Destructor,
# 25 ConversionFunction, 27 Using, 31 Concept.
KINDS = ["Unknown", "Module", "Namespace", "NamespaceAlias", "Macro", "?5", "Enum", "Struct", "Class",
         "Protocol", "Extension", "Union", "TypeAlias", "Function", "Variable", "Field",
         "EnumConstant", "InstanceMethod", "ClassMethod", "StaticMethod", "InstanceProperty",
         "ClassProperty", "StaticProperty", "Constructor", "Destructor", "ConversionFunction",
         "Parameter", "Using", "TemplateTypeParm", "TemplateTemplateParm", "NonTypeTemplateParm",
         "Concept"]

# clangd::RefKind bits
REF_DECL, REF_DEF, REF_REF, REF_SPELLED, REF_CALL = 1, 2, 4, 8, 16


class _Reader:
    __slots__ = ("b", "i")

    def __init__(self, b, i=0):
        self.b, self.i = b, i

    def var(self):
        # LLVM's writeVar: 7 bits a byte, low first, high bit = more.
        b, i, shift, v = self.b, self.i, 0, 0
        while True:
            x = b[i]
            i += 1
            v |= (x & 0x7F) << shift
            if x < 0x80:
                break
            shift += 7
        self.i = i
        return v

    def byte(self):
        x = self.b[self.i]
        self.i += 1
        return x

    def id(self):
        x = self.b[self.i:self.i + 8]
        self.i += 8
        return x.hex().upper()

    def done(self):
        return self.i >= len(self.b)


def chunks(data):
    if data[:4] != b"RIFF" or data[8:12] != b"CdIx":
        raise ValueError("not a clangd index (RIFF CdIx)")
    out, i = {}, 12
    while i + 8 <= len(data):
        cid = data[i:i + 4].decode("ascii")
        n = struct.unpack_from("<I", data, i + 4)[0]
        out[cid] = data[i + 8:i + 8 + n]
        i += 8 + n + (n & 1)
    return out


def strings(raw):
    size = struct.unpack_from("<I", raw, 0)[0]
    body = raw[4:] if size == 0 else zlib.decompress(raw[4:])
    return [s.decode("utf-8", "replace") for s in body.split(b"\0")[:-1]]


def read(data):
    """The whole index: {'strings', 'symbols': [dict], 'refs': {id: [(kind, loc, container)]},
    'relations': [(subject, predicate, object)]}. A location is (file_uri, l0, c0, l1, c1), lines
    and columns 0-based as clangd stores them."""
    ch = chunks(data)
    ver = struct.unpack_from("<I", ch["meta"], 0)[0]
    if ver != VERSION:
        raise ValueError(f"clangd index format {ver}; this reader knows {VERSION}")
    S = strings(ch["stri"])

    def loc(r):
        uri = S[r.var()]
        return (uri, r.var(), r.var(), r.var(), r.var())

    symbols = []
    r = _Reader(ch.get("symb", b""))
    while not r.done():
        s = {"id": r.id(), "kind": KINDS[k] if (k := r.byte()) < len(KINDS) else f"?{k}", "lang": r.byte(),
             "name": S[r.var()], "scope": S[r.var()], "tsargs": S[r.var()]}
        s["def"] = loc(r)
        s["decl"] = loc(r)
        s["nrefs"] = r.var()
        s["flags"] = r.byte()
        s["signature"] = S[r.var()]
        s["snippet"] = S[r.var()]
        s["doc"] = S[r.var()]
        s["ret"] = S[r.var()]
        s["type"] = S[r.var()]
        for _ in range(r.var()):            # include headers: header string, (refs << 2) | directives
            r.var()
            r.var()
        symbols.append(s)

    refs = {}
    r = _Reader(ch.get("refs", b""))
    while not r.done():
        sid = r.id()
        lst = refs.setdefault(sid, [])
        for _ in range(r.var()):
            kind = r.byte()
            lst.append((kind, loc(r), r.id()))

    rels = []
    r = _Reader(ch.get("rela", b""))
    while not r.done():
        rels.append((r.id(), r.byte(), r.id()))
    return {"strings": S, "symbols": symbols, "refs": refs, "relations": rels}


def _check(idx_path, yaml_path):
    """Compare this reader with the YAML of the same TU: every symbol's name / scope / kind /
    definition line, and every ref's kind / line / container, must agree."""
    import re
    with open(idx_path, "rb") as fh:
        ix = read(fh.read())
    ysyms, yrefs, cur, mode, ref = {}, {}, None, None, None
    with open(yaml_path, encoding="utf-8") as fh:
        for line in fh:
            if line.startswith("--- !"):
                mode, cur = line[5:].strip(), {}
                continue
            if mode == "Symbol":
                m = re.match(r"^(ID|Name|Scope):\s+'?(.*?)'?$", line.rstrip("\n"))
                if m:
                    cur[m.group(1)] = m.group(2)
                m = re.match(r"^  Kind:\s+(\w+)", line)
                if m:
                    cur["Kind"] = m.group(1)
                if line.startswith("..."):
                    ysyms[cur["ID"]] = cur
            elif mode == "Refs":
                if line.startswith("ID:"):
                    cur["ID"] = line.split()[1].strip("'")   # quoted when it reads as a number (97E95...)
                    yrefs.setdefault(cur["ID"], [])   # one symbol's refs may span several documents
                elif line.startswith("  - Kind:"):
                    ref = [int(line.split()[-1])]
                    yrefs[cur["ID"]].append(ref)
                elif line.startswith("        Line:") and len(ref) == 1:
                    ref.append(int(line.split()[-1]))
                elif line.startswith("      ID:"):
                    ref.append(line.split()[-1].strip("'"))
    bad = 0
    for s in ix["symbols"]:
        y = ysyms.get(s["id"])
        # The YAML writer prints a Concept's kind empty ("Kind: Lang: C"), so a concept is checked by name.
        if y is None or y.get("Name", "") != s["name"] or y.get("Scope", "") != s["scope"] or (
                y.get("Kind") != s["kind"] and s["kind"] != "Concept"):
            bad += 1
            if bad < 5:
                print("symbol differs:", s["id"], s["name"], s["kind"], "| yaml:", y)
    nref = 0
    for sid, lst in ix["refs"].items():
        ylst = yrefs.get(sid, [])
        mine = sorted((k, l[1], c) for k, l, c in lst)
        theirs = sorted((y[0], y[1], y[2]) for y in ylst)
        nref += len(mine)
        if mine != theirs:
            bad += 1
            if bad < 10:
                print("refs differ:", sid, mine[:3], "| yaml:", theirs[:3])
    print(f"{len(ix['symbols'])} symbols (yaml {len(ysyms)}), {nref} refs in {len(ix['refs'])} lists "
          f"(yaml {sum(len(v) for v in yrefs.values())}), {len(ix['relations'])} relations; {bad} differences")
    return bad


if __name__ == "__main__":
    import sys
    sys.exit(1 if _check(sys.argv[1], sys.argv[2]) else 0)
