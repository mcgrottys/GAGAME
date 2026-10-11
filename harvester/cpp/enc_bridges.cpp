// ================================================================================================
//  enc_bridges.cpp - NOAA ENC (IHO S-57 in ISO 8211) into GABRDG01 (src/compose/RoadWays.h defines the
//  bytes): every bridge, overhead cable and pipeline, tunnel, pylon / pier, conveyor, gate, dam and
//  causeway of every base cell, with its clearances and its real geometry.
//
//  The reader is written from the specifications (ISO/IEC 8211, IHO S-57 edition 3.1), no GDAL:
//    LOAD      the DDR (the first record) describes every field: its subfield names and its format
//              controls, "(b11,b14,2b11)", "(B(40),3b11)", "(b12,A)" ... The data records are decoded
//              by those descriptions, not by a hardcoded layout; binary subfields are little-endian.
//              A base cell (.000) is read whole: vectors (VRID / VRPT / SG2D) and features (FRID /
//              FOID / ATTF / FSPT). Update files (.001 ...) are NOT applied this time: counted per cell
//              and declared in the manifest.
//    NORMALIZE coordinates are kept as the file's integers: NOAA's COMF is 10,000,000, so YCOO / XCOO
//              ARE int32 1e-7 degree exactly. A cell with another COMF is REFUSED by name, never
//              rounded; so is a cell whose DSPM HUNI is not metres (1), and a feature whose own HUNITS
//              is not metres keeps its geometry but REFUSES its lengths (counted). A clearance that is
//              not a number is refused by attribute name; an empty value is "unknown" (NaN).
//              VERDAT: the feature's own if present, else the dataset's DSPM VDAT (S-57: the feature's
//              overrides); the manifest counts which.
//    GEOMETRY  a point -> its VI / VC node; a line or area -> its FSPT edges in order, each edge
//              begin node + SG2D run + end node (VRPT TOPI 1 / 2), reversed when ORNT = 2. Consecutive
//              edges chain where they meet (within 1 unit); a line that breaks starts a new part (the
//              break counted); an area's rings close on themselves (USAG 2 = inner, 1 / 3 outer), the
//              outer rings first, a ring that does not close counted. Rings keep their closing vertex.
//    OUTPUT    records sorted by (iy, ix, id, origin) of the 0.05-degree cell of the first point, the
//              "GABRDG01" magic, the binary cell index sidecar (int32 ix, int32 iy, int64 offset,
//              int64 count, by (iy, ix)) and the .json manifest, as harvester/cpp/planet_buildings.
//              Overlapping cells of different usage bands DUPLICATE a feature: counted per band, not
//              deduplicated (origin = the cell name, its third character the band).
//    MEMBERS   VERCLR / VERCCL / VERCOP / VERCSA / HORCLR / HEIGHT to their floats, OBJNAM the name, INFORM
//              the note, CATBRG (a pylon: CATPYL) the category; the NBI-only bytes are 255 (blank).
//
//  Usage: enc_bridges <ENC_ROOT> <out dir> [threads]       the whole root -> enc-20261009.bridges.*
//         enc_bridges --dump <cell.000>                     the DDR and every wanted feature
//  Licence of the data: NOAA ENC may be used for any purpose (15 CFR 995), not redistributed as
//  provided; (c) NOAA Office of Coast Survey.
// ================================================================================================
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr double kCellDeg = 0.05;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr const char* kStem = "enc-20261009";   // the set was built 2026-10-09
constexpr int64_t kComf = 10000000;

auto t0 = std::chrono::steady_clock::now();
double Secs() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }

// ---- the S-57 codes (IHO object / attribute catalogue; GDAL's s57objectclasses / s57attributes) --
struct Wanted {
    uint16_t objl;
    const char* acronym;
    uint8_t kind;   // BridgeKind
};
const Wanted kWanted[] = {{11, "BRIDGE", 0}, {21, "CBLOHD", 1}, {93, "PIPOHD", 2}, {151, "TUNNEL", 3}, {98, "PYLONS", 4},
                          {34, "CONVYR", 5}, {61, "GATCON", 6}, {38, "DAMCON", 7}, {26, "CAUSWY", 8}};
const char* const kKindNames[] = {"Bridge", "OverheadCable", "OverheadPipeline", "Tunnel", "Pylon",
                                  "Conveyor", "Gate", "Dam", "Causeway"};
int KindOfObjl(uint16_t objl) {
    for (const Wanted& w : kWanted)
        if (w.objl == objl) return w.kind;
    return -1;
}
enum Attl : uint16_t {
    CATBRG = 9, CATPYL = 49, HEIGHT = 95, HUNITS = 96, HORCLR = 98, INFORM = 102, OBJNAM = 116, SCAMIN = 133, SORIND = 148,
    VERCLR = 181, VERCCL = 182, VERCOP = 183, VERCSA = 184, VERDAT = 185, VERLEN = 186
};
const std::map<uint16_t, const char*> kAttlNames = {
    {5, "BURDEP"}, {9, "CATBRG"}, {11, "CATCBL"}, {17, "CATCON"}, {20, "CATDAM"}, {29, "CATGAT"}, {47, "CATPIP"},
    {49, "CATPYL"}, {75, "COLOUR"}, {76, "COLPAT"}, {81, "CONDTN"}, {82, "CONRAD"}, {83, "CONVIS"}, {85, "DATEND"},
    {86, "DATSTA"}, {87, "DRVAL1"}, {90, "ELEVAT"}, {95, "HEIGHT"}, {96, "HUNITS"}, {97, "HORACC"}, {98, "HORCLR"},
    {99, "HORLEN"}, {100, "HORWID"}, {101, "ICEFAC"}, {102, "INFORM"}, {106, "LIFCAP"}, {112, "NATCON"},
    {114, "NATQUA"}, {116, "OBJNAM"}, {120, "PICREP"}, {123, "PRODCT"}, {125, "QUASOU"}, {128, "RECDAT"},
    {129, "RECIND"}, {132, "SCAMAX"}, {133, "SCAMIN"}, {147, "SORDAT"}, {148, "SORIND"}, {149, "STATUS"},
    {158, "TXTDSC"}, {180, "VERACC"}, {181, "VERCLR"}, {182, "VERCCL"}, {183, "VERCOP"}, {184, "VERCSA"},
    {185, "VERDAT"}, {186, "VERLEN"}, {187, "WATLEV"}, {301, "NOBJNM"}, {304, "NTXTDS"}};
std::string AttlName(uint16_t c) {
    auto it = kAttlNames.find(c);
    return it != kAttlNames.end() ? it->second : "#" + std::to_string(c);
}
const char* const kVerdat[] = {"",
                               "Mean low water springs", "Mean lower low water springs", "Mean sea level",
                               "Lowest low water", "Mean low water", "Lowest low water springs",
                               "Approximate mean low water springs", "Indian spring low water", "Low water springs",
                               "Approximate lowest astronomical tide", "Nearly lowest low water", "Mean lower low water",
                               "Low water", "Approximate mean low water", "Approximate mean lower low water",
                               "Mean high water", "Mean high water springs", "High water",
                               "Approximate mean sea level", "High water springs", "Mean higher high water",
                               "Equinoctial spring low water", "Lowest astronomical tide", "Local datum",
                               "International Great Lakes Datum 1985", "Mean water level",
                               "Lower low water large tide", "Higher high water large tide", "Nearly highest high water"};

// ---- ISO 8211 --------------------------------------------------------------------------------
constexpr char kFT = 0x1E, kUT = 0x1F;

struct Fmt {
    char t = 'A';   // A I R B b
    int w = 0;      // bytes; 0 = variable, ended by the unit terminator
    int bt = 0;     // b: 1 unsigned, 2 signed, 3..5 float
};
struct FieldDef {
    std::string tag, name, arr, fmtText;
    bool repeat = false;
    std::vector<std::string> subs;
    std::vector<Fmt> fmts;
    int Index(const char* s) const {
        for (size_t i = 0; i < subs.size(); ++i)
            if (subs[i] == s) return static_cast<int>(i);
        return -1;
    }
};
struct Val {
    int64_t i = 0;
    std::string_view s;
};
struct Field {
    std::string_view tag, data;
    const FieldDef* def = nullptr;
};

int64_t Digits(std::string_view s) {
    int64_t v = 0;
    for (char c : s) {
        if (c == ' ') continue;
        if (c < '0' || c > '9') return -1;
        v = v * 10 + (c - '0');
    }
    return v;
}
uint32_t TagKey(std::string_view t) {
    uint32_t k = 0;
    for (size_t i = 0; i < 4 && i < t.size(); ++i) k |= uint32_t(uint8_t(t[i])) << (8 * i);
    return k;
}

// "(b11,b14,2b11,3A,2A(8),R(4))", "(B(40),3b11)", nested groups with repeat counts.
bool ParseFmtList(std::string_view s, size_t& i, std::vector<Fmt>& out, std::string& why) {
    while (i < s.size()) {
        const char c = s[i];
        if (c == ')') { ++i; return true; }
        if (c == ',' || c == ' ') { ++i; continue; }
        int count = 0;
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) count = count * 10 + (s[i++] - '0');
        if (count == 0) count = 1;
        if (i >= s.size()) break;
        if (s[i] == '(') {
            ++i;
            std::vector<Fmt> g;
            if (!ParseFmtList(s, i, g, why)) return false;
            for (int k = 0; k < count; ++k) out.insert(out.end(), g.begin(), g.end());
            continue;
        }
        Fmt f;
        f.t = s[i++];
        if (f.t == 'b') {
            if (i + 2 > s.size()) { why = "format b without type and width"; return false; }
            f.bt = s[i++] - '0';
            f.w = s[i++] - '0';
            if (f.bt < 1 || f.bt > 5 || (f.w != 1 && f.w != 2 && f.w != 4 && f.w != 8)) { why = "format b" + std::to_string(f.bt) + std::to_string(f.w); return false; }
        } else if (f.t == 'A' || f.t == 'I' || f.t == 'R' || f.t == 'B') {
            if (i < s.size() && s[i] == '(') {
                ++i;
                int n = 0;
                while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) n = n * 10 + (s[i++] - '0');
                if (i >= s.size() || s[i] != ')') { ++i; why = "format width"; return false; }
                ++i;
                if (f.t == 'B') {
                    if (n % 8) { why = "format B(" + std::to_string(n) + ") not whole bytes"; return false; }
                    n /= 8;
                }
                f.w = n;
            } else if (f.t == 'B') {
                why = "format B without width";
                return false;
            }
        } else {
            why = std::string("format control '") + f.t + "' not supported";
            return false;
        }
        for (int k = 0; k < count; ++k) out.push_back(f);
    }
    why = "format controls not closed";
    return false;
}

class Ddf {
public:
    std::vector<FieldDef> defs;
    std::unordered_map<uint32_t, size_t> byTag;
    std::string why;

    bool Open(std::string_view file) {
        m_file = file;
        size_t base, recLen;
        std::vector<std::array<size_t, 3>> dir;   // tag offset (relative to record), length, position
        std::vector<std::string_view> tags;
        if (!Record(0, recLen, base, tags, dir, true)) return false;
        const int fcl = static_cast<int>(Digits(file.substr(10, 2)));
        if (file[6] != 'L') { why = "first record is not a DDR (leader id " + std::string(1, file[6]) + ")"; return false; }
        for (size_t k = 0; k < dir.size(); ++k) {
            const std::string_view tag = tags[k];
            std::string_view d = file.substr(base + dir[k][2], dir[k][1]);
            FieldDef f;
            f.tag = std::string(tag);
            if (tag == "0000") continue;   // the file control field
            if (d.size() < static_cast<size_t>(fcl)) { why = "DDR field " + f.tag + " short"; return false; }
            const char structure = d[0];
            d.remove_prefix(fcl);
            auto upto = [&](std::string_view& v) {
                size_t e = v.find_first_of(std::string_view("\x1f\x1e", 2));
                std::string_view r = v.substr(0, e);
                v.remove_prefix(e == std::string_view::npos ? v.size() : e + 1);
                return r;
            };
            f.name = std::string(upto(d));
            f.arr = std::string(upto(d));
            f.fmtText = std::string(upto(d));
            std::string_view a = f.arr;
            if (!a.empty() && a[0] == '*') { f.repeat = true; a.remove_prefix(1); }
            while (!a.empty()) {
                size_t e = a.find('!');
                f.subs.emplace_back(a.substr(0, e));
                a.remove_prefix(e == std::string_view::npos ? a.size() : e + 1);
            }
            const size_t open = f.fmtText.find('(');
            if (open != std::string::npos) {
                size_t i = open + 1;
                if (!ParseFmtList(f.fmtText, i, f.fmts, why)) { why = "DDR field " + f.tag + ": " + why; return false; }
            }
            if (f.subs.empty() && structure == '0' && f.fmts.size() <= 1) f.subs.push_back("");   // elementary
            if (f.fmts.empty() && !f.subs.empty()) f.fmts.assign(f.subs.size(), Fmt{});   // no formats: all variable A
            if (f.fmts.size() != f.subs.size()) {
                why = "DDR field " + f.tag + ": " + std::to_string(f.subs.size()) + " subfields, " +
                      std::to_string(f.fmts.size()) + " formats";
                return false;
            }
            byTag[TagKey(tag)] = defs.size();
            defs.push_back(std::move(f));
        }
        m_pos = recLen;
        return true;
    }
    // The next data record's fields; false at the end of the file or on a fault (why set).
    bool Next(std::vector<Field>& out) {
        out.clear();
        if (m_pos >= m_file.size()) return false;
        size_t base, recLen;
        std::vector<std::array<size_t, 3>> dir;
        std::vector<std::string_view> tags;
        if (!Record(m_pos, recLen, base, tags, dir, false)) return false;
        for (size_t k = 0; k < dir.size(); ++k) {
            Field f;
            f.tag = tags[k];
            f.data = m_file.substr(m_pos + base + dir[k][2], dir[k][1]);
            auto it = byTag.find(TagKey(f.tag));
            f.def = it == byTag.end() ? nullptr : &defs[it->second];
            out.push_back(f);
        }
        m_pos += recLen;
        return true;
    }

private:
    std::string_view m_file;
    size_t m_pos = 0;

    bool Record(size_t at, size_t& recLen, size_t& base, std::vector<std::string_view>& tags,
                std::vector<std::array<size_t, 3>>& dir, bool ddr) {
        if (at + 24 > m_file.size()) { why = "truncated leader at " + std::to_string(at); return false; }
        const std::string_view L = m_file.substr(at, 24);
        const int64_t len = Digits(L.substr(0, 5));
        const int64_t b = Digits(L.substr(12, 5));
        const int sl = L[20] - '0', sp = L[21] - '0', st = L[23] - '0';
        if (b < 24 || sl < 1 || sp < 1 || st < 1 || sl > 9 || sp > 9 || st > 9) { why = "bad leader at " + std::to_string(at); return false; }
        if (!ddr && L[6] != 'D') { why = std::string("leader id '") + L[6] + "' (only 'D' records are read)"; return false; }
        base = static_cast<size_t>(b);
        const size_t es = sl + sp + st;
        size_t p = at + 24, end = 0;
        while (p < m_file.size() && m_file[p] != kFT) {
            if (p + es > m_file.size()) { why = "truncated directory"; return false; }
            const std::string_view e = m_file.substr(p, es);
            const int64_t fl = Digits(e.substr(st, sl)), fp = Digits(e.substr(st + sl, sp));
            if (fl < 0 || fp < 0) { why = "bad directory entry at " + std::to_string(p); return false; }
            tags.push_back(e.substr(0, st));
            dir.push_back({0, static_cast<size_t>(fl), static_cast<size_t>(fp)});
            end = std::max(end, static_cast<size_t>(fp + fl));
            p += es;
        }
        // A record longer than 99,999 bytes cannot state its length in five digits: the directory does.
        recLen = len > 0 ? static_cast<size_t>(len) : base + end;
        if (base + end > recLen) recLen = base + end;
        if (at + recLen > m_file.size()) { why = "truncated record at " + std::to_string(at); return false; }
        return true;
    }
};

// A field's subfields by the DDR, flattened row after row (a repeating field: many rows).
bool Decode(const Field& f, std::vector<Val>& out) {
    out.clear();
    if (!f.def) return false;
    const FieldDef& d = *f.def;
    const char* p = f.data.data();
    const char* e = p + f.data.size();
    if (e > p && e[-1] == kFT) --e;
    const size_t ns = d.fmts.size();
    if (ns == 0) return false;
    do {
        for (size_t k = 0; k < ns; ++k) {
            const Fmt& m = d.fmts[k];
            Val v;
            if (m.w == 0) {
                const char* q = p;
                while (q < e && *q != kUT) ++q;
                v.s = std::string_view(p, q - p);
                p = q < e ? q + 1 : q;
                if (m.t == 'I') v.i = std::strtoll(std::string(v.s).c_str(), nullptr, 10);
            } else {
                if (p + m.w > e) return false;
                if (m.t == 'b') {
                    uint64_t u = 0;
                    for (int b = 0; b < m.w; ++b) u |= uint64_t(uint8_t(p[b])) << (8 * b);
                    if (m.bt == 2 && m.w < 8 && (u >> (8 * m.w - 1)) & 1) u |= ~uint64_t(0) << (8 * m.w);
                    v.i = static_cast<int64_t>(u);
                } else {
                    v.s = std::string_view(p, m.w);
                    if (m.t == 'I') v.i = std::strtoll(std::string(v.s).c_str(), nullptr, 10);
                }
                p += m.w;
            }
            out.push_back(v);
        }
    } while (d.repeat && p < e);
    return true;
}
uint64_t NameKey(std::string_view b40) {   // RCNM (1 byte) + RCID (uint32 LE)
    uint32_t id = 0;
    for (int k = 0; k < 4; ++k) id |= uint32_t(uint8_t(b40[1 + k])) << (8 * k);
    return (uint64_t(uint8_t(b40[0])) << 32) | id;
}

std::string Latin1ToUtf8(std::string_view s) {   // ATTF is lexical level 0 or 1 (ISO 8859-1)
    std::string o;
    for (unsigned char c : s) {
        if (c < 0x80) o += static_cast<char>(c);
        else {
            o += static_cast<char>(0xC0 | (c >> 6));
            o += static_cast<char>(0x80 | (c & 0x3F));
        }
    }
    return o;
}
std::string Json(const std::string& s) {
    std::string o = "\"";
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += static_cast<char>(c); }
        else if (c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); o += b; }
        else o += static_cast<char>(c);
    }
    return o + "\"";
}

// ---- one cell -----------------------------------------------------------------------------------
struct XY {
    int32_t x, y;
    bool Near(const XY& o) const { return std::abs(int64_t(x) - o.x) <= 1 && std::abs(int64_t(y) - o.y) <= 1; }
    bool operator==(const XY& o) const { return x == o.x && y == o.y; }
};
struct Vec {
    std::vector<XY> pts;   // SG2D, as the file orders it
    uint64_t beg = 0, end = 0;
};
struct Sp {
    uint64_t name;
    int ornt, usag, mask;
};
struct Feat {
    int64_t id = 0;
    uint32_t agen = 0, fidn = 0, fids = 0;
    int prim = 0, grup = 0;
    uint16_t objl = 0;
    std::vector<std::pair<uint16_t, std::string>> attrs;
    std::vector<Sp> sp;
};
struct Part {
    bool outer;
    std::vector<XY> xy;
};

struct Stats {
    uint64_t features = 0, records = 0, perKind[9] = {}, perKindBand[9][8] = {}, perKindPrim[9][3] = {};
    uint64_t noGeometry = 0, missingVector = 0, lineBreaks = 0, linesMultiPart = 0, areaBreaks = 0, ringsOpen = 0,
             rings = 0, innerRings = 0, edges = 0, reversedEdges = 0;
    uint64_t catbrgMulti = 0, hunitsNotMetres = 0, nonAscii = 0, notes = 0;
    std::map<int, uint64_t> catpyl;
    std::map<std::string, uint64_t> refusedAttr, emptyAttr, noMember;
    std::map<int, uint64_t> verdatFeature, verdatDataset;
    uint64_t verdatNone = 0;
    void Add(const Stats& o) {
        features += o.features;
        records += o.records;
        for (int k = 0; k < 9; ++k) {
            perKind[k] += o.perKind[k];
            for (int b = 0; b < 8; ++b) perKindBand[k][b] += o.perKindBand[k][b];
            for (int p = 0; p < 3; ++p) perKindPrim[k][p] += o.perKindPrim[k][p];
        }
        noGeometry += o.noGeometry, missingVector += o.missingVector, lineBreaks += o.lineBreaks;
        linesMultiPart += o.linesMultiPart, areaBreaks += o.areaBreaks, ringsOpen += o.ringsOpen, rings += o.rings;
        innerRings += o.innerRings, edges += o.edges, reversedEdges += o.reversedEdges, catbrgMulti += o.catbrgMulti, hunitsNotMetres += o.hunitsNotMetres;
        nonAscii += o.nonAscii;
        notes += o.notes;
        for (auto& [k, v] : o.catpyl) catpyl[k] += v;
        for (auto& [k, v] : o.refusedAttr) refusedAttr[k] += v;
        for (auto& [k, v] : o.emptyAttr) emptyAttr[k] += v;
        for (auto& [k, v] : o.noMember) noMember[k] += v;
        for (auto& [k, v] : o.verdatFeature) verdatFeature[k] += v;
        for (auto& [k, v] : o.verdatDataset) verdatDataset[k] += v;
        verdatNone += o.verdatNone;
    }
};
struct Rec {
    int32_t iy, ix;
    int64_t id;
    std::string origin, bytes;
};
struct CellResult {
    std::string name;   // folder name
    std::string why;    // refused when not empty
    std::string dsnm;
    uint64_t bytes = 0;
    int updates = 0;
    Stats st;
    std::vector<Rec> recs;
};

template <class T>
void Put(std::string& s, const T& v) { s.append(reinterpret_cast<const char*>(&v), sizeof(T)); }
void CellOf(int32_t x, int32_t y, int32_t& iy, int32_t& ix) {   // planet_buildings' CellOf
    iy = static_cast<int32_t>(std::floor(static_cast<double>(y) * 1e-7 / kCellDeg));
    ix = static_cast<int32_t>(std::floor(static_cast<double>(x) * 1e-7 / kCellDeg));
}

// Reads one base cell; `dump` prints the DDR and every wanted feature.
void ReadCell(const std::string& file, CellResult& r, bool dump) {
    std::string bytes;
    {
        std::ifstream f(file, std::ios::binary);
        if (!f) { r.why = "cannot open"; return; }
        bytes.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    r.bytes = bytes.size();
    Ddf ddf;
    if (!ddf.Open(bytes)) { r.why = "ISO 8211: " + ddf.why; return; }
    if (dump) {
        printf("DDR: %zu field descriptions\n", ddf.defs.size());
        for (const FieldDef& d : ddf.defs)
            printf("  %s  %-40s %s%s  %s\n", d.tag.c_str(), d.name.c_str(), d.repeat ? "*" : "", d.arr.c_str() + (d.repeat ? 1 : 0),
                   d.fmtText.c_str());
    }
    std::unordered_map<uint64_t, Vec> vecs;
    std::vector<Feat> feats;
    int64_t comf = -1, huni = -1, vdat = 0;
    std::vector<Field> fields;
    std::vector<Val> v;
    while (ddf.Next(fields)) {
        if (fields.size() < 2) continue;
        const std::string_view t = fields[1].tag;   // fields[0] is 0001, the record identifier
        if (t == "DSID") {
            for (const Field& f : fields) {
                if (f.tag != "DSID" || !Decode(f, v)) continue;
                const FieldDef& d = *f.def;
                auto s = [&](const char* n) { int i = d.Index(n); return i < 0 ? std::string() : std::string(v[i].s); };
                r.dsnm = s("DSNM");
                if (dump) printf("DSID: DSNM %s EDTN %s UPDN %s UADT %s ISDT %s STED %s PRSP %lld INTU %lld\n", s("DSNM").c_str(),
                                 s("EDTN").c_str(), s("UPDN").c_str(), s("UADT").c_str(), s("ISDT").c_str(), s("STED").c_str(),
                                 (long long)v[d.Index("PRSP") < 0 ? 0 : d.Index("PRSP")].i,
                                 (long long)v[d.Index("INTU") < 0 ? 0 : d.Index("INTU")].i);
            }
        } else if (t == "DSPM") {
            if (!Decode(fields[1], v)) continue;
            const FieldDef& d = *fields[1].def;
            auto g = [&](const char* n) -> int64_t { int i = d.Index(n); return i < 0 ? -1 : v[i].i; };
            comf = g("COMF"), huni = g("HUNI"), vdat = g("VDAT");
            if (dump) printf("DSPM: HDAT %lld VDAT %lld SDAT %lld CSCL %lld DUNI %lld HUNI %lld PUNI %lld COMF %lld SOMF %lld\n",
                             (long long)g("HDAT"), (long long)vdat, (long long)g("SDAT"), (long long)g("CSCL"), (long long)g("DUNI"),
                             (long long)huni, (long long)g("PUNI"), (long long)comf, (long long)g("SOMF"));
        } else if (t == "VRID") {
            if (!Decode(fields[1], v)) continue;
            const FieldDef& d = *fields[1].def;
            const uint64_t key = (uint64_t(v[d.Index("RCNM")].i & 0xFF) << 32) | uint32_t(v[d.Index("RCID")].i);
            Vec& vec = vecs[key];
            int tb = 0;   // pointers without TOPI: first begin, second end
            for (size_t k = 2; k < fields.size(); ++k) {
                const Field& f = fields[k];
                if (f.tag == "SG2D" && Decode(f, v)) {
                    const int iy = f.def->Index("YCOO"), ix = f.def->Index("XCOO"), n = static_cast<int>(f.def->subs.size());
                    for (size_t q = 0; q + n <= v.size(); q += n)
                        vec.pts.push_back({static_cast<int32_t>(v[q + ix].i), static_cast<int32_t>(v[q + iy].i)});
                } else if (f.tag == "VRPT" && Decode(f, v)) {
                    const int in = f.def->Index("NAME"), it = f.def->Index("TOPI"), n = static_cast<int>(f.def->subs.size());
                    for (size_t q = 0; q + n <= v.size(); q += n) {
                        const uint64_t nk = NameKey(v[q + in].s);
                        const int topi = it < 0 ? 0 : static_cast<int>(v[q + it].i);
                        if (topi == 1 || (topi != 2 && tb == 0)) vec.beg = nk;
                        else vec.end = nk;
                        ++tb;
                    }
                }
            }
        } else if (t == "FRID") {
            if (!Decode(fields[1], v)) continue;
            const FieldDef& d = *fields[1].def;
            const uint16_t objl = static_cast<uint16_t>(v[d.Index("OBJL")].i);
            if (KindOfObjl(objl) < 0) continue;
            Feat ft;
            ft.objl = objl;
            ft.prim = static_cast<int>(v[d.Index("PRIM")].i);
            ft.grup = static_cast<int>(v[d.Index("GRUP")].i);
            for (size_t k = 2; k < fields.size(); ++k) {
                const Field& f = fields[k];
                if (f.tag == "FOID" && Decode(f, v)) {
                    ft.agen = static_cast<uint32_t>(v[f.def->Index("AGEN")].i);
                    ft.fidn = static_cast<uint32_t>(v[f.def->Index("FIDN")].i);
                    ft.fids = static_cast<uint32_t>(v[f.def->Index("FIDS")].i);
                    ft.id = static_cast<int64_t>((uint64_t(ft.agen & 0xFFFF) << 48) | (uint64_t(ft.fidn) << 16) | (ft.fids & 0xFFFF));
                } else if (f.tag == "ATTF" && Decode(f, v)) {
                    const int ia = f.def->Index("ATTL"), iv = f.def->Index("ATVL"), n = static_cast<int>(f.def->subs.size());
                    for (size_t q = 0; q + n <= v.size(); q += n)
                        ft.attrs.push_back({static_cast<uint16_t>(v[q + ia].i), std::string(v[q + iv].s)});
                } else if (f.tag == "FSPT" && Decode(f, v)) {
                    const FieldDef& g = *f.def;
                    const int in = g.Index("NAME"), io = g.Index("ORNT"), iu = g.Index("USAG"), im = g.Index("MASK");
                    const int n = static_cast<int>(g.subs.size());
                    for (size_t q = 0; q + n <= v.size(); q += n)
                        ft.sp.push_back({NameKey(v[q + in].s), static_cast<int>(v[q + io].i), static_cast<int>(v[q + iu].i),
                                         static_cast<int>(v[q + im].i)});
                }
            }
            feats.push_back(std::move(ft));
        }
    }
    if (!ddf.why.empty()) { r.why = "ISO 8211: " + ddf.why; return; }
    if (comf < 0) { r.why = "no DSPM"; return; }
    if (comf != kComf) { r.why = "COMF " + std::to_string(comf) + " (not 10,000,000: the integers are not 1e-7 degree)"; return; }
    if (huni != 1) { r.why = "HUNI " + std::to_string(huni) + " (heights not in metres)"; return; }

    const char bandCh = r.name.size() > 2 ? r.name[2] : '0';
    const int band = (bandCh >= '1' && bandCh <= '6') ? bandCh - '0' : 7;
    Stats& st = r.st;
    for (const Feat& ft : feats) {
        const int kind = KindOfObjl(ft.objl);
        ++st.features;
        // ---- attributes
        float clr[6] = {kNaN, kNaN, kNaN, kNaN, kNaN, kNaN};   // VERCLR VERCCL VERCOP VERCSA HORCLR HEIGHT
        uint8_t category = 0;
        int verdat = -1;
        std::string name, note;
        bool lengthsRefused = false;
        for (const auto& [c, val] : ft.attrs)
            if (c == HUNITS && !val.empty() && val != "1") lengthsRefused = true;
        if (lengthsRefused) ++st.hunitsNotMetres;
        for (const auto& [c, val] : ft.attrs) {
            int slot = -1;
            switch (c) {
            case VERCLR: slot = 0; break;
            case VERCCL: slot = 1; break;
            case VERCOP: slot = 2; break;
            case VERCSA: slot = 3; break;
            case HORCLR: slot = 4; break;
            case HEIGHT: slot = 5; break;
            case INFORM:
                note = Latin1ToUtf8(val);
                break;
            case CATPYL: {   // a pylon's category (RoadWays.h): 4 bridge tower, 5 bridge pier
                if (kind != 4 || val.empty()) break;
                char* e = nullptr;
                const long n = std::strtol(val.c_str(), &e, 10);
                if (*e || n < 1 || n > 5) ++st.refusedAttr["CATPYL"];
                else { category = static_cast<uint8_t>(n); ++st.catpyl[static_cast<int>(n)]; }
                break;
            }
            case OBJNAM:
                name = Latin1ToUtf8(val);
                if (name.size() != val.size()) ++st.nonAscii;
                break;
            case CATBRG: {
                if (val.empty()) { ++st.emptyAttr["CATBRG"]; break; }
                if (val.find(',') != std::string::npos) ++st.catbrgMulti;
                char* e = nullptr;
                const long n = std::strtol(val.c_str(), &e, 10);
                if (e == val.c_str() || n < 1 || n > 12) ++st.refusedAttr["CATBRG"];
                else category = static_cast<uint8_t>(n);
                break;
            }
            case VERDAT: {
                if (val.empty()) break;
                char* e = nullptr;
                const long n = std::strtol(val.c_str(), &e, 10);
                if (*e || n < 1 || n > 255) ++st.refusedAttr["VERDAT"];
                else verdat = static_cast<int>(n);
                break;
            }
            case VERLEN: case SCAMIN: case SORIND:
                if (!val.empty()) ++st.noMember[AttlName(c)];
                break;
            default: break;
            }
            if (slot >= 0) {
                if (val.empty()) { ++st.emptyAttr[AttlName(c)]; continue; }
                if (lengthsRefused) { ++st.refusedAttr[AttlName(c) + " (HUNITS not metres)"]; continue; }
                char* e = nullptr;
                const double d = std::strtod(val.c_str(), &e);
                if (*e || !std::isfinite(d)) ++st.refusedAttr[AttlName(c)];
                else clr[slot] = static_cast<float>(d);
            }
        }
        uint8_t vd = 0;
        if (verdat > 0) { vd = static_cast<uint8_t>(verdat); ++st.verdatFeature[verdat]; }
        else if (vdat > 0 && vdat < 256) { vd = static_cast<uint8_t>(vdat); ++st.verdatDataset[static_cast<int>(vdat)]; }
        else ++st.verdatNone;

        // ---- geometry
        std::vector<Part> parts;
        bool missing = false;
        int breaks = 0, open = 0;
        auto nodeXY = [&](uint64_t key, XY& out) {
            auto it = vecs.find(key);
            if (it == vecs.end() || it->second.pts.empty()) return false;
            out = it->second.pts[0];
            return true;
        };
        const uint8_t primitive = ft.prim == 1 ? 0 : ft.prim == 2 ? 1 : ft.prim == 3 ? 2 : 255;
        if (primitive == 0) {
            for (const Sp& s : ft.sp) {
                XY p;
                if (!nodeXY(s.name, p)) { missing = true; continue; }
                parts.push_back({true, {p}});
            }
        } else if (primitive == 1 || primitive == 2) {
            for (const Sp& s : ft.sp) {
                auto it = vecs.find(s.name);
                XY b, e;
                if (it == vecs.end() || !nodeXY(it->second.beg, b) || !nodeXY(it->second.end, e)) { missing = true; continue; }
                ++st.edges;
                std::vector<XY> run;
                run.reserve(it->second.pts.size() + 2);
                run.push_back(b);
                run.insert(run.end(), it->second.pts.begin(), it->second.pts.end());
                run.push_back(e);
                if (s.ornt == 2) { std::reverse(run.begin(), run.end()); ++st.reversedEdges; }
                const bool outer = s.usag != 2;
                Part* cur = parts.empty() ? nullptr : &parts.back();
                const bool closed = cur && primitive == 2 && cur->xy.size() > 1 && cur->xy.front() == cur->xy.back();
                if (cur && !closed && cur->xy.back().Near(run.front()) && (primitive == 1 || cur->outer == outer)) {
                    cur->xy.insert(cur->xy.end(), run.begin() + 1, run.end());
                } else {
                    if (cur && !closed) ++breaks;
                    parts.push_back({primitive == 1 ? true : outer, std::move(run)});
                }
            }
            if (primitive == 2) {
                for (Part& p : parts) {
                    ++st.rings;
                    if (!p.outer) ++st.innerRings;
                    if (!p.xy.front().Near(p.xy.back())) ++open;
                }
                std::stable_sort(parts.begin(), parts.end(), [](const Part& a, const Part& b) { return a.outer > b.outer; });
            }
        }
        if (missing) ++st.missingVector;
        if (primitive == 1) { st.lineBreaks += breaks; if (parts.size() > 1) ++st.linesMultiPart; }
        if (primitive == 2) { st.areaBreaks += breaks; st.ringsOpen += open; }

        size_t nv = 0;
        for (const Part& p : parts) nv += p.xy.size();
        if (dump) {
            printf("\n%s (OBJL %u) FOID %u/%u/%u id %lld  PRIM %d GRUP %d  edges %zu  parts %zu  vertices %zu", kKindNames[kind],
                   ft.objl, ft.agen, ft.fidn, ft.fids, (long long)ft.id, ft.prim, ft.grup, ft.sp.size(), parts.size(), nv);
            if (nv) printf("  first %.7f %.7f", parts[0].xy[0].x * 1e-7, parts[0].xy[0].y * 1e-7);
            if (breaks || open || missing) printf("  [breaks %d open rings %d missing %d]", breaks, open, missing ? 1 : 0);
            printf("\n   ");
            for (const auto& [c, val] : ft.attrs) printf(" %s=%s", AttlName(c).c_str(), val.c_str());
            printf("\n    -> verdat %u (%s)\n", vd, verdat > 0 ? "feature" : vd ? "dataset" : "none");
        }
        if (primitive == 255 || nv == 0) { ++st.noGeometry; continue; }

        // ---- the GABRDG01 record (RoadWays.h)
        std::string rec;
        Put(rec, ft.id);
        // clrUnderRef, serviceOn, serviceUnder, nbiKind, nbiType, navigation: NBI-only, 255 blank
        for (uint8_t b : {uint8_t(1), uint8_t(kind), category, primitive, vd, uint8_t(255), uint8_t(255), uint8_t(255),
                          uint8_t(255), uint8_t(255), uint8_t(255), uint8_t(0)})
            Put(rec, b);
        // vertClr .. horClr, clrOverDeck, clrUnder, minVertClrRoute, lengthM, maxSpanM, deckWidthM, heightM
        for (float f : {clr[0], clr[1], clr[2], clr[3], clr[4], kNaN, kNaN, kNaN, kNaN, kNaN, kNaN, clr[5]}) Put(rec, f);
        const std::string& full = r.dsnm.empty() ? r.name : r.dsnm;
        const std::string origin = full.substr(0, full.find('.'));   // "US5MA1VH", not "US5MA1VH.000"
        const std::string crosses;
        for (uint16_t u : {uint16_t(0), uint16_t(0), uint16_t(0), static_cast<uint16_t>(std::min<size_t>(name.size(), 65535)),
                           uint16_t(0), static_cast<uint16_t>(origin.size()), static_cast<uint16_t>(std::min<size_t>(note.size(), 65535))})
            Put(rec, u);
        rec.append(name, 0, std::min<size_t>(name.size(), 65535));
        rec += origin;
        rec.append(note, 0, std::min<size_t>(note.size(), 65535));
        if (!note.empty()) ++st.notes;
        Put(rec, static_cast<uint32_t>(parts.size()));
        for (const Part& p : parts) {
            Put(rec, static_cast<uint32_t>(p.xy.size()));
            Put(rec, static_cast<uint8_t>(p.outer ? 1 : 0));
            rec.append(3, '\0');
            rec.append(reinterpret_cast<const char*>(p.xy.data()), p.xy.size() * sizeof(XY));
        }
        Rec out;
        CellOf(parts[0].xy[0].x, parts[0].xy[0].y, out.iy, out.ix);
        out.id = ft.id;
        out.origin = origin;
        out.bytes = std::move(rec);
        r.recs.push_back(std::move(out));
        ++st.records;
        ++st.perKind[kind];
        ++st.perKindBand[kind][band];
        ++st.perKindPrim[kind][primitive];
    }
}

size_t PeakMemory() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS pmc{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) return pmc.PeakWorkingSetSize;
#endif
    return 0;
}

struct CellRow {
    int32_t iy, ix;
    int64_t off, n;
};

}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc >= 3 && std::string(argv[1]) == "--dump") {
        for (int a = 2; a < argc; ++a) {
            CellResult r;
            r.name = fs::path(argv[a]).stem().string();
            printf("==== %s\n", argv[a]);
            ReadCell(argv[a], r, true);
            if (!r.why.empty()) printf("REFUSED: %s\n", r.why.c_str());
            printf("\n%s: %llu wanted features, %llu records\n", r.name.c_str(), (unsigned long long)r.st.features,
                   (unsigned long long)r.st.records);
        }
        return 0;
    }
    if (argc < 3) {
        fprintf(stderr, "usage: enc_bridges <ENC_ROOT> <out dir> [threads]\n       enc_bridges --dump <cell.000> [more]\n");
        return 2;
    }
    const fs::path root = argv[1], outDir = argv[2];
    const unsigned threads = argc > 3 ? static_cast<unsigned>(std::atoi(argv[3])) : std::max(1u, std::thread::hardware_concurrency() - 2);
    fs::create_directories(outDir);

    // Every cell folder: its base cell, and the updates it holds (counted, not applied).
    std::vector<CellResult> cells;
    std::vector<std::string> noBase;
    for (const auto& e : fs::directory_iterator(root)) {
        if (!e.is_directory()) continue;
        CellResult r;
        r.name = e.path().filename().string();
        bool base = false;
        for (const auto& g : fs::directory_iterator(e.path())) {
            const std::string ext = g.path().extension().string();
            if (g.path().stem().string() != r.name || ext.size() != 4) continue;
            if (ext == ".000") base = true;
            else if (std::isdigit(static_cast<unsigned char>(ext[1])) && std::isdigit(static_cast<unsigned char>(ext[2])) &&
                     std::isdigit(static_cast<unsigned char>(ext[3])))
                ++r.updates;
        }
        if (!base) { noBase.push_back(r.name); continue; }
        cells.push_back(std::move(r));
    }
    std::sort(cells.begin(), cells.end(), [](const CellResult& a, const CellResult& b) { return a.name < b.name; });
    printf("[enc] %zu cells with a base cell, %zu folders without (%.1f s)\n", cells.size(), noBase.size(), Secs());

    std::atomic<size_t> next{0}, done{0};
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < threads; ++t) {
        pool.emplace_back([&] {
            for (size_t i; (i = next++) < cells.size();) {
                CellResult& r = cells[i];
                ReadCell((root / r.name / (r.name + ".000")).string(), r, false);
                const size_t d = ++done;
                if (d % 1000 == 0) printf("[enc] %zu / %zu cells (%.1f s)\n", d, cells.size(), Secs());
            }
        });
    }
    for (std::thread& t : pool) t.join();

    Stats all;
    std::vector<Rec> recs;
    uint64_t bytes = 0, updates = 0, cellsWithUpdates = 0, read = 0;
    std::map<std::string, std::vector<std::string>> refused;   // why -> cells
    for (CellResult& r : cells) {
        bytes += r.bytes;
        updates += r.updates;
        cellsWithUpdates += r.updates > 0;
        if (!r.why.empty()) { refused[r.why].push_back(r.name); continue; }
        ++read;
        all.Add(r.st);
        for (Rec& x : r.recs) recs.push_back(std::move(x));
        r.recs.clear();
        r.recs.shrink_to_fit();
    }
    std::sort(recs.begin(), recs.end(), [](const Rec& a, const Rec& b) {
        return std::tie(a.iy, a.ix, a.id, a.origin) < std::tie(b.iy, b.ix, b.id, b.origin);
    });

    const std::string stem = kStem;
    const fs::path bin = outDir / (stem + ".bridges.bin");
    std::vector<CellRow> rows;
    {
        std::ofstream f(bin, std::ios::binary);
        f.write("GABRDG01", 8);
        int64_t off = 8;
        for (const Rec& x : recs) {
            if (rows.empty() || rows.back().iy != x.iy || rows.back().ix != x.ix) rows.push_back({x.iy, x.ix, off, 0});
            ++rows.back().n;
            f.write(x.bytes.data(), static_cast<std::streamsize>(x.bytes.size()));
            off += static_cast<int64_t>(x.bytes.size());
        }
    }
    {
        std::ofstream f(outDir / (stem + ".bridges.idx"), std::ios::binary);
        for (const CellRow& c : rows) {
            f.write(reinterpret_cast<const char*>(&c.ix), 4);
            f.write(reinterpret_cast<const char*>(&c.iy), 4);
            f.write(reinterpret_cast<const char*>(&c.off), 8);
            f.write(reinterpret_cast<const char*>(&c.n), 8);
        }
    }
    std::string readmeTitle;
    {
        std::ifstream f(root / "README.TXT");
        std::string line;
        while (std::getline(f, line)) {
            while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
            if (!line.empty()) { readmeTitle = line; break; }
        }
    }
    const double wall = Secs();
    const size_t peak = PeakMemory();
    {
        std::ofstream f(outDir / (stem + ".bridges.json"));
        f << "{\n \"format\": \"GABRDG01\",\n";
        f << " \"source\": {\"root\": " << Json(root.string()) << ", \"cells\": " << cells.size() << ", \"cellsRead\": " << read
          << ", \"foldersWithoutBaseCell\": " << noBase.size() << ", \"bytes\": " << bytes
          << ", \"readmeFirstLine\": " << Json(readmeTitle)
          << ", \"built\": \"2026-10-09\", \"standard\": \"IHO S-57 edition 3.1 in ISO/IEC 8211\""
          << ", \"tool\": \"harvester/cpp/enc_bridges (own ISO 8211 reader, no GDAL)\"},\n";
        f << " \"licence\": \"NOAA ENC: use for any purpose (15 CFR 995); not for redistribution as provided; (c) NOAA Office of "
             "Coast Survey\",\n";
        f << " \"units\": {\"coordinates\": \"int32, 1e-7 degree, WGS84 lon/lat (the cell's YCOO/XCOO integers, COMF 10,000,000)\", "
             "\"clearances\": \"float32 metres (DSPM HUNI 1); NaN untagged or unknown\", \"verdat\": {";
        for (int k = 1; k < static_cast<int>(sizeof(kVerdat) / sizeof(kVerdat[0])); ++k)
            f << (k > 1 ? ", " : "") << "\"" << k << "\": " << Json(kVerdat[k]);
        f << "}},\n";
        f << " \"members\": {\"vertClr\": \"VERCLR\", \"vertClrClosed\": \"VERCCL\", \"vertClrOpen\": \"VERCOP\", \"vertClrSafe\": "
             "\"VERCSA\", \"horClr\": \"HORCLR\", \"heightM\": \"HEIGHT\", \"note\": \"INFORM (ISO 8859-1 to UTF-8)\", \"category\": \"CATBRG (the first of a list); a pylon's CATPYL\", \"nbiOnlyBytes\": \"255 blank\", \"name\": \"OBJNAM (ISO 8859-1 "
             "to UTF-8)\", \"origin\": \"the cell name (DSID DSNM)\", \"crosses\": \"(empty)\", \"verdat\": \"the feature's VERDAT if "
             "present, else the cell's DSPM VDAT\", \"id\": \"(AGEN << 48) | (FIDN << 16) | FIDS\"},\n";
        f << " \"geometry\": \"point: one part of one; line: its edges chained, a new part where they break; area: rings, the outer "
             "(USAG 1, 3) first, each ring keeping its closing vertex\",\n";
        f << " \"updates\": {\"applied\": false, \"files\": " << updates << ", \"cells\": " << cellsWithUpdates << ", \"perCell\": {";
        bool first = true;
        for (const CellResult& r : cells)
            if (r.updates) { f << (first ? "" : ", ") << Json(r.name) << ": " << r.updates; first = false; }
        f << "}},\n";
        f << " \"counts\": {\"records\": " << recs.size() << ", \"wantedFeatures\": " << all.features
          << ", \"noGeometry\": " << all.noGeometry << ", \"perKind\": {";
        for (int k = 0; k < 9; ++k) f << (k ? ", " : "") << "\"" << kKindNames[k] << "\": " << all.perKind[k];
        f << "}, \"perKindPrimitive\": {";
        for (int k = 0; k < 9; ++k)
            f << (k ? ", " : "") << "\"" << kKindNames[k] << "\": [" << all.perKindPrim[k][0] << ", " << all.perKindPrim[k][1] << ", "
              << all.perKindPrim[k][2] << "]";
        f << "}, \"perKindBand\": {";
        for (int k = 0; k < 9; ++k) {
            f << (k ? ", " : "") << "\"" << kKindNames[k] << "\": {";
            for (int b = 1; b <= 7; ++b) f << (b > 1 ? ", " : "") << "\"" << (b == 7 ? std::string("other") : std::to_string(b)) << "\": " << all.perKindBand[k][b];
            f << "}";
        }
        f << "}, \"bandNote\": \"the usage band is the cell name's third character (1 overview .. 6 berthing); overlapping cells of "
             "different bands duplicate a feature: not deduplicated\"},\n";
        f << " \"verdat\": {\"fromFeature\": {";
        first = true;
        for (auto& [k, n] : all.verdatFeature) { f << (first ? "" : ", ") << "\"" << k << "\": " << n; first = false; }
        f << "}, \"fromDataset\": {";
        first = true;
        for (auto& [k, n] : all.verdatDataset) { f << (first ? "" : ", ") << "\"" << k << "\": " << n; first = false; }
        f << "}, \"none\": " << all.verdatNone << "},\n";
        f << " \"refusedCells\": {";
        first = true;
        for (auto& [why, ns] : refused) {
            f << (first ? "" : ", ") << Json(why) << ": [";
            for (size_t i = 0; i < ns.size(); ++i) f << (i ? ", " : "") << Json(ns[i]);
            f << "]";
            first = false;
        }
        f << "},\n \"foldersWithoutBaseCell\": [";
        for (size_t i = 0; i < noBase.size(); ++i) f << (i ? ", " : "") << Json(noBase[i]);
        f << "],\n \"refusedAttributes\": {";
        first = true;
        for (auto& [k, n] : all.refusedAttr) { f << (first ? "" : ", ") << Json(k) << ": " << n; first = false; }
        f << "}, \"notes\": " << all.notes << ", \"catpyl\": {";
        first = true;
        for (auto& [k, n] : all.catpyl) { f << (first ? "" : ", ") << "\"" << k << "\": " << n; first = false; }
        f << "}, \"featuresHunitsNotMetres\": " << all.hunitsNotMetres << ", \"catbrgMultiValued\": " << all.catbrgMulti
          << ", \"emptyAttributes\": {";
        first = true;
        for (auto& [k, n] : all.emptyAttr) { f << (first ? "" : ", ") << Json(k) << ": " << n; first = false; }
        f << "},\n \"carriedWithoutMember\": {";
        first = true;
        for (auto& [k, n] : all.noMember) { f << (first ? "" : ", ") << Json(k) << ": " << n; first = false; }
        f << "},\n \"geometryChecks\": {\"edges\": " << all.edges << ", \"reversedEdges\": " << all.reversedEdges << ", \"lineChainBreaks\": " << all.lineBreaks
          << ", \"linesMultiPart\": " << all.linesMultiPart << ", \"areaChainBreaks\": " << all.areaBreaks << ", \"rings\": " << all.rings
          << ", \"innerRings\": " << all.innerRings << ", \"ringsOpen\": " << all.ringsOpen << ", \"featuresMissingVector\": "
          << all.missingVector << "},\n";
        f << " \"cellDeg\": 0.05,\n \"cellIndex\": \"" << stem << ".bridges.idx\",\n \"cellCount\": " << rows.size() << "\n}\n";
    }
    printf("[enc] %zu cells read of %zu (%zu refused), %llu wanted features -> %zu records in %zu cells, %.1f s, peak %.0f MB\n",
           (size_t)read, cells.size(), cells.size() - read, (unsigned long long)all.features, recs.size(), rows.size(), wall,
           peak / 1048576.0);
    for (int k = 0; k < 9; ++k) {
        printf("[enc]   %-17s %7llu  bands", kKindNames[k], (unsigned long long)all.perKind[k]);
        for (int b = 1; b <= 7; ++b) printf(" %llu", (unsigned long long)all.perKindBand[k][b]);
        printf("  prim %llu/%llu/%llu\n", (unsigned long long)all.perKindPrim[k][0], (unsigned long long)all.perKindPrim[k][1],
               (unsigned long long)all.perKindPrim[k][2]);
    }
    printf("[enc] verdat from feature:");
    for (auto& [k, n] : all.verdatFeature) printf(" %d:%llu", k, (unsigned long long)n);
    printf("; from dataset:");
    for (auto& [k, n] : all.verdatDataset) printf(" %d:%llu", k, (unsigned long long)n);
    printf("; none %llu\n", (unsigned long long)all.verdatNone);
    printf("[enc] geometry: edges %llu (reversed %llu), line breaks %llu (multi-part lines %llu), area breaks %llu, rings %llu (inner %llu, open %llu), "
           "missing vector %llu, no geometry %llu\n",
           (unsigned long long)all.edges, (unsigned long long)all.reversedEdges, (unsigned long long)all.lineBreaks, (unsigned long long)all.linesMultiPart,
           (unsigned long long)all.areaBreaks, (unsigned long long)all.rings, (unsigned long long)all.innerRings,
           (unsigned long long)all.ringsOpen, (unsigned long long)all.missingVector, (unsigned long long)all.noGeometry);
    printf("[enc] updates skipped: %llu files in %llu cells\n", (unsigned long long)updates, (unsigned long long)cellsWithUpdates);
    for (auto& [why, ns] : refused) printf("[enc] refused %zu: %s (e.g. %s)\n", ns.size(), why.c_str(), ns[0].c_str());
    printf("[enc] %s: %llu bytes\n", bin.string().c_str(), (unsigned long long)fs::file_size(bin));
    return 0;
}
