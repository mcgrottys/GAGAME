// ================================================================================================
//  nbi_bridges.cpp - the FHWA National Bridge Inventory into GABRDG01 (src/compose/RoadWays.h).
//
//  The NBI "all records, delimited" file: comma separated, the text qualifier a SINGLE quote (a quote
//  inside a quoted field doubled), one header row naming the items (STATE_CODE_001, ...). Columns are
//  found BY NAME, never by position. Every row is one record: RECORD_TYPE_005A '1' is the inventory
//  route carried ON the structure, '2' a single route UNDER it, 'A'..'Z' one of several routes under
//  it (the coding guide's Item 5A). The "under" rows carry the route items only: their structure,
//  clearance and navigation items are blank, which is UNTAGGED (NaN / 0 / 255), not a refusal.
//
//  The laws, as the building harvest's (planet_buildings.cpp):
//    LOAD reports the file faithfully: every row is a record or a counted refusal.
//    NORMALIZE converts only where the coding guide makes the conversion legal, and REFUSES (counts,
//    never guesses) where it does not. The sentinels, each with its item and the guide's words (the
//    1995 metric "Recording and Coding Guide for the Structure Inventory and Appraisal of the Nation's
//    Bridges"), are written into the manifest by kRules below; the data's own tallies confirmed them.
//    Coordinates: Items 16/17 are degrees, minutes, seconds to the hundredth (DDMMSSss, DDDMMSSss),
//    longitudes coded positive and WEST (negated here); a coordinate that is blank, not digits, zero,
//    with minutes or seconds of 60 or more, or outside the US states and territories the file holds
//    (latitude 13..72 N, longitude 64..180 W) is refused and the row dropped (counted by reason).
//    Metric items (the _MT_ columns, and 10 / 54B, which the file also states in metres) pass through
//    as float32 metres; a number that does not parse is refused (counted per column), the member NaN.
//
//  Output (little-endian; the layouts are RoadWays.h's): <out>/nbi-2025.bridges.bin ("GABRDG01" then
//  the records sorted by (iy, ix, id) of their 0.05 degree cell), .bridges.idx (int32 ix, int32 iy,
//  int64 offset, int64 count, by (iy, ix)), .bridges.json (source, licence, units, rules, counts).
//
//  Usage: nbi_bridges <nbi delimited .txt> <out dir>            harvest
//         nbi_bridges --probe <manifest .json> lon lat radiusKm  every record within the radius
//  Licence of the data: US public domain (FHWA).
// ================================================================================================
#include <algorithm>
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
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr double kCellDeg = 0.05;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
const char* const kStem = "nbi-2025";

auto t0 = std::chrono::steady_clock::now();
double Secs() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }

template <class T>
void Put(std::string& s, const T& v) { s.append(reinterpret_cast<const char*>(&v), sizeof(T)); }

uint64_t Fnv1a64(const std::string& s) {
    uint64_t h = 1469598103934665603ull;
    for (unsigned char c : s) {
        h ^= c;
        h *= 1099511628211ull;
    }
    return h;
}

std::string Trim(const std::string& s, bool lead) {
    size_t a = 0, b = s.size();
    if (lead)
        while (a < b && (s[a] == ' ' || s[a] == '\t')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r')) --b;
    return s.substr(a, b - a);
}

// The file's text is ASCII almost everywhere; a byte string that is not valid UTF-8 is read as
// Latin-1 (the FHWA files' own code page) and written as UTF-8. Counted in the manifest.
bool ValidUtf8(const std::string& s) {
    for (size_t i = 0; i < s.size();) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        const int n = c < 0x80 ? 0 : (c >> 5) == 6 ? 1 : (c >> 4) == 14 ? 2 : (c >> 3) == 30 ? 3 : -1;
        if (n < 0) return false;
        for (int k = 1; k <= n; ++k)
            if (i + k >= s.size() || (static_cast<unsigned char>(s[i + k]) >> 6) != 2) return false;
        i += n + 1;
    }
    return true;
}
std::string Latin1ToUtf8(const std::string& s) {
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

std::string JsonEsc(const std::string& s) {
    std::string o;
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += static_cast<char>(c); }
        else if (c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); o += b; }
        else o += static_cast<char>(c);
    }
    return o;
}

// ---- LOAD: one record of the delimited file (single-quote qualifier, '' inside a quoted field) ---
// Returns false at the end of the data. Fields are the raw text with the qualifier removed.
bool NextRow(const std::string& d, size_t& p, std::vector<std::string>& f) {
    f.clear();
    if (p >= d.size()) return false;
    std::string cur;
    bool quoted = false, wasQuoted = false;
    while (p < d.size()) {
        const char c = d[p];
        if (quoted) {
            if (c == '\'') {
                if (p + 1 < d.size() && d[p + 1] == '\'') { cur += '\''; p += 2; continue; }
                quoted = false;
                ++p;
                continue;
            }
            cur += c;
            ++p;
            continue;
        }
        if (c == '\'' && cur.empty() && !wasQuoted) { quoted = wasQuoted = true; ++p; continue; }
        if (c == ',') { f.push_back(std::move(cur)); cur.clear(); wasQuoted = false; ++p; continue; }
        if (c == '\n' || c == '\r') {
            if (c == '\r' && p + 1 < d.size() && d[p + 1] == '\n') ++p;
            ++p;
            break;
        }
        cur += c;
        ++p;
    }
    f.push_back(std::move(cur));
    return true;
}

// ---- NORMALIZE ---------------------------------------------------------------------------------
// The columns read, by header name; the index into the row is resolved from the header.
enum Col {
    cState, cStruct, cRecType, cFeatures, cFacility, c010, cLat, cLon, cYear, c038, c039, c040, c042A, c042B, c043A,
    c043B, c045, c046, c048, c049, c052, c053, c054A, c054B, c116, kCols
};
const char* const kColNames[kCols] = {
    "STATE_CODE_001", "STRUCTURE_NUMBER_008", "RECORD_TYPE_005A", "FEATURES_DESC_006A", "FACILITY_CARRIED_007",
    "MIN_VERT_CLR_010", "LAT_016", "LONG_017", "YEAR_BUILT_027", "NAVIGATION_038", "NAV_VERT_CLR_MT_039",
    "NAV_HORR_CLR_MT_040", "SERVICE_ON_042A", "SERVICE_UND_042B", "STRUCTURE_KIND_043A", "STRUCTURE_TYPE_043B",
    "MAIN_UNIT_SPANS_045", "APPR_SPANS_046", "MAX_SPAN_LEN_MT_048", "STRUCTURE_LEN_MT_049", "DECK_WIDTH_MT_052",
    "VERT_CLR_OVER_MT_053", "VERT_CLR_UND_REF_054A", "VERT_CLR_UND_054B", "MIN_NAV_CLR_MT_116"};

// Each sentinel rule: the item, what the file holds, what is written, and the guide's words.
struct Rule {
    const char* item;
    const char* coded;
    const char* written;
    const char* guide;
};
const Rule kRules[] = {
    {"10", "99.99", "+inf (no restriction)",
     "Item 10, Inventory Route, Minimum Vertical Clearance: 'If the restriction is 30 meters or greater, code 9999' "
     "(XX.XX m, so 99.99 in this file); 'When no restriction exists, code 9999'"},
    {"10", "0", "NaN, refused (counted)",
     "the guide has no code 0 for Item 10; FHWA's NBI data checks flag 'Minimum Vertical Clearance equal to 0' as an error"},
    {"53", "99.99", "+inf (no restriction)",
     "Item 53, Minimum Vertical Clearance Over Bridge Roadway: 'When no superstructure restriction exists above the "
     "bridge roadway, or when a restriction is 30 meters or greater, code 9999' (99.99 in this file)"},
    {"53", "0", "NaN, refused (counted)", "the guide has no code 0 for Item 53"},
    {"54B", "99.99", "+inf (no restriction)",
     "Item 54B, Minimum Vertical Underclearance: 'When the vertical clearance restriction is 30 meters or greater, it "
     "should be coded as 9999'"},
    {"54B", "0 with 54A = N", "NaN (not applicable)",
     "Item 54: 'If the feature is not a highway or railroad, code the minimum vertical clearance 0000'; 54A 'N Feature "
     "not a highway or railroad'"},
    {"54B", "0 with 54A = H, R or blank", "NaN, refused (counted)", "a zero underclearance over a highway or railroad is not a code of the guide"},
    {"39", "0 with 38 = 0 or N", "NaN (not applicable)",
     "Item 39, Navigation Vertical Clearance: 'If Item 38 - Navigation Control is coded 0 or N, code 0000 to indicate "
     "not applicable'"},
    {"40", "0 with 38 = 0 or N", "NaN (not applicable)",
     "Item 40, Navigation Horizontal Clearance: 'If Item 38 - Navigation Control has been coded 0 or N, code 00000 to "
     "indicate not applicable'"},
    {"39, 40", "0 with 38 = 1 or blank", "NaN, refused (counted)", "a zero navigation clearance under navigation control is not a code of the guide"},
    {"116", "0", "NaN (not applicable)",
     "Item 116, Minimum Navigation Vertical Clearance, Vertical Lift Bridge: 'Item 116 should be coded only for "
     "vertical lift bridges in the dropped or closed position' (state guides: 'otherwise leave blank', the field "
     "defaulting to zeros)"},
    {"all", "blank", "NaN / 0 / 255 (untagged)", "not coded in the row (the route-under records, 5A = 2 or A..Z, leave the structure items blank)"},
    {"16, 17", "DDMMSSss, DDDMMSSss", "int32 1e-7 degree, longitude negated (west)",
     "Items 16/17: 'degrees, minutes, and seconds to the nearest hundredth of a second (8 digits)' / '(9 digits)'"},
};

struct Tally {
    uint64_t refused = 0;        // a number that does not parse
    uint64_t toInf = 0;          // 99.99 -> +inf
    uint64_t toNaN = 0;          // a not-applicable code -> NaN
    uint64_t zeroRefused = 0;    // a zero the guide gives no meaning to
    uint64_t nearSentinel = 0;   // 99, 99.9, 99.97, 999.x, 9999.x: passed literally, counted
    uint64_t blank = 0;
};

// A metres field: blank -> NaN (untagged); not a number -> refused.
bool Number(const std::string& raw, float& out, Tally& t) {
    const std::string s = Trim(raw, true);
    if (s.empty()) { ++t.blank; out = kNaN; return false; }
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end != s.c_str() + s.size() || !std::isfinite(v) || v < 0) { ++t.refused; out = kNaN; return false; }
    out = static_cast<float>(v);
    return true;
}
bool IsNearSentinel(const std::string& raw, float v) {
    const std::string s = Trim(raw, true);
    return s != "99.99" && ((v >= 98.9f && v < 100.0f) || v >= 999.0f);
}
uint16_t Count16(const std::string& raw, uint64_t& refused) {
    const std::string s = Trim(raw, true);
    if (s.empty()) return 0;
    char* end = nullptr;
    const long v = std::strtol(s.c_str(), &end, 10);
    if (end != s.c_str() + s.size() || v < 0 || v > 65535) { ++refused; return 0; }
    return static_cast<uint16_t>(v);
}
// A raw code (42A/B, 43A/B): 0 and 00 are real codes ("other"), so a blank is 255 (RoadWays.h).
uint8_t Code8(const std::string& raw, uint64_t& refused) {
    const std::string s = Trim(raw, true);
    if (s.empty()) return 255;
    char* end = nullptr;
    const long v = std::strtol(s.c_str(), &end, 10);
    if (end != s.c_str() + s.size() || v < 0 || v > 254) { ++refused; return 255; }
    return static_cast<uint8_t>(v);
}

enum CoordWhy { kOk, kBlank, kNotDigits, kZero, kSec60, kMinSec, kRange, kWhyCount };
// kSec60: seconds coded exactly 60.00 (a carry the coder did not make, e.g. 122 08 60.00); refused
// like any other out-of-range field, but counted apart so the choice to carry it stays visible.
const char* const kCoordWhy[] = {"ok", "blank", "not_digits", "zero", "seconds_exactly_60", "minutes_or_seconds_over_60", "outside_us_range"};
// DDMMSSss / DDDMMSSss -> degrees. `digits` is the field's width (8 or 9).
CoordWhy Dms(const std::string& raw, size_t digits, double& deg) {
    const std::string s = Trim(raw, true);
    if (s.empty()) return kBlank;
    if (s.size() > digits) return kNotDigits;
    for (char c : s)
        if (c < '0' || c > '9') return kNotDigits;
    const long long v = std::atoll(s.c_str());
    if (v == 0) return kZero;
    const long long d = v / 1000000, m = (v / 10000) % 100, cs = v % 10000;
    if (m < 60 && cs == 6000) return kSec60;
    if (m >= 60 || cs >= 6000) return kMinSec;
    deg = static_cast<double>(d) + m / 60.0 + cs / 360000.0;
    return kOk;
}

struct Rec {
    int32_t iy, ix;
    int64_t id;
    uint32_t row;
    std::string bytes;
};

int Harvest(const fs::path& in, const fs::path& outDir) {
    std::string data;
    {
        std::ifstream f(in, std::ios::binary);
        if (!f) { fprintf(stderr, "[nbi] cannot read %s\n", in.string().c_str()); return 1; }
        data.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    printf("[nbi] read %zu bytes (%.1f s)\n", data.size(), Secs());
    size_t p = 0;
    std::vector<std::string> f;
    if (data.size() >= 3 && static_cast<unsigned char>(data[0]) == 0xEF) p = 3;   // a UTF-8 BOM
    NextRow(data, p, f);
    std::unordered_map<std::string, size_t> where;
    for (size_t i = 0; i < f.size(); ++i) where[Trim(f[i], true)] = i;
    size_t col[kCols];
    for (int c = 0; c < kCols; ++c) {
        auto it = where.find(kColNames[c]);
        if (it == where.end()) { fprintf(stderr, "[nbi] no column %s in the header\n", kColNames[c]); return 1; }
        col[c] = it->second;
    }
    const size_t width = f.size();

    Tally t010, t039, t040, t048, t049, t052, t053, t054B, t116;
    uint64_t refusedCount = 0, refusedCode = 0, transcoded = 0, badWidth = 0, rows = 0;
    uint64_t coordRefused[kWhyCount] = {};
    std::map<std::string, uint64_t> byState, byType, byStateRefused;
    std::map<std::string, uint64_t> nav039Raw;   // raw 39 where 38 = '1'
    uint64_t literal38NoControl39 = 0, literal38NoControl40 = 0, positive116NotLift = 0;
    std::map<std::pair<std::string, std::string>, int> onUnder;   // (state, structure) -> bit 1 on, bit 2 under
    std::unordered_map<int64_t, uint32_t> ids;
    uint64_t idDup = 0;
    std::vector<std::string> dupOrigins;
    std::vector<Rec> recs;
    recs.reserve(760000);
    std::map<int, uint64_t> byCategory;
    uint64_t blank255[6] = {};
    const char* const kCodeCols[6] = {"clrUnderRef (54A)", "serviceOn (42A)", "serviceUnder (42B)",
                                      "nbiKind (43A)",     "nbiType (43B)",   "navigation (38)"};

    while (NextRow(data, p, f)) {
        if (f.size() == 1 && Trim(f[0], true).empty()) continue;   // a trailing empty line
        ++rows;
        if (f.size() != width) { ++badWidth; continue; }
        auto F = [&](Col c) -> const std::string& { return f[col[c]]; };
        const std::string state = Trim(F(cState), true);
        const std::string structure = Trim(F(cStruct), true);
        const std::string type = Trim(F(cRecType), true);
        double latDeg = 0, lonDeg = 0;
        CoordWhy w = Dms(F(cLat), 8, latDeg);
        if (w == kOk) w = Dms(F(cLon), 9, lonDeg);
        if (w == kOk && !(latDeg >= 13.0 && latDeg <= 72.0 && lonDeg >= 64.0 && lonDeg <= 180.0)) w = kRange;
        if (w != kOk) { ++coordRefused[w]; ++byStateRefused[state]; continue; }
        ++byState[state];
        ++byType[type];
        onUnder[{state, structure}] |= (type == "1" ? 1 : 2);

        const std::string origin = state + "|" + structure + "|" + type;
        const int64_t id = static_cast<int64_t>(Fnv1a64(origin));
        if (!ids.emplace(id, static_cast<uint32_t>(rows)).second) {
            ++idDup;
            if (dupOrigins.size() < 20) dupOrigins.push_back(origin);
        }

        // Codes, raw.
        const std::string s038 = Trim(F(c038), true), s054A = Trim(F(c054A), true), s043B = Trim(F(c043B), true);
        const uint8_t navigation = s038 == "0" ? 0 : s038 == "1" ? 1 : s038 == "N" ? 2 : 255;
        if (navigation == 255 && !s038.empty()) ++refusedCode;
        const uint8_t clrUnderRef = (s054A == "H" || s054A == "R" || s054A == "N") ? static_cast<uint8_t>(s054A[0]) : 255;
        if (!clrUnderRef && !s054A.empty()) ++refusedCode;
        const uint8_t serviceOn = Code8(F(c042A), refusedCode), serviceUnder = Code8(F(c042B), refusedCode);
        const uint8_t nbiKind = Code8(F(c043A), refusedCode), nbiType = Code8(F(c043B), refusedCode);
        // 43B -> CATBRG: blank (255) or 00 "other" -> 0 unknown; a known design -> its CATBRG or 1 fixed.
        uint8_t kind = 0, category = 0;   // BridgeKind::Bridge, CATBRG unknown
        if (nbiType != 255 && nbiType != 0) {
            switch (nbiType) {
                case 13: category = 12; break;   // suspension
                case 15: category = 4; break;    // lifting
                case 16: category = 5; break;    // bascule
                case 17: category = 3; break;    // swing
                case 18: kind = 3; break;        // BridgeKind::Tunnel, CATBRG not applicable
                default: category = 1; break;    // fixed (19 culvert stays a Bridge with nbiType 19)
            }
        }

        // Clearances.
        float v010, v039, v040, v053, v054B, v116, v048, v049, v052;
        if (Number(F(c010), v010, t010)) {
            if (Trim(F(c010), true) == "99.99") { v010 = kInf; ++t010.toInf; }
            else if (v010 == 0) { v010 = kNaN; ++t010.zeroRefused; }
            else if (IsNearSentinel(F(c010), v010)) ++t010.nearSentinel;
        }
        if (Number(F(c053), v053, t053)) {
            if (Trim(F(c053), true) == "99.99") { v053 = kInf; ++t053.toInf; }
            else if (v053 == 0) { v053 = kNaN; ++t053.zeroRefused; }
            else if (IsNearSentinel(F(c053), v053)) ++t053.nearSentinel;
        }
        if (Number(F(c054B), v054B, t054B)) {
            if (Trim(F(c054B), true) == "99.99") { v054B = kInf; ++t054B.toInf; }
            else if (v054B == 0) {
                v054B = kNaN;
                if (clrUnderRef == 'N') ++t054B.toNaN; else ++t054B.zeroRefused;
            } else if (IsNearSentinel(F(c054B), v054B)) ++t054B.nearSentinel;
        }
        auto nav = [&](Col c, float& v, Tally& t, uint64_t& literalNoControl) {
            if (!Number(F(c), v, t)) return;
            if (v == 0) {
                v = kNaN;
                if (navigation == 0 || navigation == 2) ++t.toNaN; else ++t.zeroRefused;
            } else {
                if (navigation == 0 || navigation == 2) ++literalNoControl;
                if (IsNearSentinel(F(c), v)) ++t.nearSentinel;
            }
        };
        nav(c039, v039, t039, literal38NoControl39);
        nav(c040, v040, t040, literal38NoControl40);
        if (navigation == 1) ++nav039Raw[Trim(F(c039), true)];
        if (Number(F(c116), v116, t116)) {
            if (v116 == 0) { v116 = kNaN; ++t116.toNaN; }
            else {
                if (nbiType != 15) ++positive116NotLift;
                if (IsNearSentinel(F(c116), v116)) ++t116.nearSentinel;
            }
        }
        Number(F(c048), v048, t048);
        Number(F(c049), v049, t049);
        Number(F(c052), v052, t052);
        const uint16_t spansMain = Count16(F(c045), refusedCount), spansApproach = Count16(F(c046), refusedCount);
        const uint16_t year = Count16(F(cYear), refusedCount);

        auto text = [&](const std::string& raw) {
            std::string s = Trim(raw, false);
            if (!ValidUtf8(s)) { s = Latin1ToUtf8(s); ++transcoded; }
            if (s.size() > 65535) s.resize(65535);
            return s;
        };
        const std::string name = text(F(cFacility)), crosses = text(F(cFeatures)), org = text(origin);

        const int32_t x = static_cast<int32_t>(std::llround(-lonDeg * 1e7));
        const int32_t y = static_cast<int32_t>(std::llround(latDeg * 1e7));
        Rec r;
        r.iy = static_cast<int32_t>(std::floor(static_cast<double>(y) * 1e-7 / kCellDeg));
        r.ix = static_cast<int32_t>(std::floor(static_cast<double>(x) * 1e-7 / kCellDeg));
        r.id = id;
        r.row = static_cast<uint32_t>(rows);
        std::string& b = r.bytes;
        Put(b, id);
        for (uint8_t u : {uint8_t(0) /*source Nbi*/, kind, category, uint8_t(0) /*point*/, uint8_t(0) /*verdat*/,
                          clrUnderRef, serviceOn, serviceUnder, nbiKind, nbiType, navigation, uint8_t(0)}) {
            Put(b, u);
        }
        // vertClr, vertClrClosed, vertClrOpen, vertClrSafe, horClr, clrOverDeck, clrUnder, minVertClrRoute,
        // lengthM, maxSpanM, deckWidthM, heightM
        for (float v : {v039, v116, kNaN, kNaN, v040, v053, v054B, v010, v049, v048, v052, kNaN /* heightM */}) Put(b, v);
        for (uint16_t u : {spansMain, spansApproach, year, static_cast<uint16_t>(name.size()),
                           static_cast<uint16_t>(crosses.size()), static_cast<uint16_t>(org.size()), uint16_t(0) /* noteLen */}) {
            Put(b, u);
        }
        b += name;
        b += crosses;
        b += org;   // note: empty
        Put(b, uint32_t(1));   // one part
        Put(b, uint32_t(1));   // of one point
        Put(b, uint8_t(1));    // outer
        b.append(3, '\0');
        Put(b, x);
        Put(b, y);
        ++byCategory[category];
        const uint8_t codes[6] = {clrUnderRef, serviceOn, serviceUnder, nbiKind, nbiType, navigation};
        for (int k = 0; k < 6; ++k)
            if (codes[k] == 255) ++blank255[k];
        recs.push_back(std::move(r));
    }
    printf("[nbi] %llu rows, %zu records (%.1f s)\n", (unsigned long long)rows, recs.size(), Secs());

    std::sort(recs.begin(), recs.end(), [](const Rec& a, const Rec& b) {
        if (a.iy != b.iy) return a.iy < b.iy;
        if (a.ix != b.ix) return a.ix < b.ix;
        if (a.id != b.id) return a.id < b.id;
        return a.row < b.row;
    });
    fs::create_directories(outDir);
    const fs::path bin = outDir / (std::string(kStem) + ".bridges.bin");
    struct CellRow {
        int32_t iy, ix;
        int64_t off, n;
    };
    std::vector<CellRow> cells;
    {
        std::ofstream o(bin, std::ios::binary);
        o.write("GABRDG01", 8);
        int64_t off = 8;
        for (const Rec& r : recs) {
            if (cells.empty() || cells.back().iy != r.iy || cells.back().ix != r.ix) cells.push_back({r.iy, r.ix, off, 0});
            ++cells.back().n;
            o.write(r.bytes.data(), static_cast<std::streamsize>(r.bytes.size()));
            off += static_cast<int64_t>(r.bytes.size());
        }
    }
    {
        std::ofstream o(outDir / (std::string(kStem) + ".bridges.idx"), std::ios::binary);
        for (const CellRow& c : cells) {
            o.write(reinterpret_cast<const char*>(&c.ix), 4);
            o.write(reinterpret_cast<const char*>(&c.iy), 4);
            o.write(reinterpret_cast<const char*>(&c.off), 8);
            o.write(reinterpret_cast<const char*>(&c.n), 8);
        }
    }
    uint64_t structures = onUnder.size(), both = 0, onOnly = 0, underOnly = 0;
    for (const auto& [k, v] : onUnder) (v == 3 ? both : v == 1 ? onOnly : underOnly)++;
    std::vector<std::pair<uint64_t, std::string>> top39;
    for (const auto& [k, v] : nav039Raw) top39.push_back({v, k});
    std::sort(top39.begin(), top39.end(), [](const auto& a, const auto& b) { return a.first != b.first ? a.first > b.first : a.second < b.second; });
    if (top39.size() > 10) top39.resize(10);

    auto tally = [](const Tally& t) {
        char s[320];
        snprintf(s, sizeof(s),
                 "{\"blank\": %llu, \"unparsable\": %llu, \"9999_to_inf\": %llu, \"not_applicable_to_nan\": %llu, "
                 "\"zero_refused\": %llu, \"near_sentinel_literal\": %llu}",
                 (unsigned long long)t.blank, (unsigned long long)t.refused, (unsigned long long)t.toInf,
                 (unsigned long long)t.toNaN, (unsigned long long)t.zeroRefused, (unsigned long long)t.nearSentinel);
        return std::string(s);
    };
    auto counts = [](const std::map<std::string, uint64_t>& m) {
        std::string s = "{";
        for (const auto& [k, v] : m) s += (s.size() > 1 ? ", \"" : "\"") + JsonEsc(k) + "\": " + std::to_string(v);
        return s + "}";
    };
    {
        std::ofstream o(outDir / (std::string(kStem) + ".bridges.json"));
        o << "{\n \"format\": \"GABRDG01\",\n \"source\": {\"file\": \"" << JsonEsc(in.filename().string()) << "\", \"bytes\": "
          << fs::file_size(in)
          << ", \"name\": \"FHWA National Bridge Inventory 2025, all records, delimited\", \"tool\": \"harvester/cpp/nbi_bridges\"},\n"
          << " \"licence\": \"US public domain (FHWA NBI 2025)\",\n"
          << " \"units\": {\"coordinates\": \"int32, 1e-7 degree, lon/lat (NBI items 16/17, datum not stated in the file)\", "
             "\"lengths\": \"float32 metres; NaN untagged or not applicable; +inf no restriction\", \"yearBuilt\": \"year\", "
             "\"spans\": \"count\"},\n"
          << " \"datum\": \"NBI clearances are measured from the feature's own reference; the navigation clearance datum is "
             "not stated in the file\",\n"
          << " \"identity\": {\"origin\": \"<state code>|<structure number, spaces trimmed>|<record type 5A>\", \"id\": "
             "\"FNV-1a 64 of origin\", \"name\": \"item 7 facility carried\", \"crosses\": \"item 6A features intersected\"},\n"
          << " \"members\": {\"vertClr\": \"39\", \"vertClrClosed\": \"116\", \"horClr\": \"40\", \"clrOverDeck\": \"53\", "
             "\"clrUnder\": \"54B\", \"clrUnderRef\": \"54A\", \"minVertClrRoute\": \"10\", \"lengthM\": \"49\", \"maxSpanM\": "
             "\"48\", \"deckWidthM\": \"52\", \"spansMain\": \"45\", \"spansApproach\": \"46\", \"yearBuilt\": \"27\", "
             "\"navigation\": \"38 ('0' -> 0, '1' -> 1, 'N' -> 2, blank -> 255)\", \"serviceOn\": \"42A\", \"serviceUnder\": "
             "\"42B\", \"nbiKind\": \"43A\", \"nbiType\": \"43B\", \"category\": \"43B: 13 -> 12, 15 -> 4, 16 -> 5, 17 -> 3, 18 -> "
             "kind Tunnel (category 0), blank or 00 -> 0, every other design -> 1 fixed\", \"blankCodes\": \"255 in clrUnderRef, "
             "serviceOn, serviceUnder, nbiKind, nbiType, navigation (0 / 00 are NBI's real 'other')\", \"heightM\": "
             "\"NaN (NBI has no height)\", \"note\": \"empty\"},\n"
          << " \"rules\": [\n";
        for (size_t i = 0; i < sizeof(kRules) / sizeof(kRules[0]); ++i) {
            o << "  {\"item\": \"" << kRules[i].item << "\", \"coded\": \"" << JsonEsc(kRules[i].coded) << "\", \"written\": \""
              << JsonEsc(kRules[i].written) << "\", \"guide\": \"" << JsonEsc(kRules[i].guide) << "\"}"
              << (i + 1 < sizeof(kRules) / sizeof(kRules[0]) ? ",\n" : "\n");
        }
        o << " ],\n \"guide\": \"FHWA Recording and Coding Guide for the Structure Inventory and Appraisal of the Nation's "
             "Bridges (1995, FHWA-PD-96-001); wording as transcribed in the avi8/coding_guide dataset and the North Dakota "
             "and Ohio state guides\",\n"
          << " \"counts\": {\"rows\": " << rows << ", \"records\": " << recs.size() << ", \"structures\": " << structures
          << ", \"structures_on_and_under\": " << both << ", \"structures_on_only\": " << onOnly
          << ", \"structures_under_only\": " << underOnly << ", \"id_collisions\": " << idDup
          << ", \"text_transcoded_latin1\": " << transcoded << ",\n  \"by_record_type\": " << counts(byType)
          << ",\n  \"by_state\": " << counts(byState) << ",\n  \"by_category\": {";
        {
            bool first = true;
            for (const auto& [k, v] : byCategory) {
                o << (first ? "" : ", ") << "\"" << k << "\": " << v;
                first = false;
            }
        }
        o << "},\n  \"blank_255\": {";
        for (int k = 0; k < 6; ++k) o << (k ? ", " : "") << "\"" << kCodeCols[k] << "\": " << blank255[k];
        o << "}},\n"
          << " \"refused\": {\"row_width\": " << badWidth << ", \"coordinates\": {";
        for (int i = 1; i < kWhyCount; ++i) o << "\"" << kCoordWhy[i] << "\": " << coordRefused[i] << (i + 1 < kWhyCount ? ", " : "");
        o << "}, \"coordinates_by_state\": " << counts(byStateRefused) << ",\n  \"codes_unparsable\": " << refusedCode
          << ", \"counts_unparsable\": " << refusedCount << "},\n"
          << " \"items\": {\n  \"10\": " << tally(t010) << ",\n  \"39\": " << tally(t039) << ",\n  \"40\": " << tally(t040)
          << ",\n  \"53\": " << tally(t053) << ",\n  \"54B\": " << tally(t054B) << ",\n  \"116\": " << tally(t116)
          << ",\n  \"48\": " << tally(t048) << ",\n  \"49\": " << tally(t049) << ",\n  \"52\": " << tally(t052) << "},\n"
          << " \"notes\": {\"39_literal_with_38_no_control\": " << literal38NoControl39
          << ", \"40_literal_with_38_no_control\": " << literal38NoControl40
          << ", \"116_positive_not_43B_lift\": " << positive116NotLift << ", \"39_top_raw_with_38_eq_1\": [";
        for (size_t i = 0; i < top39.size(); ++i) o << (i ? ", " : "") << "[\"" << JsonEsc(top39[i].second) << "\", " << top39[i].first << "]";
        o << "], \"id_collision_origins\": [";
        for (size_t i = 0; i < dupOrigins.size(); ++i) o << (i ? ", " : "") << "\"" << JsonEsc(dupOrigins[i]) << "\"";
        o << "]},\n \"cellDeg\": 0.05,\n \"cellIndex\": \"" << kStem << ".bridges.idx\",\n \"cellCount\": " << cells.size() << "\n}\n";
    }
    printf("[nbi] %zu records in %zu cells -> %s (%.1f MB) in %.1f s\n", recs.size(), cells.size(), bin.string().c_str(),
           fs::file_size(bin) / 1048576.0, Secs());
    printf("[nbi] record types:");
    for (const auto& [k, v] : byType) printf(" %s=%llu", k.c_str(), (unsigned long long)v);
    printf("\n[nbi] category:");
    for (const auto& [k, v] : byCategory) printf(" %d=%llu", k, (unsigned long long)v);
    printf("\n[nbi] blank (255):");
    for (int k = 0; k < 6; ++k) printf(" %s=%llu", kCodeCols[k], (unsigned long long)blank255[k]);
    printf("\n[nbi] structures %llu: on+under %llu, on only %llu, under only %llu; id collisions %llu\n",
           (unsigned long long)structures, (unsigned long long)both, (unsigned long long)onOnly,
           (unsigned long long)underOnly, (unsigned long long)idDup);
    printf("[nbi] coordinates refused:");
    for (int i = 1; i < kWhyCount; ++i) printf(" %s=%llu", kCoordWhy[i], (unsigned long long)coordRefused[i]);
    printf("\n[nbi] items (blank / unparsable / ->inf / ->NaN n.a. / zero refused / near-sentinel literal):\n");
    const std::pair<const char*, const Tally*> items[] = {{"10", &t010}, {"39", &t039}, {"40", &t040}, {"53", &t053},
                                                          {"54B", &t054B}, {"116", &t116}, {"48", &t048}, {"49", &t049}, {"52", &t052}};
    for (const auto& [n, t] : items)
        printf("  %-4s %7llu %5llu %7llu %7llu %5llu %5llu\n", n, (unsigned long long)t->blank, (unsigned long long)t->refused,
               (unsigned long long)t->toInf, (unsigned long long)t->toNaN, (unsigned long long)t->zeroRefused,
               (unsigned long long)t->nearSentinel);
    printf("[nbi] 39 literal with 38 no control %llu, 40 %llu; 116 positive on a non-lift 43B %llu; codes unparsable %llu, "
           "counts unparsable %llu, text transcoded %llu\n[nbi] top raw 39 where 38 = '1':",
           (unsigned long long)literal38NoControl39, (unsigned long long)literal38NoControl40,
           (unsigned long long)positive116NotLift, (unsigned long long)refusedCode, (unsigned long long)refusedCount,
           (unsigned long long)transcoded);
    for (const auto& [n, v] : top39) printf(" '%s'=%llu", v.c_str(), (unsigned long long)n);
    printf("\n");
    return 0;
}

// ---- --probe: read the harvest back through its own index ---------------------------------------
std::string Fmt(float v) {
    if (std::isnan(v)) return "NaN";
    if (std::isinf(v)) return "inf";
    char s[32];
    snprintf(s, sizeof(s), "%.2f", v);
    return s;
}
int Probe(const fs::path& manifest, double lon, double lat, double km) {
    std::string stem = manifest.string();
    stem = stem.substr(0, stem.size() - 5);   // drop ".json"
    std::ifstream fi(stem + ".idx", std::ios::binary);
    std::ifstream fb(stem + ".bin", std::ios::binary);
    char magic[8];
    if (!fi || !fb || !fb.read(magic, 8) || std::memcmp(magic, "GABRDG01", 8) != 0) {
        fprintf(stderr, "[probe] cannot open %s.idx / .bin\n", stem.c_str());
        return 1;
    }
    const double dLat = km / 111.32, dLon = km / (111.32 * std::cos(lat * 3.14159265358979 / 180.0));
    const int64_t iy0 = (int64_t)std::floor((lat - dLat) / kCellDeg), iy1 = (int64_t)std::floor((lat + dLat) / kCellDeg);
    const int64_t ix0 = (int64_t)std::floor((lon - dLon) / kCellDeg), ix1 = (int64_t)std::floor((lon + dLon) / kCellDeg);
#pragma pack(push, 1)
    struct {
        int32_t ix, iy;
        int64_t off, n;
    } row;
    struct {
        int64_t id;
        uint8_t source, kind, category, primitive, verdat, clrUnderRef, serviceOn, serviceUnder, nbiKind, nbiType, navigation, pad;
        float vertClr, vertClrClosed, vertClrOpen, vertClrSafe, horClr, clrOverDeck, clrUnder, minVertClrRoute, lengthM, maxSpanM, deckWidthM, heightM;
        uint16_t spansMain, spansApproach, yearBuilt, nameLen, crossesLen, originLen, noteLen;
    } rec;
#pragma pack(pop)
    static_assert(sizeof(rec) == 82, "GABRDG01 record layout");
    uint64_t found = 0;
    while (fi.read(reinterpret_cast<char*>(&row), sizeof(row))) {
        if (row.iy < iy0 || row.iy > iy1 || row.ix < ix0 || row.ix > ix1) continue;
        fb.clear();
        fb.seekg(row.off);
        for (int64_t i = 0; i < row.n; ++i) {
            fb.read(reinterpret_cast<char*>(&rec), sizeof(rec));
            std::string name(rec.nameLen, '\0'), crosses(rec.crossesLen, '\0'), origin(rec.originLen, '\0');
            fb.read(name.data(), rec.nameLen);
            fb.read(crosses.data(), rec.crossesLen);
            fb.read(origin.data(), rec.originLen);
            fb.seekg(rec.noteLen, std::ios::cur);
            uint32_t parts, n;
            uint8_t outer[4];
            int32_t xy[2];
            fb.read(reinterpret_cast<char*>(&parts), 4);
            fb.read(reinterpret_cast<char*>(&n), 4);
            fb.read(reinterpret_cast<char*>(outer), 4);
            fb.read(reinterpret_cast<char*>(xy), 8);
            const double x = xy[0] * 1e-7, y = xy[1] * 1e-7;
            const double ex = (x - lon) * 111.32 * std::cos(lat * 3.14159265358979 / 180.0), ny = (y - lat) * 111.32;
            const double d = std::sqrt(ex * ex + ny * ny);
            if (d > km) continue;
            ++found;
            printf("%.3f km  %-28s | %-28s | %s  %.6f %.6f\n"
                   "          10=%s 39=%s 40=%s 53=%s 54A=%c 54B=%s 116=%s  38=%u 43A=%u 43B=%u kind=%u cat=%u year=%u len=%s span=%s\n",
                   d, name.c_str(), crosses.c_str(), origin.c_str(), y, x, Fmt(rec.minVertClrRoute).c_str(),
                   Fmt(rec.vertClr).c_str(), Fmt(rec.horClr).c_str(), Fmt(rec.clrOverDeck).c_str(),
                   rec.clrUnderRef != 255 ? rec.clrUnderRef : '-', Fmt(rec.clrUnder).c_str(), Fmt(rec.vertClrClosed).c_str(),
                   rec.navigation, rec.nbiKind, rec.nbiType, rec.kind, rec.category, rec.yearBuilt,
                   Fmt(rec.lengthM).c_str(), Fmt(rec.maxSpanM).c_str());
        }
    }
    printf("[probe] %llu records within %.2f km of %.4f, %.4f\n", (unsigned long long)found, km, lat, lon);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc == 6 && std::strcmp(argv[1], "--probe") == 0) return Probe(argv[2], std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5]));
    if (argc != 3) {
        fprintf(stderr, "usage: nbi_bridges <nbi delimited .txt> <out dir>\n"
                        "       nbi_bridges --probe <nbi-2025.bridges.json> lon lat radiusKm\n");
        return 2;
    }
    return Harvest(argv[1], argv[2]);
}
