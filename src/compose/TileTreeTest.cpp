// ================================================================================================
//  TileTreeTest - the tile tree made fit for a deep pyramid (HIERARCHY 4a), gated in --selftest.
//
//  The design puts one 25-level pyramid, Lattice::Cube(16384 << 17), where today's trees have 8
//  levels. What changed changes no tile's name and no tile's bytes, and every gate here is on
//  synthetic sources under a scratch root of the run's own (out\treetest\<run>), so no tile of the
//  suite lands in the real cache; a gate that passes removes its files by name.
//
//    A1  THE STRIPE COUNT: at most one stripe held per thread over every walk below. The walk it
//        replaced, kept behind TileTree::FoldWalkForTest, must drive it past one: CAUGHT.
//    A2  DEPTH: two dozen leaves at mip 8 of the pyramid. Every ancestor on disk is the source's
//        own paint with the fold of each child on disk in its quadrant, byte for byte against a
//        fold computed here -- in FloatW, whose folds the 4/255 rule never stops; in RGBA8 that
//        rule stops a chain where a leaf's share of a coarse texel falls under 4/255, so there a
//        child's quadrant is held to the rule's own tolerance. The cost of a chain per leaf,
//        beside the old walk's for one cold leaf.
//    A3  TODAY'S DEPTH: one sequence of paints on Lattice::Cube(16384) through the old walk and
//        the new leaves the same files with the same bytes. The one-pass walk that does not carry
//        an absent parent's own paint is measured beside them: the files it would change.
//    A4  TWELVE THREADS on the pyramid, their chains meeting, against the serial tree: no deadlock
//        (a watchdog), no lost fold. The free-running threads did not meet the one race that
//        loses a fold in six runs with its guard taken out, so it is also STAGED at the walk's
//        pause point (TileTree::FoldStepForTest): the walk it replaced loses the fold there
//        (CAUGHT), this one keeps it.
//    B   the tile-native marker walk stops at the first marked ancestor: the wave tree's prefill
//        (its frame and box, a synthetic node) leaves the same files, and the probes it saved.
//    C   Prefill's integer box is the old float box at every mip of the wave tree's call; on the
//        pyramid a box of known size visits exactly its tiles.
//    D   the stripe hash: even over a million deep addresses, and no two addresses one field apart
//        share an input word (the old packing's do: CAUGHT).
//    E   the archive packer with one loose file unreadable: every other record reads back as its
//        own bytes; the old packer serves a neighbour's: CAUGHT.
//    F   the fetch provider's refusals, from a scratch cache with the budget at zero and no
//        network ever: each missing tile counted once and remembered, and what a painted tile asks.
// ================================================================================================
#include "compose/TileTree.h"

#include "compose/ColorStackSource.h"
#include "compose/Sources.h"
#include "core/Common.h"
#include "core/TileProviders.h"
#include "sim/WaveFieldSource.h"

#include <windows.h>
#include <wincodec.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace ga {

namespace {

bool g_ok = true;
bool Check(bool cond, const char* what) {
    if (!cond) {
        Log("[treetest]   FAIL: %s", what);
        g_ok = false;
    }
    return cond;
}

constexpr double kR2D = 180.0 / 3.14159265358979;   // TileTree's own constant

uint64_t Mix64(uint64_t v) {   // splitmix64's finalizer
    v ^= v >> 30;
    v *= 0xBF58476D1CE4E5B9ull;
    v ^= v >> 27;
    v *= 0x94D049BB133111EBull;
    v ^= v >> 31;
    return v;
}
uint64_t Bits(double d) {
    uint64_t u;
    std::memcpy(&u, &d, sizeof(u));
    return u;
}
uint64_t HashAt(double a, double b, double c) { return Mix64(Bits(a) ^ Mix64(Bits(b) ^ Mix64(Bits(c)))); }

// ---- the scratch roots ------------------------------------------------------------------------
const std::string& RunDir() {
    static const std::string d = [] {
        char b[64];
        snprintf(b, sizeof(b), "out\\treetest\\%lu_%llu", GetCurrentProcessId(),
                 static_cast<unsigned long long>(GetTickCount64()));
        return std::string(b);
    }();
    return d;
}
std::string Scratch(const char* gate) {
    const std::string r = RunDir() + "\\" + gate;
    tree_detail::MakeDirs(r);
    return r;
}
// Removes what a gate wrote: the files IN each folder named (never below it), then the folder,
// in the order given. Only under this run's folder; anything else is refused and said.
void RemoveFolders(const std::vector<std::string>& dirs) {
    for (const std::string& d : dirs) {
        if (d.compare(0, RunDir().size(), RunDir()) != 0) {
            Log("[treetest]   refused to remove %s: not under %s", d.c_str(), RunDir().c_str());
            continue;
        }
        WIN32_FIND_DATAA fd{};
        HANDLE h = FindFirstFileA((d + "\\*").c_str(), &fd);
        if (h != INVALID_HANDLE_VALUE) {
            do {
                if (fd.dwFileAttributes & (FILE_ATTRIBUTE_DIRECTORY | FILE_ATTRIBUTE_REPARSE_POINT)) {
                    continue;
                }
                DeleteFileA((d + "\\" + fd.cFileName).c_str());
            } while (FindNextFileA(h, &fd));
            FindClose(h);
        }
        RemoveDirectoryA(d.c_str());
    }
}
// A tree's frame folder, its node folder, its archive, and the gate root above them.
void RemoveTree(const TileTree& t, const std::string& tag, const std::string& root) {
    DeleteFileA((t.Folder() + "\\" + tag + ".gaa").c_str());
    RemoveFolders({t.Folder() + "\\" + tag, t.Folder(), root});
}

bool ReadAll(const std::string& path, std::vector<uint8_t>& out) {
    out.clear();
    FILE* f = nullptr;
    if (fopen_s(&f, path.c_str(), "rb") != 0 || !f) return false;
    _fseeki64(f, 0, SEEK_END);
    const long long n = _ftelli64(f);
    _fseeki64(f, 0, SEEK_SET);
    out.resize(size_t(n));
    const size_t got = n ? fread(out.data(), 1, out.size(), f) : 0;
    fclose(f);
    return got == out.size();
}
// The names in one folder (files only). The folder's stamp (tree_detail::StampLive, ".live") is
// not among them: it says when a run used the folder, not what the tree holds, and it differs
// every run.
std::vector<std::string> Names(const std::string& dir) {
    std::vector<std::string> out;
    WIN32_FIND_DATAA fd{};
    HANDLE h = FindFirstFileA((dir + "\\*").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) return out;
    do {
        if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
        if (fd.cFileName[0] == '.') continue;
        out.push_back(fd.cFileName);
    } while (FindNextFileA(h, &fd));
    FindClose(h);
    std::sort(out.begin(), out.end());
    return out;
}
bool ParseTile(const std::string& name, TileRequest& r, std::string& ext) {
    unsigned f = 0, m = 0, x = 0, y = 0;
    if (sscanf_s(name.c_str(), "f%u_m%u_x%u_y%u", &f, &m, &x, &y) != 4) return false;
    r = TileRequest{f, m, x, y};
    const size_t dot = name.find('.');
    ext = dot == std::string::npos ? std::string() : name.substr(dot);
    return true;
}
std::string TileName(const TileRequest& r, const char* ext) {
    char b[96];
    snprintf(b, sizeof(b), "f%u_m%u_x%u_y%u%s", r.face, r.mip, r.x, r.y, ext);
    return b;
}
// Two folders, file for file: names in one only, and names in both whose bytes differ.
struct Diff {
    std::vector<std::string> onlyA, onlyB, differ;
    size_t same = 0;
    bool Equal() const { return onlyA.empty() && onlyB.empty() && differ.empty(); }
};
Diff CompareFolders(const std::string& a, const std::string& b) {
    Diff d;
    const std::vector<std::string> na = Names(a), nb = Names(b);
    std::set<std::string> sb(nb.begin(), nb.end());
    for (const std::string& n : na) {
        if (!sb.count(n)) {
            d.onlyA.push_back(n);
            continue;
        }
        sb.erase(n);
        std::vector<uint8_t> ba, bb;
        ReadAll(a + "\\" + n, ba);
        ReadAll(b + "\\" + n, bb);
        if (ba == bb) ++d.same;
        else d.differ.push_back(n);
    }
    d.onlyB.assign(sb.begin(), sb.end());
    return d;
}
std::string FirstFew(const std::vector<std::string>& v, size_t k = 4) {
    std::string s;
    for (size_t i = 0; i < v.size() && i < k; ++i) s += (i ? ", " : "") + v[i];
    if (v.size() > k) s += ", ...";
    return s;
}

// ---- synthetic sources ------------------------------------------------------------------------

// Full cover, and every texel a hash of where it is and at what grain: no level's own paint
// resembles the fold of the level below, so the 4/255 rule never stops a fold of it.
class HashColor : public ColorSource {
public:
    HashColor() {
        m_info = {"selftest.hashcolor", "analytic hash texture", "EPSG:4326", 1, -180, -90, 180, 90};
    }
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double lat, double lon, double groundResM, const PaintCtx&,
                 uint8_t rgba[4]) override {
        const uint64_t h = HashAt(lat, lon, groundResM);
        rgba[0] = uint8_t(h);
        rgba[1] = uint8_t(h >> 8);
        rgba[2] = uint8_t(h >> 16);
        rgba[3] = 255;
        return 1.0f;
    }
    SourceInfo m_info;
};

// The scalar twin, for FloatW trees (the height leaves' format): metres hashed the same way.
class HashScalar : public DomainSource {
public:
    HashScalar() : m_unit(UnitSpec::Parse("m")) {}
    const char* Name() const override { return "selftest.hashscalar"; }
    SourceDomain Domain() const override { return SourceDomain::Raster; }
    uint8_t GradeSig() const override { return 1u; }
    uint32_t Channels() const override { return 1; }
    const UnitSpec& Unit() const override { return m_unit; }
    std::string Identity() const override { return "selftest.hashscalar|v1"; }
    bool SampleAt(const DomainQuery& q, DomainValue& out) const override {
        out.c[0] = float(HashAt(q.lat, q.lon, q.groundM) % 1000000u) * 0.001f;
        out.weight = 1.0f;
        return true;
    }

private:
    UnitSpec m_unit;
};

// A region for today's depth. A feathered disc of cover (void tiles outside it, partial ones on
// its edge, RGB under zero alpha where the paint leaves it), a gradient under a hashed grain that
// is coarse west of the centre and fine east of it (so the 4/255 rule stops some folds and passes
// others), and a band it cannot answer at coarse grains, east of the centre and south (Transient:
// never cached, and a chain that meets it stops).
class PatchColor : public ColorSource {
public:
    static constexpr double kLat0 = -0.02, kLon0 = -0.02, kR = 0.035, kF = 0.012;   // radians
    PatchColor() {
        m_info = {"selftest.patchcolor", "analytic feathered disc", "EPSG:4326", 1, -4, -4, 2, 2};
    }
    const SourceInfo& Info() const override { return m_info; }
    float Sample(double lat, double lon, double groundResM, const PaintCtx&,
                 uint8_t rgba[4]) override {
        if (groundResM > 5000.0 && lat < -0.045 && lon > 0.0) return -1.0f;
        const double d = std::hypot(lat - kLat0, lon - kLon0);
        const double t = std::clamp((kR - d) / kF, 0.0, 1.0);
        const float w = float(t * t * (3.0 - 2.0 * t));
        if (w <= 0.0f) return 0.0f;
        const int amp = lon < kLon0 ? 9 : 2;
        const int n = int(HashAt(lat, lon, groundResM) % uint64_t(2 * amp + 1)) - amp;
        rgba[0] = uint8_t(std::clamp(int(100 + 1500 * lat) + n, 0, 255));
        rgba[1] = uint8_t(std::clamp(int(120 + 1200 * lon) + n, 0, 255));
        rgba[2] = uint8_t(std::clamp(90 + n, 0, 255));
        rgba[3] = 255;
        return w;
    }
    SourceInfo m_info;
};

// A tile-native node on the wave tree's own frame, painting planes as WaveFieldSource does: a
// texel is the box mean of the solve cells it covers, weighted by how many of them the window
// holds. Each plane has its own land, cells of all zeros, which the tree stores as no cover.
class NativeWindow : public DomainSource {
public:
    explicit NativeWindow(const WaveFieldSource::Frame& f)
        : m_f(f), m_unit(UnitSpec::Parse("fraction")) {}
    const char* Name() const override { return "selftest.native"; }
    SourceDomain Domain() const override { return SourceDomain::Raster; }
    uint8_t GradeSig() const override { return 1u; }
    uint32_t Channels() const override { return 4; }
    const UnitSpec& Unit() const override { return m_unit; }
    bool TileNative() const override { return true; }
    std::string Identity() const override { return "selftest.native|v1"; }
    bool SampleAt(const DomainQuery&, DomainValue& out) const override {
        out.weight = 0.0f;
        return false;
    }
    bool PaintTile(const ColorFrame& frame, const TileRequest& r, uint32_t texW, uint32_t texH,
                   std::vector<DomainValue>& vals) const override {
        const long long nx = m_f.nx, ny = m_f.ny, step = 1ll << r.mip;
        const long long X = (frame.orgPxX >> r.mip) + static_cast<long long>(r.x) * texW;
        const long long Y = (frame.orgPxY >> r.mip) + static_cast<long long>(r.y) * texH;
        vals.assign(size_t(texW) * texH, DomainValue{});
        for (uint32_t py = 0; py < texH; ++py) {
            for (uint32_t px = 0; px < texW; ++px) {
                const long long cx0 = ((X + px) << r.mip) - m_f.winPxX;
                const long long cy0 = ((Y + py) << r.mip) - m_f.winPxY;
                double acc[4] = {0, 0, 0, 0};
                uint32_t n = 0;
                for (long long by = 0; by < step; ++by) {
                    const long long j = cy0 + by;
                    if (j < 0 || j >= ny) continue;
                    for (long long bx = 0; bx < step; ++bx) {
                        const long long i = cx0 + bx;
                        if (i < 0 || i >= nx) continue;
                        uint8_t c[4];
                        Cell(r.face, i, j, c);
                        for (int q = 0; q < 4; ++q) acc[q] += c[q];
                        ++n;
                    }
                }
                if (!n) continue;
                DomainValue& v = vals[size_t(py) * texW + px];
                for (int q = 0; q < 4; ++q) v.c[q] = float(acc[q] / (255.0 * n));
                v.weight = float(double(n) / double(step * step));
            }
        }
        return true;
    }

private:
    void Cell(uint32_t plane, long long i, long long j, uint8_t c[4]) const {
        const double x = double(i), y = double(j), nx = double(m_f.nx), ny = double(m_f.ny);
        bool land = false;
        if (plane == 0) land = (x - 0.4 * nx) * (x - 0.4 * nx) + (y - 0.5 * ny) * (y - 0.5 * ny) < (0.3 * ny) * (0.3 * ny);
        if (plane == 1) land = x < 0.3 * nx;
        if (land) {
            c[0] = c[1] = c[2] = c[3] = 0;
            return;
        }
        const uint64_t h = Mix64((uint64_t(plane) << 48) ^ (uint64_t(i) << 24) ^ uint64_t(j));
        for (int q = 0; q < 4; ++q) c[q] = uint8_t(1 + ((h >> (8 * q)) % 254));
    }
    WaveFieldSource::Frame m_f;
    UnitSpec m_unit;
};

// A tile-native node on the cube: every texel covered, a hash of its address.
class NativeCube : public DomainSource {
public:
    NativeCube() : m_unit(UnitSpec::Parse("fraction")) {}
    const char* Name() const override { return "selftest.nativecube"; }
    SourceDomain Domain() const override { return SourceDomain::Raster; }
    uint8_t GradeSig() const override { return 1u; }
    uint32_t Channels() const override { return 4; }
    const UnitSpec& Unit() const override { return m_unit; }
    bool TileNative() const override { return true; }
    std::string Identity() const override { return "selftest.nativecube|v1"; }
    bool SampleAt(const DomainQuery&, DomainValue& out) const override {
        out.weight = 0.0f;
        return false;
    }
    bool PaintTile(const ColorFrame&, const TileRequest& r, uint32_t texW, uint32_t texH,
                   std::vector<DomainValue>& vals) const override {
        vals.assign(size_t(texW) * texH, DomainValue{});
        for (size_t i = 0; i < vals.size(); ++i) {
            const uint64_t h = Mix64((uint64_t(r.face) << 58) ^ (uint64_t(r.mip) << 52) ^
                                     (uint64_t(r.x) << 26) ^ uint64_t(r.y) ^ (uint64_t(i) << 40));
            for (int q = 0; q < 4; ++q) vals[i].c[q] = float(1 + ((h >> (8 * q)) % 254)) / 255.0f;
            vals[i].weight = 1.0f;
        }
        return true;
    }

private:
    UnitSpec m_unit;
};

// ---- the fold law and the source's own paint, computed here and not by the tree -------------
// TileTree::FoldQuadrant's law, as ALGEBRA states it: the coverage-weighted mean of the four,
// the coverage their mean; RGBA8 keeps the parent's RGB where the four carry no cover.
void RefFold(TileTree::Fmt fmt, const Lattice& L, const TileRequest& c,
             const std::vector<uint8_t>& child, std::vector<uint8_t>& parent) {
    const uint32_t W = L.texW, H = L.texH, hw = W / 2, hh = H / 2;
    const uint32_t ox = (c.x & 1u) * hw, oy = (c.y & 1u) * hh;
    for (uint32_t py = 0; py < hh; ++py) {
        for (uint32_t px = 0; px < hw; ++px) {
            if (fmt == TileTree::Fmt::FloatW) {
                const float* s = reinterpret_cast<const float*>(child.data());
                float* d = reinterpret_cast<float*>(parent.data());
                double vw = 0.0, ws = 0.0;
                for (uint32_t k = 0; k < 4; ++k) {
                    const size_t i = (size_t(2 * py + (k >> 1)) * W + (2 * px + (k & 1))) * 2;
                    vw += double(s[i]) * s[i + 1];
                    ws += s[i + 1];
                }
                const size_t o = (size_t(oy + py) * W + (ox + px)) * 2;
                d[o] = ws > 0.0 ? float(vw / ws) : 0.0f;
                d[o + 1] = float(ws * 0.25);
                continue;
            }
            double rgb[3] = {0, 0, 0}, a = 0.0;
            for (uint32_t k = 0; k < 4; ++k) {
                const uint8_t* s = &child[(size_t(2 * py + (k >> 1)) * W + (2 * px + (k & 1))) * 4];
                const double w = s[3] / 255.0;
                for (int q = 0; q < 3; ++q) rgb[q] += s[q] * w;
                a += w;
            }
            uint8_t* d = &parent[(size_t(oy + py) * W + (ox + px)) * 4];
            if (a > 0.0) {
                for (int q = 0; q < 3; ++q) d[q] = uint8_t((std::max)(0.0, (std::min)(255.0, rgb[q] / a)));
            }
            d[3] = uint8_t((std::min)(1.0, a * 0.25) * 255.0 + 0.5);
        }
    }
}
// The source's own paint of one tile: FloatW through SampleAt, RGBA8 through Sample, both at the
// lattice's texel centres and ground resolution.
std::vector<uint8_t> OwnPaint(TileTree::Fmt fmt, const Lattice& L, const TileRequest& r,
                              const DomainSource* scalar, ColorSource* color) {
    const size_t n = size_t(L.texW) * L.texH;
    std::vector<uint8_t> out(fmt == TileTree::Fmt::FloatW ? n * 8 : n * 4, 0);
    for (uint32_t py = 0; py < L.texH; ++py) {
        for (uint32_t px = 0; px < L.texW; ++px) {
            double lat = 0, lon = 0;
            L.Texel(r, px, py, lat, lon);
            const size_t i = size_t(py) * L.texW + px;
            if (fmt == TileTree::Fmt::FloatW) {
                DomainQuery q;
                q.lon = lon * kR2D;
                q.lat = lat * kR2D;
                q.groundM = L.GroundRes(r.mip);
                DomainValue v;
                if (!scalar->SampleAt(q, v) || v.weight <= 0.0f) continue;
                float* d = reinterpret_cast<float*>(out.data());
                d[i * 2] = v.c[0];
                d[i * 2 + 1] = (std::min)(1.0f, v.weight);
                continue;
            }
            PaintCtx ctx;
            uint8_t rgba[4] = {0, 0, 0, 0};
            const float w = color->Sample(lat, lon, L.GroundRes(r.mip), ctx, rgba);
            if (w <= 0.0f) continue;
            uint8_t* d = &out[i * 4];
            d[0] = rgba[0];
            d[1] = rgba[1];
            d[2] = rgba[2];
            d[3] = uint8_t(w * 255.0f + 0.5f);
        }
    }
    return out;
}

// What one call cost this thread, in the tree's own tallies.
struct Cost {
    uint64_t reads = 0, writes = 0, probes = 0, announces = 0;
};
Cost Since(const tree_detail::IoTally& t0) {
    const tree_detail::IoTally& t = tree_detail::Io();
    return {t.reads - t0.reads, t.writes - t0.writes, t.probes - t0.probes,
            t.announces - t0.announces};
}

std::vector<TileRequest> Ancestors(const TileRequest& r, uint32_t maxMip) {
    std::vector<TileRequest> out;
    TileRequest a = r;
    while (a.mip < maxMip) {
        a = TileRequest{a.face, a.mip + 1, a.x / 2, a.y / 2};
        out.push_back(a);
    }
    return out;
}
bool Less(const TileRequest& a, const TileRequest& b) {
    if (a.face != b.face) return a.face < b.face;
    if (a.mip != b.mip) return a.mip < b.mip;
    if (a.y != b.y) return a.y < b.y;
    return a.x < b.x;
}
struct ReqLess {
    bool operator()(const TileRequest& a, const TileRequest& b) const { return Less(a, b); }
};

// Two dozen leaves at mip 8 of a pyramid lattice whose tiles are `texW` wide: a 3 x 3 block, a
// 2 x 2 beside it, a row of four, five far apart, and one on another face.
std::vector<TileRequest> DeepLeaves(uint32_t texW) {
    std::vector<TileRequest> v;
    const uint32_t bx = texW == 256 ? 20000u : 40000u, by = 40000u, f = 4;
    for (uint32_t j = 0; j < 3; ++j)
        for (uint32_t i = 0; i < 3; ++i) v.push_back({f, 8, bx + i, by + j});
    for (uint32_t j = 0; j < 2; ++j)
        for (uint32_t i = 0; i < 2; ++i) v.push_back({f, 8, bx + 5 + i, by + 1 + j});
    for (uint32_t i = 0; i < 4; ++i) v.push_back({f, 8, bx + 300 + i, by + 77});
    for (uint32_t i = 0; i < 5; ++i) v.push_back({f, 8, 1000 + 6000 * i, 2000 + 11000 * i});
    v.push_back({1, 8, 12345, 23456});
    return v;
}

// ================================================================================================
//  A1 + A2: the pyramid, one format.
// ================================================================================================
struct DepthResult {
    Cost first, oldFirst;   // one cold leaf: the new walk, and the walk it replaced
    uint64_t writesMin = ~0ull, writesMax = 0, writesSum = 0, readsSum = 0, announcesSum = 0;
    uint32_t maxHeldNew = 0, maxHeldOld = 0;
    size_t ancestors = 0, exactQuads = 0, tolQuads = 0, ownQuads = 0;
};

bool DepthGate(TileTree::Fmt fmt, DepthResult& res) {
    const bool fw = fmt == TileTree::Fmt::FloatW;
    const Lattice L = Lattice::Cube(16384u << 17, fw ? 256u : 128u, 128u);
    const std::string tag = L.Tag();
    HashScalar scalar;
    HashColor color;
    ColorLayerSource colorNode(&color);
    const DomainSource* node = fw ? static_cast<const DomainSource*>(&scalar) : &colorNode;
    const uint32_t maxMip = L.MaxMip();
    const char* fname = fw ? "floatw" : "rgba8";
    bool ok = true;

    // The walk it replaced, on one cold leaf: the planted failure (A1) and its cost.
    const std::vector<TileRequest> leaves = DeepLeaves(L.texW);
    const std::string rootOld = Scratch((std::string("depth_old_") + fname).c_str());
    const std::string rootOne = Scratch((std::string("depth_one_") + fname).c_str());
    {
        TileTree told(node, fmt, nullptr, rootOld);
        told.EnsureFrame(tag);
        tree_detail::Stripes().Reset();
        TileTree::FoldWalkForTest(TileTree::kWalkNested);
        const tree_detail::IoTally t0 = tree_detail::Io();
        std::vector<uint8_t> out;
        told.Tile(L, tag, leaves[0], out);
        res.oldFirst = Since(t0);
        TileTree::FoldWalkForTest(TileTree::kWalkOnce);
        res.maxHeldOld = tree_detail::Stripes().maxHeld.load();
        // ...and the same leaf through the new walk, beside it: the same files, byte for byte.
        TileTree tone(node, fmt, nullptr, rootOne);
        tone.EnsureFrame(tag);
        tree_detail::Stripes().Reset();
        const tree_detail::IoTally t1 = tree_detail::Io();
        tone.Tile(L, tag, leaves[0], out);
        res.first = Since(t1);
        const Diff d = CompareFolders(told.Folder() + "\\" + tag, tone.Folder() + "\\" + tag);
        ok &= Check(d.Equal(), "A2: one cold leaf on the pyramid, old walk and new, same files and bytes");
        if (!d.Equal()) {
            Log("[treetest]   %s one leaf: old only %zu (%s), new only %zu (%s), differ %zu (%s)",
                fname, d.onlyA.size(), FirstFew(d.onlyA).c_str(), d.onlyB.size(),
                FirstFew(d.onlyB).c_str(), d.differ.size(), FirstFew(d.differ).c_str());
        }
        ok &= Check(tree_detail::Stripes().maxHeld.load() <= 1 && tree_detail::Stripes().nested.load() == 0,
                    "A1: the new walk held one stripe at a time");
        if (ok) {
            RemoveTree(told, tag, rootOld);
            RemoveTree(tone, tag, rootOne);
        }
    }

    // A2: every leaf through the new walk.
    const std::string root = Scratch((std::string("depth_") + fname).c_str());
    TileTree tree(node, fmt, nullptr, root);
    tree.EnsureFrame(tag);
    tree_detail::Stripes().Reset();
    std::vector<uint8_t> out;
    for (const TileRequest& r : leaves) {
        const tree_detail::IoTally t0 = tree_detail::Io();
        const TileTree::Status st = tree.Tile(L, tag, r, out);
        const Cost c = Since(t0);
        ok &= Check(st == TileTree::Status::Content, "A2: a leaf paints");
        res.writesMin = (std::min)(res.writesMin, c.writes);
        res.writesMax = (std::max)(res.writesMax, c.writes);
        res.writesSum += c.writes;
        res.readsSum += c.reads;
        res.announcesSum += c.announces;
    }
    res.maxHeldNew = tree_detail::Stripes().maxHeld.load();
    ok &= Check(res.maxHeldNew <= 1 && tree_detail::Stripes().nested.load() == 0,
                "A1: every leaf's walk held one stripe at a time");

    // The files: exactly the leaves and all their ancestors, each as the law says.
    std::set<TileRequest, ReqLess> want;
    for (const TileRequest& r : leaves) {
        want.insert(r);
        for (const TileRequest& a : Ancestors(r, maxMip)) want.insert(a);
    }
    const std::string dir = tree.Folder() + "\\" + tag;
    std::set<TileRequest, ReqLess> have;
    for (const std::string& n : Names(dir)) {
        TileRequest r;
        std::string ext;
        if (!ParseTile(n, r, ext) || ext != ".bin") {
            ok &= Check(false, "A2: only tiles are written");
            Log("[treetest]   stray %s", n.c_str());
            continue;
        }
        have.insert(r);
    }
    ok &= Check(have.size() == want.size() && std::equal(have.begin(), have.end(), want.begin(),
                                                         [](const TileRequest& a, const TileRequest& b) {
                                                             return !Less(a, b) && !Less(b, a);
                                                         }),
                "A2: the files are the leaves and every one of their ancestors, and nothing else");
    const size_t tb = fw ? 262144u : 65536u;
    const size_t qBytes = size_t(L.texW / 2) * (fw ? 8 : 4);   // one quadrant row
    for (const TileRequest& r : have) {
        std::vector<uint8_t> disk;
        ReadAll(dir + "\\" + TileName(r, ".bin"), disk);
        const std::vector<uint8_t> own = OwnPaint(fmt, L, r, &scalar, &color);
        if (r.mip == 8) {
            ok &= Check(disk == own, "A2: a leaf is the source's own paint");
            continue;
        }
        ++res.ancestors;
        std::vector<uint8_t> expect = own;
        bool child[4] = {false, false, false, false};
        std::vector<uint8_t> kid[4];
        for (uint32_t k = 0; k < 4; ++k) {
            const TileRequest c{r.face, r.mip - 1, r.x * 2 + (k & 1), r.y * 2 + (k >> 1)};
            if (!ReadAll(dir + "\\" + TileName(c, ".bin"), kid[k]) || kid[k].size() != tb) continue;
            child[k] = true;
            RefFold(fmt, L, c, kid[k], expect);
        }
        // Per quadrant: a quadrant with no child on disk is the own paint; one with a child is
        // its fold -- exactly in FloatW, within the rule's 4/255 in RGBA8.
        for (uint32_t k = 0; k < 4; ++k) {
            const uint32_t ox = (k & 1) * (L.texW / 2), oy = (k >> 1) * (L.texH / 2);
            int worst = 0;
            bool same = true;
            for (uint32_t y = 0; y < L.texH / 2; ++y) {
                const size_t o = (size_t(oy + y) * L.texW + ox) * (fw ? 8 : 4);
                if (std::memcmp(&disk[o], &expect[o], qBytes) != 0) same = false;
                if (!fw) {
                    for (size_t b = 0; b < qBytes; ++b) worst = (std::max)(worst, std::abs(int(disk[o + b]) - int(expect[o + b])));
                }
            }
            if (!child[k]) {
                ++res.ownQuads;
                ok &= Check(same, "A2: a quadrant with no child on disk is the source's own paint");
            } else if (same) {
                ++res.exactQuads;
            } else {
                ++res.tolQuads;
                ok &= Check(!fw && worst <= 4, fw ? "A2 (FloatW): a quadrant is the exact fold of its child"
                                                  : "A2 (RGBA8): a quadrant is within 4/255 of its child's fold");
            }
        }
    }
    Log("[treetest] A2 %s: %zu leaves, %zu ancestors to mip %u; quadrants: %zu the exact fold of the "
        "child on disk, %zu within the 4/255 rule (RGBA8 only), %zu the own paint (no child); "
        "stripes held at once: %u",
        fname, leaves.size(), res.ancestors, maxMip, res.exactQuads, res.tolQuads, res.ownQuads,
        res.maxHeldNew);
    Log("[treetest] A2 %s cost, one cold leaf (%u levels above it): new walk %llu writes, %llu reads, "
        "%llu notices; the walk it replaced %llu writes, %llu reads, %llu notices, %u stripes held at once",
        fname, maxMip - 8, (unsigned long long)res.first.writes, (unsigned long long)res.first.reads,
        (unsigned long long)res.first.announces, (unsigned long long)res.oldFirst.writes,
        (unsigned long long)res.oldFirst.reads, (unsigned long long)res.oldFirst.announces,
        res.maxHeldOld);
    Log("[treetest] A2 %s cost over %zu leaves painted in turn (later ones meet warm chains): writes "
        "%llu..%llu a leaf, mean %.1f; reads mean %.1f; notices mean %.1f",
        fname, leaves.size(), (unsigned long long)res.writesMin, (unsigned long long)res.writesMax,
        double(res.writesSum) / leaves.size(), double(res.readsSum) / leaves.size(),
        double(res.announcesSum) / leaves.size());
    if (res.maxHeldOld > 1) {
        Log("[treetest] A1 CAUGHT: the walk this replaced held %u stripes at once on one cold leaf "
            "(the new walk: %u)",
            res.maxHeldOld, res.maxHeldNew);
    } else {
        ok &= Check(false, "A1: the planted failure (the old walk) must hold more than one stripe");
    }
    if (ok) RemoveTree(tree, tag, root);
    return ok;
}

// ================================================================================================
//  A3: today's depth, the old walk against the new, byte for byte.
// ================================================================================================
bool TodayGate() {
    const Lattice L = Lattice::Cube(16384);
    const std::string tag = L.Tag();
    PatchColor patch;
    ColorLayerSource node(&patch);
    // The sequence: leaves at mip 0 around the disc (siblings, cousins, its edge, void ones
    // outside it, one whose chain meets the Transient band), ancestors asked for directly before
    // their descendants (the parent painted on demand, then folded into), a leaf asked twice.
    const std::vector<TileRequest> ops = {
        {0, 0, 65, 65}, {0, 0, 64, 65}, {0, 0, 65, 64}, {0, 0, 66, 67}, {0, 0, 62, 66},
        {0, 0, 70, 65}, {0, 1, 35, 32}, {0, 0, 70, 64}, {0, 0, 67, 68}, {0, 0, 63, 65},
        {0, 3, 8, 8},   {0, 2, 15, 17}, {0, 0, 61, 69}, {0, 0, 65, 65}, {0, 0, 10, 10},
        {0, 0, 66, 64}, {0, 0, 67, 65}, {0, 0, 67, 66}, {0, 1, 33, 33}, {0, 0, 68, 66},
        {0, 0, 64, 66}, {0, 0, 63, 66}, {0, 4, 3, 4},   {0, 0, 69, 67}};
    struct Run {
        int walk;
        const char* name;
        std::string root;
        std::unique_ptr<TileTree> tree;
        uint32_t transient = 0, voids = 0;
    };
    Run runs[3] = {{TileTree::kWalkNested, "old"}, {TileTree::kWalkOnce, "new"},
                   {TileTree::kWalkChildOnly, "child-only"}};
    for (Run& run : runs) {
        run.root = Scratch((std::string("today_") + run.name).c_str());
        run.tree = std::make_unique<TileTree>(&node, TileTree::Fmt::Rgba8, nullptr, run.root);
        run.tree->EnsureFrame(tag);
        TileTree::FoldWalkForTest(run.walk);
        tree_detail::Stripes().Reset();
        std::vector<uint8_t> out;
        for (const TileRequest& r : ops) {
            const TileTree::Status st = run.tree->Tile(L, tag, r, out);
            if (st == TileTree::Status::Transient) ++run.transient;
            if (st == TileTree::Status::Void) ++run.voids;
        }
        TileTree::FoldWalkForTest(TileTree::kWalkOnce);
        if (run.walk == TileTree::kWalkOnce) {
            Check(tree_detail::Stripes().maxHeld.load() <= 1, "A1: today's depth, one stripe at a time");
        }
    }
    const std::string dOld = runs[0].tree->Folder() + "\\" + tag;
    const std::string dNew = runs[1].tree->Folder() + "\\" + tag;
    const std::string dOne = runs[2].tree->Folder() + "\\" + tag;
    const Diff d = CompareFolders(dOld, dNew);
    size_t bins = 0, voidMarks = 0;
    for (const std::string& n : Names(dOld)) {
        if (n.find(".bin") != std::string::npos) ++bins;
        if (n.find(".void") != std::string::npos) ++voidMarks;
    }
    bool ok = Check(d.Equal(), "A3: today's depth, the old walk and the new leave the same files, byte for byte");
    Log("[treetest] A3 today's depth (Cube(16384), %zu paints): old walk %zu files (%zu tiles, %zu "
        "void markers), new walk the same %zu byte for byte, %zu only old, %zu only new, %zu differ; "
        "folds the 4/255 rule stopped: old %u, new %u; accepted: old %u, new %u; paints that "
        "answered Transient %u / void %u",
        ops.size(), d.same + d.onlyA.size() + d.differ.size(), bins, voidMarks, d.same,
        d.onlyA.size(), d.onlyB.size(), d.differ.size(), runs[0].tree->redundant.load(),
        runs[1].tree->redundant.load(), runs[0].tree->folded.load(), runs[1].tree->folded.load(),
        runs[1].transient, runs[1].voids);
    if (!d.Equal()) {
        Log("[treetest]   only old: %s | only new: %s | differ: %s", FirstFew(d.onlyA).c_str(),
            FirstFew(d.onlyB).c_str(), FirstFew(d.differ).c_str());
    }
    ok &= Check(runs[1].tree->redundant.load() > 0 && runs[1].tree->folded.load() > 0 &&
                    voidMarks > 0 && runs[1].transient > 0,
                "A3: the sequence exercised redundant and accepted folds, voids and a Transient");
    // The measure the brief asked for: the one-pass walk that does NOT carry an absent parent's
    // own paint upward. Not a gate on this code -- the evidence for why the walk carries it.
    const Diff c = CompareFolders(dOld, dOne);
    size_t rgbOnly = 0;
    for (const std::string& n : c.differ) {
        std::vector<uint8_t> a, b;
        ReadAll(dOld + "\\" + n, a);
        ReadAll(dOne + "\\" + n, b);
        bool onlyUnderZeroAlpha = a.size() == b.size();
        for (size_t i = 0; onlyUnderZeroAlpha && i < a.size(); i += 4) {
            if (a[i + 3] != b[i + 3] || (a[i + 3] != 0 && std::memcmp(&a[i], &b[i], 3) != 0)) {
                onlyUnderZeroAlpha = false;
            }
        }
        if (onlyUnderZeroAlpha) ++rgbOnly;
    }
    Log("[treetest] A3 the one-pass walk WITHOUT an absent parent's own paint rising, against "
        "today's: %zu files the same, %zu that today writes and it never does (%s), %zu only it "
        "writes, %zu whose bytes differ (%zu of them only in RGB under zero alpha): %s",
        c.same, c.onlyA.size(), FirstFew(c.onlyA, 16).c_str(), c.onlyB.size(), c.differ.size(),
        rgbOnly, FirstFew(c.differ, 16).c_str());
    if (ok) {
        for (Run& run : runs) RemoveTree(*run.tree, tag, run.root);
    }
    return ok;
}

// ================================================================================================
//  A4: twelve threads, chains meeting, against the serial tree.
// ================================================================================================
bool ConcurrencyGate() {
    const Lattice L = Lattice::Cube(16384u << 17, 256u, 128u);
    const std::string tag = L.Tag();
    static const HashScalar scalar;   // static: a deadlocked tree must not outlive its node
    std::vector<TileRequest> leaves;
    for (uint32_t j = 0; j < 5; ++j)
        for (uint32_t i = 0; i < 5; ++i) leaves.push_back({2, 8, 7000 + i, 9000 + j});
    for (uint32_t i = 0; i < 11; ++i) leaves.push_back({2, 8, 7000 + 37 * i, 9000 + 53 * (i % 4)});
    const std::string rootC = Scratch("threads_concurrent"), rootS = Scratch("threads_serial");
    // Held by pointer so a deadlock can leave the tree (and the stripes its threads block on)
    // alive while the run reports and exits.
    auto tcOwned = std::make_unique<TileTree>(&scalar, TileTree::Fmt::FloatW, nullptr, rootC);
    TileTree& tc = *tcOwned;
    TileTree ts(&scalar, TileTree::Fmt::FloatW, nullptr, rootS);
    tc.EnsureFrame(tag);
    ts.EnsureFrame(tag);
    tree_detail::Stripes().Reset();
    constexpr int kThreads = 12;
    std::atomic<bool> go{false};
    std::atomic<int> done{0};
    std::vector<std::thread> pool;
    for (int t = 0; t < kThreads; ++t) {
        pool.emplace_back([&, t] {
            while (!go.load(std::memory_order_acquire)) std::this_thread::yield();
            std::vector<uint8_t> out;
            for (size_t i = size_t(t); i < leaves.size(); i += kThreads) tc.Tile(L, tag, leaves[i], out);
            done.fetch_add(1, std::memory_order_acq_rel);
        });
    }
    const auto t0 = std::chrono::steady_clock::now();
    go.store(true, std::memory_order_release);
    bool finished = false;
    while (!finished) {
        finished = done.load(std::memory_order_acquire) == kThreads;
        if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(240)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!finished) {
        Check(false, "A4: twelve threads finished within the watchdog's 240 s -- DEADLOCK");
        for (std::thread& th : pool) th.detach();   // they hold stripes forever; the run exits
        tcOwned.release();                          // ...on a tree that must outlive them
        return false;
    }
    for (std::thread& th : pool) th.join();
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const uint32_t held = tree_detail::Stripes().maxHeld.load();
    std::vector<uint8_t> out;
    for (const TileRequest& r : leaves) ts.Tile(L, tag, r, out);
    const Diff d = CompareFolders(tc.Folder() + "\\" + tag, ts.Folder() + "\\" + tag);
    bool ok = Check(d.Equal(), "A4: twelve threads leave the serial tree, file for file and byte for byte");
    ok &= Check(held <= 1, "A1: twelve threads, one stripe each at a time");
    Log("[treetest] A4 %d threads, %zu leaves on the pyramid (FloatW), chains meeting from mip 9 to "
        "the root: done in %.2f s (watchdog 240 s); against the serial tree %zu files the same, "
        "%zu only concurrent, %zu only serial, %zu differ; stripes held at once: %u",
        kThreads, leaves.size(), secs, d.same, d.onlyA.size(), d.onlyB.size(), d.differ.size(), held);
    if (!d.Equal()) {
        Log("[treetest]   differ: %s | only concurrent: %s | only serial: %s", FirstFew(d.differ).c_str(),
            FirstFew(d.onlyA).c_str(), FirstFew(d.onlyB).c_str());
    }
    if (ok) {
        RemoveTree(tc, tag, rootC);
        RemoveTree(ts, tag, rootS);
    }
    return ok;
}

// The race the free-running threads above do not reliably meet (six runs of them passed with the
// child re-read in FoldInto taken out), STAGED: thread A paints leaf a and is held at the pause
// point after its walk published a's parent P, before the level above; the main thread paints
// a's sibling b, whose walk folds b into P and carries P, now holding both, to the root; then A
// goes on with the P it wrote. Every level above P must end holding both, as the serial tree
// does. The walk this replaced carries A's P regardless and loses b above P: CAUGHT.
bool StagedRaceGate() {
    const Lattice L = Lattice::Cube(16384u << 17, 256u, 128u);
    const std::string tag = L.Tag();
    static const HashScalar scalar;   // static: a stuck walk must not outlive its node
    const TileRequest a{3, 8, 5000, 6000}, b{3, 8, 5001, 6000}, P{3, 9, 2500, 3000};
    const std::string rootR = Scratch("staged_serial");
    TileTree ref(&scalar, TileTree::Fmt::FloatW, nullptr, rootR);
    ref.EnsureFrame(tag);
    std::vector<uint8_t> out;
    ref.Tile(L, tag, a, out);
    ref.Tile(L, tag, b, out);
    const std::string dRef = ref.Folder() + "\\" + tag;
    struct Run {
        int walk;
        const char* name;
        Diff d;
        bool paused = false;
    };
    Run runs[2] = {{TileTree::kWalkNested, "old"}, {TileTree::kWalkOnce, "new"}};
    bool ok = true;
    std::vector<std::pair<std::unique_ptr<TileTree>, std::string>> trees;
    for (Run& run : runs) {
        const std::string root = Scratch((std::string("staged_") + run.name).c_str());
        trees.emplace_back(std::make_unique<TileTree>(&scalar, TileTree::Fmt::FloatW, nullptr, root), root);
        TileTree& tree = *trees.back().first;
        tree.EnsureFrame(tag);
        TileTree::FoldWalkForTest(run.walk);
        std::atomic<int> phase{0};
        std::atomic<bool> go{false};
        std::thread::id aId;
        TileTree::FoldStepForTest([&](const TileRequest& p) {
            if (p.face != P.face || p.mip != P.mip || p.x != P.x || p.y != P.y) return;
            if (std::this_thread::get_id() != aId) return;
            int expect = 0;
            if (!phase.compare_exchange_strong(expect, 1)) return;
            const auto t0 = std::chrono::steady_clock::now();
            while (phase.load() != 2 && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(60)) {
                std::this_thread::yield();
            }
        });
        std::thread ta([&] {
            while (!go.load(std::memory_order_acquire)) std::this_thread::yield();
            std::vector<uint8_t> o;
            tree.Tile(L, tag, a, o);
        });
        aId = ta.get_id();
        go.store(true, std::memory_order_release);
        const auto t0 = std::chrono::steady_clock::now();
        while (phase.load() != 1 && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(60)) {
            std::this_thread::yield();
        }
        run.paused = phase.load() == 1;
        tree.Tile(L, tag, b, out);   // the sibling's whole walk, while A is held
        phase.store(2);
        ta.join();
        TileTree::FoldStepForTest(nullptr);
        TileTree::FoldWalkForTest(TileTree::kWalkOnce);
        run.d = CompareFolders(tree.Folder() + "\\" + tag, dRef);
    }
    ok &= Check(runs[0].paused && runs[1].paused, "A4: the staged race held A at its pause point in both walks");
    ok &= Check(runs[1].d.Equal(), "A4: the staged race -- the new walk keeps the sibling's fold above P");
    Log("[treetest] A4 staged race (A held after publishing P, B's whole walk, then A): the new "
        "walk against the serial tree %zu files the same, %zu differ; the walk it replaced %zu the "
        "same, %zu differ (%s)",
        runs[1].d.same, runs[1].d.differ.size() + runs[1].d.onlyA.size() + runs[1].d.onlyB.size(),
        runs[0].d.same, runs[0].d.differ.size() + runs[0].d.onlyA.size() + runs[0].d.onlyB.size(),
        FirstFew(runs[0].d.differ, 3).c_str());
    if (!runs[0].d.Equal()) {
        Log("[treetest] A4 CAUGHT: the walk this replaced lost the sibling's fold at %zu tiles above P",
            runs[0].d.differ.size());
    } else {
        ok &= Check(false, "A4: the planted failure (the old walk under the staged race) must lose a fold");
    }
    if (ok) {
        for (auto& t : trees) RemoveTree(*t.first, tag, t.second);
        RemoveTree(ref, tag, rootR);
    }
    return ok;
}

// ================================================================================================
//  B + C: the tile-native marker walk and Prefill's box.
// ================================================================================================
bool PrefillGates() {
    bool ok = true;
    // ---- today's lattice: the wave tree's own call (FrameLoop's wave.prefill), on the shipped
    // window (WaveFieldConfig's defaults are data/wave_scene.json's), a synthetic node on it.
    WaveFieldConfig cfg;
    // The shipped window about the shipped scenes' anchor (test data, pinned at the anchor).
    const WaveFieldSource::Frame wf =
        WaveFieldSource::Align(cfg, Space::Anchor::About(42.81833, -70.81, 6371000.0));
    const WaveFieldSource shipped(nullptr, wf);
    float u0, v0, u1, v1;
    shipped.WindowUv(u0, v0, u1, v1);
    uint32_t x0, y0, x1, y1;
    shipped.WindowTiles(x0, y0, x1, y1);
    const Lattice& L = wf.color;
    const uint32_t maxMip = L.MaxMip();
    // C: the float box the old Prefill computed at each mip, beside the integer one it takes now.
    uint32_t boxesSame = 0;
    for (uint32_t m = 0; m <= maxMip; ++m) {
        const uint32_t dim = L.faceDim >> m;
        const uint32_t tw = (std::max)(1u, dim / L.texW), th = (std::max)(1u, dim / L.texH);
        const uint32_t ox0 = uint32_t((std::max)(0.0f, u0) * tw), oy0 = uint32_t((std::max)(0.0f, v0) * th);
        const uint32_t ox1 = (std::min)(tw - 1, uint32_t((std::min)(0.9999f, u1) * tw));
        const uint32_t oy1 = (std::min)(th - 1, uint32_t((std::min)(0.9999f, v1) * th));
        const uint32_t nx0 = x0 >> m, ny0 = y0 >> m;
        const uint32_t nx1 = (std::min)(tw - 1, x1 >> m), ny1 = (std::min)(th - 1, y1 >> m);
        if (ox0 == nx0 && oy0 == ny0 && ox1 == nx1 && oy1 == ny1) ++boxesSame;
        else Log("[treetest]   mip %u: float box [%u..%u]x[%u..%u], integer [%u..%u]x[%u..%u]", m, ox0, ox1, oy0, oy1, nx0, nx1, ny0, ny1);
    }
    ok &= Check(boxesSame == maxMip + 1, "C: the integer box is the old float box at every mip of the wave tree's call");
    Log("[treetest] C the wave tree's call: window %ux%u cells at z16 px (%lld, %lld), frame origin "
        "(%lld, %lld); uv box (%.6f, %.6f)-(%.6f, %.6f) = mip-0 tiles [%u..%u] x [%u..%u] (closed: the "
        "tile starting on the far edge is in it, as it was); the two boxes agree at %u of %u mips",
        wf.nx, wf.ny, wf.winPxX, wf.winPxY, wf.orgPxX, wf.orgPxY, u0, v0, u1, v1, x0, x1, y0, y1,
        boxesSame, maxMip + 1);
    // B: the same prefill through the old marker walk and the new one.
    NativeWindow native(wf);
    const uint32_t planes = 3;
    const std::string tag = L.Tag();
    struct Run {
        int walk;
        const char* name;
        std::string root;
        std::unique_ptr<TileTree> tree;
        uint32_t n = 0;
        uint64_t probes = 0;
    };
    Run runs[2] = {{TileTree::kWalkNested, "old"}, {TileTree::kWalkOnce, "new"}};
    for (Run& run : runs) {
        run.root = Scratch((std::string("prefill_") + run.name).c_str());
        run.tree = std::make_unique<TileTree>(&native, TileTree::Fmt::Raw4, nullptr, run.root);
        TileTree::FoldWalkForTest(run.walk);
        tree_detail::Stripes().Reset();
        const tree_detail::IoTally t0 = tree_detail::Io();
        run.n = run.tree->Prefill(L, 0u, planes, 0u, x0, y0, x1, y1);
        run.probes = Since(t0).probes;
        TileTree::FoldWalkForTest(TileTree::kWalkOnce);
        if (run.walk == TileTree::kWalkOnce) {
            ok &= Check(tree_detail::Stripes().maxHeld.load() <= 1, "A1: the marker walk, one stripe at a time");
        }
    }
    const std::string dOld = runs[0].tree->Folder() + "\\" + tag, dNew = runs[1].tree->Folder() + "\\" + tag;
    const Diff d = CompareFolders(dOld, dNew);
    size_t bins = 0, folds = 0, voids = 0;
    for (const std::string& n : Names(dNew)) {
        if (n.find(".bin") != std::string::npos) ++bins;
        if (n.find(".fold") != std::string::npos) ++folds;
        if (n.find(".void") != std::string::npos) ++voids;
    }
    ok &= Check(d.Equal() && runs[0].n == runs[1].n,
                "B: the wave tree's prefill leaves the same files through the old marker walk and the new");
    Log("[treetest] B the wave tree's prefill (%u planes, mips 0..%u): %u tiles realized by each; "
        "%zu files the same (%zu tiles, %zu .fold, %zu .void), %zu only old, %zu only new, %zu "
        "differ; existence probes: old walk %llu, new %llu -- %llu saved (%.0f%%)",
        planes, maxMip, runs[1].n, d.same, bins, folds, voids, d.onlyA.size(), d.onlyB.size(),
        d.differ.size(), (unsigned long long)runs[0].probes, (unsigned long long)runs[1].probes,
        (unsigned long long)(runs[0].probes - runs[1].probes),
        runs[0].probes ? 100.0 * double(runs[0].probes - runs[1].probes) / double(runs[0].probes) : 0.0);
    if (!d.Equal()) {
        Log("[treetest]   only old: %s | only new: %s | differ: %s", FirstFew(d.onlyA).c_str(),
            FirstFew(d.onlyB).c_str(), FirstFew(d.differ).c_str());
    }
    if (ok) {
        for (Run& run : runs) RemoveTree(*run.tree, tag, run.root);
    }

    // ---- C on the pyramid: a box of known size at mip 8 visits exactly its image at every mip.
    {
        const Lattice D = Lattice::Cube(16384u << 17);
        const std::string dtag = D.Tag();
        NativeCube cube;
        const std::string root = Scratch("prefill_deep");
        TileTree tree(&cube, TileTree::Fmt::Raw4, nullptr, root);
        const uint32_t bx0 = 3000, by0 = 7000, bx1 = 3009, by1 = 7006, face = 1, finest = 8;
        const uint32_t n = tree.Prefill(D, face, face + 1, finest, bx0, by0, bx1, by1);
        std::set<std::string> want;
        uint32_t expect = 0;
        std::string perMip;
        for (uint32_t m = finest; m <= D.MaxMip(); ++m) {
            const uint32_t k = m - finest, cnt = ((bx1 >> k) - (bx0 >> k) + 1) * ((by1 >> k) - (by0 >> k) + 1);
            expect += cnt;
            perMip += (m == finest ? "" : " ") + std::to_string(cnt);
            for (uint32_t y = by0 >> k; y <= (by1 >> k); ++y) {
                for (uint32_t x = bx0 >> k; x <= (bx1 >> k); ++x) {
                    want.insert(TileName({face, m, x, y}, m == finest ? ".bin" : ".fold"));
                }
            }
        }
        const std::vector<std::string> names = Names(tree.Folder() + "\\" + dtag);
        const std::set<std::string> got(names.begin(), names.end());
        const bool okD = Check(n == expect && got == want,
                               "C: on the pyramid a box of known size visits exactly its tiles at every mip");
        ok &= okD;
        Log("[treetest] C the pyramid (Cube(2^31), mips %u..%u): box [%u..%u] x [%u..%u] at mip %u; "
            "tiles a mip %s = %u expected, %u realized, %zu files (%zu expected, %s)",
            finest, D.MaxMip(), bx0, bx1, by0, by1, finest, perMip.c_str(), expect, n, got.size(),
            want.size(), got == want ? "the same names" : "DIFFERENT names");
        if (okD) RemoveTree(tree, dtag, root);
    }
    return ok;
}

// ================================================================================================
//  D: the stripe hash.
// ================================================================================================
bool StripeGate() {
    bool ok = true;
    const std::string tag = Lattice::Cube(16384u << 17).Tag();
    const int node = 0;   // any stable node address
    std::mt19937_64 rng(20260928);
    auto oldWord = [](const TileRequest& r) {
        return (uint64_t(r.face) << 44) | (uint64_t(r.mip) << 40) | (uint64_t(r.y) << 20) | uint64_t(r.x);
    };
    auto oldStripe = [&](const TileRequest& r) {
        uint64_t h = tree_detail::Fnv1a(1469598103934665603ull, tag);
        const uint64_t v1 = reinterpret_cast<uintptr_t>(&node), v2 = oldWord(r);
        for (const uint64_t v : {v1, v2}) {
            for (int i = 0; i < 8; ++i) {
                h ^= static_cast<uint8_t>(v >> (i * 8));
                h *= 1099511628211ull;
            }
        }
        return size_t(h % kStripes);
    };
    std::vector<uint32_t> hitNew(kStripes, 0), hitOld(kStripes, 0);
    // Deep: mips 0..8, where a face is 2^24 .. 2^16 tiles a side -- a million draws are a million
    // distinct addresses. (Drawn over all 25 mips, a 25th of them fall on mip 24's six tiles and
    // pile onto six stripes whatever the hash: that measures the draw, not the hash.)
    auto randomDeep = [&]() {
        TileRequest r;
        r.face = uint32_t(rng() % 6);
        r.mip = uint32_t(rng() % 9);
        const uint32_t tiles = 1u << (24 - r.mip);
        r.x = uint32_t(rng() % tiles);
        r.y = uint32_t(rng() % tiles);
        return r;
    };
    constexpr int kN = 1000000;
    for (int i = 0; i < kN; ++i) {
        const TileRequest r = randomDeep();
        ++hitNew[TileTree::StripeOf(tag, &node, r)];
        ++hitOld[oldStripe(r)];
    }
    const auto mmN = std::minmax_element(hitNew.begin(), hitNew.end());
    const auto mmO = std::minmax_element(hitOld.begin(), hitOld.end());
    const double mean = double(kN) / double(kStripes);
    ok &= Check(*mmN.first > 0.9 * mean && *mmN.second < 1.1 * mean,
                "D: a million deep addresses use the 256 stripes evenly (within 10% of the mean)");
    // One field apart: flip a field so the old word's OR hides it (x's bit 20 under y's bit 0,
    // y's bit 0 under x's bit 20, mip's bit 0 under y's bit 20, face's bit 0 under mip's bit 4).
    uint32_t oldSame[4] = {0, 0, 0, 0}, newSame[4] = {0, 0, 0, 0}, pairs[4] = {0, 0, 0, 0};
    for (int i = 0; i < 100000; ++i) {
        TileRequest a = randomDeep();
        a.mip = uint32_t(rng() % 5);   // x and y reach 2^20 only at mips 0..4
        const uint32_t tiles = 1u << (24 - a.mip);
        a.x = uint32_t(rng() % tiles);
        a.y = uint32_t(rng() % tiles);
        for (int f = 0; f < 4; ++f) {
            TileRequest b = a, c = a;
            if (f == 0) { c.y |= 1u; b.y |= 1u; if (tiles <= (1u << 20)) continue; b.x &= ~(1u << 20); c.x |= (1u << 20); }
            if (f == 1) { if (tiles <= (1u << 20)) continue; b.x |= (1u << 20); c.x |= (1u << 20); b.y &= ~1u; c.y |= 1u; }
            if (f == 2) { if (tiles <= (1u << 20)) continue; b.y |= (1u << 20); c.y |= (1u << 20); b.mip &= ~1u; c.mip = b.mip | 1u; }
            if (f == 3) { b.mip = c.mip = 16u + (a.mip & 7u); b.x &= 0xFFu; c.x = b.x; b.y &= 0xFFu; c.y = b.y; b.face &= ~1u; c.face = b.face | 1u; }
            ++pairs[f];
            uint64_t wb[4], wc[4];
            TileTree::StripeWords(b, wb);
            TileTree::StripeWords(c, wc);
            if (oldWord(b) == oldWord(c)) ++oldSame[f];
            if (std::memcmp(wb, wc, sizeof(wb)) == 0) ++newSame[f];
        }
    }
    const char* field[4] = {"x", "y", "mip", "face"};
    std::string oldLine, newLine;
    uint32_t oldTotal = 0, newTotal = 0;
    for (int f = 0; f < 4; ++f) {
        oldLine += std::string(f ? ", " : "") + field[f] + " " + std::to_string(oldSame[f]) + "/" + std::to_string(pairs[f]);
        newLine += std::string(f ? ", " : "") + field[f] + " " + std::to_string(newSame[f]);
        oldTotal += oldSame[f];
        newTotal += newSame[f];
    }
    ok &= Check(newTotal == 0, "D: no two addresses one field apart hash from the same input words");
    Log("[treetest] D a million deep addresses (mips 0..8, x and y to 2^24): stripe use %u..%u "
        "(mean %.1f); the old packed word %u..%u",
        *mmN.first, *mmN.second, mean, *mmO.first, *mmO.second);
    Log("[treetest] D addresses one field apart sharing their input: fields as words %s; the old "
        "packed word %s",
        newLine.c_str(), oldLine.c_str());
    if (oldTotal > 0) {
        Log("[treetest] D CAUGHT: the old packing gives %u pairs of addresses one field apart one "
            "input word (so one stripe, always)",
            oldTotal);
    } else {
        ok &= Check(false, "D: the planted failure (the old packing) must collide");
    }
    return ok;
}

// ================================================================================================
//  E: the archive packer.
// ================================================================================================
// The packer as it was (TileArchive::PackDir before this change), verbatim but for its name and
// the superseded count kept local: offsets assigned before the payloads are read.
uint32_t PackDirBefore(const std::string& dir, const std::string& out) {
    using Rec = TileArchive::Rec;
    std::vector<Rec> recs;
    std::vector<std::string> names;
    std::vector<uint64_t> stamps;
    WIN32_FIND_DATAA fd{};
    HANDLE h = FindFirstFileA((dir + "\\*.bin").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) return 0;
    do {
        uint32_t f = 0, m = 0, x = 0, y = 0, sub = 0;
        if (sscanf_s(fd.cFileName, "f%u_m%u_x%u_y%u_%8x.bin", &f, &m, &x, &y, &sub) != 5) {
            sub = 0;
            if (sscanf_s(fd.cFileName, "f%u_m%u_x%u_y%u.bin", &f, &m, &x, &y) != 4) continue;
        }
        Rec r;
        r.key = (uint64_t(f) << 61) | (uint64_t(m) << 56) | (uint64_t(y & 0xFFFFFFFull) << 28) |
                uint64_t(x & 0xFFFFFFFull);
        r.size = fd.nFileSizeLow;
        r.subset = sub;
        recs.push_back(r);
        names.push_back(fd.cFileName);
        stamps.push_back((uint64_t(fd.ftLastWriteTime.dwHighDateTime) << 32) |
                         fd.ftLastWriteTime.dwLowDateTime);
    } while (FindNextFileA(h, &fd));
    FindClose(h);
    if (recs.empty()) return 0;
    std::vector<uint32_t> order(recs.size());
    for (uint32_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
        if (recs[a].key != recs[b].key) return recs[a].key < recs[b].key;
        return recs[a].subset < recs[b].subset;
    });
    {
        std::vector<uint32_t> keep;
        for (size_t i = 0; i < order.size();) {
            size_t j = i, best = i;
            while (j < order.size() && recs[order[j]].key == recs[order[i]].key) {
                if (stamps[order[j]] > stamps[order[best]]) best = j;
                ++j;
            }
            keep.push_back(order[best]);
            i = j;
        }
        order.swap(keep);
    }
    FILE* fo = nullptr;
    if (fopen_s(&fo, out.c_str(), "wb") != 0 || !fo) return 0;
    const uint32_t n = uint32_t(order.size());
    const uint64_t dirBytes = uint64_t(n) * sizeof(Rec);
    const uint64_t payload0 = ((16 + dirBytes) + 65535ull) & ~65535ull;
    uint64_t off = payload0;
    std::vector<Rec> sorted;
    for (uint32_t i : order) {
        Rec r = recs[i];
        r.offset = off;
        off += r.size;
        sorted.push_back(r);
    }
    const uint32_t hdr[4] = {TileArchive::kMagic, TileArchive::kVersion, n, 0u};
    fwrite(hdr, sizeof(hdr), 1, fo);
    fwrite(sorted.data(), sizeof(Rec), n, fo);
    {
        const std::vector<uint8_t> pad(static_cast<size_t>(payload0 - (16 + dirBytes)), 0);
        if (!pad.empty()) fwrite(pad.data(), 1, pad.size(), fo);
    }
    std::vector<uint8_t> buf;
    uint32_t wrote = 0;
    for (uint32_t k = 0; k < n; ++k) {
        const std::string src = dir + "\\" + names[order[k]];
        FILE* fi = nullptr;
        if (fopen_s(&fi, src.c_str(), "rb") != 0 || !fi) continue;
        buf.resize(sorted[k].size);
        const size_t got = fread(buf.data(), 1, buf.size(), fi);
        fclose(fi);
        if (got != buf.size()) continue;
        fwrite(buf.data(), 1, buf.size(), fo);
        ++wrote;
    }
    fclose(fo);
    return wrote;
}

bool ArchiveGate() {
    const std::string root = Scratch("archive");
    const std::string dir = root + "\\tiles";
    tree_detail::MakeDir(dir);
    constexpr uint32_t kTiles = 8, kBad = 3;
    std::vector<std::vector<uint8_t>> body(kTiles);
    for (uint32_t i = 0; i < kTiles; ++i) {
        body[i].resize(65536);
        for (size_t k = 0; k < body[i].size(); ++k) body[i][k] = uint8_t(i * 37 + k * 7 + (k >> 9));
        tree_detail::WriteTile(dir + "\\" + TileName({0, 0, i, 5}, ".bin"), body[i]);
    }
    // The unreadable one: held open by this test with no sharing, so the packer's open fails.
    const std::string badPath = dir + "\\" + TileName({0, 0, kBad, 5}, ".bin");
    const HANDLE hold = CreateFileA(badPath.c_str(), GENERIC_READ, 0, nullptr, OPEN_EXISTING,
                                    FILE_ATTRIBUTE_NORMAL, nullptr);
    bool ok = Check(hold != INVALID_HANDLE_VALUE, "E: the test holds one loose tile open exclusively");
    const std::string arcNew = root + "\\new.gaa", arcOld = root + "\\old.gaa";
    const uint32_t wroteNew = TileArchive::PackDir(dir, arcNew);
    const uint32_t wroteOld = PackDirBefore(dir, arcOld);
    if (hold != INVALID_HANDLE_VALUE) CloseHandle(hold);
    auto readBack = [&](const std::string& path, uint32_t& own, uint32_t& wrong, uint32_t& missing,
                        size_t& count) {
        TileArchive a;
        own = wrong = missing = 0;
        count = 0;
        if (!a.OpenPath(path)) return;
        count = a.Count();
        for (uint32_t i = 0; i < kTiles; ++i) {
            const uint64_t key = (uint64_t(0) << 61) | (uint64_t(0) << 56) | (uint64_t(5) << 28) | uint64_t(i);
            const TileArchive::Rec* rec = a.Find(key, 0u);
            std::vector<uint8_t> got;
            if (!rec || !a.ReadPayload(*rec, got)) {
                ++missing;
                continue;
            }
            if (got == body[i]) ++own;
            else ++wrong;
        }
    };
    uint32_t ownN, wrongN, missN, ownO, wrongO, missO;
    size_t countN, countO;
    readBack(arcNew, ownN, wrongN, missN, countN);
    readBack(arcOld, ownO, wrongO, missO, countO);
    ok &= Check(wroteNew == kTiles - 1 && countN == kTiles - 1 && ownN == kTiles - 1 && wrongN == 0 &&
                    missN == 1,
                "E: with one loose tile unreadable, every other record reads back as its own bytes");
    Log("[treetest] E %u loose tiles, the fourth held open by the test: the packer wrote %u, its "
        "header counts %zu, %u read back as their own bytes, %u as another's, %u absent (the one held)",
        kTiles, wroteNew, countN, ownN, wrongN, missN);
    if (wrongO > 0) {
        Log("[treetest] E CAUGHT: the old packer wrote %u, its header counts %zu, and %u of its "
            "records serve another tile's bytes (%u read back as their own, %u fail)",
            wroteOld, countO, wrongO, ownO, missO);
    } else {
        ok &= Check(false, "E: the planted failure (the old packer) must serve a neighbour's bytes");
    }
    // With every loose tile readable, the two packers write one archive, byte for byte: the
    // change moves no archive a pack of a healthy folder writes.
    const std::string allNew = root + "\\all_new.gaa", allOld = root + "\\all_old.gaa";
    TileArchive::PackDir(dir, allNew);
    PackDirBefore(dir, allOld);
    std::vector<uint8_t> bn, bo;
    ReadAll(allNew, bn);
    ReadAll(allOld, bo);
    ok &= Check(!bn.empty() && bn == bo,
                "E: with every loose tile readable, the new packer's archive is the old one's, byte for byte");
    Log("[treetest] E all %u loose tiles readable: the new packer's archive %s the old one's (%zu bytes)",
        kTiles, (!bn.empty() && bn == bo) ? "IS byte for byte" : "DIFFERS FROM", bn.size());
    if (ok) {
        for (const std::string& a : {arcNew, arcOld, allNew, allOld}) DeleteFileA(a.c_str());
        RemoveFolders({dir, root});
    }
    return ok;
}

// ================================================================================================
//  F: the fetch provider's refusals.
// ================================================================================================
// A real JPEG (WIC's encoder) of one flat colour, 256 x 256, so the provider decodes a cache hit.
bool WriteJpeg(const std::string& path, uint8_t r, uint8_t g, uint8_t b) {
    CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    Com<IWICImagingFactory> fac;
    if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                IID_PPV_ARGS(&fac)))) {
        return false;
    }
    Com<IWICStream> stream;
    if (FAILED(fac->CreateStream(&stream))) return false;
    const std::wstring wpath(path.begin(), path.end());
    if (FAILED(stream->InitializeFromFilename(wpath.c_str(), GENERIC_WRITE))) return false;
    Com<IWICBitmapEncoder> enc;
    if (FAILED(fac->CreateEncoder(GUID_ContainerFormatJpeg, nullptr, &enc))) return false;
    if (FAILED(enc->Initialize(stream.Get(), WICBitmapEncoderNoCache))) return false;
    Com<IWICBitmapFrameEncode> frame;
    Com<IPropertyBag2> props;
    if (FAILED(enc->CreateNewFrame(&frame, &props)) || FAILED(frame->Initialize(props.Get()))) {
        return false;
    }
    frame->SetSize(256, 256);
    WICPixelFormatGUID fmt = GUID_WICPixelFormat24bppBGR;
    frame->SetPixelFormat(&fmt);
    std::vector<uint8_t> px(256 * 256 * 3);
    for (size_t i = 0; i < px.size(); i += 3) {
        px[i] = b;
        px[i + 1] = g;
        px[i + 2] = r;
    }
    if (FAILED(frame->WritePixels(256, 256 * 3, UINT(px.size()), px.data()))) return false;
    return SUCCEEDED(frame->Commit()) && SUCCEEDED(enc->Commit());
}

bool RefusalGate() {
    const std::string root = Scratch("google");
    const std::string maps = root + "\\satellite";
    tree_detail::MakeDir(maps);
    // The cache holds z3 (2,3) and (3,3); nothing else. The budget is zero and the provider has
    // no connection to make: InitCacheOnly never sends a request, whatever the budget.
    const std::string hitA = maps + "\\z3_x2_y3.jpg", hitB = maps + "\\z3_x3_y3.jpg";
    bool ok = Check(WriteJpeg(hitA, 200, 40, 40) && WriteJpeg(hitB, 40, 200, 40),
                    "F: the scratch cache's two JPEGs are written");
    GoogleTileProvider prov;
    prov.InitCacheOnly(root, "satellite", 0u);
    const bool hit = prov.Decoded(3, 2, 3) != nullptr;
    const uint32_t afterHit = prov.Refused();
    const bool miss = prov.Decoded(3, 4, 3) == nullptr;
    const uint32_t opensAfterMiss = prov.CacheOpens();
    for (int i = 0; i < 100; ++i) prov.Decoded(3, 4, 3);
    const uint32_t opensAfterRepeat = prov.CacheOpens();
    ok &= Check(hit && afterHit == 0, "F: a cached tile is served and is no refusal");
    ok &= Check(miss && prov.Refused() == 1, "F: a missing tile with the budget spent is refused, and counted");
    ok &= Check(opensAfterRepeat == opensAfterMiss,
                "F: a refusal is remembered: asked 100 times more, the cache is not opened again");
    prov.Decoded(3, 5, 3);
    prov.Decoded(3, 5, 4);
    ok &= Check(prov.Refused() == 3 && prov.Fetched() == 0,
                "F: distinct refusals are counted, and no fetch is made");
    // A painted tile, through a fresh provider over the same cache: 128 x 128 texels of the z3
    // page from pixel (448, 704), straddling four source tiles -- (1,2) (2,2) (1,3) (2,3), of which
    // the cache holds (2,3). Every texel asks the provider; before, every texel of a missing tile
    // opened its file again (three quadrants of 4096 texels: 12288 opens).
    GoogleTileProvider prov2;
    prov2.InitCacheOnly(root, "satellite", 0u);
    GoogleColorSource src(&prov2);
    const Lattice page = Lattice::Window(448, 704, 3);
    const TileRequest r{0, 0, 0, 0};
    Compositor::TileBox box{};
    page.Box(r, box);
    std::vector<uint8_t> tile;
    bool complete = true, anyCover = false, full = false;
    Compositor::PaintSourceTile(&src, page, r, box, tile, complete, anyCover, full);
    const uint32_t opens = prov2.CacheOpens(), refusedPaint = prov2.Refused();
    uint32_t refusals = 0;
    ok &= Check(src.Refusals(refusals) && refusals == refusedPaint,
                "F: the source reports its provider's refusals");
    ok &= Check(!complete && anyCover && opens == 4 && refusedPaint == 3 && prov2.Fetched() == 0,
                "F: a painted tile asks each of its four source tiles once; the three missing are refused once each");
    Log("[treetest] F budget 0, no network: a cached tile served (0 refused); a missing one refused "
        "and remembered (%u cache opens, the same after 100 more asks); 3 distinct refused, %u "
        "fetched. A painted 128x128 tile over four source tiles, one cached: %u cache opens (each "
        "texel of a missing tile opened its file again before: 12288), %u refused, %u fetched, "
        "Transient (%s)",
        opensAfterMiss, prov.Fetched(), opens, refusedPaint, prov2.Fetched(),
        complete ? "NO" : "yes");
    if (ok) {
        DeleteFileA(hitA.c_str());
        DeleteFileA(hitB.c_str());
        RemoveFolders({maps, root});
    }
    return ok;
}

}  // namespace

bool RunTileTreeSelfTest() {
    g_ok = true;
    Log("[treetest] ---- the tile tree fit for a deep pyramid (HIERARCHY 4a), scratch root %s ----",
        RunDir().c_str());
    DepthResult fw, rgba;
    g_ok &= DepthGate(TileTree::Fmt::FloatW, fw);
    g_ok &= DepthGate(TileTree::Fmt::Rgba8, rgba);
    g_ok &= TodayGate();
    g_ok &= ConcurrencyGate();
    g_ok &= StagedRaceGate();
    g_ok &= PrefillGates();
    g_ok &= StripeGate();
    g_ok &= ArchiveGate();
    g_ok &= RefusalGate();
    TileTree::FoldWalkForTest(TileTree::kWalkOnce);
    RemoveDirectoryA(RunDir().c_str());   // empty when every gate passed and removed its own
    Log("[treetest] ---- %s: one stripe at a time (the old walk CAUGHT), the pyramid folds exactly "
        "and costs a write a level, today's depth byte for byte, twelve threads = serial, the marker "
        "walk and the integer box, the stripe hash, the archive's offsets, the refusals ----",
        g_ok ? "PASS" : "FAIL");
    return g_ok;
}

}  // namespace ga
