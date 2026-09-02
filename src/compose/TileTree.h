// ================================================================================================
//  TileTree - M9am: THE SPARSE GA TREE ON DISK, FOR ANY NODE OF THE COMPOSE GRAPH.
//
//  The water already had the graph: DomainSource leaves, NormalizedSource for the unit stage,
//  DomainCompositor::LayeredOver for the blend, CompositeSource so a compositor is itself a source
//  and trees nest, PrintTree so a mis-wired node shows up in the log before it shows up in pixels.
//  What the water never needed -- one page over the estuary fits in RAM -- is the thing imagery
//  cannot live without: the OUTPUT OF EVERY NODE, cached on the NVMe, one 64 KB tile at a time,
//  on the tile addresses every tree shares. This file is that, and only that. It adds no blend
//  rule of its own: the kernel is DomainCompositor::OverStep, the same one the point query runs.
//
//  The pipeline, per the user, and where each arrow lands:
//
//      INGEST      raw files in a local cache                    (the loaders; unchanged)
//      NORMALIZE   -> each source's OWN sparse tree on disk       TileTree over a leaf node
//      COMPOSE     -> a third tree of REFERENCES into the inputs, TileTree over a compose node
//                     with composited tiles only where inputs OVERLAP
//      ...the same again for the seafloor, and for the GIS mask
//      MEGATEXTURE the three trees -> ONE tree for the atlas       TileTree over the root
//
//  THE REFERENCE IS STORED, and it is a directory entry. A compose tile that turns out to be one
//  input's tile, untouched, is written as a zero-byte file whose NAME carries the input tree's
//  identity: f0_m3_x12_y7_<key>.ref-<childId>. The composite tree on disk therefore describes
//  itself -- an index can be built from its folder alone, and a reference resolves to a place in
//  the input tree without running the compositor's rules. That is what lets DirectStorage read
//  it later: a reference IS (path, offset, size) in someone else's archive.
//
//  A tile's KEY is its inputs. Not a global version: each contributing child's identity is
//  already folded into this node's identity, and the per-tile key folds in what each child HOLDS
//  at this address (content or void). It updates when new tiles appear in a source, and at no
//  other time. Peek() answers from directory lookups, so a warm hit reads no tile bytes.
//
//  ABSENT AND VOID ARE DIFFERENT ANSWERS. A child outside the tile's footprint is absent -- for a
//  gate, that is "no opinion" and the layer passes. A child inside the footprint that stored a
//  zero-byte .void covers nothing -- for a gate, that blocks the whole tile. Conflating them
//  inverted the land/sea gate once; the distinction is carried in the per-tile key.
//
//  TILE FORMAT, every node: RGBA8, 128x128 for colour. RGB is the node's value; ALPHA IS ITS
//  COVERAGE WEIGHT, straight (not premultiplied), because OverFinish divides by coverage -- so a
//  composite's tile is exactly the kind of tile a source produces, and can feed the next node
//  with no second format. One consequence worth stating: a single-input compose at partial weight
//  is a REFERENCE, byte for byte, with no full-coverage condition -- straight alpha makes that
//  identity exact where the old over-black lerp could not.
// ================================================================================================
#pragma once

#ifndef NOMINMAX
#define NOMINMAX   // windows.h's min/max macros would break std::min in every includer
#endif
#include <windows.h>
#undef min
#undef max

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <map>
#include <mutex>

#include "compose/ColorStackSource.h"
#include "compose/ComposeTree.h"
#include "compose/Compositor.h"
#include "compose/DomainSource.h"
#include "compose/TileArchive.h"
#include "core/Common.h"

namespace ga {

// Bump when the tile FORMAT or the leaf paint changes. Not when a source does -- a source's
// identity is its own, and coupling the two would orphan every tree on every edit.
inline constexpr int kTileTreeVersion = 1;

namespace tree_detail {

inline uint64_t Fnv1a(uint64_t h, const std::string& s) {
    for (const char c : s) {
        h ^= static_cast<uint8_t>(c);
        h *= 1099511628211ull;
    }
    return h;
}
inline std::string Hex8(uint64_t h) {
    char b[16];
    snprintf(b, sizeof(b), "%08x", static_cast<uint32_t>(h & 0xFFFFFFFFu));
    return b;
}
inline std::string Sanitize(const std::string& s) {
    std::string o;
    o.reserve(s.size());
    for (const char c : s) {
        o += (isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '-' || c == '_') ? c
                                                                                         : '_';
    }
    return o.empty() ? std::string("node") : o;
}
inline bool ReadTile(const std::string& path, std::vector<uint8_t>& out, size_t bytes = 65536) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (out.size() != bytes) {
        out.clear();
        return false;   // torn write (crash mid-save): repaint it
    }
    return true;
}
// M9as: half floats for the height trees. Value and weight both ride as halves; the weight's
// 10-bit mantissa is far finer than the 8-bit alpha colour uses, and a bed in metres at half
// precision is the same choice the height tenant already made (R16F).
inline uint16_t F2H(float f) {
    const uint32_t x = *reinterpret_cast<const uint32_t*>(&f);
    const uint32_t sign = (x >> 16) & 0x8000u;
    int32_t e = static_cast<int32_t>((x >> 23) & 0xFF) - 127 + 15;
    const uint32_t m = (x >> 13) & 0x3FFu;
    if (e <= 0) return static_cast<uint16_t>(sign);
    if (e >= 31) return static_cast<uint16_t>(sign | 0x7BFFu);
    return static_cast<uint16_t>(sign | (static_cast<uint32_t>(e) << 10) | m);
}
inline float H2F(uint16_t h) {
    const uint32_t s = (h >> 15) & 1u, e = (h >> 10) & 31u, m = h & 1023u;
    if (e == 0) return s ? -0.0f : 0.0f;   // denormals flushed: sub-millimetre at this scale
    const uint32_t x = (s << 31) | ((e + 112u) << 23) | (m << 13);
    return *reinterpret_cast<const float*>(&x);
}
// A failed write must be loud: a whole audit pass once painted 1742 tiles into folders that did
// not exist, reported good numbers, and stored nothing.
inline void WriteTile(const std::string& path, const std::vector<uint8_t>& data) {
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(data.data()), data.size());
    if (!f) {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) {
            Log("[tiletree] CANNOT WRITE %s -- painting and storing NOTHING (missing folder? "
                "disk full?)",
                path.c_str());
        }
    }
}
inline void Touch(const std::string& path) {
    std::ofstream f(path, std::ios::binary);   // zero bytes: the entry IS the record
    if (!f) {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) Log("[tiletree] CANNOT WRITE %s", path.c_str());
    }
}
inline bool Exists(const std::string& path) {
    return GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES;
}
inline void MakeDir(const std::string& path) { CreateDirectoryA(path.c_str(), nullptr); }
// The one wildcard lookup a stored reference costs: "<base>.ref-*" -> the suffix.
inline bool FindRef(const std::string& base, std::string& childId) {
    WIN32_FIND_DATAA fd{};
    HANDLE h = FindFirstFileA((base + ".ref-*").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) return false;
    const std::string name = fd.cFileName;
    FindClose(h);
    const size_t p = name.rfind(".ref-");
    if (p == std::string::npos) return false;
    childId = name.substr(p + 5);
    return !childId.empty();
}

}  // namespace tree_detail

class TileTree {
public:
    enum class Held : uint8_t { Absent, Void, Content };
    enum class Status : uint8_t { Void, Content, Transient };
    // M9as: WHAT A TILE IS. Rgba8 is colour: 128x128, RGB + coverage byte, 64 KB, and it is
    // the GPU tile. FloatW is a scalar field with its coverage: 256x128 texels of (float value,
    // float weight), 256 KB -- the form a node needs to be an INPUT (straight alpha, section
    // 31), and LOSSLESS, which a leaf tree must be: storing the leaves as half and composing
    // from them cost one half-ULP against the incumbent (2 m at 3 km depth, 0.125 m at 100 m).
    // Half is the GPU form: 256x128 half values, 64 KB, R16F exactly -- the ONE quantization,
    // at the root, which always materializes because its tile is what DirectStorage reads.
    enum class Fmt : uint8_t { Rgba8, FloatW, Half };

    // Builds the tree of caches under this node, recursively. Every compose and gate node gets
    // its own folder; every leaf gets one. Nodes are borrowed -- the graph outlives the caches.
    explicit TileTree(const DomainSource* node, Fmt fmt = Fmt::Rgba8) : m_node(node), m_fmt(fmt) {
        const std::string kind = node->NodeKind();
        if (kind == "compose" || kind == "gate") {
            // Under a scalar root every input is FloatW: coverage rides with the value.
            const Fmt kidFmt = (fmt == Fmt::Rgba8) ? Fmt::Rgba8 : Fmt::FloatW;
            for (size_t i = 0; i < node->InputCount(); ++i) {
                m_kids.push_back(std::make_unique<TileTree>(node->Input(i), kidFmt));
            }
        }
        // A leaf is anything not composed here -- a loader, or a loader under a normalize.
        // Find the colour source underneath, if there is one: it carries BeginTile, which is
        // where the vector GIS mask sweeps its rings once per tile.
        const DomainSource* n = node;
        while (n && std::string(n->NodeKind()) == "normalize" && n->InputCount() == 1) {
            n = n->Input(0);
        }
        if (n && std::string(n->NodeKind()) == "load") {
            if (auto* cl = dynamic_cast<const ColorLayerSource*>(n)) m_raw = cl->Raw();
        }
        const uint64_t h = tree_detail::Fnv1a(
            tree_detail::Fnv1a(14695981039346656037ull, node->Identity()),
            "#" + std::to_string(kTileTreeVersion) + (fmt == Fmt::Rgba8 ? "" : fmt == Fmt::FloatW ? "fw" : "h"));
        m_id = tree_detail::Hex8(h);
        m_root = "cache\\trees\\" + tree_detail::Sanitize(node->Name()) + "." + m_id;
        tree_detail::MakeDir("cache");
        tree_detail::MakeDir("cache\\trees");
        tree_detail::MakeDir(m_root);
    }

    const std::string& Id() const { return m_id; }
    Fmt Format() const { return m_fmt; }
    size_t TileBytes() const { return m_fmt == Fmt::FloatW ? 262144u : 65536u; }
    const char* Name() const { return m_node->Name(); }
    const DomainSource* Node() const { return m_node; }
    size_t KidCount() const { return m_kids.size(); }
    TileTree& Kid(size_t i) { return *m_kids[i]; }
    // M9ay: a node by name, depth-first, outermost match first (a normalize wrapper shares
    // its leaf's name; its tiles are references into the leaf, so either answers).
    TileTree* Find(const char* name) {
        if (std::strcmp(Name(), name) == 0) return this;
        for (auto& k : m_kids) {
            if (TileTree* f = k->Find(name)) return f;
        }
        return nullptr;
    }

    void EnsureFrame(const std::string& tag) {
        tree_detail::MakeDir(m_root + "\\" + tag);
        for (auto& k : m_kids) k->EnsureFrame(tag);
    }

    // ---- what is held here, from the directory alone ----------------------------------------
    // For a leaf: two lookups. For a compose node: its children's answers decide the key, so if
    // any child is Absent this node cannot yet know its own key and answers Absent.
    Held Peek(const ColorFrame& frame, const std::string& tag, const TileRequest& r) {
        if (m_kids.empty()) return PeekAt(Base(tag, r));
        Compositor::TileBox box{};
        frame.Box(r, box);
        std::vector<Held> held;
        std::vector<size_t> inc;
        if (!KidsHeld(frame, tag, r, box, inc, held)) return Held::Absent;
        return PeekAt(Base(tag, r) + "_" + tree_detail::Hex8(KeyOf(inc, held)));
    }

    // ---- the tile ---------------------------------------------------------------------------
    // M9ao: `loc`, when given, lets a tile that lives in an archive be answered as a PLACE --
    // path, offset, size -- and the bytes never enter this process: Content with `out` empty and
    // `loc` valid. A stored reference resolves through the child it names, so a reference in
    // the megatexture's folder becomes a TileLoc into earth.land's archive, or google's. That
    // is the user's rule made literal: painted tiles live on disk and DirectStorage loads them.
    Status Tile(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                std::vector<uint8_t>& out, TileLoc* loc = nullptr) {
        Compositor::TileBox box{};
        frame.Box(r, box);
        if (m_kids.empty()) return LeafTile(frame, tag, r, box, out, loc);
        return std::string(m_node->NodeKind()) == "gate" ? GateTile(frame, tag, r, box, out, loc)
                                                         : ComposeTile(frame, tag, r, box, out, loc);
    }

    // The residency manager's view of this tree: a place if it can take one, else bytes, else
    // zeros where nothing covers.
    TileProviderFn Provider(const ColorFrame& frame) {
        const std::string tag = frame.Tag();
        EnsureFrame(tag);
        return [this, frame, tag](const TileRequest& r, std::vector<uint8_t>& out, TileLoc* loc) {
            const Status st = Tile(frame, tag, r, out, loc);
            if (st != Status::Content) {
                out.assign(65536, 0);
                if (loc) *loc = TileLoc{};
            }
            return true;
        };
    }

    // M9ao: pack this node's frame folders (and the children's) into one archive per frame.
    // The loose files stay: the archive is derived. Returns tiles packed across the tree.
    uint32_t Pack(const std::vector<std::string>& tags) {
        uint32_t n = 0;
        for (const std::string& tag : tags) {
            if (m_fmt == Fmt::FloatW) continue;   // 256 KB tiles: read through files, never places
            n += TileArchive::PackDir(m_root + "\\" + tag, ArchivePath(tag));
        }
        {
            std::lock_guard<std::mutex> lk(m_arcMx);
            m_arcs.clear();   // a fresh pack supersedes whatever was open
        }
        for (auto& k : m_kids) n += k->Pack(tags);
        return n;
    }
    // How many tiles the archives under this node answered as places or through a handle.
    std::atomic<uint32_t> arcPlaces{0}, arcReads{0};

    // ---- accounting --------------------------------------------------------------------------
    std::atomic<uint32_t> painted{0}, read{0}, voids{0}, refs{0}, composed{0}, hits{0};
    std::string Stats(int depth = 0) const {
        std::string pad(size_t(depth) * 2, ' ');
        char b[256];
        snprintf(b, sizeof(b),
                 "%s%s.%s: %u painted, %u read, %u void, %u ref, %u composed, %u hit | archive: "
                 "%u places, %u handle reads",
                 pad.c_str(), m_node->Name(), m_id.c_str(), painted.load(), read.load(),
                 voids.load(), refs.load(), composed.load(), hits.load(), arcPlaces.load(),
                 arcReads.load());
        std::string s = b;
        for (const auto& k : m_kids) s += "\n" + k->Stats(depth + 1);
        return s;
    }
    void Print(int depth = 0) const {
        std::string pad(size_t(depth) * 2, ' ');
        Log("[tiletree] %s%-24s %-8s %s  %s", pad.c_str(), m_node->Name(), m_node->NodeKind(),
            m_id.c_str(), m_root.c_str());
        for (const auto& k : m_kids) k->Print(depth + 1);
    }

private:
    static constexpr int kBlendVersion = 3;   // v3: straight alpha, stored refs, gates

    std::string ArchivePath(const std::string& tag) const { return m_root + "\\" + tag + ".gaa"; }

    // The archive for one frame, opened once. Null when there is none, which is the loose path.
    TileArchive* Archive(const std::string& tag) {
        std::lock_guard<std::mutex> lk(m_arcMx);
        auto it = m_arcs.find(tag);
        if (it == m_arcs.end()) {
            TileArchive a;
            const bool ok = a.OpenPath(ArchivePath(tag));
            it = m_arcs.emplace(tag, std::move(a)).first;
            if (ok) {
                Log("[tiletree] %s/%s: archive open, %zu tiles answer as places", m_node->Name(),
                    tag.c_str(), it->second.Count());
            }
        }
        return it->second.Valid() ? &it->second : nullptr;
    }
    // Try the archive for (r, subset). A caller with `loc` gets a place; one without gets the
    // bytes through the archive's one handle. False = not archived: fall through to the files.
    bool FromArchive(const std::string& tag, const TileRequest& r, uint32_t subset,
                     std::vector<uint8_t>& out, TileLoc* loc) {
        if (m_fmt == Fmt::FloatW) loc = nullptr;   // 256 KB on disk is not a GPU tile: bytes only
        TileArchive* arc = Archive(tag);
        if (!arc) return false;
        const uint64_t key = (uint64_t(r.face) << 61) | (uint64_t(r.mip) << 56) |
                             (uint64_t(r.y & 0xFFFFFFFull) << 28) | uint64_t(r.x & 0xFFFFFFFull);
        const TileArchive::Rec* rec = arc->Find(key, subset);
        if (!rec || rec->size == 0) return false;
        if (loc) {
            loc->path = arc->WPath().c_str();
            loc->offset = rec->offset;
            loc->size = rec->size;
            out.clear();
            ++arcPlaces;
            return true;
        }
        if (!arc->ReadPayload(*rec, out)) return false;
        ++arcReads;
        return true;
    }

    std::string Base(const std::string& tag, const TileRequest& r) const {
        char buf[320];
        snprintf(buf, sizeof(buf), "%s\\%s\\f%u_m%u_x%u_y%u", m_root.c_str(), tag.c_str(),
                 r.face, r.mip, r.x, r.y);
        return buf;
    }
    static Held PeekAt(const std::string& base) {
        if (tree_detail::Exists(base + ".void")) return Held::Void;
        if (tree_detail::Exists(base + ".bin")) return Held::Content;
        std::string id;
        if (tree_detail::FindRef(base, id)) return Held::Content;
        return Held::Absent;
    }

    // Which children can touch this tile, and what each holds. False if any is still Absent.
    bool KidsHeld(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                  const Compositor::TileBox& box, std::vector<size_t>& inc,
                  std::vector<Held>& held) {
        constexpr double kR2D = 180.0 / 3.14159265358979;
        bool all = true;
        for (size_t i = 0; i < m_kids.size(); ++i) {
            // The SAME membership test the incumbent uses -- a declared footprint must span
            // ~2 texels of this tile to be in its subset; an undeclared one is always in.
            double fl0, fb0, fl1, fb1;
            const bool in = m_kids[i]->m_node->Footprint(fl0, fb0, fl1, fb1)
                                ? Compositor::Touches(fl0, fb0, fl1, fb1, box)
                                : m_kids[i]->m_node->MayCover(box.lonMin * kR2D, box.latMin * kR2D,
                                                              box.lonMax * kR2D, box.latMax * kR2D);
            if (!in) continue;   // absent: not in this tile's subset at all
            inc.push_back(i);
            const Held h = m_kids[i]->Peek(frame, tag, r);
            held.push_back(h);
            if (h == Held::Absent) all = false;
        }
        return all;
    }
    uint64_t KeyOf(const std::vector<size_t>& inc, const std::vector<Held>& held) const {
        uint64_t h = tree_detail::Fnv1a(14695981039346656037ull,
                                        "#" + std::to_string(kBlendVersion));
        for (size_t k = 0; k < inc.size(); ++k) {
            h = tree_detail::Fnv1a(h, m_kids[inc[k]]->m_id);
            h = tree_detail::Fnv1a(h, held[k] == Held::Void ? "-" : "+");
        }
        return h;
    }
    // Serve a stored tile at `cpath` if there is one: bytes, a void, or a reference resolved
    // through the child it names.
    bool Serve(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
               const std::string& cpath, uint32_t key32, std::vector<uint8_t>& out, TileLoc* loc,
               Status& st) {
        if (FromArchive(tag, r, key32, out, loc)) {
            ++hits;
            st = Status::Content;
            return true;
        }
        if (tree_detail::Exists(cpath + ".void")) {
            ++hits;
            st = Status::Void;
            return true;
        }
        if (tree_detail::ReadTile(cpath + ".bin", out, TileBytes())) {
            ++hits;
            st = Status::Content;
            return true;
        }
        std::string id;
        if (tree_detail::FindRef(cpath, id)) {
            for (auto& k : m_kids) {
                if (k->m_id == id) {
                    ++hits;
                    ++refs;
                    st = k->Tile(frame, tag, r, out, loc);   // the place is the child's
                    return true;
                }
            }
        }
        return false;
    }

    // ---- leaf: paint ONE source over the frame's addresses -----------------------------------
    Status LeafTile(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                    const Compositor::TileBox& box, std::vector<uint8_t>& out, TileLoc* loc) {
        const std::string base = Base(tag, r);
        if (FromArchive(tag, r, 0u, out, loc)) {
            ++read;
            return Status::Content;
        }
        if (tree_detail::Exists(base + ".void")) {
            ++hits;
            return Status::Void;
        }
        if (tree_detail::ReadTile(base + ".bin", out, TileBytes())) {
            ++read;
            return Status::Content;
        }
        bool complete = true, anyCover = false, full = false;
        if (m_fmt != Fmt::Rgba8) {
            // A scalar leaf: sample the node per texel, store (half value, half weight).
            constexpr double kR2D = 180.0 / 3.14159265358979;
            out.assign(TileBytes(), 0);
            float* d32 = reinterpret_cast<float*>(out.data());
            for (uint32_t py = 0; py < frame.texH; ++py) {
                for (uint32_t px = 0; px < frame.texW; ++px) {
                    double lat = 0, lon = 0;
                    frame.Texel(r, px, py, lat, lon);
                    DomainQuery q;
                    q.lon = lon * kR2D;
                    q.lat = lat * kR2D;
                    q.groundM = frame.GroundRes(r.mip);
                    DomainValue v;
                    if (!m_node->SampleAt(q, v) || v.weight <= 0.0f) continue;
                    const size_t i = (size_t(py) * frame.texW + px) * 2;
                    d32[i] = v.c[0];
                    d32[i + 1] = (std::min)(1.0f, v.weight);
                    anyCover = true;
                }
            }
        } else if (m_raw) {
            // The colour path: the same paint the composed cache was built with, alpha = weight.
            Compositor::PaintSourceTile(m_raw, frame, r, box, out, complete, anyCover, full);
        } else {
            // A generic node: sample it per texel through the graph, quantize.
            constexpr double kR2D = 180.0 / 3.14159265358979;
            const uint32_t chan = (std::min)(4u, m_node->Channels());
            out.assign(65536, 0);
            for (uint32_t py = 0; py < frame.texH; ++py) {
                for (uint32_t px = 0; px < frame.texW; ++px) {
                    double lat = 0, lon = 0;
                    frame.Texel(r, px, py, lat, lon);
                    DomainQuery q;
                    q.lon = lon * kR2D;
                    q.lat = lat * kR2D;
                    q.groundM = frame.GroundRes(r.mip);
                    DomainValue v;
                    if (!m_node->SampleAt(q, v) || v.weight <= 0.0f) continue;
                    uint8_t* d = &out[(py * frame.texW + px) * 4];
                    for (uint32_t c = 0; c < chan && c < 3; ++c) {
                        d[c] = static_cast<uint8_t>((std::max)(0.0f, (std::min)(1.0f, v.c[c])) * 255.0f + 0.5f);
                    }
                    d[3] = static_cast<uint8_t>((std::min)(1.0f, v.weight) * 255.0f + 0.5f);
                    anyCover = true;
                }
            }
        }
        if (!complete) return Status::Transient;   // never cached; the next run repaints
        ++painted;
        if (!anyCover) {
            tree_detail::Touch(base + ".void");
            ++voids;
            return Status::Void;
        }
        tree_detail::WriteTile(base + ".bin", out);
        return Status::Content;
    }

    // ---- compose: the children's tiles through OverStep ---------------------------------------
    Status ComposeTile(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                       const Compositor::TileBox& box, std::vector<uint8_t>& out, TileLoc* loc) {
        std::vector<size_t> inc;
        std::vector<Held> held;
        const bool known = KidsHeld(frame, tag, r, box, inc, held);
        const std::string base = Base(tag, r);
        Status st = Status::Void;
        if (known) {
            const uint64_t key = KeyOf(inc, held);
            if (Serve(frame, tag, r, base + "_" + tree_detail::Hex8(key),
                      static_cast<uint32_t>(key & 0xFFFFFFFFu), out, loc, st)) {
                return st;
            }
        }
        // Gather. A child's answer here is authoritative, so the key is rebuilt from it.
        std::vector<std::vector<uint8_t>> tiles(inc.size());
        std::vector<Held> got(inc.size(), Held::Void);
        std::vector<size_t> live;
        bool complete = true;
        for (size_t k = 0; k < inc.size(); ++k) {
            const Status s = m_kids[inc[k]]->Tile(frame, tag, r, tiles[k]);
            if (s == Status::Transient) {
                complete = false;
                continue;
            }
            if (s == Status::Void) continue;
            got[k] = Held::Content;
            live.push_back(k);
        }
        const std::string cpath = base + "_" + tree_detail::Hex8(KeyOf(inc, got));
        // THE REFERENCE. Straight alpha makes a lone input's tile the composite exactly --
        // OverFinish divides by the coverage it just multiplied by -- so the record is the
        // child's identity and nothing else. A Half ROOT never references: its tile is the
        // GPU's, and a 128 KB FloatW child is not that.
        if (live.size() == 1 && m_fmt != Fmt::Half) {
            out = std::move(tiles[live[0]]);
            if (complete) tree_detail::Touch(cpath + ".ref-" + m_kids[inc[live[0]]]->m_id);
            ++refs;
            return Status::Content;
        }
        if (live.empty()) {
            out.assign(TileBytes(), 0);
            if (complete) {
                tree_detail::Touch(cpath + ".void");
                ++voids;
            }
            return Status::Void;
        }
        if (m_fmt != Fmt::Rgba8) {
            // M9as: the scalar compose. Same OverStep, one channel, coverage from the halves.
            // Inputs are FloatW (value, weight); the output is FloatW again, or Half at the root.
            out.assign(TileBytes(), 0);
            float* d32 = reinterpret_cast<float*>(out.data());
            uint16_t* d16 = reinterpret_cast<uint16_t*>(out.data());
            const uint32_t n = frame.texW * frame.texH;
            bool any = false;
            for (uint32_t i = 0; i < n; ++i) {
                double acc[4] = {0, 0, 0, 0}, cov = 0.0;
                for (const size_t k : live) {
                    const float* s32 = reinterpret_cast<const float*>(tiles[k].data());
                    const float w = s32[i * 2 + 1];
                    if (w <= 0.0f) continue;
                    const float c[4] = {s32[i * 2], 0, 0, 0};
                    DomainCompositor::OverStep(acc, cov, c, w, 1);
                }
                if (cov <= 0.0) continue;
                DomainValue v;
                DomainCompositor::OverFinish(acc, cov, 1, v);
                if (m_fmt == Fmt::FloatW) {
                    d32[i * 2] = v.c[0];
                    d32[i * 2 + 1] = (std::min)(1.0f, v.weight);
                } else {
                    d16[i] = tree_detail::F2H(v.c[0]);   // the one quantization: the GPU's
                }
                any = true;
            }
            if (!any) {
                if (complete) {
                    tree_detail::Touch(cpath + ".void");
                    ++voids;
                }
                return Status::Void;
            }
            if (complete) {
                tree_detail::WriteTile(cpath + ".bin", out);
                ++composed;
            }
            return Status::Content;
        }
        out.assign(65536, 0);
        const uint32_t n = frame.texW * frame.texH;
        bool any = false;
        for (uint32_t i = 0; i < n; ++i) {
            double acc[4] = {0, 0, 0, 0}, cov = 0.0;
            for (const size_t k : live) {
                const uint8_t* s = &tiles[k][i * 4];
                if (s[3] == 0) continue;
                const float c[4] = {s[0] / 255.0f, s[1] / 255.0f, s[2] / 255.0f, 0.0f};
                DomainCompositor::OverStep(acc, cov, c, s[3] / 255.0f, 3);
            }
            if (cov <= 0.0) continue;
            DomainValue v;
            DomainCompositor::OverFinish(acc, cov, 3, v);
            uint8_t* d = &out[i * 4];
            // Truncate, as the incumbent does: the audit compares against it, and a rounding
            // policy is a separate decision from a storage structure.
            for (int c = 0; c < 3; ++c) {
                d[c] = static_cast<uint8_t>((std::max)(0.0f, (std::min)(255.0f, v.c[c] * 255.0f)));
            }
            d[3] = static_cast<uint8_t>((std::min)(1.0f, v.weight) * 255.0f + 0.5f);
            any = true;
        }
        if (!any) {
            if (complete) {
                tree_detail::Touch(cpath + ".void");
                ++voids;
            }
            return Status::Void;
        }
        if (complete) {
            tree_detail::WriteTile(cpath + ".bin", out);
            ++composed;
        }
        return Status::Content;
    }

    // ---- gate: layer tile x gate tile ----------------------------------------------------------
    Status GateTile(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                    const Compositor::TileBox& box, std::vector<uint8_t>& out, TileLoc* loc) {
        std::vector<size_t> inc;
        std::vector<Held> held;
        const bool known = KidsHeld(frame, tag, r, box, inc, held);
        const std::string base = Base(tag, r);
        Status st = Status::Void;
        if (known) {
            const uint64_t key = KeyOf(inc, held);
            if (Serve(frame, tag, r, base + "_" + tree_detail::Hex8(key),
                      static_cast<uint32_t>(key & 0xFFFFFFFFu), out, loc, st)) {
                return st;
            }
        }
        TileTree& layer = *m_kids[0];
        TileTree& gate = *m_kids[1];
        std::vector<uint8_t> lt, gt;
        const Status ls = layer.Tile(frame, tag, r, lt);
        if (ls == Status::Transient) return Status::Transient;
        std::vector<Held> got;
        bool gateIn = false;
        for (size_t k = 0; k < inc.size(); ++k) gateIn |= (inc[k] == 1);
        Status gs = Status::Void;
        if (gateIn) {
            gs = gate.Tile(frame, tag, r, gt);
            if (gs == Status::Transient) return Status::Transient;
        }
        for (size_t k = 0; k < inc.size(); ++k) {
            got.push_back(inc[k] == 0 ? (ls == Status::Void ? Held::Void : Held::Content)
                                      : (gs == Status::Void ? Held::Void : Held::Content));
        }
        const std::string cpath = base + "_" + tree_detail::Hex8(KeyOf(inc, got));
        if (ls == Status::Void) {
            out.assign(65536, 0);
            tree_detail::Touch(cpath + ".void");
            ++voids;
            return Status::Void;
        }
        if (!gateIn) {
            // Absent gate: no opinion. The layer passes untouched -- a reference.
            out = std::move(lt);
            tree_detail::Touch(cpath + ".ref-" + layer.m_id);
            ++refs;
            return Status::Content;
        }
        if (gs == Status::Void) {
            // Present and empty: the gate covers nothing here, so nothing passes.
            out.assign(65536, 0);
            tree_detail::Touch(cpath + ".void");
            ++voids;
            return Status::Void;
        }
        out = std::move(lt);
        bool any = false;
        for (size_t i = 3; i < out.size(); i += 4) {
            const uint32_t a = (uint32_t(out[i]) * uint32_t(gt[i]) + 127u) / 255u;
            out[i] = static_cast<uint8_t>(a);
            any |= a != 0;
        }
        if (!any) {
            tree_detail::Touch(cpath + ".void");
            ++voids;
            return Status::Void;
        }
        tree_detail::WriteTile(cpath + ".bin", out);
        ++composed;
        return Status::Content;
    }

    const DomainSource* m_node = nullptr;
    Fmt m_fmt = Fmt::Rgba8;
    ColorSource* m_raw = nullptr;
    std::vector<std::unique_ptr<TileTree>> m_kids;
    std::string m_id, m_root;
    std::map<std::string, TileArchive> m_arcs;
    std::mutex m_arcMx;
};

// ================================================================================================
//  AuditTileTree -- the number that decides whether a tree may carry pixels. Walks the incumbent
//  composed cache, keeps every tile whose filename carries the subset the compositor would use
//  TODAY, and asks the tree for the same address. `warmOnly` composes every address regardless
//  and compares nothing: after a source is added there is no comparable tile, which is exactly
//  when the trees most need building.
// ================================================================================================
struct TreeAudit {
    uint32_t tiles = 0, exact = 0;
    uint32_t worstDelta = 0;
    uint64_t difTexels = 0, texels = 0;
    uint32_t alphaDif = 0;
    uint32_t skippedStale = 0;
    uint32_t dumped = 0;   // height: worst-texel leaf dumps printed
};

inline void AuditTileTree(Compositor& comp, int channel, TileTree& tree, const ColorFrame& frame,
                          uint32_t maxTiles, TreeAudit& a, bool warmOnly = false,
                          bool height = false, const char* composedKind = "window") {
    const Compositor::Channel& ch = comp.ChannelAt(channel);
    const std::string tag = frame.Tag();
    tree.EnsureFrame(tag);
    const std::string dir = std::string("cache") + "\\composed\\" + ch.name + "\\" + frame.Tag(composedKind);   // the incumbent names its window realizations by kind
    WIN32_FIND_DATAA fd{};
    HANDLE h = FindFirstFileA((dir + "\\*.bin").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) {
        Log("[tree-audit] %s/%s: no composed tiles to compare against", ch.name.c_str(),
            tag.c_str());
        return;
    }
    std::vector<std::string> names;
    do {
        names.push_back(fd.cFileName);
    } while (FindNextFileA(h, &fd));
    FindClose(h);
    const size_t stride = names.empty() ? 1 : (std::max)(size_t(1), names.size() / maxTiles);
    std::vector<uint8_t> direct, tt;
    for (size_t ni = 0; ni < names.size(); ni += stride) {
        if (a.tiles >= maxTiles) break;
        const std::string& fname = names[ni];
        uint32_t f = 0, m = 0, x = 0, y = 0, sub = 0;
        if (sscanf_s(fname.c_str(), "f%u_m%u_x%u_y%u_%8x.bin", &f, &m, &x, &y, &sub) != 5) continue;
        const TileRequest r{f, m, x, y};
        Compositor::TileBox box{};
        frame.Box(r, box);
        std::vector<size_t> inc;
        const uint64_t subset = height ? comp.HeightSubset(ch, box, inc)
                                       : comp.ColorSubset(ch, box, inc);
        if (!warmOnly && static_cast<uint32_t>(subset & 0xFFFFFFFFu) != sub) {
            ++a.skippedStale;
            continue;
        }
        if (!warmOnly && !tree_detail::ReadTile(dir + "\\" + fname, direct)) continue;
        const TileTree::Status st = tree.Tile(frame, tag, r, tt);
        if (st == TileTree::Status::Transient) continue;
        if (st == TileTree::Status::Void) tt.assign(tree.TileBytes(), 0);
        ++a.tiles;
        if (warmOnly) continue;
        if (tt.size() != direct.size()) continue;
        bool same = true;
        if (height) {
            // R16F vs R16F: the worst difference in millimetres, in worstDelta.
            const uint16_t* d16 = reinterpret_cast<const uint16_t*>(direct.data());
            const uint16_t* t16 = reinterpret_cast<const uint16_t*>(tt.data());
            for (size_t i = 0; i < direct.size() / 2; ++i) {
                const float dm = std::fabs(tree_detail::H2F(d16[i]) - tree_detail::H2F(t16[i]));
                if (dm > 0.0f) {
                    ++a.difTexels;
                    same = false;
                    a.worstDelta = (std::max)(a.worstDelta, uint32_t(dm * 1000.0f));
                    if (dm > 1.0f && a.dumped < 3) {
                        ++a.dumped;
                        std::string leaves;
                        for (size_t q = 0; q < tree.KidCount(); ++q) {
                            std::vector<uint8_t> kt;
                            const TileTree::Status ks = tree.Kid(q).Tile(frame, tag, r, kt);
                            char b[96];
                            if (ks == TileTree::Status::Content && kt.size() >= (i + 1) * 8) {
                                const float* k32 = reinterpret_cast<const float*>(kt.data());
                                snprintf(b, sizeof(b), " %s=%.2f(w%.3f)", tree.Kid(q).Name(), k32[i * 2], k32[i * 2 + 1]);
                            } else {
                                snprintf(b, sizeof(b), " %s=void", tree.Kid(q).Name());
                            }
                            leaves += b;
                        }
                        Log("[tree-audit]   f%u m%u (%u,%u) texel %zu: incumbent %.2f m, tree %.2f m, leaves:%s",
                            f, m, x, y, i, tree_detail::H2F(d16[i]), tree_detail::H2F(t16[i]), leaves.c_str());
                    }
                }
                ++a.texels;
            }
            if (same) ++a.exact;
            continue;
        }
        for (size_t i = 0; i < direct.size(); i += 4) {
            uint32_t d = 0;
            for (int c = 0; c < 3; ++c) {
                d = (std::max)(d, uint32_t(std::abs(int(direct[i + c]) - int(tt[i + c]))));
            }
            // The incumbent's alpha is binary cover; the tree's is coverage weight. Compare
            // COVERAGE, not the byte: covered vs not.
            if ((direct[i + 3] != 0) != (tt[i + 3] != 0)) ++a.alphaDif;
            if (d) {
                ++a.difTexels;
                same = false;
                a.worstDelta = (std::max)(a.worstDelta, d);
            }
            ++a.texels;
        }
        if (same) ++a.exact;
    }
    Log("[tree-audit] %s/%s: %u of %zu tiles (stride %zu), %u byte-identical, worst "
        "|direct - tree| = %u/255, %llu of %llu texels differ (%.4f%%), %u cover mismatches; "
        "%u stale skipped",
        ch.name.c_str(), tag.c_str(), a.tiles, names.size(), stride, a.exact, a.worstDelta,
        static_cast<unsigned long long>(a.difTexels), static_cast<unsigned long long>(a.texels),
        a.texels ? 100.0 * double(a.difTexels) / double(a.texels) : 0.0, a.alphaDif,
        a.skippedStale);
}

}  // namespace ga
