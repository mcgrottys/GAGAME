// ================================================================================================
//  SourceTree - M9aj: THE THREE TREES. Each source is its own sparse tree on disk; the composite
//  is a THIRD tree that mostly REFERENCES the other two.
//
//  The pipeline the user has been stating, per source type:
//
//      1. INGEST      GeoTIFFs; Google tiles            -> raw files in a local cache
//      2. NORMALIZE   to the body's scale, units, projection
//                     -> cached as its OWN sparse GA tree on disk
//      3. COMPOSE     the trees -> ONE composite sparse GA tree
//                     (tile REFERENCES into the input trees, and genuinely composited tiles
//                      cached only where the inputs OVERLAP)
//
//  Stage 2's output did not exist. `cache/composed/<channel>/<realization>/` held only the
//  FINISHED blend, and that is why the cache behaves the way it does today: a tile's filename
//  carries the source-SUBSET hash it was painted from, and the subset seed carries
//  kComposeVersion, so bumping the blend version orphans every tile in the channel -- including
//  the 97% of cube tiles that only ever had ONE source in them and could not have changed.
//
//  Measured on this machine's cache before any of this ran:
//
//      cube16k        4340 tiles   271 MB   of which 4221 (97.3%) are google-only,
//                                           1792 of those superseded by a version bump alone
//      window_z14     4510 tiles   282 MB
//      window_z17     6801 tiles   425 MB
//
//  With the source tree in the middle, a source's tiles are identified by what the SOURCE says
//  -- its name, its structure, and the tree format version -- and by nothing about the stack
//  above it. Changing the blend, reordering the stack, or adding a fourth layer cannot orphan
//  them. Only the composite layer churns, and the composite layer is, where one source covers,
//  not a file at all.
//
//  THE REFERENCE IS COMPUTED, NOT STORED. A marker file per referenced tile would carry no
//  information the compositor does not already have: `ColorSubset` decides which sources touch a
//  tile from their declared footprints, in microseconds, with no filesystem contact. So a
//  reference is "exactly one source has coverage here, therefore the composite IS its tree
//  tile", re-derived on the spot. That is one directory entry saved per referenced tile and,
//  more importantly, nothing on disk that can go stale.
//
//  WHEN A REFERENCE IS EXACT, and it is not always. The composite starts from black and lerps
//  toward each layer by its weight, so a LONE source at weight 0.5 composites to half its own
//  colour, not to its colour. A reference is therefore valid only where the single covering
//  source claims the tile FULLY (alpha 255 at every texel). Where it does not -- a feather, the
//  bed's intertidal alpha ramp -- the tile is composed and stored like any overlap. `fullCover`
//  is that test, and it is computed from the tile, never assumed from the footprint.
//
//  WHAT THIS BUYS THE MEGATEXTURE. The land/sea mask arrives as a GIS tree, the seafloor as its
//  own tree, and "land over sea, transparent over water per pixel" becomes a compose of trees
//  that already exist -- 64 KB in, 64 KB in, 64 KB out, no reprojection, no HTTP, no resample.
//  Composition stops being a thing that repaints the planet and becomes a thing that reads it.
// ================================================================================================
#pragma once

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "compose/Compositor.h"
#include "core/Common.h"

namespace ga {

// Bump ONLY when the per-source tile format or the per-source paint changes. Deliberately not
// the compositor's kComposeVersion: that one versions the BLEND, which a source tree does not
// contain, and coupling them would reintroduce exactly the orphaning this file exists to stop.
inline constexpr int kSourceTreeVersion = 1;

namespace tree_detail {

inline uint64_t Fnv1a(uint64_t h, const std::string& s) {
    for (const char c : s) {
        h ^= static_cast<uint8_t>(c);
        h *= 1099511628211ull;
    }
    return h;
}

// A folder name has to survive a source calling itself whatever it likes.
inline std::string Sanitize(const std::string& s) {
    std::string o;
    o.reserve(s.size());
    for (const char c : s) {
        o += (isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '-' || c == '_') ? c
                                                                                         : '_';
    }
    return o.empty() ? std::string("src") : o;
}

inline bool ReadTile(const std::string& path, std::vector<uint8_t>& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (out.size() != 65536) {
        out.clear();
        return false;   // torn write (crash mid-save): repaint it
    }
    return true;
}

inline void WriteTile(const std::string& path, const std::vector<uint8_t>& data) {
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(data.data()), data.size());
    if (!f) {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) {
            Log("[trees] CANNOT WRITE %s -- the tree is painting and storing NOTHING (missing "
                "folder? disk full?). Every later miss will repaint.",
                path.c_str());
        }
    }
}

inline bool Exists(const std::string& path) {
    return GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES;
}

inline void Touch(const std::string& path) {
    std::ofstream f(path, std::ios::binary);   // zero bytes: the entry IS the record
    if (!f) {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) Log("[trees] CANNOT WRITE %s", path.c_str());
    }
}

inline void MakeDir(const std::string& path) { CreateDirectoryA(path.c_str(), nullptr); }

// Does every texel claim this tile outright? The reference test, read back off disk rather than
// remembered, so a tile written by an older run answers the same question the same way.
inline bool FullCover(const std::vector<uint8_t>& t) {
    for (size_t i = 3; i < t.size(); i += 4) {
        if (t[i] != 255) return false;
    }
    return true;
}

}  // namespace tree_detail

// ================================================================================================
//  SourceTree -- ONE source's normalized tiles, on the shared addresses, on disk.
// ================================================================================================
class SourceTree {
public:
    enum class Status : uint8_t {
        Void,        // the source has no coverage anywhere in this tile
        Content,     // `out` holds 64 KB of RGB + weight
        Transient,   // coverage exists but a fetch failed: cache nothing derived from this
    };

    void Open(ColorSource* src) {
        m_src = src;
        const SourceInfo& si = src->Info();
        const uint64_t h = tree_detail::Fnv1a(
            tree_detail::Fnv1a(14695981039346656037ull, si.name + "|" + si.structure),
            "#" + std::to_string(kSourceTreeVersion));
        char id[16];
        snprintf(id, sizeof(id), "%08x", static_cast<uint32_t>(h & 0xFFFFFFFFu));
        m_id = id;
        m_name = si.name;
        m_root = "cache\\trees\\" + tree_detail::Sanitize(si.name) + "." + m_id;
        tree_detail::MakeDir("cache");
        tree_detail::MakeDir("cache\\trees");
        tree_detail::MakeDir(m_root);
    }

    void EnsureFrame(const std::string& frameTag) {
        tree_detail::MakeDir(m_root + "\\" + frameTag);
    }

    const std::string& Id() const { return m_id; }
    const std::string& Name() const { return m_name; }
    const std::string& Root() const { return m_root; }
    ColorSource* Source() const { return m_src; }

    // Read the tile if it is held, otherwise paint this ONE source over the frame's addresses
    // and hold it. `fullCover` says whether every texel claims the tile outright -- the
    // question the composite asks to decide between a reference and a composition.
    Status Tile(const ColorFrame& frame, const std::string& frameTag, const TileRequest& r,
                const Compositor::TileBox& box, std::vector<uint8_t>& out, bool& fullCover) {
        const std::string base = Base(frameTag, r);
        if (tree_detail::Exists(base + ".void")) {
            ++hits;
            fullCover = false;
            return Status::Void;
        }
        if (tree_detail::ReadTile(base + ".bin", out)) {
            ++hits;
            fullCover = tree_detail::FullCover(out);
            return Status::Content;
        }
        bool complete = true, anyCover = false;
        Compositor::PaintSourceTile(m_src, frame, r, box, out, complete, anyCover, fullCover);
        if (!complete) return Status::Transient;   // never cached; the next run repaints
        ++painted;
        if (!anyCover) {
            // A regional source inside a tile it declared but does not actually reach. Storing
            // 64 KB of zeros to say "nothing here" is the opposite of a sparse tree, so the
            // absence is a zero-byte entry and the answer next run is a directory lookup.
            tree_detail::Touch(base + ".void");
            ++voids;
            return Status::Void;
        }
        tree_detail::WriteTile(base + ".bin", out);
        return Status::Content;
    }

    std::atomic<uint32_t> painted{0}, hits{0}, voids{0};

private:
    std::string Base(const std::string& frameTag, const TileRequest& r) const {
        char buf[320];
        snprintf(buf, sizeof(buf), "%s\\%s\\f%u_m%u_x%u_y%u", m_root.c_str(), frameTag.c_str(),
                 r.face, r.mip, r.x, r.y);
        return buf;
    }

    ColorSource* m_src = nullptr;
    std::string m_id, m_name, m_root;
};

// ================================================================================================
//  ColorTreeStack -- the composite tree, over the source trees.
//
//  It owns one SourceTree per layer of a colour channel and produces a TileProviderFn per frame,
//  so it is a drop-in for Compositor::ColorRealization and the residency manager cannot tell the
//  difference. What differs is what it does when nobody has the tile yet: instead of walking
//  every source per texel, it asks each source's TREE for its tile -- which after the first run
//  is a 64 KB read -- and blends those.
// ================================================================================================
class ColorTreeStack {
public:
    void Init(Compositor* comp, int channel) {
        m_comp = comp;
        m_channel = channel;
        const Compositor::Channel& ch = comp->ChannelAt(channel);
        for (ColorSource* c : ch.color) {
            m_trees.push_back(std::make_unique<SourceTree>());
            m_trees.back()->Open(c);
        }
        m_root = "cache\\trees\\" + tree_detail::Sanitize(ch.name) + ".composite";
        tree_detail::MakeDir("cache");
        tree_detail::MakeDir("cache\\trees");
        tree_detail::MakeDir(m_root);
        std::string names;
        for (const auto& t : m_trees) names += " " + t->Name() + "." + t->Id();
        Log("[trees] %s: %zu source trees +composite --%s", ch.name.c_str(), m_trees.size(),
            names.c_str());
    }

    // Every frame the trees will store into, made once. Both callers -- the provider factory
    // and the audit -- go through this, because the one that did not silently threw away
    // everything it painted.
    void EnsureFrame(const std::string& tag) {
        for (auto& t : m_trees) t->EnsureFrame(tag);
        tree_detail::MakeDir(m_root + "\\" + tag);
    }

    TileProviderFn Realization(const ColorFrame& frame) {
        const std::string tag = frame.Tag();
        EnsureFrame(tag);
        return [this, frame, tag](const TileRequest& r, std::vector<uint8_t>& out, TileLoc*) {
            return Compose(frame, tag, r, out, nullptr);
        };
    }

    // The blend, from tiles instead of from sources. It is the SAME walk as
    // Compositor::PaintColorTile -- bottom to top, lerp toward each layer by its weight, cover
    // is the max -- with the per-texel Sample() replaced by a byte already on disk. `refIdx`,
    // when given, reports which tree the tile was taken from wholesale (-1 = composed).
    bool Compose(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                 std::vector<uint8_t>& out, int* refIdx) {
        const Compositor::Channel& ch = m_comp->ChannelAt(m_channel);
        Compositor::TileBox box{};
        frame.Box(r, box);
        std::vector<size_t> inc;
        const uint64_t subset = m_comp->ColorSubset(ch, box, inc);
        if (refIdx) *refIdx = -1;

        const std::string cpath = CompositePath(tag, r, subset);
        if (tree_detail::Exists(cpath + ".void")) {
            out.assign(65536, 0);
            ++compositeHits;
            return true;
        }
        if (tree_detail::ReadTile(cpath + ".bin", out)) {
            ++compositeHits;
            return true;
        }

        // Gather the covering layers from their own trees, bottom to top.
        std::vector<std::vector<uint8_t>> tiles(inc.size());
        std::vector<size_t> live;      // indices into inc that actually have coverage here
        std::vector<uint8_t> full;     // ...and whether each claims the tile outright
        bool complete = true;
        for (size_t k = 0; k < inc.size(); ++k) {
            bool fc = false;
            const SourceTree::Status st =
                m_trees[inc[k]]->Tile(frame, tag, r, box, tiles[k], fc);
            if (st == SourceTree::Status::Transient) {
                complete = false;
                continue;   // absent for this tile, exactly as a zero weight would be
            }
            if (st == SourceTree::Status::Void) continue;
            live.push_back(k);
            full.push_back(fc ? 1u : 0u);
        }

        // THE REFERENCE. One covering source that claims every texel composes to itself: the
        // lerp from black by weight 1 is the identity, and the composite's alpha (255 where
        // anything covers) is the weight this tree already stored. So the composite IS that
        // tree's tile, byte for byte, and there is nothing to write down.
        if (live.size() == 1 && full[0]) {
            out = std::move(tiles[live[0]]);
            if (refIdx) *refIdx = static_cast<int>(inc[live[0]]);
            ++refs;
            return true;
        }

        out.assign(65536, 0);
        if (live.empty()) {
            if (complete) {
                tree_detail::Touch(cpath + ".void");
                ++voids;
            }
            return true;
        }

        const uint32_t n = frame.texW * frame.texH;
        for (uint32_t i = 0; i < n; ++i) {
            float acc[3] = {0, 0, 0};
            float cover = 0.0f;
            for (const size_t k : live) {
                const uint8_t* s = &tiles[k][i * 4];
                if (s[3] == 0) continue;
                const float w = s[3] * (1.0f / 255.0f);
                for (int c = 0; c < 3; ++c) acc[c] += (s[c] - acc[c]) * w;
                cover = (std::max)(cover, w);
            }
            uint8_t* dst = &out[i * 4];
            dst[0] = static_cast<uint8_t>(acc[0]);
            dst[1] = static_cast<uint8_t>(acc[1]);
            dst[2] = static_cast<uint8_t>(acc[2]);
            dst[3] = cover > 0.0f ? 255 : 0;
        }
        if (complete) {
            tree_detail::WriteTile(cpath + ".bin", out);
            ++composed;
        }
        return true;
    }

    size_t TreeCount() const { return m_trees.size(); }
    SourceTree& TreeAt(size_t i) { return *m_trees[i]; }

    std::string Stats() const {
        char b[256];
        uint32_t sp = 0, sh = 0, sv = 0;
        for (const auto& t : m_trees) {
            sp += t->painted.load();
            sh += t->hits.load();
            sv += t->voids.load();
        }
        snprintf(b, sizeof(b),
                 "composite: %u refs, %u composed, %u void, %u cached | source trees: %u "
                 "painted, %u read, %u void",
                 refs.load(), composed.load(), voids.load(), compositeHits.load(), sp, sh, sv);
        return b;
    }

    std::atomic<uint32_t> refs{0}, composed{0}, voids{0}, compositeHits{0};

private:
    std::string CompositePath(const std::string& tag, const TileRequest& r,
                              uint64_t subset) const {
        char buf[320];
        snprintf(buf, sizeof(buf), "%s\\%s\\f%u_m%u_x%u_y%u_%08x", m_root.c_str(), tag.c_str(),
                 r.face, r.mip, r.x, r.y, static_cast<uint32_t>(subset & 0xFFFFFFFFu));
        return buf;
    }

    Compositor* m_comp = nullptr;
    int m_channel = -1;
    std::string m_root;
    std::vector<std::unique_ptr<SourceTree>> m_trees;
};

// ================================================================================================
//  AuditColorTrees -- the number that decides whether this path may carry pixels.
//
//  The composed cache is the INCUMBENT's answer: tiles the shipped paint loop wrote, on real
//  addresses, from the real stack. So the audit walks that folder, keeps every tile whose
//  filename still carries the subset the compositor would use TODAY (a stale one would only be
//  measuring an old stack), and asks the tree path for the same address.
//
//  It reports the worst per-channel difference and how many texels move at all. A reference is
//  expected to be exact -- it is the same bytes. A composition is expected to be exact wherever
//  the weights are 0 or 1, and to differ by at most one LSB on a feather, which is the alpha
//  quantization and nothing else. Anything beyond that is a real disagreement about the picture
//  and the path does not ship.
// ================================================================================================
struct TreeAudit {
    uint32_t tiles = 0, exact = 0;
    uint32_t worstDelta = 0;           // largest |direct - tree| over R, G, B
    uint64_t difTexels = 0, texels = 0;
    uint32_t alphaDif = 0;
    uint32_t skippedStale = 0;
    uint32_t refs = 0, composed = 0;   // how the composite answered
};

inline void AuditColorTrees(Compositor& comp, int channel, ColorTreeStack& trees,
                            const ColorFrame& frame, uint32_t maxTiles, TreeAudit& a) {
    const Compositor::Channel& ch = comp.ChannelAt(channel);
    const std::string tag = frame.Tag();
    trees.EnsureFrame(tag);
    const std::string dir = "cache\\composed\\" + ch.name + "\\" + tag;
    WIN32_FIND_DATAA fd{};
    HANDLE h = FindFirstFileA((dir + "\\*.bin").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) {
        Log("[tree-audit] %s/%s: no composed tiles to compare against", ch.name.c_str(),
            tag.c_str());
        return;
    }
    // Enumerate the WHOLE folder first, then stride. FindFirstFile hands back names in
    // directory order, which here is dominated by the coarse mips -- and the coarse mips are
    // exactly where one source covers, so taking the first N would sample the case that is
    // trivially exact and call it a result.
    std::vector<std::string> names;
    do {
        names.push_back(fd.cFileName);
    } while (FindNextFileA(h, &fd));
    FindClose(h);
    const size_t stride = names.empty() ? 1 : (std::max)(size_t(1), names.size() / maxTiles);

    std::vector<uint8_t> direct, tree;
    for (size_t ni = 0; ni < names.size(); ni += stride) {
        if (a.tiles >= maxTiles) break;
        const std::string& fname = names[ni];
        uint32_t f = 0, m = 0, x = 0, y = 0, sub = 0;
        if (sscanf_s(fname.c_str(), "f%u_m%u_x%u_y%u_%8x.bin", &f, &m, &x, &y, &sub) != 5) continue;
        const TileRequest r{f, m, x, y};
        Compositor::TileBox box{};
        frame.Box(r, box);
        std::vector<size_t> inc;
        const uint64_t subset = comp.ColorSubset(ch, box, inc);
        if (static_cast<uint32_t>(subset & 0xFFFFFFFFu) != sub) {
            ++a.skippedStale;   // painted from a stack that is not the one running now
            continue;
        }
        if (!tree_detail::ReadTile(dir + "\\" + fname, direct)) continue;
        int refIdx = -1;
        if (!trees.Compose(frame, tag, r, tree, &refIdx) || tree.size() != direct.size()) continue;
        ++a.tiles;
        if (refIdx >= 0) ++a.refs; else ++a.composed;
        bool same = true;
        for (size_t i = 0; i < direct.size(); i += 4) {
            uint32_t d = 0;
            for (int c = 0; c < 3; ++c) {
                const int diff = std::abs(int(direct[i + c]) - int(tree[i + c]));
                d = (std::max)(d, static_cast<uint32_t>(diff));
            }
            if (direct[i + 3] != tree[i + 3]) ++a.alphaDif;
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
        "|direct - tree| = %u/255, %llu of %llu texels differ (%.4f%%), %u alpha mismatches; "
        "%u referenced a tree, %u genuinely composed; %u stale skipped",
        ch.name.c_str(), tag.c_str(), a.tiles, names.size(), stride, a.exact, a.worstDelta,
        static_cast<unsigned long long>(a.difTexels), static_cast<unsigned long long>(a.texels),
        a.texels ? 100.0 * double(a.difTexels) / double(a.texels) : 0.0, a.alphaDif, a.refs,
        a.composed, a.skippedStale);
}

}  // namespace ga
