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
#include <array>
#include <atomic>
#include <chrono>
#include <functional>
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
#include <unordered_set>

#include "compose/ColorStackSource.h"
#include "compose/ComposeTree.h"
#include "compose/Compositor.h"
#include "compose/DomainSource.h"
#include "compose/TileArchive.h"
#include "core/BuildInfo.h"
#include "core/Common.h"
#include "core/ThreadAudit.h"

namespace ga {

// Bump when the tile FORMAT or the leaf paint changes. Not when a source does -- a source's
// identity is its own, and coupling the two would orphan every tree on every edit.
// 2 (M9bb): parent mips are FOLDED from painted children (the pyramid), not only resampled.
inline constexpr int kTileTreeVersion = 2;

// One recursive mutex per tile ADDRESS, striped. 256 is far more than the twelve loader threads
// can be inside at once, so a collision is rare -- and a collision is only a wait, never a
// deadlock, because no code path here ever holds two stripes (see TileTree::Stripe; the
// StripeLock below counts it, and the tiletree selftest holds it to that count).
inline constexpr size_t kStripes = 256;

namespace tree_detail {

// THE TREE'S I/O, PER THREAD: tile reads and writes that moved bytes, existence probes, and
// refetch notices. Thread-local, so a caller can read what exactly its own call cost -- the
// tiletree selftest prices one leaf's chain this way -- and always on, because an increment is
// nothing beside the file operation it counts.
struct IoTally {
    uint64_t reads = 0, writes = 0, probes = 0, announces = 0;
};
inline IoTally& Io() {
    static thread_local IoTally t;
    return t;
}

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
// FILE_SHARE_DELETE IS THE LOAD-BEARING FLAG HERE, and it is half of the atomic publish below.
// WriteTile publishes by renaming a temp over this path, and MoveFileEx onto a file another
// thread holds open fails with ERROR_ACCESS_DENIED unless EVERY reader allowed delete-sharing.
// std::ifstream does not. MEASURED, before this was written: 366 of 400 concurrent publishes
// lost that way -- the "atomic" publish would have destroyed 91% of tile writes to buy back a
// torn read. With the flag the rename supersedes the directory entry, this handle keeps reading
// the tile it opened, and the reader gets a WHOLE OLD tile instead of a torn new one. It also
// makes DropCachedAddress's deletes stop failing against a reader, which they could before.
inline bool ReadTile(const std::string& path, std::vector<uint8_t>& out, size_t bytes = 65536) {
    threadaudit::ReadScope audit(path);
    const HANDLE h =
        CreateFileA(path.c_str(), GENERIC_READ,
                    FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;
    LARGE_INTEGER sz{};
    if (!GetFileSizeEx(h, &sz) || static_cast<uint64_t>(sz.QuadPart) != bytes) {
        CloseHandle(h);
        out.clear();
        return false;   // torn write (a crash mid-save, now the only way to get one): repaint it
    }
    out.resize(bytes);
    DWORD got = 0;
    const bool ok = ReadFile(h, out.data(), static_cast<DWORD>(bytes), &got, nullptr) &&
                    got == static_cast<DWORD>(bytes);
    CloseHandle(h);
    if (!ok) out.clear();
    if (ok) ++Io().reads;
    return ok;
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
//
// AND A TILE IS PUBLISHED WHOLE OR NOT AT ALL. Writing straight to the target truncates it at
// open and flushes the bytes at close, and every reader in this file treats a short read as
// ABSENT: ReadTile returns false (the size check at the top of this namespace), Serve falls
// through to a repaint, and FoldUp takes its ph = Held::Absent branch and repaints the parent
// FROM THE SOURCE -- discarding every sibling's fold already accumulated in it. Silent pyramid
// data loss, decided by which thread was in the window. m_foldMx did not cover it: it guards
// FoldUp on one node against another FoldUp, not against any reader, and not against LeafTile's
// own write of the same address.
//
// So: write a per-thread temp beside the target and MoveFileExA over it. The NTFS rename swaps
// one directory entry, so a reader sees the whole old tile or the whole new one and never
// neither. The temp is named ~pub<n> with no extension, which matches none of the three
// patterns anything in this codebase scans a tile folder with (*.bin, <base>.ref-*, <base>_*).
// Returns whether the tile is now ON DISK under `path`. A caller that ignores it gets the old
// behaviour; the concurrency gate in ThreadTest.cpp needs to know, because a publish a reader
// makes fail is the same lost tile as a torn one.
//
// `loud` false keeps a failure out of the log, and out of the once-only warnings below, for a
// caller that says its own failure in its own words (StampLive): those warnings speak about
// TILES, and the first failure of the run is the only one they ever print.
inline bool WriteTile(const std::string& path, const std::vector<uint8_t>& data,
                      bool loud = true) {
    threadaudit::WriteScope audit(path);
    static std::atomic<uint32_t> seq{0};
    char stem[24];
    snprintf(stem, sizeof(stem), "~pub%08x", seq.fetch_add(1, std::memory_order_relaxed));
    const size_t slash = path.find_last_of('\\');
    const std::string tmp =
        (slash == std::string::npos) ? std::string(stem) : path.substr(0, slash + 1) + stem;
    // The temp is opened with DELETE access and full sharing, and kept OPEN, because the rename
    // is issued on this handle rather than by MoveFileEx. MoveFileEx cannot replace a target
    // another thread holds open -- it fails ERROR_ACCESS_DENIED even when every reader allows
    // delete-sharing (measured: 352 of 400 publishes lost that way). A rename issued through
    // the handle with POSIX semantics supersedes the entry exactly as rename(2) does.
    const HANDLE h = CreateFileA(tmp.c_str(), GENERIC_WRITE | DELETE,
                                 FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                                 CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        static std::atomic<bool> warned{false};
        if (loud && !warned.exchange(true)) {
            Log("[tiletree] CANNOT WRITE %s (err %lu) -- painting and storing NOTHING (missing "
                "folder? disk full?)",
                path.c_str(), GetLastError());
        }
        return false;
    }
    DWORD wrote = 0;
    if (!WriteFile(h, data.data(), static_cast<DWORD>(data.size()), &wrote, nullptr) ||
        wrote != static_cast<DWORD>(data.size())) {
        static std::atomic<bool> warned{false};
        if (loud && !warned.exchange(true)) {
            Log("[tiletree] SHORT WRITE %s (%lu of %zu bytes) -- storing NOTHING", path.c_str(),
                wrote, data.size());
        }
        CloseHandle(h);
        DeleteFileA(tmp.c_str());
        return false;
    }
    // FILE_RENAME_INFO wants a fully qualified UTF-16 destination.
    char full[MAX_PATH * 2];
    if (!GetFullPathNameA(path.c_str(), sizeof(full), full, nullptr)) {
        CloseHandle(h);
        DeleteFileA(tmp.c_str());
        return false;
    }
    const int wn = MultiByteToWideChar(CP_ACP, 0, full, -1, nullptr, 0);   // includes the NUL
    std::vector<uint8_t> info(sizeof(FILE_RENAME_INFO) + static_cast<size_t>(wn) * sizeof(wchar_t));
    auto* ri = reinterpret_cast<FILE_RENAME_INFO*>(info.data());
    ri->RootDirectory = nullptr;
    ri->FileNameLength = static_cast<DWORD>((wn - 1) * sizeof(wchar_t));
    MultiByteToWideChar(CP_ACP, 0, full, -1, ri->FileName, wn);
    // POSIX semantics first (Win10 1607+); fall back to the plain replace where it is refused,
    // which is still correct whenever no reader happens to hold the target open.
    ri->Flags = 0x1 /*REPLACE_IF_EXISTS*/ | 0x2 /*POSIX_SEMANTICS*/;
    bool ok = SetFileInformationByHandle(h, static_cast<FILE_INFO_BY_HANDLE_CLASS>(22),
                                         info.data(), static_cast<DWORD>(info.size())) != 0;
    if (!ok) {
        ri->ReplaceIfExists = TRUE;
        ok = SetFileInformationByHandle(h, FileRenameInfo, info.data(),
                                        static_cast<DWORD>(info.size())) != 0;
    }
    const DWORD err = ok ? 0 : GetLastError();
    CloseHandle(h);
    if (!ok) {
        static std::atomic<bool> warned{false};
        if (loud && !warned.exchange(true)) {
            Log("[tiletree] CANNOT PUBLISH %s (err %lu) -- the tile was written and then LOST",
                path.c_str(), err);
        }
        DeleteFileA(tmp.c_str());
        return false;
    }
    ++Io().writes;
    return true;
}
// Markers (.void, .fold, .ref-<id>) do NOT need the temp-and-rename above, and it would only
// cost them a second file operation: the only thing anyone ever reads from a marker is whether
// it EXISTS (Peek/PeekAt/FindRef call GetFileAttributesA, never open it), and an ofstream that
// creates-or-truncates a zero-byte file leaves it existing throughout. There is no window in
// which the marker is half a marker. (The window that DOES exist -- Delete(.void) followed by
// Touch(.fold) in FoldUp, where a reader between them sees neither -- is an ordering problem
// and no atomic write fixes it.)
inline void Touch(const std::string& path) {
    threadaudit::WriteScope audit(path);
    std::ofstream f(path, std::ios::binary);   // zero bytes: the entry IS the record
    if (!f) {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) Log("[tiletree] CANNOT WRITE %s", path.c_str());
    }
}
inline bool Exists(const std::string& path) {
    ++Io().probes;
    return GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES;
}
inline void MakeDir(const std::string& path) { CreateDirectoryA(path.c_str(), nullptr); }
// Every folder on the way down to `path`, then `path` itself: a tree rooted outside
// cache\trees (a scratch root) may name folders that do not exist yet.
inline void MakeDirs(const std::string& path) {
    for (size_t i = path.find('\\'); i != std::string::npos; i = path.find('\\', i + 1)) {
        if (i > 0 && path[i - 1] != ':') MakeDir(path.substr(0, i));
    }
    MakeDir(path);
}
// A folder as this file spells it. WriteTile finds a tile's folder by its last backslash, so a
// root given with forward slashes would put every publish's temp file in the working folder.
inline std::string Backslashes(std::string s) {
    for (char& c : s) {
        if (c == '/') c = '\\';
    }
    while (!s.empty() && s.back() == '\\') s.pop_back();
    return s;
}
// Every unlink goes through here: DropCachedAddress deletes tiles on the PARENT node under no
// lock, and a delete inside someone else's read is the same collision a torn write is.
inline void Delete(const std::string& path) {
    threadaudit::Deleted(path);
    DeleteFileA(path.c_str());
}
// The one wildcard lookup a stored reference costs: "<base>.ref-*" -> the suffix.
inline bool FindRef(const std::string& base, std::string& childId) {
    ++Io().probes;
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

// ---- THE TREE SAYS WHEN IT WAS LAST USED ------------------------------------------------------
// The prune tool (compose/TreePrune.h) has to know which tag folders a run still uses, and no
// file time can tell it: NTFS keeps no usable last-access time, and a tree that is fully painted
// is read every day and written never. So every run STAMPS each tag folder it ensures --
// <node>.<id>\<tag>\.live, one line of JSON: the UTC time, the engine's rev (the [boot] line's)
// and the scene -- published through WriteTile's temp-and-rename, so a reader sees the whole old
// stamp or the whole new one. The unit is the TAG folder, not the node's: when the engine stops
// using a lattice, that lattice's folders go stale inside a node folder that is still live.
//
// THE NAME MATCHES NOTHING THAT SCANS A TILE FOLDER. Every scanner, checked: the packer and the
// index take *.bin; FindRef takes <base>.ref-* and DropCachedAddress <base>_*, where <base> is
// f<face>_m<mip>_x<x>_y<y>; tools/hierarchy/tree_census.py counts names matching
// ^f(\d+)_m(\d+)_x(\d+)_y(\d+)...; finding 25's archive audit opens only <tag>.gaa, at the node
// level. A name that starts with a dot starts none of those, and ".live" ends in none of them.
//
// AT MOST ONCE PER FOLDER PER RUN. The tenant dispatcher reads a holder's tree per request
// (hal/Tenant.cpp), so EnsureFrame runs on every tile of the wave and exposure trees; the set
// below turns that into one file write per folder. A stamp that cannot be written is said once
// and is never fatal: the tool judges an unstamped folder by its newest write, at twice the age.
struct LiveStamps {
    std::mutex mx;
    std::unordered_set<std::string> done;   // the folders stamped this run
    std::string scene, file;                // what every stamp names (SetLiveScene)
    bool warned = false;
};
inline LiveStamps& Live() {
    static LiveStamps s;
    return s;
}
// main() names the scene once, when it has resolved and before any tree exists.
inline void SetLiveScene(const std::string& scene, const std::string& file) {
    LiveStamps& l = Live();
    std::lock_guard<std::mutex> lk(l.mx);
    l.scene = scene;
    l.file = file;
}
// A JSON string. A scene path typed on Windows carries backslashes, which JSON forbids bare.
inline std::string JsonQuote(const std::string& s) {
    std::string o = "\"";
    for (const char c : s) {
        const unsigned u = static_cast<unsigned char>(c);
        if (u < 0x20) {
            char b[8];
            snprintf(b, sizeof(b), "\\u%04x", u);
            o += b;
            continue;
        }
        if (c == '"' || c == '\\') o += '\\';
        o += c;
    }
    return o + "\"";
}
inline void StampLive(const std::string& folder) {
    LiveStamps& l = Live();
    std::string line;
    {
        std::lock_guard<std::mutex> lk(l.mx);
        if (!l.done.insert(folder).second) return;
        SYSTEMTIME t{};
        GetSystemTime(&t);
        char utc[32];
        snprintf(utc, sizeof(utc), "%04u-%02u-%02uT%02u:%02u:%02uZ", t.wYear, t.wMonth, t.wDay,
                 t.wHour, t.wMinute, t.wSecond);
        line = "{\"utc\": " + JsonQuote(utc) + ", \"rev\": " + JsonQuote(BuildGitRev()) +
               ", \"scene\": " + JsonQuote(l.scene) + ", \"file\": " + JsonQuote(l.file) + "}\n";
    }
    if (WriteTile(folder + "\\.live", std::vector<uint8_t>(line.begin(), line.end()), false)) {
        return;
    }
    std::lock_guard<std::mutex> lk(l.mx);
    if (!l.warned) {
        l.warned = true;
        Log("[tiletree] cannot stamp %s\\.live -- the tree works as before; the prune tool will "
            "judge this folder by its newest write",
            folder.c_str());
    }
}

// THE STRIPE INSTRUMENT. Stripe()'s rule is that a thread holds at most one stripe at a time;
// every stripe is taken through StripeLock, which keeps the stripes its thread holds and counts
// the acquisition of a second DISTINCT one. Taking the stripe already held again is the
// recursive mutex doing its job, not a nesting, and is not counted. `maxHeld` is the most any
// thread held at once; the tiletree selftest requires 1, and its planted failure -- the fold
// walk this replaced, kept behind TileTree::FoldWalkForTest -- must drive it past 1.
struct StripeTally {
    std::atomic<uint32_t> maxHeld{0};
    std::atomic<uint64_t> nested{0};   // acquisitions made with another stripe already held
    void Reset() {
        maxHeld.store(0);
        nested.store(0);
    }
};
inline StripeTally& Stripes() {
    static StripeTally t;
    return t;
}
class StripeLock {
public:
    explicit StripeLock(std::recursive_mutex& m) : m_m(m) {
        m_m.lock();
        for (int i = 0; i < Depth() && i < kMaxHeld; ++i) {
            if (Held()[i] == &m) return;   // re-entered: one stripe, held twice
        }
        if (Depth() > 0) Stripes().nested.fetch_add(1, std::memory_order_relaxed);
        if (Depth() < kMaxHeld) Held()[Depth()] = &m;
        const uint32_t d = uint32_t(++Depth());
        uint32_t was = Stripes().maxHeld.load(std::memory_order_relaxed);
        while (d > was && !Stripes().maxHeld.compare_exchange_weak(was, d)) {
        }
        m_counted = true;
    }
    ~StripeLock() {
        if (m_counted) --Depth();   // scoped, so the last one counted is the first released
        m_m.unlock();
    }
    StripeLock(const StripeLock&) = delete;
    StripeLock& operator=(const StripeLock&) = delete;

private:
    static constexpr int kMaxHeld = 64;
    static int& Depth() {
        static thread_local int d = 0;
        return d;
    }
    static const std::recursive_mutex** Held() {
        static thread_local const std::recursive_mutex* h[kMaxHeld] = {};
        return h;
    }
    std::recursive_mutex& m_m;
    bool m_counted = false;
};

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
    // Raw4 (M9bc): four quantized channels, no coverage byte -- a tile-native node's planes
    // (the wave field's a, k, cos, sin); absence is the tile's absence. 128x128, 64 KB, a GPU tile.
    enum class Fmt : uint8_t { Rgba8, FloatW, Half, Raw4 };

    // Builds the tree of caches under this node, recursively. Every compose and gate node gets
    // its own folder; every leaf gets one. Nodes are borrowed -- the graph outlives the caches.
    // `trees` is the folder the node folders go in: a child takes its parent's, and a root
    // given none takes TreeRoot() -- the scene's streaming.treeRoot, cache\trees by default. The
    // selftest's trees are given scratch folders, so no tile of theirs lands in the real cache.
    explicit TileTree(const DomainSource* node, Fmt fmt = Fmt::Rgba8, TileTree* parent = nullptr,
                      const std::string& trees = std::string())
        : m_node(node), m_fmt(fmt), m_parent(parent) {
        if (!parent) m_stripes = std::make_unique<std::array<std::recursive_mutex, kStripes>>();
        m_trees = parent ? parent->m_trees
                         : tree_detail::Backslashes(trees.empty() ? TreeRoot() : trees);
        const std::string kind = node->NodeKind();
        if (kind == "compose" || kind == "gate") {
            // Under a scalar root every input is FloatW: coverage rides with the value.
            const Fmt kidFmt = (fmt == Fmt::Rgba8) ? Fmt::Rgba8 : Fmt::FloatW;
            for (size_t i = 0; i < node->InputCount(); ++i) {
                m_kids.push_back(std::make_unique<TileTree>(node->Input(i), kidFmt, this));
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
            "#" + std::to_string(kTileTreeVersion) +
                (fmt == Fmt::Rgba8 ? "" : fmt == Fmt::FloatW ? "fw" : fmt == Fmt::Half ? "h" : "r4"));
        m_id = tree_detail::Hex8(h);
        m_root = m_trees + "\\" + tree_detail::Sanitize(node->Name()) + "." + m_id;
        tree_detail::MakeDirs(m_trees);
        tree_detail::MakeDir(m_root);
    }

    // WHERE THE TREES LIVE when a root is built without a folder of its own. Set once, by the
    // Assembly from the scene's streaming.treeRoot, before the first tree exists; the loader
    // threads that build trees later only read it.
    static void SetTreeRoot(const std::string& trees) { TreeRootRef() = tree_detail::Backslashes(trees); }
    static std::string TreeRoot() { return TreeRootRef(); }
    // This node's folder: <trees>\<name>.<id>, a frame's tiles in <tag> below it.
    const std::string& Folder() const { return m_root; }

    const std::string& Id() const { return m_id; }
    // M9bb: THE PYRAMID. Fires when a tile of THIS node changed under a consumer that already
    // holds it -- a child's paint folded into a parent, or a descendant's fold made a cached
    // composite stale. The residency manager hangs its per-tile refetch here. Called from the
    // painting threads: the hook must be cheap and must queue, not touch GPU state.
    std::function<void(const std::string& tag, const TileRequest& r)> onChanged;
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
        tree_detail::StampLive(m_root + "\\" + tag);   // this run uses the folder (tree_detail)
        for (auto& k : m_kids) k->EnsureFrame(tag);
    }

    // ---- M12 step 2d: the lattice is INHERITED ---------------------------------------------
    // THE FOLD, one step: a node's lattice is its own declaration (DomainSource::OwnLattice),
    // else the one it inherits; the root's is what the tenant bound. nullptr is the identity,
    // so with no declaration anywhere every node sits on the tenant's lattice -- exactly what
    // passing the same value down every call did, now a rule instead of a habit. Every public
    // entry point resolves once and hands the RESOLVED lattice to its children, so a grandchild
    // inherits the resolved value: "first declaration walking up" falls out of composition.
    const ColorFrame& Resolve(const ColorFrame& inherited) const {
        const Lattice* own = m_node->OwnLattice();
        return own ? *own : inherited;
    }
    // The tenant binds the root's lattice once (step 3e); Provider() with no argument is the
    // tree on that lattice.
    void SetLattice(const Lattice* l) { m_lattice = l; }
    const Lattice* BoundLattice() const { return m_lattice; }
    TileProviderFn Provider() {
        if (!m_lattice) {
            Log("[tree] %s: Provider() before SetLattice -- no lattice bound", m_node->Name());
            return [](const TileRequest&, std::vector<uint8_t>& out, TileLoc* loc) {
                out.assign(65536, 0);
                if (loc) *loc = TileLoc{};
                return true;
            };
        }
        return Provider(*m_lattice);
    }

    // ---- what is held here, from the directory alone ----------------------------------------
    // For a leaf: two lookups. For a compose node: its children's answers decide the key, so if
    // any child is Absent this node cannot yet know its own key and answers Absent.
    Held Peek(const ColorFrame& inherited, const std::string& tag, const TileRequest& r) {
        const ColorFrame& frame = Resolve(inherited);
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
    Status Tile(const ColorFrame& inherited, const std::string& tag, const TileRequest& r,
                std::vector<uint8_t>& out, TileLoc* loc = nullptr) {
        const ColorFrame& frame = Resolve(inherited);
        Compositor::TileBox box{};
        frame.Box(r, box);
        if (m_kids.empty()) return LeafTile(frame, tag, r, box, out, loc);
        return std::string(m_node->NodeKind()) == "gate" ? GateTile(frame, tag, r, box, out, loc)
                                                         : ComposeTile(frame, tag, r, box, out, loc);
    }

    // The residency manager's view of this tree: a place if it can take one, else bytes, else
    // zeros where nothing covers.
    TileProviderFn Provider(const ColorFrame& inherited) {
        const ColorFrame frame = Resolve(inherited);   // by value: the closure carries it
        const std::string tag = frame.Tag();
        EnsureFrame(tag);
        return [this, frame, tag](const TileRequest& r, std::vector<uint8_t>& out, TileLoc* loc) {
            // PHASE A4 (HIERARCHY 4.20, the third clause): a tile finer than every source's own
            // level is the level above, magnified. It has no bytes and nothing is mapped for it
            // (g_tileMagnified): the reader's residency byte names the parent, and the sampler
            // magnifies it.
            if (Magnified(frame, r)) {
                g_tileMagnified = true;
                out.clear();
                if (loc) *loc = TileLoc{};
                ++magnified;
                return true;
            }
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
    // F15 (instrument): the leaf's steps on the clock (microseconds summed, calls), see LeafTile.
    std::atomic<uint64_t> usArchive{0}, usProbe{0}, usRead{0}, usFold{0}, usPaint{0}, usPublish{0}, usFoldUp{0};
    std::atomic<uint32_t> nArchive{0}, nProbe{0}, nRead{0}, nFold{0}, nPaint{0}, nPublish{0}, nFoldUp{0};
    std::atomic<uint32_t> folded{0}, dropped{0};   // M9bb: pyramid folds; cached addresses dropped
    std::atomic<uint32_t> redundant{0};   // M9bd: folds the 4/255 rule found moved nothing
    std::atomic<uint32_t> magnified{0};   // PHASE A4: tiles answered as their parent, magnified

    // PHASE A4: THE FINEST LEVEL ANY SOURCE UNDER THIS NODE HAS OF ITS OWN over a box: a leaf's
    // FinestMip there, a compose the finest of the children that touch the box (the membership
    // KidsHeld keys with), -1 if any of them has every level, kNoSource if none touches. A GATE
    // paints nothing -- it multiplies the weight of what it gates, texel by texel -- so it has no
    // level of its own: its level is its layer's (child 0), however exact the gate (the mask's own
    // tenant, where the vector paints coverage, keeps every level).
    static constexpr int kNoSource = 0x7FFFFFFF;
    int FinestAt(const ColorFrame& inherited, const Compositor::TileBox& box) const {
        const ColorFrame& frame = Resolve(inherited);
        if (m_kids.empty()) {
            return Touching(box) ? m_node->FinestMip(frame, box.latMin, box.latMax, box.lonMin, box.lonMax)
                                 : kNoSource;
        }
        const bool gate = std::string(m_node->NodeKind()) == "gate";
        int f = kNoSource;
        for (size_t i = 0; i < (gate ? size_t(1) : m_kids.size()); ++i) {
            if (!KidIn(i, box)) continue;
            const int k = m_kids[i]->FinestAt(frame, box);
            if (k < 0) return -1;
            f = (std::min)(f, k);
        }
        return f;
    }
    // A tile finer than that level: its parent, magnified.
    bool Magnified(const ColorFrame& inherited, const TileRequest& r) const {
        Compositor::TileBox box{};
        Resolve(inherited).Box(r, box);
        const int f = FinestAt(inherited, box);
        return f != kNoSource && int(r.mip) < f;
    }
    // F14: the same question as a function a binding declares beside its provider (the frame
    // resolved once and carried, as Provider carries it), for the order to ask before a load.
    TileQueryFn MagnifiedQuery(const ColorFrame& inherited) {
        const ColorFrame frame = Resolve(inherited);
        return [this, frame](const TileRequest& r) { return Magnified(frame, r); };
    }
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
        if (magnified.load()) {
            snprintf(b, sizeof(b), " | %u magnified (no bytes: the parent)", magnified.load());
            s += b;
        }
        if (nArchive.load()) {   // F15: ms a call, by step (calls)
            auto ms = [](const std::atomic<uint64_t>& us, const std::atomic<uint32_t>& n) {
                return n.load() ? double(us.load()) / (1000.0 * n.load()) : 0.0;
            };
            snprintf(b, sizeof(b),
                     " | leaf ms a call: archive %.2f (%u), probe %.2f (%u), read %.2f (%u), fold %.1f (%u), "
                     "paint %.1f (%u), publish %.1f (%u), fold-up %.1f (%u)",
                     ms(usArchive, nArchive), nArchive.load(), ms(usProbe, nProbe), nProbe.load(),
                     ms(usRead, nRead), nRead.load(), ms(usFold, nFold), nFold.load(), ms(usPaint, nPaint),
                     nPaint.load(), ms(usPublish, nPublish), nPublish.load(), ms(usFoldUp, nFoldUp), nFoldUp.load());
            s += b;
        }
        // A leaf over a source that fetches says what it could not fetch: with the budget at
        // zero, this is the number of fetches the run WOULD have made.
        uint32_t refused = 0;
        if (m_raw && m_raw->Refusals(refused)) {
            snprintf(b, sizeof(b), " | %u source tiles refused (fetch budget)", refused);
            s += b;
        }
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
    TileTree* Root() {
        TileTree* t = this;
        while (t->m_parent) t = t->m_parent;
        return t;
    }
    // ONE LOCK PER TILE ADDRESS, striped. m_foldMx was per NODE and covered only FoldUp, so it
    // serialized every fold on a node against every other -- and covered neither LeafTile's own
    // publish of the same address nor any reader. A fold is a read-modify-write of the parent
    // tile; a paint of that same parent address running beside it is a LOST UPDATE, and the
    // fold is the answer that gets lost (M9bb: a coarse texel is the fold of the finer level
    // wherever one exists, and the source's own resample only where none was painted).
    //
    // THE RULE THAT MAKES STRIPING SAFE: hold AT MOST ONE STRIPE AT A TIME, and it is always the
    // stripe of the address being written. Two different addresses can collide onto one stripe
    // -- that is what striping is -- so any scheme that holds two at once could deadlock on a
    // collision no matter how the acquisition order is argued. Holding one cannot. That is why
    // FoldUp takes each level's stripe in turn and releases it before the level above, why an
    // absent parent is painted inside its fold without folding itself upward from there, and why
    // DropCachedAddress takes no stripe at all: after the atomic publish a delete leaves no
    // torn state, so a reader either finds the tile or recomposes it, and both are correct.
    // (tree_detail::StripeLock counts it.)
    std::recursive_mutex& Stripe(const std::string& tag, const TileRequest& r) {
        TileTree* root = Root();
        return (*root->m_stripes)[StripeOf(tag, this, r)];
    }

public:
    // The stripe of (tag, node, address). EACH FIELD IS ITS OWN WORD: the one word this packed
    // before -- face << 44 | mip << 40 | y << 20 | x -- ORs its fields over each other once x or
    // y reach 2^20, which the pyramid's 2^24 tiles a side do (y = 1 with x = 0 and y = 1 with
    // x = 2^20 were one input), so two addresses differing in one field hashed identically. The
    // node is mixed too: compose walks to a child at the same address and must not self-lock.
    static void StripeWords(const TileRequest& r, uint64_t w[4]) {
        w[0] = r.face;
        w[1] = r.mip;
        w[2] = r.x;
        w[3] = r.y;
    }
    static size_t StripeOf(const std::string& tag, const void* node, const TileRequest& r) {
        uint64_t h = tree_detail::Fnv1a(1469598103934665603ull, tag);
        auto mix = [&h](uint64_t v) {
            for (int i = 0; i < 8; ++i) {
                h ^= static_cast<uint8_t>(v >> (i * 8));
                h *= 1099511628211ull;
            }
        };
        mix(reinterpret_cast<uintptr_t>(node));
        uint64_t w[4];
        StripeWords(r, w);
        for (const uint64_t v : w) mix(v);
        return size_t(h % kStripes);
    }

private:

    // Returns a SHARED pointer, not a raw one. The raw pointer escaped m_arcMx while Pack()
    // could m_arcs.clear() under it, so a worker inside FromArchive held a dangling archive --
    // narrow (only --pack-trees clears, and it exits straight after) but free to close.
    std::shared_ptr<TileArchive> Archive(const std::string& tag) {
        std::lock_guard<std::mutex> lk(m_arcMx);
        auto it = m_arcs.find(tag);
        if (it == m_arcs.end()) {
            auto a = std::make_shared<TileArchive>();
            const bool ok = a->OpenPath(ArchivePath(tag));
            it = m_arcs.emplace(tag, std::move(a)).first;
            if (ok) {
                Log("[tiletree] %s/%s: archive open, %zu tiles answer as places", m_node->Name(),
                    tag.c_str(), it->second->Count());
            }
        }
        return it->second->Valid() ? it->second : nullptr;
    }
    // Try the archive for (r, subset). A caller with `loc` gets a place; one without gets the
    // bytes through the archive's one handle. False = not archived: fall through to the files.
    bool FromArchive(const std::string& tag, const TileRequest& r, uint32_t subset,
                     std::vector<uint8_t>& out, TileLoc* loc) {
        if (m_fmt == Fmt::FloatW) loc = nullptr;   // 256 KB on disk is not a GPU tile: bytes only
        const std::shared_ptr<TileArchive> arc = Archive(tag);
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
        if (tree_detail::Exists(base + ".fold")) return Held::Content;   // derived from children
        std::string id;
        if (tree_detail::FindRef(base, id)) return Held::Content;
        return Held::Absent;
    }

    // The SAME membership test the incumbent uses -- a declared footprint must span ~2 texels of
    // this tile to be in its subset; an undeclared one is in where it may cover.
    bool KidIn(size_t i, const Compositor::TileBox& box) const {
        constexpr double kR2D = 180.0 / 3.14159265358979;
        double fl0, fb0, fl1, fb1;
        return m_kids[i]->m_node->Footprint(fl0, fb0, fl1, fb1)
                   ? Compositor::Touches(fl0, fb0, fl1, fb1, box)
                   : m_kids[i]->m_node->MayCover(box.lonMin * kR2D, box.latMin * kR2D,
                                                 box.lonMax * kR2D, box.latMax * kR2D);
    }
    // Which children can touch this tile, and what each holds. False if any is still Absent.
    bool KidsHeld(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                  const Compositor::TileBox& box, std::vector<size_t>& inc,
                  std::vector<Held>& held) {
        bool all = true;
        for (size_t i = 0; i < m_kids.size(); ++i) {
            if (!KidIn(i, box)) continue;   // absent: not in this tile's subset at all
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
    // `own`, when given, asks for the tile as the fold needs an absent parent: painted, and
    // neither published nor folded upward from here -- FoldInto publishes it once, after the
    // fold, and its own paint rises with the fold's walk. *own says whether this call painted
    // (true) or served what was already there (false).
    Status LeafTile(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                    const Compositor::TileBox& box, std::vector<uint8_t>& out, TileLoc* loc,
                    bool* own = nullptr) {
        const std::string base = Base(tag, r);
        if (own) *own = false;
        // F15 (instrument): the leaf's steps on the clock, summed per node (Stats). A mask load
        // measured 35-57 ms at the loader with the sweep itself not the cost; these say which
        // step is -- the archive probe, the directory probes, the read, the paint, the publish.
        using LeafClock = std::chrono::steady_clock;
        LeafClock::time_point tc = LeafClock::now();
        auto lap = [&](std::atomic<uint64_t>& us, std::atomic<uint32_t>& n) {
            const LeafClock::time_point t = LeafClock::now();
            us += uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(t - tc).count());
            ++n;
            tc = t;
        };
        const bool arc = FromArchive(tag, r, 0u, out, loc);
        lap(usArchive, nArchive);
        if (arc) {
            ++read;
            return Status::Content;
        }
        const bool isVoid = tree_detail::Exists(base + ".void");
        lap(usProbe, nProbe);
        if (isVoid) {
            ++hits;
            return Status::Void;
        }
        const bool got = tree_detail::ReadTile(base + ".bin", out, TileBytes());
        lap(usRead, nRead);
        if (got) {
            ++read;
            return Status::Content;
        }
        const bool isFold = tree_detail::Exists(base + ".fold");
        lap(usProbe, nProbe);
        if (isFold) {
            // M9bd: a REDUNDANT parent -- within 4/255 of the fold of its children -- was never
            // stored; rebuild it from them every time it is asked. Nothing is written.
            const Status s = FoldFromChildren(frame, tag, r, out, false);
            lap(usFold, nFold);
            return s;
        }
        // HIERARCHY 4.20: A SOURCE PAINTS ITS OWN LEVEL, AND THE TREE MAKES THE OTHERS. Above the
        // node's own mip a tile is the fold of its four children, kept, and never painted: its
        // bytes are a function of the file, not of what was flown or in what order. A tile the
        // footprint does not touch (the compositor's rule) is void by arithmetic: nothing is read
        // or written for it. Below the own mip the node paints, magnifying its own texels.
        const int ownMip = m_node->OwnMip(frame);
        if (ownMip >= 0 && int(r.mip) > ownMip) {
            if (!Touching(box)) return Status::Void;
            const Status s = FoldFromChildren(frame, tag, r, out, true);
            if (s == Status::Void) {
                tree_detail::Touch(base + ".void");
                ++voids;
            }
            lap(usFold, nFold);
            return s;
        }
        bool complete = true, anyCover = false, full = false;
        // The node's inputs must hold still for the whole tile (DomainSource::BeginTile).
        m_node->BeginTile();
        if (m_node->TileNative()) {
            // M9bc: a regional node paints the whole tile; quantize per the tree's format.
            std::vector<DomainValue> vals;
            if (!m_node->PaintTile(frame, r, frame.texW, frame.texH, vals) ||
                vals.size() != size_t(frame.texW) * frame.texH) {
                m_node->EndTile();
                return Status::Transient;   // not ready for this identity: ask again
            }
            out.assign(TileBytes(), 0);
            for (size_t i = 0; i < vals.size(); ++i) {
                const DomainValue& v = vals[i];
                if (v.weight <= 0.0f) continue;
                // "Write nothing": a Raw4 texel that is all zero adds no coverage -- a tile of
                // zeros (a gated component, land) is a void marker, never 64 KB of nothing.
                if (m_fmt == Fmt::Raw4 &&
                    v.c[0] <= 0.0f && v.c[1] <= 0.0f && v.c[2] <= 0.0f && v.c[3] <= 0.0f) {
                    continue;
                }
                anyCover = true;
                if (m_fmt == Fmt::FloatW) {
                    float* d = reinterpret_cast<float*>(out.data()) + i * 2;
                    d[0] = v.c[0];
                    d[1] = (std::min)(1.0f, v.weight);
                } else if (m_fmt == Fmt::Raw4) {
                    for (int q = 0; q < 4; ++q) {
                        out[i * 4 + q] = static_cast<uint8_t>(
                            (std::max)(0.0f, (std::min)(1.0f, v.c[q])) * 255.0f + 0.5f);
                    }
                } else {
                    for (int q = 0; q < 3; ++q) {
                        out[i * 4 + q] = static_cast<uint8_t>(
                            (std::max)(0.0f, (std::min)(1.0f, v.c[q])) * 255.0f + 0.5f);
                    }
                    out[i * 4 + 3] = static_cast<uint8_t>((std::min)(1.0f, v.weight) * 255.0f + 0.5f);
                }
            }
        } else if (m_fmt == Fmt::FloatW) {
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
        // The snapshot went stale while this tile painted: its texels are two different fields
        // and the identity it would be stored under names only one of them. Never cached.
        const bool ended = m_node->EndTile();
        lap(usPaint, nPaint);
        if (!ended) return Status::Transient;
        if (!complete) return Status::Transient;   // never cached; the next run repaints
        ++painted;
        if (!anyCover) {
            tree_detail::Touch(base + ".void");
            ++voids;
            return Status::Void;
        }
        if (own) {
            *own = true;   // the caller holds this address's stripe and publishes it itself
            return Status::Content;
        }
        // PUBLISH ONLY IF NOTHING IS THERE, under this address's stripe. Residency asks for a
        // mip and its parent in the same turn, so two threads can be inside one address at once,
        // and a FOLD may have published this very address while this paint ran. The fold is the
        // better answer; overwriting it with the source's own resample is exactly the lost
        // update m_foldMx could not see, because LeafTile wrote outside it. Adopting what is
        // there is also correct for the ordinary case: two painters of one address compute the
        // same bytes, and whoever published already folded it upward.
        {
            tree_detail::StripeLock lk(Stripe(tag, r));
            std::vector<uint8_t> already;
            if (tree_detail::ReadTile(base + ".bin", already, TileBytes())) {
                out.swap(already);
                ++hits;
                lap(usPublish, nPublish);
                return Status::Content;
            }
            tree_detail::WriteTile(base + ".bin", out);
        }
        lap(usPublish, nPublish);
        // Outside the stripe: FoldUp takes the PARENT's. A node with an own mip folds nothing
        // upward: its parents are folded whole, when asked (above).
        if (ownMip < 0 && !m_node->LevelsOwn()) FoldUp(frame, tag, r, out);
        lap(usFoldUp, nFoldUp);
        return Status::Content;
    }
    // The compositor's membership rule (Compositor::Touches) against the node's footprint; a node
    // that declares none touches every tile.
    bool Touching(const Compositor::TileBox& b) const {
        double l0, b0, l1, b1;
        return !m_node->Footprint(l0, b0, l1, b1) || Compositor::Touches(l0, b0, l1, b1, b);
    }

    // ---- M9bb: THE PYRAMID, IN THE COMPOSITOR. A leaf's freshly painted tile is folded into
    // its parent's quadrant (2x2 box, coverage-weighted -- the fold law: average the answers),
    // and that parent into its own, up to the frame's coarsest mip. A parent that does not
    // exist yet is painted from the source first (as every mip always was), then overwritten
    // where children exist; so a coarse mip is the source's own resample where nothing finer
    // has been painted, and exactly the fold of the finer level where it has. Composites above
    // are not folded here: they compose their children's (folded) parents through the same
    // OverStep, so fold(compose) and compose(fold) meet up to the over's own nonlinearity, and
    // their cached tiles at the changed address are dropped so they recompose.
    static uint32_t MaxMip(const ColorFrame& f) {
        uint32_t m = 0;
        for (uint32_t d = f.faceDim; d > f.texW; d >>= 1) ++m;
        return m;
    }
    void FoldQuadrant(const ColorFrame& frame, const TileRequest& child,
                      const std::vector<uint8_t>& src, std::vector<uint8_t>& parent) const {
        const uint32_t W = frame.texW, H = frame.texH, hw = W / 2, hh = H / 2;
        const uint32_t ox = (child.x & 1u) * hw, oy = (child.y & 1u) * hh;
        if (m_fmt == Fmt::FloatW) {
            const float* s = reinterpret_cast<const float*>(src.data());
            float* d = reinterpret_cast<float*>(parent.data());
            for (uint32_t py = 0; py < hh; ++py) {
                for (uint32_t px = 0; px < hw; ++px) {
                    double vw = 0.0, wsum = 0.0;
                    for (uint32_t k = 0; k < 4; ++k) {
                        const size_t i = (size_t(2 * py + (k >> 1)) * W + (2 * px + (k & 1))) * 2;
                        vw += double(s[i]) * s[i + 1];
                        wsum += s[i + 1];
                    }
                    const size_t o = (size_t(oy + py) * W + (ox + px)) * 2;
                    d[o] = wsum > 0.0 ? float(vw / wsum) : 0.0f;
                    d[o + 1] = float(wsum * 0.25);
                }
            }
            return;
        }
        if (m_fmt == Fmt::Raw4) {
            // Four channels, componentwise means -- on (cos, sin) this IS the spinor blend law.
            for (uint32_t py = 0; py < hh; ++py) {
                for (uint32_t px = 0; px < hw; ++px) {
                    uint32_t acc[4] = {0, 0, 0, 0};
                    for (uint32_t k = 0; k < 4; ++k) {
                        const uint8_t* s = &src[(size_t(2 * py + (k >> 1)) * W + (2 * px + (k & 1))) * 4];
                        for (int q = 0; q < 4; ++q) acc[q] += s[q];
                    }
                    uint8_t* d = &parent[(size_t(oy + py) * W + (ox + px)) * 4];
                    for (int q = 0; q < 4; ++q) d[q] = static_cast<uint8_t>((acc[q] + 2u) / 4u);
                }
            }
            return;
        }
        if (m_fmt != Fmt::Rgba8) return;   // Half tiles are roots; they compose, never fold
        for (uint32_t py = 0; py < hh; ++py) {
            for (uint32_t px = 0; px < hw; ++px) {
                double c[3] = {0, 0, 0}, a = 0.0;
                for (uint32_t k = 0; k < 4; ++k) {
                    const uint8_t* s = &src[(size_t(2 * py + (k >> 1)) * W + (2 * px + (k & 1))) * 4];
                    const double w = s[3] / 255.0;
                    for (int q = 0; q < 3; ++q) c[q] += s[q] * w;
                    a += w;
                }
                uint8_t* d = &parent[(size_t(oy + py) * W + (ox + px)) * 4];
                if (a > 0.0) {
                    for (int q = 0; q < 3; ++q) {
                        d[q] = static_cast<uint8_t>((std::max)(0.0, (std::min)(255.0, c[q] / a)));
                    }
                }
                d[3] = static_cast<uint8_t>((std::min)(1.0, a * 0.25) * 255.0 + 0.5);
            }
        }
    }
    // THE CHAIN IS WALKED ONCE, one level and one stripe at a time. What rises from a level to
    // the one above is not one tile but the VERSIONS that level took on this walk, in order: its
    // own paint first, when it was absent and had to be painted, then each fold that moved it.
    // That is the sequence the walk this replaced produced -- it painted an absent parent with
    // LeafTile, which published it and folded its own paint up to the root before the child's
    // fold went up the same chain again -- and each level's redundancy test is decided against
    // that sequence, so the same folds in the same order leave the same bytes. What changed is
    // the cost of a cold chain k levels long: up to k stripes nested and k(k+3)/2 writes before,
    // one stripe and k writes now. MEASURED (the tiletree selftest, the 25-level pyramid): one
    // cold FloatW leaf 15 levels below its root cost 135 ancestor writes and held 15 stripes at
    // once; now 15 and one. The read-modify-publish of ONE address happens under ONE stripe,
    // inside FoldInto; the announcement and the step to the next level happen out here, with
    // nothing held.
    void FoldUp(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                const std::vector<uint8_t>& child) {
        if (FoldWalk() == kWalkNested) {
            FoldUpNested(frame, tag, r, child);
            return;
        }
        if (m_fmt == Fmt::Half) return;
        std::vector<std::vector<uint8_t>> rising{child};
        TileRequest c = r;
        while (!rising.empty() && c.mip < MaxMip(frame)) {
            const TileRequest p{c.face, c.mip + 1, c.x / 2, c.y / 2};
            std::vector<std::vector<uint8_t>> next;
            if (FoldInto(frame, tag, c, p, rising, next)) InvalidateAbove(tag, p);
            if (FoldStepHook()) FoldStepHook()(p);
            rising.swap(next);
            c = p;
        }
    }

    // Folds the versions `rising` of the child `c`, in order, into its parent `p`, and publishes
    // p once. `next` receives the versions p took, for the level above; the return says whether
    // a fold moved p, so whoever holds it refetches. Holds exactly one stripe: p's, for the
    // whole read-modify-write -- which is what makes the fold atomic against a LeafTile
    // publishing the same address.
    bool FoldInto(const ColorFrame& frame, const std::string& tag, const TileRequest& c,
                  const TileRequest& p, std::vector<std::vector<uint8_t>>& rising,
                  std::vector<std::vector<uint8_t>>& next) {
        if (m_node->TileNative()) {
            MarkAncestors(frame, tag, c, p);
            return false;   // the marker chain IS the whole walk; nothing carries
        }
        const std::string pbase = Base(tag, p);
        if (tree_detail::Exists(pbase + ".fold")) {
            // The parent is derived from its children: nothing to rewrite, but whoever holds
            // it must refetch, and the level above sees each version through the rebuilt bytes.
            // Siblings still unpainted stop the chain here; the last of them carries it up. No
            // stripe: nothing here is written, and the rebuild reads the siblings through
            // Tile(), which may paint one and fold it upward -- never with an address held.
            bool any = false;
            for (const std::vector<uint8_t>& v : rising) {
                std::vector<uint8_t> parent;
                if (FoldFromChildren(frame, tag, p, parent, false, &c, &v) != Status::Content) {
                    continue;
                }
                next.push_back(std::move(parent));
                any = true;
            }
            return any;
        }
        tree_detail::StripeLock lk(Stripe(tag, p));
        // A CHILD PUBLISHED SINCE OURS IS THE ONE TO FOLD. The versions in hand are what this
        // thread wrote at c. If c on disk is no longer the last of them, another thread wrote c
        // after us -- over ours, under c's stripe -- and carries that up itself; folding ours
        // here would put p back to a c that no longer exists, after the other thread's fold if
        // it got here first: a lost fold. The walk this replaced loses it that way, and so does
        // this one without the read (the tiletree selftest stages the race and sees both).
        // Alone, c on disk IS the last version, and this changes nothing.
        {
            std::vector<uint8_t> now;
            if (tree_detail::ReadTile(Base(tag, c) + ".bin", now, TileBytes()) &&
                now != rising.back()) {
                rising.assign(1, std::move(now));
            }
        }
        Held ph = PeekAt(pbase);
        std::vector<uint8_t> parent, stored;
        if (ph == Held::Content) {
            if (tree_detail::ReadTile(pbase + ".bin", parent, TileBytes())) stored = parent;
            else ph = Held::Absent;
        }
        if (ph == Held::Void) parent.assign(TileBytes(), 0);
        bool have = ph != Held::Absent, voidMarked = ph == Held::Void, fresh = false, moved = false;
        for (const std::vector<uint8_t>& v : rising) {
            if (!have) {
                // THE ABSENT PARENT IS PAINTED HERE AND NOT FOLDED UPWARD FROM HERE: its own
                // paint rises with this walk, ahead of the fold, where the nested walk put it.
                // (Left out, as kWalkChildOnly does for the selftest's evidence, a parent whose
                // fold is redundant ends the chain with its ancestors never painted, and every
                // fresh level above a moved one is decided against a different sequence.)
                Compositor::TileBox pbox{};
                frame.Box(p, pbox);
                bool own = false;
                const Status s = LeafTile(frame, tag, p, pbox, parent, nullptr, &own);
                if (s == Status::Transient) continue;   // not now: the next version asks again
                have = true;
                if (s != Status::Content) {
                    parent.assign(TileBytes(), 0);   // LeafTile left a .void; the fold takes it
                    ph = Held::Void;
                    voidMarked = true;
                } else if (own) {
                    fresh = true;
                    if (FoldWalk() != kWalkChildOnly) next.push_back(parent);
                }
            }
            if (parent.size() != TileBytes()) continue;
            const std::vector<uint8_t> before = parent;
            FoldQuadrant(frame, c, v, parent);
            // M9bd: THE CHAIN STOPS WHERE THE CHILD STOPS MATTERING. A one-tile source folds
            // into its parent and grandparent and is then invisible: if folding it in moves the
            // parent by less than 4/255 anywhere, nothing is written and nothing above is
            // touched -- the composite there keeps its reference to the base tree (the soak rule
            // already left the small source out of that subset), and no coarse tile is
            // materialized for it.
            if (ph != Held::Void && Redundant(before, parent, kFoldRedundantTol)) {
                parent = before;
                ++redundant;
                continue;
            }
            ph = Held::Content;
            moved = true;
            ++folded;
            next.push_back(parent);
        }
        // Published once, as the last version the walk left it -- unless that is what the disk
        // already holds (a version and its undoing, which only a concurrent chain produces).
        if ((fresh || moved) && parent != stored) {
            if (voidMarked) tree_detail::Delete(pbase + ".void");
            tree_detail::WriteTile(pbase + ".bin", parent);
        }
        return moved;
    }

    // A tile-native node's parents ARE the fold of their children (a box mean): never painted,
    // never stored, never read here -- every ancestor gets a marker if it has none, and a
    // refetch notice. (Reading the children at each level cost 4^m reads per paint: 44 s of
    // prefill for one bucket.)
    //
    // THE MARKING STOPS AT THE FIRST ANCESTOR THAT ALREADY HAS A .fold OR A .bin, because every
    // writer of either leaves that ancestor's own ancestors marked: this walk goes on upward
    // until it meets such an ancestor (so, by induction, the one it meets has them); Prefill
    // marks level by level from its finest mip up, so a parent it marks is marked in the same
    // prefill; and a tile-native .bin is LeafTile's, whose fold is this walk from its parent. A
    // .void is not a stop -- a child with content has just arrived under it -- it becomes a
    // marker and the walk goes on. The notices do not stop where the marking does, when anyone
    // listens: a derived tile's bytes are rebuilt from what lies below it. (The wave tree binds
    // no hook and has no parent node, so its walk ends at the first marked ancestor.)
    void MarkAncestors(const ColorFrame& frame, const std::string& tag, const TileRequest& c,
                       const TileRequest& p) {
        tree_detail::StripeLock lk(Stripe(tag, p));
        const bool listening = bool(onChanged) || m_parent != nullptr;
        bool marking = true;
        TileRequest a = p;
        for (uint32_t m = c.mip + 1; m <= MaxMip(frame); ++m) {
            if (marking) {
                const std::string ab = Base(tag, a);
                if (tree_detail::Exists(ab + ".fold") || tree_detail::Exists(ab + ".bin")) {
                    marking = false;
                    if (!listening) return;
                } else if (tree_detail::Exists(ab + ".void")) {
                    tree_detail::Delete(ab + ".void");   // a child with content arrived
                    tree_detail::Touch(ab + ".fold");
                } else {
                    tree_detail::Touch(ab + ".fold");
                }
            }
            InvalidateAbove(tag, a);
            a = TileRequest{a.face, a.mip + 1, a.x / 2, a.y / 2};
        }
    }

    // ---- THE WALK THIS REPLACED, kept for the tiletree selftest alone and selected by nothing
    // else (FoldWalkForTest(kWalkNested)): the planted failure the stripe count must trip on --
    // an absent parent's LeafTile folds its own paint to the root while FoldInto holds the
    // parent's stripe -- and the reference the new walk's files are held byte-equal to on
    // today's depth. Verbatim but for the StripeLock that lets the count see it, the redundant
    // count, and the pause point.
    void FoldUpNested(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                      const std::vector<uint8_t>& child) {
        if (r.mip >= MaxMip(frame)) return;
        if (m_fmt == Fmt::Half) return;
        const TileRequest p{r.face, r.mip + 1, r.x / 2, r.y / 2};
        std::vector<uint8_t> parent;
        if (!FoldIntoNested(frame, tag, r, p, child, parent)) return;
        InvalidateAbove(tag, p);
        if (FoldStepHook()) FoldStepHook()(p);
        FoldUpNested(frame, tag, p, parent);
    }
    bool FoldIntoNested(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                        const TileRequest& p, const std::vector<uint8_t>& child,
                        std::vector<uint8_t>& parent) {
        const std::string pbase = Base(tag, p);
        tree_detail::StripeLock lk(Stripe(tag, p));
        if (m_node->TileNative()) {
            TileRequest a = p;
            for (uint32_t m = r.mip + 1; m <= MaxMip(frame); ++m) {
                const std::string ab = Base(tag, a);
                if (!tree_detail::Exists(ab + ".bin") && !tree_detail::Exists(ab + ".void") &&
                    !tree_detail::Exists(ab + ".fold")) {
                    tree_detail::Touch(ab + ".fold");
                } else if (tree_detail::Exists(ab + ".void")) {
                    tree_detail::Delete(ab + ".void");
                    tree_detail::Touch(ab + ".fold");
                }
                InvalidateAbove(tag, a);
                a = TileRequest{a.face, a.mip + 1, a.x / 2, a.y / 2};
            }
            return false;
        }
        if (tree_detail::Exists(pbase + ".fold")) {
            return FoldFromChildren(frame, tag, p, parent, false) == Status::Content;
        }
        Held ph = PeekAt(pbase);
        if (ph == Held::Content && !tree_detail::ReadTile(pbase + ".bin", parent, TileBytes())) {
            ph = Held::Absent;
        }
        if (ph == Held::Absent) {
            Compositor::TileBox pbox{};
            frame.Box(p, pbox);
            const Status s = LeafTile(frame, tag, p, pbox, parent, nullptr);
            if (s == Status::Transient) return false;
            if (s != Status::Content) {
                parent.assign(TileBytes(), 0);
                ph = Held::Void;
            }
        } else if (ph == Held::Void) {
            parent.assign(TileBytes(), 0);
        }
        if (parent.size() != TileBytes()) return false;
        const std::vector<uint8_t> before = parent;
        FoldQuadrant(frame, r, child, parent);
        if (ph != Held::Void && Redundant(before, parent, kFoldRedundantTol)) {
            ++redundant;
            return false;
        }
        if (ph == Held::Void) tree_detail::Delete(pbase + ".void");
        tree_detail::WriteTile(pbase + ".bin", parent);
        ++folded;
        return true;
    }
    static std::atomic<int>& FoldWalkFlag() {
        static std::atomic<int> walk{kWalkOnce};
        return walk;
    }
    static int FoldWalk() { return FoldWalkFlag().load(std::memory_order_relaxed); }
    static std::function<void(const TileRequest&)>& FoldStepHook() {
        static std::function<void(const TileRequest&)> step;
        return step;
    }

public:
    // The tiletree selftest's switch between the walks: kWalkOnce, the one every run uses;
    // kWalkNested, the walk it replaced (the planted failure, and the byte reference); and
    // kWalkChildOnly, the one-pass walk WITHOUT an absent parent's own paint rising -- measured,
    // never used, to show which files it would change. Process-wide; nothing else sets it.
    static constexpr int kWalkOnce = 0, kWalkNested = 1, kWalkChildOnly = 2;
    static void FoldWalkForTest(int walk) { FoldWalkFlag().store(walk); }
    // ...and its pause point: called on the painting thread after each level of either walk is
    // published and announced, with no stripe held -- the instant a chain carries a version of
    // a tile that another thread may replace before the level above is folded. Set only while
    // no walk runs; empty in every run but the selftest's.
    static void FoldStepForTest(std::function<void(const TileRequest&)> step) {
        FoldStepHook() = std::move(step);
    }

    // M9bc: PREFILL THE PYRAMID. Realize every tile of a box at every mip from its finest to the
    // root, so the disk holds the whole chain before anyone asks: the residency manager's
    // ancestor walk then reads files and never waits on a paint (no waves popping in). Returns
    // tiles realized. Faces/planes are [f0, f1).
    //
    // THE BOX IS WHOLE TILES OF MIP `finest`, CLOSED: [x0, x1] x [y0, y1]. A coarser mip's box
    // is its image, x >> (m - finest), clamped to that mip's tiles -- integers throughout, and
    // no level finer than the caller names. It was a uv box in floats, visited from mip 0: on
    // the pyramid (2^24 tiles a side at mip 0) a modest box is 10^10 addresses there, a float
    // uv cannot name every tile, and the 0.9999f clamp on the far edge dropped the last 1677
    // columns of a face. For every box a float uv could state exactly, the image of its mip-0
    // box IS what the float arithmetic gave at each mip (the tiletree selftest holds the two
    // equal at every mip of the wave tree's box).
    uint32_t Prefill(const ColorFrame& inherited, uint32_t f0, uint32_t f1, uint32_t finest,
                     uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1) {
        const ColorFrame& frame = Resolve(inherited);
        const std::string tag = frame.Tag();
        EnsureFrame(tag);
        uint32_t n = 0, markers = 0;
        const uint32_t maxMip = MaxMip(frame);
        for (uint32_t f = f0; f < f1; ++f) {
            // Finest first: the level above is then the fold of what was just written.
            for (uint32_t m = finest; m <= maxMip; ++m) {
                const uint32_t dim = frame.faceDim >> m, up = m - finest;
                const uint32_t tw = (std::max)(1u, dim / frame.texW), th = (std::max)(1u, dim / frame.texH);
                const uint32_t bx0 = x0 >> up, by0 = y0 >> up;
                const uint32_t bx1 = (std::min)(tw - 1, x1 >> up), by1 = (std::min)(th - 1, y1 >> up);
                for (uint32_t y = by0; y <= by1; ++y) {
                    for (uint32_t x = bx0; x <= bx1; ++x) {
                        const TileRequest r{f, m, x, y};
                        const std::string base = Base(tag, r);
                        if (PeekAt(base) != Held::Absent) { ++n; continue; }
                        std::vector<uint8_t> bytes;
                        if (m == finest) {
                            if (Tile(frame, tag, r, bytes, nullptr) != Status::Transient) ++n;
                            continue;
                        }
                        // A parent: the fold of its children. A tile-native node's parent IS
                        // that fold (a box mean), so it is never stored -- a marker says so.
                        // Any other node's parent is painted and kept only where it differs
                        // from the fold by more than 4/255 somewhere.
                        if (m_node->TileNative()) {
                            // Derived, so decide by what the children HOLD -- no reads.
                            bool anyKid = false;
                            for (uint32_t k = 0; k < 4 && !anyKid; ++k) {
                                const TileRequest c{f, m - 1, x * 2 + (k & 1), y * 2 + (k >> 1)};
                                anyKid = PeekAt(Base(tag, c)) == Held::Content;
                            }
                            tree_detail::Touch(base + (anyKid ? ".fold" : ".void"));
                            if (anyKid) ++markers;
                            ++n;
                            continue;
                        }
                        std::vector<uint8_t> fold;
                        const Status fs = FoldFromChildren(frame, tag, r, fold, false);
                        if (fs == Status::Transient) continue;
                        if (fs == Status::Void) {
                            tree_detail::Touch(base + ".void");
                            ++n;
                            continue;
                        }
                        if (Tile(frame, tag, r, bytes, nullptr) == Status::Content &&
                            Redundant(bytes, fold, kFoldRedundantTol)) {
                            tree_detail::Delete(base + ".bin");
                            tree_detail::Touch(base + ".fold");
                            ++markers;
                        }
                        ++n;
                    }
                }
            }
        }
        folded += markers;
        return n;
    }
private:
    // M9bd: rebuild a parent from its four children (content, void or markers themselves).
    // Transient if any child is not ready; Void if none has content. `materialize` writes the
    // result as bytes and retires the marker -- for deep levels that are actually served, so
    // a mip-6 read is one file and not 4^6 of them. `known`, when given, is one child's bytes
    // as the fold walk carries them: the version it is folding, not whatever was last written.
    // Its one caller with `materialize` is the own-mip fold (LeafTile), where a child the
    // footprint does not touch is void by arithmetic and is never asked for.
    Status FoldFromChildren(const ColorFrame& frame, const std::string& tag, const TileRequest& r,
                            std::vector<uint8_t>& out, bool materialize,
                            const TileRequest* known = nullptr,
                            const std::vector<uint8_t>* knownBytes = nullptr) {
        if (r.mip == 0) return Status::Transient;
        out.assign(TileBytes(), 0);
        bool any = false;
        for (uint32_t k = 0; k < 4; ++k) {
            const TileRequest c{r.face, r.mip - 1, r.x * 2 + (k & 1), r.y * 2 + (k >> 1)};
            std::vector<uint8_t> cb;
            const bool isKnown = known && knownBytes && known->face == c.face &&
                                 known->mip == c.mip && known->x == c.x && known->y == c.y;
            if (materialize) {
                Compositor::TileBox kb{};
                frame.Box(c, kb);
                if (!Touching(kb)) continue;
            }
            if (isKnown) cb = *knownBytes;
            const Status s = isKnown ? Status::Content : Tile(frame, tag, c, cb, nullptr);
            if (s == Status::Transient) return Status::Transient;
            if (s != Status::Content || cb.size() != TileBytes()) continue;
            FoldQuadrant(frame, c, cb, out);
            any = true;
        }
        if (!any) return Status::Void;
        if (materialize) {
            const std::string base = Base(tag, r);
            tree_detail::WriteTile(base + ".bin", out);
            tree_detail::Delete(base + ".fold");
            ++folded;
        }
        return Status::Content;
    }
    // Within tol/255 on every byte? (Byte formats only; a float tile is never called redundant.)
    bool Redundant(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, int tol) const {
        if (m_fmt == Fmt::FloatW || a.size() != b.size()) return false;
        for (size_t i = 0; i < a.size(); ++i) {
            const int d = int(a[i]) - int(b[i]);
            if (d > tol || d < -tol) return false;
        }
        return true;
    }
    static constexpr int kFoldRedundantTol = 4;   // the user's "4 pixel": 4/255 per channel
    // A tile of this node changed: consumers holding it refetch; ancestors' cached composites
    // of the same address are dropped (their key folds what children HOLD, not what they say).
    void InvalidateAbove(const std::string& tag, const TileRequest& r) {
        ++tree_detail::Io().announces;
        if (onChanged) onChanged(tag, r);
        if (m_parent) m_parent->DropCachedAddress(tag, r);
    }
    void DropCachedAddress(const std::string& tag, const TileRequest& r) {
        const std::string base = Base(tag, r);
        WIN32_FIND_DATAA fd{};
        HANDLE h = FindFirstFileA((base + "_*").c_str(), &fd);
        if (h != INVALID_HANDLE_VALUE) {
            const std::string dir = base.substr(0, base.find_last_of('\\') + 1);
            do {
                tree_detail::Delete(dir + fd.cFileName);
            } while (FindNextFileA(h, &fd));
            FindClose(h);
        }
        tree_detail::Delete(base + ".bin");
        tree_detail::Delete(base + ".void");
        ++dropped;
        InvalidateAbove(tag, r);
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
            if (!RefuseLattice(*m_kids[inc[k]], frame)) {   // M12 step 2d: not this tile's texels
                complete = false;
                continue;
            }
            const Status s = m_kids[inc[k]]->Tile(frame, tag, r, tiles[k]);
            if (s == Status::Transient) {
                complete = false;
                continue;
            }
            if (s == Status::Void) continue;
            got[k] = Held::Content;
            live.push_back(k);
        }
        if (!complete) g_tileIncomplete = true;   // step 5 E: answered without a source (finding 83)
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
        if (!RefuseLattice(layer, frame)) return Status::Transient;   // M12 step 2d
        const Status ls = layer.Tile(frame, tag, r, lt);
        if (ls == Status::Transient) return Status::Transient;
        std::vector<Held> got;
        bool gateIn = false;
        for (size_t k = 0; k < inc.size(); ++k) gateIn |= (inc[k] == 1);
        Status gs = Status::Void;
        if (gateIn) {
            if (!RefuseLattice(gate, frame)) return Status::Transient;   // M12 step 2d
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
        // THE VALUE GATE, the per-sample law of GateSource::SampleAt said per texel: the gate tile's
        // alpha is its OPINION (surveyed), its red the VALUE (the survey's water coverage), and the
        // factor is (1 - opinion) + opinion x value -- no opinion passes the layer, an opinion's
        // value decides. This multiplied by the alpha alone until 2026-10-10, so the mask blocked
        // only where it had no tile and passed the seafloor's sand over surveyed LAND wherever the
        // height said "under water" -- unseen in New England, whose heights are exact; at Tokyo
        // Bay, over the 4.9 km ETOPO, the reclaimed shore came out sand (Mark).
        for (size_t i = 3; i < out.size(); i += 4) {
            const uint32_t op = gt[i], val = gt[i - 3];
            const uint32_t f = ((255u - op) * 255u + op * val + 127u) / 255u;
            const uint32_t a = (uint32_t(out[i]) * f + 127u) / 255u;
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
    std::string m_trees;   // the folder m_root is in (the constructor's `trees`)
    static std::string& TreeRootRef() {
        static std::string trees = "cache\\trees";
        return trees;
    }
    std::map<std::string, std::shared_ptr<TileArchive>> m_arcs;
    std::mutex m_arcMx;
    // THE ADDRESS STRIPES, replacing the per-node m_foldMx. Owned by the ROOT so every node of
    // one graph shares them; see Stripe() for the rule that keeps them deadlock-free.
    std::unique_ptr<std::array<std::recursive_mutex, kStripes>> m_stripes;
    TileTree* m_parent = nullptr;
    // M12 step 2d: the tenant-bound lattice (SetLattice); nullptr until bound.
    const Lattice* m_lattice = nullptr;
    bool m_latticeRefused = false;   // the refusal is said once per node

    // A child on a lattice of its own cannot be composed BY BYTES: its texels are not this
    // tile's texels (ATLAS: resampling is integration against the dual cell, and there is no
    // resample node yet). True = same ground, go ahead; false = refused, said once.
    bool RefuseLattice(const TileTree& kid, const ColorFrame& frame) {
        if (kid.Resolve(frame).SameGround(frame)) return true;
        if (!m_latticeRefused) {
            m_latticeRefused = true;
            Log("[tree] %s: child %s declares its own lattice (%s under %s) -- composing "
                "across lattices needs a resample node; refused, the tile stays Transient",
                m_node->Name(), kid.m_node->Name(), kid.Resolve(frame).Tag().c_str(),
                frame.Tag().c_str());
        }
        return false;
    }
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

// The tree made fit for a deep pyramid, gated (compose/TileTreeTest.cpp), on synthetic sources
// under scratch roots in out\treetest: the stripe count and its planted failure, a 25-level
// chain against an independent fold, today's depth old walk against new byte for byte, twelve
// threads against the serial tree, the tile-native marker walk and Prefill's integer box, the
// stripe hash, the archive's offsets, and the fetch provider's refusals.
bool RunTileTreeSelfTest();

}  // namespace ga
