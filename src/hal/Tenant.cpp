#include "hal/Tenant.h"

#include "compose/TileTree.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <string>

namespace ga::hal {

namespace {

const char* FormatName(DXGI_FORMAT f) {
    switch (f) {
        case DXGI_FORMAT_R16_FLOAT: return "R16F";
        case DXGI_FORMAT_R8G8B8A8_UNORM: return "RGBA8";
        case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB: return "RGBA8 sRGB";
        case DXGI_FORMAT_BC1_UNORM: return "BC1";
        case DXGI_FORMAT_BC1_UNORM_SRGB: return "BC1 sRGB";
        case DXGI_FORMAT_BC5_SNORM: return "BC5";
        default: return "?";
    }
}
const char* Name(Semantics s) { return s == Semantics::Field ? "Field" : "Texture"; }
const char* Name(Residence r) {
    switch (r) {
        case Residence::Streamable: return "Streamable";
        case Residence::Recomputable: return "Recomputable";
        default: return "Volatile";
    }
}
const char* Name(Absence a) {
    switch (a) {
        case Absence::Zero: return "Zero";
        case Absence::Unloaded: return "Unloaded";
        case Absence::OutOfDomain: return "OutOfDomain";
        default: return "Coarse";
    }
}
std::string Narrow(const wchar_t* w) {
    std::string s;
    for (; w && *w; ++w) s += static_cast<char>(*w < 128 ? *w : '?');
    return s;
}
// A block slice's name in the boot line and the pages ledger: the pyramid's tag, then the block.
std::string BlockLabel(const std::string& pyramidTag, const BlockBinding& k) {
    char buf[128];
    if (k.sx || k.sy) {
        snprintf(buf, sizeof(buf), "%s f%u r%d window at (%llu,%llu)", pyramidTag.c_str(), k.face,
                 k.rung, static_cast<unsigned long long>(k.OrgX()),
                 static_cast<unsigned long long>(k.OrgY()));
    } else {
        snprintf(buf, sizeof(buf), "%s f%u r%d (%u,%u)", pyramidTag.c_str(), k.face, k.rung, k.bx,
                 k.by);
    }
    return buf;
}

}  // namespace

// ---- HIERARCHY 4.17: the block binding (see Tenant.h) ----------------------------------------

uint32_t BlockBinding::Mips(uint32_t texW, uint32_t texH) {
    const uint32_t tile = (std::max)(texW, texH);
    if (!texW || !texH || (texW & (texW - 1)) || (texH & (texH - 1)) || tile > Lattice::kFaceDim) {
        return 0;
    }
    uint32_t mips = 1;   // AddTextureInternal's loop, on a slice's side
    while ((Lattice::kFaceDim >> (mips - 1)) > tile) ++mips;
    return mips;
}

bool BlockBinding::Global(const TileRequest& slot, uint32_t texW, uint32_t texH,
                          TileRequest& global) const {
    if (slot.mip >= Mips(texW, texH)) return false;
    const uint32_t side = Lattice::kFaceDim >> slot.mip;   // the slice's texels a side there
    const uint32_t tw = side / texW, th = side / texH;
    if (slot.x >= tw || slot.y >= th) return false;
    // The box's own tiles at this mip, [O, O + tw): the one whose index is the slot's modulo tw.
    // A block (sx = 0) has O = bx tw, and this is bx tw + x.
    const uint64_t ox = OrgX() / (uint64_t(texW) << slot.mip), oy = OrgY() / (uint64_t(texH) << slot.mip);
    global = TileRequest{face, uint32_t(kFinestRung - rung) + slot.mip,
                         uint32_t(ox + (slot.x + tw - ox % tw) % tw),
                         uint32_t(oy + (slot.y + th - oy % th) % th)};
    return true;
}

bool BlockBinding::Slot(const TileRequest& global, uint32_t texW, uint32_t texH,
                        TileRequest& slot) const {
    const uint32_t top = uint32_t(kFinestRung - rung);   // the pyramid's mip at the slice's mip 0
    if (global.face != face || global.mip < top || global.mip - top >= Mips(texW, texH)) {
        return false;
    }
    const uint32_t m = global.mip - top;
    const uint32_t side = Lattice::kFaceDim >> m;
    const uint32_t tw = side / texW, th = side / texH;
    // Unsigned: a tile before the box's origin wraps far past the grid and is refused with the
    // tiles after its end. The origin's tile stays below 2^24, so nothing else wraps. The slot is
    // the index modulo the tiles a side (a block's is x - bx tw).
    const uint64_t ox = OrgX() / (uint64_t(texW) << m), oy = OrgY() / (uint64_t(texH) << m);
    if (uint64_t(global.x) - ox >= tw || uint64_t(global.y) - oy >= th || global.x < ox ||
        global.y < oy) {
        return false;
    }
    slot = TileRequest{0, m, global.x % tw, global.y % th};
    return true;
}

void BlockBinding::Ground(double& u0, double& v0, double& u1, double& v1) const {
    const double w = std::ldexp(1.0, -rung);   // exact: a power of two, and bx below 2^17
    const double t = w / Lattice::kFaceDim;    // one texel of the rung, exact
    u0 = double(OrgX()) * t;
    v0 = double(OrgY()) * t;
    u1 = u0 + w;
    v1 = v0 + w;
}

double BlockBinding::GroundRes(uint32_t mip) const {
    return Lattice::Cube(kPyramidDim).GroundRes(uint32_t(kFinestRung - rung) + mip);
}

std::string BlockBinding::Refusal(uint32_t slice, uint32_t texW, uint32_t texH) const {
    if (rung < 0 || rung > kFinestRung) {
        return "rung " + std::to_string(rung) + " is not one of the pyramid's 0 to " +
               std::to_string(kFinestRung);
    }
    if (face >= 6) return "face " + std::to_string(face) + " is not one of the cube's six";
    const uint32_t side = 1u << rung;
    if (bx >= side || by >= side) {
        return "block (" + std::to_string(bx) + ", " + std::to_string(by) + ") lies outside face " +
               std::to_string(face) + ", which holds " + std::to_string(side) +
               " blocks a side at rung " + std::to_string(rung);
    }
    // PHASE B2 (D1): a window's origin is a whole tile at its floor (mip 3) on each axis: a multiple of
    // the tile's width x 8 in x and its height x 8 in y -- 1024 for 128 x 128, 2048 x 1024 for 256 x 128.
    const uint32_t qx = texW << kFloorMip, qy = texH << kFloorMip;
    if (sx >= Lattice::kFaceDim || sy >= Lattice::kFaceDim || (qx && sx % qx) || (qy && sy % qy)) {
        return "a window's origin (" + std::to_string(sx) + ", " + std::to_string(sy) +
               ") within its block is not a multiple of " + std::to_string(qx) + " x " +
               std::to_string(qy) + " texels below 16384: its mips 0..3 would not be whole " +
               std::to_string(texW) + " x " + std::to_string(texH) + " tiles";
    }
    if ((sx && bx + 1 >= side) || (sy && by + 1 >= side)) {
        return "the window at (" + std::to_string(OrgX()) + ", " + std::to_string(OrgY()) +
               ") reaches past face " + std::to_string(face) + "'s edge";
    }
    if (slice < 6) return "slice " + std::to_string(slice) + " is one of the cube's six faces";
    return "";
}

struct Tenant::State {
    TenantDesc desc;
    std::vector<std::string> tags;   // bindings[i].lattice.Tag(), computed once
    ResidencyManager* mgr = nullptr;
    int id = -1;
    uint32_t dim = 0;                // the face dimension the bindings agree on
    // Read per request with std::atomic_load; written by Bind(holder) with std::atomic_store.
    std::shared_ptr<std::shared_ptr<TileTree>> holder;
    mutable std::atomic<bool> unknownTagSaid{false};
    // HIERARCHY 4.17: the pyramid's lattice in the fiber's tile shape, and its Tag() -- the
    // name a tree on it announces its changes by.
    Lattice pyramid;
    std::string pyramidTag;
    // Where an invalidation goes when there is no manager (Unregistered: the caller's).
    std::function<void(const TileRequest&)> sink;
    // PHASE A1: a window's binding moves (Move, main thread) while the loaders dispatch and the
    // painters change tiles; every read of a block slice's binding takes this lock and a copy.
    mutable std::mutex blockMx;
    BlockBinding BindingCopy(const BlockSlice& k) const {
        std::lock_guard<std::mutex> lk(blockMx);
        return k.block;
    }

    // The gates (Sparse's list), and the state they admit. Nothing is registered and nothing is
    // said here, so Sparse and Unregistered refuse the same declarations in the same words.
    static std::shared_ptr<State> Admit(TenantDesc desc);

    const SliceBinding* BindingOf(uint32_t slice) const {
        for (const SliceBinding& b : desc.bindings) {
            if (slice >= b.first && slice < b.first + b.count) return &b;
        }
        return nullptr;
    }
    const BlockSlice* BlockOf(uint32_t slice) const {
        for (const BlockSlice& k : desc.blocks) {
            if (k.slice == slice) return &k;
        }
        return nullptr;
    }
    uint32_t SliceOf(const std::string& tag) const {
        for (size_t i = 0; i < tags.size(); ++i) {
            if (tags[i] == tag) return desc.bindings[i].first;
        }
        // PHASE B2 (D2): a tenant of window slices alone has no lattice binding to route to.
        if (desc.bindings.empty()) return desc.blocks.empty() ? 0u : desc.blocks.front().slice;
        uint32_t to = desc.bindings.front().first;
        for (const SliceBinding& b : desc.bindings) {
            if (b.lattice.kind == Lattice::Kind::Cube) {
                to = b.first;
                break;
            }
        }
        if (!unknownTagSaid.exchange(true)) {
            Log("[tenant] %ls: a change on lattice %s, which no binding declares -- routed to "
                "slice %u",
                desc.name, tag.c_str(), to);
        }
        return to;
    }
    // The tree in the holder, read per request: two atomic loads, the holder then the tree in it.
    std::shared_ptr<TileTree> Held() const {
        const std::shared_ptr<std::shared_ptr<TileTree>> h = std::atomic_load(&holder);
        return h ? std::atomic_load(h.get()) : nullptr;
    }
    // THE DISPATCHER (banner): the binding's provider with the request re-based on the
    // binding's first slice; a block slice's global tile (DispatchBlock); an unbound slice
    // answers the declared pattern; a holder binding reads the tree per request and answers
    // false when there is none.
    bool Dispatch(const TileRequest& r, std::vector<uint8_t>& out, TileLoc* loc) const {
        const SliceBinding* b = BindingOf(r.face);
        if (!b) {
            if (const BlockSlice* k = BlockOf(r.face)) return DispatchBlock(*k, r, out, loc);
            if (desc.absentTile.empty()) return false;
            out = desc.absentTile;
            if (loc) *loc = TileLoc{};
            return true;
        }
        TileRequest w = r;
        w.face = r.face - b->first;
        if (b->provider) return b->provider(w, out, loc);
        const std::shared_ptr<TileTree> t = Held();
        if (!t) return false;
        return t->Provider(b->lattice)(w, out, loc);
    }
    // THE BLOCK HALF (the banner): the slot's tile of the pyramid, asked of the binding's
    // provider or of the tree in the holder ON THE PYRAMID'S LATTICE. A slot outside the slice's
    // grid has no global tile and answers false, as a provider does for a tile it cannot make.
    bool DispatchBlock(const BlockSlice& k, const TileRequest& r, std::vector<uint8_t>& out,
                       TileLoc* loc) const {
        TileRequest g;
        if (!BindingCopy(k).Global(r, desc.fiber.texW, desc.fiber.texH, g)) return false;
        if (k.provider) return k.provider(g, out, loc);
        const std::shared_ptr<TileTree> t = Held();
        if (!t) return false;
        return t->Provider(pyramid)(g, out, loc);
    }
    // THE CHANGE LAW (Tenant.h, Changed). A tile of the pyramid reaches every block slice that
    // holds it -- one tile can lie in several -- and none that does not; it never falls through
    // to SliceOf, which would name a cube face at a global coordinate.
    void Changed(const std::string& tag, const TileRequest& r) const {
        if (!desc.blocks.empty() && tag == pyramidTag) {
            for (const BlockSlice& k : desc.blocks) {
                TileRequest q;
                if (!BindingCopy(k).Slot(r, desc.fiber.texW, desc.fiber.texH, q)) continue;
                q.face = k.slice;
                Invalidate(q);
            }
            return;
        }
        TileRequest q = r;
        q.face = SliceOf(tag) + r.face;
        Invalidate(q);
    }
    void Invalidate(const TileRequest& q, bool moved = false) const {
        if (mgr) mgr->Invalidate(id, q, moved);
        else if (sink) sink(q);
    }
    // The [tenant] line: the declaration as the boot log's law (Tenant.h's banner), bindings and
    // blocks in slice order with the unbound runs between them.
    void LogDeclaration() const;
};

std::shared_ptr<Tenant::State> Tenant::State::Admit(TenantDesc desc) {
    auto s = std::make_shared<State>();
    s->desc = std::move(desc);
    const TenantDesc& d = s->desc;
    auto refuse = [&](const std::string& why) {
        throw std::runtime_error("tenant " + Narrow(d.name) + ": " + why);
    };
    // ---- the gates: a declaration the dispatcher can honour, or no tenant at all.
    if (d.slices == 0) refuse("no slices");
    // PHASE B2 (D2): A TENANT CONSISTS OF THE SLICES ITS READERS NEED. One that is read only from the
    // eye's windows (the swell exposure) declares its window slices and no lattice at all; its tree
    // is bound on the pyramid's lattice (Bind), and its unbound slices answer what it declares absent.
    if (d.bindings.empty() && d.blocks.empty()) refuse("no slice binding and no block slice");
    if (!d.absentTile.empty() && d.absentTile.size() != 65536) {
        refuse("absentTile is " + std::to_string(d.absentTile.size()) + " bytes, not one 64 KB tile");
    }
    std::vector<uint8_t> bound(d.slices, 0);
    uint32_t dim = 0;
    for (const SliceBinding& b : d.bindings) {
        const std::string where = "binding [" + std::to_string(b.first) + ".." +
                                  std::to_string(b.first + b.count - 1) + "]";
        if (b.count == 0 || b.first >= d.slices || b.first + b.count > d.slices) {
            refuse(where + " outside the tenant's " + std::to_string(d.slices) + " slices");
        }
        for (uint32_t f = b.first; f < b.first + b.count; ++f) {
            if (bound[f]) refuse("slice " + std::to_string(f) + " bound twice");
            bound[f] = 1;
        }
        if (b.lattice.texW != d.fiber.texW || b.lattice.texH != d.fiber.texH) {
            refuse(where + " lattice tiles " + std::to_string(b.lattice.texW) + "x" +
                   std::to_string(b.lattice.texH) + " texels, the fiber " +
                   std::to_string(d.fiber.texW) + "x" + std::to_string(d.fiber.texH));
        }
        if (dim == 0) dim = b.lattice.faceDim;
        if (dim != b.lattice.faceDim) refuse(where + " disagrees on the face dimension");
        if (!b.provider && !d.holder) refuse(where + " has neither a provider nor a holder");
        s->tags.push_back(b.lattice.Tag());
    }
    // HIERARCHY 4.17: a block slice is one slice of 16384 texels a side, the fiber's tiles must
    // tile it, and it is bound once, beside the lattice bindings, like any slice.
    for (const BlockSlice& k : d.blocks) {
        const std::string where = "block slice " + std::to_string(k.slice);
        const std::string why = k.block.Refusal(k.slice, d.fiber.texW, d.fiber.texH);
        if (!why.empty()) refuse(where + ": " + why);
        if (k.slice >= d.slices) {
            refuse(where + " outside the tenant's " + std::to_string(d.slices) + " slices");
        }
        if (bound[k.slice]) refuse("slice " + std::to_string(k.slice) + " bound twice");
        bound[k.slice] = 1;
        if (dim == 0) dim = Lattice::kFaceDim;   // blocks alone (D2): the slices are blocks'
        if (dim != Lattice::kFaceDim) {
            refuse(where + ": a block is " + std::to_string(Lattice::kFaceDim) +
                   " texels a side, the tenant's slices " + std::to_string(dim));
        }
        if (BlockBinding::Mips(d.fiber.texW, d.fiber.texH) == 0) {
            refuse(where + ": the fiber's " + std::to_string(d.fiber.texW) + "x" +
                   std::to_string(d.fiber.texH) + " tiles tile no block");
        }
        if (!k.provider && !d.holder) refuse(where + " has neither a provider nor a holder");
    }
    s->dim = dim;
    s->pyramid = BlockBinding::Pyramid(d.fiber.texW, d.fiber.texH);
    s->pyramidTag = s->pyramid.Tag();
    return s;
}

Tenant Tenant::Sparse(Gpu& gpu, ResidencyManager& mgr, TenantDesc desc) {
    const std::shared_ptr<State> s = State::Admit(std::move(desc));
    s->mgr = &mgr;
    const TenantDesc& d = s->desc;
    Tenant t;
    t.m_s = s;
    // The holder first: the coarsest tiles load synchronously inside AddTexturePages.
    if (d.holder) t.Bind(d.holder);
    TileProviderFn dispatch = [s](const TileRequest& r, std::vector<uint8_t>& out, TileLoc* loc) {
        return s->Dispatch(r, out, loc);
    };
    std::vector<uint8_t> top(d.slices, uint8_t(255));   // 255: the array's coarsest
    for (const BlockSlice& k : d.blocks) {
        if (k.slice < d.slices) top[k.slice] = uint8_t(BlockBinding::kFloorMip);
    }
    s->id = mgr.AddTexturePages(gpu, d.name, s->dim, d.fiber.fmt, std::move(dispatch), d.slices,
                                std::move(top));
    // The pages ledger (Residency.h pagesEvery) names each slice by the lattice it sits on --
    // the one fact about a slice the manager did not already hold. A block slice is named by
    // its block, and its mip-0 ground is the pyramid's at its rung.
    {
        std::vector<std::string> tags(d.slices);
        std::vector<double> ground(d.slices, 0.0);
        for (const SliceBinding& b : d.bindings) {
            for (uint32_t f = b.first; f < b.first + b.count; ++f) {
                tags[f] = b.lattice.Tag();
                ground[f] = b.lattice.GroundRes(0);
            }
        }
        for (const BlockSlice& k : d.blocks) {
            tags[k.slice] = BlockLabel(s->pyramidTag, k.block);
            ground[k.slice] = k.block.GroundRes(0);
        }
        mgr.LabelSlices(s->id, std::move(tags), std::move(ground));
    }
    // 4.7: the address of a block slice's slot (ResidencyManager::SetGlobalOf): the pool holds
    // bytes by address, once, and every slot of the address is a mapping of them.
    if (!d.blocks.empty()) {
        mgr.SetGlobalOf(s->id, [s](const TileRequest& r, TileRequest& g) {
            const BlockSlice* k = s->BlockOf(r.face);
            return k && s->BindingCopy(*k).Global(r, s->desc.fiber.texW, s->desc.fiber.texH, g);
        });
    }
    // The watchdog's global name for a block slice's tile (ResidencyManager::SetTileNamer).
    if (!d.blocks.empty()) {
        mgr.SetTileNamer(s->id, [s](const TileRequest& r) {
            const BlockSlice* k = s->BlockOf(r.face);
            if (!k) return std::string();
            const BlockBinding b = s->BindingCopy(*k);
            TileRequest g;
            char buf[160];
            if (!b.Global(r, s->desc.fiber.texW, s->desc.fiber.texH, g)) {
                snprintf(buf, sizeof(buf), "outside the slice's grid (face %u rung %d)", b.face, b.rung);
            } else {
                snprintf(buf, sizeof(buf), "face %u rung %d pyramid m%u (%u,%u), box origin (%llu,%llu)", b.face,
                         b.rung, g.mip, g.x, g.y, static_cast<unsigned long long>(b.OrgX()),
                         static_cast<unsigned long long>(b.OrgY()));
            }
            return std::string(buf);
        });
    }
    s->LogDeclaration();
    return t;
}

Tenant Tenant::Unregistered(TenantDesc desc, std::function<void(const TileRequest&)> invalidate) {
    const std::shared_ptr<State> s = State::Admit(std::move(desc));
    s->sink = std::move(invalidate);
    Tenant t;
    t.m_s = s;
    if (s->desc.holder) t.Bind(s->desc.holder);
    s->LogDeclaration();
    return t;
}

void Tenant::State::LogDeclaration() const {
    const TenantDesc& d = desc;
    // Every declared slice run, lattice bindings and blocks alike, in slice order. They are
    // disjoint (the gates), so the order is total and a tenant with no blocks prints what it
    // always printed.
    struct Run {
        uint32_t first, count;
        std::string label;
        const char* field;
    };
    std::vector<Run> runs;
    for (size_t i = 0; i < d.bindings.size(); ++i) {
        runs.push_back({d.bindings[i].first, d.bindings[i].count, tags[i], d.bindings[i].astField});
    }
    for (const BlockSlice& k : d.blocks) {
        runs.push_back({k.slice, 1, BlockLabel(pyramidTag, k.block), k.astField});
    }
    std::sort(runs.begin(), runs.end(), [](const Run& a, const Run& b) { return a.first < b.first; });
    std::string slices;
    char buf[320];
    auto range = [&](uint32_t a, uint32_t b) {
        if (a == b) snprintf(buf, sizeof(buf), " [%u]", a);
        else snprintf(buf, sizeof(buf), " [%u..%u]", a, b);
        return std::string(buf);
    };
    uint32_t next = 0;
    auto unbound = [&](uint32_t upto) {
        if (next >= upto) return;
        std::string pat = "none";
        if (!d.absentTile.empty()) {
            snprintf(buf, sizeof(buf), "%02x %02x ..", d.absentTile[0], d.absentTile[1]);
            pat = buf;
        }
        slices += (slices.empty() ? "" : ",") + range(next, upto - 1) + " unbound: " +
                  Name(d.absence) + " (" + pat + ")";
    };
    for (const Run& b : runs) {
        unbound(b.first);
        const std::string at = range(b.first, b.first + b.count - 1);   // its own string: range()
        snprintf(buf, sizeof(buf), " %s <-- %s (%s)", b.label.c_str(),   // and this share buf
                 d.astNode, b.field);
        slices += (slices.empty() ? "" : ",") + at + buf;
        next = b.first + b.count;
    }
    unbound(d.slices);
    Log("[tenant] %ls: id %d, %s %ux%u \"%s\", %s/%s/%s, slices %u:%s", d.name, id,
        FormatName(d.fiber.fmt), d.fiber.texW, d.fiber.texH, d.fiber.quantity, Name(d.semantics),
        Name(d.residence), Name(d.absence), d.slices, slices.c_str());
}

int Tenant::Id() const { return m_s ? m_s->id : -1; }

const Lattice* Tenant::LatticeOf(uint32_t slice) const {
    if (!m_s) return nullptr;
    const SliceBinding* b = m_s->BindingOf(slice);
    return b ? &b->lattice : nullptr;
}

bool Tenant::BlockOf(uint32_t slice, BlockBinding& out) const {
    const BlockSlice* k = m_s ? m_s->BlockOf(slice) : nullptr;
    if (!k) return false;
    out = m_s->BindingCopy(*k);
    return true;
}

uint32_t Tenant::Move(uint32_t slice, const BlockBinding& to) {
    if (!m_s) return 0;
    BlockSlice* k = const_cast<BlockSlice*>(m_s->BlockOf(slice));
    if (!k) return 0;
    BlockBinding from;
    {
        std::lock_guard<std::mutex> lk(m_s->blockMx);
        from = k->block;
        if (from == to) return 0;
        k->block = to;
    }
    // The window's mips, 0..3 (the banner): a slot whose global tile is not the one it held.
    const uint32_t tw0 = m_s->desc.fiber.texW, th0 = m_s->desc.fiber.texH;
    const uint32_t mips = (std::min)(4u, BlockBinding::Mips(tw0, th0));
    uint32_t told = 0;
    auto changed = [&](const TileRequest& s) {   // the slot (face 0): its ground under from vs to
        TileRequest a, b;
        const bool ha = from.Global(s, tw0, th0, a), hb = to.Global(s, tw0, th0, b);
        return !(ha == hb && (!ha || (a.face == b.face && a.mip == b.mip && a.x == b.x && a.y == b.y)));
    };
    if (m_s->mgr) {
        // F4: the tiles the manager tracks in the slice, told where their ground changed; the
        // slots that hold nothing have nothing to be told (a claimed set's step is free).
        std::vector<TileRequest> tracked;
        m_s->mgr->ForTrackedIn(m_s->id, slice, [&](const TileRequest& q, uint32_t) {
            if (q.mip < mips) tracked.push_back(q);
        });
        for (const TileRequest& q : tracked) {
            if (!changed(TileRequest{0, q.mip, q.x, q.y})) continue;
            m_s->Invalidate(q, /*moved=*/true);   // B11: the ground moved
            ++told;
        }
        return told;
    }
    // No manager tracks the slice (Unregistered: the caller's sink): any slot may hold, so every
    // slot whose ground changed is told.
    for (uint32_t m = 0; m < mips; ++m) {
        const uint32_t side = Lattice::kFaceDim >> m, tw = side / tw0, th = side / th0;
        for (uint32_t y = 0; y < th; ++y) {
            for (uint32_t x = 0; x < tw; ++x) {
                if (!changed(TileRequest{0, m, x, y})) continue;
                m_s->Invalidate(TileRequest{slice, m, x, y}, /*moved=*/true);   // B11: the ground moved
                ++told;
            }
        }
    }
    return told;
}

uint32_t Tenant::SliceOf(const std::string& latticeTag) const {
    return m_s ? m_s->SliceOf(latticeTag) : 0u;
}

void Tenant::Bind(TileTree& tree) {
    if (!m_s) return;
    // A tenant of window slices alone (D2) paints on the pyramid's lattice.
    tree.SetLattice(m_s->desc.bindings.empty() ? &m_s->pyramid : &m_s->desc.bindings.front().lattice);
    std::shared_ptr<State> s = m_s;
    tree.onChanged = [s](const std::string& tag, const TileRequest& r) { s->Changed(tag, r); };
}

void Tenant::Changed(const std::string& latticeTag, const TileRequest& r) const {
    if (m_s) m_s->Changed(latticeTag, r);
}

bool Tenant::Dispatch(const TileRequest& r, std::vector<uint8_t>& out, TileLoc* loc) const {
    return m_s ? m_s->Dispatch(r, out, loc) : false;
}

void Tenant::Bind(std::shared_ptr<std::shared_ptr<TileTree>> holder) {
    if (!m_s) return;
    std::atomic_store(&m_s->holder, std::move(holder));
}

const TenantDesc& Tenant::Desc() const {
    static const TenantDesc kNone;
    return m_s ? m_s->desc : kNone;
}

}  // namespace ga::hal
