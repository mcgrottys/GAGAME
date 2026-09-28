#include "hal/Tenant.h"

#include "compose/TileTree.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
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

}  // namespace

struct Tenant::State {
    TenantDesc desc;
    std::vector<std::string> tags;   // bindings[i].lattice.Tag(), computed once
    ResidencyManager* mgr = nullptr;
    int id = -1;
    // Read per request with std::atomic_load; written by Bind(holder) with std::atomic_store.
    std::shared_ptr<std::shared_ptr<TileTree>> holder;
    mutable std::atomic<bool> unknownTagSaid{false};

    const SliceBinding* BindingOf(uint32_t slice) const {
        for (const SliceBinding& b : desc.bindings) {
            if (slice >= b.first && slice < b.first + b.count) return &b;
        }
        return nullptr;
    }
    uint32_t SliceOf(const std::string& tag) const {
        for (size_t i = 0; i < tags.size(); ++i) {
            if (tags[i] == tag) return desc.bindings[i].first;
        }
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
    // THE DISPATCHER (banner): the binding's provider with the request re-based on the
    // binding's first slice; an unbound slice answers the declared pattern; a holder binding
    // reads the tree per request and answers false when there is none.
    bool Dispatch(const TileRequest& r, std::vector<uint8_t>& out, TileLoc* loc) const {
        const SliceBinding* b = BindingOf(r.face);
        if (!b) {
            if (desc.absentTile.empty()) return false;
            out = desc.absentTile;
            if (loc) *loc = TileLoc{};
            return true;
        }
        TileRequest w = r;
        w.face = r.face - b->first;
        if (b->provider) return b->provider(w, out, loc);
        const std::shared_ptr<std::shared_ptr<TileTree>> h = std::atomic_load(&holder);
        if (!h) return false;
        const std::shared_ptr<TileTree> t = std::atomic_load(h.get());
        if (!t) return false;
        return t->Provider(b->lattice)(w, out, loc);
    }
};

Tenant Tenant::Sparse(Gpu& gpu, ResidencyManager& mgr, TenantDesc desc) {
    auto s = std::make_shared<State>();
    s->desc = std::move(desc);
    s->mgr = &mgr;
    const TenantDesc& d = s->desc;
    auto refuse = [&](const std::string& why) {
        throw std::runtime_error("tenant " + Narrow(d.name) + ": " + why);
    };
    // ---- the gates: a declaration the dispatcher can honour, or no tenant at all.
    if (d.slices == 0) refuse("no slices");
    if (d.bindings.empty()) refuse("no slice binding");
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
    Tenant t;
    t.m_s = s;
    // The holder first: the coarsest tiles load synchronously inside AddTexturePages.
    if (d.holder) t.Bind(d.holder);
    TileProviderFn dispatch = [s](const TileRequest& r, std::vector<uint8_t>& out, TileLoc* loc) {
        return s->Dispatch(r, out, loc);
    };
    s->id = mgr.AddTexturePages(gpu, d.name, dim, d.fiber.fmt, std::move(dispatch), d.slices);
    // The pages ledger (Residency.h pagesEvery) names each slice by the lattice it sits on --
    // the one fact about a slice the manager did not already hold.
    {
        std::vector<std::string> tags(d.slices);
        std::vector<double> ground(d.slices, 0.0);
        for (const SliceBinding& b : d.bindings) {
            for (uint32_t f = b.first; f < b.first + b.count; ++f) {
                tags[f] = b.lattice.Tag();
                ground[f] = b.lattice.GroundRes(0);
            }
        }
        mgr.LabelSlices(s->id, std::move(tags), std::move(ground));
    }

    // ---- the declaration, as the boot log's law.
    std::vector<size_t> order(d.bindings.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return d.bindings[a].first < d.bindings[b].first;
    });
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
    for (const size_t i : order) {
        const SliceBinding& b = d.bindings[i];
        unbound(b.first);
        const std::string at = range(b.first, b.first + b.count - 1);   // its own string: range()
        snprintf(buf, sizeof(buf), " %s <-- %s (%s)", s->tags[i].c_str(),  // and this share buf
                 d.astNode, b.astField);
        slices += (slices.empty() ? "" : ",") + at + buf;
        next = b.first + b.count;
    }
    unbound(d.slices);
    Log("[tenant] %ls: id %d, %s %ux%u \"%s\", %s/%s/%s, slices %u:%s", d.name, s->id,
        FormatName(d.fiber.fmt), d.fiber.texW, d.fiber.texH, d.fiber.quantity, Name(d.semantics),
        Name(d.residence), Name(d.absence), d.slices, slices.c_str());
    return t;
}

int Tenant::Id() const { return m_s ? m_s->id : -1; }

const Lattice* Tenant::LatticeOf(uint32_t slice) const {
    if (!m_s) return nullptr;
    const SliceBinding* b = m_s->BindingOf(slice);
    return b ? &b->lattice : nullptr;
}

uint32_t Tenant::SliceOf(const std::string& latticeTag) const {
    return m_s ? m_s->SliceOf(latticeTag) : 0u;
}

void Tenant::Bind(TileTree& tree) {
    if (!m_s) return;
    tree.SetLattice(&m_s->desc.bindings.front().lattice);
    std::shared_ptr<State> s = m_s;
    tree.onChanged = [s](const std::string& tag, const TileRequest& r) {
        TileRequest q = r;
        q.face = s->SliceOf(tag) + r.face;
        s->mgr->Invalidate(s->id, q);
    };
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
