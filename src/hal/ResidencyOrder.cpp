// ResidencyOrder - step 5 (docs/HIERARCHY.md 4.19): the manager of one order.
// It is the engine's residency manager: the held set is the order's first P, the map a function of it.
//
// LAW 1, THE MAP IS A FUNCTION OF WHAT IS HELD. Today's byte is kept by increments (min on a map,
// max on an unmap: UpdateResidencyByte), which equal the function only while tiles are mapped
// coarse to fine and unmapped fine to coarse; four orders of events are not that (findings 3,
// 24, 63, 64). Here the byte over a cell is recomputed from the held set alone, over the
// footprint of every tile whose held state changed in the turn, once the turn's maps, claims and
// releases are all made. HELD is Mapped and landed: the boot's fill, a ring fill of a whole tile,
// a DirectStorage batch whose fence signalled -- the manager's own claim rule, and the audit's.
#include "hal/Residency.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace ga {

bool ResidencyManager::HeldInSlots(const void* ctx, uint32_t face, uint32_t mip, uint32_t x,
                                   uint32_t y) {
    const Tenant& t = *static_cast<const Tenant*>(ctx);
    const auto& ti = t.tilings[face * t.mips + mip];
    if (x >= ti.WidthInTiles || y >= ti.HeightInTiles) return false;
    const Tracked* p = t.slot[StampIndex(t, face, mip, x, y)];
    return p && p->state == TileState::Mapped && p->landed;
}

void ResidencyManager::WriteHeldFootprint(Tenant& t, const TileRequest& r, HeldFn held,
                                          const void* ctx) {
    if (r.face >= t.faces || r.mip >= t.mips || t.resMap.width == 0) return;
    const uint32_t face = r.face, top = TopOf(t, r.face), p0 = face * t.mips;
    if (r.mip > top) return;   // F1: above the slice's floor no tile is held and no byte speaks
    const uint32_t bw = t.tilings[p0].WidthInTiles, bh = t.tilings[p0].HeightInTiles;
    const uint32_t rdim = t.resMap.width;
    // The map is square and uv-addressed; a mip-0 cell spans sx x sy of its texels
    // (UpdateResidencyByte's arithmetic, which the audit reads back).
    const uint32_t sx = rdim / (std::max)(1u, bw), sy = rdim / (std::max)(1u, bh);
    // Mip-0 cells a side of one tile at mip m: every plane is a power of two tiles on each axis
    // (AddTextureInternal refuses any other), so a parent is (x >> 1, y >> 1) wherever it halves.
    const auto cw = [&](uint32_t m) {
        return bw / (std::max)(1u, static_cast<uint32_t>(t.tilings[p0 + m].WidthInTiles));
    };
    const auto ch = [&](uint32_t m) {
        return bh / (std::max)(1u, static_cast<uint32_t>(t.tilings[p0 + m].HeightInTiles));
    };
    std::vector<uint8_t>& bytes = t.resCpu[face];
    const auto fill = [&](uint32_t m, uint32_t x, uint32_t y, uint8_t v) {
        const uint32_t x0 = x * cw(m), x1 = (std::min)((x + 1) * cw(m), bw);
        const uint32_t y0 = y * ch(m), y1 = (std::min)((y + 1) * ch(m), bh);
        if (x1 <= x0) return;
        for (uint32_t cy = y0; cy < y1; ++cy) {
            for (uint32_t my = cy * sy; my < (cy + 1) * sy; ++my) {
                memset(&bytes[size_t(my) * rdim + size_t(x0) * sx], v, size_t(x1 - x0) * sx);
            }
        }
    };
    // Under a tile at m that is not held, over a whole chain: the level above it, or nothing.
    const auto below = [&](uint32_t m) -> uint8_t {
        return m >= top ? uint8_t(255) : static_cast<uint8_t>((m + 1) * 16);
    };
    // The chain above r, from the coarsest down: the first tile not held decides r's footprint.
    for (uint32_t m = top; m > r.mip; --m) {
        const uint32_t ax = (r.x * cw(r.mip)) / cw(m), ay = (r.y * ch(r.mip)) / ch(m);
        if (!held(ctx, face, m, ax, ay)) {
            fill(r.mip, r.x, r.y, below(m));
            return;
        }
    }
    // Whole above r: down through what is held. A tile's children are at most four, so the
    // depth-first stack holds at most three siblings a level and the walk itself.
    uint32_t stk[3 * 64];
    int n = 0;
    stk[n++] = r.mip;
    stk[n++] = r.x;
    stk[n++] = r.y;
    while (n) {
        const uint32_t y = stk[--n], x = stk[--n], m = stk[--n];
        if (!held(ctx, face, m, x, y)) {
            fill(m, x, y, below(m));
            continue;
        }
        if (m == 0) {
            fill(0, x, y, 0);
            continue;
        }
        const auto& ti = t.tilings[p0 + m - 1];
        const uint32_t k = cw(m) / cw(m - 1), l = ch(m) / ch(m - 1);   // 2 where the plane halves
        for (uint32_t cy = y * l; cy < (y + 1) * l && cy < ti.HeightInTiles; ++cy) {
            for (uint32_t cx = x * k; cx < (x + 1) * k && cx < ti.WidthInTiles; ++cx) {
                if (n + 3 > int(sizeof(stk) / sizeof(stk[0]))) {
                    throw std::runtime_error("residency: WriteHeldFootprint's stack overflowed");
                }
                stk[n++] = m - 1;
                stk[n++] = cx;
                stk[n++] = cy;
            }
        }
    }
}

void ResidencyManager::ApplyChanged() {
    if (m_changed.empty()) return;
    std::lock_guard<std::mutex> lk(m_mx);   // the loads write Tracked::state under it
    for (const auto& [k, r] : m_changed) {
        Tenant& t = m_tenants[k];
        WriteHeldFootprint(t, r, &HeldInSlots, &t);
        t.resDirty = true;
    }
    m_changed.clear();
}

void ResidencyManager::BirthMap(Gpu& gpu, int tenant) {
    Tenant& t = m_tenants[tenant];
    for (const auto& tr : t.tracked) WriteHeldFootprint(t, tr->req, &HeldInSlots, &t);
    // The GPU's copy is created zeroed, and zero says that mip 0 is here: it is written now, on
    // an upload that waits, so the first reader of any frame reads what the CPU's copy says.
    const uint32_t rdim = t.resMap.width;
    const uint32_t pitch = (rdim + 255u) & ~255u;
    // Each face on D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT (512), whatever rdim is.
    constexpr uint64_t kPlace = D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT;
    const uint64_t faceBytes = (static_cast<uint64_t>(pitch) * rdim + kPlace - 1u) & ~(kPlace - 1u);
    GpuBuffer stage = gpu.CreateUploadBuffer(faceBytes * t.faces, L"residency map at birth");
    auto* cl = gpu.BeginUpload();
    for (uint32_t f = 0; f < t.faces; ++f) {
        for (uint32_t y = 0; y < rdim; ++y) {
            memcpy(stage.cpu + static_cast<uint64_t>(f) * faceBytes + static_cast<uint64_t>(y) * pitch,
                   &t.resCpu[f][size_t(y) * rdim], rdim);
        }
        D3D12_TEXTURE_COPY_LOCATION dl{}, sl{};
        dl.pResource = t.resMap.res.Get();
        dl.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        dl.SubresourceIndex = f;
        sl.pResource = stage.res.Get();
        sl.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
        sl.PlacedFootprint.Offset = static_cast<uint64_t>(f) * faceBytes;
        sl.PlacedFootprint.Footprint = {DXGI_FORMAT_R8_UNORM, rdim, rdim, 1, pitch};
        cl->CopyTextureRegion(&dl, 0, 0, 0, &sl, nullptr);
    }
    gpu.Transition(cl, t.resMap, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE |
                                     D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    gpu.EndUpload();   // waits: the stage may die, the map is on the GPU
    t.resDirty = false;
    Log("[residency] %S: map born saying nothing (255), written from the %zu tiles the boot holds "
        "and on the GPU before any reader (step 5, law 9)",
        t.name.c_str(), t.tracked.size());
}


// ================================================================================================
//  THE ORDER (laws 2, 4 and 5; HIERARCHY 4.19). One comparison over every tile the manager knows:
//    1. a pin (a tile a standing reader asked for, or the floor a slice keeps) first;
//    2. then how lately it was wanted: 0 in a reader's standing statement, 1 in an earlier one
//       within the glance, 2 older. A reader's statement stands until it speaks again, and a
//       reader silent in a turn when another spoke has said nothing (F12); a
//       PREDICTION is a statement about an interval (1a): what it asks for stands until the
//       instant it predicted has passed, its lead of `m_predLead` frames;
//    3. then THE SIZE OF THE TILE'S TEXEL ON ITS READER'S SCREEN (F: the third and fourth keys made
//       one): its texel in metres over the distance from the reader's eye to the tile's nearest
//       point, floored at the texel, the larger the sooner; where several readers hold it, the
//       largest of theirs. Bucketed by its power of two, partitioned inside the straddling bucket
//       by the measure itself, the coarser mip and then the address breaking a tie.
//  THE HELD SET IS THE FIRST P OF IT. The loader reads the first tiles of the order not held; the
//  evictor releases the held tiles past the first P in two steps; a tile released and wanted
//  back before its slot is unmapped is taken back from the retire list (1b); bytes read for a
//  tile that has left the first P are let go and counted.
//
//  EVERY TILE'S PART OF IT IS ONE RECORD IN ONE ARRAY (m_rec, decision 3): the pass reads the
//  array front to back -- lateness, pin, bucket, the count a bucket, the cut as a walk over the
//  counts, the one straddling bucket gathered and partitioned by weight -- and touches a tile's
//  own object only for the few that need its state (the first P not held, the forgotten, the
//  released). The pass runs on events only: a statement that changed, a tile tracked or
//  untracked, a load that failed, a DirectStorage tile that landed, a want that expired, the
//  exact hold begun or ended.
// ================================================================================================
float ResidencyManager::Measure(float texel, float dist) {
    if (!(texel > 0.0f)) return 1.0f;   // a slice with no ground: as large as a tile can be
    return texel / (std::max)(dist, texel);
}

uint32_t ResidencyManager::MeasureBucket(float measure) {
    if (!(measure > 0.0f)) return kRungs - 1u;
    const int e = std::ilogb(measure);   // floor(log2): 0 for a texel as large as its distance
    return static_cast<uint32_t>(std::clamp(-e, 0, static_cast<int>(kRungs) - 1));
}

uint32_t ResidencyManager::RungIndexOf(int r0, uint32_t mip, uint32_t mips, int rungTop,
                                       int rungs) {
    if (r0 == kNoRung) {   // a slice that declared no ground: its own mips, ahead of all ground
        const uint32_t up = mips - 1u - mip;
        return up < 8u ? up : 7u;
    }
    const int r = r0 + static_cast<int>(mip);
    return static_cast<uint32_t>(std::clamp(rungTop - r, 8, rungs - 1));
}

uint32_t ResidencyManager::RungIndex(const Tenant& t, uint32_t face, uint32_t mip) {
    const int r0 = face < t.sliceRung0.size() ? t.sliceRung0[face] : kNoRung;
    return RungIndexOf(r0, mip, t.mips, static_cast<int>(kRungTop), static_cast<int>(kRungs));
}

ResidencyManager::Gate ResidencyManager::ParentGate(const Tenant& t, const Tracked* tr,
                                                   const Tracked** at) const {
    // Up the chain to the first HELD ancestor. A refused one on the way (not magnified) makes the
    // tile unreachable, however far up: its parent can never land under it. Any other ancestor not
    // held on the way (tracked and coming, or not tracked) makes it wait.
    if (at) *at = nullptr;
    bool first = true, waiting = false;
    uint32_t x = tr->req.x, y = tr->req.y;
    for (uint32_t m = tr->req.mip + 1; m <= TopOf(t, tr->req.face); ++m) {
        x >>= 1;
        y >>= 1;
        const Tracked* p = t.slot[StampIndex(t, tr->req.face, m, x, y)];
        if (p && p->state == TileState::Failed && p->magnified) continue;   // answered as its parent
        if (first && at) *at = p;
        first = false;
        if (!p) {   // not tracked: not held
            waiting = true;
            continue;
        }
        if (p->state == TileState::Mapped && p->landed) return waiting ? Gate::Wait : Gate::Open;
        if (p->state == TileState::Failed) return Gate::Refused;
        waiting = true;
    }
    return waiting ? Gate::Wait : Gate::Open;   // the floor reached: no parent above
}

namespace {
uint32_t WeightBits(float w) {
    if (!(w > 0.0f)) return 0u;
    uint32_t b = 0;
    memcpy(&b, &w, sizeof(b));   // a positive float orders as its bits
    return b;
}
float BitsWeight(uint32_t b) {
    float w = 0.0f;
    memcpy(&w, &b, sizeof(w));
    return w;
}
}  // namespace

int ResidencyManager::SlotFor(const OrdRec& r, int sid) {
    int k = -1, oldest = 0;
    for (int i = 0; i < OrdRec::kSlots && k < 0; ++i) {
        if (r.ssid[i] == sid) k = i;
    }
    for (int i = 0; i < OrdRec::kSlots && k < 0; ++i) {
        if (r.ssid[i] < 0) k = i;
    }
    for (int i = 1; i < OrdRec::kSlots && k < 0; ++i) {
        if (r.sstamp[i] < r.sstamp[oldest]) oldest = i;
    }
    return k >= 0 ? k : oldest;
}

void ResidencyManager::SlotApply(OrdRec& r, int sid, uint32_t stampFrame, uint32_t wbits) {
    const int k = SlotFor(r, sid);   // H5: this reader's statement only; the others stand as said
    if (r.ssid[k] != sid || r.sstamp[k] != stampFrame) {
        r.ssid[k] = static_cast<int8_t>(sid);
        r.sstamp[k] = stampFrame;
        r.sweight[k] = wbits;
    } else {
        r.sweight[k] = (std::min)(r.sweight[k], wbits);
    }
}

uint32_t ResidencyManager::StandingWeight(const OrdRec& r, uint32_t now, int predSid,
                                          uint32_t predLead, const uint32_t* sampLast) {
    // The nearest of the standing statements: a reader's stands until it speaks again, the
    // prediction's for its lead. None standing: the last frame's statements (the record's stamp).
    uint32_t best = UINT32_MAX, last = UINT32_MAX;
    for (int k = 0; k < OrdRec::kSlots; ++k) {
        const int s = r.ssid[k];
        if (s < 0) continue;
        const bool stands = s == predSid ? r.sstamp[k] + predLead >= now : sampLast[s] == r.sstamp[k];
        if (stands) best = (std::min)(best, r.sweight[k]);
        if (r.sstamp[k] == r.stamp) last = (std::min)(last, r.sweight[k]);
    }
    return best != UINT32_MAX ? best : last != UINT32_MAX ? last : WeightBits(1e30f);
}

void ResidencyManager::RecApply(OrdRec& r, int sid, uint32_t stampFrame, uint32_t wbits) const {
    const uint16_t bit = static_cast<uint16_t>(1u << sid);
    SlotApply(r, sid, stampFrame, wbits);
    if (r.stamp != stampFrame) {   // the first reader of this frame: the frame's own record
        r.stamp = stampFrame;
        r.mask = bit;
    } else {
        r.mask = static_cast<uint16_t>(r.mask | bit);
    }
    if (sid == m_predSid) r.predUntil = stampFrame + m_predLead;
    if ((m_pinMask >> sid) & 1u) r.pinFrame = stampFrame;
}

void ResidencyManager::OrderNote(Tenant& t, uint32_t rec, int tenant, int sid, uint32_t stampFrame,
                                 uint32_t face, uint32_t m, uint32_t x, uint32_t y, size_t idx,
                                 float nearM, float fu, float fv) {
    // THE WEIGHT (decision 2): metres from the reader's eye to the tile's NEAREST point. A walk's
    // leaf gives its own distance; a reader of one wide rect gives its eye's ground point in the
    // slice's uv (the focus) and the eye's height (nearM): the nearest point of the tile's
    // footprint on the ground, and up to the eye.
    float w = nearM;
    if (fu >= 0.0f) {
        const auto& ti = t.tilings[face * t.mips + m];
        const double W = double(ti.WidthInTiles), Hh = double(ti.HeightInTiles);
        const double u0 = x / W, u1 = (x + 1) / W, v0 = y / Hh, v1 = (y + 1) / Hh;
        const double du = fu < u0 ? u0 - fu : (fu > u1 ? fu - u1 : 0.0);
        const double dv = fv < v0 ? v0 - fv : (fv > v1 ? fv - v1 : 0.0);
        const double g0 = face < t.sliceGround0M.size() ? t.sliceGround0M[face] : 0.0;
        const double side = (g0 > 0.0 ? g0 : 1.0) * double(t.faceDim);   // metres across the slice
        w = static_cast<float>(std::hypot(std::hypot(du, dv) * side, double(nearM)));
    }
    const uint32_t wb = WeightBits(w);
    // F19: the least distance said of the tile this frame (the stamp is still the old one here:
    // Want's mark writes it after this note).
    if ((t.stamp[idx] >> 1) == stampFrame && t.leastSid[idx] == uint8_t(sid)) {
        t.least[idx] = (std::min)(t.least[idx], wb);
    } else {
        t.least[idx] = wb;
        t.leastSid[idx] = uint8_t(sid);
    }
    // THE STATEMENT's hash, in call order: did this reader say something else this frame.
    const uint64_t key = MakeKey(tenant, TileRequest{face, m, x, y});
    if (m_stFrame[sid] != stampFrame) {
        m_stFrame[sid] = stampFrame;
        m_stHash[sid] = kStatementBasis;
    }
    m_stHash[sid] = (m_stHash[sid] ^ key) * 1099511628211ull;
    if (rec != UINT32_MAX) {
        RecApply(m_rec[rec], sid, stampFrame, wb);
        return;
    }
    // Not tracked yet: Want tracks it right after this mark, and RecAdd takes this note.
    m_pend.tenant = tenant;
    m_pend.idx = idx;
    m_pend.r = OrdRec{};
    RecApply(m_pend.r, sid, stampFrame, wb);
}

void ResidencyManager::RecAdd(Tenant& t, int tenant, Tracked* tr) {
    OrdRec r;
    const size_t idx = StampIndex(t, tr->req.face, tr->req.mip, tr->req.x, tr->req.y);
    if (m_pend.tenant == tenant && m_pend.idx == idx) {
        r = m_pend.r;
        m_pend.tenant = -1;
    } else {   // tracked without a mark (the boot, a rescue): the stamp array's record, weight far
        r.stamp = t.stamp[idx] >> 1;
        r.mask = t.want[idx];
        r.weight = WeightBits(1e30f);
    }
    r.key = MakeKey(tenant, tr->req);
    r.tile = tr;
    r.rung = static_cast<uint8_t>(RungIndex(t, tr->req.face, tr->req.mip));
    {
        const double g0 = tr->req.face < t.sliceGround0M.size() ? t.sliceGround0M[tr->req.face] : 0.0;
        r.texel = g0 > 0.0 ? static_cast<float>(std::ldexp(g0, static_cast<int>(tr->req.mip))) : 0.0f;
    }
    r.flags = static_cast<uint8_t>((tr->req.mip + 1 == t.mips ? kFloorBit : 0) |
                                   (tr->state == TileState::Mapped && tr->landed ? kHeldBit : 0) |
                                   (tr->state == TileState::Failed ? kDeadBit : 0));
    r.bucket = kNoBucket;
    tr->rec = static_cast<uint32_t>(m_rec.size());
    t.recIx[idx] = tr->rec;   // F20: beside the stamp
    m_rec.push_back(r);
}

void ResidencyManager::RecDrop(Tracked* tr) {
    const uint32_t i = tr->rec;
    if (i == UINT32_MAX || i >= m_rec.size()) return;
    tr->rec = UINT32_MAX;
    {
        Tenant& t = m_tenants[tr->tenant];
        t.recIx[StampIndex(t, tr->req.face, tr->req.mip, tr->req.x, tr->req.y)] = UINT32_MAX;
    }
    if (i + 1 != m_rec.size()) {
        m_rec[i] = m_rec.back();
        Tracked* mv = m_rec[i].tile;
        mv->rec = i;
        Tenant& tm = m_tenants[mv->tenant];   // F20: the moved record's index, where its stamp is
        tm.recIx[StampIndex(tm, mv->req.face, mv->req.mip, mv->req.x, mv->req.y)] = i;
    }
    m_rec.pop_back();
}

void ResidencyManager::CountTail(uint32_t cut, uint32_t* byTenant, uint32_t* byRung,
                                 uint32_t& tail, uint32_t& wanted) const {
    // The wanted tiles (a pin, or lateness 0) of the last pass's order past `cut`, by tenant and
    // rung: the bucket that holds position `cut` is put in (weight, address) order on a copy.
    for (uint32_t i = 0; i < 16; ++i) byTenant[i] = 0;
    for (uint32_t i = 0; i < kRungs; ++i) byRung[i] = 0;
    tail = wanted = 0;
    const auto isWanted = [](uint32_t b) { return b != kNoBucket && (b / kRungs < 3u || b / kRungs == 3u); };
    std::vector<uint32_t> count(kBuckets + 1, 0u);
    for (const OrdRec& r : m_rec) {
        if (r.bucket == kNoBucket) continue;
        ++count[r.bucket];
        wanted += isWanted(r.bucket) ? 1u : 0u;
    }
    uint32_t acc = 0, s = kBuckets;
    for (uint32_t b = 0; b < kBuckets; ++b) {
        if (acc + count[b] > cut) {
            s = b;
            break;
        }
        acc += count[b];
    }
    if (s == kBuckets) return;
    std::vector<const OrdRec*> mid;
    for (const OrdRec& r : m_rec) {
        if (r.bucket == s) mid.push_back(&r);
    }
    std::nth_element(mid.begin(), mid.begin() + (cut - acc), mid.end(), [](const OrdRec* a, const OrdRec* b) {
        if (a->meas != b->meas) return a->meas > b->meas;
        const uint32_t ma = uint32_t(a->key >> 42) & 0x3Fu, mb = uint32_t(b->key >> 42) & 0x3Fu;
        return ma != mb ? ma > mb : a->key < b->key;
    });
    const auto add = [&](const OrdRec& r) {
        if (!isWanted(r.bucket)) return;
        ++tail;
        ++byTenant[(r.key >> 56) & 15u];
        ++byRung[r.bucket % kRungs];
    };
    for (size_t i = cut - acc; i < mid.size(); ++i) add(*mid[i]);
    for (const OrdRec& r : m_rec) {
        if (r.bucket != kNoBucket && r.bucket > s) add(r);
    }
}

// ---- H2: THE PASS'S PIECES (static: OrderPass and OrderSelfTest run the same code). ----------------
float ResidencyManager::PassMeasure(float texel, uint32_t weightBits, bool countsHeld, float margin) {
    const float m = Measure(texel, BitsWeight(weightBits));
    return countsHeld ? m * margin : m;   // a held tile gives its slot up only to one larger by more
}

void ResidencyManager::MarkUpFrom(std::vector<OrdRec>& rec, uint32_t i,
                                  uint32_t (*parentOf)(const void*, uint32_t), const void* ctx) {
    for (uint32_t a = parentOf(ctx, i); a != UINT32_MAX; a = parentOf(ctx, a)) {
        if (rec[a].flags & (kHeldBit | kUpBit)) break;   // held: its own chain is its own
        rec[a].flags |= kUpBit;
    }
}

void ResidencyManager::MarkAboveHeld(std::vector<OrdRec>& rec,
                                     uint32_t (*parentOf)(const void*, uint32_t), const void* ctx) {
    for (uint32_t i = 0; i < rec.size(); ++i) {
        if (rec[i].flags & kHeldBit) MarkUpFrom(rec, i, parentOf, ctx);
    }
}

void ResidencyManager::FindOrphans() {
    // Where the closure can break: a held tile whose parent is not held. Its own held state or its
    // parent's changed since the pass before (every change is noted for law 1's map, and a tile
    // let go leaves its held children without a parent), or it was one at the pass before.
    const auto heldAt = [&](int k, const TileRequest& r) {
        const Tracked* p = Find(k, r);
        return p && p->state == TileState::Mapped && p->landed;
    };
    std::vector<std::pair<int, TileRequest>> cand;
    cand.swap(m_orphans);
    for (const auto& [k, r] : m_edgeNotes) {
        cand.push_back({k, r});
        if (r.mip == 0) continue;
        for (uint32_t j = 0; j < 4u; ++j) {
            cand.push_back({k, TileRequest{r.face, r.mip - 1u, 2u * r.x + (j & 1u), 2u * r.y + (j >> 1)}});
        }
    }
    m_edgeNotes.clear();
    const auto key = [](const std::pair<int, TileRequest>& a) { return MakeKey(a.first, a.second); };
    std::sort(cand.begin(), cand.end(), [&](const auto& a, const auto& b) { return key(a) < key(b); });
    cand.erase(std::unique(cand.begin(), cand.end(), [&](const auto& a, const auto& b) { return key(a) == key(b); }),
               cand.end());
    for (const auto& [k, r] : cand) {
        if (!heldAt(k, r) || r.mip >= TopOf(m_tenants[k], r.face)) continue;   // F1: a floor has no parent
        if (heldAt(k, TileRequest{r.face, r.mip + 1u, r.x >> 1, r.y >> 1})) continue;
        m_orphans.push_back({k, r});
    }
    m_orphansMax = (std::max)(m_orphansMax, static_cast<uint64_t>(m_orphans.size()));
}

uint32_t ResidencyManager::ParentRecInSlots(const void* ctx, uint32_t i) {
    const ResidencyManager& M = *static_cast<const ResidencyManager*>(ctx);
    const Tracked* tr = M.m_rec[i].tile;
    const Tenant& t = M.m_tenants[tr->tenant];
    uint32_t x = tr->req.x, y = tr->req.y;
    for (uint32_t m = tr->req.mip + 1; m <= TopOf(t, tr->req.face); ++m) {   // the nearest tracked ancestor
        x >>= 1;
        y >>= 1;
        const Tracked* p = t.slot[StampIndex(t, tr->req.face, m, x, y)];
        if (p && p->rec != UINT32_MAX) return p->rec;
    }
    return UINT32_MAX;
}

uint32_t ResidencyManager::CutOrder(std::vector<OrdRec>& rec, const std::vector<uint32_t>& count,
                                    uint32_t cut, std::vector<uint32_t>& mid, uint32_t& keep,
                                    uint16_t units, uint32_t* unitPlanes) {
    uint32_t acc = 0, s = kBuckets;
    for (uint32_t b = 0; b < kBuckets; ++b) {
        if (acc + count[b] > cut) {
            s = b;
            break;
        }
        acc += count[b];
    }
    mid.clear();
    keep = 0;
    if (s < kBuckets) {
        mid.reserve(count[s]);
        for (uint32_t i = 0; i < rec.size(); ++i) {
            if (rec[i].bucket == s) mid.push_back(i);
        }
        const auto byWeight = [&](uint32_t a, uint32_t b) {   // the measure, the larger the sooner
            const OrdRec &ra = rec[a], &rb = rec[b];
            if (ra.meas != rb.meas) return ra.meas > rb.meas;
            const uint32_t ma = uint32_t(ra.key >> 42) & 0x3Fu, mb = uint32_t(rb.key >> 42) & 0x3Fu;
            // a tie: the coarser mip, the address (F18: a tile's planes together, its face last)
            return ma != mb ? ma > mb : TieKey(ra.key, units) < TieKey(rb.key, units);
        };
        keep = cut - acc;
        std::nth_element(mid.begin(), mid.begin() + keep, mid.end(), byWeight);
        // F18: THE CUT IS OVER TILES. The first record lost is the pivot; if its tile is a field's,
        // the planes of that tile among the kept are let go with it -- the tile goes whole, and the
        // held set is never more than the cut. One tile at most straddles: its planes are
        // consecutive in the order (TieKey), so no other tile is parted by this cut.
        if (units && keep > 0 && keep < mid.size()) {
            const uint64_t lost = rec[mid[keep]].key;
            if ((units >> (lost >> 56)) & 1u) {
                const uint64_t kNoFace = ~(uint64_t(0xFFu) << 48), tile = lost & kNoFace;
                const auto e = std::partition(mid.begin(), mid.begin() + keep,
                                              [&](uint32_t i) { return (rec[i].key & kNoFace) != tile; });
                const uint32_t moved = keep - static_cast<uint32_t>(e - mid.begin());
                keep -= moved;
                if (unitPlanes) *unitPlanes += moved;
            }
        }
        for (uint32_t j = 0; j < keep; ++j) rec[mid[j]].flags |= kInPBit;
    }
    for (OrdRec& r : rec) {
        if (r.bucket != kNoBucket && r.bucket < s) r.flags |= kInPBit;
    }
    return s;
}

bool ResidencyManager::OrderSelfTest() {
    // Records made to order: class in `rung` (pin * 3 + lateness, as the pass makes it), measure m
    // as a texel of 0.6 m * 2^mip at texel / m metres.
    const auto make = [](uint32_t cls, float m, bool held, uint32_t mip, uint32_t x) {
        OrdRec r;
        r.key = MakeKey(0, TileRequest{0, mip, x, 0});
        r.texel = std::ldexp(0.6f, static_cast<int>(mip));
        r.weight = WeightBits(r.texel / m);
        r.flags = held ? kHeldBit : 0;
        r.rung = static_cast<uint8_t>(cls);
        return r;
    };
    // The pass's (a'), (b) and (c) on them: the closure, the measure with the margin, the cut.
    const auto order = [](std::vector<OrdRec>& v, float margin, uint32_t cut, bool closure,
                          uint32_t (*parentOf)(const void*, uint32_t), const void* ctx) {
        for (OrdRec& r : v) r.flags = static_cast<uint8_t>(r.flags & ~(kInPBit | kUpBit));
        if (closure && margin != 1.0f) MarkAboveHeld(v, parentOf, ctx);
        std::vector<uint32_t> count(kBuckets + 1, 0u);
        for (OrdRec& r : v) {
            const float m = PassMeasure(r.texel, r.weight, (r.flags & (kHeldBit | kUpBit)) != 0, margin);
            memcpy(&r.meas, &m, sizeof(m));
            r.bucket = static_cast<uint16_t>(r.rung * kRungs + MeasureBucket(m));
            ++count[r.bucket];
        }
        std::vector<uint32_t> mid;
        uint32_t keep = 0;
        CutOrder(v, count, cut, mid, keep);
    };
    const auto noParent = [](const void*, uint32_t) { return UINT32_MAX; };
    bool ok = true;
    // (1) THE MARGIN AT 1 IS F'S ORDER: F's key by a whole sort, against the pass at margin 1.
    std::vector<OrdRec> v;
    uint32_t seed = 12345u;
    const auto rnd = [&]() {
        seed = seed * 1664525u + 1013904223u;
        return seed >> 8;
    };
    const uint32_t N = 3000;
    for (uint32_t i = 0; i < N; ++i) {
        const uint32_t c = rnd() % 100u;
        const uint32_t cls = c < 5u ? 0u : c < 75u ? 3u : c < 90u ? 4u : 5u;
        float m = std::ldexp(0.5f + 1.5f * float(rnd() % 10000u) / 10000.0f, -9);
        if (i > 10 && rnd() % 10u == 0u) m = BitsWeight(v[i - 7].meas);   // a tie now and then
        OrdRec r = make(cls, m, rnd() % 2u == 0u, rnd() % 8u, i);
        memcpy(&r.meas, &m, sizeof(m));
        v.push_back(r);
    }
    const auto fFirstP = [](const std::vector<OrdRec>& w, uint32_t cut) {
        std::vector<uint32_t> idx(w.size()), b(w.size());
        std::vector<float> m(w.size());
        for (uint32_t i = 0; i < w.size(); ++i) {
            idx[i] = i;
            m[i] = Measure(w[i].texel, BitsWeight(w[i].weight));
            b[i] = w[i].rung * kRungs + MeasureBucket(m[i]);
        }
        std::sort(idx.begin(), idx.end(), [&](uint32_t x, uint32_t y) {
            if (b[x] != b[y]) return b[x] < b[y];
            if (m[x] != m[y]) return m[x] > m[y];
            const uint32_t mx = uint32_t(w[x].key >> 42) & 0x3Fu, my = uint32_t(w[y].key >> 42) & 0x3Fu;
            return mx != my ? mx > my : w[x].key < w[y].key;
        });
        std::vector<bool> in(w.size(), false);
        for (uint32_t j = 0; j < cut && j < idx.size(); ++j) in[idx[j]] = true;
        return in;
    };
    uint32_t differ1 = 0, differM = 0;
    const uint32_t cuts[4] = {N / 4, N / 2, 3 * N / 4, N - 1};
    for (const uint32_t cut : cuts) {
        const std::vector<bool> f = fFirstP(v, cut);
        order(v, 1.0f, cut, true, noParent, nullptr);
        for (uint32_t i = 0; i < N; ++i) differ1 += ((v[i].flags & kInPBit) != 0) != f[i];
        order(v, 1.41421356f, cut, true, noParent, nullptr);
        for (uint32_t i = 0; i < N; ++i) differM += ((v[i].flags & kInPBit) != 0) != f[i];
    }
    Log("[order-test] with the margin at 1 the order is F's: the first P at 4 cuts of %u records "
        "(classes, measures over two buckets, ties) against F's key by a whole sort: %u differ -- %s; "
        "at the margin 1.41421 the same records differ in %u places (the test sees a margin)",
        N, differ1, differ1 == 0 ? "THE SAME" : "NOT THE SAME", differM);
    ok = ok && differ1 == 0 && differM > 0;
    // (2) A HELD TILE AT m IS KEPT AGAINST 1.4 m AND GIVES WAY TO 1.5 m, inside a bucket and where the
    // margin lifts it into the bucket above; the plant: at margin 1 it gives way to 1.4 m.
    bool twoOk = true, plantSeen = true;
    for (const float base : {std::ldexp(1.2f, -9), std::ldexp(0.9f, -9)}) {
        for (const float k : {1.4f, 1.5f}) {
            std::vector<OrdRec> w{make(3, base, true, 0, 0), make(3, base * k, false, 0, 1)};
            order(w, 1.41421356f, 1, true, noParent, nullptr);
            const bool kept = (w[0].flags & kInPBit) != 0 && !(w[1].flags & kInPBit);
            twoOk = twoOk && (k < 1.41f ? kept : !kept);
            if (k < 1.41f) {
                order(w, 1.0f, 1, true, noParent, nullptr);
                plantSeen = plantSeen && !(w[0].flags & kInPBit);
            }
        }
    }
    Log("[order-test] at the margin 1.41421 a held tile at measure m is kept against a tile at 1.4 m "
        "and gives way to one at 1.5 m, inside a bucket (m = 1.2 * 2^-9) and across a bucket's edge "
        "(m = 0.9 * 2^-9): %s; the plant, the margin at 1, gives way at 1.4 m: %s",
        twoOk ? "HOLDS" : "FAILS", plantSeen ? "CAUGHT" : "NOT CAUGHT");
    ok = ok && twoOk && plantSeen;
    // (3) THE HELD SET CLOSED UPWARD WITH THE MARGIN ON: a held child C at m, its parent P and
    // grandparent G not held at 1.2 m and 1.25 m (a parent's measure is no less than its child's),
    // a stranger X at 1.3 m. At every cut, a held tile in the first P has every ancestor in it.
    // The plant: the rule off, so P counts its measure alone against C's margin.
    static const uint32_t kUp[4] = {1u, 2u, UINT32_MAX, UINT32_MAX};   // C -> P -> G; X alone
    const auto chainParent = [](const void*, uint32_t i) { return i < 4u ? kUp[i] : UINT32_MAX; };
    const float m0 = std::ldexp(1.1f, -9);
    const auto closedAt = [&](bool rule, bool parentHeld, uint32_t cut) {
        std::vector<OrdRec> w{make(3, m0, true, 0, 0), make(3, 1.2f * m0, parentHeld, 1, 0),
                              make(3, 1.25f * m0, false, 2, 0), make(3, 1.3f * m0, false, 0, 9)};
        order(w, 1.41421356f, cut, rule, chainParent, nullptr);
        for (uint32_t i = 0; i < 4u; ++i) {
            if (!(w[i].flags & kHeldBit) || !(w[i].flags & kInPBit)) continue;
            for (uint32_t a = kUp[i]; a != UINT32_MAX; a = kUp[a]) {
                if (!(w[a].flags & kInPBit)) return false;
            }
        }
        return true;
    };
    bool closed = true, broken = false;
    for (uint32_t cut = 1; cut <= 4u; ++cut) {
        closed = closed && closedAt(true, false, cut) && closedAt(true, true, cut);
        broken = broken || !closedAt(false, false, cut);
    }
    Log("[order-test] the held set closed upward with the margin on: a held child at m, its parent and "
        "grandparent not held at 1.2 m and 1.25 m, a stranger at 1.3 m, and again with the parent held: "
        "at every cut a held tile in the first P has its ancestors in it: %s; the plant, a tile above a "
        "held tile counted at its measure alone, keeps the child and lets its parent go: %s",
        closed ? "HOLDS" : "FAILS", broken ? "CAUGHT" : "NOT CAUGHT");
    ok = ok && closed && broken;
    // (4) H5, THE WEIGHT STANDS WITH ITS STATEMENT: the view (every frame) and the prediction (its
    // (3) F18: A FIELD'S PLANES ARE ONE TILE. Three tiles of four planes at one measure (one bucket,
    // one tie), the cut at 6 records. Without the unit the cut keeps a tile and a half; with it the
    // half is let go: 4 kept, every kept tile whole, and the loser is the pivot's tile. A cut at 8
    // keeps two tiles whole either way (the plant: the unit never changes a cut on a tile's edge).
    {
        const float m = std::ldexp(1.1f, -9);
        auto field = [&](uint32_t cut, uint16_t units, uint32_t& keptOut, bool& wholeOut, uint32_t& planesOut) {
            std::vector<OrdRec> w;
            for (uint32_t x = 0; x < 3; ++x) {
                for (uint32_t f = 6; f < 10; ++f) {
                    OrdRec r;
                    r.key = MakeKey(1, TileRequest{f, 0, x, 0});
                    r.texel = 0.6f;
                    r.weight = WeightBits(r.texel / m);
                    r.rung = 3;
                    memcpy(&r.meas, &m, sizeof(m));
                    r.bucket = static_cast<uint16_t>(3u * kRungs + MeasureBucket(m));
                    w.push_back(r);
                }
            }
            std::vector<uint32_t> count(kBuckets + 1, 0u);
            for (const OrdRec& r : w) ++count[r.bucket];
            std::vector<uint32_t> mid;
            uint32_t keep = 0;
            planesOut = 0;
            CutOrder(w, count, cut, mid, keep, units, &planesOut);
            keptOut = 0;
            wholeOut = true;
            for (uint32_t x = 0; x < 3; ++x) {
                uint32_t k = 0;
                for (uint32_t f = 0; f < 4; ++f) k += (w[x * 4 + f].flags & kInPBit) != 0;
                keptOut += k;
                wholeOut = wholeOut && (k == 0 || k == 4);
            }
        };
        uint32_t k6u = 0, k6p = 0, k8u = 0, p6u = 0, p6p = 0, p8u = 0;
        bool w6u = false, w6p = false, w8u = false;
        field(6, 0, k6p, w6p, p6p);        // the plant: no unit, the cut parts a tile
        field(6, 1u << 1, k6u, w6u, p6u);  // the unit: the parted tile goes whole
        field(8, 1u << 1, k8u, w8u, p8u);  // on a tile's edge the unit changes nothing
        const bool unitOk = k6u == 4 && w6u && p6u == 2 && k8u == 8 && w8u && p8u == 0;
        Log("[order-test] a field's planes are one tile (F18): 3 tiles x 4 planes at one measure, the cut "
            "at 6: without the unit %u kept, whole tiles %s (the plant: %s); with it %u kept, whole tiles "
            "%s, %u planes let go; at 8: %u kept, whole %s, %u let go -- %s",
            k6p, w6p ? "yes" : "no", (k6p == 6 && !w6p) ? "CAUGHT" : "NOT CAUGHT", k6u, w6u ? "yes" : "no",
            p6u, k8u, w8u ? "yes" : "no", p8u, unitOk ? "HOLDS" : "FAILS");
        ok = ok && unitOk && k6p == 6 && !w6p;
    }
    // lead 24) on one tile. The prediction's nearer distance stands on the frames it is silent; a
    // reader that speaks again replaces its own slot only; a statement that no longer stands leaves
    // the measure. The plant is F's rule: the weight remade each frame from the readers who spoke.
    {
        const int view = 0, pred = 1;
        uint32_t last[2] = {0, 0};
        OrdRec r;
        const auto say = [&](int s, uint32_t f, float dist) {
            SlotApply(r, s, f, WeightBits(dist));
            r.stamp = f;
            last[s] = f;
        };
        const auto at = [&](uint32_t now) { return BitsWeight(StandingWeight(r, now, pred, 24u, last)); };
        const auto fRule = [&]() {
            uint32_t w = UINT32_MAX;
            for (int k = 0; k < OrdRec::kSlots; ++k) {
                if (r.ssid[k] >= 0 && r.sstamp[k] == r.stamp) w = (std::min)(w, r.sweight[k]);
            }
            return BitsWeight(w);
        };
        say(view, 10, 100.0f);
        say(pred, 10, 50.0f);
        const float w10 = at(10);
        say(view, 11, 100.0f);   // the view again; the prediction silent
        const float w11 = at(11), plant = fRule();
        say(view, 35, 100.0f);   // the prediction's lead has passed
        const float w35 = at(35);
        say(pred, 36, 80.0f);    // the prediction again: its own slot only
        say(view, 36, 100.0f);
        const float w36 = at(36);
        last[view] = 37;         // the view speaks without the tile: its statement falls
        const float w37 = at(37);
        const bool standOk = w10 == 50.0f && w11 == 50.0f && w35 == 100.0f && w36 == 80.0f && w37 == 80.0f;
        Log("[order-test] the weight stands with its statement (H5): the view at 100 m every frame, the "
            "prediction at 50 m at frame 10 with a lead of 24: %.0f m at frame 10, %.0f at 11 (the plant, "
            "remade from frame 11's readers, says %.0f: %s), %.0f at 35 (the lead passed), %.0f at 36 (the "
            "prediction again, at 80), %.0f at 37 (the view silent on it): %s",
            w10, w11, plant, plant != w11 ? "CAUGHT" : "NOT CAUGHT", w35, w36, w37, standOk ? "HOLDS" : "FAILS");
        ok = ok && standOk && plant != w11;
    }
    Log(ok ? "[order-test] ---- PASS (H2: the hold's margin, the order at margin 1 F's, the closure; H5: "
             "the weight stands with its statement) ----"
           : "[order-test] ---- FAIL (H2/H5) ----");
    return ok;
}

// THE ORDER of the first P not held: the bucket, then the measure (the larger the sooner), then a
// tile's planes together (F18). Total: no two entries are equal.
bool ResidencyManager::NeedBefore(const OrderEntry& a, const OrderEntry& b, uint16_t units) {
    if (a.bucket != b.bucket) return a.bucket < b.bucket;
    if (a.weight != b.weight) return a.weight > b.weight;
    const uint32_t ma = uint32_t(a.key >> 42) & 0x3Fu, mb = uint32_t(b.key >> 42) & 0x3Fu;
    return ma != mb ? ma > mb : TieKey(a.key, units) < TieKey(b.key, units);
}

void ResidencyManager::SortNeedTo(size_t n) {
    n = (std::min)(n, m_need.size());
    if (n <= m_needSorted) return;
    const auto sort0 = std::chrono::steady_clock::now();
    const uint16_t units = m_needUnits;
    const auto before = [units](const OrderEntry& a, const OrderEntry& b) { return NeedBefore(a, b, units); };
    const auto first = m_need.begin() + static_cast<std::ptrdiff_t>(m_needSorted);
    const auto last = m_need.begin() + static_cast<std::ptrdiff_t>(n);
    if (last != m_need.end()) std::nth_element(first, last, m_need.end(), before);
    std::sort(first, last, before);
    m_needSorted = n;
    passSortMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - sort0).count();
}

void ResidencyManager::OrderPass(OrderTurnLedger& L) {
    const auto p0 = std::chrono::steady_clock::now();
    auto pc = p0;
    const auto passLap = [&](int k) {
        const auto c = std::chrono::steady_clock::now();
        passMs[k] += std::chrono::duration<double, std::milli>(c - pc).count();
        pc = c;
    };
    ++passTurns;
    m_passFrame = m_frame;
    // ---- (a) failures and tiles not whole, said by the loads since the last pass: unreachable.
    std::vector<const Tracked*> refused;   // refused this pass, not held: the guard walks down from them
    for (const auto& [k, r] : m_failedKeys) {
        const Tracked* tr = Find(k, r);
        if (tr && tr->rec != UINT32_MAX) m_rec[tr->rec].flags |= kDeadBit;
        if (tr && tr->state == TileState::Failed && !tr->magnified) refused.push_back(tr);
    }
    m_failedKeys.clear();
    // ---- (a'') THE GUARD (4.7 at the moment a parent is refused): a tile held under a parent
    // refused and not held is released this turn. The loader asks no child before its parent is
    // held and the version law keeps a refused replacement's tile held, so this should never fire;
    // its count must read 0.
    if (!refused.empty()) {
        // Down from each refused tile through the tracked slots below it (a held tile under it has
        // a tracked chain to it: the want walk tracks every ancestor of what it asks for).
        bool compact = false;
        std::vector<Tracked*> under;
        std::vector<const Tracked*> stack(refused.begin(), refused.end());
        while (!stack.empty()) {
            const Tracked* p = stack.back();
            stack.pop_back();
            if (p->req.mip == 0) continue;
            const Tenant& t = m_tenants[p->tenant];
            for (uint32_t j = 0; j < 4u; ++j) {
                Tracked* c = t.slot[StampIndex(t, p->req.face, p->req.mip - 1u, 2u * p->req.x + (j & 1u),
                                               2u * p->req.y + (j >> 1))];
                if (!c) continue;
                if (c->state == TileState::Mapped && c->landed && !c->dropped && c->pos != UINT32_MAX) {
                    under.push_back(c);
                }
                stack.push_back(c);
            }
        }
        for (Tracked* tr : under) {
            Tenant& t = m_tenants[tr->tenant];
            const std::shared_ptr<Tracked> keep = t.tracked[tr->pos];
            if (releasedUnderRefusedTotal < 12) {
                Log("[residency] order: THE GUARD released %s, held under a refused parent not held",
                    TileName(MakeKey(tr->tenant, tr->req)).c_str());
            }
            Untrack(t, tr);
            if (DropOne(keep)) compact = true;
            ++releasedUnderRefusedTotal;
        }
        if (compact) std::erase_if(m_mapped, [](const std::shared_ptr<Tracked>& p) { return p->dropped; });
    }
    // ---- (a') H2, THE HELD SET CLOSED UPWARD UNDER THE MARGIN. A child that lands before its
    // parent (law 3) is held while the parent is not; it would count for the margin and its parent
    // only for its measure, which is no less than the child's but may be less than the margin
    // times it, and a cut between them would let the parent go while the child stays held under
    // a hole. A tile above a held tile counts as held.
    const float margin = holdMargin;
    if (margin != 1.0f) {
        // Walked from where it can break (FindOrphans), not from every held tile: the same marks.
        FindOrphans();
        for (const auto& [k, r] : m_orphans) {
            const Tracked* tr = Find(k, r);
            if (tr && tr->rec != UINT32_MAX) MarkUpFrom(m_rec, tr->rec, &ParentRecInSlots, this);
        }
        if (auditEvery) {   // an audited run checks them against the law walked from every held tile
            std::vector<uint8_t> ev(m_rec.size());
            for (size_t i = 0; i < m_rec.size(); ++i) {
                ev[i] = m_rec[i].flags & kUpBit;
                m_rec[i].flags = static_cast<uint8_t>(m_rec[i].flags & ~kUpBit);
            }
            MarkAboveHeld(m_rec, &ParentRecInSlots, this);
            uint64_t miss = 0;
            for (size_t i = 0; i < m_rec.size(); ++i) miss += (ev[i] != 0) != ((m_rec[i].flags & kUpBit) != 0);
            ++m_closureChecks;
            if (miss) {
                ++m_closureMissTurns;
                m_closureMissMax = (std::max)(m_closureMissMax, miss);
            }
        }
    } else {
        m_edgeNotes.clear();
        m_orphans.clear();
    }
    if (passTurns == 1) {
        Log("[order] the hold's margin x%.6g (streaming.holdMargin): a held tile, and a tile above a "
            "held tile, count for it times their measure%s",
            margin, margin == 1.0f ? " -- 1: the order without it (F's)" : "");
    }
    passLap(0);
    // ---- (b) ONE PASS FRONT TO BACK: lateness, pin, bucket, the count a bucket.
    std::vector<uint32_t> count(kBuckets + 1, 0u);
    uint32_t wantSet = 0;
    m_nextExpiry = UINT32_MAX;
    const uint32_t now = m_frame;
    // F12: WHOSE STATEMENT STANDS. A reader's stands until it speaks again -- and a reader silent
    // in a turn when another spoke has spoken: it said nothing (a window that left the view, a
    // subject that left the scene). The pins' rule (m_pinSpoke), said of every reader. A turn in
    // which no reader spoke (a tool's wait) leaves every statement standing.
    uint32_t live[kMaxSamplers];
    for (int s = 0; s < kMaxSamplers; ++s) live[s] = m_sampLast[s] == m_lastSpoke ? m_sampLast[s] : 0u;
    for (OrdRec& r : m_rec) {
        const bool countsHeld = (r.flags & (kHeldBit | kUpBit)) != 0;   // H2
        r.flags = static_cast<uint8_t>((r.flags & ~(kInPBit | kForgetBit | kWasInBit | kUpBit)) |
                                       ((r.flags & kInPBit) ? kWasInBit : 0));   // H1's memory
        if (r.flags & kDeadBit) {
            r.bucket = kNoBucket;
            continue;
        }
        bool stand = false;   // in a reader's standing statement
        if (settleExact) {
            stand = r.stamp == now;   // the hold's own definition: what the walks stamped now
        } else {
            stand = r.predUntil >= now;
            for (uint16_t mk = static_cast<uint16_t>(r.mask & ~(m_predSid >= 0 ? (1u << m_predSid) : 0u));
                 mk && !stand; mk = static_cast<uint16_t>(mk & (mk - 1))) {
                unsigned s = 0;
                while (!((mk >> s) & 1u)) ++s;
                stand = live[s] == r.stamp;   // its reader has not spoken since, nor been silent
            }
        }
        const uint32_t late = stand ? 0u : (r.stamp != 0u && now - r.stamp <= kGlanceTurns) ? 1u : 2u;
        const bool pin = (r.flags & kFloorBit) ||
                         (settleExact ? (r.stamp == now && (r.mask & m_pinMask))
                                      : (r.pinFrame != 0u && r.pinFrame == m_pinSpoke));
        if (late == 1) m_nextExpiry = (std::min)(m_nextExpiry, r.stamp + kGlanceTurns + 1u);
        if (r.predUntil >= now) m_nextExpiry = (std::min)(m_nextExpiry, r.predUntil + 1u);
        if (!(r.flags & kHeldBit) && !pin && late == 2) {
            r.flags |= kForgetBit;   // leaves the order if it is not busy (its state, below)
            r.bucket = kNoBucket;
            continue;
        }
        r.weight = StandingWeight(r, now, m_predSid, m_predLead, live);   // H5
        const float meas = PassMeasure(r.texel, r.weight, countsHeld, margin);
        memcpy(&r.meas, &meas, sizeof(r.meas));
        r.bucket = static_cast<uint16_t>(((pin ? 0u : 1u) * 3u + late) * kRungs + MeasureBucket(meas));
        ++count[r.bucket];
        if (pin || late == 0) ++wantSet;
    }
    L.candidates = static_cast<uint32_t>(m_rec.size());
    passEntries += m_rec.size();
    passLap(1);
    // ---- (c) THE CUT: a walk over the counts; the one straddling bucket gathered and partitioned.
    const uint32_t P = kPoolCapTiles - kSlotReserve;
    const uint32_t cut = settleExact ? wantSet : P;
    m_passCut = cut;
    std::vector<uint32_t> mid;   // record indices of the straddling bucket
    uint32_t kept = 0;   // of the straddling bucket
    const uint16_t units = UnitTenants();   // F18: the tenants whose planes are one tile
    uint32_t unitPlanes = 0;
    const uint32_t s = CutOrder(m_rec, count, cut, mid, kept, units, &unitPlanes);
    if (unitPlanes) {
        ++loader.unitCuts;
        loader.unitPlanes += unitPlanes;
    }
    if (s < kBuckets) {
        // The straddling bucket's boundary in the measure's terms: the smallest measure kept, the
        // largest lost; and, by rung, the farthest kept and the nearest lost in metres from the eye.
        m_cutRung = s;
        m_keptFar = UINT32_MAX;   // here: the smallest measure kept
        m_lostNear = 0;           // here: the largest measure lost
        for (uint32_t i = 0; i < kRungs; ++i) m_cutKeptM[i] = m_cutLostM[i] = -1.0f;
        m_cutUnitPlanes = unitPlanes;   // F18: mid[kept .. kept + unitPlanes) are the pivot's tile's
        m_cutUnitMeas = unitPlanes ? m_rec[mid[kept]].meas : 0u;
        for (uint32_t j = 0; j < mid.size(); ++j) {
            const OrdRec& r = m_rec[mid[j]];
            const float d = BitsWeight(r.weight);
            if (j >= kept && j < kept + unitPlanes) continue;   // let go with its tile, by the law, not by the measure
            if (j < kept) {
                if (r.meas <= m_keptFar) {
                    m_keptFar = r.meas;
                    m_keptFarTile = r.key;
                }
                m_cutKeptM[r.rung] = (std::max)(m_cutKeptM[r.rung], d);
            } else {
                if (r.meas >= m_lostNear) {
                    m_lostNear = r.meas;
                    m_lostNearTile = r.key;
                }
                if (m_cutLostM[r.rung] < 0.0f || d < m_cutLostM[r.rung]) m_cutLostM[r.rung] = d;
            }
        }
    } else {
        m_cutRung = kNoBucket;
    }
    const bool motion = traceTurn || auditEvery != 0;   // the instrument rides the traced runs only
    for (OrdRec& r : m_rec) {
        if (!motion) break;
        if (r.bucket == kNoBucket) continue;
        // H1: THE MEASURE'S MOTION AND THE CUT'S CROSSINGS, a ledger (nothing here decides): the
        // reader's measure against its own at the pass before, and a tile that changed sides.
        const float mNow = Measure(r.texel, BitsWeight(r.weight));
        if (r.ppass != 0 && r.ppass + 1 == passTurns) {
            const float mPrev = BitsWeight(r.pmeas);
            const float q = mPrev > 0.0f ? mNow / mPrev : 1.0f;
            const float hi = q >= 1.0f ? q : 1.0f / q;
            OrderMotion& M = m_motion;
            ++M.pairs;
            if (hi > 1.01f) ++M.left[0];
            if (hi > 1.1f) ++M.left[1];
            if (hi > 1.41f) ++M.left[2];
            if (hi > 2.0f) ++M.left[3];
            if (hi > M.qmax) {
                M.qmax = hi;
                M.qmaxPrev = mPrev;
                M.qmaxNow = mNow;
                M.qmaxKey = r.key;
                M.qmaxFrame = now;
            }
            const bool in = (r.flags & kInPBit) != 0, was = (r.flags & kWasInBit) != 0;
            if (in != was) {
                ++(in ? L.crossIn : L.crossOut);
                ++M.crossOwn[hi == 1.0f ? 0 : hi <= 1.01f ? 1 : hi <= 1.1f ? 2 : hi <= 1.41f ? 3 : hi <= 2.0f ? 4 : 5];
                float gap = 0.0f;
                if (!in && (r.flags & kHeldBit)) {
                    gap = r.bucket == s ? BitsWeight(m_keptFar) / (std::max)(BitsWeight(r.meas), 1e-30f) : 0.0f;
                    ++M.heldOutGap[r.bucket != s ? 4 : gap <= 1.01f ? 0 : gap <= 1.1f ? 1 : gap <= 1.41f ? 2 : gap <= 2.0f ? 3 : 4];
                }
                const int w = now >= 1060u ? 1 : now >= 400u ? 0 : -1;
                if (w >= 0 && M.printed[w] < 20u) {
                    ++M.printed[w];
                    const auto who = [&](uint32_t mk) {
                        std::string o;
                        for (int i = 0; i < kMaxSamplers; ++i) {
                            if (!((mk >> i) & 1u)) continue;
                            if (!o.empty()) o += "+";
                            o += SamplerName(i);
                        }
                        return o.empty() ? std::string("none") : o;
                    };
                    const auto cls = [](uint16_t b) {
                        char t[48];
                        if (b == kNoBucket) return std::string("-");
                        snprintf(t, sizeof(t), "%s late %u 2^-%u", b / kRungs < 3u ? "pin" : "want",
                                 (b / kRungs) % 3u, b % kRungs);
                        return std::string(t);
                    };
                    Log("[order-cross] f%u pass %llu | %s %s | %s | measure %.5g -> %.5g (x%.4f) | "
                        "distance %.1f -> %.1f m (texel %.3g m) | %s -> %s | readers %s -> %s (spoke "
                        "this turn: %s) | stamp f%u, prediction stands to f%u | the cut: smallest kept "
                        "%.5g, largest lost %.5g%s",
                        now, static_cast<unsigned long long>(passTurns), in ? "IN " : "OUT",
                        TileName(r.key).c_str(), (r.flags & kHeldBit) ? "held" : "not held", mPrev,
                        mNow, q, mPrev > 0.0f ? r.texel / mPrev : 0.0f, BitsWeight(r.weight), r.texel,
                        cls(r.pbucket).c_str(), cls(r.bucket).c_str(), who(r.pmask).c_str(),
                        who(r.mask).c_str(), who(L.spoke).c_str(), r.stamp, r.predUntil,
                        BitsWeight(m_keptFar), BitsWeight(m_lostNear),
                        gap > 0.0f ? (" | held, lost by x" + std::to_string(gap)).c_str() : "");
                }
            }
        }
        r.ppass = static_cast<uint32_t>(passTurns);
        memcpy(&r.pmeas, &mNow, sizeof(r.pmeas));
        r.pmask = r.mask;
        r.pbucket = r.bucket;
    }
    if (L.crossIn || L.crossOut) {
        m_motion.crossIn += L.crossIn;
        m_motion.crossOut += L.crossOut;
        ++m_motion.passesCrossed;
        m_motion.lastCrossFrame = now;
    }
    // The first P not held, in the order: the loader's list and the gather's.
    m_need.clear();
    for (const OrdRec& r : m_rec) {
        // F13: a held tile whose bytes went stale (the version law) is in the loader's list at
        // its own measure: its refill competes with every other load as a want does.
        if (!(r.flags & kInPBit) || ((r.flags & kHeldBit) && !r.tile->stale)) continue;
        m_need.push_back({r.bucket, r.meas, r.key, r.tile});
        r.tile->firstP = now;
    }
    passNeed += m_need.size();
    m_needSorted = 0;
    m_needUnits = units;
    SortNeedTo(kNeedChunk);   // the front; the readers sort on as they reach past it
    // The want's tail past this cut, by tenant and rung ([order-tail]).
    for (uint32_t i = 0; i < 16; ++i) m_tailTenant[i] = 0;
    for (uint32_t i = 0; i < kRungs; ++i) m_tailRung[i] = 0;
    m_tailTotal = m_wantTotal = 0;
    for (const OrdRec& r : m_rec) {
        if (r.bucket == kNoBucket || !(r.bucket / kRungs < 3u || r.bucket / kRungs == 3u)) continue;
        ++m_wantTotal;
        if (r.flags & kInPBit) continue;
        ++m_tailTotal;
        ++m_tailTenant[(r.key >> 56) & 15u];
        ++m_tailRung[r.bucket % kRungs];
    }
    passLap(2);
    // ---- (d) THE EVICTOR: every held tile past the first P is released, in two steps, and may be
    // taken back while its slot retires (1b). A tile whose bytes are in flight is not held yet.
    std::vector<Tracked*> release, forget;
    for (const OrdRec& r : m_rec) {
        if ((r.flags & kHeldBit) && !(r.flags & kInPBit) && r.bucket != kNoBucket) release.push_back(r.tile);
        if ((r.flags & kForgetBit) && r.tile->state == TileState::Seen) forget.push_back(r.tile);
    }
    bool compact = false;
    for (Tracked* tr : release) {
        ++releaseLedger.cut;   // PHASE B2w
        if (LastSeenOf(tr) + 1u >= m_frame) ++releaseLedger.cutNamed;
        Tenant& t = m_tenants[tr->tenant];
        const std::shared_ptr<Tracked> keep = t.tracked[tr->pos];   // outlives Untrack
        Untrack(t, tr);
        if (DropOne(keep)) {
            compact = true;
            m_retiring.back().rescuable = true;   // good bytes, released only for the cut
        }
        ++turn.evicted;
        ++releasedTotal;
        if (settleExact) {
            ++t.exDropped;
            ++t.exDroppedMapped;
        }
    }
    if (compact) {
        std::erase_if(m_mapped, [](const std::shared_ptr<Tracked>& p) { return p->dropped; });
    }
    for (Tracked* tr : forget) {
        if (tr->pos != UINT32_MAX) Untrack(m_tenants[tr->tenant], tr);
    }
    L.forgotten = static_cast<uint32_t>(forget.size());
    // ---- (e) THE RESCUE (1b), before the loader reads anything.
    Rescue(L);
    passLap(3);
    m_passEpoch = m_trackEpoch;
    m_passFails = m_failEvents;
    m_passClaims = m_claimEvents;
    m_passHold = settleExact;
}

void ResidencyManager::Rescue(OrderTurnLedger& L) {
    // A tile of the first P, not held, whose address still has a released slot retiring with the
    // tile's own good bytes (the evictor's release, not a drop or an invalidation, and not mapped
    // over since): the old tile comes back -- no read, no new slot -- and the map names it again
    // in this turn. The tile tracked at the address meanwhile (Seen, Loading or Loaded) goes.
    if (m_retiring.empty() || m_need.empty()) return;
    std::vector<std::pair<uint64_t, size_t>> ret;
    for (size_t i = 0; i < m_retiring.size(); ++i) {
        if (m_retiring[i].rescuable && !m_retiring[i].rewanted) {
            ret.push_back({MakeKey(m_retiring[i].tile->tenant, m_retiring[i].tile->req), i});
        }
    }
    if (ret.empty()) return;
    std::sort(ret.begin(), ret.end());
    std::vector<size_t> taken;
    for (OrderEntry& e : m_need) {
        auto it = std::lower_bound(ret.begin(), ret.end(), std::make_pair(e.key, size_t(0)));
        if (it == ret.end() || it->first != e.key) continue;
        Retiring& R = m_retiring[it->second];
        if (R.rewanted) continue;
        R.rewanted = true;
        Tracked* now = e.tile;   // the tile tracked at the address since the release
        if (now->state == TileState::Mapped) continue;   // mapped over: the old bytes are gone
        ++L.rewanted;
        ++rewantedTotal;
        Tenant& t = m_tenants[now->tenant];
        const OrdRec keepRec = m_rec[now->rec];
        {
            const std::shared_ptr<Tracked> hold = t.tracked[now->pos];
            Untrack(t, now);
            now->dropped = true;   // a load in flight for it is let go when it lands
            now->data.clear();
        }
        std::shared_ptr<Tracked> back = R.tile;
        back->dropped = false;
        Track(t, back);
        OrdRec& r = m_rec[back->rec];
        r.weight = keepRec.weight;
        r.stamp = keepRec.stamp;
        r.mask = keepRec.mask;
        r.predUntil = keepRec.predUntil;
        r.pinFrame = keepRec.pinFrame;
        r.bucket = keepRec.bucket;
        for (int k = 0; k < OrdRec::kSlots; ++k) {   // H5: the readers' standing statements too
            r.ssid[k] = keepRec.ssid[k];
            r.sstamp[k] = keepRec.sstamp[k];
            r.sweight[k] = keepRec.sweight[k];
        }
        r.ppass = keepRec.ppass;   // H1: the address's history goes with it
        r.pmeas = keepRec.pmeas;
        r.pmask = keepRec.pmask;
        r.pbucket = keepRec.pbucket;
        r.flags = static_cast<uint8_t>(r.flags | kHeldBit | kInPBit);
        back->firstP = m_passFrame;
        m_mapped.push_back(back);
        NoteChanged(back->tenant, back->req);
        e.tile = back.get();
        taken.push_back(it->second);
        ++L.rescued;
        ++rescuedTotal;
    }
    std::sort(taken.begin(), taken.end());
    for (size_t j = taken.size(); j-- > 0;) {
        m_retiring.erase(m_retiring.begin() + static_cast<std::ptrdiff_t>(taken[j]));
    }
}

void ResidencyManager::OrderTurn(std::vector<std::shared_ptr<Tracked>>& toLoad,
                                 std::vector<std::shared_ptr<Tracked>>& batch,
                                 std::vector<std::shared_ptr<Tracked>>& refills) {
    using Clock = std::chrono::steady_clock;
    auto c0 = Clock::now();
    const auto lapTo = [&](int k) {
        const auto c = Clock::now();
        phaseMs[k] += std::chrono::duration<double, std::milli>(c - c0).count();
        c0 = c;
    };
    std::lock_guard<std::mutex> lk(m_mx);   // the loads write Tracked::state under it
    OrderTurnLedger& L = orderTurn;
    L = OrderTurnLedger{};
    // ---- THE READERS' STATEMENTS: one that spoke this turn and said something else is an event.
    bool spoke = false;
    for (int s = 0; s < kMaxSamplers; ++s) {
        if (m_stFrame[s] != m_frame) continue;
        if (m_stHash[s] != m_stHashLatest[s]) {
            spoke = true;
            L.spoke |= 1u << s;   // H1: who said something else
        }
        m_stHashLatest[s] = m_stHash[s];
    }
    // F12: ...and one that said something last time and is silent while another speaks has said
    // something else too: nothing. The order is made again without its statement.
    if (m_lastSpoke == m_frame) {
        for (int s = 0; s < kMaxSamplers; ++s) {
            if (s == m_predSid || m_stFrame[s] == m_frame || m_stHashLatest[s] == 0ull) continue;
            m_stHashLatest[s] = 0ull;
            spoke = true;
            L.spoke |= 1u << s;
        }
    }
    const bool dirty = spoke || passTurns == 0 || m_trackEpoch != m_passEpoch ||
                       m_failEvents != m_passFails || m_claimEvents != m_passClaims ||
                       m_frame >= m_nextExpiry || settleExact != m_passHold;
    L.pass = dirty ? 1u : 0u;
    if (dirty) OrderPass(L);
    else ++passSkipped;
    L.cut = m_passCut;
    L.candidates = static_cast<uint32_t>(m_rec.size());
    lapTo(2);

    // ---- LAW 4's second half: bytes read for a tile that has left the first P are let go now.
    for (auto it = m_loading.begin(); it != m_loading.end();) {
        Tracked* tr = it->get();
        if (tr->dropped || tr->state == TileState::Failed || tr->state == TileState::Seen) {
            if (tr->dropped && tr->state == TileState::Loaded) {
                ++L.letGo;
                if (!tr->data.empty()) ++L.letGoRead;
            }
            it = m_loading.erase(it);
            continue;
        }
        if (tr->state == TileState::Loaded && tr->firstP != m_passFrame) {
            ++L.letGo;
            if (!tr->data.empty()) ++L.letGoRead;
            m_letGoAt[MakeKey(tr->tenant, tr->req)] = m_frame;
            tr->data.clear();
            tr->data.shrink_to_fit();
            tr->loc = TileLoc{};
            tr->state = TileState::Seen;
            it = m_loading.erase(it);
            continue;
        }
        ++it;
    }
    letGoTotal += L.letGo;
    letGoReadTotal += L.letGoRead;
    lapTo(4);

    // ---- THE VERSION LAW: the replacements of held tiles. A finished load: whole -> a refill (the
    // swap, in MapAndFill); refused, or its version changed again, or its tile no longer held ->
    // let go (the old bytes stand; a refusal waits for the tree's next change). Then the asked ones
    // take the queue first: they are drawn tiles.
    staleHeld = 0;
    for (size_t i = 0; i < m_refresh.size();) {
        Refresh& f = m_refresh[i];
        Tracked* h = f.held.get();
        const bool stillHeld = h->state == TileState::Mapped && h->landed && !h->dropped && h->pos != UINT32_MAX;
        const TileState js = f.job->state;
        if (js == TileState::Loading) {
            ++i;
            continue;
        }
        if (!stillHeld) {   // released by the cut or dropped meanwhile: nothing to swap
            h->stale = false;
            h->refill.reset();
            m_refresh.erase(m_refresh.begin() + static_cast<std::ptrdiff_t>(i));
            continue;
        }
        if (js == TileState::Loaded && !f.again && refills.size() / 2 < kMaxMapsPerFrame / 2) {
            refills.push_back(f.held);
            refills.push_back(f.job);
            h->refill.reset();
            m_refresh.erase(m_refresh.begin() + static_cast<std::ptrdiff_t>(i));
            continue;
        }
        if (js == TileState::Failed && !f.again) {   // refused: the old bytes stand, stale
            ++refreshRefusedTotal;
            h->refill.reset();
            m_refresh.erase(m_refresh.begin() + static_cast<std::ptrdiff_t>(i));
            continue;
        }
        if (js != TileState::Seen) {   // finished, but its version changed again: ask anew
            auto job = std::make_shared<Tracked>();
            job->tenant = h->tenant;
            job->req = h->req;
            job->refresh = true;
            f.job = job;
            h->refill = job;
            f.again = false;
        }
        ++i;   // F13: the job is asked from the loader's list below, at the held tile's measure
    }
    for (const Refresh& f : m_refresh) staleHeld += f.held->stale ? 1u : 0u;

    // ---- THE LOADER: the first tiles of the order not held and with no job (the last pass's
    // list; an untrack is an event, so every tile in it is still tracked). It STOPS when the queue
    // is at capacity (after B3: a turn's cost is bounded by the queue, not by the pool); the
    // settle's exact hold walks on, because its ledger counts every pending tile.
    uint32_t pending = 0;
    m_loaderStop = m_need.size();
    ++loader.turns;
    loader.inFlightAtTurn += static_cast<uint64_t>((std::max)(m_inFlight.load(), 0));
    const auto issuedAt = std::chrono::steady_clock::now();
    for (size_t ni = 0; ni < m_need.size(); ++ni) {
        if (ni >= m_needSorted) SortNeedTo(ni + kNeedChunk);
        const OrderEntry& e = m_need[ni];
        Tracked* tr = e.tile;
        if (tr->state == TileState::Mapped && tr->landed) {
            if (!tr->stale || tr->dropped) continue;
            // F13: a stale held tile: its refresh job takes the queue here, in the order.
            ++pending;
            if (m_inFlight >= static_cast<int>(kMaxLoadsInFlight)) {
                if (!settleExact) {
                    m_loaderStop = ni;
                    pending += static_cast<uint32_t>(m_need.size() - ni - 1);
                    break;
                }
                continue;
            }
            if (tr->refill && tr->refill->state == TileState::Seen) {
                tr->refill->state = TileState::Loading;
                tr->refill->issuedTurn = m_frame;
                tr->refill->issuedAt = issuedAt;
                toLoad.push_back(tr->refill);
                ++m_inFlight;
                ++L.refills;
            }
            continue;
        }
        // A tile under a refused parent not held is unreachable as its parent is (until the tree
        // changes for that parent): never named, and not pending. A tile whose parent is not held
        // yet is pending and not asked: it comes the turn after its parent lands (4.7).
        const Gate gate = ParentGate(m_tenants[tr->tenant], tr);
        if (gate == Gate::Refused) continue;
        // F14: a tile the tree answers as its parent, magnified, is known here by arithmetic and
        // takes no slot of the queue: Failed + magnified now, the state a load's answer gave it
        // (its children walk past it in ParentGate as they did; nothing is mapped, by law).
        if (tr->state == TileState::Seen && !tr->magKnown) {
            tr->magKnown = true;
            const Tenant& tq = m_tenants[tr->tenant];
            if (tq.magnifiedOf && tq.magnifiedOf(tr->req)) {
                tr->state = TileState::Failed;
                tr->magnified = true;
                ++m_failEvents;
                m_failedKeys.push_back({tr->tenant, tr->req});
                ++L.magnifiedKnown;
                ++m_tenants[tr->tenant].magnifiedKnown;
                continue;
            }
        }
        ++pending;
        if (m_inFlight >= static_cast<int>(kMaxLoadsInFlight)) {
            if (!settleExact) {
                m_loaderStop = ni;
                pending += static_cast<uint32_t>(m_need.size() - ni - 1);   // the rest, uncounted
                break;
            }
            continue;
        }
        if (tr->state != TileState::Seen || tr->pos == UINT32_MAX) continue;
        if (gate == Gate::Wait) continue;   // its parent first
        if (starvePlant != 0 && tr->req.face == starvePlant) continue;   // the watchdog's plant
        if (!m_letGoAt.empty()) {   // the livelock's instrument: read and let go within the glance
            const auto it = m_letGoAt.find(e.key);
            if (it != m_letGoAt.end() && m_frame - it->second <= kGlanceTurns) {
                ++L.reloaded;
                ++reloadedTotal;
            }
        }
        // 4.7: ONE TILE, MANY WINDOWS. A slot whose address is held takes no read: it is a mapping
        // (alias), gathered as a loaded tile is. One whose address has a read on the way waits for
        // it. One whose address has a record with no reader behind it any more (failed, let go,
        // dropped before it mapped) takes the read itself.
        {
            Tenant& tn = m_tenants[tr->tenant];
            if (!tr->gkey) tr->gkey = KeyOf(tn, tr->req);
            if (tr->gkey) {
                auto hit = tn.held.find(tr->gkey);
                if (hit != tn.held.end()) {
                    if (hit->second.pool != UINT32_MAX) {
                        tr->alias = true;
                        tr->state = TileState::Loaded;   // the gather maps it this turn
                        m_loading.push_back(tn.tracked[tr->pos]);   // in the queue, as a read is: let go if it leaves the first P
                        ++L.aliases;
                        continue;
                    }
                    const Tracked* rd = Find(tr->tenant, hit->second.reader);
                    if (rd && rd != tr && rd->gkey == tr->gkey &&
                        (rd->state == TileState::Loading || rd->state == TileState::Loaded)) {
                        continue;   // its bytes are on the way for another slot
                    }
                    tn.held.erase(hit);
                }
                Tenant::Held& h = tn.held[tr->gkey];   // the read is this slot's
                h.reader = tr->req;
            }
            tr->alias = false;   // a read, whatever this slot was before
        }
        tr->state = TileState::Loading;
        tr->issuedTurn = m_frame;
        tr->issuedAt = issuedAt;
        const std::shared_ptr<Tracked> sp = m_tenants[tr->tenant].tracked[tr->pos];
        m_loading.push_back(sp);
        toLoad.push_back(sp);
        ++m_inFlight;
        ++L.issued;
    }
    m_orderPending = pending;
    L.pending = pending;
    L.atCap = m_loaderStop < m_need.size();
    passLoaderStop += (std::min)(m_loaderStop, m_need.size());
    ++passLoaderTurns;
    loader.turnsAtCap += L.atCap ? 1u : 0u;
    loader.pendingAtTurn += pending;
    loader.reads += L.issued;
    loader.refills += L.refills;
    loader.aliases += L.aliases;
    loader.magnifiedKnown += L.magnifiedKnown;
    StarveWatch();
    if ((m_frame & 63u) == 0u) {
        std::erase_if(m_letGoAt, [&](const auto& kv) { return m_frame - kv.second > kGlanceTurns; });
    }
    lapTo(3);

    // ---- THE GATHER: loaded tiles of the first P, in the order, as many as there are slots (in
    // the exact hold the pool grows to the want set).
    const size_t made = m_heaps.size() * kPoolChunkTiles;
    const size_t slots = settleExact ? SIZE_MAX
                                     : m_freePool.size() + (made < kPoolCapTiles ? kPoolCapTiles - made : 0u);
    const size_t maxBatch = (std::min)(static_cast<size_t>(kMaxMapsPerFrame) - refills.size() / 2, slots);
    // The loaded tiles of the first P, in the order: the few that are loaded are picked out of the
    // list and only they are sorted (the list itself is sorted only as far as the loader read it).
    std::vector<const OrderEntry*> ready;
    for (const OrderEntry& e : m_need) {
        if (e.tile->state == TileState::Loaded && e.tile->pos != UINT32_MAX) ready.push_back(&e);
    }
    {
        const uint16_t units = m_needUnits;
        const size_t keep = (std::min)(ready.size(), maxBatch);
        const auto before = [units](const OrderEntry* a, const OrderEntry* b) { return NeedBefore(*a, *b, units); };
        std::partial_sort(ready.begin(), ready.begin() + static_cast<std::ptrdiff_t>(keep), ready.end(), before);
        ready.resize(keep);
    }
    for (const OrderEntry* pe : ready) {
        if (batch.size() >= maxBatch) break;
        const OrderEntry& e = *pe;
        Tracked* tr = e.tile;
        if (tr->state != TileState::Loaded || tr->pos == UINT32_MAX) continue;
        batch.push_back(m_tenants[tr->tenant].tracked[tr->pos]);
        if (!tr->alias) {   // F14: from the turn it was issued to the turn its bytes are mapped
            ++loader.gathered;
            loader.gatherTurns += m_frame - tr->issuedTurn;
        }
    }
    if (!batch.empty()) {
        std::vector<const Tracked*> taken;
        for (const auto& b : batch) taken.push_back(b.get());
        std::sort(taken.begin(), taken.end());
        std::erase_if(m_loading, [&](const std::shared_ptr<Tracked>& p) {
            return std::binary_search(taken.begin(), taken.end(), p.get());
        });
    }
    L.waiting = 0;
    for (const auto& p : m_loading) L.waiting += p->state == TileState::Loaded ? 1u : 0u;
    if ((m_frame % 30u) == 0u && (m_tailTotal || m_cutRung != kNoBucket)) {
        std::string ten, rung;
        for (uint32_t k = 0; k < m_tenants.size() && k < 16; ++k) {
            if (!m_tailTenant[k]) continue;
            char b[160];
            snprintf(b, sizeof(b), " %S %u;", m_tenants[k].name.c_str(), m_tailTenant[k]);
            ten += b;
        }
        // F9: a record's bucket is class * kRungs + MeasureBucket (the measure's power of two,
        // texel over distance), so its low part is a MEASURE bucket; it was printed as a rung.
        for (uint32_t r = 0; r < kRungs; ++r) {
            if (!m_tailRung[r]) continue;
            char b[48];
            snprintf(b, sizeof(b), " 2^-%u %u;", r, m_tailRung[r]);
            rung += b;
        }
        Log("[order-tail] rec%u f%u | the want's tail past the cut: %u of %u wanted | by tenant:%s | "
            "by measure (texel over distance):%s",
            traceRecFrame, m_frame, m_tailTotal, m_wantTotal, ten.c_str(), rung.c_str());
        if (m_cutRung != kNoBucket) {
            const auto name = [&](uint64_t key) {
                const uint32_t k = uint32_t(key >> 56) & 0xFFu;
                char b[200];
                snprintf(b, sizeof(b), "%S [%u] m%u (%u,%u)",
                         k < m_tenants.size() ? m_tenants[k].name.c_str() : L"?",
                         uint32_t(key >> 48) & 0xFFu, uint32_t(key >> 42) & 0x3Fu,
                         uint32_t(key) & 0x1FFFFFu, uint32_t(key >> 21) & 0x1FFFFFu);
                return std::string(b);
            };
            const uint32_t cls = m_cutRung / kRungs;
            std::string byRung;
            for (uint32_t r = 0; r < kRungs; ++r) {
                if (m_cutKeptM[r] < 0.0f && m_cutLostM[r] < 0.0f) continue;
                char b[96];
                snprintf(b, sizeof(b), " 2^%d m: kept to %.0f m, lost from %.0f m;",
                         static_cast<int>(kRungTop) - static_cast<int>(r), m_cutKeptM[r], m_cutLostM[r]);
                byRung += b;
            }
            char unit[160] = "";
            if (m_cutUnitPlanes) {   // F18
                snprintf(unit, sizeof(unit), " | the pivot's tile let go whole: %u kept planes at measure %.3g",
                         m_cutUnitPlanes, BitsWeight(m_cutUnitMeas));
            }
            const bool noneKept = m_keptFar == UINT32_MAX;
            Log("[order-cut] rec%u f%u | the straddling bucket: class %s, lateness %u, measure 2^-%u | "
                "smallest measure kept %s (%s) | largest lost %.3g (%s) | %s%s | in metres from the eye, "
                "by rung:%s",
                traceRecFrame, m_frame, cls < 3u ? "pin" : "want", cls % 3u, m_cutRung % kRungs,
                noneKept ? "none" : std::to_string(BitsWeight(m_keptFar)).c_str(),
                noneKept ? "the bucket keeps nothing" : name(m_keptFarTile).c_str(), BitsWeight(m_lostNear),
                name(m_lostNearTile).c_str(),
                m_lostNear == 0 || noneKept || m_keptFar >= m_lostNear ? "they do not cross" : "THEY CROSS",
                unit, byRung.c_str());
        }
    }
    lapTo(5);

    // ---- THE EXACT HOLD (settleExact): the settle's ledger in the order's terms.
    settleTurn = SettleTurn{};
    if (settleExact) {
        for (size_t k = 0; k < m_tenants.size(); ++k) {
            Tenant& t = m_tenants[k];
            uint32_t wanted = 0, mapped = 0, deficit = 0, stale = 0, lost = 0, unreachable = 0;
            uint32_t magnified = 0;
            for (const auto& sp : t.tracked) {
                const Tracked* tr = sp.get();
                const size_t idx = StampIndex(t, tr->req.face, tr->req.mip, tr->req.x, tr->req.y);
                const bool w = (t.stamp[idx] >> 1) == m_frame;
                const bool held = tr->state == TileState::Mapped && tr->landed;
                if (w) {
                    ++wanted;
                    if (tr->state == TileState::Mapped) ++mapped;
                    if (tr->state == TileState::Failed) {
                        ++(tr->magnified ? magnified : unreachable);
                        continue;
                    }
                    if (!held && ParentGate(t, tr) == Gate::Refused) {   // under a refused parent
                        ++unreachable;
                        continue;
                    }
                    if (tr->firstP != m_passFrame && !held) ++lost;
                    else if (!held) ++deficit;
                } else if (tr->state == TileState::Mapped && tr->req.mip + 1 != t.mips) {
                    ++stale;
                }
            }
            t.exWanted = wanted;
            t.exMapped = mapped;
            t.exDeficit = deficit;
            t.exStale = stale;
            t.exLost = lost;
            t.exUnreachable = unreachable;
            t.exMagnified = magnified;
            settleTurn.wanted += wanted;
            settleTurn.mapped += mapped;
            settleTurn.deficit += deficit;
            settleTurn.unreachable += unreachable;
            settleTurn.magnified += magnified;
            settleTurn.stale += stale;
            L.lost += lost;
        }
        settleTurn.dropped = turn.evicted;
        settleTurn.pending = m_orderPending;
        settleTurn.reads = static_cast<uint32_t>(m_inFlightReads.size());
        settleTurn.retiring = static_cast<uint32_t>(m_retiring.size());
        settleTurn.exact = settleTurn.deficit == 0 && settleTurn.stale == 0 &&
                           settleTurn.pending == 0 && settleTurn.reads == 0 &&
                           settleTurn.retiring == 0;
    }
}

// THE WATCHDOG (Residency.h): starved = in the first P, not held, no load in flight, no retiring
// slot; in flight = Loading, Loaded (waiting for the gather) or Mapped with its bytes not landed.
void ResidencyManager::StarveWatch() {
    starvedNow = starvedPast = stuckPast = 0;
    if (m_need.empty()) return;
    // THE PRINT'S OWN COST: whole lines are capped a turn (kStarveLinesPerTurn, the first in the
    // order); every tile due a line is counted by its reason in one summary line instead.
    uint32_t lines = 0, due = 0;
    std::vector<std::pair<std::string, uint32_t>> byWhy;
    std::vector<uint64_t> ret;
    for (const Retiring& R : m_retiring) ret.push_back(MakeKey(R.tile->tenant, R.tile->req));
    std::sort(ret.begin(), ret.end());
    // Only the entries the loader passed over this turn (to where the queue stopped it): the rest
    // wait behind the queue, which is latency, not a stall.
    const size_t walk = (std::min)(m_loaderStop, m_need.size());
    for (size_t ni = 0; ni < walk; ++ni) {
        const OrderEntry& e = m_need[ni];
        Tracked* tr = e.tile;
        const bool held = tr->state == TileState::Mapped && tr->landed;
        const bool flight = tr->state == TileState::Loading || tr->state == TileState::Loaded ||
                            (tr->state == TileState::Mapped && !tr->landed);
        const bool retiring = std::binary_search(ret.begin(), ret.end(), e.key);
        uint8_t cls = held || retiring ? 0 : flight ? 2 : 1;
        if (cls == 1) {
            // Under a refused parent: unreachable, not starved. Waiting on a TRACKED parent: latency
            // behind it (the parent is watched itself). Waiting on an untracked one stays starved.
            const Tracked* at = nullptr;
            const Gate g = ParentGate(m_tenants[tr->tenant], tr, &at);
            if (g == Gate::Refused || (g == Gate::Wait && at)) cls = 0;
        }
        if (cls == 0) {
            tr->starveClass = 0;
            continue;
        }
        if (tr->starveClass != cls || tr->starveSeen + 1u != m_frame) {
            tr->starveSince = m_frame;
            tr->starvePrinted = 0;
        }
        tr->starveClass = cls;
        tr->starveSeen = m_frame;
        if (cls == 1) ++starvedNow;
        const uint32_t age = m_frame - tr->starveSince;
        const uint32_t past = cls == 1 ? kGlanceTurns : kStarvePrintTurns;
        if (age <= past) continue;
        ++(cls == 1 ? starvedPast : stuckPast);
        if (tr->starvePrinted != 0 && m_frame - tr->starvePrinted < kStarvePrintTurns) continue;
        tr->starvePrinted = m_frame;
        ++starvedPrinted;
        ++due;
        // Its whole state: the reader's statement, its slot, and why no load was started.
        const Tenant& t = m_tenants[tr->tenant];
        std::string readers = "none", why;
        float dist = 0.0f, meas = 0.0f;
        uint32_t stamp = 0, bucket = kNoBucket;
        if (tr->rec != UINT32_MAX) {
            const OrdRec& r = m_rec[tr->rec];
            readers.clear();
            for (int i = 0; i < kMaxSamplers; ++i) {
                if (!((r.mask >> i) & 1u)) continue;
                if (!readers.empty()) readers += "+";
                readers += SamplerName(i);
            }
            if (readers.empty()) readers = "none";
            dist = BitsWeight(r.weight);
            memcpy(&meas, &r.meas, sizeof(meas));
            stamp = r.stamp;
            bucket = r.bucket;
        }
        const char* st = tr->state == TileState::Seen ? "Seen" : tr->state == TileState::Loading ? "Loading"
                         : tr->state == TileState::Loaded ? "Loaded" : tr->state == TileState::Mapped ? "Mapped"
                         : tr->state == TileState::Failed ? "Failed" : "?";
        if (cls == 2) {
            why = tr->state == TileState::Mapped ? "mapped, its bytes never landed (the copy's fence)"
                  : tr->state == TileState::Loaded ? "loaded, never gathered (no slot given to it)"
                                                   : "its load never returned";
        } else if (tr->state == TileState::Failed) {
            why = tr->magnified ? "magnified (the tree answered it as its parent; never mapped, by law)"
                                : "failed (" + std::to_string(tr->retries) + " retries)";
        } else if (tr->pos == UINT32_MAX) {
            why = "untracked";
        } else if (m_inFlight >= static_cast<int>(kMaxLoadsInFlight)) {
            why = "the queue full (" + std::to_string(m_inFlight.load()) + " in flight of " +
                  std::to_string(kMaxLoadsInFlight) + ")";
        } else if (ParentGate(t, tr) == Gate::Wait) {
            why = "its parent is not tracked (never asked)";
        } else if (starvePlant != 0 && tr->req.face == starvePlant) {
            why = "PLANTED: --starve-plant skips its slice";
        } else {
            why = "NOTHING: the loader passed it with room in the queue";
        }
        {
            // The reason's kind (a failed parent's name stripped), for the summary.
            const std::string kind = why.substr(0, why.find(" earth") == std::string::npos ? why.find(" (")
                                                                                            : why.find(" earth"));
            bool found = false;
            for (auto& [k, n] : byWhy) {
                if (k == kind) { ++n; found = true; break; }
            }
            if (!found) byWhy.push_back({kind, 1u});
        }
        if (lines >= kStarveLinesPerTurn) continue;
        ++lines;
        const std::string global = t.namer ? t.namer(tr->req) : std::string();
        const std::string slot = tr->pool == UINT32_MAX ? std::string("none") : std::to_string(tr->pool);
        Log("[%s] f%u | %s%s%s | %s for %u turns (since f%u) | state %s, slot %s | readers %s, last "
            "stamp f%u, distance %.0f m, measure %.4g, bucket %s %u 2^-%u | why no load: %s | in flight "
            "%d, retiring %zu, first P not held %zu",
            cls == 1 ? "starved" : "stuck", m_frame, TileName(e.key).c_str(), global.empty() ? "" : " = ",
            global.c_str(), cls == 1 ? "starved" : "in flight", age, tr->starveSince, st, slot.c_str(),
            readers.c_str(), stamp, dist, meas, bucket == kNoBucket ? "-" : (bucket / kRungs < 3u ? "pin" : "want"),
            bucket == kNoBucket ? 0u : (bucket / kRungs) % 3u, bucket == kNoBucket ? 0u : bucket % kRungs,
            why.c_str(), m_inFlight.load(), m_retiring.size(), m_need.size());
    }
    if (due > lines) {
        std::string w;
        for (const auto& [kd, n] : byWhy) w += " " + std::to_string(n) + " " + kd + ";";
        Log("[starved] f%u | %u more past the glance this turn (whole lines capped at %u) -- by why:%s | "
            "starved now %u, past the glance %u, stuck %u",
            m_frame, due - lines, kStarveLinesPerTurn, w.c_str(), starvedNow, starvedPast, stuckPast);
    }
}

void ResidencyManager::FlushNullMaps(
    Gpu& gpu, std::vector<std::pair<int, D3D12_TILED_RESOURCE_COORDINATE>>& nulls) {
    // Law 7's second step, batched: one UpdateTileMappings a tenant, as the map side's are.
    std::stable_sort(nulls.begin(), nulls.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
    std::vector<D3D12_TILED_RESOURCE_COORDINATE> coords;
    for (size_t i = 0; i < nulls.size();) {
        const int k = nulls[i].first;
        coords.clear();
        for (; i < nulls.size() && nulls[i].first == k; ++i) coords.push_back(nulls[i].second);
        const UINT n = static_cast<UINT>(coords.size());
        const std::vector<D3D12_TILE_REGION_SIZE> sizes(n, D3D12_TILE_REGION_SIZE{1, FALSE, 0, 0, 0});
        const std::vector<D3D12_TILE_RANGE_FLAGS> flags(n, D3D12_TILE_RANGE_FLAG_NULL);
        const std::vector<UINT> offs(n, 0u), counts(n, 1u);
        gpu.Queue()->UpdateTileMappings(m_tenants[k].res.Get(), n, coords.data(), sizes.data(),
                                        nullptr, n, flags.data(), offs.data(), counts.data(),
                                        D3D12_TILE_MAPPING_FLAG_NONE);
    }
    nulls.clear();
}

// ---- LAW 8's TABLE (decision 6): where, in one frame, each reader's want and each read stands.
void ResidencyManager::FrameBegin() {
    ++m_markFrame;
    if (m_markFrame == kMarkFrame + 1 && !m_marks.empty()) {
        Log("[frame-table] frame %u of the run, in the order of first appearance within it (law 8: "
            "each reader's wants and reads, and the residency turn):",
            kMarkFrame);
        for (size_t i = 0; i < m_marks.size(); ++i) {
            Log("[frame-table]   %2zu  %s", i + 1, m_marks[i].c_str());
        }
    }
}

void ResidencyManager::Mark(const char* what) {
    if (m_markFrame != kMarkFrame || !what) return;
    for (const auto& m : m_marks) {
        if (m == what) return;
    }
    m_marks.push_back(what);
}

std::string ResidencyManager::TileName(uint64_t key) const {
    const uint32_t k = uint32_t(key >> 56) & 0xFFu;
    char b[200];
    snprintf(b, sizeof(b), "%S [%u] m%u (%u,%u)", k < m_tenants.size() ? m_tenants[k].name.c_str() : L"?",
             uint32_t(key >> 48) & 0xFFu, uint32_t(key >> 42) & 0x3Fu, uint32_t(key) & 0x1FFFFFu,
             uint32_t(key >> 21) & 0x1FFFFFu);
    return b;
}

// H1: the measure's motion over the run, and the cut's crossings (OrderPass keeps the ledger).
void ResidencyManager::LogLoader() const {
    const LoaderLedger& l = loader;
    const double turns = double((std::max)(l.turns, uint64_t(1)));
    Log("[loader] %llu turns, stopped at the cap (%u in flight) on %llu (%.0f%%); in flight as a turn began "
        "%.1f on average, pending (first P, not held) %.0f | issued %llu reads + %llu refills, %llu slots mapped to bytes already held, %llu "
        "known magnified without a load | a read's way from its issue to the batch that mapped it: %.2f "
        "turns on average over %llu | the cut fell inside a field tile's planes on %llu passes and let "
        "%llu kept planes go with it (F18)",
        static_cast<unsigned long long>(l.turns), kMaxLoadsInFlight,
        static_cast<unsigned long long>(l.turnsAtCap), 100.0 * double(l.turnsAtCap) / turns,
        double(l.inFlightAtTurn) / turns, double(l.pendingAtTurn) / turns,
        static_cast<unsigned long long>(l.reads),
        static_cast<unsigned long long>(l.refills), static_cast<unsigned long long>(l.aliases),
        static_cast<unsigned long long>(l.magnifiedKnown),
        double(l.gatherTurns) / double((std::max)(l.gathered, uint64_t(1))),
        static_cast<unsigned long long>(l.gathered), static_cast<unsigned long long>(l.unitCuts),
        static_cast<unsigned long long>(l.unitPlanes));
    for (const Tenant& t : m_tenants) {
        const uint64_t n = uint64_t(t.loadsRead) + t.loadsPaint;
        if (!n && !t.magnifiedKnown) continue;
        Log("[loader]   %S: %llu loads -- %u read (%.2f ms each), %u painted (%.1f ms each); of them %u "
            "answered magnified, %u failed or not whole | queue wait %.2f ms a load | %u known magnified, no load",
            t.name.c_str(), static_cast<unsigned long long>(n), t.loadsRead,
            t.loadsRead ? double(t.readUs) / (1000.0 * t.loadsRead) : 0.0, t.loadsPaint,
            t.loadsPaint ? double(t.paintUs) / (1000.0 * t.loadsPaint) : 0.0, t.loadsMagnified,
            t.loadsFailed, n ? double(t.queueUs) / (1000.0 * double(n)) : 0.0, t.magnifiedKnown);
    }
}

void ResidencyManager::LogOrderMotion() const {
    if (!passTurns) return;
    const OrderMotion& M = m_motion;
    const auto u = [](uint64_t v) { return static_cast<unsigned long long>(v); };
    Log("[order-motion] the closure (H2), walked from the held tiles whose parent is not held: %llu of "
        "them at most; checked against the law walked from every held tile at %llu audited passes: "
        "%llu differ (the largest by %llu records)",
        u(m_orphansMax), u(m_closureChecks), u(m_closureMissTurns), u(m_closureMissMax));
    if (!M.pairs && !M.passesCrossed) {
        Log("[order-motion] (the instrument rides traced runs: this run traced no pass)");
        return;
    }
    Log("[order-motion] the measure between two passes running, tile by tile: %llu pairs over %llu "
        "passes; moved by more than x1.01: %llu, x1.1: %llu, x1.41: %llu, x2: %llu; the largest x%.4f "
        "(%s at f%u: %.5g -> %.5g)",
        u(M.pairs), u(passTurns), u(M.left[0]), u(M.left[1]), u(M.left[2]), u(M.left[3]), M.qmax,
        TileName(M.qmaxKey).c_str(), M.qmaxFrame, M.qmaxPrev, M.qmaxNow);
    Log("[order-motion] the cut: %llu tiles crossed in and %llu out, on %llu passes, the last at f%u | "
        "the crossers by their own move: unmoved %llu, to x1.01 %llu, to x1.1 %llu, to x1.41 %llu, "
        "to x2 %llu, more %llu | held tiles crossing out, the smallest kept over theirs: to x1.01 %llu, "
        "to x1.1 %llu, to x1.41 %llu, to x2 %llu, more or past the straddling bucket %llu",
        u(M.crossIn), u(M.crossOut), u(M.passesCrossed), M.lastCrossFrame, u(M.crossOwn[0]),
        u(M.crossOwn[1]), u(M.crossOwn[2]), u(M.crossOwn[3]), u(M.crossOwn[4]), u(M.crossOwn[5]),
        u(M.heldOutGap[0]), u(M.heldOutGap[1]), u(M.heldOutGap[2]), u(M.heldOutGap[3]),
        u(M.heldOutGap[4]));
}


}  // namespace ga
