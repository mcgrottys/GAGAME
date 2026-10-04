// ResidencyAudit - the residency bytes against the mapped set (ResidencyAudit.h). Three parts:
// the pure audit, the manager's side of it (the gather at the end of a turn, the report, the count
// of applied invalidations) and the selftest that replays finding 3 through the real byte rule.
#include "hal/ResidencyAudit.h"

#include "core/Common.h"
#include "hal/Residency.h"

#include <algorithm>
#include <chrono>
#include <cstdio>

namespace ga {

void AuditCount::Add(const AuditCount& o) {
    cells += o.cells;
    finer += o.finer;
    coarser += o.coarser;
    holes += o.holes;
    holesInFlight += o.holesInFlight;
    split += o.split;
    worstFiner = (std::max)(worstFiner, o.worstFiner);
    worstCoarser = (std::max)(worstCoarser, o.worstCoarser);
}

namespace {

// A virtual tile's record in the audit's own planes: 0 = nothing tracked at the address, else 1 +
// its state in the low bits, with kLanded set when it is Mapped and its bytes are on the GPU.
constexpr uint8_t kLanded = 0x80u;
constexpr uint8_t kMappedCode = 1u + static_cast<uint8_t>(AuditState::Mapped);
bool Resident(uint8_t v) { return (v & 0x7Fu) == kMappedCode && (v & kLanded) != 0; }

std::string MipText(uint32_t m, uint32_t mips) {
    return m >= mips ? std::string("nothing") : "m" + std::to_string(m);
}

// A tile's address and what the manager holds there, as a clause: "m2 (1,1)" + "is mapped, its
// bytes still in flight". `joint` goes between the two ("", or ", which" inside a sentence).
std::string TileText(const AuditTileRef& r, const char* joint = "") {
    const char* what = r.state < 0 ? "is not tracked"
                       : r.state == int(AuditState::Mapped)
                           ? (r.landed ? "is mapped and landed" : "is mapped, its bytes still in flight")
                       : r.state == int(AuditState::Seen)    ? "is tracked, Seen (not loaded)"
                       : r.state == int(AuditState::Loading) ? "is tracked, Loading"
                       : r.state == int(AuditState::Loaded)  ? "is tracked, Loaded (not mapped)"
                                                             : "is tracked, Failed (it never maps)";
    char b[160];
    snprintf(b, sizeof(b), "m%u (%u,%u)%s %s", r.mip, r.x, r.y, joint, what);
    return b;
}

}  // namespace

AuditResult AuditResidency(const std::vector<AuditTenant>& tenants, size_t keep) {
    AuditResult res;
    res.tenants.resize(tenants.size());
    auto offend = [&](const AuditOffender& o) {
        ++res.offendersTotal;
        if (res.offenders.size() < keep) res.offenders.push_back(o);
    };
    for (size_t k = 0; k < tenants.size(); ++k) {
        const AuditTenant& t = tenants[k];
        AuditTenantResult& out = res.tenants[k];
        out.slices.assign(t.faces, AuditCount{});
        const size_t planes = size_t(t.faces) * t.mips;
        if (!planes || t.tilesW.size() < planes || t.tilesH.size() < planes || !t.bytes ||
            t.bytes->size() < t.faces || !t.mapDim) {
            continue;
        }
        // The planes, laid out as the manager lays out its stamp array: one flat array, a
        // row-major block per (face, mip).
        std::vector<size_t> base(planes);
        size_t total = 0;
        for (size_t p = 0; p < planes; ++p) {
            base[p] = total;
            total += size_t(t.tilesW[p]) * t.tilesH[p];
        }
        std::vector<uint8_t> st(total, 0);
        for (const AuditTile& a : t.tiles) {
            if (a.face >= t.faces || a.mip >= t.mips) continue;
            const size_t p = size_t(a.face) * t.mips + a.mip;
            if (a.x >= t.tilesW[p] || a.y >= t.tilesH[p]) continue;
            uint8_t v = static_cast<uint8_t>(1u + static_cast<uint8_t>(a.state));
            if (a.state == AuditState::Mapped) {
                ++out.mapped;
                if (a.landed) {
                    v |= kLanded;
                    ++out.landed;
                }
            } else if (a.state != AuditState::Failed && !a.queued) {
                // Tracked, not mapped, not given up on, and in no queue: nothing will load or map
                // it again, and the ring gate holds every child of it behind it for good.
                ++out.orphans;
                if (a.state == AuditState::Loaded) ++out.orphansLoaded;
                AuditOffender o;
                o.kind = AuditKind::Orphan;
                o.tenant = static_cast<uint32_t>(k);
                o.face = a.face;
                o.cellX = a.x;
                o.cellY = a.y;
                o.says = AuditTileRef{a.mip, a.x, a.y, int(a.state), false};
                offend(o);
            }
            st[base[p] + size_t(a.y) * t.tilesW[p] + a.x] = v;
        }
        for (uint32_t f = 0; f < t.faces; ++f) {
            const uint32_t top = f < t.top.size() ? t.top[f] : t.mips - 1;   // F1: the slice's floor
            const size_t p0 = size_t(f) * t.mips;
            const uint32_t bw = t.tilesW[p0], bh = t.tilesH[p0];
            // A tile's footprint in mip-0 cells, per mip: UpdateResidencyByte's own arithmetic.
            std::vector<uint32_t> cw(t.mips), ch(t.mips);
            for (uint32_t m = 0; m < t.mips; ++m) {
                cw[m] = (std::max)(1u, bw / (std::max)(1u, t.tilesW[p0 + m]));
                ch[m] = (std::max)(1u, bh / (std::max)(1u, t.tilesH[p0 + m]));
            }
            auto code = [&](uint32_t m, uint32_t x, uint32_t y) -> uint8_t {
                const size_t p = p0 + m;
                x = (std::min)(x, t.tilesW[p] - 1);
                y = (std::min)(y, t.tilesH[p] - 1);
                return st[base[p] + size_t(y) * t.tilesW[p] + x];
            };
            auto ref = [&](uint32_t m, uint32_t x, uint32_t y) {
                const uint8_t v = code(m, x, y);
                AuditTileRef r;
                r.mip = m;
                r.x = x;
                r.y = y;
                r.state = (v & 0x7Fu) ? int((v & 0x7Fu) - 1u) : -1;
                r.landed = (v & kLanded) != 0;
                return r;
            };
            // THE CHAIN, per tile, from the coarsest down: the tile and every tile above it
            // resident. A resident tile whose parent's chain is broken is a hole tile, counted once.
            std::vector<std::vector<uint8_t>> chain(t.mips);
            for (int m = int(top); m >= 0; --m) {
                const size_t p = p0 + uint32_t(m);
                const uint32_t W = t.tilesW[p], H = t.tilesH[p];
                chain[m].assign(size_t(W) * H, 0);
                for (uint32_t y = 0; y < H; ++y) {
                    for (uint32_t x = 0; x < W; ++x) {
                        const bool here = Resident(st[base[p] + size_t(y) * W + x]);
                        bool above = true;
                        if (uint32_t(m) < top) {
                            const uint32_t ax = (x * cw[m]) / cw[m + 1], ay = (y * ch[m]) / ch[m + 1];
                            above = chain[m + 1][size_t(ay) * t.tilesW[p + 1] + ax] != 0;
                            if (here && !above) ++out.holeTiles;
                        }
                        chain[m][size_t(y) * W + x] = (here && above) ? 1u : 0u;
                    }
                }
            }
            const std::vector<uint8_t>& bytes = (*t.bytes)[f];
            if (bytes.size() < size_t(t.mapDim) * t.mapDim) continue;
            const uint32_t sx = (std::max)(1u, t.mapDim / (std::max)(1u, bw));
            const uint32_t sy = (std::max)(1u, t.mapDim / (std::max)(1u, bh));
            AuditCount& c = out.slices[f];
            for (uint32_t by = 0; by < bh; ++by) {
                for (uint32_t bx = 0; bx < bw; ++bx) {
                    ++c.cells;
                    // The truth: down the chain from the coarsest while it holds.
                    uint32_t truth = t.mips;   // nothing resident over the cell
                    int gap = -1;
                    for (int m = int(top); m >= 0; --m) {
                        if (chain[m][size_t(by / ch[m]) * t.tilesW[p0 + m] + bx / cw[m]]) {
                            truth = uint32_t(m);
                            continue;
                        }
                        gap = m;
                        break;
                    }
                    const size_t row0 = size_t(by) * sy * t.mapDim + size_t(bx) * sx;
                    const uint8_t b = bytes[row0];
                    // A resident tile under the gap: the coarsest one names the hole.
                    for (int m = gap - 1; m >= 0; --m) {
                        const uint32_t ax = bx / cw[m], ay = by / ch[m];
                        if (!Resident(code(uint32_t(m), ax, ay))) continue;
                        AuditOffender o;
                        o.kind = AuditKind::Hole;
                        o.tenant = static_cast<uint32_t>(k);
                        o.face = f;
                        o.cellX = bx;
                        o.cellY = by;
                        o.byte = b;
                        o.truth = truth;
                        o.says = ref(uint32_t(m), ax, ay);
                        o.gap = ref(uint32_t(gap), bx / cw[gap], by / ch[gap]);
                        ++c.holes;
                        if (o.gap.state == int(AuditState::Mapped)) ++c.holesInFlight;
                        offend(o);
                        break;
                    }
                    // The byte: every map texel of the cell says the same, or the cell is split.
                    bool split = false;
                    for (uint32_t j = 0; j < sy && !split; ++j) {
                        for (uint32_t i = 0; i < sx; ++i) {
                            if (bytes[row0 + size_t(j) * t.mapDim + i] != b) {
                                split = true;
                                break;
                            }
                        }
                    }
                    if (split) {
                        AuditOffender o;
                        o.kind = AuditKind::Split;
                        o.tenant = static_cast<uint32_t>(k);
                        o.face = f;
                        o.cellX = bx;
                        o.cellY = by;
                        o.byte = b;
                        o.truth = truth;
                        ++c.split;
                        offend(o);
                    }
                    const uint32_t said = (std::min)(uint32_t(b) / 16u, t.mips);
                    if (said == truth) continue;
                    AuditOffender o;
                    o.tenant = static_cast<uint32_t>(k);
                    o.face = f;
                    o.cellX = bx;
                    o.cellY = by;
                    o.byte = b;
                    o.truth = truth;
                    if (said < truth) {
                        // The chain broke at `gap` (truth > said >= 0 puts it at a real mip): the
                        // first tile a sample at the byte's mip would read and not find.
                        o.kind = AuditKind::Finer;
                        o.says = ref(uint32_t(gap), bx / cw[gap], by / ch[gap]);
                        ++c.finer;
                        c.worstFiner = (std::max)(c.worstFiner, truth - said);
                    } else {
                        o.kind = AuditKind::Coarser;
                        o.says = ref(truth, bx / cw[truth], by / ch[truth]);
                        ++c.coarser;
                        c.worstCoarser = (std::max)(c.worstCoarser, said - truth);
                    }
                    offend(o);
                }
            }
            out.sum.Add(c);
        }
        res.sum.Add(out.sum);
        res.mapped += out.mapped;
        res.landed += out.landed;
        res.holeTiles += out.holeTiles;
        res.orphans += out.orphans;
        res.orphansLoaded += out.orphansLoaded;
    }
    return res;
}

std::string AuditOffenderText(const AuditOffender& o, const std::vector<AuditTenant>& tenants) {
    const AuditTenant* t = o.tenant < tenants.size() ? &tenants[o.tenant] : nullptr;
    const char* name = t ? t->name.c_str() : "?";
    const uint32_t mips = t ? t->mips : 0u;
    const std::string said = MipText(o.byte / 16u, mips), truth = MipText(o.truth, mips);
    char b[512];
    switch (o.kind) {
        case AuditKind::Finer:
            snprintf(b, sizeof(b),
                     "FINER   %s [%u] cell (%u,%u): byte %u says %s, the tiles hold %s -- %s",
                     name, o.face, o.cellX, o.cellY, o.byte, said.c_str(), truth.c_str(),
                     TileText(o.says).c_str());
            break;
        case AuditKind::Coarser:
            snprintf(b, sizeof(b),
                     "COARSER %s [%u] cell (%u,%u): byte %u says %s, the tiles hold %s -- %s, and "
                     "so is every tile above it",
                     name, o.face, o.cellX, o.cellY, o.byte, said.c_str(), truth.c_str(),
                     TileText(o.says).c_str());
            break;
        case AuditKind::Hole:
            snprintf(b, sizeof(b),
                     "HOLE    %s [%u] cell (%u,%u): %s, under %s (byte %u says %s, the unbroken "
                     "chain holds %s)",
                     name, o.face, o.cellX, o.cellY, TileText(o.says).c_str(),
                     TileText(o.gap, ", which").c_str(), o.byte, said.c_str(), truth.c_str());
            break;
        case AuditKind::Split:
            snprintf(b, sizeof(b),
                     "SPLIT   %s [%u] cell (%u,%u): its map texels disagree (the first says %s, "
                     "the tiles hold %s)",
                     name, o.face, o.cellX, o.cellY, said.c_str(), truth.c_str());
            break;
        case AuditKind::Orphan:
            snprintf(b, sizeof(b),
                     "ORPHAN  %s [%u] %s, and in no queue: nothing will load or map it again and "
                     "the ring gate holds its children behind it (finding 2's batch tail)",
                     name, o.face, TileText(o.says).c_str());
            break;
    }
    return b;
}

uint64_t AuditOffenderKey(const AuditOffender& o) {
    // kind 3 | tenant 6 | slice 7 | mip 4 | y 22 | x 22 bits. A cell names mip 0; an orphan its own.
    const uint64_t mip = (o.kind == AuditKind::Orphan) ? o.says.mip : 0u;
    return (uint64_t(o.kind) << 61) | (uint64_t(o.tenant & 63u) << 55) |
           (uint64_t(o.face & 127u) << 48) | ((mip & 15u) << 44) |
           (uint64_t(o.cellY & 0x3FFFFFu) << 22) | uint64_t(o.cellX & 0x3FFFFFu);
}

// ================================================================================================
//  THE MANAGER'S SIDE. Everything here reads the manager's own state; nothing writes it but the
//  audit's ledger.
// ================================================================================================
namespace {
constexpr size_t kAuditKeep = size_t(1) << 16;   // offenders kept by name per audit (counts are whole)
constexpr int kAuditNamed = 6;                    // new offenders printed per audit
constexpr size_t kAuditWatch = 64;                // invalidated addresses watched at once
constexpr uint64_t kAuditEvents = 200;            // INVALIDATE lines printed per run
constexpr uint32_t kAuditWatchTurns = 900;        // a watch that outlives this says so and ends

std::string Narrow(const std::wstring& w) {
    std::string s;
    for (wchar_t c : w) s.push_back(c < 128 ? static_cast<char>(c) : '?');
    return s;
}
}  // namespace

void ResidencyManager::AuditInvalidation(int tenant, const TileRequest& r, const Tracked* tr) {
    AuditLedger& a = m_audit;
    ++a.queued;
    ++a.runQueued;
    if (!tr) return;   // nothing tracked at the address: nothing drops, no byte moves
    ++a.tracked;
    ++a.runTracked;
    std::lock_guard<std::mutex> lk(m_mx);   // the loads write Tracked::state under it
    if (tr->state != TileState::Mapped) return;   // only a mapped tile's drop raises a byte
    ++a.mapped;
    ++a.runMapped;
    // Its mapped descendants, at every finer mip: the tiles whose cells DropOne's max() is about to
    // raise although they stay mapped. Every plane is a power of two tiles wide (AddTextureInternal
    // refuses any other), so a tile's descendants k mips down are a 2^k square of addresses.
    const Tenant& t = m_tenants[tenant];
    uint32_t below = 0;
    TileRequest first{};
    for (uint32_t m = r.mip; m-- > 0;) {
        const uint32_t k = r.mip - m;
        const auto& ti = t.tilings[r.face * t.mips + m];
        const uint32_t x1 = (std::min)((r.x + 1u) << k, uint32_t(ti.WidthInTiles));
        const uint32_t y1 = (std::min)((r.y + 1u) << k, uint32_t(ti.HeightInTiles));
        for (uint32_t y = r.y << k; y < y1; ++y) {
            for (uint32_t x = r.x << k; x < x1; ++x) {
                const Tracked* d = t.slot[StampIndex(t, r.face, m, x, y)];
                if (!d || d->state != TileState::Mapped) continue;
                if (below == 0) first = d->req;
                ++below;
            }
        }
    }
    if (!below) return;
    ++a.overMapped;
    ++a.runOverMapped;
    // Watched until it is mapped again: that is when finding 3 would leave its mark.
    if (a.watch.size() < kAuditWatch) {
        AuditLedger::Watch w;
        w.tenant = tenant;
        w.face = r.face;
        w.mip = r.mip;
        w.x = r.x;
        w.y = r.y;
        w.frame = m_frame;
        w.below = below;
        w.dMip = first.mip;
        w.dX = first.x;
        w.dY = first.y;
        a.watch.push_back(w);
    }
    if (a.eventsLogged < kAuditEvents) {
        ++a.eventsLogged;
        const auto& b0 = t.tilings[r.face * t.mips];
        const auto& bm = t.tilings[r.face * t.mips + r.mip];
        const uint32_t cells = (b0.WidthInTiles / (std::max)(1u, uint32_t(bm.WidthInTiles))) *
                               (uint32_t(b0.HeightInTiles) / (std::max)(1u, uint32_t(bm.HeightInTiles)));
        Log("[res-audit] rec%u f%u | INVALIDATE %s [%u] m%u (%u,%u): mapped, over %u mapped "
            "descendant(s), the first m%u (%u,%u) -- the drop raises the bytes over all %u of its "
            "cells to %s, the descendants' cells included",
            traceRecFrame, m_frame, Narrow(t.name).c_str(), r.face, r.mip, r.x, r.y, below,
            first.mip, first.x, first.y, cells, MipText(r.mip + 1u, t.mips).c_str());
    }
}

void ResidencyManager::LogAudit() {
    static_assert(uint8_t(TileState::Seen) == uint8_t(AuditState::Seen) &&
                      uint8_t(TileState::Loading) == uint8_t(AuditState::Loading) &&
                      uint8_t(TileState::Loaded) == uint8_t(AuditState::Loaded) &&
                      uint8_t(TileState::Mapped) == uint8_t(AuditState::Mapped) &&
                      uint8_t(TileState::Failed) == uint8_t(AuditState::Failed),
                  "AuditState mirrors TileState value for value");
    const auto t0 = std::chrono::steady_clock::now();   // the audit's own cost, printed with it
    // LANDED is the manager's own claim rule -- the landed loop claims a DirectStorage tile's byte
    // when its batch's fence signals, so a tile in a batch still in m_inFlightReads is mapped and
    // not landed -- and QUEUED is m_seen or m_loading. Both as sorted sets of the tiles.
    std::vector<const Tracked*> reading, queued;
    for (const auto& b : m_inFlightReads) {
        for (const auto& tile : b.tiles) reading.push_back(tile.get());
    }
    for (const auto& tile : m_loading) queued.push_back(tile.get());
    std::sort(reading.begin(), reading.end());
    std::sort(queued.begin(), queued.end());
    std::vector<AuditTenant> in(m_tenants.size());
    std::string deferred;   // tenants whose map upload found no staging room this turn
    // How each tenant's mapped tiles were answered: as a PLACE in an archive (the DirectStorage
    // path, the provider's TileLoc) or as BYTES (a loose file or a paint, the upload ring).
    std::vector<uint32_t> byPlace(m_tenants.size(), 0u), byBytes(m_tenants.size(), 0u);
    {
        std::lock_guard<std::mutex> lk(m_mx);   // the loads write Tracked::state under it
        for (size_t k = 0; k < m_tenants.size(); ++k) {
            const Tenant& t = m_tenants[k];
            AuditTenant& a = in[k];
            a.name = Narrow(t.name);
            a.faces = t.faces;
            a.mips = t.mips;
            a.tilesW.resize(t.tilings.size());
            a.tilesH.resize(t.tilings.size());
            for (size_t p = 0; p < t.tilings.size(); ++p) {
                a.tilesW[p] = t.tilings[p].WidthInTiles;
                a.tilesH[p] = t.tilings[p].HeightInTiles;
            }
            a.mapDim = t.resMap.width;
            a.bytes = &t.resCpu;
            a.top.assign(t.sliceTop.begin(), t.sliceTop.end());
            if (t.resDirty) deferred += " " + a.name;
            a.tiles.reserve(t.tracked.size());
            for (const auto& tr : t.tracked) {
                AuditTile x;
                x.face = tr->req.face;
                x.mip = tr->req.mip;
                x.x = tr->req.x;
                x.y = tr->req.y;
                x.state = static_cast<AuditState>(static_cast<uint8_t>(tr->state));
                x.landed = tr->state == TileState::Mapped &&
                           !std::binary_search(reading.begin(), reading.end(), tr.get());
                // Step 5: a Seen tile is in the order by construction (every tracked
                // tile is judged every turn); a Loading or a Loaded one must be in m_loading, or
                // nothing will map it or let it go: finding 2's orphan.
                x.queued = std::binary_search(queued.begin(), queued.end(), tr.get()) ||
                           tr->state == TileState::Seen;
                a.tiles.push_back(x);
                if (tr->state == TileState::Mapped) ++(tr->loc.Valid() ? byPlace[k] : byBytes[k]);
            }
        }
    }
    const AuditResult res = AuditResidency(in, kAuditKeep);
    const double auditMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    AuditLedger& L = m_audit;
    ++L.audits;
    const bool clean = res.Clean();
    if (!clean) ++L.dirty;
    std::vector<uint64_t> keys;
    keys.reserve(res.offenders.size());
    for (const AuditOffender& o : res.offenders) keys.push_back(AuditOffenderKey(o));
    std::sort(keys.begin(), keys.end());
    auto known = [&](uint64_t key) {
        return std::binary_search(L.lastKeys.begin(), L.lastKeys.end(), key);
    };
    size_t fresh = 0;
    for (const uint64_t key : keys) fresh += known(key) ? 0u : 1u;
    // Every mapped tile is tracked by exactly one tenant, so the tenants' mapped tiles ARE the
    // pool's; the pages ledger checks the same sum at its own point of the turn.
    const size_t pool = m_mapped.size();
    char poolText[96] = "";
    if (pool != res.mapped) {
        snprintf(poolText, sizeof(poolText), " -- POOL DISAGREES: the pool holds %zu", pool);
    }
    Log("[res-audit] rec%u f%u | %s | %zu tenants, %llu cells: finer %llu (worst %u mips), coarser "
        "%llu (worst %u), holes %llu cells / %u tiles (%llu under bytes in flight), split %llu, "
        "orphans %u (%u Loaded) | %llu new since the last audit, %llu persisting | mapped %u (%u "
        "landed, %u in flight)%s%s%s | invalidated %llu since the last audit: %llu tracked, %llu "
        "mapped, %llu over a mapped descendant; the run %llu / %llu / %llu / %llu | %.1f ms",
        traceRecFrame, m_frame, clean ? "clean" : "NOT CLEAN", in.size(),
        static_cast<unsigned long long>(res.sum.cells),
        static_cast<unsigned long long>(res.sum.finer), res.sum.worstFiner,
        static_cast<unsigned long long>(res.sum.coarser), res.sum.worstCoarser,
        static_cast<unsigned long long>(res.sum.holes), res.holeTiles,
        static_cast<unsigned long long>(res.sum.holesInFlight),
        static_cast<unsigned long long>(res.sum.split), res.orphans, res.orphansLoaded,
        static_cast<unsigned long long>(fresh), static_cast<unsigned long long>(keys.size() - fresh),
        res.mapped, res.landed, res.mapped - res.landed, poolText,
        deferred.empty() ? "" : " | map upload deferred:", deferred.c_str(),
        static_cast<unsigned long long>(L.queued), static_cast<unsigned long long>(L.tracked),
        static_cast<unsigned long long>(L.mapped), static_cast<unsigned long long>(L.overMapped),
        static_cast<unsigned long long>(L.runQueued), static_cast<unsigned long long>(L.runTracked),
        static_cast<unsigned long long>(L.runMapped), static_cast<unsigned long long>(L.runOverMapped),
        auditMs);
    L.queued = L.tracked = L.mapped = L.overMapped = 0;
    if (L.audits == 1 || L.audits % 150 == 0) {
        std::string paths;
        for (size_t k = 0; k < in.size(); ++k) {
            char s[160];
            snprintf(s, sizeof(s), " %s %u as places / %u as bytes;", in[k].name.c_str(), byPlace[k],
                     byBytes[k]);
            paths += s;
        }
        Log("[res-audit] rec%u f%u | the mapped tiles' paths:%s", traceRecFrame, m_frame,
            paths.c_str());
    }
    if (!clean) {
        // The tenants and slices that offend, then the offenders not reported before, by name.
        for (size_t k = 0; k < res.tenants.size(); ++k) {
            const AuditTenantResult& tr = res.tenants[k];
            if (tr.sum.Clean() && !tr.orphans) continue;
            std::string slices;
            for (size_t f = 0; f < tr.slices.size(); ++f) {
                const AuditCount& c = tr.slices[f];
                if (c.Clean()) continue;
                char s[128];
                snprintf(s, sizeof(s), " [%zu] finer %llu coarser %llu holes %llu split %llu;", f,
                         static_cast<unsigned long long>(c.finer),
                         static_cast<unsigned long long>(c.coarser),
                         static_cast<unsigned long long>(c.holes),
                         static_cast<unsigned long long>(c.split));
                slices += s;
            }
            Log("[res-audit]   %s: mapped %u (%u landed), hole tiles %u, orphans %u (%u Loaded) |%s",
                in[k].name.c_str(), tr.mapped, tr.landed, tr.holeTiles, tr.orphans,
                tr.orphansLoaded, slices.empty() ? " -" : slices.c_str());
        }
        int named = 0;
        for (const AuditOffender& o : res.offenders) {
            if (named >= kAuditNamed) break;
            if (known(AuditOffenderKey(o))) continue;
            Log("[res-audit]   %s", AuditOffenderText(o, in).c_str());
            ++named;
        }
        if (res.offendersTotal > res.offenders.size()) {
            Log("[res-audit]   (%llu offenders, the first %zu kept by name: new/persisting counts "
                "cover those)",
                static_cast<unsigned long long>(res.offendersTotal), res.offenders.size());
        }
    }
    L.lastKeys.swap(keys);
    // THE WATCHED INVALIDATIONS: an address dropped over a mapped descendant, back in the pool.
    // The bytes over its first descendant's first cell and that descendant's state are the whole
    // of finding 3 for that cell: the byte says the re-mapped tile's mip while the descendant is
    // still mapped and landed below it.
    std::lock_guard<std::mutex> lk(m_mx);
    for (size_t i = 0; i < L.watch.size();) {
        const AuditLedger::Watch& w = L.watch[i];
        const Tenant& t = m_tenants[w.tenant];
        const Tracked* now = Find(w.tenant, TileRequest{w.face, w.mip, w.x, w.y});
        const bool back = now && now->state == TileState::Mapped &&
                          !std::binary_search(reading.begin(), reading.end(), now);
        const bool expired = m_frame - w.frame > kAuditWatchTurns;
        if (!back && !expired) {
            ++i;
            continue;
        }
        const Tracked* d = Find(w.tenant, TileRequest{w.face, w.dMip, w.dX, w.dY});
        const auto& b0 = t.tilings[w.face * t.mips];
        const auto& bd = t.tilings[w.face * t.mips + w.dMip];
        const uint32_t cx = w.dX * (b0.WidthInTiles / (std::max)(1u, uint32_t(bd.WidthInTiles)));
        const uint32_t cy = w.dY * (uint32_t(b0.HeightInTiles) / (std::max)(1u, uint32_t(bd.HeightInTiles)));
        const uint32_t rdim = t.resMap.width;
        const uint32_t sx = rdim / (std::max)(1u, uint32_t(b0.WidthInTiles));
        const uint32_t sy = rdim / (std::max)(1u, uint32_t(b0.HeightInTiles));
        const uint8_t byte = t.resCpu[w.face][size_t(cy * sy) * rdim + cx * sx];
        const char* dState = !d ? "is no longer tracked"
                             : d->state != TileState::Mapped ? "is tracked and no longer mapped"
                             : std::binary_search(reading.begin(), reading.end(), d)
                                 ? "is mapped, its bytes in flight"
                                 : "is still mapped and landed";
        Log("[res-audit] rec%u f%u | %s %s [%u] m%u (%u,%u), invalidated at f%u over %u mapped "
            "descendant(s)%s: the byte over its descendant m%u (%u,%u)'s first cell (%u,%u) says "
            "%s, and that descendant %s",
            traceRecFrame, m_frame, back ? "REMAPPED" : "UNWATCHED", Narrow(t.name).c_str(), w.face,
            w.mip, w.x, w.y, w.frame, w.below, back ? "" : " (not mapped again within the watch)",
            w.dMip, w.dX, w.dY, cx, cy, MipText(byte / 16u, t.mips).c_str(), dState);
        L.watch.erase(L.watch.begin() + static_cast<std::ptrdiff_t>(i));
    }
}

// ================================================================================================
//  THE SELFTEST. The audit is held to states built here, on the manager's own layout law
//  (AddTextureInternal: a plane per face and mip, a square uv-addressed map over the mip-0 grid),
//  with the bytes written by the manager's REAL UpdateResidencyByte -- the rule under suspicion,
//  not a copy of it. Each planted failure must be CAUGHT at the cell it was planted in; an audit
//  that flags the rule's own correct sequences, or misses a plant, fails the suite.
// ================================================================================================
bool ResidencyManager::AuditSuite() {
    bool ok = true;
    // Unstarted: UpdateResidencyByte reads the tilings and writes the bytes, and nothing else --
    // no device, no pool, no thread takes part in the rule under test.
    ResidencyManager mgr;
    struct Model {
        Tenant t;
        std::vector<AuditTile> tiles;
    };
    // STEP 5's writer (order): the map a function of the held set, the held set the model's own
    // tiles -- Mapped and landed, the audit's definition of resident.
    auto heldIn = [](const void* ctx, uint32_t f, uint32_t m, uint32_t x, uint32_t y) -> bool {
        for (const AuditTile& a : static_cast<const Model*>(ctx)->tiles) {
            if (a.face == f && a.mip == m && a.x == x && a.y == y) {
                return a.state == AuditState::Mapped && a.landed;
            }
        }
        return false;
    };
    // A tenant on the manager's layout: faces x mips planes of (bw >> m) x (bh >> m) tiles, the
    // map at max(bw, bh) a side, every byte born "coarsest" as AddTextureInternal writes it.
    auto make = [](Model& md, const wchar_t* name, uint32_t faces, uint32_t bw, uint32_t bh,
                   uint32_t mips) {
        md.t.name = name;
        md.t.faces = faces;
        md.t.mips = mips;
        md.t.tilings.assign(size_t(faces) * mips, D3D12_SUBRESOURCE_TILING{});
        for (uint32_t f = 0; f < faces; ++f) {
            for (uint32_t m = 0; m < mips; ++m) {
                auto& ti = md.t.tilings[size_t(f) * mips + m];
                ti.WidthInTiles = (std::max)(1u, bw >> m);
                ti.HeightInTiles = static_cast<UINT16>((std::max)(1u, bh >> m));
                ti.DepthInTiles = 1;
            }
        }
        const uint32_t rdim = (std::max)(bw, bh);
        md.t.resMap.width = md.t.resMap.height = rdim;
        // The map is born saying nothing (law 9).
        md.t.resCpu.assign(faces, std::vector<uint8_t>(size_t(rdim) * rdim, uint8_t(255)));
        md.tiles.clear();
    };
    // Mapped through the real rule and held landed; `claim` false is a DirectStorage tile whose
    // fence has not signalled: Mapped, no byte written, not landed (MapAndFill's claim-if-ring).
    // Under step 5's writer the footprint is rewritten from the held set whatever the claim.
    auto map = [&](Model& md, uint32_t f, uint32_t m, uint32_t x, uint32_t y, bool claim = true) {
        AuditTile a;
        a.face = f;
        a.mip = m;
        a.x = x;
        a.y = y;
        a.state = AuditState::Mapped;
        a.landed = claim;
        md.tiles.push_back(a);
        WriteHeldFootprint(md.t, TileRequest{f, m, x, y}, heldIn, &md);
    };
    // Dropped: the real rule's max(), and gone from the set (DropOne and the evictor's release).
    auto unmap = [&](Model& md, uint32_t f, uint32_t m, uint32_t x, uint32_t y) {
        std::erase_if(md.tiles, [&](const AuditTile& a) {
            return a.face == f && a.mip == m && a.x == x && a.y == y;
        });
        WriteHeldFootprint(md.t, TileRequest{f, m, x, y}, heldIn, &md);
    };
    auto plain = [](const Model& md) {
        AuditTenant a;
        a.name = Narrow(md.t.name);
        a.faces = md.t.faces;
        a.mips = md.t.mips;
        for (const auto& ti : md.t.tilings) {
            a.tilesW.push_back(ti.WidthInTiles);
            a.tilesH.push_back(ti.HeightInTiles);
        }
        a.mapDim = md.t.resMap.width;
        a.bytes = &md.t.resCpu;
        a.tiles = md.tiles;
        a.top.assign(md.t.sliceTop.begin(), md.t.sliceTop.end());
        return a;
    };
    // The byte of a cell, written whole (every map texel of it) or one texel alone.
    auto setByte = [](Model& md, uint32_t f, uint32_t cx, uint32_t cy, uint8_t v, bool whole) {
        const uint32_t rdim = md.t.resMap.width;
        const uint32_t sx = rdim / md.t.tilings[size_t(f) * md.t.mips].WidthInTiles;
        const uint32_t sy = rdim / md.t.tilings[size_t(f) * md.t.mips].HeightInTiles;
        for (uint32_t j = 0; j < (whole ? sy : 1u); ++j) {
            for (uint32_t i = 0; i < (whole ? sx : 1u); ++i) {
                md.t.resCpu[f][size_t(cy * sy + j) * rdim + cx * sx + i] = v;
            }
        }
    };
    auto byteAt = [](const Model& md, uint32_t f, uint32_t cx, uint32_t cy) {
        const uint32_t rdim = md.t.resMap.width;
        const uint32_t sx = rdim / md.t.tilings[size_t(f) * md.t.mips].WidthInTiles;
        const uint32_t sy = rdim / md.t.tilings[size_t(f) * md.t.mips].HeightInTiles;
        return md.t.resCpu[f][size_t(cy * sy) * rdim + cx * sx];
    };

    // THE CLEAN STATES. A: an RGBA8 page's square grid (16 x 16 tiles, 5 mips, two slices).
    // B: an R16F page's 2:1 tiles (8 x 16, 4 mips), whose map texel is half a mip-0 tile wide.
    // Built coarse to fine -- the birth's coarsest tiles, then each tile only under a mapped
    // parent, the ring gate's order -- and then evicted fine to coarse, the evictor's.
    auto buildA = [&](Model& a) {
        make(a, L"selftest.rgba8", 2, 16, 16, 5);
        map(a, 0, 4, 0, 0);
        map(a, 1, 4, 0, 0);
        for (uint32_t y = 0; y < 2; ++y) {
            for (uint32_t x = 0; x < 2; ++x) map(a, 0, 3, x, y);
        }
        for (uint32_t y = 0; y < 2; ++y) {
            for (uint32_t x = 0; x < 2; ++x) map(a, 0, 2, x, y);
        }
        for (uint32_t y = 0; y < 2; ++y) {
            for (uint32_t x = 0; x < 2; ++x) map(a, 0, 1, x, y);
        }
        map(a, 0, 1, 2, 2);
        map(a, 0, 0, 0, 0);
        map(a, 0, 0, 1, 0);
        map(a, 0, 0, 2, 1);
        map(a, 1, 3, 1, 1);
        unmap(a, 0, 0, 2, 1);   // fine to coarse: the child, then the parent it leaves bare
        unmap(a, 0, 1, 1, 0);
    };
    auto buildB = [&](Model& b) {
        make(b, L"selftest.r16f", 1, 8, 16, 4);
        map(b, 0, 3, 0, 0);
        map(b, 0, 3, 0, 1);
        map(b, 0, 2, 0, 0);
        map(b, 0, 2, 1, 1);
        map(b, 0, 1, 0, 0);
        map(b, 0, 1, 1, 1);
        map(b, 0, 0, 0, 0);
        map(b, 0, 0, 2, 3);
    };
    auto run = [&](std::initializer_list<const Model*> ms) {
        std::vector<AuditTenant> in;
        for (const Model* m : ms) in.push_back(plain(*m));
        return std::make_pair(AuditResidency(in, 4096), in);
    };
    auto summary = [](const AuditResult& r) {
        char b[192];
        snprintf(b, sizeof(b), "%llu cells: %llu finer, %llu coarser, %llu holes, %llu split, %u orphans",
                 static_cast<unsigned long long>(r.sum.cells),
                 static_cast<unsigned long long>(r.sum.finer),
                 static_cast<unsigned long long>(r.sum.coarser),
                 static_cast<unsigned long long>(r.sum.holes),
                 static_cast<unsigned long long>(r.sum.split), r.orphans);
        return std::string(b);
    };
    // A plant is CAUGHT when the audit names an offender of the planted kind at the planted
    // place (tenant, slice, cell -- an orphan's own address), never merely "something".
    auto expect = [&](const char* what, const std::pair<AuditResult, std::vector<AuditTenant>>& r,
                      AuditKind kind, uint32_t tenant, uint32_t face, uint32_t cx, uint32_t cy) {
        const AuditOffender* hit = nullptr;
        for (const AuditOffender& o : r.first.offenders) {
            if (o.kind == kind && o.tenant == tenant && o.face == face && o.cellX == cx &&
                o.cellY == cy) {
                hit = &o;
                break;
            }
        }
        if (hit) {
            Log("[res-audit] %s: CAUGHT -- %s", what, AuditOffenderText(*hit, r.second).c_str());
        } else {
            Log("[res-audit] %s: MISSED (%s)", what, summary(r.first).c_str());
            ok = false;
        }
    };
    // Step 5's writer over a gap: the tile waits in the map and no byte names it (law 3).
    auto noFiner = [&](const char* what, const std::pair<AuditResult, std::vector<AuditTenant>>& r) {
        Log("[res-audit] %s through step 5's writer: %llu cells FINER than the tiles -- %s", what,
            static_cast<unsigned long long>(r.first.sum.finer),
            r.first.sum.finer ? "the writer named a tile under a gap" : "the map names the whole chain");
        if (r.first.sum.finer) ok = false;
    };

    // ---- 0. The controls: the rule's own orders audit clean on both grids.
    {
        Model a, b;
        buildA(a);
        buildB(b);
        const auto r = run({&a, &b});
        const bool clean = r.first.Clean() && r.first.mapped == a.tiles.size() + b.tiles.size();
        Log("[res-audit] control: the real %s, mapped coarse to fine and evicted "
            "fine to coarse on a square and a 2:1 tile grid (%u tiles mapped), audits %s -- %s",
            "WriteHeldFootprint", r.first.mapped,
            clean ? "CLEAN" : "NOT CLEAN", summary(r.first).c_str());
        if (!clean) {
            ok = false;
            for (size_t i = 0; i < r.first.offenders.size() && i < 4; ++i) {
                Log("[res-audit]   %s", AuditOffenderText(r.first.offenders[i], r.second).c_str());
            }
        }
    }
    // ---- 1. A byte lowered one mip. Cell (8,8) of A's slice 0 stands on m3 (1,1) and no finer.
    {
        Model a;
        buildA(a);
        setByte(a, 0, 8, 8, static_cast<uint8_t>(byteAt(a, 0, 8, 8) - 16), true);
        expect("plant 1, a byte lowered one mip", run({&a}), AuditKind::Finer, 0, 0, 8, 8);
    }
    // ---- 2. A byte raised one mip. Cell (0,0) stands on m0 (0,0).
    {
        Model a;
        buildA(a);
        setByte(a, 0, 0, 0, static_cast<uint8_t>(byteAt(a, 0, 0, 0) + 16), true);
        expect("plant 2, a byte raised one mip", run({&a}), AuditKind::Coarser, 0, 0, 0, 0);
    }
    // ---- 3. A tile gone from the mapped set while its byte stays: m0 (1,0), no rule applied.
    {
        Model a;
        buildA(a);
        std::erase_if(a.tiles, [](const AuditTile& t) {
            return t.face == 0 && t.mip == 0 && t.x == 1 && t.y == 0;
        });
        expect("plant 3, a tile unmapped under its byte", run({&a}), AuditKind::Finer, 0, 0, 1, 0);
    }
    // ---- 4. A chain with a hole: m0 (6,6) mapped through the real rule under an unmapped m1 (3,3).
    {
        Model a;
        buildA(a);
        map(a, 0, 0, 6, 6);
        const auto r = run({&a});
        expect("plant 4, a mip mapped under a missing parent", r, AuditKind::Hole, 0, 0, 6, 6);
        noFiner("plant 4", r);
    }
    // ---- 5. Finding 2's batch tail: m0 (5,5) Loaded, tracked, in no queue.
    {
        Model a;
        buildA(a);
        AuditTile t;
        t.face = 0;
        t.mip = 0;
        t.x = 5;
        t.y = 5;
        t.state = AuditState::Loaded;
        a.tiles.push_back(t);
        expect("plant 5, a loaded tile in no queue", run({&a}), AuditKind::Orphan, 0, 0, 5, 5);
    }
    // ---- 6. Finding 24: m2 (1,1) mapped by DirectStorage and not landed, its child m1 (2,2)
    // ring-filled and claimed. The child's cells claim m1 over a parent the sampler cannot read.
    {
        Model a;
        make(a, L"selftest.rgba8", 2, 16, 16, 5);
        map(a, 0, 4, 0, 0);
        map(a, 1, 4, 0, 0);
        map(a, 0, 3, 0, 0);
        map(a, 0, 2, 1, 1, false);
        map(a, 0, 1, 2, 2);
        const auto r = run({&a});
        // The writer names the whole chain, m3; the child waits in the map as a HOLE (law 3).
        expect("plant 6, ...and the chain's gap is named", r, AuditKind::Hole, 0, 0, 4, 4);
        noFiner("plant 6", r);
    }
    // ---- 7. A split cell on the 2:1 grid: one of cell (2,3)'s two map texels written alone.
    {
        Model b;
        buildB(b);
        setByte(b, 0, 2, 3, static_cast<uint8_t>(byteAt(b, 0, 2, 3) + 16), false);
        expect("plant 7, a cell whose map texels disagree", run({&b}), AuditKind::Split, 0, 0, 2, 3);
    }
    // ---- 8. FINDING 3, REPLAYED THROUGH THE REAL RULE on A's slice 1: map a parent P = m2 (2,2),
    // map its child C = m1 (4,4) (cells (8..9, 8..9)), invalidate P (DropOne's own call: the rule's
    // max, and P gone from the set), map P again.
    {
        Model a;
        buildA(a);
        map(a, 1, 2, 2, 2);
        map(a, 1, 1, 4, 4);
        const auto r1 = run({&a});
        const bool before = r1.first.Clean();
        unmap(a, 1, 2, 2, 2);
        const auto r2 = run({&a});
        uint32_t holeCells = 0, holeBytesTrue = 0;
        for (const AuditOffender& o : r2.first.offenders) {
            if (o.kind != AuditKind::Hole || o.face != 1) continue;
            ++holeCells;
            if (o.byte / 16u == o.truth) ++holeBytesTrue;
        }
        Log("[res-audit] finding 3, replayed through the real %s on a constructed "
            "tenant: map P = m2 (2,2) and its child C = m1 (4,4): %s. Unmap P (the invalidation's "
            "DropOne): %u HOLE cells under C, and on %u of them the byte says the truth of the "
            "broken chain (%s) -- the rule is right while the hole stands (%s)",
            "WriteHeldFootprint", before ? "clean" : "NOT CLEAN",
            holeCells, holeBytesTrue,
            MipText(byteAt(a, 1, 8, 8) / 16u, a.t.mips).c_str(), summary(r2.first).c_str());
        if (!before || holeCells != 4 || holeBytesTrue != 4) ok = false;
        map(a, 1, 2, 2, 2);
        const auto r3 = run({&a});
        uint32_t coarse = 0;
        for (const AuditOffender& o : r3.first.offenders) {
            coarse += (o.kind == AuditKind::Coarser && o.face == 1) ? 1u : 0u;
        }
        {
            // The answer to finding 3: the footprint is rewritten from the held set, so P's
            // return names C's mip again under C and nothing is left coarser.
            (void)coarse;
            const bool clean = r3.first.Clean();
            Log("[res-audit] finding 3 through step 5's writer: map P again -- the audit reports %s, "
                "%s: the byte over C says %s again",
                summary(r3.first).c_str(), clean ? "CLEAN" : "NOT CLEAN",
                MipText(byteAt(a, 1, 8, 8) / 16u, a.t.mips).c_str());
            if (!clean) ok = false;
        }
    }
    // ---- 6. F1: A WINDOW'S FLOOR. A slice whose chain ends at mip 3 (16 x 16 tiles, 6 mips): its
    // byte is computed from mips 0..3 alone, 255 where the floor's tile is not held (the rank above
    // answers), and nothing above the floor is ever held or spoken of.
    {
        auto buildW = [&](Model& w) {
            make(w, L"selftest.window", 1, 16, 16, 6);
            w.t.sliceTop.assign(1, uint8_t(3));
            map(w, 0, 3, 0, 0);
            map(w, 0, 3, 1, 0);
            map(w, 0, 2, 0, 0);
            map(w, 0, 1, 0, 0);
            map(w, 0, 0, 0, 0);
        };
        Model w;
        buildW(w);
        const auto r = run({&w});
        const uint8_t b0 = byteAt(w, 0, 0, 0), b8 = byteAt(w, 0, 8, 0), b12 = byteAt(w, 0, 12, 12);
        const bool clean = r.first.Clean() && b0 == 0 && b8 == 48 && b12 == 255;
        Log("[res-audit] window floor: a slice held to its mip 3 audits %s (%s); the byte over a cell "
            "on m0 says %u, on the floor alone %u, under an unheld floor tile %u (255: the rank above "
            "answers) -- %s", clean ? "CLEAN" : "NOT CLEAN", summary(r.first).c_str(), b0, b8, b12,
            clean ? "right" : "WRONG");
        if (!clean) ok = false;
        {   // PLANT a: the audit without the floor -- the chain must reach mips 4..5, never held.
            auto rr = run({&w});
            rr.second[0].top.clear();
            rr.first = AuditResidency(rr.second, 4096);
            expect("plant 8, a window's floor forgotten by the audit", rr, AuditKind::Finer, 0, 0, 0, 0);
        }
        {   // PLANT b: a byte raised past the floor, mip 4, which the slice never holds.
            Model w2;
            buildW(w2);
            setByte(w2, 0, 8, 0, uint8_t(4 * 16), true);
            expect("plant 9, a window byte raised past its floor", run({&w2}), AuditKind::Coarser, 0, 0, 8, 0);
        }
        {   // PLANT c: the writer without the floor (today's chain to the array's coarsest): the
            // floor's tiles held, the bytes say nothing -- key7km's rank 5 at A4.
            Model w3;
            buildW(w3);
            w3.t.sliceTop.clear();
            WriteHeldFootprint(w3.t, TileRequest{0, 3, 0, 0}, heldIn, &w3);
            w3.t.sliceTop.assign(1, uint8_t(3));
            expect("plant 10, the writer's chain past a window's floor", run({&w3}), AuditKind::Coarser, 0, 0, 0, 0);
        }
    }
    {
        Log(ok ? "[res-audit] ---- PASS (step 5's writer, WriteHeldFootprint): the controls clean on "
                 "both grids from a map born saying nothing; the 7 plants CAUGHT where they were "
                 "planted, a tile under a missing parent and a claim over a parent in flight as HOLES "
                 "with no byte finer than the tiles (law 3); finding 3's sequence reads clean once P "
                 "is back ----"
               : "[res-audit] ---- FAIL (step 5's writer) ----");
    }
    return ok;
}

bool ResidencyManager::AuditSelfTest() {
    const bool order = AuditSuite();   // the one writer: the map a function of the held set
    // STEP 5 D, THE ORDER'S RUNG KEY (the coordinator's finding a). The held set is closed upward
    // by construction only if a parent's bucket comes strictly before its child's: at every rung
    // of the pyramid, 0 (611 m) to 17 (4.7 mm), and every mip of a window standing on it. With too
    // few buckets the finest rungs clamp into one, and a parent and its child are then ordered by
    // weight: the plant (the bucket range this manager was first built with) must fail at rung 16.
    // F: the key is the measure, texel / max(distance, texel). For every rung of the pyramid, every
    // mip of a window on it and eye distances from 1 m to 10,000 km, a parent (twice the texel, at
    // no greater distance: the child's distance, the worst case) is never in a bucket after its
    // child's. The plant is the texel taken the wrong way up the mips (halving with the mip).
    auto closedUp = [](bool inverted, uint32_t& pairs, int& badRung, uint32_t& badMip) {
        pairs = 0;
        badRung = -1;
        badMip = 0;
        const double rung0M = 6371000.0 * 1.5707963267948966 / 16384.0;   // the cube's mip 0
        for (int k = 0; k <= 17; ++k) {
            const double g0 = rung0M / double(1 << k);
            for (uint32_t m = 0; m + 1 < 8; ++m) {
                const float tc = static_cast<float>(inverted ? g0 / double(1u << m) : g0 * double(1u << m));
                const float tp = static_cast<float>(inverted ? g0 / double(2u << m) : g0 * double(2u << m));
                for (double d = 1.0; d <= 1.0e7; d *= 1.3) {
                    ++pairs;
                    const uint32_t child = MeasureBucket(Measure(tc, static_cast<float>(d)));
                    const uint32_t parent = MeasureBucket(Measure(tp, static_cast<float>(d)));
                    if (parent > child && badRung < 0) {
                        badRung = k;
                        badMip = m;
                    }
                }
            }
        }
        return badRung < 0;
    };
    uint32_t pairs = 0, badMip = 0, plantPairs = 0, plantMip = 0;
    int badRung = -1, plantRung = -1;
    const bool keyOk = closedUp(false, pairs, badRung, badMip);
    const bool plantCaught = !closedUp(true, plantPairs, plantRung, plantMip);
    Log("[res-audit] the order's key, the texel's size on the reader's screen: a parent's bucket never "
        "after its child's at rungs 0 to 17, mips 0 to 7 and eye distances 1 m to 10,000 km (%u "
        "pairs): %s; the plant, the texel taken the wrong way up the mips, %s",
        pairs, keyOk ? "HOLDS" : "FAILS", plantCaught ? "is caught" : "is NOT caught");
    if (!keyOk) Log("[res-audit]   the key fails first at rung %d, mip %u", badRung, badMip);
    if (plantCaught) Log("[res-audit]   the plant fails first at rung %d, mip %u", plantRung, plantMip);
    const bool marginOk = OrderSelfTest();   // H2: the hold's margin and the closure under it
    return order && keyOk && plantCaught && marginOk;
}

bool RunResidencyAuditSelfTest() { return ResidencyManager::AuditSelfTest(); }

}  // namespace ga
