// ================================================================================================
//  ResidencyAudit - THE RESIDENCY BYTES AGAINST THE MAPPED SET (docs/HIERARCHY.md section 6,
//  step 1: the instrument the residency manager's replacement is held to).
//
//  A page tenant's residency map holds one byte per mip-0 tile of each slice: the finest mip
//  resident there, times 16. Every shader trusts it as its min-LOD clamp (PageSample.hlsli), so a
//  byte FINER than what is resident sends the sampler into a NULL tile's zeros, and a byte COARSER
//  than what is resident draws a softer picture than the memory holds. The manager keeps the
//  bytes INCREMENTALLY (ResidencyManager::UpdateResidencyByte: min on a map, max on an unmap),
//  which is right only while maps and unmaps arrive in the order the rule assumes -- coarse before
//  fine, and nothing unmapped while a finer tile stands over its cells. The review read two ways
//  that order breaks (docs/REVIEW_2026-09-28.md): an invalidation drops a mapped tile whose
//  descendants stay mapped, and the re-map restores only the tile's own mip (finding 3); at the
//  pool's cap a batch's tail is left Loaded, tracked and in no queue (finding 2). Neither had been
//  seen when this was written.
//
//  WHAT A BYTE SHOULD BE is a function of the tiles alone: the finest mip m such that the tile over
//  the cell at every mip from m up to the coarsest is MAPPED AND LANDED. Landed, because a
//  DirectStorage tile is Mapped the turn its read is issued and its bytes arrive a turn or more
//  later (finding 24); the manager claims its byte only once the batch's fence has signalled, so
//  "landed" here is the manager's own claim rule -- Mapped, and in no batch still in flight --
//  and a tile that is Mapped but not landed is a gap in the chain, which is what the sampler meets.
//  A resident tile under a gap is a HOLE, reported apart: the byte's rule assumes there are none,
//  and over a hole the byte can agree with the truth and still be the wrong law. A tracked tile
//  that is neither mapped nor failed and sits in no queue is an ORPHAN: finding 2's signature,
//  which no byte shows because no byte ever claimed it.
//
//  AuditResidency is a PURE function of plain data -- the tilings, the tiles and their states, the
//  bytes -- so the selftest runs it on states it constructs, with no device, and the engine runs it
//  on the manager's own at the end of a turn (ResidencyManager::LogAudit; the scene's
//  capture.residencyAudit, the flag --res-audit N). What it cannot see: the GPU's copy of the
//  bytes (it audits the CPU bytes a turn uploads, and names a tenant whose upload was deferred),
//  the ORDER of the queue's work inside a frame (finding 34), a byte that is right over a tile
//  whose bytes are wrong, and a picture.
// ================================================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ga {

// A tracked tile's state, value for value ResidencyManager::TileState (static_assert'd where both
// are visible, ResidencyAudit.cpp).
enum class AuditState : uint8_t { Seen = 0, Loading = 1, Loaded = 2, Mapped = 3, Failed = 4 };

struct AuditTile {
    uint32_t face = 0, mip = 0, x = 0, y = 0;
    AuditState state = AuditState::Seen;
    bool landed = false;   // Mapped, and no DirectStorage batch still carries its bytes
    bool queued = false;   // in the seen or the loading queue
};

// One tenant, as the manager holds it.
struct AuditTenant {
    std::string name;
    uint32_t faces = 0, mips = 0;
    std::vector<uint32_t> tilesW, tilesH;   // per plane (face * mips + mip), in tiles
    std::vector<uint32_t> top;              // per face, the coarsest mip it holds (F1); empty = mips - 1
    uint32_t mapDim = 0;                     // the residency map's side: it is square, uv-addressed
    const std::vector<std::vector<uint8_t>>* bytes = nullptr;   // per slice, mapDim * mapDim
    std::vector<AuditTile> tiles;            // every tracked tile, in any state
};

enum class AuditKind : uint8_t { Finer, Coarser, Hole, Split, Orphan };

// A tile named in a report: its address, and what the manager holds there.
struct AuditTileRef {
    uint32_t mip = 0, x = 0, y = 0;
    int state = -1;        // an AuditState, or -1: nothing is tracked at this address
    bool landed = false;
};

struct AuditOffender {
    AuditKind kind = AuditKind::Finer;
    uint32_t tenant = 0, face = 0;
    uint32_t cellX = 0, cellY = 0;   // the mip-0 cell (an ORPHAN: its own tile's address)
    uint8_t byte = 0;                // the residency byte (a SPLIT: the cell's first map texel)
    uint32_t truth = 0;              // the mip the byte should say; the tenant's `mips` = nothing
    AuditTileRef says;               // the tile that says so (AuditOffenderText spells which)
    AuditTileRef gap;                // a HOLE's missing coarser tile
};

// Counts in mip-0 cells; the worst offence in mips.
struct AuditCount {
    uint64_t cells = 0, finer = 0, coarser = 0, holes = 0, holesInFlight = 0, split = 0;
    uint32_t worstFiner = 0, worstCoarser = 0;
    void Add(const AuditCount& o);
    bool Clean() const { return !finer && !coarser && !holes && !split; }
};

struct AuditTenantResult {
    std::vector<AuditCount> slices;
    AuditCount sum;
    uint32_t mapped = 0, landed = 0;   // tiles in state Mapped, and of them landed
    uint32_t holeTiles = 0;            // resident tiles under a gap (a hole counted once per tile)
    uint32_t orphans = 0, orphansLoaded = 0;
};

struct AuditResult {
    std::vector<AuditTenantResult> tenants;
    AuditCount sum;
    uint32_t mapped = 0, landed = 0, holeTiles = 0, orphans = 0, orphansLoaded = 0;
    std::vector<AuditOffender> offenders;   // in scan order, at most the `keep` asked for
    uint64_t offendersTotal = 0;            // ...and how many there were
    bool Clean() const { return sum.Clean() && !orphans; }
};

// Every byte of every tenant against the truth its tiles make, every tile's chain, every
// orphan. `keep` bounds the offenders returned by name; the counts are always whole.
AuditResult AuditResidency(const std::vector<AuditTenant>& tenants, size_t keep);

// One offender as a line of prose: what the byte says, what the tiles hold, and the tile that
// says so -- for FINER the first tile down the chain that is not resident (the one a sample at
// the byte's mip would read), for COARSER the finest tile of the intact chain, for a HOLE the
// resident tile and the missing one above it.
std::string AuditOffenderText(const AuditOffender& o, const std::vector<AuditTenant>& tenants);

// A stable name for an offender -- kind, tenant, slice, cell -- so the engine can tell an offender
// it has not reported before from one that persists.
uint64_t AuditOffenderKey(const AuditOffender& o);

// What the engine remembers between two audits (ResidencyManager's m_audit): the invalidations
// applied since the last one and over the run, the offenders last reported, and the addresses an
// invalidation dropped over a mapped descendant, watched until they are mapped again.
struct AuditLedger {
    uint64_t queued = 0, tracked = 0, mapped = 0, overMapped = 0;            // since the last audit
    uint64_t runQueued = 0, runTracked = 0, runMapped = 0, runOverMapped = 0;   // over the run
    uint64_t audits = 0, dirty = 0, eventsLogged = 0;
    std::vector<uint64_t> lastKeys;   // the last audit's offenders, sorted
    struct Watch {
        int tenant = 0;
        uint32_t face = 0, mip = 0, x = 0, y = 0;   // the invalidated tile
        uint32_t frame = 0;                          // the turn that dropped it
        uint32_t below = 0;                          // its mapped descendants then
        uint32_t dMip = 0, dX = 0, dY = 0;           // the first of them
    };
    std::vector<Watch> watch;
};

// --selftest: the audit against states it constructs, the planted failures, and finding 3's own
// sequence replayed through the manager's real byte rule (ResidencyManager::AuditSelfTest).
bool RunResidencyAuditSelfTest();

}  // namespace ga
