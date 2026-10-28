// ================================================================================================
//  M5c: the sparse shallow-water solver -- the estuary actually FLOWING.
//
//  THE WATER AS COLUMNS (WATER_V2 section 12, 2026-10-23). The state is the water each column HOLDS,
//  h >= 0 (volume per area, gH), over its bed b (the one stack, cached a frame in gBedC), and the faces'
//  velocities. The surface is b + h; wet is h > gWetEps (1 cm, the solver's one tolerance); a NULL tile
//  holds no water. No reference level appears in the state: not the planet's waterline, not a station's
//  plane. The tide enters only through the boundary -- Flather faces where the sea continues past the
//  edge (the Level along the side, gOceanEta) and on the river faces -- and the prism arrives because the
//  boundary rises. The old state was the DEVIATION from a moving tide plane, with the plane's rise booked
//  as debt in every cell (dry ones too: their stored value sank under their own beds) and the open sea a
//  sponge that ate whatever arrived; both are gone. C2: the land soaks (Green and Ampt) and the
//  residency follows the water (CsSweTileStats); C4: the sea the columns carry pushes them (CsSweWaves,
//  the radiation stress in FaceUpdate).
//
//  Scheme: Stelling & Duinmeijer (2003) on the staggered C-grid -- face velocities (ping-pong: the
//  advection reads the substep before), each substep's fluxes the velocity times the water's UPWIND
//  depth at the face. The momentum advection conserves momentum where the flow decelerates (a bore, a
//  jump, a front slowing onto the flats) and the energy head where it accelerates (over a crest, down a
//  rarefaction onto a dry bed); both vanish together where the flow is uniform, so the choice is
//  continuous. The pressure is the hydrostatic reconstruction at the sill (Audusse et al. 2004): a
//  lake at rest stays at rest. Positivity: a draining column's outgoing fluxes are scaled together to
//  the water it holds (Bollermann et al. 2013's limit on draining cells: CsSweLimit).
//
//  PUBLISHED for the readers (CsSweDerive): gEta = the column's surface less the frame's tide plane
//  gTideNavd (the readers add it back: WaterBank.hlsl, SweSolver::DeviationAt), and gUv = (u east,
//  v north, h, wet).
//
//  Texture space: +x = east, +y = SOUTH (bathy row 0 is the northern edge). CsSweDerive flips v.
// ================================================================================================

#include "WindowRows.hlsli"

cbuffer SweCb : register(b0) {
    uint  gNx, gNy;              // bathy grid dims (1701 x 890)
    uint  gEtaTilesX, gFluxTilesX;
    uint  gEtaTileW, gEtaTileH, gFluxTileW, gFluxTileH;
    uint  gListCount;
    float gUnused0;              // the slot the open-side bitmask held (the edge is one law since 2026-10-27); keeps gFaceA on its register
    float gDy;                   // M6r: north-south texel metres (equiangular CUDEM grid:
                                 // dy = 13.65 != dx = 10.08 -- the per-axis metric)
    float gFaceCount;            // the river faces (gFaceA, gFaceB; SweSolver.h SweConfig::faces)
    float gDx, gDt, gDamp, gTideNavd;
    float gWetEps, gSoilRate, gCd, gGravity;   // wet is h > gWetEps; gSoilRate: C2's soil (0 = none);
                                               // gCd: the bed's quadratic drag coefficient
    // THE RIVER FACES (2026-10-21): where a river enters the domain (sim/RiverModel.h). A face: its
    // side (0 N, 1 S, 2 W, 3 E), the run of edge cells [c0, c1] along it (gFaceA); Flather's u_ext
    // (m/s, + into the domain: the face's TRANSPORT -- river Q minus the upriver prism's demand --
    // over its live section; radiation alone cannot carry a prism) and its exterior's level as a
    // departure from gTideNavd (river tide - ocean tide) (gFaceB).
    float4 gFaceA[4];
    float4 gFaceB[4];
    float gOceanU;               // the open sides' Flather u_ext, m/s + inward (SweSolver::Record: the
                                 // domain's stateless prism less the rivers' transport, over the sides' section)
    // STAGE 1c OF ONE WATER ON THE TREE (2026-10-26, docs/WATER_V2.md 14): the mip this dispatch runs at, how
    // many mips the chain holds, and the tiles across at the mip above (UpLiveAt's row stride).
    float gMip, gMips, gEtaTilesXUp;
    // PHASE C1: a cell's place in the domain's chart (SweSolver::SetBed): the first cell's centre in
    // the anchor's tangent plane over R and the steps (s0, ds, t0, dt); the anchor's up and R; its
    // east and the cell's grain (m). North = east x up.
    float4 gCellX;
    float4 gAnc;
    float4 gEast;
    HP_WINDOW_ROWS_DECL
    // THE OPEN SEA'S LEVEL (C1): per side (0 N, 1 S, 2 W, 3 E), the manager's Level at four points along
    // it (a = 0, 1/3, 2/3, 1 of its length, from the side's first cell), as departures from gTideNavd.
    float4 gOceanEta[4];
    float4 gSoil;                // C2: the soil store (x capacity m, y conductivity m/s, z wetting front
                                 // suction head m, w 1 = on)
    float4 gTestA;               // the scheme's tests (SWE_TEST): the case, its square's first cell, its side
    float4 gTestB;               // hL, hR, the lake's level, 0
    // C4: THE SEA AT THE DOMAIN'S OPEN WATER (SweSolver::SetSeaSource: the global wave grid there) --
    // gSea: Hs (m), the peak's angular frequency (rad/s), its energy Hs^2 / 16 (m^2), 1 = waves push;
    // gSea2: the direction it travels (a unit vector, texture frame: x east, y south), 0, 0.
    float4 gSea;
    float4 gSea2;
    // STAGE 3 OF ONE WATER ON THE TREE (2026-10-27, docs/WATER_V2.md 14): AN IMPACT IMPOSED ON THE COLUMNS
    // (CsSweImpose). gImpactA: the impact's place in the anchor's chart (east, north; metres), the seconds since
    // its zero at this batch's start, the basin depth h0 its table was built over; gImpactB: the table's term
    // count (gImpactTerms: k, omega, A, 0 a term; 0 = nothing to impose), 0, 0, 0. APPENDED AT THE END on both sides.
    float4 gImpactA;
    float4 gImpactB;
    // STAGE 2b (2026-10-27, docs/WATER_V2.md 14): THE TAIL -- the coarse lattice across the ocean, about the same
    // anchor, its cells 2^rank of this lattice's. gTail: the shift from this lattice's cells to the tail's
    // (rank less this mip; < 0 = no tail, or this IS the tail), the tail cell the window's corner lies in (x, y),
    // the tail's cells across; gTail2: the tail's cells down, its tiles across, 0, 0. The lattice edge's
    // exterior Level adds the tail's departure there (TailDepAt) and its velocity (TailUAt).
    float4 gTail;
    float4 gTail2;
    float4 gImpactC;             // stage 3: the event's cavity radius R_c and depth D_c (m), 0, 0
    // STAGE 2d (2026-10-27): THE MAGNIFY'S PARENT -- the shift from this lattice's cells to the one above's and the
    // cell above the corner lies in (x, y): (1, 0, 0) for the window (mip 1 of its own banks), the parent level's for
    // a level (UpLiveAt, the chain reads). A level's non-resident cell is held by its parent: the front of a train
    // leaving the level's resident set meets the parent's water, not a frontier. w: 0.
    float4 gUp;
};

#define HP_WINDOW_ROWS 1
#include "HeightPages.hlsli"
// The water's laws this solver shares with the bank and the hull (shaders/laws): Guo's dispersion
// and Battjes-Janssen's height, one text.
typedef float real;
#define GA_PLANET_G (gGravity)
#include "laws/Dispersion.hlsli"
#include "laws/Breaking.hlsli"
#include "laws/Impact.hlsli"

// THE CELL'S POINT about the domain's anchor C (the standing window's centre), in C's east / up /
// north: the cell's place (s, t) in the anchor's tangent plane (over R) carried onto the sphere along
// its ray from the centre -- R (s k, k - 1, t k), k = 1 / sqrt(1 + s^2 + t^2), with k - 1 taken as
// -q / (sqrt(1 + q) (1 + sqrt(1 + q))) so no two large numbers are subtracted.
float3 SwePointAbout(float s, float t) {
    const float q = s * s + t * t;
    const float r = sqrt(1.0f + q);
    const float k = 1.0f / r;
    return gAnc.w * float3(s * k, -q / (r * (1.0f + r)), t * k);
}

// THE CHART'S METRIC (stage 2b, 2026-10-27): a cell's true size, metres along east and north, from its place
// (s, t) in the anchor's tangent plane over R. A step ds on the plane is R ds sqrt(1 + t^2) / (1 + s^2 + t^2)
// on the sphere along s, and dt is R dt sqrt(1 + s^2) / (1 + s^2 + t^2) along t: the gnomonic chart's diagonal
// metric. Its shear, s t / (1 + s^2 + t^2), is left out -- nought on the axes through the anchor, 0.2 at the
// tail's far corners (the sphere question of 14.4; the cube-sphere tree is its answer). gDx and gDy are the
// sizes at the anchor (q = 1): a 28 km window's cells change by 5e-6, the tail's edge 5000 km out by 0.6.
float2 CellM(int2 t) {
    const float s = gCellX.x + float(t.x) * gCellX.y, tn = gCellX.z + float(t.y) * gCellX.w;
    const float q = 1.0f + s * s + tn * tn;
    return float2(gDx * sqrt(1.0f + tn * tn) / q, gDy * sqrt(1.0f + s * s) / q);
}

StructuredBuffer<uint> gTileList : register(t0);
// THE LIVE TILES (C1): one word a tile of the state's bank, 1 where it is resident. A NULL tile holds
// no water and takes none: its faces are walls until the residency maps it (SweSolver::Record).
StructuredBuffer<uint> gLive : register(t3);
// STAGE 1c: the live words of the mip above this dispatch's (SweSolver: m_live1 under mip 0; none above mip 1)
StructuredBuffer<uint> gLiveUp : register(t4);
StructuredBuffer<uint> gImpactTerms : register(t5);   // stage 3: an impact's table (k, omega, A, 0 a term), as floats
StructuredBuffer<uint> gLiveTail : register(t6);      // stage 2b: the tail's live words (one a tile)
// M9ar: THE BED IS THE HEIGHT MEGATEXTURE. M9ax: the tenant's WHOLE array (cube faces 0..5 and
// the eye's windows and the standing window: PHASE B2/B3) with its residency-map array, resolved by HeightPages.hlsli
// exactly as the pixel stage resolves it. There is no solver-private copy of the bed, and no
// wall past the page: a lattice anywhere on the planet has a bed. (C1: the solver reads it ONCE a
// frame per cell, into gBedC -- CsSweBedFill -- not eight times a cell a substep.)
Texture2DArray<float4> gBathy    : register(t1);   // NAVD88 m; R16F loads as .x
Texture2DArray<float4> gBathyRes : register(t2);   // R8: finest resident mip * 16
RWTexture2D<float> gEta  : register(u0);         // PUBLISHED: the column's surface less gTideNavd, m
RWTexture2D<float2> gFlux : register(u1);        // SIGNED face fluxes, m^3/s: x = across the
                                                 // EAST face (+east), y = across the SOUTH
                                                 // face (+south). Staggered C-grid. The substep's own:
                                                 // continuity's, and the readers'.
// M9ax: the derived currents are a Volatile GradeBank like gMv below (the dense committed
// texture is gone); the view is pinned to slice 0, so the solve writes uint3(xy, 0).
RWTexture2DArray<float4> gUv : register(u2);     // derived: u east, v north, h (m), wet
// M9h: grad(flow) -- the FIRST field in this engine whose residency was decided by the Cayley
// closure rather than by a physics policy. grad is a grade-1 operator, so nabla U is the
// geometric product of two vectors: Cl2ProductSignature(kG1, kG1) = kG0 | kG2. The scalar part
// is the divergence, the bivector part is the vorticity, and NOTHING else can appear -- the
// algebra says so before a single texel is read. Two channels, exactly the two grades.
// x = div (grade 0), y = curl (grade 2), z = COVERAGE. Coverage is what makes the composite
// honest: a null tile reads zero on every channel, so zero coverage IS "no solve here" and the
// consumer keeps whatever is underneath -- pixel by pixel, never tile by tile, exactly as
// Compose.hlsli's window overlay already does with its per-texel alpha. Without it "inside the
// window" gets mistaken for "has data", and dry land renders as a black rectangle punched
// through the regional field.
// M9j: an ARRAY, because the bank is paged -- the view is pinned to slice 0 (this window's
// page), so the solve writes uint3(xy, 0) and never has to know how many pages exist.
RWTexture2DArray<float4> gMv : register(u3);
RWTexture2D<float> gH : register(u6);            // THE STATE: the water each column holds, m (NULL: none)
RWTexture2D<float2> gVelPrev : register(u7);     // the face velocities of the substep before, m/s (ping-pong)
RWTexture2D<float> gBedC : register(u8);         // the column's bed, m NAVD88 (CsSweBedFill, a frame)
RWTexture2D<float> gSoilW : register(u9);        // C2: the water the column's soil holds, m
RWTexture2D<float2> gVel : register(u10);        // the face velocities this substep makes, m/s
// C2: per state tile (CsSweTileStats) -- x the water flags (1 a column holds water, 2 one on the tile's
// north edge, 4 south, 8 west, 16 east), y the deepest column, z the fastest (float bits), w the water
// it holds (m^3 over the cell area: the sum of h, float bits). The residency reads it back.
RWTexture2D<uint4> gTileStats : register(u11);
// C2: THE BOUNDARY'S LEDGER (CsSweLedger): x the water in through the open faces, m^3, Kahan-summed
// (y its compensation); a 1 x 1 target.
RWTexture2D<float4> gLedger : register(u12);
// C4: THE SEA IN THE COLUMNS (CsSweWaves): its energy E (as Hs^2 / 16, m^2), n = cg / c, cg (m/s), and
// the direction it travels (radians, texture frame). Two banks: a sweep reads the other's and writes
// its own; the solve reads gWave (an even number of sweeps a batch ends on it).
RWTexture2D<float4> gWave : register(u13);
RWTexture2D<float4> gWavePrev : register(u14);
// POSITIVITY (CsSweLimit): per column, the share of its outgoing fluxes its water can pay this substep,
// min(1, V / (out dt)) -- Bollermann, Chen, Kurganov and Noelle's limit on draining cells (2013). Every
// reader of a face's flux takes it times its donor's share (FluxEff).
RWTexture2D<float> gLim : register(u15);
// STAGE 1c: THE CHAINS -- the column field, the bed and the read velocity bank, every mip of each, in one
// root table (SweSolver: m_tableChain[fine parity][twin parity]); a mip's kernels read the mip above through
// them (UpLiveAt, the magnify). The registers are the chains', indexed by the mip.
// STAGE 2b: index 2 is the tail's (its h, its bed, the velocity bank its last step wrote, a sea bank unread).
RWTexture2D<float> gHChain[3] : register(u16);
RWTexture2D<float> gBedChain[3] : register(u19);
RWTexture2D<float2> gVelChain[3] : register(u22);
RWTexture2D<float4> gWaveChain[3] : register(u25);   // the sea bank each mip's solve reads (the push, the sweeps' source)

// The bed as the stack answers it at the cell (the instrument CsSweBedTrace reads this too).
float BedAt(int2 t) {
    if (any(t < 0) || t.x >= (int)gNx || t.y >= (int)gNy) return 100.0f;   // outside = wall
    // PHASE C1: the cell's centre in the domain's chart -> its point on the sphere; the standing
    // window's chain there at the cell's grain (PHASE B2 D4), the cube by the direction where the
    // window does not hold it.
    const float s = gCellX.x + float(t.x) * gCellX.y, tn = gCellX.z + float(t.y) * gCellX.w;
    const float3 dir = normalize(gAnc.xyz + s * gEast.xyz + tn * cross(gEast.xyz, gAnc.xyz));
    return HpHeightChain(gBathy, gBathyRes, dir, WindowChain(SwePointAbout(s, tn), 0u), gEast.w);
}

bool InLattice(int2 t) { return all(t >= 0) && t.x < (int)gNx && t.y < (int)gNy; }

// The cell's state tile is resident: it can hold water.
bool LiveAt(int2 t) {
    if (!InLattice(t)) return false;
    return gLive[(uint(t.y) / gEtaTileH) * gEtaTilesX + uint(t.x) / gEtaTileW] != 0u;
}

// STAGE 1c OF ONE WATER ON THE TREE (2026-10-26, docs/WATER_V2.md 14): THE MAGNIFY. A cell this mip holds no
// tile for is held by the mip above where its tile there is resident (gLiveUp): its water, its bed and its
// face velocities are the coarse cell's, magnified (nearest: the coarse cell's own value stands over its
// 2 x 2 fine cells; HIERARCHY 4.20's magnify law). The fine set's edge thus borders the coarse water, not a
// frontier; the frontier (the stateless sea) stands only where NO mip of the chain holds the cell.
// the cell above this one (stage 2d: the parent's lattice by gUp -- mip 1 of the same banks for the window)
int2 UpCell(int2 t) { return (t >> int(gUp.x)) + int2(gUp.yz); }
bool UpLiveAt(int2 t) {
    if (gMip + 1.0f >= gMips || !InLattice(t)) return false;
    const uint2 c = uint2(UpCell(t));
    return gLiveUp[(c.y / gEtaTileH) * uint(gEtaTilesXUp) + c.x / gEtaTileW] != 0u;
}
bool HoldsAt(int2 t) { return LiveAt(t) || UpLiveAt(t); }   // some mip of the chain holds the cell
float BedC(int2 t) { return LiveAt(t) ? gBedC[t] : UpLiveAt(t) ? gBedChain[uint(gMip) + 1u][UpCell(t)] : 0.0f; }
float HAt(int2 t) { return LiveAt(t) ? gH[t] : UpLiveAt(t) ? gHChain[uint(gMip) + 1u][UpCell(t)] : 0.0f; }   // NULL: no water
float SurfaceAt(int2 t) { return BedC(t) + HAt(t); }
float2 VelPrevAt(int2 t) { return LiveAt(t) ? gVelPrev[t] : UpLiveAt(t) ? gVelChain[uint(gMip) + 1u][UpCell(t)] : 0.0f; }

// STAGE 2b: THE TAIL UNDER A CELL OF THIS LATTICE -- the tail cell it lies in (the window's corner lies in gTail.yz;
// the shift is the rank less this mip), live where the tail holds a tile there. The tail's water is over mean
// sea (its reset is MSL over the bed; its tide plane is 0 m): its DEPARTURE is its column less the column mean
// sea would stand over the same bed -- the train it carries, a surge, nothing where it rests. Its velocity is the
// cell's east and south faces'.
// The tail cell BEYOND this lattice's side s at edge cell t: the one the cell lies in, stepped one cell outward
// (Berger and Oliger's ghost: the tail cells under the window are the fold's, and reading them back at the edge
// would echo the window into itself).
bool TailCellOf(int s, int2 t, out int2 c) {
    c = int2(0, 0);
    if (gTail.x < 0.0f) return false;
    c = (t >> int(gTail.x)) + int2(gTail.yz) + ((s == 0) ? int2(0, -1) : (s == 1) ? int2(0, 1) : (s == 2) ? int2(-1, 0) : int2(1, 0));
    if (any(c < 0) || c.x >= int(gTail.w) || c.y >= int(gTail2.x)) return false;
    return gLiveTail[(uint(c.y) / gEtaTileH) * uint(gTail2.y) + uint(c.x) / gEtaTileW] != 0u;
}
float TailDepAt(int s, int2 t) {
    int2 c;
    if (!TailCellOf(s, t, c)) return 0.0f;
    const float b = gBedChain[2][c];
    return (b < 0.0f) ? gHChain[2][c] + b : 0.0f;   // h - (0 - b): the column less mean sea's
}
// The tail's velocity INTO this lattice across its side s (0 N, 1 S, 2 W, 3 E), m/s: the ghost cell's faces.
float TailUAt(int s, int2 t) {
    int2 c;
    if (!TailCellOf(s, t, c)) return 0.0f;
    const float2 v = gVelChain[2][c];   // x across the east face (+ east), y across the south face (+ south)
    return (s == 0) ? v.y : (s == 1) ? -v.y : (s == 2) ? v.x : -v.x;
}

uint2 EtaTexel(uint3 id) {
    const uint t = gTileList[id.z];
    return uint2(t % gEtaTilesX, t / gEtaTilesX) * uint2(gEtaTileW, gEtaTileH) + id.xy;
}

uint2 FluxTexel(uint3 id) {
    const uint t = gTileList[id.z];
    return uint2(t % gFluxTilesX, t / gFluxTilesX) * uint2(gFluxTileW, gFluxTileH) + id.xy;
}

// ---- the boundary: the open sea's sides and the river faces ---------------------------------------

// The side whose EXTERIOR cell `e` is (-1: an interior cell). THE EDGE IS THE FRONTIER'S LAW (2026-10-27,
// one water on the tree): every edge cell with a bed is the stateless sea's, pinned to the Level along
// its side (OceanEtaAt). It holds the water that Level stands over its bed and none where the Level is
// under it (CsSweHeight), and the faces between it and the interior are Flather -- so a channel the edge
// cuts fills and drains with the sea and a dune at the edge stands over it, each instant, with no side
// "open" and none a wall: the law residency's frontier already had (ExteriorLevel: the tide plane over
// the megatexture's bed), standing at the lattice's edge too. (The river faces are tested before this:
// ExteriorLevel.) A corner cell answers its first side; SweDomain::SeaEdgeCell is the twin. Until this
// date a side was open when every cell of it lay under +1.2 m, else only its cells below mean sea were
// the sea's: a harbour mouth across 1525 of 2048 cells had no open side.
int OceanSideOf(int2 e) {
    const int s = (e.y == 0) ? 0 : (e.y == (int)gNy - 1) ? 1 : (e.x == 0) ? 2 : (e.x == (int)gNx - 1) ? 3 : -1;
    if (s < 0) return -1;
    return (BedC(e) > -9000.0f) ? s : -1;
}

// The Level along open side `s` at its exterior cell `e`, NAVD: the four samples' cubic through the
// cell's place along the side (Lagrange at a = 0, 1/3, 2/3, 1), one smooth curve.
float OceanEtaAt(int s, int2 e) {
    const float len = (s < 2) ? float(gNx) : float(gNy);
    const float a = saturate(((s < 2) ? float(e.x) + 0.5f : float(e.y) + 0.5f) / len) * 3.0f;   // 0..3
    const float4 y = gOceanEta[s];
    const float l0 = -(a - 1.0f) * (a - 2.0f) * (a - 3.0f) / 6.0f;
    const float l1 = a * (a - 2.0f) * (a - 3.0f) / 2.0f;
    const float l2 = -a * (a - 1.0f) * (a - 3.0f) / 2.0f;
    const float l3 = a * (a - 1.0f) * (a - 2.0f) / 6.0f;
    // stage 2b: the tide atlas's Level, plus what the tail carries over it where the tail holds the cell
    return gTideNavd + l0 * y.x + l1 * y.y + l2 * y.z + l3 * y.w + TailDepAt(s, e);
}

// The river face whose EXTERIOR cell `e` is (-1: none): a cell of a face's side, in its run, whose
// bed can be wet (under +2 m).
int FaceOfExterior(int2 e) {
    [loop] for (int f = 0; f < (int)gFaceCount; ++f) {
        const int side = (int)gFaceA[f].x;
        const bool onEdge = side == 0 ? e.y == 0 : side == 1 ? e.y == (int)gNy - 1 : side == 2 ? e.x == 0 : e.x == (int)gNx - 1;
        const int a = side < 2 ? e.x : e.y;
        if (onEdge && a >= (int)gFaceA[f].y && a <= (int)gFaceA[f].z && BedC(e) < 2.0f) return f;
    }
    return -1;
}

// An exterior cell's pinned level (the sea's or a river's), NAVD; false where `e` is interior.
bool ExteriorLevel(int2 e, out float etaX, out float uExt) {
    etaX = 0.0f;
    uExt = 0.0f;
    if (gFaceCount > 0.0f) {
        const int f = FaceOfExterior(e);
        if (f >= 0) {
            etaX = gTideNavd + gFaceB[f].y;
            uExt = gFaceB[f].x;
            return true;
        }
    }
    const int s = OceanSideOf(e);
    if (s >= 0) {
        etaX = OceanEtaAt(s, e);
        uExt = gOceanU + TailUAt(s, e);   // stage 2b: the prism's transport, and the tail's wave's
        return true;
    }
    // THE FRONTIER (stage 0b of one water on the tree, docs/WATER_V2.md 14; 2026-10-26): wherever the
    // resident set ends, at whatever cell, the neighbour is the stateless sea -- the tide plane over the
    // megatexture's bed (FlatherFlux reads that bed for a cell no state tile holds). The lattice's edge
    // rows keep their law above (the side's cubic, the rivers' faces); this is the same Flather, standing
    // at residency's edge instead of the box's. A NULL tile whose bed stands over the tide takes nothing,
    // as before; one under it is open sea. STAGE 1c: a cell the mip above holds is not the frontier's --
    // its coarse water is the neighbour (FaceUpdate reads it magnified).
    if (!HoldsAt(e)) {
        etaX = gTideNavd;
        uExt = 0.0f;
        return true;
    }
    return false;
}

// ---- the bed, the reset -------------------------------------------------------------------------

#ifdef SWE_TEST
// THE SCHEME'S TESTS (an instrument: SweSolver::RunSchemeTests). CsSweBedFill and CsSweReset compiled
// with SWE_TEST stand an analytic bed and an analytic first instant in a square of the lattice (gTestA:
// the case, its first cell, its side), walls everywhere else; every other kernel is the one the solver
// runs. Cases: 0 a lake at rest over a rough bed with an island standing out of it; 1 Ritter's dam
// break onto a dry flat bed; 2 Stoker's, onto a wet one; 3 a hump on the lake (the volume ledger); 4 a
// planar beach, 8 m deep at the square's west edge rising 1 in 437 eastward, a sea entering from the
// west (C4's setup); 5 a solitary wave onto a plane beach (C5's run-up).
bool TestIn(int2 t) {
    const int2 o = int2(gTestA.yz);
    return all(t >= o) && all(t < o + int(gTestA.w));
}
float TestBed(int2 t) {
    if (!TestIn(t)) return 100.0f;
    const float2 c = float2(t - int2(gTestA.yz)) + 0.5f;
    const uint kind = (uint)gTestA.x;
    if (kind == 0u || kind == 3u) {
        const float2 r = c - 0.5f * gTestA.w;
        return -4.0f + 2.5f * sin(6.2831853f * c.x / 37.0f) * cos(6.2831853f * c.y / 29.0f) +
               6.0f * exp(-dot(r, r) / (2.0f * 18.0f * 18.0f));
    }
    if (kind == 4u) return -8.0f + c.x * gDx * (8.0f / 3500.0f);
    if (kind == 5u) {   // gTestB.x: the offshore depth d; gTestB.y: the slope's cotangent; the toe at x = 0.5 side
        const float xt = 0.5f * gTestA.w * gDx;
        return -gTestB.x + max(c.x * gDx - xt, 0.0f) / gTestB.y;
    }
    return 0.0f;
}
float TestH(int2 t, float b) {
    if (!TestIn(t)) return 0.0f;
    const float2 c = float2(t - int2(gTestA.yz)) + 0.5f;
    const uint kind = (uint)gTestA.x;
    if (kind == 0u) return max(gTestB.z - b, 0.0f);
    if (kind == 3u) {
        const float2 r = c - 0.3f * gTestA.w;
        return max(gTestB.z + 0.5f * exp(-dot(r, r) / (2.0f * 12.0f * 12.0f)) - b, 0.0f);
    }
    if (kind == 4u) return max(gTestB.z - b, 0.0f);
    if (kind == 5u) {   // Synolakis's solitary wave, H = gTestB.w, its crest 0.3 of the square in
        const float d = gTestB.x, H = gTestB.w;
        const float kk = sqrt(3.0f * H / (4.0f * d * d * d));
        const float xs = (c.x - 0.3f * gTestA.w) * gDx;
        const float sech = 1.0f / cosh(min(kk * xs, 40.0f));
        return max(H * sech * sech - b, 0.0f);
    }
    return (c.x < 0.5f * gTestA.w) ? gTestB.x : gTestB.y;   // the dam at the square's middle
}
#endif

// THE COLUMN'S BED, read from the stack once a frame (SweSolver::Record) over the state's resident
// tiles. Every kernel of the substep reads it from here.
[numthreads(16, 16, 1)]
void CsSweBedFill(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const uint2 t = EtaTexel(id);
    if (t.x >= gNx || t.y >= gNy) return;
#ifdef SWE_TEST
    gBedC[t] = TestBed(int2(t));
#else
    gBedC[t] = BedAt(int2(t));
#endif
}

// THE STATELESS COLUMN (a reset, a scrub, the spin-up's first instant): each column holds the water
// the tide plane would stand over it, h = max(L - b, 0), and is still. (The open sides' and the river
// faces' exterior cells take their own levels at the first substep.)
[numthreads(16, 16, 1)]
void CsSweReset(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const uint2 t = EtaTexel(id);
    if (t.x >= gNx || t.y >= gNy) return;
    const float b = gBedC[t];
#ifdef SWE_TEST
    gH[t] = TestH(int2(t), b);
#else
    gH[t] = max(gTideNavd - b, 0.0f);
#endif
    // the soil under standing water is full; under none, empty -- across the wet tolerance
    gSoilW[t] = gSoil.x * saturate(gH[t] / gWetEps);
    gLim[t] = 1.0f;
}

// STAGE 2d: A FRESH TILE OF A LEVEL TAKES ITS PARENT'S STATE (Berger and Oliger's interpolation, by the magnify's
// nearest law): its column mean sea's over its own bed plus the parent cell's departure, its faces the parent
// cell's velocities. A tile rested instead erased, through the fold, the train its parent had carried under it
// (the 1 km stone's train reached the window at 3 % of the law's amplitude that way).
[numthreads(16, 16, 1)]
void CsSweMagnifyInit(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const uint2 t = EtaTexel(id);
    if (t.x >= gNx || t.y >= gNy) return;
    const float b = gBedC[t];
    const int2 c = UpCell(int2(t));
    const float bP = gBedChain[1][c], hP = gHChain[1][c];
    const float dep = (bP < 0.0f) ? hP + bP : 0.0f;
    gH[t] = max(gTideNavd - b + dep, 0.0f);
    gVelPrev[t] = gVelChain[1][c];
    gSoilW[t] = 0.0f;
    gLim[t] = 1.0f;
}

// STAGE 3 OF ONE WATER ON THE TREE (2026-10-27): AN IMPACT IMPOSED ON THE COLUMN FIELD, at the first batch the
// field stands past the event's zero, at every mip the field holds (the twin's own constants on its own list).
// The law's state then: its level, the sum over the table at the cell's distance (laws/Impact.hlsli ImpactTerm;
// at T = 0 the cavity itself, the lipped parabola of net volume nought; later the train linear theory has
// carried), and its radial velocity from the same sum by continuity, u_r = sum A omega / (h0 k) J1(k r)
// sin(omega T) -- nought at T = 0, the cavity at rest -- each shoaled from the basin depth to the column's own
// water by Green's law, eta by (h0 / h)^(1/4) and u by (h0 / h)^(3/4), the WKB limit of the same theory,
// unbounded (the columns are the nonlinear reader). The level goes into the column, h >= 0 (a cavity deeper
// than the water stands on the bed); the velocity onto the cell's east and south faces, radial from the impact.
// From here the law carries it: the solver's own ring, shoaling, steepening, run-up and bore, with no analytic
// reader inside the window. The distance is in the anchor's chart with the impact placed its great-circle
// distance along its bearing (SweSolver::BeginImpose): the chart's error against the sphere is second order in
// the window's size over that distance, 2 m at 2600 km for this window.
[numthreads(16, 16, 1)]
void CsSweImpose(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const int2 t = int2(EtaTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;
    const uint n = (uint)gImpactB.x;
    if (n == 0u) return;
    const float R = gAnc.w;
    const float so = gCellX.x + float(t.x) * gCellX.y, to = gCellX.z + float(t.y) * gCellX.w;   // over R
    const float s = so * R, tn = to * R;                                                          // east, north, m
    const float2 d = float2(s - gImpactA.x, tn - gImpactA.y);   // the radial direction, in the chart
    // the distance on the sphere, by the chord between the cell's direction and the impact's (stage 2b: the chart's
    // planar distance was 2 m off at 2600 km for the window; across the tail it would be a tenth)
    const float3 dir = normalize(gAnc.xyz + so * gEast.xyz + to * cross(gEast.xyz, gAnc.xyz));
    const float r = max(2.0f * R * asin(min(0.5f * length(dir - gImpactB.yzw), 1.0f)), 1.0f);
    const float T = max(gImpactA.z, 0.0f);
    const float h0 = gImpactA.w;
    float eta = 0.0f, ur = 0.0f;
    // AT ITS INSTANT THE LAW IS THE CAVITY ITSELF, closed, while the long-wave front is still within the lip (T c < R_D:
    // a minute for a 1 km stone in 4 km of water, the batch that imposes it a few seconds late); after, the sum. THE SUM
    // IS CAUSAL (2026-10-28): its truncation plants metres thousands of km out -- imposed at T = 10 s instead of 4 (a
    // batch boundary moved by the window's step) every tile of the event's patch departed at once, the ring mapped the
    // whole ocean in minutes, and the line's probes read the planted field as the train -- so beyond the front the
    // linear theory has carried, r > R_D + c T, the sum is nothing, fading over a lip's radius.
    const float c0 = sqrt(gGravity * h0), RD = kImpactLipRatio * gImpactC.x;
    if (T * c0 < RD) {
        eta = ImpactCavityAt(r, gImpactC.x, gImpactC.y);
    } else {
        const float front = RD + c0 * T;
        const float w = 1.0f - smoothstep(front, front + RD, r);
        if (w > 0.0f) {
            [loop] for (uint i = 0u; i < n; ++i) {
                const float k = asfloat(gImpactTerms[4u * i]);
                const float om = asfloat(gImpactTerms[4u * i + 1u]);
                const float A = asfloat(gImpactTerms[4u * i + 2u]);
                eta += ImpactTerm(A, k, om, r, T);
                ur += A * om / (h0 * k) * BesselJ1(k * r) * sin(om * T);
            }
            eta *= w;
            ur *= w;
        }
    }
    const float h = max(gH[t], kDepthFloorM);
    const float G = pow(h0 / h, 0.25f);
    gH[t] = max(gH[t] + eta * G, 0.0f);
    // the faces: radial from the impact at each face's centre (the east face + east, the south face + south)
    const float2 dE = float2(d.x + 0.5f * gDx, d.y), dS = float2(d.x, d.y - 0.5f * gDy);
    const float u = ur * G * G * G;
    gVelPrev[t] = float2(u * dE.x / max(length(dE), 1.0f), -u * dS.y / max(length(dS), 1.0f));
}

[numthreads(16, 16, 1)]
void CsSweClearFlux(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gFluxTileW || id.y >= gFluxTileH) return;
    gFlux[FluxTexel(id)] = 0;
    float2 u0 = 0.0f;
#ifdef SWE_TEST
    // the solitary wave (case 5) travels: its own velocity, u = c eta / (d + eta), c = sqrt(g (d + H))
    if ((uint)gTestA.x == 5u) {
        const int2 t = int2(FluxTexel(id));
        if (TestIn(t)) {
            const float d = gTestB.x, H = gTestB.w;
            const float kk = sqrt(3.0f * H / (4.0f * d * d * d));
            const float xs = (float(t.x - int(gTestA.y)) + 1.0f - 0.3f * gTestA.w) * gDx;   // the east face
            const float sech = 1.0f / cosh(min(kk * xs, 40.0f));
            const float eta = H * sech * sech;
            u0.x = sqrt(gGravity * (d + H)) * eta / (d + eta);
        }
    }
#endif
    gVelPrev[FluxTexel(id)] = u0;
    gVel[FluxTexel(id)] = u0;
    gWave[FluxTexel(id)] = 0;
    gWavePrev[FluxTexel(id)] = 0;
}

// Full-surface (non-list) clear of the derived-current texture: texels the derive pass never
// visits (NULL tiles, land) must read valid=0 so the sea falls back to the analytic jet.
[numthreads(16, 16, 1)]
void CsSweUvClear(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gNx || id.y >= gNy) return;
    gUv[uint3(id.xy, 0)] = float4(0, 0, 0, 0);
}

// ---- one substep ---------------------------------------------------------------------------

// The face between cells a and b (b east or south of a): its depth upwind of the velocity u (at rest,
// the higher surface -- the hydrostatic reconstruction: h_f = max(eta_up - max(b_a, b_b), 0)).
float FaceDepth(int2 a, int2 b, float u) {
    const float ba = BedC(a), bb = BedC(b);
    const float ea = ba + HAt(a), eb = bb + HAt(b);
    const float up = (u > 0.0f) ? ea : (u < 0.0f) ? eb : max(ea, eb);
    return max(up - max(ba, bb), 0.0f);
}

// The velocity a face held the substep before: axis `ax` (0: the east face, 1: the south face) of cell c.
float PrevU(int2 c, int ax) {
    if (!InLattice(c)) return 0.0f;
    const float2 v = VelPrevAt(c);   // stage 1c: the coarse cell's where this mip holds none
    return (ax == 0) ? v.x : v.y;
}

// The discharge per unit width across axis `ax` of cell c (the face to its east or south neighbour):
// the velocity before times the face's upwind depth now; none where either side is not live.
float FaceP(int2 c, int ax) {
    const int2 e = (ax == 0) ? int2(1, 0) : int2(0, 1);
    if (!HoldsAt(c) || !HoldsAt(c + e)) return 0.0f;
    const float u = PrevU(c, ax);
    return u * FaceDepth(c, c + e, u);
}

// The sea's radiation stress at cell c over the water's density, (S_xx, S_yy, S_xy) / rho in the texture
// frame (m^3/s^2): S_xx = E (n (1 + cos^2) - 1/2), S_yy = E (n (1 + sin^2) - 1/2), S_xy = E n sin cos,
// E = g Hs^2 / 16 per density (Longuet-Higgins and Stewart 1964). None outside the lattice.
// stage 1c: the sea of a cell the mip above holds is its coarse cell's -- without it the radiation stress fell to
// nothing across the fine set's edge, a push that rang the inlet a metre in the first gate run
float4 WaveAt(int2 c) { return LiveAt(c) ? gWave[c] : UpLiveAt(c) ? gWaveChain[uint(gMip) + 1u][UpCell(c)] : 0.0f; }
float3 WaveS(int2 c) {
    if (!InLattice(c)) return 0.0f;
    const float4 w = WaveAt(c);
    const float cx = cos(w.w), cy = sin(w.w);
    const float e = gGravity * w.x;
    return e * float3(w.y * (1.0f + cx * cx) - 0.5f, w.y * (1.0f + cy * cy) - 0.5f, w.y * cx * cy);
}

// ONE FACE, ONE SUBSTEP (Stelling & Duinmeijer 2003): the face between cell t and t + d (d = (1, 0)
// east, (0, 1) south), of length faceLen, its gradient over span metres; u its velocity the substep
// before (+ along d). Returns the new velocity and the flux it carries (m^3/s), the water's upwind depth
// times the velocity times the length.
// M6r, the per-axis metric: the grid is equiangular, so a face is faceLen metres LONG and its
// gradient acts over span metres -- (faceLen, span) = (dy, dx) for east faces, (dx, dy) for
// south faces.
//
//   du/dt + u du/dx + v du/dy = -g d(eta)/dx - C_d |u| u / h
//
// THE ADVECTION, the scheme's point: where the flow DECELERATES along itself (an expansion: a bore, a
// hydraulic jump, a front slowing onto the flats) the term conserves MOMENTUM -- the upstream cell's
// mean discharge over the face's mean depth, times the velocity's difference; where it ACCELERATES (a
// contraction: over a bar's crest, through the inlet's throat, down a rarefaction onto a dry bed) it
// conserves the ENERGY HEAD -- the mean of the two velocities times their difference, d(u^2/2)/dx. Both
// vanish with the difference, so the choice between them is continuous. Across the face, the transverse
// velocity (its four faces' mean) carries u from upstream. All first order, upwind.
// THE PRESSURE: the hydrostatic reconstruction of each side's depth at the sill (Audusse et al. 2004),
// g (hL* - hR*) / span: the surface's slope where both are wet, a column's push onto a dry bed where one
// is not, nothing where neither can stand over the sill (a lake at rest stays at rest).
float2 FaceUpdate(float u, int2 t, int2 d, float faceLen, float span) {
    const int2 n = t + d;
    // a NULL tile holds no water and takes none; a cell the mip above holds is read magnified (stage 1c), and
    // a face neither of whose cells this mip holds is the mip above's, not this one's
    if (!HoldsAt(t) || !HoldsAt(n) || !(LiveAt(t) || LiveAt(n))) return 0.0f;
    const float b = BedC(t), bn = BedC(n);
    const float h = HAt(t), hn = HAt(n);
    const float sill = max(b, bn);
    const float hL = max(b + h - sill, 0.0f), hR = max(bn + hn - sill, 0.0f);
    if (hL <= 0.0f && hR <= 0.0f) return 0.0f;   // no water can stand over the sill

    const int ax = (d.x != 0) ? 0 : 1;
    const int2 dn = int2(d.y, d.x);              // the transverse axis (south for an east face, east for a south face)
    const float2 cm = CellM(t);                  // the chart's metric at the cell (stage 2b)
    const float spanT = (ax == 0) ? cm.y : cm.x;   // the transverse cell size
    const float hBar = max(0.5f * (h + hn), gWetEps);
    float adv = 0.0f;
    if (u > 0.0f) {
        const float uB = PrevU(t - d, ax);                       // the face upstream (behind)
        if (uB >= 0.0f && u > uB) adv = 0.5f * (u + uB) * (u - uB) / span;   // accelerating: the energy head
        else adv = 0.5f * (FaceP(t - d, ax) + u * hL) / hBar * (u - uB) / span;   // momentum
    } else if (u < 0.0f) {
        const float uA = PrevU(n, ax);                           // the face upstream (ahead)
        if (uA <= 0.0f && u < uA) adv = 0.5f * (u + uA) * (uA - u) / span;
        else adv = 0.5f * (u * hR + FaceP(n, ax)) / hBar * (uA - u) / span;
    }
    {
        const float vT = 0.25f * (PrevU(t, 1 - ax) + PrevU(n, 1 - ax) + PrevU(t - dn, 1 - ax) + PrevU(n - dn, 1 - ax));
        if (vT > 0.0f) adv += vT * (u - PrevU(t - dn, ax)) / spanT;
        else if (vT < 0.0f) adv += vT * (PrevU(t + dn, ax) - u) / spanT;
    }
    // THE WAVES PUSH (C4): F = -div S, S the radiation stress of the sea the column carries (CsSweWaves;
    // Longuet-Higgins and Stewart 1964) -- across the face, the normal stress's difference between the two
    // cells, and the shear's across the face's two ends. Per unit mass of the face's water, F / (rho h).
    // Setup, a bar going awash, overwash and run-up come out of it; no setup law is written.
    if (gSea.w > 0.0f) {
        const float3 St = WaveS(t), Sn = WaveS(n);
        const float sxyP = 0.25f * (St.z + Sn.z + WaveS(t + dn).z + WaveS(n + dn).z);
        const float sxyM = 0.25f * (St.z + Sn.z + WaveS(t - dn).z + WaveS(n - dn).z);
        const float F = -(((ax == 0) ? (Sn.x - St.x) : (Sn.y - St.y)) / span + (sxyP - sxyM) / spanT);
        adv -= F / hBar;
    }
    float uN = u + gDt * (gGravity * (hL - hR) / span - adv);
    // the water the face carries: upwind of the new velocity
    const float hUp = (uN > 0.0f) ? hL : hR;
    if (hUp <= 0.0f) return 0.0f;   // its upstream side holds none over the sill
    // Implicit quadratic bottom drag (gCd 0.0015: sandy-estuary range -- 0.0025 was fine over the 5 km
    // mouth window; across the M6d wide window's 15 km reach the extra friction accumulated ~40 min of
    // phase lag); a whisper of linear damping for numerical hygiene.
    uN *= gDamp / (1.0f + gDt * gCd * abs(uN) / max(hUp, 0.3f));
    // the flux; positivity is the donor's, all its faces together (CsSweLimit, FluxEff)
    return float2(uN, uN * hUp * faceLen);
}

// THE FACE TRACE (an instrument: SweSolver::TraceFace). CsSweFlux compiled with SWE_FACE_TRACE
// records the first river face's Flather terms as it makes them -- the kernel's own expressions, not
// a copy -- and writes them for that face's exterior cells instead of the flux: the traced state
// does not advance. Without the define the hooks are nothing and the kernel is the one the solver runs.
RWTexture2D<float4> gFaceTrace : register(u5);   // 4 texels a cell along the face's side
#ifdef SWE_FACE_TRACE
static float4 gWtFace = 0.0f;   // etaX, etaI, sill, hf
static float4 gWtLaw = 0.0f;    // u_ext, the radiation term, ub, q before the clamp
static float4 gWtClamp = 0.0f;  // q after the clamp, the clamp's two bounds, 1 = the cell is pinned
static float gWtHi = 0.0f;      // the first interior cell's depth
static int gWtAlong = -1;       // the cell along the side the thread traced
static float4 gWtCells = 0.0f;  // the exterior cell's surface less the tide plane, the interior depth, both beds
#define SWE_FACE_LAW(f, rad, ub, qRaw, lo, hi) if ((f) == 0) { gWtLaw = float4(gFaceB[0].x, (rad), (ub), (qRaw)); gWtClamp.yz = float2((lo), (hi)); }
#define SWE_FACE_ROW(f, a, etaX, etaI, sill, hf, hI, q) if ((f) == 0) { gWtFace = float4((etaX), (etaI), (sill), (hf)); gWtClamp.xw = float2((q), 1.0f); gWtHi = (hI); gWtAlong = (a); gWtCells = float4(SurfaceAt(e) - gTideNavd, (hI), BedC(e), BedC(i)); }
#else
#define SWE_FACE_LAW(f, rad, ub, qRaw, lo, hi)
#define SWE_FACE_ROW(f, a, etaX, etaI, sill, hf, hI, q)
#endif

// M6r: FLATHER, the open boundary RADIATES. The exterior cell rides its level (the sea's Level, the
// river tide) as pinned data; the flux across the face to its interior neighbour `i` is
//     u_b = u_ext + sqrt(g / h) (eta_ext - eta_int),
// so an interior surplus flows OUT at the gravity-wave speed instead of reflecting off a relaxed
// strip. u_ext is the PRESCRIBED transport over the live section: radiation alone cannot carry a
// prism -- without u_ext the through-flow throttles behind the standing departure it needs to sustain
// itself (measured on the Merrimack: the gap current lost a third of its amplitude). L: the face's
// length; f: the river face for the trace (-1: the sea). Returns the flux INTO the domain and the
// face's depth.
float2 FlatherFlux(int f, float etaX, float uExt, int2 e, int2 i, float L) {
    const float hI = HAt(i);
    const float etaI = SurfaceAt(i);
    // the exterior's bed: the cached one where a state tile holds it, the megatexture's at the frontier
    const float bedE = HoldsAt(e) ? BedC(e) : BedAt(e);
    const float sill = max(bedE, BedC(i));
    const float hf = max(max(etaX, etaI) - sill, 0.0f);
    float q = 0.0f;
    if (hf > 0.05f) {
        const float rad = sqrt(gGravity / hf) * (etaX - etaI);
        const float ub = uExt + rad;
        const float2 cm = CellM(i);
        const float V = cm.x * cm.y / gDt;               // positivity, as everywhere (the cell's true area)
        q = clamp(hf * L * ub, -0.25f * hI * V, 0.25f * hf * V);
        SWE_FACE_LAW(f, rad, ub, hf * L * ub, -0.25f * hI * V, 0.25f * hf * V);
    }
    SWE_FACE_ROW(f, (((int)gFaceA[max(f, 0)].x < 2) ? e.x : e.y), etaX, etaI, sill, hf, hI, q);
    return float2(q, hf);
}

// A boundary face's (velocity, flux) along +d from Flather's inward flux: `sgn` +1 when the exterior is
// the face's first cell (W, N sides), -1 when its second (E, S).
float2 BoundaryFace(float2 qh, float sgn, float L) {
    const float q = sgn * qh.x;
    return float2((qh.y > 0.05f) ? q / (qh.y * L) : 0.0f, q);
}

[numthreads(16, 16, 1)]
void CsSweFlux(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gFluxTileW || id.y >= gFluxTileH) return;
    const int2 t = int2(FluxTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;

    const float2 u = gVelPrev[t];
    const float2 cm = CellM(t);   // stage 2b: the cell's true size from the chart's metric
    float2 fe = FaceUpdate(u.x, t, int2(1, 0), cm.y, cm.x);   // east face (dy long, dx span): (u, q)
    float2 fs = FaceUpdate(u.y, t, int2(0, 1), cm.x, cm.y);   // south face (texture +y)

    // THE BOUNDARY (FlatherFlux): a thread owns its cell's east and south faces. A face with an
    // exterior cell on either side is the boundary's: between an exterior and an interior cell it is
    // Flather (+ east and + south: inward from a W or N exterior, outward toward an E or S one);
    // between two exterior cells (along a side) it carries nothing -- both are pinned.
    float etaT, uT, etaE, uE, etaS, uS;
    const bool xT = ExteriorLevel(t, etaT, uT);
    const bool xE = InLattice(t + int2(1, 0)) && ExteriorLevel(t + int2(1, 0), etaE, uE);
    const bool xS = InLattice(t + int2(0, 1)) && ExteriorLevel(t + int2(0, 1), etaS, uS);
    const int fT = (gFaceCount > 0.0f) ? FaceOfExterior(t) : -1;
    if (xT && xE) fe = 0.0f;
    else if (xT && LiveAt(t + int2(1, 0))) fe = BoundaryFace(FlatherFlux(fT, etaT, uT, t, t + int2(1, 0), cm.y), 1.0f, cm.y);
    else if (xE) fe = BoundaryFace(FlatherFlux((gFaceCount > 0.0f) ? FaceOfExterior(t + int2(1, 0)) : -1, etaE, uE, t + int2(1, 0), t, cm.y), -1.0f, cm.y);
    if (xT && xS) fs = 0.0f;
    else if (xT && LiveAt(t + int2(0, 1))) fs = BoundaryFace(FlatherFlux(fT, etaT, uT, t, t + int2(0, 1), cm.x), 1.0f, cm.x);
    else if (xS) fs = BoundaryFace(FlatherFlux((gFaceCount > 0.0f) ? FaceOfExterior(t + int2(0, 1)) : -1, etaS, uS, t + int2(0, 1), t, cm.x), -1.0f, cm.x);
#ifdef SWE_FACE_TRACE
    if (gWtAlong >= 0) {
        gFaceTrace[uint2(0, gWtAlong)] = gWtFace;
        gFaceTrace[uint2(1, gWtAlong)] = gWtLaw;
        gFaceTrace[uint2(2, gWtAlong)] = gWtClamp;
        gFaceTrace[uint2(3, gWtAlong)] = gWtCells;
    }
    return;
#endif
    gVel[t] = float2(fe.x, fs.x);
    gFlux[t] = float2(fe.y, fs.y);
}

// The share of a column's outgoing fluxes its water pays this substep (gLim); 1 off the lattice.
// stage 1c: a cell this mip does not hold pays the whole share (the coarse donor's positivity is its own step's)
float LimAt(int2 c) { return LiveAt(c) ? gLim[c] : 1.0f; }

// A face's flux as the water moves it: the face's own times its donor's share. axis 0: the east face of
// c (to c + (1, 0)); 1: the south face (to c + (0, 1)).
float FluxEff(int2 c, int ax) {
    if (!InLattice(c)) return 0.0f;
    const float q = (ax == 0) ? gFlux[c].x : gFlux[c].y;
    const int2 n = c + ((ax == 0) ? int2(1, 0) : int2(0, 1));
    return q * ((q > 0.0f) ? LimAt(c) : LimAt(n));
}

// POSITIVITY, ALL FACES TOGETHER (Bollermann, Chen, Kurganov and Noelle 2013): each column sums what its
// faces would take out of it this substep; where that is more than it holds, every outgoing face is
// scaled by the same share, so the column empties exactly and none goes below its bed. A column whose
// faces take less is not touched (the share is 1). The exterior cells are the boundary's water: 1.
[numthreads(16, 16, 1)]
void CsSweLimit(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const int2 t = int2(EtaTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;
    float etaX, uX;
    if (ExteriorLevel(t, etaX, uX)) {
        gLim[t] = 1.0f;
        return;
    }
    const float2 qc = gFlux[t];
    const float qw = (t.x > 0) ? gFlux[int2(t.x - 1, t.y)].x : 0.0f;
    const float qn = (t.y > 0) ? gFlux[int2(t.x, t.y - 1)].y : 0.0f;
    const float drain = max(qc.x, 0.0f) + max(qc.y, 0.0f) + max(-qw, 0.0f) + max(-qn, 0.0f);
    const float2 cm = CellM(t);
    const float V = gH[t] * cm.x * cm.y;   // the cell's true area (stage 2b)
    // a hair under the water it holds: an exact share lands the column a rounding either side of empty,
    // and the side below is lifted back to 0 by h >= 0 -- water from nowhere, every draining column, every
    // substep (the ledger saw 8e-5 of the prism)
    const float Vh = V * (1.0f - 1.0e-6f);
    gLim[t] = (drain * gDt > Vh) ? Vh / (drain * gDt) : 1.0f;
}

[numthreads(16, 16, 1)]
void CsSweHeight(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const int2 t = int2(EtaTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;

    // The exterior cells (the sea's sides, the river faces) hold their levels' water, pinned: the
    // domain ends here but the DATA keeps going, and the truncation becomes a boundary condition.
    float etaX, uX;
    if (ExteriorLevel(t, etaX, uX)) {
        gH[t] = max(etaX - BedC(t), 0.0f);
        return;
    }
    // Mass: the divergence of the face fluxes as the water moves them (in = the west and north faces,
    // out = my own; each its donor's share: FluxEff).
    const float qe = FluxEff(t, 0), qs = FluxEff(t, 1);
    const float qw = FluxEff(t - int2(1, 0), 0);   // across my west face (+ = into me)
    const float qn = FluxEff(t - int2(0, 1), 1);   // across my north face (+ = into me)
    const float2 cm = CellM(t);
    float h = gH[t] + gDt * ((qw - qe) + (qn - qs)) / (cm.x * cm.y);   // over the cell's true area (stage 2b)
    // THE LAND SOAKS (C2; Mark: "water gets sponged up by dry cells but still spreads out and goes down
    // rivers; eventually soil saturation"). The column's soil holds water of its own, W <= C (gSoil.x),
    // and takes standing water at Green and Ampt's rate, K (1 + S / W) -- K the saturated conductivity
    // (gSoil.y), S the wetting front's suction times the moisture deficit (gSoil.z) -- falling
    // continuously to nothing as the soil fills (1 - W / C). What it does not take keeps flowing, over
    // the land and down the channels. One stated soil for now; the soil's own measurements (SoilGrids2,
    // HiHydroSoil v2) later, and then the soil's drainage, so a second surge runs off ground the first
    // one wet.
    if (gSoil.w > 0.0f) {
        const float W = gSoilW[t];
        const float f = gSoil.y * (1.0f + gSoil.z / max(W, 1e-3f)) * saturate(1.0f - W / gSoil.x);
        const float take = clamp(min(f * gDt, h), 0.0f, max(gSoil.x - W, 0.0f));
        h -= take;
        gSoilW[t] = W + take;
    }
    // h >= 0 is the only bound (positivity keeps it; this takes the last float's rounding).
    gH[t] = max(h, 0.0f);
}

// ---- published for the readers (rendering, Doppler, the hull, validation probes) -----------------

[numthreads(16, 16, 1)]
void CsSweDerive(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const int2 t = int2(EtaTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;

    const float h = gH[t];
    // The column's surface, b + h (a dry column's is its bed), less the frame's tide plane.
    gEta[t] = gBedC[t] + h - gTideNavd;
    if (h <= gWetEps) {
        gUv[uint3(t, 0)] = float4(0, 0, h, 0);
        return;
    }
    // Cell-centred velocity: the mean of the two faces in each axis, each over its OWN face
    // area (M6r: east faces are dy long, south faces dx -- the isotropic gDx overstated v
    // by 35%).
    const float2 qc = float2(FluxEff(t, 0), FluxEff(t, 1));
    const float qw = FluxEff(t - int2(1, 0), 0);
    const float qn = FluxEff(t - int2(0, 1), 1);
    const float2 cm = CellM(t);
    const float u = 0.5f * (qc.x + qw) / (h * cm.y);
    const float v = -0.5f * (qc.y + qn) / (h * cm.x);   // texture +y is south; world v is north
    gUv[uint3(t, 0)] = float4(u, v, h, 1.0f);
}

// ---- C2: the water's own residency, and the boundary's ledger -------------------------------------

// THE WATER, TILE BY TILE: one group a resident state tile (a thread 8 x 8 of its cells) -- which of its
// columns hold water and which of its edges water touches (the residency's ring), its deepest and
// fastest column (the step), and the water it holds (the ledger of a tile the residency releases).
// A LEVEL's fourth word is instead THE RANGE OF ITS DEPARTURE WITHIN A PARENT CELL, the greatest over the
// tile (2026-10-28, the release behind a train): what the parent's one value cannot hold of the field. The
// residency keeps a tile while that range is the departure's own scale (the field is the level's to resolve)
// and releases it once the parent holds the field at the resolution the level was made for (SweSolver::
// UpdateLevelResidency). A thread's block is 8 x 8 cells: the range is taken over each parent cell within
// it (per <= 8), else over the block (a lower bound for a level four or more ranks under its parent).
groupshared uint gsFlags, gsHmax, gsUmax, gsRange;
groupshared float gsVol[256];
[numthreads(16, 16, 1)]
void CsSweTileStats(uint3 gid : SV_GroupID, uint3 tid : SV_GroupThreadID, uint gi : SV_GroupIndex) {
    if (gi == 0u) {
        gsFlags = 0u;
        gsHmax = 0u;
        gsUmax = 0u;
        gsRange = 0u;
    }
    GroupMemoryBarrierWithGroupSync();
    const uint tile = gTileList[gid.z];
    const uint2 org = uint2(tile % gEtaTilesX, tile / gEtaTilesX) * uint2(gEtaTileW, gEtaTileH);
    const uint sx = gEtaTileW / 16u, sy = gEtaTileH / 16u;
    uint flags = 0u;
    float hm = 0.0f, um = 0.0f, vol = 0.0f;
    for (uint j = 0u; j < sy; ++j) {
        for (uint i = 0u; i < sx; ++i) {
            const uint2 c = uint2(tid.x * sx + i, tid.y * sy + j);
            const uint2 t = org + c;
            if (t.x >= gNx || t.y >= gNy) continue;
            const float h = gH[t];
            vol += h;
            // stage 2d: a level of mean sea's gauge (gTail2.z = 1) counts a column live where it departs from rest,
            // |h + b| over the wet tolerance -- the train, a surge -- so a front demands the tiles ahead of itself
            // and the sea it has left is released; the window counts a column that holds water
            const float b = gBedC[t];
            const float sig = (gTail2.z > 0.5f) ? ((b < 0.0f) ? abs(h + b) : 0.0f) : h;   // no sea over land: nothing departs there
            if (sig <= gWetEps) continue;
            flags |= 1u;
            if (c.y == 0u) flags |= 2u;
            if (c.y == gEtaTileH - 1u) flags |= 4u;
            if (c.x == 0u) flags |= 8u;
            if (c.x == gEtaTileW - 1u) flags |= 16u;
            hm = max(hm, (gTail2.z > 0.5f) ? sig : h);   // a level's statistic is its greatest departure
            um = max(um, length(gUv[uint3(t, 0)].xy));
        }
    }
    float rm = 0.0f;
    if (gTail2.z > 0.5f) {   // a level: the departure's range within each parent cell of this thread's block
        const uint per = (gTail.x >= 0.0f) ? (1u << uint(gTail.x)) : 1u;
        const uint cs = min(per, min(sx, sy));
        for (uint by = 0u; by < sy; by += cs) {
            for (uint bx = 0u; bx < sx; bx += cs) {
                float lo = 1.0e30f, hi = -1.0e30f;
                bool any = false;
                for (uint j = 0u; j < cs; ++j) {
                    for (uint i = 0u; i < cs; ++i) {
                        const uint2 t = org + uint2(tid.x * sx + bx + i, tid.y * sy + by + j);
                        if (t.x >= gNx || t.y >= gNy) continue;
                        const float b = gBedC[t];
                        if (b >= 0.0f) continue;   // land holds no departure
                        const float d = gH[t] + b;
                        lo = min(lo, d);
                        hi = max(hi, d);
                        any = true;
                    }
                }
                if (any) rm = max(rm, hi - lo);
            }
        }
    }
    InterlockedOr(gsFlags, flags);
    InterlockedMax(gsHmax, asuint(hm));
    InterlockedMax(gsUmax, asuint(um));
    InterlockedMax(gsRange, asuint(rm));
    gsVol[gi] = vol;
    GroupMemoryBarrierWithGroupSync();
    for (uint s = 128u; s > 0u; s >>= 1u) {
        if (gi < s) gsVol[gi] += gsVol[gi + s];
        GroupMemoryBarrierWithGroupSync();
    }
    if (gi == 0u) gTileStats[uint2(tile % gEtaTilesX, tile / gEtaTilesX)] = uint4(gsFlags, gsHmax, gsUmax, (gTail2.z > 0.5f) ? gsRange : asuint(gsVol[0]));
}

// THE BOUNDARY'S LEDGER: the water in through every boundary face this substep -- the list in t0 (its
// count, then an edge cell a word: x, y << 12), each cell the kernel's own exterior test keeps
// (ExteriorLevel) giving every face it shares with a live interior cell (toward the inside, and along
// the edge where a sea face's run meets an interior edge cell) -- times the step, added to gLedger with
// Kahan's compensation. One group; run after each flux kernel while the ledger is asked.
groupshared float gsIn[256];
bool LedgerInterior(int2 c) {
    float etaX, uX;
    return LiveAt(c) && !ExteriorLevel(c, etaX, uX);
}
[numthreads(256, 1, 1)]
void CsSweLedger(uint gi : SV_GroupIndex) {
    const uint count = gTileList[0];
    float s = 0.0f;
    for (uint k = gi; k < count; k += 256u) {
        const uint w = gTileList[1u + k];
        const int2 e = int2(w & 0xFFFu, (w >> 12) & 0xFFFu);
        float etaX, uX;
        if (!LiveAt(e) || !ExteriorLevel(e, etaX, uX)) continue;
        // the flux out of e into each interior neighbour: + its east and south faces, - the faces its
        // west and north neighbours own
        if (LedgerInterior(e + int2(1, 0))) s += FluxEff(e, 0);
        if (LedgerInterior(e + int2(0, 1))) s += FluxEff(e, 1);
        if (LedgerInterior(e - int2(1, 0))) s -= FluxEff(e - int2(1, 0), 0);
        if (LedgerInterior(e - int2(0, 1))) s -= FluxEff(e - int2(0, 1), 1);
    }
    gsIn[gi] = s;
    GroupMemoryBarrierWithGroupSync();
    for (uint r = 128u; r > 0u; r >>= 1u) {
        if (gi < r) gsIn[gi] += gsIn[gi + r];
        GroupMemoryBarrierWithGroupSync();
    }
    if (gi == 0u) {
        float4 L = gLedger[uint2(0, 0)];
        const float y = gsIn[0] * gDt - L.y;
        const float t = L.x + y;
        L.y = (t - L.x) - y;
        L.x = t;
        gLedger[uint2(0, 0)] = L;
    }
}

// ---- C4: the sea in the columns ----------------------------------------------------------------

// THE SEA CARRIED SHOREWARD (a sweep; a few a batch -- SweSolver::Record): the steady balance of the
// sea's energy flux along its rays, d(E cg)/ds = -D, marched one cell a sweep from the open sea (the
// exterior cells: the deep-water sea shoaled to their depth) over the columns that hold water. Each
// column takes the flux arriving from one cell up-ray (bilinear over the sweep before), over its own
// group speed (SHOALING); its direction is the open sea's turned at the local contour by Snell's law
// (REFRACTION, the bed's slope as the contour's normal, the open sea's direction where the bed is flat);
// dry ground passes none on (SHELTER: the inlet's lee, the river behind the bar); and no wave stands
// taller than the bank's own law lets it (BREAKING: Battjes and Janssen's clipped Rayleigh sea,
// WaterTerms.h BreakHmax / BreakFactor, the same law the drawn sea is broken by). The dissipation D is
// what the breaking takes; its force is the radiation stress's divergence (FaceUpdate).
// The wavenumber: Guo's explicit approximation of the dispersion relation (laws/Dispersion.hlsli
// ShoaledKh, the one text the bank and the hull read it from), from the deep-water k = omega^2 / g.
float WaveK(float omega, float d) {
    return ShoaledKh(omega * omega / gGravity, d) / d;
}

float4 WavePrevAt(float2 p) {   // bilinear over the sweep before, at a point in cell units (centres at +0.5)
    const float2 tf = p - 0.5f;
    const int2 t0 = int2(floor(tf));
    const float2 fr = tf - float2(t0);
    float4 acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 c = t0 + int2(k & 1, k >> 1);
        const float w = ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y);
        // stage 1c: up-ray of the fine set's edge the coarse cell's sea is the source (the twin sweeps its own
        // lattice from the open sides; the fine set has no side of its own)
        if (InLattice(c)) acc += w * (LiveAt(c) ? gWavePrev[c] : UpLiveAt(c) ? gWaveChain[uint(gMip) + 1u][UpCell(c)] : 0.0f);
    }
    return acc;
}

#ifdef SWE_TEST
bool WaveSource(int2 t) { return TestIn(t) && t.x < int(gTestA.y) + 2; }   // the test square's first columns
#else
bool WaveSource(int2 t) {
    float etaX, uX;
    return OceanSideOf(t) >= 0 && ExteriorLevel(t, etaX, uX) && !(gFaceCount > 0.0f && FaceOfExterior(t) >= 0);
}
#endif

[numthreads(16, 16, 1)]
void CsSweWaves(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const int2 t = int2(EtaTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;
    const float h = gH[t];
    if (gSea.w <= 0.0f || h <= gWetEps) {   // no sea, or no water to carry it
        gWave[t] = 0.0f;
        return;
    }
    const float omega = gSea.y;
    const float k = WaveK(omega, h);
    const float kd = k * h;
    const float n = 0.5f * (1.0f + 2.0f * kd / sinh(min(2.0f * kd, 80.0f)));
    const float c = omega / k;
    const float cg = n * c;
    // the direction: the open sea's, turned at the local contour (Snell)
    const float2 u0 = gSea2.xy;
    const float2 gb = float2((BedC(t + int2(1, 0)) - BedC(t - int2(1, 0))) / (2.0f * gDx),
                             (BedC(t + int2(0, 1)) - BedC(t - int2(0, 1))) / (2.0f * gDy));
    const float gl = length(gb);
    float2 dir = u0;
    if (gl > 1e-6f) {
        const float2 nr = gb / gl;                 // up the bed: toward the shore
        const float2 tg = float2(-nr.y, nr.x);
        const float c0 = gGravity / omega;         // deep water's phase speed
        const float sinA = clamp(dot(u0, tg) * c / c0, -1.0f, 1.0f);
        const float cosA = sqrt(1.0f - sinA * sinA) * ((dot(u0, nr) >= 0.0f) ? 1.0f : -1.0f);
        const float2 snell = cosA * nr + sinA * tg;
        dir = normalize(lerp(u0, snell, gl / (gl + 2e-3f)));   // a flat bed keeps the open sea's direction
    }
    // the energy arriving: the flux from one cell up-ray, less what breaking dissipates over that step
    float E;
    if (WaveSource(t)) {
        E = gSea.z * (0.5f * gGravity / omega) / cg;   // the deep-water flux, shoaled to this depth
    } else {
        const float s = min(gDx, gDy);
        const float4 up = WavePrevAt(float2(t) + 0.5f - float2(dir.x * s / gDx, dir.y * s / gDy));
        // up = (E, n, cg, angle) bilinear: the flux is the bilinear E cg, read from its two parts
        const float F = up.x * up.z;
        // BREAKING AS A DISSIPATION per metre of the ray (Baldock et al. 1998, Battjes and Janssen's model
        // made explicit): D = (alpha / 4) f_p exp(-(Hb / Hrms)^2) (Hb^2 + Hrms^2), per rho g, alpha = 1,
        // f_p the sea's peak frequency, Hb the height the water lets a wave stand to -- the bank's law
        // (WaterTerms.h BreakHmax: Battjes and Janssen's Miche-bounded gamma h, a limit on Hs, read for one
        // wave as sqrt(2) of it: Hs = sqrt(2) Hrms) -- Hrms the arriving sea's.
        const float hrms2 = 8.0f * F / cg;    // Hrms^2 = 8 E with E = Hs^2 / 16
        const float hb = 1.41421356f * BreakHmax(k, h);   // laws/Breaking.hlsli: the bank's own law
        const float D = 0.25f * (omega / 6.2831853f) * exp(-hb * hb / max(hrms2, 1e-8f)) * (hb * hb + hrms2);
        E = max(F - D * s, 0.0f) / cg;   // F and D both per rho g: the flux of Hrms^2 / 8
    }
    const float L2 = 1.0f;
    gWave[t] = float4(E * L2, n, cg, atan2(dir.y, dir.x));
}

// C2: a freshly mapped tile holds nothing (its pool memory is whatever it last held).
[numthreads(16, 16, 1)]
void CsSweClearTile(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const uint2 t = EtaTexel(id);
    gH[t] = 0.0f;
    gSoilW[t] = 0.0f;
    gEta[t] = 0.0f;
    gLim[t] = 1.0f;
}

// STAGE 1b OF ONE WATER ON THE TREE (2026-10-26, docs/WATER_V2.md 14): THE FOLD. A coarse cell under four
// resident fine cells IS their fold -- the water their mean (the column's water per area), the bed their
// mean (so the level's mean follows), a face's velocity the mean of the two fine faces that lie on it (the
// flux-weighted fold, by the faces' water, is 1c's). Dispatched over the coarse tiles whose four children
// are resident, with the twin's constants and the FOLD TABLE: gH, gBedC and gVelPrev are the fine banks at
// mip 0 (read), gEta, gSoilW and gFlux the water, the bed and the twin's read velocity bank at mip 1
// (written) -- the registers are the kernel's names, the views the fold's. The twin's own evolution
// under the fine set is discarded each batch; the fine solve is the truth there, the twin carries the
// rest and reads the folded cells at its edge.
[numthreads(16, 16, 1)]
void CsSweFold(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const uint2 c = EtaTexel(id);
    if (c.x >= gNx || c.y >= gNy) return;
    const int2 f = int2(c) * 2;
    const int2 fE = f + int2(1, 0), fS = f + int2(0, 1), fSE = f + int2(1, 1);
    const float h = 0.25f * (gH[f] + gH[fE] + gH[fS] + gH[fSE]);
    const float b = 0.25f * (gBedC[f] + gBedC[fE] + gBedC[fS] + gBedC[fSE]);
    // the coarse east face is the right column's two east faces; its south face the bottom row's two
    const float2 v = float2(0.5f * (gVelPrev[fE].x + gVelPrev[fSE].x), 0.5f * (gVelPrev[fS].y + gVelPrev[fSE].y));
    gEta[c] = h;
    gSoilW[c] = b;
    gFlux[c] = v;
}

// STAGE 2c/2d: THE FOLD INTO THE PARENT (SweSolver::RecordLevel, before each parent step; the child's constants and
// tile list bound, the fold's table: the child's water, bed and newest velocities in the read slots, the parent's
// water and read velocities in the written ones, the parent's bed at gSoilW). One thread a parent cell under each
// of the child's resident tiles (id.z the tile): the parent's column there becomes mean sea's over ITS bed plus
// the mean, over the child's wet cells, of the child's surface less its tide plane (0 m for a level: the departure
// the gauges share); its faces take the child's wet cells' mean velocities. Nothing where the child holds no
// water or the parent no sea.
[numthreads(16, 16, 1)]
void CsSweFoldLevel(uint3 id : SV_DispatchThreadID) {
    const uint per = 1u << uint(gTail.x);                 // child cells a parent cell
    const uint across = max(gEtaTileW / per, 1u);         // parent cells across a child tile
    if (id.x >= across || id.y >= across) return;
    const uint t = gTileList[id.z];
    const uint2 o = uint2(t % gEtaTilesX, t / gEtaTilesX) * uint2(gEtaTileW, gEtaTileH) + id.xy * per;   // the block's first child cell
    if (o.x >= gNx || o.y >= gNy) return;
    const int2 cellT = int2(o >> uint(gTail.x)) + int2(gTail.yz);
    float sh = 0.0f, su = 0.0f, sv = 0.0f, n = 0.0f;
    for (uint j = 0u; j < per; ++j) {
        for (uint i = 0u; i < per; ++i) {
            const uint2 c = o + uint2(i, j);
            if (c.x >= gNx || c.y >= gNy) continue;
            const float h = gH[c];
            if (h <= gWetEps) continue;
            sh += gBedC[c] + h - gTideNavd;
            su += gVelPrev[c].x;
            sv += gVelPrev[c].y;
            n += 1.0f;
        }
    }
    if (n <= 0.0f) return;
    const float bT = gSoilW[cellT];   // the parent's bed
    if (bT >= 0.0f) return;           // the parent holds no sea here
    gEta[cellT] = max(-bT + sh / n, 0.0f);
    gFlux[cellT] = float2(su, sv) / n;
}

// M9h: THE DERIVED FIELD. Central differences on the derived current, one dispatch over the
// gradient bank's OWN resident list -- which is the eta bank's list dilated by one tile,
// because this stencil reads its neighbours and a null tile would silently swallow the write
// (GAMEPLAN 4.2, hazard 1 and hazard 3 in the same kernel).
//
// Land and dry cells present zero: a null tile reads zero anyway, so the two agree and the
// consumer never has to ask which it got.
[numthreads(16, 16, 1)]
void CsSweVelGrad(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const uint2 t = EtaTexel(id);
    if (t.x >= gNx || t.y >= gNy) return;
    const float4 c = gUv[uint3(t, 0)];
    if (c.w < 0.5f) { gMv[uint3(t, 0)] = float4(0, 0, 0, 0); return; }

    const uint2 xm = uint2(max(int(t.x) - 1, 0), t.y);
    const uint2 xp = uint2(min(t.x + 1u, gNx - 1u), t.y);
    const uint2 ym = uint2(t.x, max(int(t.y) - 1, 0));
    const uint2 yp = uint2(t.x, min(t.y + 1u, gNy - 1u));
    float4 a = gUv[uint3(xm, 0)], b = gUv[uint3(xp, 0)], d = gUv[uint3(ym, 0)], e = gUv[uint3(yp, 0)];
    if (a.w < 0.5f) a = c;
    if (b.w < 0.5f) b = c;
    if (d.w < 0.5f) d = c;
    if (e.w < 0.5f) e = c;

    const float dudx = (b.x - a.x) / (2.0f * gDx);
    const float dvdx = (b.y - a.y) / (2.0f * gDx);
    const float dudy = (e.x - d.x) / (2.0f * gDy);
    const float dvdy = (e.y - d.y) / (2.0f * gDy);
    // grade 0 = div, grade 2 = curl. The wedge is the bivector coefficient in e1^e2.
    gMv[uint3(t, 0)] = float4(dudx + dvdy, dvdx - dudy, 1.0f, 0.0f);
}

// ---- the bed trace (an instrument: SweSolver::TraceBed) -------------------------------------
// THE BED THE SOLVER READS, read back. BedAt over the whole lattice -- the kernel's own function
// through the rule's own residency reads, so what comes back is the bed CsSweBedFill caches for the
// flux and height kernels at this instant, not a second reader's idea of it -- with the mip the
// rule read at and the slice it read from (HeightPages.hlsli, HP_TRACE); the floor is the
// planted failure's (0 = the residency map's own). TraceBed compiles it with HP_TRACE. Without it
// -- the dxtest compiles every entry it finds, with no defines -- the same kernel writes the bed
// and -1 where the choice would be, and nothing else in this file reads the two bindings below.
cbuffer SweTraceCb : register(b1) {
    float gTraceFloor;
    float gTracePadA, gTracePadB, gTracePadC;
};
RWTexture2D<float2> gBedTrace : register(u4);   // x = BedAt (m NAVD), y = mip + 16 * slice

[numthreads(16, 16, 1)]
void CsSweBedTrace(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gNx || id.y >= gNy) return;
#ifdef HP_TRACE
    gHpTraceFloor = gTraceFloor;
    const float bed = BedAt(int2(id.xy));
    gBedTrace[id.xy] = float2(bed, gHpTraceMip + 16.0f * float(gHpTraceSlice));
#else
    gBedTrace[id.xy] = float2(BedAt(int2(id.xy)), -1.0f);
#endif
}
