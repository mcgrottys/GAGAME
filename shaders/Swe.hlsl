// ================================================================================================
//  M5c: the sparse shallow-water solver -- the estuary actually FLOWING.
//
//  State lives in atlas banks as the DEVIATION from the analytic tide plane: eta_bank = eta -
//  tide(t). A NULL tile therefore means "the stateless model is exactly right here", which is
//  the gameplan's thesis expressed as hydrodynamics: the offshore sponge relaxes the deviation
//  to zero, and everything the basin does differently -- the lag behind the ocean, the throat
//  jet, the river's standing slope -- IS the resident state.
//
//  Scheme: the classic virtual-pipe model (Mei et al. 2007): per-cell outflow fluxes to the four
//  neighbours accelerate down the water-surface gradient, are scaled so a cell never ships more
//  volume than it holds (wet/dry handled for free), and the height update is the flux
//  divergence. Damping keeps it unconditionally friendly at dt <= ~0.3 dx / sqrt(g h_max).
//
//  Texture space: +x = east, +y = SOUTH (bathy row 0 is the northern edge). CsSweDerive flips v.
//  Flux channels: x = +x (east), y = +y (south), z = -x (west), w = -y (north).
// ================================================================================================

#include "WindowRows.hlsli"

cbuffer SweCb : register(b0) {
    uint  gNx, gNy;              // bathy grid dims (1701 x 890)
    uint  gEtaTilesX, gFluxTilesX;
    uint  gEtaTileW, gEtaTileH, gFluxTileW, gFluxTileH;
    uint  gListCount;
    float gSpongeSides;          // PHASE C1: the open sides, a bitmask: 1 N, 2 S, 4 W, 8 E
    float gDy;                   // M6r: north-south texel metres (equiangular CUDEM grid:
                                 // dy = 13.65 != dx = 10.08 -- the per-axis metric)
    float gFaceCount;            // the river faces (gFaceA, gFaceB; SweSolver.h SweConfig::faces)
    float gDx, gDt, gDamp, gTideNavd;
    float gSpongeD0, gSpongeRate, gPadR, gGravity;
    // THE RIVER FACES (2026-10-21): where a river enters the domain (sim/RiverModel.h). A face: its
    // side (0 N, 1 S, 2 W, 3 E), the run of edge cells [c0, c1] along it (gFaceA); Flather's u_ext
    // (m/s, + into the domain: the face's TRANSPORT -- river Q minus the upriver prism's demand --
    // over its live section; radiation alone cannot carry a prism) and its exterior's target
    // DEVIATION (river tide - ocean tide) (gFaceB).
    float4 gFaceA[4];
    float4 gFaceB[4];
    float gTideRate;             // M6r: d(tide plane)/dt, m/s -- the prism source term
    float gPadA, gPadB, gPadC;
    // PHASE C1: a cell's place in the domain's chart (SweSolver::SetBed): the first cell's centre in
    // the anchor's tangent plane over R and the steps (s0, ds, t0, dt); the anchor's up and R; its
    // east and the cell's grain (m). North = east x up.
    float4 gCellX;
    float4 gAnc;
    float4 gEast;
    HP_WINDOW_ROWS_DECL
};

#define HP_WINDOW_ROWS 1
#include "HeightPages.hlsli"

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

StructuredBuffer<uint> gTileList : register(t0);
// M9ar: THE BED IS THE HEIGHT MEGATEXTURE. M9ax: the tenant's WHOLE array (cube faces 0..5 and
// the eye's windows and the standing window: PHASE B2/B3) with its residency-map array, resolved by HeightPages.hlsli
// exactly as the pixel stage resolves it. There is no solver-private copy of the bed, and no
// wall past the page: a lattice anywhere on the planet has a bed.
Texture2DArray<float4> gBathy    : register(t1);   // NAVD88 m; R16F loads as .x
Texture2DArray<float4> gBathyRes : register(t2);   // R8: finest resident mip * 16
RWTexture2D<float>  gEta  : register(u0);        // DEVIATION from the tide plane, m
RWTexture2D<float2> gFlux : register(u1);        // SIGNED face fluxes, m^3/s: x = across the
                                                 // EAST face (+east), y = across the SOUTH
                                                 // face (+south). Staggered C-grid.
// M9ax: the derived currents are a Volatile GradeBank like gMv below (the dense committed
// texture is gone); the view is pinned to slice 0, so the solve writes uint3(xy, 0).
RWTexture2DArray<float4> gUv : register(u2);     // derived: u east, v north, speed, valid
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

float BedAt(int2 t) {
    if (any(t < 0) || t.x >= (int)gNx || t.y >= (int)gNy) return 100.0f;   // outside = wall
    // PHASE C1: the cell's centre in the domain's chart -> its point on the sphere; the standing
    // window's chain there at the cell's grain (PHASE B2 D4), the cube by the direction where the
    // window does not hold it.
    const float s = gCellX.x + float(t.x) * gCellX.y, tn = gCellX.z + float(t.y) * gCellX.w;
    const float3 dir = normalize(gAnc.xyz + s * gEast.xyz + tn * cross(gEast.xyz, gAnc.xyz));
    return HpHeightChain(gBathy, gBathyRes, dir, WindowChain(SwePointAbout(s, tn), 0u), gEast.w);
}

float EtaAt(int2 t) {
    if (any(t < 0) || t.x >= (int)gNx || t.y >= (int)gNy) return 0.0f;
    return gEta[t];   // NULL tiles read 0 = "exactly the tide plane"
}

// Water-surface elevation (absolute, NAVD) if wet, else the bed (dry cells present their ground
// so gradients push water back downhill instead of through it).
float SurfaceAt(int2 t, out float h) {
    const float bed = BedAt(t);
    const float eta = gTideNavd + EtaAt(t);
    h = max(eta - bed, 0.0f);
    return (h > 0.0f) ? eta : bed;
}

// The offshore sponge, RAMPED: 0 inside the estuary and over the entrance bar (live SWE), 1 in
// open water where the stateless ocean is the truth. A hard sponge edge is a reflecting wall --
// the first cut had one, and the deep zone checkerboarded and flooded the throat with slosh.
// PHASE C1: it stands on the domain's OPEN SIDES (SweDomain::Bound: every cell along the side can be
// wet, and no river enters there), by the cell's distance to the nearest of them.
float SpongeAt(int2 t) {
    const uint sides = (uint)gSpongeSides;
    const float2 c = float2(t) + 0.5f;
    float d = 3.0e38f;
    if (sides & 1u) d = min(d, c.y * gDy);
    if (sides & 2u) d = min(d, (float(gNy) - c.y) * gDy);
    if (sides & 4u) d = min(d, c.x * gDx);
    if (sides & 8u) d = min(d, (float(gNx) - c.x) * gDx);
    return 1.0f - smoothstep(gSpongeD0 - 700.0f, gSpongeD0, d);
}

uint2 EtaTexel(uint3 id) {
    const uint t = gTileList[id.z];
    return uint2(t % gEtaTilesX, t / gEtaTilesX) * uint2(gEtaTileW, gEtaTileH) + id.xy;
}

uint2 FluxTexel(uint3 id) {
    const uint t = gTileList[id.z];
    return uint2(t % gFluxTilesX, t / gFluxTilesX) * uint2(gFluxTileW, gFluxTileH) + id.xy;
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

// ---- init / reset --------------------------------------------------------------------------

[numthreads(16, 16, 1)]
void CsSweClearEta(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    gEta[EtaTexel(id)] = 0.0f;
}

[numthreads(16, 16, 1)]
void CsSweClearFlux(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gFluxTileW || id.y >= gFluxTileH) return;
    gFlux[FluxTexel(id)] = 0;
}

// Full-surface (non-list) clear of the derived-current texture: texels the derive pass never
// visits (NULL tiles, land) must read valid=0 so the sea falls back to the analytic jet.
[numthreads(16, 16, 1)]
void CsSweUvClear(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gNx || id.y >= gNy) return;
    gUv[uint3(id.xy, 0)] = float4(0, 0, 0, 0);
}

// ---- one substep ---------------------------------------------------------------------------

// One signed face update. eta/h describe the owner cell; nbr is the cell across the face.
// The first cut used the classic game pipe model (four RECTIFIED outflows, max(0, .)): with
// honest ocean-scale friction, passing gravity waves pump both opposing pipes up and nothing
// drains them -- a ratchet that filled the offshore with ~30 m^3/s phantom fluxes and froze
// eta. Signed staggered faces cannot ratchet.
// M6r, the per-axis metric: the grid is equiangular, so a face is faceLen metres LONG and its
// gradient acts over span metres -- (faceLen, span) = (dy, dx) for east faces, (dx, dy) for
// south faces. The old isotropic gDx overdrove every north-south term by dy/dx = 1.35.
float FaceUpdate(float q, int2 t, int2 nbr, float faceLen, float span) {
    float h, hn;
    const float eta = SurfaceAt(t, h);
    const float etan = SurfaceAt(nbr, hn);
    const float sill = max(BedAt(t), BedAt(nbr));
    const float hface = max(max(eta, etan) - sill, 0.0f);   // upwind-ish face depth
    if (hface <= 0.0f) return 0.0f;

    // Accelerate down the surface gradient; implicit quadratic bottom drag (Cd = 0.0025);
    // a whisper of linear damping for numerical hygiene.
    q += gDt * gGravity * hface * (eta - etan) * faceLen / span;   // g*hface*(dEta/span)*faceLen
    const float uf = q / max(hface * faceLen, 1e-3f);
    // Cd 0.0015: sandy-estuary range. 0.0025 was fine over the 5 km mouth window; across the
    // M6d wide window's 15 km reach the extra friction accumulated ~40 min of phase lag.
    q *= gDamp / (1.0f + gDt * 0.0015f * abs(uf) / max(hface, 0.3f));

    // Positivity: no face may move more than a quarter of its DONOR cell's volume per step.
    const float Vself = h * gDx * gDy;
    const float Vnbr = hn * gDx * gDy;
    return clamp(q, -0.25f * Vnbr / gDt, 0.25f * Vself / gDt);
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
static float4 gWtCells = 0.0f;  // the exterior cell's eta, the interior depth, both beds
#define SWE_FACE_LAW(f, rad, ub, qRaw, lo, hi) if ((f) == 0) { gWtLaw = float4(gFaceB[0].x, (rad), (ub), (qRaw)); gWtClamp.yz = float2((lo), (hi)); }
#define SWE_FACE_ROW(f, a, etaX, etaI, sill, hf, hI, q) if ((f) == 0) { gWtFace = float4((etaX), (etaI), (sill), (hf)); gWtClamp.xw = float2((q), 1.0f); gWtHi = (hI); gWtAlong = (a); gWtCells = float4(EtaAt(e), (hI), BedAt(e), BedAt(i)); }
#else
#define SWE_FACE_LAW(f, rad, ub, qRaw, lo, hi)
#define SWE_FACE_ROW(f, a, etaX, etaI, sill, hf, hI, q)
#endif

// The river face whose EXTERIOR cell `e` is (-1: none): a cell of a face's side, in its run, whose
// bed can be wet (under +2 m).
int FaceOfExterior(int2 e) {
    [loop] for (int f = 0; f < (int)gFaceCount; ++f) {
        const int side = (int)gFaceA[f].x;
        const bool onEdge = side == 0 ? e.y == 0 : side == 1 ? e.y == (int)gNy - 1 : side == 2 ? e.x == 0 : e.x == (int)gNx - 1;
        const int a = side < 2 ? e.x : e.y;
        if (onEdge && a >= (int)gFaceA[f].y && a <= (int)gFaceA[f].z && BedAt(e) < 2.0f) return f;
    }
    return -1;
}

// M6r: FLATHER, the open boundary RADIATES. The exterior cell rides the river tide as pinned data;
// the flux across the face to its interior neighbour `i` is the Flather condition
//     u_b = u_ext + sqrt(g / h) (eta_ext - eta_int),
// so an interior surplus flows OUT at the gravity-wave speed instead of reflecting off a relaxed
// strip (the Dirichlet strip was a soft wall: transients bounced, and the basin's phase carried the
// echo). u_ext is the PRESCRIBED transport (the river's discharge less the upriver prism's demand)
// over the live section: radiation alone cannot carry a prism -- without u_ext the through-flow
// throttles behind the standing dEta it needs to sustain itself (measured on the Merrimack: the gap
// current lost a third of its amplitude). L: the face's length. Returns the flux INTO the domain.
float FaceFlux(int f, int2 e, int2 i, float L) {
    float hI;
    const float etaI = SurfaceAt(i, hI);
    const float etaX = gTideNavd + gFaceB[f].y;          // exterior = the river-tide data
    const float sill = max(BedAt(e), BedAt(i));
    const float hf = max(max(etaX, etaI) - sill, 0.0f);
    float q = 0.0f;
    if (hf > 0.05f) {
        const float rad = sqrt(gGravity / hf) * (etaX - etaI);
        const float ub = gFaceB[f].x + rad;
        const float V = gDx * gDy / gDt;                 // positivity, as everywhere
        q = clamp(hf * L * ub, -0.25f * hI * V, 0.25f * hf * V);
        SWE_FACE_LAW(f, rad, ub, hf * L * ub, -0.25f * hI * V, 0.25f * hf * V);
    }
    SWE_FACE_ROW(f, ((int)gFaceA[f].x < 2 ? e.x : e.y), etaX, etaI, sill, hf, hI, q);
    return q;
}

[numthreads(16, 16, 1)]
void CsSweFlux(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gFluxTileW || id.y >= gFluxTileH) return;
    const int2 t = int2(FluxTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;

    float2 q = gFlux[t];
    q.x = FaceUpdate(q.x, t, t + int2(1, 0), gDy, gDx);   // east face (dy long, dx span)
    q.y = FaceUpdate(q.y, t, t + int2(0, 1), gDx, gDy);   // south face (texture +y)

    // THE RIVER FACES (FaceFlux): a thread owns its cell's east and south faces, so a face whose
    // exterior is this cell (a W or N side) or its east or south neighbour (an E or S side) is
    // this thread's -- + east and + south, inward on W and N, outward on E and S.
    if (gFaceCount > 0.0f) {
        const int fw = FaceOfExterior(t);
        if (fw >= 0 && (int)gFaceA[fw].x == 2) q.x = FaceFlux(fw, t, t + int2(1, 0), gDy);
        if (fw >= 0 && (int)gFaceA[fw].x == 0) q.y = FaceFlux(fw, t, t + int2(0, 1), gDx);
        const int fe = (t.x + 1 == (int)gNx - 1) ? FaceOfExterior(t + int2(1, 0)) : -1;
        if (fe >= 0 && (int)gFaceA[fe].x == 3) q.x = -FaceFlux(fe, t + int2(1, 0), t, gDy);
        const int fs = (t.y + 1 == (int)gNy - 1) ? FaceOfExterior(t + int2(0, 1)) : -1;
        if (fs >= 0 && (int)gFaceA[fs].x == 1) q.y = -FaceFlux(fs, t + int2(0, 1), t, gDx);
    }

    // Kill reflections off the sponge: waves entering the analytic ocean just fade.
    const float sp = SpongeAt(t);
    if (sp > 0.0f) q *= 1.0f - 0.5f * sp * gSpongeRate;
#ifdef SWE_FACE_TRACE
    if (gWtAlong >= 0) {
        gFaceTrace[uint2(0, gWtAlong)] = gWtFace;
        gFaceTrace[uint2(1, gWtAlong)] = gWtLaw;
        gFaceTrace[uint2(2, gWtAlong)] = gWtClamp;
        gFaceTrace[uint2(3, gWtAlong)] = gWtCells;
    }
    return;
#endif
    gFlux[t] = q;
}

[numthreads(16, 16, 1)]
void CsSweHeight(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const int2 t = int2(EtaTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;

    // Divergence of the signed face fluxes: volume in = west+north faces, out = my own faces.
    const float2 qc = gFlux[t];
    const float qw = gFlux[int2(t.x - 1, t.y)].x;   // across my west face (+ = into me)
    const float qn = gFlux[int2(t.x, t.y - 1)].y;   // across my north face (+ = into me)
    float dEta = gEta[t] + gDt * ((qw - qc.x) + (qn - qc.y)) / (gDx * gDy);

    // M6r, THE PRISM TERM. The eta bank stores deviation from a MOVING plane: when the plane
    // rises the volume must still ARRIVE, so every resident cell books the rise as debt. The
    // debt's gradient against the free boundaries (the sponge = the analytic ocean, the pinned
    // river faces = the river data) IS the flood current; the ebb is the debt paid back. This
    // also makes wet/dry hydrodynamic: a flat's surface no longer rides the plane for free --
    // it waits for the water. (Measured before this term: 43 m^3/s of storage flux across the
    // whole harbor at peak flood, ~5% of the real prism; the x5 render gain was compensation.)
    dEta -= gTideRate * gDt;

    // Offshore sponge: the open sea IS the analytic tide; deviations die here. This is also the
    // tidal forcing -- the basin drains into / fills from a boundary pinned to the real clock.
    dEta *= 1.0f - SpongeAt(t) * gSpongeRate;

    // M6r, the Flather pair's other half: a river face's EXTERIOR cells are the upriver tide from
    // the station fits, interpolated to the face, held as data (hard pin -- the radiation lives in
    // the face flux, not in a relax rate). The domain ends here but the DATA keeps going: the
    // truncation becomes a boundary condition, and the boundary condition is the tide stations'.
    if (gFaceCount > 0.0f) {
        const int f = FaceOfExterior(t);
        if (f >= 0) dEta = gFaceB[f].y;
    }

    gEta[t] = clamp(dEta, -6.0f, 6.0f);
}

// ---- derived surface currents (for rendering, Doppler, and validation probes) ---------------

[numthreads(16, 16, 1)]
void CsSweDerive(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gEtaTileW || id.y >= gEtaTileH) return;
    const int2 t = int2(EtaTexel(id));
    if (t.x >= (int)gNx || t.y >= (int)gNy) return;

    float h;
    SurfaceAt(t, h);
    // Dry cells and the sponged open sea hand the render back to the stateless model (the
    // analytic jet is ACT-calibrated; out there it is the better answer).
    if (h < 0.05f || SpongeAt(t) > 0.5f) {
        gUv[uint3(t, 0)] = float4(0, 0, 0, 0);
        return;
    }
    // Cell-centred velocity: the mean of the two faces in each axis, each over its OWN face
    // area (M6r: east faces are dy long, south faces dx -- the isotropic gDx overstated v
    // by 35%).
    const float2 qc = gFlux[t];
    const float qw = gFlux[int2(t.x - 1, t.y)].x;
    const float qn = gFlux[int2(t.x, t.y - 1)].y;
    const float u = 0.5f * (qc.x + qw) / (h * gDy);
    const float v = -0.5f * (qc.y + qn) / (h * gDx);   // texture +y is south; world v is north
    gUv[uint3(t, 0)] = float4(u, v, length(float2(u, v)), 1.0f);
}

// ---- the bed trace (an instrument: SweSolver::TraceBed) -------------------------------------
// THE BED THE SOLVER READS, read back. BedAt over the whole lattice -- the kernel's own function
// through the rule's own residency reads, so what comes back is the bed the flux and height
// kernels integrate on at this instant, not a second reader's idea of it -- with the mip the
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
