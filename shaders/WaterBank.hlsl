// ================================================================================================
//  WaterBank.hlsl - M7: THE WAVE VERTEX BANK -- the water's geometry as ONE tiled resource.
//
//  A camera-anchored window of mip rings (finest tiles at the camera, coarser rings outward)
//  over TWO RGBA16F banks:
//      disp  = (dx, dy, dz, foam)          -- the wave vertex, metres about the local level
//      param = (levelNavd, sigma2, u, v)   -- the mean surface + shed variance + current
//
//  Each resident tile is RECOMPUTED every frame by this kernel from the state the weather
//  manager federates: the FFT cascades (folded by the tile's own texel footprint -- the M6t
//  grade shedding, so a coarse ring carries variance where a fine ring carries geometry),
//  the SWE window banks where resident (eta + solved currents), and per-tile corner params
//  the CPU sampled from the atlas stacks (tide level via the constituent rotors, local Hs
//  from the global wave grid, bed for the dry guard). Land tiles are NULL -- never mapped,
//  never dispatched: no water here costs nothing, which is the atlas thesis.
//
//  In the state-diagram reading (the user's architecture): this kernel is an EDGE -- the
//  geometric product carrying the weather-manager node's state into the renderer node. The
//  accepting state samples the bank; it never asks who computed what.
// ================================================================================================

#include "WindowRows.hlsli"

cbuffer BankCb : register(b0) {
    float4 gOrg;        // xy = window origin (world m, snapped), z = base texel m, w = time s
    float4 gPatch;      // xyz = cascade patch sizes m, w = height exaggeration
    float4 gBandK;      // xyz = representative wavenumber per cascade, w = list count
    float4 gSweDims;    // xy = swe grid nx/ny, zw = 1 / eta-atlas padded dims
    float4 gMisc;       // x = tile texels, y = seaLevel fallback, zw unused
    uint4  gSlotsA;     // cascade disp SRV slots x3, swe eta SRV slot
    uint4  gSlotsB;     // swe uv SRV slot, disp/param/detail bank UAV slots
    uint4  gSlotsC;     // x = churn atlas SRV (foam memory), y = swell-shadow SRV
    float4 gChurn;      // xy = churn world origin, z = 1/domain, w = atlas texels
    float4 gPeakDir;    // xy = peak propagation dir (world unit), z = valid, w unused
    uint4  gSlotsD;     // x = height window SRV, y = its residency-map SRV (M7q),
                        // z = the window's SLICE when x/y are array views (M9aq), else ~0
    uint4  gSlotsE;     // M8 foamlaw: cascade DERIV SRVs x3 (hx, hz, J, foam)
    float4 gRmsRef;     // M8: unit-sea rms envelope per band (xyz), w spare
    // ---- M8 THE SOLVED WAVE FIELD (ALGEBRA.md wavefield; src/sim/WaveField) ----
    // One RGBA8 atlas of 17 slices in a 2-wide grid (slice s at ((s&1)*nx, (s/2)*ny)):
    // per-component (a/aMax, k/kMax, cos*.5+.5, sin*.5+.5), slice 16 = the envelope
    // (rms/envMax, excess/2.5, sum/sumMax, -). Inside its feathered window the solved
    // field OWNS the structure-bearing bands (cascades 0-1 yield); the chop band and
    // the ripple tail stay local. Time is the rotor e^{-i sigma t}, applied to the
    // stored spinor with CPU-computed (cos, sin)(sigma t) -- phase never wraps here.
    uint4  gWaveU;      // M9bc: x = wave PAGE tenant array SRV (0xFFFFFFFF = absent), y = its
                        // residency SRV, z = nUsed comps, w = the envelope plane
    float4 gWaveA;      // window org xy (world m), z = 1/cellM, w = feather m
    float4 gWaveB;      // x = envMax, y = sumMax, z = chop, w = solved-at level (NAVD)
    float4 gFoamA;      // scene closures: churnGain, shedSteepCap, shedMssCeil, crestLo
    float4 gFoamB;      // crestHi, depthLo, depthHi, spare (data/wave_scene.json)
    float4 gWaveSig[8];    // (cos, sin)(sigma_c t) packed 2 comps/row: c even .xy, odd .zw
    float4 gWaveDir[8];    // unit propagation (east, north), same packing
    float4 gWaveScale[8];  // (aMax, kMax) dequant scales, same packing (aMax 0 = unused)
    // ---- M8 THINGS THAT FLOAT (ALGEBRA.md wake; proofs/kelvin_wake.py) ----
    // Up to 8 vessels, the reference's table: A = (x, z world m, heading rad, speed m/s),
    // B = (wake amp m, hull half-length m, enabled, spare). LAYOUT LAW (learned the hard
    // way): new rows append at the END on BOTH sides -- a same-size permutation passes
    // the byte-parity gate and silently offsets every later row (the solved field read
    // boat zeros as its dequant scales and vanished from the water for three renders).
    float4 gBoatA[8];
    float4 gBoatB[8];
    // M9c: the wavenumber the FOLD judges each band by -- energy-weighted over the live
    // spectrum, not the geometric midpoint of the band's cuts. gBandK above is unchanged
    // and still drives phase speed / shoaling / wave-current. Appended at the END, per the
    // layout law above.
    float4 gBandKFold;
    // M9p: --flat-bed. x != 0 replaces the sampled bed with y everywhere, so the SAME scene
    // can be filled twice -- real bathymetry and a flat floor -- and the two banks diffed.
    // Reading the code proves the bed is WIRED to the geometry (ShoalFactor on ab.x, the
    // hmax = 0.55*depth breaking clamp); only a diff proves it MOVES it. Appended at the end
    // on both sides, per the law twelve rows up.
    float4 gDebugA;
    float4 gWaveP;      // M9bc: the z16 page frame: org px x, y, 1/16384, world px at z16
    float4 gWaveD;      // M9bc: window nx, ny (solver cells = page texels); zw = the window's NW
                        // texel in the page frame (step 2: the page texel IS the cell)
    // M9bl: THE SECOND SIXTEEN. kMaxComp went 16 -> 32 because a 16-component directional
    // sum IS a regular comb -- sixteen long-crested trains 1.6 deg apart superpose into a
    // fixed interference lattice, which is what the storm face's straight parallel ridges
    // are. A real sea's crests are irregular because its spectrum is continuous; the cure
    // is more components, not more mesh (proved: 2x vertex density left the ridges intact).
    // These APPEND at the end rather than widening the arrays above, per the layout law
    // sixty rows up -- widening in place slides gBoatA and every row after it.
    float4 gWaveSig2[8];
    float4 gWaveDir2[8];
    float4 gWaveScale2[8];
    // M9bt: the fold's SECOND moment per band -- the energy-weighted width of ln k. Appended
    // at the end on both sides, per the layout law above.
    float4 gBandKSpread;
    // THE SOLVER IS TRUTH (the water match, step 1): x = the tide plane the solver was forced by
    // this frame (NAVD m) -- its deviation is measured from it. Appended at the end on both sides.
    float4 gSweB;
    // PHASE B2 (out/integration/plan_phase_b.md): THE WINDOWS THE RINGS STAND IN -- the rows of the
    // level whose eye the rings stand about (bank A: the camera's world; set B: the window's world),
    // its chain found at each texel's own point (BankTile.point*). Appended at the END on both sides.
    HP_WINDOW_ROWS_DECL
    HP_SOLVER_ROWS_DECL     // the solver's chart about the rings' frame (appended LAST)
};

// M9bl: one component's rows, from whichever half holds it. r = comp >> 1.
float4 WaveSigRow(uint r) { return (r < 8u) ? gWaveSig[r] : gWaveSig2[r - 8u]; }
float4 WaveDirRow(uint r) { return (r < 8u) ? gWaveDir[r] : gWaveDir2[r - 8u]; }
float4 WaveScaleRow(uint r) { return (r < 8u) ? gWaveScale[r] : gWaveScale2[r - 8u]; }

struct BankTile {
    float2 orgXZ;       // this tile's window-frame origin, world m
    float texelM;       // this tile's texel size (mip ladder)
    uint dstX;          // atlas texel origin of the tile slot
    uint dstY;
    float lvl00, lvl10, lvl01, lvl11;   // tide level at corners (atlas stack, CPU rotors)
    float bed00, bed10, bed01, bed11;   // bed at corners (the one height stack)
    float hs00, hs10, hs01, hs11;       // local Hs / reference Hs at corners (sim/WaveScale.h)
    // M13 step 2: THE TILE'S PLACE, ADDRESSED ONCE (WaterBankLayer.h's BankTile says how they
    // are built). lat0/lon0 at the tile's origin, the tangent map at its centre, in degrees and
    // degrees per metre; placeB.z = 0 means this tile has no place (past the frame's horizon)
    // and the chart row is all it has.
    float4 placeA;      // lat0, lon0, dLat/dx, dLon/dx
    float4 placeB;      // dLat/dz, dLon/dz, valid, spare
    // M13 step 2: the cascade sea's four planes for this tile (sim/WaveChart.h), in the law's own
    // order. Per chart: its coordinate at the tile's ORIGIN wrapped into each cascade's period
    // (rows 0..5), the tangent map of that coordinate over the tile (6..9), its axes said in the
    // place's east/north (10..13), two spare. bandX/bandY carry the edge distances the shares are
    // computed from -- the shares themselves are recomputed per texel, as the hull recomputes
    // them per point, so both processors run one expression.
    float4 chart[4][4];
    float4 bandX;       // edgeX at the origin, d/dex, d/dez, the band (m)
    float4 bandY;       // edgeY at the origin, d/dex, d/dez, 1 = the charts are valid
    // PHASE B2: THE TILE'S POINT for the windows' address -- the texel's ground point relative to
    // its level's eye in the tangent axes the rows were pulled into: pointA at the tile's origin
    // (w = 1 valid), its derivative along the ring's x and z at the tile's centre (midpoint rule).
    // The windows' planes are central, so the sphere's sag across a tile (radial) moves no address.
    float4 pointA;
    float4 pointX;
    float4 pointZ;
};
StructuredBuffer<BankTile> gTiles : register(t0);

// THE PLACE OF A TEXEL, from its own metres inside the tile: the tile's rows, exact at its origin
// and second-order over the tile (PHASE C5: the anchor-linear fallback and its gGeoA row are gone;
// a tile past the frame's horizon has no place and no rows).
float2 TilePlace(const BankTile t, float2 exz) {
    return float2(t.placeA.x + exz.x * t.placeA.z + exz.y * t.placeB.x,
                  t.placeA.y + exz.x * t.placeA.w + exz.y * t.placeB.y);
}

// THE SHARES AT A TEXEL, from the tile's two edge rows -- WaveChart::Shares, said in HLSL. Half
// exactly on a cell edge, one a half-band inside it, the neighbour taking the rest.
void TileShares(const BankTile t, float2 exz, out float w[4]) {
    const float band = max(t.bandX.w, 1.0f);
    const float wx = smoothstep(0.0f, 1.0f,
                                0.5f + (t.bandX.x + exz.x * t.bandX.y + exz.y * t.bandX.z) / band);
    const float wy = smoothstep(0.0f, 1.0f,
                                0.5f + (t.bandY.x + exz.x * t.bandY.y + exz.y * t.bandY.z) / band);
    w[0] = wx * wy;
    w[1] = (1.0f - wx) * wy;
    w[2] = wx * (1.0f - wy);
    w[3] = (1.0f - wx) * (1.0f - wy);
}

// One chart's coordinate for cascade c at a texel: the tile's origin row (wrapped into that
// cascade's own period on the CPU) plus the tangent map across the tile. Rows: [0] = cascade 0
// and 1's origins, [1].xy = cascade 2's, [2] = the tangent map, [3] = the axes.
float2 ChartUv(const BankTile t, uint k, uint c, float2 exz) {
    const float2 u0 = (c == 0u) ? t.chart[k][0].xy
                    : (c == 1u) ? t.chart[k][0].zw
                                : t.chart[k][1].xy;
    const float4 j = t.chart[k][2];   // du/dex, du/dez, dv/dex, dv/dez
    return u0 + float2(j.x * exz.x + j.y * exz.y, j.z * exz.x + j.w * exz.y);
}
float4 ChartRot(const BankTile t, uint k) { return t.chart[k][3]; }

#include "Jet.hlsli"

// Bindless over the shared heap (the renderer's doctrine, compute-side): textures by SLOT,
// never by name -- the residency machinery can move data under this kernel freely. All
// reads are manual-bilinear LOADS: the static-sampler SampleLevel path silently returns
// zero on this driver for bindless arrays outside the pixel stage (stage-bisected in M7
// bring-up; the mesh stage showed the same).
Texture2D gT[] : register(t0, space1);
Texture2DArray gTA[] : register(t0, space5);   // M9aq: the height PAGE tenant's array views
#define HP_WINDOW_ROWS 1
#include "HeightPages.hlsli"
RWTexture2D<float4> gU[] : register(u0, space2);

// M9bo: THE CASCADE, PREFILTERED TO THIS RING.
//
// The cascades are realized at 2.95 / 0.73 / 0.18 m per texel (L / 256). A bank tile stores at
// its RING's texel -- 1.2 m on ring 0, doubling outward to 38 m -- so this read is coarser than
// its source by 1.6x on cascade 1, 6.7x on cascade 2, and up to 200x on the outer rings. It was
// a bilinear tap at MIP 0, which prefilters nothing: every wavelength between the cascade's
// texel and the ring's folded straight into the stored displacement as alias, and aliasing a
// directional spectrum on a regular lattice makes exactly the kind of structured pattern this
// engine spends its fold law avoiding.
//
// The fold does not cover this. It judges a whole band by ONE representative wavenumber
// (gBandKFold) and admits or sheds it entire; the texture underneath still carries the band's
// full spread, and the part of that spread below the ring's Nyquist is what aliased. So the
// fold decides WHETHER a band is geometry, and the mip chain decides WHAT of it survives at
// this tile's scale. Two different questions -- the second one was simply never asked.
//
// mip = log2(ringTexel / cascadeTexel), clamped: 0 where the ring already oversamples the
// cascade (cascade 0 on ring 0), rising to the top of the chain on the outer rings.
float CascadeMip(uint c, float ringTexelM) {
    const float cascadeTexelM = gPatch[c] * (1.0f / 256.0f);
    return clamp(log2(max(ringTexelM, 1e-4f) / max(cascadeTexelM, 1e-6f)), 0.0f, 8.0f);
}

float4 LoadWrapAtLevel(uint slot, float2 uv, int lvl) {
    const float dim = max(256.0f / exp2(float(lvl)), 1.0f);
    const int idim = int(dim);
    const float2 tf = uv * dim - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    float4 acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = (int2(t0) + int2(k & 1, k >> 1) + idim) % idim;
        acc += ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y) *
               gT[slot].Load(int3(tc, lvl));
    }
    return acc;
}

// TRILINEAR, not a rounded pick. The ring texel almost never lands on a power of two of the
// cascade's -- ring 0 against cascade 2 wants log2(1.2 / 0.1836) = 2.71 -- and rounding that to
// 3 filters to 1.47 m when the ring holds 1.2 m, which throws away resolvable chop and reads as
// the near water going soft. Blending the two levels lands the filter exactly on the ring's
// scale: no alias from under-filtering, no lost detail from over-filtering, and no step in the
// look when a tile's texel crosses a power of two.
float4 LoadBilinearWrapMip(uint slot, float2 uv, float mip) {
    const int lo = int(floor(mip));
    const float f = mip - float(lo);
    const float4 a = LoadWrapAtLevel(slot, uv, lo);
    if (f < 0.002f) return a;
    return lerp(a, LoadWrapAtLevel(slot, uv, min(lo + 1, 8)), f);
}

float4 LoadBilinearWrap(uint slot, float2 uv, float dim) {
    const float2 tf = uv * dim - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    float4 acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = (int2(t0) + int2(k & 1, k >> 1) + int(dim)) % int(dim);
        acc += ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y) *
               gT[slot][tc];
    }
    return acc;
}

// M8: bilinear over ONE SLICE of the solved-wave atlas (slice s at ((s&1)*nx, (s/2)*ny));
// taps clamp INSIDE the slice so components never bleed into each other. The spinor
// channels blend componentwise in the plane -- the cl2 law; the caller renormalizes.
// M9bc: the solved wave field is PAGES of the wave.field tenant (z16, slices 6.. = planes:
// component p's (a, k, cos, sin), then the envelope). Mip 0 is what the window pins; a plane
// not resident at mip 0 here reads as absent (0) and the caller's window weight drops it.
// M12 step 4e: the frame, the floor and the read are PageSample.hlsli's (the same lat/lon
// spelling HeightPages.hlsli takes for the bed).
// THE SOLVED FIELD'S PAGE TEXEL IS ITS CELL (the water match, step 2). The page provider paints solver
// cell (i, j) -- row 0 south -- into page texel (winPx + i, winPy + ny - 1 - j) (WaveFieldSource::
// PaintTile), so a point of the flat frame finds its texel through the solver's own grid: cells from the
// window's SW corner, flipped to rows from its north edge, offset by the window's texel in the page frame
// (gWaveD.zw). Through Mercator latitude instead -- the old PageUvLatLon -- the rows came out 0.67 %
// shorter than the solver's cells (a Mercator texel's metre of latitude is 111319.49 m/deg, the chart's
// 110574), so the drawn solved waves stood up to 13.6 m north of the bathymetry they were solved over and
// WaveField::ProbeAt, on the grid, disagreed with them.
float2 WavePageUv(float2 xz) {
    const float2 cells = (xz - gWaveA.xy) * gWaveA.z;
    return float2(gWaveD.z + cells.x, gWaveD.w + gWaveD.y - cells.y) * gWaveP.z;
}
// M9bl: the finest RESIDENT mip of one plane here (byte = finest mip * 16, conservative
// per 128th of the page). > 7.5 means nothing is resident at all.
float WavePageHave(float2 uv, uint plane) {
    return PageHaveLoad(gTA[gWaveU.y], uv, 6u + plane);
}
// Was "< 0.5f": resident meant MIP 0 RESIDENT, so a window holding coarse levels counted as
// absent and the solved field contributed nothing at all. That is the opposite of this
// engine's residency law (priors: a miss degrades to the best resident ANCESTOR -- blur --
// never to nothing and never to unmapped garbage), and it is why the field could only ever
// be all-or-nothing instead of arriving as a gradient.
bool WavePageResident(float2 uv, uint plane) {
    return WavePageHave(uv, plane) <= 7.5f;
}
float4 WavePageSample(float2 uv, uint plane) {
    // M9bl: read the finest mip actually RESIDENT here, not mip 0. Same clamp the height
    // pages, the exposure and the churn already use -- the window refines as its levels
    // land instead of appearing whole.
    const float have = WavePageHave(uv, plane);
    if (have > 7.5f) return 0.0f;                 // nothing resident: no opinion
    const float mip = max(round(have), 0.0f);
    // The byte's decode CENTRE (the water match, step 2): the solve truncates to bytes, so a byte b stands
    // for [b, b + 1) / 255 and its unbiased value is (b + 0.5) / 255 -- the decode WaveField::ProbeAt has
    // always used. The UNORM read gives b / 255; the half-LSB is added after the filter, which is linear.
    return PageLoad4(gTA[gWaveU.x], uv, 6u + plane, mip) + (0.5f / 255.0f);
}

float4 LoadBilinearClamp(uint slot, float2 texel, float2 dims) {
    const float2 tf = texel - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    float4 acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = clamp(int2(t0) + int2(k & 1, k >> 1), int2(0, 0),
                              int2(dims) - int2(1, 1));
        acc += ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y) *
               gT[slot][tc];
    }
    return acc;
}

// (M8 foam breakup noise MOVED to the pixel stage: at ring resolution its fine octaves
// folded away and the throat painted as featureless milk -- a shading texture must live
// at shading resolution. The kernel ships PURE physics foam; Globe.hlsl breaks it up.)

// ================================================================================================
//  M8 KELVIN WAKES (ALGEBRA.md wake; proofs/kelvin_wake.py). Station keeping c = U cos
//  theta gives k = K0 sec^2 theta, K0 = g/U^2; stationarity reduces to the quadratic
//  2 zeta t^2 + xi t + zeta = 0 (t = tan theta) and the famous 19.4712-degree wedge IS its
//  discriminant. THE SIGNED PHASE: this port CORRECTS its reference -- the vqview shader
//  folded |t| into a non-stationary angle (measured |grad ph|/k up to 6.65 where theory
//  demands 1); the true phase at the signed roots is ph = K0 sec theta (xi - zeta |t|),
//  already mirror-symmetric, with slope direction d = grad(ph)/k. Gatest block 10 pins the
//  quadratic, the wedge, and the gradient identity the reference fails.
//  Every bound is physics with its argument recorded (closures, gate-pinned): steepness
//  cap ak <= 0.30 (pre-breaking Miche family), divergent damping exp(-(k/5K0)^2) (viscous
//  removal of the short arm), band-limit vs THIS TILE'S TEXEL (the kernel is the mesh --
//  smoothstep(2, 5, lambda/texel)), cusp boost faded in over dRel 1..3.5 (Airy validity),
//  near fade 0.5..2.5 hull-lengths (linear theory dies at the hull), draught amp cap.
// ================================================================================================
void WakeBranch(float t, float K0, float xi, float zeta, float amp, float sampleM,
                float2 fwd, float2 rgt, float side, inout float eta, inout float2 slope,
                inout float akMax) {
    const float sec = sqrt(1.0f + t * t);
    const float k = K0 * sec * sec;
    const float lam = 6.2831853f / k;
    const float w = smoothstep(2.0f, 5.0f, lam / max(sampleM, 1e-3f));
    if (w <= 0.0f) return;
    // the SIGNED stationary phase + its consistent slope direction (ph and d flip together)
    const float ph = K0 * sec * (xi - zeta * abs(t));
    const float ct = 1.0f / sec, st = abs(t) / sec;
    const float2 d = -(fwd * ct + rgt * (side * st));
    const float kRel = k / max(K0, 1e-6f);
    const float damp = exp(-(kRel / 5.0f) * (kRel / 5.0f));
    const float aMaxK = 0.30f / max(k, 1e-3f);
    const float a = min(amp * w * damp, aMaxK);
    eta += a * cos(ph);
    slope -= (a * k) * sin(ph) * d;
    akMax = max(akMax, a * k);
}

void WakeOne(float4 A, float4 B, float2 posM, float sampleM, inout float eta,
             inout float2 slope, inout float akMax, inout float stern) {
    if (B.z < 0.5f || A.w < 0.5f) return;
    const float2 fwd = float2(cos(A.z), sin(A.z));
    const float2 rgt = float2(-fwd.y, fwd.x);
    const float2 r = posM - A.xy;
    if (dot(r, r) > 1200.0f * 1200.0f) return;   // spread + damp are dead past this
    const float xi = -dot(r, fwd);               // metres astern
    const float across = dot(r, rgt);
    const float side = (across >= 0.0f) ? 1.0f : -1.0f;
    const float zeta = abs(across);
    const float halfLen = max(B.y, 1.0f);
    // stern turbulence envelope (the aerated prop wash): short, narrow, decaying --
    // the reference's tuned closure; joins the foam UNION, never adds.
    if (xi > 0.0f) {
        const float wid = halfLen * 0.34f * (1.0f + xi / (halfLen * 10.0f));
        stern = max(stern, smoothstep(0.0f, halfLen * 0.3f, xi) *
                               exp(-xi / (halfLen * 3.2f)) *
                               (1.0f - smoothstep(wid * 0.45f, wid * 1.15f, zeta)));
    }
    if (xi <= 0.5f) return;                      // nothing ahead of the bow
    const float disc = xi * xi - 8.0f * zeta * zeta;
    if (disc <= 0.0f) return;                    // outside the wedge: no stationary phase
    const float sq = sqrt(disc);
    const float U = A.w;
    const float K0 = 9.81f / (U * U);
    const float dist = max(length(r), 1.0f);
    const float dRel = dist / halfLen;
    const float cuspFar = 1.0f + 1.8f * pow(saturate(1.0f - sq / max(xi, 1e-3f)), 3.0f);
    const float cusp = 1.0f + (cuspFar - 1.0f) * smoothstep(1.0f, 3.5f, dRel);
    const float nearFade = smoothstep(0.5f, 2.5f, dRel);
    const float ampCap = 0.22f * halfLen * 0.34f;
    const float spread = 1.0f / sqrt(0.6f + dRel);
    const float amp =
        min(B.x * smoothstep(0.0f, halfLen * 2.0f, xi) * cusp * spread, ampCap) * nearFade;
    if (amp < 1e-4f) return;
    if (zeta < 1e-3f) {
        WakeBranch(0.0f, K0, xi, zeta, amp, sampleM, fwd, rgt, side, eta, slope, akMax);
        return;
    }
    const float t1 = (-xi + sq) / (4.0f * zeta);
    const float t2 = (-xi - sq) / (4.0f * zeta);
    WakeBranch(abs(t1), K0, xi, zeta, amp, sampleM, fwd, rgt, side, eta, slope, akMax);
    WakeBranch(abs(t2), K0, xi, zeta, amp * 0.85f, sampleM, fwd, rgt, side, eta, slope,
               akMax);
}

// M7n: THE COINCIDENCE CARD. Quadrant shading (4 distinct levels -- any flip or rotation
// permutes them visibly) + thin border lines at the wrap seams.
float CardPattern(float2 uv) {
    const float2 f = frac(uv);
    float v = 0.20f + 0.30f * step(0.5f, f.x) + 0.40f * step(0.5f, f.y);
    if (any(f < 0.03f) || any(f > 0.97f)) v = 1.0f;   // wrap-seam border
    return v;
}

[numthreads(16, 16, 1)]
void CsBankFill(uint3 id : SV_DispatchThreadID) {
    const uint texels = (uint)gMisc.x;
    if (id.x >= texels || id.y >= texels) return;
    const BankTile t = gTiles[id.z];
    const float2 f = (float2(id.xy) + 0.5) / gMisc.x;
    // M7n: texel id holds the field at its CENTER (id + 0.5) -- the coincidence card
    // caught this line writing CORNERS while BankSample reconstructs centers (local -
    // 0.5): every bank field sat half a texel off, 2.4 m at ring 0 and 77 m at ring 5.
    // The corner-lerp f above already used centers; now the whole kernel agrees.
    const float2 exz = (float2(id.xy) + 0.5f) * t.texelM;   // this texel's own metres in the tile
    const float2 xz = t.orgXZ + exz;
    // M13 step 2: the texel's PLACE, from the tile's rows (TilePlace above). Every lat/lon read
    // below -- the bed, the swell shadow -- is asked for here, once, instead of being re-derived
    // from the chart at each site.
    const float2 place = TilePlace(t, exz);
#ifdef BANK_TRACE
    // PHASE B0, THE BANK'S BED TRACE (WaterBankLayer::TraceBank): this entry is compiled with
    // HP_TRACE and BANK_TRACE and dispatched beside the fill on a traced frame; it computes what
    // the fill computes and writes, in place of the banks, the bed it read, the slice and mip the
    // rule chose (255 = the corner lerp: no tenant), the depth and the vertical excursion before
    // the breaking clamp. gDebugA.w is the planted residency floor (0 = the map's own).
    gHpTraceFloor = gDebugA.w;
    gHpTraceSlice = 255u;
    gHpTraceMip = 0.0f;
#endif

    // Corner-lerped spatial context (the CPU sampled the atlas stacks at the corners; a tile
    // spans well under the tide's or the wave grid's own resolution, so bilinear is honest
    // for the LEVEL. The BED is not that smooth -- and with M7p's shoaling and breaking the
    // corner-lerp quantized the surf geography to ~600 m patches that CUT at tile edges
    // (the data lens showed it; the user called it). M7q: per-texel bed from the COMPOSED
    // HEIGHT WINDOW, residency-clamped, corner-lerp as the out-of-window fallback.
    const float level = lerp(lerp(t.lvl00, t.lvl10, f.x), lerp(t.lvl01, t.lvl11, f.x), f.y);
    float bed = lerp(lerp(t.bed00, t.bed10, f.x), lerp(t.bed01, t.bed11, f.x), f.y);
    // The local sea-state scale: one law with the hull's twin (sim/WaveScale.h), continuous across
    // the tile and across the wave grid's nodes (it was one value per tile, read off one node).
    const float hsScale = lerp(lerp(t.hs00, t.hs10, f.x), lerp(t.hs01, t.hs11, f.x), f.y);
    // PHASE B2: the texel's direction (the cube's read) and its chain (the windows').
    const float latR = place.x * 0.01745329252f, lonR = place.y * 0.01745329252f;
    const float3 dirT = float3(cos(latR) * cos(lonR), sin(latR), cos(latR) * sin(lonR));
    const float3 pT = t.pointA.xyz + exz.x * t.pointX.xyz + exz.y * t.pointZ.xyz;
    WalkChain wcT = WindowChain(pT, 0u);
    if (t.pointA.w == 0.0f) wcT.n = 0u;
#ifdef BANK_TRACE
    float traceMip = 0.0f;
    uint traceSlice = 255u;
#endif
    if (gSlotsD.x != 0xFFFFFFFFu) {
        // M9ax: the whole tenant -- the z14 page where it is resident and fine, the cube face
        // everywhere else on the planet -- so shoaling, the current amplification and the
        // depth-limited breaking act on every coast the rings reach, not only inside one page.
        // The corner lerp above remains only for a bank with no height tenant at all.
        const float lat = place.x;
        const float lon = place.y;
        // THE BED AT THE RING'S OWN GRAIN (the water match, step 3): the page level whose texel is
        // no finer than this ring's, floored to a whole level -- mip 0 (9.55 m of Mercator, ~7 m
        // here) for the rings a hull and an eye stand in, rising with the coarse rings. The constant
        // 2 it replaces ("their own texels are 1.2 m and up") held every ring to a z14 page's level
        // 2 -- 28 m texels here -- and beside the jetty the depth laws saw the wall's 28 m average:
        // the drawn sea stood at a dry weight of 0.14 where the hull, on a 1 m bed, stood at 1.0.
        // compose/HeightPage gives a hull this bed at the finest level.
        // PHASE B2: the pyramid's windows at the ring's own grain (HpHeightChain: each rank at the
        // finest level no finer than the ring's texel, the cube where no window holds the texel).
        bed = HpHeightChain(gTA[gSlotsD.x], gTA[gSlotsD.y], dirT, wcT, t.texelM);
#ifdef BANK_TRACE
        traceMip = gHpTraceMip;
        traceSlice = gHpTraceSlice;
#endif
    }

    // THE LEVEL (the water match, step 1 -- the solver is truth). The atlas everywhere; inside the
    // solver's domain the surface IS the solver's: the tide plane it was forced by (gSweB.x) plus
    // the deviation it holds, over the domain's weight -- rising from 0 at the grid's edge to 1 one
    // cell in (the field is defined at cell centres). The old `level + dEta` added the deviation
    // from a UNIFORM plane to the spatially varying atlas, and a hard 0.1 % edge cut the domain.
    // TreeWater reads this expression (WeatherManager::SolverRefine) from the same texels,
    // delivered through a region readback; --water-probe is the gate that the two stand together.
    float lvl = level;
    float2 cur = 0.0f;
    // The solver is read where the texel's GROUND lies in its chart (PHASE C1: the domain's planes at
    // the point the windows' chain is given); the planes give the cell.
    if (gSvO.z > 0.0f && t.pointA.w != 0.0f && SolverDen(pT) > 0.0f) {
        const float2 texel = SolverCell(pT);
        const float eCells =
            min(min(texel.x, gSvO.x - texel.x), min(texel.y, gSvO.y - texel.y));
        const float wDom = smoothstep(0.0f, 1.0f, eCells);
        if (wDom > 0.0f) {
            const float dEta = LoadBilinearClamp(gSlotsA.w, texel, gSweDims.xy).x;
            lvl = lerp(level, gSweB.x + dEta, wDom);
            const float4 s = LoadBilinearClamp(gSlotsB.x, texel, gSweDims.xy);
            if (s.w > 0.5f) cur = s.xy;
        }
    }
    // --flat-bed: substitute a constant floor AFTER every real sample, so the only thing that
    // changes between the two runs is the bed itself -- same window, same residency, same
    // solver, same instant.
    if (gDebugA.x != 0.0f) bed = gDebugA.y;
    const float depth = lvl - bed;
    const float dry = smoothstep(0.05f, 0.65f, depth);

    // M7j: THE SWELL SHADOW. The solver's line-of-sight exposure field (CPU march toward
    // the peak-wave source) always sheltered the OLD renderer; one-water lost the edge
    // silently and whitecapped the harbor basin -- the GA AST's orphan rule exists because
    // of this bug. Ocean bands fold by exposure; sigma^2 rides the same amplitude-squared
    // law; local chop keeps a floor. The shadow shares the SWE window's frame (row 0 =
    // north), so the same (1 - v) flip applies.
    // M9ba: the exposure is a PAGE of the swell.exposure tenant (z14 slice, mips >= 3), read
    // through the same lat/lon -> page frame as the bed; nothing resident = exposed.
    // PHASE B2: window slices alone (D2), at its grain -- 76 m, rung 3, today's floor; no window
    // holding the texel is no opinion: exposed.
    float expo = 1.0f;
    if (gSlotsC.y != 0xFFFFFFFFu && gSlotsC.z != 0xFFFFFFFFu) {
        const float e = HpChainRead(gTA[gSlotsC.y], gTA[gSlotsC.z], dirT, wcT, HpRankGround(0u), false, -1.0f);
        if (e >= 0.0f) expo = max(e, 0.18f);
    }

    // M8: the solved wave field's window weight -- inside it the solved field OWNS the
    // structure-bearing bands (cascades 0-1 yield by (1 - wWin)); the chop band and the
    // foam machinery stay local. Feathered so the handover is invisible (every rung
    // earns its place and VANISHES where it cannot -- the M7h symmetric-ladder doctrine).
    float wWin = 0.0f;
    float wCas = 0.0f;   // what the CASCADES yield by -- see the delivery law below
    float2 wuv = 0.0f;
    if (gWaveU.x != 0xFFFFFFFFu) {
        const float2 wcell = (xz - gWaveA.xy) * gWaveA.z;
        const float2 dimsW = gWaveD.xy;
        const float eM =
            min(min(wcell.x, dimsW.x - wcell.x), min(wcell.y, dimsW.y - wcell.y)) /
            gWaveA.z;
        wWin = smoothstep(0.0f, max(gWaveA.w, 1.0f), eM);
        wuv = WavePageUv(xz);
        // ---- M9by: THE CASCADES YIELD BY WHAT THE FIELD SUPPLIED, NOT BY THE WINDOW. -----
        //
        // This was the SAME bug a74382c fixed on the CPU, in the renderer, and it was left
        // here because that commit's own report asserted "the GPU was already correct --
        // WaterBank.hlsl zeroes wWin when the page isn't resident, so its cascades take 1".
        // It zeroes wWin on the residency of PLANE 0. The solved field has 33 planes, each
        // streaming on its own schedule, and WavePageSample returns ZERO for any plane that
        // has not landed. So the instant plane 0 arrived, wWin went to 1, cascades 0 and 1
        // stood down entirely, and every component still in flight contributed nothing. The
        // blend stopped summing to one and the difference was simply lost.
        //
        // Systematic, not random: main.cpp Wants the planes in order p = 0..nPlanes-1, so
        // plane 0 is always first to land. And it inverts with RANGE, which is the tell the
        // user reported -- "big waves in the distance for a moment, then they disappear by
        // the time I get near". Far off the window wants a coarse mip: few tiles, the whole
        // field lands, the sea is whole. Closing in, wantMip walks down to 0, all 33 planes
        // re-request at the finer level, and plane 0 re-arms wWin = 1 while the rest are
        // still in flight. The water flattens as you approach it.
        //
        // The cure is the ingest rule, exactly as on the CPU: a point the solved field has
        // not supplied is not a point with less sea, it is a point the cascades own. The
        // solved sum already carries only what answered, so it keeps the geometric weight;
        // the cascades yield by that weight scaled by the FRACTION that answered. f = 0 and
        // the cascades take the whole sea; f = 1 and it is the old handover; in between the
        // sea is whole and streaming decides only WHICH description carries it.
        //
        // Counted only inside the window, where wWin > 0 -- open water never pays for it,
        // and the map is 128x128 for the whole page, so a tile's threads all hit the same
        // few texels.
        if (wWin > 0.0f) {
            float nHave = 0.0f;
            [loop] for (uint q = 0; q < gWaveU.z; ++q) {
                if (WavePageResident(wuv, q)) nHave += 1.0f;
            }
            wCas = wWin * (nHave / max(float(gWaveU.z), 1.0f));
        }
    }

    // THE FOLD, per ring (M6t): a band is geometry while THIS tile's texels resolve its
    // phase; past its Nyquist it sheds to sigma^2. Coarse rings carry the same energy as
    // statistics that fine rings carry as vertexes -- no popping between rings possible.
    float3 d = 0.0f;
    float sig2 = 0.0015f;
    // M8 FOAM DISCIPLINE: the old additive sum double-counted one physical event (the
    // Jacobian foam and the Miche steepness are an affine bijection -- gatest block 9)
    // and adding churn on top fused the surf into a white sheet. The new law: the
    // crest-gated UNION of triggers, noise-broken; churn is the same quantity
    // REMEMBERED, max-composited below.
    float steepFoam = 0.0f;   // union over bands of Jacobian foam (the deriv fiber)
    float blockFoam = 0.0f;   // union over bands of current-blocking foam
    // Per-band gains -> the detail plane. Band 1 has fed the PS sparkle since M7a; M8
    // adds bands 0 and 2 in the free fibers so the PS can amplitude-scale the caustic
    // Jacobian and Laplacian it assembles from the cascade derivative textures
    // (ALGEBRA.md caustics: areaJac_fold = 1 + sum w*amp*(J_c - 1); lap folds linearly).
    float gain0 = hsScale * expo;
    float gain1 = hsScale * expo;
    float gain2 = hsScale * expo;
    // M13 step 2: this texel's shares of the lattice's planes, ONCE -- they are a function of the
    // texel, not of the band, so the three bands below read them rather than re-deriving them.
    float chartW[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float chartNorm = 1.0f;
    const bool chartsOn = t.bandY.w != 0.0f;
    if (chartsOn) {
        TileShares(t, exz, chartW);
        chartNorm = sqrt(max(chartW[0] * chartW[0] + chartW[1] * chartW[1] +
                                 chartW[2] * chartW[2] + chartW[3] * chartW[3], 1e-12f));
    }
    [unroll] for (uint c = 0; c < 3; ++c) {
        // ---- M9bt: THE FOLD ASKS FOR A FRACTION, NOT A VERDICT. --------------------------
        //
        // The question is "how much of this band can a grid of THIS spacing still carry", and
        // Nyquist answers it exactly: a grid of spacing T resolves k < pi/T and nothing above.
        // But a cascade band is not one wavenumber -- band 2 spans lambda 0.41..12 m, thirty
        // to one -- so the answer is not yes or no, it is the FRACTION OF THE BAND'S VARIANCE
        // BELOW NYQUIST. With the band log-normal about its own energy-weighted mean
        // (gBandKFold) and its own energy-weighted width (gBandKSpread), that fraction is a
        // logistic in ln(k_nyquist / k_fold), and sqrt(3)/pi is the scale that matches a
        // logistic to a normal of that width. Nothing here is placed: both moments come from
        // the live spectrum, and Nyquist is Nyquist.
        //
        // WHAT IT REPLACES, and why the sea was glass: smoothstep(lam*0.12, lam*0.5, texelM)
        // reached ZERO at texelM = lam/2 -- exactly AT Nyquist, where half the band's energy
        // still sits at wavelengths the grid resolves perfectly well -- and it began shedding
        // at lam/8, four times finer than the limit. Band 2 folds at 6.96 m in a 7 s sea, so
        // its geometry weight was 0.36 on the 2.4 m ring and 0 on every ring beyond: the chop
        // existed only within ~300 m of the camera and the whole sea past it was a mirror.
        // Measured, a declared Hs 0.80 arrived as rms 0.13 m where a Gaussian sea wants 0.20.
        //
        // Nothing is lost by keeping more: the shed term below is (1 - w), so whatever leaves
        // geometry still arrives as slope variance. The two are complementary by construction,
        // which is why this cannot double-count and cannot pop between rings.
        const float kNy = 3.14159265f / max(t.texelM, 1e-4f);
        const float sLog = max(gBandKSpread[c], 0.05f) * 0.5513289f;   // sigma -> logistic
        const float w = saturate(1.0f / (1.0f + exp(-log(kNy / gBandKFold[c]) / sLog)));
        // M7p: the two ORPHANED PHYSICS EDGES, restored from the retired SeaLayer path
        // and found by the 2D proof figure: SHOALING (Green's-law growth as the group
        // speed drops entering shallow water) and WAVE-CURRENT amplification (the ebb
        // standing the entrance up toward blocking -- the 7-foot-standing-wave term).
        // proofs/inlet_storm.py runs the SAME pure functions on the SAME fields; the
        // match report holds this kernel to it.
        float amp = hsScale * expo;
        float blocked = 0.0f;
        if (depth > 0.05f) {
            const float cB = BandPhaseSpeed(gBandK[c], max(depth, 0.3f));
            float2 ab = (gPeakDir.z > 0.5f) ? WaveCurrentAmp(cur, gPeakDir.xy, cB)
                                            : float2(1.0f, 0.0f);
            ab.x *= ShoalFactor(gBandK[c], depth);
            amp *= ab.x;
            blocked = ab.y;
        }
        // The detail-plane gains ship the FULL closure (the PS sparkle and caustic
        // tiers key on them everywhere -- zeroing them inside the solved window killed
        // the throat's caustics; the waterdata lens convicted it in one probe).
        if (c == 0) gain0 = amp;
        if (c == 1) gain1 = amp;
        if (c == 2) gain2 = amp;
        // M8: inside the solved window the cascades' structure bands stand down -- the
        // solved field carries shoaling/refraction/limiting per cell, not per band.
        if (c != 2) amp *= 1.0f - wCas;   // M9by: the DELIVERED weight, not the window
        const float cmip = CascadeMip(c, t.texelM);
        // ---- M13 step 2: THE BAND IS READ IN THE LATTICE'S OWN PLANES (sim/WaveChart.h). Where
        // this used to be frac(xz / L) on the root's tangent plane -- one plane for a planet,
        // which is why a hull carried 2054 km rode a different sea from the one drawn around it
        // -- the texel's charts are read at their own coordinates and blended variance-
        // preservingly, the vector channels rotated into this place's east/north first. In a
        // cell's plateau (nine places in ten) exactly one share is non-zero and this is the
        // single read it always was.
        float4 s = 0.0f, dv = 0.0f;
        if (chartsOn) {
            [unroll] for (uint k = 0; k < 4u; ++k) {
                if (chartW[k] < 1e-4f) continue;   // the law's own skip; the norm keeps it
                const float2 cuvK = frac(ChartUv(t, k, c, exz) / gPatch[c]);
                const float4 sK = LoadBilinearWrapMip(gSlotsA[c], cuvK, cmip);
                const float4 dK = LoadBilinearWrapMip(gSlotsE[c], cuvK, cmip);
                const float4 R = ChartRot(t, k);
                const float wk = chartW[k] / chartNorm;
                // The heights and the scalar channels blend as they are; the horizontal pair is
                // a vector in the chart's axes and turns into the place's.
                s += wk * float4(R.x * sK.x + R.y * sK.z, sK.y, R.z * sK.x + R.w * sK.z, sK.w);
                dv += wk * float4(R.x * dK.x + R.y * dK.y, R.z * dK.x + R.w * dK.y, dK.z, dK.w);
            }
        } else {
            const float2 cuv = frac(xz / gPatch[c]);   // no charts (Mars, or no rows): as before
            s = LoadBilinearWrapMip(gSlotsA[c], cuv, cmip);
            dv = LoadBilinearWrapMip(gSlotsE[c], cuv, cmip);
        }
        d += s.xyz * (w * amp);
        // The Jacobian foam lives in the DERIV fiber (the disp fiber's w is zero --
        // the old additive term here read it and contributed nothing since M7).
        // Monahan-gated: the Jacobian says WHERE a whitecap sits, the wind says HOW MANY
        // there are. Depth/blocking/wake foam stay ungated -- that breaking is geometry
        // and current physics, not wind climatology.
        steepFoam = max(steepFoam, dv.w * w * saturate(amp) * gFoamB.w);
        blockFoam = max(blockFoam, blocked * 0.35f * w * (c == 2 ? 1.0f : 0.4f));
        // shed variance: amplitude squared (gain included -- the far field sees the
        // steepened bar as a brighter glint band even when texels cannot draw it)
        sig2 += (1.0f - w) * amp * amp *
                (c == 0 ? 0.0004f : (c == 1 ? 0.0018f : 0.0060f));
    }

    // ---- M8 THE SOLVED WAVE FIELD (ALGEBRA.md wavefield): per-cell a, k, and the
    // integrated phase spinor, advanced by the rotor e^{-i sigma t} and folded by THIS
    // ring's texel exactly like every band -- comps the ring cannot resolve shed their
    // slope variance (0.5 (a k)^2) to sigma^2 instead of aliasing. eta = sum a cos(phi -
    // sigma t): the reference's surface, wearing our live gains through the window blend.
    float rmsW = 0.0f, excW = 0.0f;
    if (wWin > 0.001f) {
        float3 dW = 0.0f;
        float sigW = 0.0f;
        [loop] for (uint s = 0; s < gWaveU.z; ++s) {
            const float4 sc = WaveScaleRow(s >> 1);
            const float aMax = (s & 1) ? sc.z : sc.x;
            if (aMax <= 0.0f) continue;
            const float kMax = (s & 1) ? sc.w : sc.y;
            const float4 t4 = WavePageSample(wuv, s);
            // The swell shadow shelters the SOLVED bands exactly as it does the
            // cascades (the helm-in-the-lee shot exposed the asymmetry: solved comps
            // sailed through the jetty's lee unsheltered).
            const float aW = t4.x * aMax * expo;
            if (aW < 1e-4f) continue;
            const float kW = max(t4.y * kMax, 1e-4f);
            // ---- M9bx: THE SOLVED FIELD GETS THE SAME FOLD THE CASCADES GOT. -------------
            //
            // M9bt replaced smoothstep(lam*0.12, lam*0.5, texelM) for the CASCADE bands and
            // left it here, on the solved components -- the half of the sea the inlet is
            // ENTIRELY made of, because inside the window every cascade but band 2 stands
            // down against (1 - wWin). So the fix for "flat and glassy" never reached the
            // water the boat is actually driving through, and the same complaint came back
            // in the same place. Same law, same file, the half that was missed.
            //
            // What the old curve did: reached ZERO at texelM = lam/2 -- exactly AT Nyquist,
            // where the grid still resolves the wave perfectly -- and started shedding at
            // lam/8, four times finer than the limit. In the inlet the solved comps SHOAL, so
            // their local lambda is short: a 20 m wave was drawn at 0.009 of its amplitude on
            // the 9.6 m ring, i.e. deleted about 300 m out. Beyond that the sea was a mirror.
            //
            // What replaces it needs no spread and no placed constant, because a solved
            // component is not a band -- it is ONE wavenumber, and the ring texel is a BOX
            // AVERAGE of width texelM. A sinusoid through a box of width T comes out scaled
            // by sinc(kT/2), and the mesh's linear reconstruction between texels squares it.
            // So the fold IS the filter's own transfer function, evaluated at this component's
            // own k. It is 1 for long waves, 0.44 at Nyquist, and reaches its first null at
            // lambda = texelM, where a wave exactly one texel long genuinely averages to
            // nothing. The sidelobes past the null stay (they are <5% and a real box really
            // does pass them) -- no clamp, no gate, no special case.
            //
            // The cascade fold needed the band's log-WIDTH because a band is a distribution
            // and the answer there is the fraction of its variance below Nyquist. Here the
            // distribution is a delta, so the transfer function is the whole answer.
            //
            // Energy is conserved exactly as before: whatever leaves geometry, (1 - wFc)
            // below carries as slope variance.
            const float xF = 0.5f * kW * t.texelM;
            const float sF = sin(xF) / max(xF, 1e-6f);
            const float wF = sF * sF;
            // Shed steepness rides the MICHE cap (ak <= 0.44): the limiter bounds HEIGHT
            // by depth, but an opposing current grows k unbounded while a stays -- the
            // raw (a k)^2 shed painted arrested zones as a white sigma^2 wash stepping
            // at ring boundaries. A wave steeper than the limit has BROKEN (the excess
            // gate already turns that energy into foam); the glint keeps only what a
            // real sea can carry. Total capped below -- both closures, ALGEBRA.md.
            const float akS = min(aW * kW, gFoamA.y);
            // ---- M9bu: THE SPINOR'S LENGTH IS NOT NOISE, IT IS THE ANSWER. --------------
            //
            // WavePageSample reads the finest mip actually RESIDENT (M9bl, so the window
            // refines as its levels land rather than appearing whole). Reading a coarser mip
            // BILINEARLY AVERAGES THE PHASE SPINOR over that footprint -- and averaging
            // phasors shortens them. The length that comes back is exactly the phase
            // COHERENCE of the component over the footprint that was read: 1 where the phase
            // is smooth across it, 0 where the wave turns over inside it.
            //
            // Normalising that back to unit length threw the answer away and restored FULL
            // AMPLITUDE to an incoherent average, pointing at the circular MEAN of the phases
            // in the footprint -- a wave of the right size at a fabricated, shifted phase.
            // Per plane, because each component's page streams on its own schedule, so a
            // subset of the 32 comps sat at shifted phases and the subset changed tile by
            // tile as levels landed. The user saw it exactly: "every few rows gets an offset
            // ... like a signal got shifted when going into the tiles", only in the inlet
            // (the solved field's window), and in the GEOMETRY rather than the shading.
            //
            // The length is the footprint filter's own transfer function on this component --
            // the same quantity the band fold computes from Nyquist, here measured directly
            // instead of derived. So it multiplies the amplitude, exactly as wF does, and
            // what it takes out of geometry the shed below carries as slope variance. A
            // component the page cannot resolve now contributes NOTHING rather than a
            // full-amplitude lie, and the window still refines as its levels land -- it just
            // grows in honestly, from low amplitude to full, instead of arriving at full
            // amplitude in the wrong place.
            float2 sp = t4.zw * 2.0f - 1.0f;
            const float coh = saturate(length(sp));
            sp /= max(length(sp), 1e-4f);          // the spinor stays unit (cl2 law)
            const float wFc = wF * coh;
            sigW += (1.0f - wFc) * 0.5f * akS * akS;
            if (wFc <= 0.001f) continue;
            const float4 rr = WaveSigRow(s >> 1);
            const float2 rot = (s & 1) ? rr.zw : rr.xy;
            const float cT = sp.x * rot.x + sp.y * rot.y;   // cos(phi - sigma t)
            const float sT = sp.y * rot.x - sp.x * rot.y;   // sin(phi - sigma t)
            const float4 dd = WaveDirRow(s >> 1);
            const float2 dir2 = (s & 1) ? dd.zw : dd.xy;
            dW.y += wFc * aW * cT;
            dW.xz -= gWaveB.z * wFc * aW * sT * dir2;
        }
        const float4 env = WavePageSample(wuv, gWaveU.w);
        rmsW = env.x * gWaveB.x * expo;   // the solver's envelope, sheltered like its comps
        excW = env.y * 2.5f * expo;       // its breaking indicator, rms_raw / rms_limit
        d += dW * wWin;
        // Storm-sea mss ceiling on the solved shed (default ~ hurricane Cox-Munk): past
        // it the surface is breaking, and breaking is foam's business, not the glint's.
        sig2 += wWin * min(sigW, gFoamA.z) * gPatch.w * gPatch.w;
    }

    // M8 KELVIN WAKES: real displacement, band-limited against THIS tile's texel (the
    // kernel IS the mesh); wake steepness joins the foam union through the same Miche
    // band the sea uses, stern turbulence joins ungated (prop wash cares nothing for
    // crests). Superposition: 8 boats cost a loop, no interaction to resolve.
    float sternF = 0.0f;
    {
        float wEta = 0.0f;
        float2 wSlope = 0.0f;
        float wakeAk = 0.0f;
        [unroll] for (uint b = 0; b < 8; ++b) {
            WakeOne(gBoatA[b], gBoatB[b], xz, t.texelM, wEta, wSlope, wakeAk, sternF);
        }
        d.y += wEta;
        steepFoam = max(steepFoam, smoothstep(0.352f, 0.528f, wakeAk));
    }
    d *= dry * gPatch.w;

    // The local rms ENVELOPE: unit-sea band rms scaled by this texel's own gains. The
    // depth-excess trigger tests it (never instantaneous |eta| -- television static),
    // and the crest gate normalizes eta by it (foam rides crests only: a Gaussian sea
    // passes ~22.6% of area through this gate; the strength noise breaks the rest).
    // (gRmsRef already carries the vertical exaggeration -- the CPU bakes it -- so it
    // is commensurate with d.y as written; no second gPatch.w here.) Inside the solved
    // window the solver's OWN envelope and breaking indicator take over -- they carry
    // per-cell shoaling/refraction/limiting the band closures can only approximate.
    const float envRmsC =
        max(sqrt(gain0 * gain0 * gRmsRef.x * gRmsRef.x +
                 gain1 * gain1 * gRmsRef.y * gRmsRef.y +
                 gain2 * gain2 * gRmsRef.z * gRmsRef.z), 1e-4f);
    const float envRms = lerp(envRmsC, max(rmsW * gPatch.w, 1e-4f), wCas);
    // The breaking indicator is PHYSICAL: divide the display exaggeration back out
    // (the exaggerated envelope inflated depth foam ~15% everywhere -- part of the
    // "rapids" look the user called; the solved excW is physical by construction).
    const float excessC =
        (envRmsC / max(gPatch.w, 1e-3f)) * 2.8284271f / (0.60f * max(depth, 0.05f));
    const float excess = lerp(excessC, excW, wCas);
    // The kernel writes PURE physics foam (triggers x crest); the visual BREAKUP is the
    // pixel stage's job -- noise at ring resolution folded its fine octaves away and
    // painted the throat as featureless milk (the fold law, learned again: a shading
    // texture must live at shading resolution).
    const float depthFoam = smoothstep(gFoamB.y, gFoamB.z, excess);
    const float crest = smoothstep(gFoamA.w, gFoamB.x, d.y / max(envRms, 1e-3f));
    float foam = saturate(max(steepFoam, max(depthFoam, blockFoam)) * crest);

    // M7e/M8: THE FOAM MEMORY. The churn atlas remembers where water has been aerated
    // (breaking deposits advected by the solved current -- the seaward streaks off an
    // ebbing entrance). M8: churn is the SAME quantity remembered, so it composites by
    // MAX, never + (adding memory to fresh foam brightened the throat into uniform fog);
    // the live noise modulates the memory so old deposits stay textured, not flat.
    if (gSlotsC.x != 0xFFFFFFFFu) {
        // M9az: toroidal atlas on the world lattice: window test on the origin, wrap sample.
        const float2 cuv = (xz - gChurn.xy) * gChurn.z;
        if (all(cuv > 0.001f) && all(cuv < 0.999f)) {
            const float churnV =
                LoadBilinearWrap(gSlotsC.x, frac(xz * gChurn.z), gChurn.w).x;
            foam = max(foam, saturate(churnV) * gFoamA.x * 1.3f);
        }
    }
    // Stern turbulence: a thin aerated tail (the reference's 0.42 weight; the PS
    // noise textures it with everything else).
    foam = max(foam, sternF * 0.42f);

    // NO CAP ON THE HEIGHT (the owner, 2026-10-04: the flat-topped sea at Haulover was the
    // depth-limited breaking clamp |eta| <= 0.55 h, a min on the displacement). The surface is
    // the data the bands and the solver say; what breaks is said by the foam (depthFoam above,
    // from the envelope), never by cutting the geometry. The hull's water (WaterSurfaceTree) and
    // the sea sheet (Sea.hlsl) say the same.
#ifdef BANK_TRACE
    const float dyPre = abs(d.y);
#endif

    // M7m: EDGE PATTERN INJECTION. Flip the switch and this kernel writes a WORLD-ALIGNED
    // test card into the foam fiber instead of physics: a 50 m checker and a wedge that
    // points NORTH every 500 m. If the pattern arrives on screen continuous across rings,
    // unmirrored, wedges northward -- the bank -> render edge is clean; any flip,
    // rotation, or scale error draws itself.
    if (gMisc.z > 0.5f) {
        if (gMisc.z > 1.5f) {
            // M7n: the CASCADE-EDGE half of the coincidence test -- the card drawn from
            // THIS KERNEL's belief of cascade-1's wrap uv. The PS draws the same card
            // from ITS mapping into green; on screen, agreement is pure yellow and any
            // relative offset / flip / scale between the two samplings fringes red/green.
            foam = CardPattern(xz / gPatch[1]);
        } else {
            const float chk =
                fmod(floor(xz.x / 50.0f) + floor(xz.y / 50.0f) + 400000.0f, 2.0f);
            const float2 cell = frac(xz / 500.0f);
            const float wedge =
                (abs(cell.x - 0.5f) < 0.05f * (1.0f - cell.y) && cell.y > 0.4f) ? 1.0f
                                                                                : 0.0f;
            foam = chk * 0.30f + wedge;
        }
        d = 0.0f;
    }

    const uint2 dst = uint2(t.dstX + id.x, t.dstY + id.y);
#ifdef BANK_TRACE
    gU[gSlotsC.w][dst] = float4(bed, float(traceSlice) * 16.0f + traceMip, depth, dyPre);
    return;
#endif
    gU[gSlotsB.y][dst] = float4(d, saturate(foam * dry));
    // M9e: THE STORM-SEA CEILING, applied to the WHOLE shed. The solved field has always
    // clamped its own contribution to gFoamA.z ("past it the surface is breaking, and
    // breaking is foam's business, not the glint's") -- but the CASCADE shed above it was
    // uncapped, and it carries amp^2 with hsScale up to 3. Measured on --storm 3.0,10,95:
    // param.sigma2 reached 0.139 against the AST's declared 0.1, and the fiber dump had
    // been printing OUT OF RANGE the whole time. sqrt(0.139) = 0.37 rms slope is the glint
    // lobe of a ~27 m/s wind on a sea whose implied wind is 10.3 -- the surface reflected
    // the pale horizon sky over its entire area and the ebb ride went white.
    // One ceiling, both paths, and the declared range becomes true again.
    sig2 = min(sig2, gFoamA.z);
    gU[gSlotsB.z][dst] = float4(lvl, sig2, cur.x, cur.y);
    // The DETAIL plane: what the PS needs to recover sub-ring sparkle and the caustic
    // Jacobian -- per-band sea-state gains (cascade derivs are unit-sea) and the dry
    // guard (no sparkle on the flats). Layout: (gain1, dry, gain0, gain2).
    gU[gSlotsB.w][dst] = float4(gain1, dry, gain0, gain2);
}
