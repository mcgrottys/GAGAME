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

cbuffer BankCb : register(b0) {
    float4 gOrg;        // xy = window origin (world m, snapped), z = base texel m, w = time s
    float4 gPatch;      // xyz = cascade patch sizes m, w = height exaggeration
    float4 gBandK;      // xyz = representative wavenumber per cascade, w = list count
    float4 gSwe;        // xy = swe world x0/z0, zw = 1/sizeX, 1/sizeZ (0 = solver absent)
    float4 gSweDims;    // xy = swe grid nx/ny, zw = 1 / eta-atlas padded dims
    float4 gMisc;       // x = tile texels, y = seaLevel fallback, zw unused
    uint4  gSlotsA;     // cascade disp SRV slots x3, swe eta SRV slot
    uint4  gSlotsB;     // swe uv SRV slot, disp/param/detail bank UAV slots
    uint4  gSlotsC;     // x = churn atlas SRV (foam memory), y = swell-shadow SRV
    float4 gChurn;      // xy = churn world origin, z = 1/domain, w = atlas texels
    float4 gPeakDir;    // xy = peak propagation dir (world unit), z = valid, w unused
    uint4  gSlotsD;     // x = height window SRV, y = its residency-map SRV (M7q),
                        // z = the window's SLICE when x/y are array views (M9aq), else ~0
    float4 gGeoA;       // world->latlon: orgLat, orgLon, 1/mPerLat, 1/mPerLon
    float4 gWinA;       // height window: org px x, org px y, 1/sizePx, full-world px z14
    uint4  gSlotsE;     // M8 foamlaw: cascade DERIV SRVs x3 (hx, hz, J, foam)
    float4 gRmsRef;     // M8: unit-sea rms envelope per band (xyz), w spare
    // ---- M8 THE SOLVED WAVE FIELD (ALGEBRA.md wavefield; src/sim/WaveField) ----
    // One RGBA8 atlas of 17 slices in a 2-wide grid (slice s at ((s&1)*nx, (s/2)*ny)):
    // per-component (a/aMax, k/kMax, cos*.5+.5, sin*.5+.5), slice 16 = the envelope
    // (rms/envMax, excess/2.5, sum/sumMax, -). Inside its feathered window the solved
    // field OWNS the structure-bearing bands (cascades 0-1 yield); the chop band and
    // the ripple tail stay local. Time is the rotor e^{-i sigma t}, applied to the
    // stored spinor with CPU-computed (cos, sin)(sigma t) -- phase never wraps here.
    uint4  gWaveU;      // x = atlas SRV (0xFFFFFFFF = absent), y = nx, z = ny, w = nComp
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
};

struct BankTile {
    float2 orgXZ;       // this tile's window-frame origin, world m
    float texelM;       // this tile's texel size (mip ladder)
    uint dstX;          // atlas texel origin of the tile slot
    uint dstY;
    float lvl00, lvl10, lvl01, lvl11;   // tide level at corners (atlas stack, CPU rotors)
    float bed00, bed10, bed01, bed11;   // bed at corners (the one height stack)
    float hsScale;      // local Hs / reference Hs (the global wave grid modulates the sea)
    float pad0, pad1, pad2;
};
StructuredBuffer<BankTile> gTiles : register(t0);

#include "Jet.hlsli"

// Bindless over the shared heap (the renderer's doctrine, compute-side): textures by SLOT,
// never by name -- the residency machinery can move data under this kernel freely. All
// reads are manual-bilinear LOADS: the static-sampler SampleLevel path silently returns
// zero on this driver for bindless arrays outside the pixel stage (stage-bisected in M7
// bring-up; the mesh stage showed the same).
Texture2D gT[] : register(t0, space1);
Texture2DArray gTA[] : register(t0, space5);   // M9aq: the height PAGE tenant's array views
#include "HeightPages.hlsli"
RWTexture2D<float4> gU[] : register(u0, space2);

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
float4 WaveSample(uint slot, float2 cellUv, uint s, float2 dims) {
    const float2 org = float2(float(s & 1) * dims.x, float(s >> 1) * dims.y);
    const float2 tf = clamp(cellUv, 0.5f, dims - 0.5f) - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    float4 acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = clamp(int2(t0) + int2(k & 1, k >> 1), int2(0, 0),
                              int2(dims) - int2(1, 1));
        acc += ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y) *
               gT[slot][int2(org) + tc];
    }
    return acc;
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
// permutes them visibly) + thin border lines at the wrap seams. Globe.hlsl carries the
// SAME function for the PS half of the two-color test; the two must stay identical.
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
    const float2 xz = t.orgXZ + (float2(id.xy) + 0.5f) * t.texelM;

    // Corner-lerped spatial context (the CPU sampled the atlas stacks at the corners; a tile
    // spans well under the tide's or the wave grid's own resolution, so bilinear is honest
    // for the LEVEL. The BED is not that smooth -- and with M7p's shoaling and breaking the
    // corner-lerp quantized the surf geography to ~600 m patches that CUT at tile edges
    // (the data lens showed it; the user called it). M7q: per-texel bed from the COMPOSED
    // HEIGHT WINDOW, residency-clamped, corner-lerp as the out-of-window fallback.
    const float level = lerp(lerp(t.lvl00, t.lvl10, f.x), lerp(t.lvl01, t.lvl11, f.x), f.y);
    float bed = lerp(lerp(t.bed00, t.bed10, f.x), lerp(t.bed01, t.bed11, f.x), f.y);
    if (gSlotsD.x != 0xFFFFFFFFu && gSlotsD.z != 0xFFFFFFFFu) {
        // M9ax: the whole tenant -- the z14 page where it is resident and fine, the cube face
        // everywhere else on the planet -- so shoaling, the current amplification and the
        // depth-limited breaking act on every coast the rings reach, not only inside one page.
        // The rings are held to page mips >= 2 (their own texels are 1.2 m and up). The corner
        // lerp above remains only for a bank with no height tenant at all.
        const float lat = gGeoA.x + xz.y * gGeoA.z;
        const float lon = gGeoA.y + xz.x * gGeoA.w;
        bed = HpHeightAt(gTA[gSlotsD.x], gTA[gSlotsD.y], lat, lon, gWinA, gSlotsD.z, 2.0f);
    }

    // The SWE refinement where the solver is resident: dEta on the level, solved currents.
    float dEta = 0.0f;
    float2 cur = 0.0f;
    if (gSwe.z > 0.0f) {
        const float2 uv = (xz - gSwe.xy) * gSwe.zw;
        if (all(uv > 0.001f) && all(uv < 0.999f)) {
            const float2 texel = float2(uv.x * gSweDims.x, (1.0f - uv.y) * gSweDims.y);
            dEta = LoadBilinearClamp(gSlotsA.w, texel, gSweDims.xy).x;
            const float4 s = LoadBilinearClamp(
                gSlotsB.x, float2(uv.x * gSweDims.x, (1.0f - uv.y) * gSweDims.y),
                gSweDims.xy);
            if (s.w > 0.5f) cur = s.xy;
        }
    }
    // --flat-bed: substitute a constant floor AFTER every real sample, so the only thing that
    // changes between the two runs is the bed itself -- same window, same residency, same
    // solver, same instant.
    if (gDebugA.x != 0.0f) bed = gDebugA.y;
    const float lvl = level + dEta;
    const float depth = lvl - bed;
    const float dry = smoothstep(0.05f, 0.65f, depth);

    // M7j: THE SWELL SHADOW. The solver's line-of-sight exposure field (CPU march toward
    // the peak-wave source) always sheltered the OLD renderer; one-water lost the edge
    // silently and whitecapped the harbor basin -- the GA AST's orphan rule exists because
    // of this bug. Ocean bands fold by exposure; sigma^2 rides the same amplitude-squared
    // law; local chop keeps a floor. The shadow shares the SWE window's frame (row 0 =
    // north), so the same (1 - v) flip applies.
    float expo = 1.0f;
    if (gSlotsC.y != 0xFFFFFFFFu && gSwe.z > 0.0f) {
        const float2 suv = (xz - gSwe.xy) * float2(gSwe.z, gSwe.w);
        if (all(suv > 0.001f) && all(suv < 0.999f)) {
            expo = max(LoadBilinearClamp(gSlotsC.y,
                                         float2(suv.x * 160.0f, (1.0f - suv.y) * 160.0f),
                                         float2(160.0f, 160.0f)).x,
                       0.18f);
        }
    }

    // M8: the solved wave field's window weight -- inside it the solved field OWNS the
    // structure-bearing bands (cascades 0-1 yield by (1 - wWin)); the chop band and the
    // foam machinery stay local. Feathered so the handover is invisible (every rung
    // earns its place and VANISHES where it cannot -- the M7h symmetric-ladder doctrine).
    float wWin = 0.0f;
    float2 wcell = 0.0f;
    if (gWaveU.x != 0xFFFFFFFFu) {
        wcell = (xz - gWaveA.xy) * gWaveA.z;
        const float2 dimsW = float2(gWaveU.y, gWaveU.z);
        const float eM =
            min(min(wcell.x, dimsW.x - wcell.x), min(wcell.y, dimsW.y - wcell.y)) /
            gWaveA.z;
        wWin = smoothstep(0.0f, max(gWaveA.w, 1.0f), eM);
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
    float gain0 = t.hsScale * expo;
    float gain1 = t.hsScale * expo;
    float gain2 = t.hsScale * expo;
    [unroll] for (uint c = 0; c < 3; ++c) {
        // M9c: the fold judges the band by the wavelength its ENERGY actually sits at.
        const float lam = 6.2831853f / gBandKFold[c];
        const float w = 1.0f - smoothstep(lam * 0.12f, lam * 0.5f, t.texelM);
        // M7p: the two ORPHANED PHYSICS EDGES, restored from the retired SeaLayer path
        // and found by the 2D proof figure: SHOALING (Green's-law growth as the group
        // speed drops entering shallow water) and WAVE-CURRENT amplification (the ebb
        // standing the entrance up toward blocking -- the 7-foot-standing-wave term).
        // proofs/inlet_storm.py runs the SAME pure functions on the SAME fields; the
        // match report holds this kernel to it.
        float amp = t.hsScale * expo;
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
        if (c != 2) amp *= 1.0f - wWin;
        const float2 cuv = frac(xz / gPatch[c]);
        const float4 s = LoadBilinearWrap(gSlotsA[c], cuv, 256.0f);
        d += s.xyz * (w * amp);
        // The Jacobian foam lives in the DERIV fiber (the disp fiber's w is zero --
        // the old additive term here read it and contributed nothing since M7).
        const float4 dv = LoadBilinearWrap(gSlotsE[c], cuv, 256.0f);
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
        const float2 dimsW = float2(gWaveU.y, gWaveU.z);
        float3 dW = 0.0f;
        float sigW = 0.0f;
        [loop] for (uint s = 0; s < gWaveU.w; ++s) {
            const float4 sc = gWaveScale[s >> 1];
            const float aMax = (s & 1) ? sc.z : sc.x;
            if (aMax <= 0.0f) continue;
            const float kMax = (s & 1) ? sc.w : sc.y;
            const float4 t4 = WaveSample(gWaveU.x, wcell, s, dimsW);
            // The swell shadow shelters the SOLVED bands exactly as it does the
            // cascades (the helm-in-the-lee shot exposed the asymmetry: solved comps
            // sailed through the jetty's lee unsheltered).
            const float aW = t4.x * aMax * expo;
            if (aW < 1e-4f) continue;
            const float kW = max(t4.y * kMax, 1e-4f);
            const float lamW = 6.2831853f / kW;
            const float wF = 1.0f - smoothstep(lamW * 0.12f, lamW * 0.5f, t.texelM);
            // Shed steepness rides the MICHE cap (ak <= 0.44): the limiter bounds HEIGHT
            // by depth, but an opposing current grows k unbounded while a stays -- the
            // raw (a k)^2 shed painted arrested zones as a white sigma^2 wash stepping
            // at ring boundaries. A wave steeper than the limit has BROKEN (the excess
            // gate already turns that energy into foam); the glint keeps only what a
            // real sea can carry. Total capped below -- both closures, ALGEBRA.md.
            const float akS = min(aW * kW, gFoamA.y);
            sigW += (1.0f - wF) * 0.5f * akS * akS;
            if (wF <= 0.001f) continue;
            float2 sp = t4.zw * 2.0f - 1.0f;
            sp /= max(length(sp), 1e-4f);          // the spinor stays unit (cl2 law)
            const float4 rr = gWaveSig[s >> 1];
            const float2 rot = (s & 1) ? rr.zw : rr.xy;
            const float cT = sp.x * rot.x + sp.y * rot.y;   // cos(phi - sigma t)
            const float sT = sp.y * rot.x - sp.x * rot.y;   // sin(phi - sigma t)
            const float4 dd = gWaveDir[s >> 1];
            const float2 dir2 = (s & 1) ? dd.zw : dd.xy;
            dW.y += wF * aW * cT;
            dW.xz -= gWaveB.z * wF * aW * sT * dir2;
        }
        const float4 env = WaveSample(gWaveU.x, wcell, 16u, dimsW);
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
    const float envRms = lerp(envRmsC, max(rmsW * gPatch.w, 1e-4f), wWin);
    // The breaking indicator is PHYSICAL: divide the display exaggeration back out
    // (the exaggerated envelope inflated depth foam ~15% everywhere -- part of the
    // "rapids" look the user called; the solved excW is physical by construction).
    const float excessC =
        (envRmsC / max(gPatch.w, 1e-3f)) * 2.8284271f / (0.60f * max(depth, 0.05f));
    const float excess = lerp(excessC, excW, wWin);
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

    // Depth-limited breaking (the Sea.hlsl clamp, bank-side): the GEOMETRY constraint
    // stays; its foam side-effect retired in M8 -- the envelope-based depthFoam above is
    // the disciplined statement of the same physics (test the envelope, never |eta|).
    const float hmax = 0.55f * max(depth, 0.05f);
    if (abs(d.y) > hmax) {
        d.y *= hmax / abs(d.y);
        d.xz *= 0.85f;
    }

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
