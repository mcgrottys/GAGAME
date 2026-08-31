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
    float4 gWaveDir;    // xy = peak propagation dir (world unit), z = valid, w unused
    uint4  gSlotsD;     // x = height window SRV, y = its residency-map SRV (M7q)
    float4 gGeoA;       // world->latlon: orgLat, orgLon, 1/mPerLat, 1/mPerLon
    float4 gWinA;       // height window: org px x, org px y, 1/sizePx, full-world px z14
    uint4  gSlotsE;     // M8 foamlaw: cascade DERIV SRVs x3 (hx, hz, J, foam)
    float4 gRmsRef;     // M8: unit-sea rms envelope per band (xyz), w spare
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

// ---- M8 FOAM DISCIPLINE (ALGEBRA.md foamlaw; proofs/foam_discipline.py) ----
// Value noise in three octaves under IRRATIONAL rotations: axis-aligned octaves at 2x
// spacing share lattice seams and sum to rectangular blocks; irrational rotors leave no
// shared direction. The renormalization by the live fade sum keeps the MEAN exactly 1/2
// in every fade state -- without it foam coverage becomes a function of viewing scale.
float BankHash21(float2 p) {
    p = frac(p * float2(123.34f, 456.21f));
    p += dot(p, p + 45.32f);
    return frac(p.x * p.y);
}

float BankValueNoise(float2 p) {
    const float2 i0 = floor(p);
    const float2 f = frac(p);
    const float2 u = f * f * (3.0f - 2.0f * f);
    return lerp(lerp(BankHash21(i0), BankHash21(i0 + float2(1, 0)), u.x),
                lerp(BankHash21(i0 + float2(0, 1)), BankHash21(i0 + float2(1, 1)), u.x),
                u.y);
}

// The octave fade keys on the TILE TEXEL (the kernel's own resolution): an octave folds
// away once this ring cannot resolve its features -- the fold law in miniature. The fade
// band [0.25, 0.75] of the octave wavelength is a closure; the mean-preserving
// renormalization makes any fade choice coverage-neutral.
float BankFoamNoise(float2 posM, float texelM) {
    const float2x2 r1 = float2x2(0.8776f, -0.4794f, 0.4794f, 0.8776f);
    const float2x2 r2 = float2x2(0.6062f, -0.7952f, 0.7952f, 0.6062f);
    const float w1 = 1.0f - smoothstep(2.4f * 0.25f, 2.4f * 0.75f, texelM);
    const float w2 = 1.0f - smoothstep(6.5f * 0.25f, 6.5f * 0.75f, texelM);
    const float w3 = 1.0f - smoothstep(17.0f * 0.25f, 17.0f * 0.75f, texelM);
    const float n = 0.45f * BankValueNoise(posM * 0.42f) * w1 +
                    0.34f * BankValueNoise(mul(r1, posM) * 0.155f) * w2 +
                    0.21f * BankValueNoise(mul(r2, posM) * 0.059f) * w3;
    const float wsum = 0.45f * w1 + 0.34f * w2 + 0.21f * w3;
    return (wsum > 1e-3f) ? (n / wsum) : 0.5f;
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
    if (gSlotsD.x != 0xFFFFFFFFu) {
        const float lat = gGeoA.x + xz.y * gGeoA.z;
        const float lon = gGeoA.y + xz.x * gGeoA.w;
        const float latR = lat * 0.01745329252f;
        const float mx = (lon + 180.0f) / 360.0f * gWinA.w;
        const float my =
            (0.5f - log(tan(0.7853981634f + latR * 0.5f)) * 0.15915494309f) * gWinA.w;
        const float2 wuv = float2(mx - gWinA.x, my - gWinA.y) * gWinA.z;
        if (all(wuv > 0.002f) && all(wuv < 0.998f)) {
            // residency map: byte = finest resident mip * 16 (R8 UNORM)
            const float2 rdim = float2(128.0f, 128.0f);
            const float haveV =
                gT[gSlotsD.y][int2(clamp(wuv * rdim, 0.0f, rdim - 1.0f))].x;
            const float mip = clamp(round(haveV * 15.9375f), 2.0f, 7.0f);
            const float dim = 16384.0f / exp2(mip);
            const float2 tf2 = wuv * dim - 0.5f;
            const float2 t02 = floor(tf2);
            const float2 fr2 = tf2 - t02;
            float acc = 0.0f;
            [unroll] for (int k2 = 0; k2 < 4; ++k2) {
                const int2 tc2 = clamp(int2(t02) + int2(k2 & 1, k2 >> 1), int2(0, 0),
                                       int2(dim - 1.0f, dim - 1.0f));
                acc += ((k2 & 1) ? fr2.x : 1.0f - fr2.x) *
                       ((k2 >> 1) ? fr2.y : 1.0f - fr2.y) *
                       gT[gSlotsD.x].Load(int3(tc2, int(mip))).x;
            }
            bed = acc;
        }
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
        const float lam = 6.2831853f / gBandK[c];
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
            float2 ab = (gWaveDir.z > 0.5f) ? WaveCurrentAmp(cur, gWaveDir.xy, cB)
                                            : float2(1.0f, 0.0f);
            ab.x *= ShoalFactor(gBandK[c], depth);
            amp *= ab.x;
            blocked = ab.y;
        }
        if (c == 0) gain0 = amp;
        if (c == 1) gain1 = amp;
        if (c == 2) gain2 = amp;
        const float2 cuv = frac(xz / gPatch[c]);
        const float4 s = LoadBilinearWrap(gSlotsA[c], cuv, 256.0f);
        d += s.xyz * (w * amp);
        // The Jacobian foam lives in the DERIV fiber (the disp fiber's w is zero --
        // the old additive term here read it and contributed nothing since M7).
        const float4 dv = LoadBilinearWrap(gSlotsE[c], cuv, 256.0f);
        steepFoam = max(steepFoam, dv.w * w * saturate(amp));
        blockFoam = max(blockFoam, blocked * 0.35f * w * (c == 2 ? 1.0f : 0.4f));
        // shed variance: amplitude squared (gain included -- the far field sees the
        // steepened bar as a brighter glint band even when texels cannot draw it)
        sig2 += (1.0f - w) * amp * amp *
                (c == 0 ? 0.0004f : (c == 1 ? 0.0018f : 0.0060f));
    }
    d *= dry * gPatch.w;

    // The local rms ENVELOPE: unit-sea band rms scaled by this texel's own gains. The
    // depth-excess trigger tests it (never instantaneous |eta| -- television static),
    // and the crest gate normalizes eta by it (foam rides crests only: a Gaussian sea
    // passes ~22.6% of area through this gate; the strength noise breaks the rest).
    // (gRmsRef already carries the vertical exaggeration -- the CPU bakes it -- so it
    // is commensurate with d.y as written; no second gPatch.w here.)
    const float envRms =
        max(sqrt(gain0 * gain0 * gRmsRef.x * gRmsRef.x +
                 gain1 * gain1 * gRmsRef.y * gRmsRef.y +
                 gain2 * gain2 * gRmsRef.z * gRmsRef.z), 1e-4f);
    const float fn = BankFoamNoise(xz, t.texelM);
    const float excess = envRms * 2.8284271f / (0.60f * max(depth, 0.05f));
    const float depthFoam = smoothstep(1.05f, 1.95f, excess * (0.86f + 0.30f * fn));
    const float crest = smoothstep(0.28f, 0.80f, d.y / max(envRms, 1e-3f));
    float foam = saturate(max(steepFoam, max(depthFoam, blockFoam)) * crest) *
                 (0.55f + 0.75f * fn);

    // M7e/M8: THE FOAM MEMORY. The churn atlas remembers where water has been aerated
    // (breaking deposits advected by the solved current -- the seaward streaks off an
    // ebbing entrance). M8: churn is the SAME quantity remembered, so it composites by
    // MAX, never + (adding memory to fresh foam brightened the throat into uniform fog);
    // the live noise modulates the memory so old deposits stay textured, not flat.
    if (gSlotsC.x != 0xFFFFFFFFu) {
        const float2 cuv = (xz - gChurn.xy) * gChurn.z;
        if (all(cuv > 0.001f) && all(cuv < 0.999f)) {
            const float churnV =
                LoadBilinearClamp(gSlotsC.x, cuv * gChurn.w, gChurn.ww).x;
            foam = max(foam, saturate(churnV) * (0.5f + 0.5f * fn));
        }
    }

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
    gU[gSlotsB.z][dst] = float4(lvl, sig2, cur.x, cur.y);
    // The DETAIL plane: what the PS needs to recover sub-ring sparkle and the caustic
    // Jacobian -- per-band sea-state gains (cascade derivs are unit-sea) and the dry
    // guard (no sparkle on the flats). Layout: (gain1, dry, gain0, gain2).
    gU[gSlotsB.w][dst] = float4(gain1, dry, gain0, gain2);
}
