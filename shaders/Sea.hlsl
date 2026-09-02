// ================================================================================================
//  M2..M5b: the sea surface.
//
//  M5b restructured this from a fixed 9.4 m vertex grid into vqview's HARDWARE-TESSELLATED
//  pattern: 64x64 four-point patches generated from SV_VertexID, per-edge factors from projected
//  screen length so triangle density is a SCREEN-space invariant (sub-metre near the camera,
//  coarse at the horizon), computed from each edge's two shared corners only so neighbouring
//  patches agree bit-for-bit and no crack can open. Displacement moved from VS to the domain
//  shader.
//
//  The same pass made the wave-current physics depth-aware: each cascade band's phase speed
//  comes from finite-depth dispersion at the LOCAL depth (the ebb blocks the swell itself over
//  the shallow bar -- the 7-foot standing waves), Green's-law shoaling grows amplitudes into
//  the shallows, and a depth-limited breaking clamp (|eta| <= 0.55 h) converts the excess into
//  foam instead of geometry.
// ================================================================================================
#include "Common.hlsli"
#include "Jet.hlsli"

cbuffer SeaCb : register(b1) {
    float4 gSea;        // x seaLevel (NAVD88 m), y gridSpanM, z foamIntensity, w skirtStart
    float4 gSnap;       // xy = grid centre (camera XZ snapped to cascade-0 texels), zw unused
    uint4  gDispSrv;    // xyz = displacement SRVs per cascade
    uint4  gDerivSrv;   // xyz = derivative SRVs per cascade
    float4 gPatchL;     // xyz = cascade patch sizes m, w = target tessellated edge, PIXELS
    float4 gFadeD;      // M6u: x = model Hs (far-field whitening). M9c: yzw now carry the
                        // FOLD's wavenumber per cascade (energy-weighted; gBandK keeps the
                        // cut mean for phase speed and shoaling). The per-cascade fade
                        // DISTANCES that lived here retired in M6t -- folds are
                        // footprint-based in CascadeFade.
    float4 gJet;        // x signed speed m/s (+flood -ebb), y half-width m, z seaward decay m,
                        // w enabled
    float4 gJetDir;     // xy = flood-toward unit, zw = ebb-toward unit (x east, z north)
    float4 gWaveC;      // x HEIGHT EXAGGERATION (look knob), yz = peak dir, w advect wrap t
    float4 gBandK;      // xyz = representative WAVENUMBER per cascade (rad/m); phase speed is
                        // derived per-vertex from the local depth
    uint4  gChurnU;     // x churn atlas SRV, y residency-mask SRV, z visualizer on, w exposure page residency SRV (M9ba)
    float4 gChurnF;     // xy = atlas world origin, z = 1/domain size, w = churn foam gain
    float4 gChurnF2;    // xy = tile world size (m), zw = tile count
    uint4  gBathyU;     // x = CUDEM heightfield SRV (0xFFFFFFFF = open-ocean mode)
    float4 gBathyGeo;   // world x0, z0, 1/sizeX, 1/sizeZ (row 0 of the texture = NORTH)
    uint4  gSweU;       // M5c: x eta SRV, y uv SRV, z solver on, w exposure PAGE array SRV (M9ba)
    float4 gSweF;       // xy = bathy grid dims, zw = 1 / eta-atlas PADDED dims
    float4 gSweG;       // x = prism-truncation current gain (the CUDEM window holds ~1/3 of the
                        // real tidal prism; the solver supplies the SHAPE, this ACT-calibrated
                        // gain restores the MAGNITUDE until the M6 domain widens), yzw unused
    float4 gBandSig;    // M6t: xyz = per-cascade mean-square slope (exaggeration baked),
                        // w = sub-resolved floor; xyz+w = the globe's Cox-Munk sigma^2(wind)
    // M6i: the composed channels + the survey land masks -- the sea consults the SAME planet
    // description every other layer does (bed outside the survey, land classification).
    GA_COMPOSED_CB_ROWS
};

#include "Compose.hlsli"

static const uint kPatches = 64;    // patches per side

// The world position's planet direction, one-world style (curvature drop + frame rows).
float3 SeaPlanetDir(float2 xz) {
    const float drop = dot(xz, xz) / (2.0f * gCsF.w);
    return CsToPlanet(normalize(float3(xz.x, -drop + gCsF.w, xz.y)));
}

// M5: the bottom. Inside the surveyed CUDEM window, the real bed (physics-grade). M6i: outside
// it, the COMPOSED HEIGHT CHANNEL -- the NE 15s shelf, ETOPO beyond -- instead of a pretend
// 30 m ocean. The sea now feels the real shelf everywhere it renders, and land is land.
float BedAt(float2 xz, out bool surveyed) {
    surveyed = false;
    // M9ar: THE BED IS THE HEIGHT MEGATEXTURE, inside the survey window too. "Surveyed" is
    // the survey's own footprint (the CUDEM grid the solver runs on); the height it returns
    // there is the z14 page at its resident mip, which IS the survey at 9.55 m/px. The
    // solver-private bank this used to sample no longer exists.
    if (gBathyU.x != 0xFFFFFFFFu) {
        const float2 uv = (xz - gBathyGeo.xy) * gBathyGeo.zw;
        if (all(uv > 0.002f) && all(uv < 0.998f)) surveyed = true;
    }
    if (ComposedHeightOn()) return ComposedHeight(SeaPlanetDir(xz), surveyed ? -8.0f : -2.0f);
    return -30.0f;
}
float BedAt(float2 xz) {
    bool s;
    return BedAt(xz, s);
}

float2 BathyUv(float2 xz) {
    return (xz - gBathyGeo.xy) * gBathyGeo.zw;   // y still south-up; flip when sampling
}

// M5c: the solved water-surface deviation from the analytic tide plane. The eta atlas is
// bathy-aligned but PADDED to tile multiples, so normalise by the padded dims; NULL tiles (land,
// or solver off) read the hardware zero = "the tide plane is exactly right".
float SweDEta(float2 xz) {
    if (gSweU.z == 0u) return 0.0f;
    const float2 uv = BathyUv(xz);
    if (any(uv < 0.001f) || any(uv > 0.999f)) return 0.0f;
    const float2 texel = float2(uv.x * gSweF.x, (1.0f - uv.y) * gSweF.y);
    const float d = gTex[gSweU.x].SampleLevel(sLinearClamp, texel * gSweF.zw, 0).x;
    // In the offshore sponge the deviation is numerical bookkeeping, not physics; the analytic
    // tide plane is the truth out there. Same ramp as the solver's sponge.
    return d * (1.0f - smoothstep(1400.0f, 2100.0f, xz.x));
}

// M5c: swell exposure -- 1 in open water, ~0.12 in the geometric shadow of the jetties and
// Plum Island (CPU line-of-sight march toward the peak-wave source, rebuilt when the wave
// direction or the water level moves).
// M9ba: the exposure is a PAGE (swell.exposure tenant, z14 slice, mips >= 3): nothing resident
// = no opinion = exposed. Same frame as the bed's page, same residency clamp.
float SweShadow(float2 xz) {
    if (gSweU.w == 0xFFFFFFFFu || gChurnU.w == 0xFFFFFFFFu) return 1.0f;
    const float2 uv = CsWindowUv(SeaPlanetDir(xz));
    if (any(uv < 0.0f) || any(uv > 1.0f)) return 1.0f;
    const float have = CsHavePage(gChurnU.w, uv, gCsU6.z);
    if (have > 7.5f) return 1.0f;
    return gTexArr[gSweU.w].SampleLevel(sLinearClamp, float3(uv, gCsU6.z), max(have, 3.0f)).x;
}

float2 JetU(float2 xz) {
    // Spatial split. INSIDE the estuary the solved field wins: the real channel's shape, the
    // islands, ebb/flood asymmetry. SEAWARD of the jetty tips the face-flux scheme has no
    // momentum advection, so its jet spreads and dies within metres of the funnel -- while the
    // real ebb jet coasts a mile offshore and is what stands the entrance swell up. Out there
    // the ACT-driven analytic jet (M3; it correlates 0.93 with the solved throat current) takes
    // over along a 400 m ramp. Retires when the solver gains momentum advection.
    const float2 ana = (gJet.w > 0.5f)
                           ? JetVelocity(xz, gJet.x, gJet.y, gJet.z, gJetDir.xy, gJetDir.zw)
                           : float2(0, 0);
    if (gSweU.z != 0u) {
        const float2 uv = BathyUv(xz);
        if (all(uv > 0.001f) && all(uv < 0.999f)) {
            const float4 s = gTex[gSweU.y].SampleLevel(sLinearClamp, float2(uv.x, 1.0f - uv.y), 0);
            if (s.w > 0.5f) {
                return lerp(s.xy * gSweG.x, ana, smoothstep(500.0f, 900.0f, xz.x));
            }
        }
    }
    return ana;
}

// Texture advection must stay SMOOTH: it multiplies by a large wrap time, so a spatially-varying
// current shears the cascade textures into spiral garbage. The analytic jet (or nothing) drifts
// the textures; the SWE field does the physics.
float2 JetUAdvect(float2 xz) {
    if (gJet.w < 0.5f) return float2(0, 0);
    return JetVelocity(xz, gJet.x, gJet.y, gJet.z, gJetDir.xy, gJetDir.zw);
}

// Per-band amplitude scale at a point: current amplification at the DEPTH-LOCAL phase speed,
// times shoaling, times swell shadowing. y = blocking fraction (feeds breaking foam).
// Shadowing applies fully to the travelling swell bands; the chop band is locally generated,
// so the lee only calms it partially.
float2 BandScale(uint c, float2 U, float depth, float shadow) {
    if (gJet.w < 0.5f && gBathyU.x == 0xFFFFFFFFu) return float2(1.0f, 0.0f);
    const float cBand = BandPhaseSpeed(gBandK[c], depth);
    float2 ab = (gJet.w > 0.5f) ? WaveCurrentAmp(U, gWaveC.yz, cBand) : float2(1.0f, 0.0f);
    ab.x *= (gBathyU.x != 0xFFFFFFFFu) ? ShoalFactor(gBandK[c], depth) : 1.0f;
    ab.x *= (c == 2) ? lerp(1.0f, shadow, 0.35f) : shadow;
    return ab;
}

// M6t: THE FOLD. A band is geometry while the pixel's ground footprint resolves its phase;
// past its Nyquist the phase is meaningless and the band sheds to grade 0 -- its variance
// continues in the glint lobe's sigma^2 (accumulated in PsMain), never deleted. Footprint-
// (not distance-) based, so zoom and resolution move the split the way they move texture LOD.
float PixFootM(float dist) {
    return dist * 2.0f * length(gCamUp.xyz) * gViewport.w;
}
float CascadeFade(uint c, float dist) {
    // M9c: the wavelength the band's ENERGY actually sits at (gFadeD.yzw), so this path
    // folds identically to the bank and the globe PS. Zero = a pre-M9c constant buffer.
    const float kF = (gFadeD[c + 1] > 1e-6f) ? gFadeD[c + 1] : gBandK[c];
    const float lam = 6.2831853f / kF;
    return 1.0f - smoothstep(lam * 0.12f, lam * 0.5f, PixFootM(dist));
}

// ------------------------------------------------------------------ tessellation chain

struct VsCtl {
    float2 xz : POSITION0;
    float att : TEXCOORD0;
};

VsCtl VsMain(uint vid : SV_VertexID) {
    // Four control points per patch from SV_VertexID; the outer ring stretches toward the
    // horizon (displacement fades to zero there via att).
    const uint patch = vid / 4;
    const uint corner = vid % 4;
    const uint2 cc = uint2(corner & 1, corner >> 1);
    const uint px = patch % kPatches;
    const uint pz = patch / kPatches;
    float2 p = float2(px + cc.x, pz + cc.y) / (float)kPatches - 0.5f;

    const float r = max(abs(p.x), abs(p.y)) * 2.0f;
    const float s0 = gSea.w;
    float stretch = 1.0f;
    if (r > s0) {
        const float t = (r - s0) / max(1.0f - s0, 1e-3f);
        stretch = 1.0f + 60.0f * t * t * t;
    }
    VsCtl o;
    o.xz = gSnap.xy + p * gSea.y * stretch;
    o.att = 1.0f - smoothstep(s0 * 0.8f, s0, r);
    return o;
}

struct HsPatch {
    float edges[4] : SV_TessFactor;
    float inside[2] : SV_InsideTessFactor;
};

float2 ScreenOf(float2 xz) {
    // Corners projected at STILL-water level so factors do not crawl with the waves (vqview's
    // lesson: a time-varying tessellation factor reads as the geometry breathing).
    const float4 c = mul(float4(xz.x - gEyeRel.x, gSea.x - gEyeRel.y, xz.y - gEyeRel.z, 1.0f),
                         gViewProj);
    const float w = max(abs(c.w), 0.25f);
    return (c.xy / w) * 0.5f * gViewport.xy;
}

// Only the edge's two shared corners: neighbouring patches compute bit-identical factors, so no
// crack can open (the vqview EdgeFactor contract).
float EdgeFactor(float2 a, float2 b) {
    const float px = min(length(ScreenOf(a) - ScreenOf(b)), 16384.0f);
    return clamp(px / max(gPatchL.w, 2.0f), 2.0f, 64.0f);
}

HsPatch PatchConst(InputPatch<VsCtl, 4> p) {
    HsPatch o;
    o.edges[0] = EdgeFactor(p[0].xz, p[2].xz);   // u = 0
    o.edges[1] = EdgeFactor(p[0].xz, p[1].xz);   // v = 0
    o.edges[2] = EdgeFactor(p[1].xz, p[3].xz);   // u = 1
    o.edges[3] = EdgeFactor(p[2].xz, p[3].xz);   // v = 1
    o.inside[0] = max(o.edges[1], o.edges[3]);
    o.inside[1] = max(o.edges[0], o.edges[2]);
    return o;
}

[domain("quad")]
[partitioning("fractional_even")]
[outputtopology("triangle_cw")]
[outputcontrolpoints(4)]
[patchconstantfunc("PatchConst")]
VsCtl HsMain(InputPatch<VsCtl, 4> p, uint i : SV_OutputControlPointID) {
    return p[i];
}

struct VsOut {
    float4 pos : SV_Position;
    float3 rel : TEXCOORD0;         // camera-relative world position
    float2 worldXZ : TEXCOORD1;     // pre-displacement plane position (cascade UV source)
    float  att : TEXCOORD2;         // skirt attenuation
    float  brk : TEXCOORD3;         // depth-limited breaking fraction at this vertex
    float2 sh : TEXCOORD4;          // x = water depth (m), y = dry guard
};

[domain("quad")]
VsOut DsMain(HsPatch hs, float2 uv : SV_DomainLocation, const OutputPatch<VsCtl, 4> p) {
    VsOut o;
    const float2 xz = lerp(lerp(p[0].xz, p[1].xz, uv.x), lerp(p[2].xz, p[3].xz, uv.x), uv.y);
    o.att = lerp(lerp(p[0].att, p[1].att, uv.x), lerp(p[2].att, p[3].att, uv.x), uv.y);
    const float distCam = length(xz - gEyeRel.xz);

    const float2 U = JetU(xz);
    const float2 adv = JetUAdvect(xz) * gWaveC.w;
    const float bed = BedAt(xz);
    // M5c: the local mean surface is the tide plane PLUS the solver's deviation -- the river's
    // standing slope upstream, the basin's lag, the set-up against the ebb.
    const float lvl = gSea.x + SweDEta(xz);
    const float depth = lvl - bed;
    const float shadow = SweShadow(xz);
    // Waves survive into genuinely shallow water (that is where they stand up); only the last
    // half-metre dries them out. The breaking clamp below does the physical limiting.
    const float dryGuard = smoothstep(0.05f, 0.65f, depth);

    float3 d = 0;
    [unroll] for (uint c = 0; c < 3; ++c) {
        const float2 uvc = (xz - adv) / gPatchL[c];
        const float w = CascadeFade(c, distCam) * o.att;
        const float s = BandScale(c, U, depth, shadow).x;
        float3 dc = gTex[gDispSrv[c]].SampleLevel(sLinearWrap, uvc, 0).xyz * w;
        dc.y *= s;
        dc.xz *= 0.7f + 0.3f * s;
        d += dc;
    }
    d *= dryGuard * gWaveC.x;   // look-side exaggeration applies before the breaking clamp

    // Depth-limited breaking: the surface cannot heave beyond ~0.55 h. What the clamp removes
    // comes back as foam through `brk`.
    const float hmax = 0.55f * max(depth, 0.05f);
    const float yAbs = abs(d.y);
    o.brk = 0.0f;
    if (yAbs > hmax) {
        o.brk = saturate((yAbs - hmax) / max(hmax, 0.2f));
        d.y *= hmax / yAbs;
        d.xz *= 0.85f;
    }

    // M6g one-world: the flat frame is the tangent at the estuary origin; the sea rides the
    // SPHERE, so far vertices take the curvature drop (28 m by the window edge). Physics stays
    // flat -- only the rendered position bends.
    const float drop = dot(xz, xz) / (2.0f * 6371000.0f);
    const float3 world = float3(xz.x + d.x, lvl + d.y - drop, xz.y + d.z);
    o.worldXZ = xz;
    o.sh = float2(depth, dryGuard);
    o.rel = world - gEyeRel.xyz;
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    return o;
}

// ------------------------------------------------------------------ shading

float4 PsMain(VsOut i) : SV_Target {
    const float distCam = length(i.rel);
    const float kFoamW[3] = {0.3f, 1.0f, 1.0f};
    const float2 Upx = JetU(i.worldXZ);
    const float2 adv = JetUAdvect(i.worldXZ) * gWaveC.w;   // must match the DS sampling exactly
    const float depth = i.sh.x;
    // M6i: OUTSIDE the surveyed window the survey mask classifies per pixel -- the ocean sheet
    // simply is not drawn over land (inside the window the SWE wet/dry + terrain depth own
    // this, so the gate stays out of the estuary's way).
    {
        const float2 buv = (i.worldXZ - gBathyGeo.xy) * gBathyGeo.zw;
        const bool surveyed =
            gBathyU.x != 0xFFFFFFFFu && all(buv > 0.002f) && all(buv < 0.998f);
        if (!surveyed && ComposedHeightOn() &&
            ComposedIsLand(SeaPlanetDir(i.worldXZ), gSea.x - depth, gSea.x) &&
            depth < 0.75f) {
            discard;
        }
    }
    const float shadow = SweShadow(i.worldXZ);

    float hx = 0, hz = 0, foam = 0, chopFoam = 0, blockC2 = 0;
    // M6t: the shed variance. Starts at the sub-resolved floor (capillary tail the FFT never
    // synthesises) and collects every band the footprint has folded, scaled by the same local
    // amplitude physics the geometry would have shown (shoaling, shadow, dry guard). At the
    // helm this is a tight glitter on resolved wave faces; at altitude it telescopes to the
    // globe's Cox-Munk sigma^2 -- the two water descriptions meet at the same pixel.
    float sig2 = gBandSig.w;
    [unroll] for (uint c = 0; c < 3; ++c) {
        const float2 uvc = (i.worldXZ - adv) / gPatchL[c];
        const float w = CascadeFade(c, distCam) * i.att;
        const float2 ab = BandScale(c, Upx, depth, shadow);
        const float4 dv = gTex[gDerivSrv[c]].SampleLevel(sLinearWrap, uvc, 0);
        hx += dv.x * w * ab.x;
        hz += dv.y * w * ab.x;
        sig2 += gBandSig[c] * (1.0f - w) * ab.x * ab.x * i.sh.y * i.sh.y;
        // Foam follows the band's LOCAL amplitude: a swell that never reaches the lee cannot
        // whitecap there.
        foam += dv.w * w * kFoamW[c] * saturate(ab.x);
        if (c == 2) {
            chopFoam = dv.w * w;
            blockC2 = ab.y;
        }
    }
    hx *= i.sh.y * gWaveC.x;
    hz *= i.sh.y * gWaveC.x;
    // M6t: cap the shaded slope at the limiting steepness -- shoaling gain can push sampled
    // slopes past any real wave face (they would be BREAKING; that physics is M7's), and the
    // over-steep facets rendered as dark back-faces speckling storm aerials.
    const float sm0 = length(float2(hx, hz));
    if (sm0 > 1.1f) {
        hx *= 1.1f / sm0;
        hz *= 1.1f / sm0;
    }
    const float3 n = normalize(float3(-hx, 1.0f, -hz));
    const float3 v = normalize(-i.rel);

    // Water colour: the b/sigma asymptote in deep water, blending toward sunlit bed as the
    // two-way extinction thins.
    const float3 deep = gBscat.rgb / max(gSigmaW.rgb, 1e-4f);
    const float ndl = saturate(dot(n, gSunDir.xyz));
    float3 col = deep * (0.30f + 0.70f * ndl) * SUN_IRR_C;
    if (gBathyU.x != 0xFFFFFFFFu) {
        const float dd = max(depth, 0.04f);
        const float3 T = exp(-2.2f * gSigmaW.rgb * dd);
        // M6n: the bed the thin water reveals is the IMAGERY's bed, not an analytic sand
        // tone. Flooded marsh shows brown marsh through centimetres of tide, sandbars show
        // sand, and the dry-guard's glassy sheet stops reading as grey SPECKLE from altitude
        // -- it reads as what is under it. (ComposedColor returns linear; lit like the old
        // constant so deep-water behaviour is untouched.)
        float3 bedAlb = float3(0.42f, 0.38f, 0.28f);
        if (ComposedColorOn()) bedAlb = ComposedColor(SeaPlanetDir(i.worldXZ));
        const float3 bedCol = bedAlb * (0.35f + 0.75f * ndl) * SUN_IRR_C;
        col = lerp(col, bedCol, T);
    }

    // Fresnel sky reflection -- the one shared sky, but DISCLESS (M6t): the sun's specular
    // now belongs entirely to the Cox-Munk lobe below, which owns it at EVERY scale (a
    // mirror disc here plus the lobe at helm-tight sigma^2 would count the sun twice).
    // Steep shoaling faces can send the reflection below the horizon; a real sea shows
    // spilling whitecaps there (steepness-limited breaking -- the M7 wavelets item). Until
    // then, CLAMP the ray to the horizon: those facets read as horizon sky (what water
    // actually mirrors at grazing), not the near-black that speckled storm aerials -- and
    // not the zenith blue an abs() mirror would give (tried; it traded black for teal).
    float3 refl = reflect(-v, n);
    refl.y = max(refl.y, 0.02f);
    const float f = 0.02f + 0.98f * pow(1.0f - saturate(dot(n, v)), 5.0f);
    col = lerp(col, SkyRadianceDirDiscless(refl), f);

    // M6u: THE OTHER HALF OF ONE WATER. M6t unified the ENERGY (glint sigma^2 telescopes to
    // Cox-Munk); the mode handoff still swapped the water's COLOR -- the globe paints its
    // ocean from the shelf-tinted albedo + composed imagery with NO near-field haze, the sea
    // painted a scattering asymptote under a hazed sky mirror, and at the switch the whole
    // ocean snapped (the user's frame pair, 0:16 vs 0:17). Convergence: above the estuary's
    // own altitudes this pixel evaluates THE GLOBE'S EXACT WATER FORMULA -- same composed
    // channels, same lod the globe would pick, same lighting constants -- and the sky mirror
    // and haze fade out with it. By the handoff band the two renderers emit the same pixel.
    const float kFar = smoothstep(700.0f, 2800.0f, gEyeRel.y);
    if (kFar > 0.0f) {
        const float3 dirP = SeaPlanetDir(i.worldXZ);
        const float pixAng = 2.0f * length(gCamUp.xyz) * gViewport.w;
        const float lodFar = ComposedHeightLod(distCam, pixAng);
        const float hp = ComposedHeightOn() ? ComposedHeight(dirP, lodFar) : -30.0f;
        const float shelf = saturate(1.0f + hp / 160.0f);
        float3 albSea = lerp(float3(0.013f, 0.055f, 0.115f), float3(0.06f, 0.30f, 0.34f),
                             shelf * shelf);
        albSea = lerp(albSea, float3(0.55f, 0.62f, 0.68f),
                      saturate((gFadeD.x - 2.5f) / 9.0f) * 0.55f);
        if (ComposedColorOn()) {
            const float3 img = ComposedColor(dirP);
            const float reveal = saturate(1.0f + min(hp, 0.0f) / 80.0f);
            const float3 albWater = albSea;
            albSea = lerp(albSea, img, 0.6f * reveal);
            albSea = SeafloorReliefMod(albSea, albWater, img, hp, 1.0f - reveal);   // M9av
        }
        const float ndlG = saturate(gSunDir.y);   // the globe lights water on upT; flat up = +y
        col = lerp(col, albSea * (0.030f + ndlG * SUN_IRR_C * 1.15f), kFar);
    }

    // M6t: THE ONE GLINT -- the globe's exact Cox-Munk lobe, on the RESOLVED normal, with
    // sigma^2 = floor + shed bands. Near: sharp glitter riding wave faces (the missing sun
    // glint at the helm). Far: n flattens, sigma^2 telescopes to Cox-Munk(wind), and this
    // expression becomes literally the globe shader's ocean specular. No pop, by construction.
    {
        const float3 hv = normalize(v + gSunDir.xyz);
        const float ch = saturate(dot(hv, n));
        const float t2 = max(1.0f - ch * ch, 0.0f) / max(ch * ch, 1e-4f);
        float glint = exp(-t2 / sig2) /
                      (4.0f * 3.14159265f * sig2 * max(ch * ch * ch * ch, 1e-4f));
        glint *= (0.02f + 0.98f * pow(1.0f - saturate(dot(v, hv)), 5.0f)) *
                 saturate(dot(gSunDir.xyz, n));
        col += glint * SUN_IRR_C * 0.85f;
    }

    // Whitecaps: wind-gated Jacobian foam + current blocking of the chop + depth-limited
    // breaking (the standing-wave faces over the bar) + churn memory.
    const float slopeMag = saturate(length(float2(hx, hz)) * 1.5f);
    // Blocked chop is scattered breaking wavelets, not paint: everything here rides the LIVE
    // chop texture, and the small always-on floor exists only within the chop band's own fade
    // range. (The M5c solved field correctly blocks a WIDE throat area; a structure-free 0.10
    // baseline over it rendered as a white slab from any aerial view.)
    const float currentFoam = blockC2 * (0.10f * CascadeFade(2, distCam) +
                                         0.55f * saturate(chopFoam * 3.0f + slopeMag * 0.6f));
    float shoreFoam = 0.0f;
    if (gBathyU.x != 0xFFFFFFFFu) {
        // M6n: the shallow-DEPTH foam term is gated by BREAKING ENERGY (i.brk), not depth
        // alone -- depth alone painted a 40% white wash across every acre of quietly flooded
        // marsh (the last layer of the "flats speckle"). Foam is surf: it appears where waves
        // actually break, and the sheltered creeks stay glassy over their imagery bed.
        const float surf = saturate(i.brk * 6.0f);
        shoreFoam = saturate(i.brk * 1.5f + smoothstep(0.9f, 0.25f, depth) * 0.4f * surf) *
                    i.sh.y * saturate(0.35f + chopFoam * 2.0f + slopeMag * 0.8f);
    }

    float churn = 0.0f;
    if (gChurnU.x != 0xFFFFFFFFu) {
        // M9az: the atlas is toroidal on the world lattice -- inside the window, sample at
        // frac(world / domain) with a WRAP sampler (the wrap line runs through the window).
        const float2 rel = (i.worldXZ - gChurnF.xy) * gChurnF.z;
        if (all(rel > 0.0f) && all(rel < 1.0f)) {
            churn = gTex[gChurnU.x].SampleLevel(sLinearWrap, i.worldXZ * gChurnF.z, 0).x;
        }
        // The memory says WHERE water is aerated; the chop's live texture says what it looks
        // like this instant -- without this modulation the throat renders as uniform fog.
        churn *= 0.5f + 0.5f * saturate(chopFoam * 3.0f + slopeMag);
        // Metre-scale froth decorrelates into the mean albedo once the footprint outgrows the
        // churn's 2 m texels (M6t: footprint-based like the wave bands -- zoom-aware).
        churn *= 1.0f - smoothstep(1.5f, 7.0f, PixFootM(distCam));
    }

    // Cap below 1 so even the worst breaking keeps a thread of water colour.
    const float fm = min(saturate(foam * gSea.z + currentFoam + shoreFoam + churn * gChurnF.w),
                         0.88f);
    col = lerp(col, float3(1.05f, 1.10f, 1.15f) * (0.55f + 0.6f * ndl), fm);

    // M6u: the globe applies NO near-field haze to its ocean below the space rim; the haze
    // fades with the same convergence or the handoff keeps an 18% pale step at 3.5 km.
    col = lerp(AerialPerspective(col, normalize(i.rel), distCam), col, kFar);

    // Residency visualizer (V): green = resident churn tiles, red grid = NULL.
    if (gChurnU.z != 0u && gChurnU.x != 0xFFFFFFFFu) {
        const float2 tuv = i.worldXZ / gChurnF2.xy;   // M9az: world tile lattice
        const float2 dgrid = abs(frac(tuv) - 0.5f);
        const float2 aa = fwidth(tuv) * 1.5f;
        const float gridLine = max(smoothstep(0.5f - aa.x, 0.5f, dgrid.x),
                                   smoothstep(0.5f - aa.y, 0.5f, dgrid.y));
        const float2 tc = frac((floor(tuv) + 0.5f) / gChurnF2.zw);   // the slot it aliases to
        const float resident = gTex[gChurnU.y].SampleLevel(sPointClamp, tc, 0).x;
        const float3 gridCol = (resident > 0.5f) ? float3(0.25f, 1.7f, 0.45f)
                                                 : float3(0.55f, 0.14f, 0.14f);
        col = lerp(col, gridCol, gridLine * (resident > 0.5f ? 0.6f : 0.15f));
        col += resident * float3(0.015f, 0.09f, 0.025f);
    }

    return float4(col, 1.0f);
}
