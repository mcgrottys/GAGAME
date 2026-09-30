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
    float4 gSnap;       // xy = grid centre (camera XZ snapped to cascade-0 texels),
                        // z = M9bh --pixel-water (shade in PsMain, not DsMain), w unused
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
    // M6i: the composed channels + the survey land masks the sea consults (the bed outside
    // the survey, land classification) -- the SAME planet description every other layer
    // reads -- are Common.hlsli's SurfaceCb (b2) since M12 step 4g.
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
// PRIORS 1: this runs in the DOMAIN shader too, and a bindless SampleLevel outside the pixel
// stage returns ZERO on this GPU (the first version did exactly that: the node said 0.85 at
// the helm and the near field lay flat). Loads, manual bilinear, like the bank kernel.
float SweShadow(float2 xz) {
    if (gSweU.w == 0xFFFFFFFFu || gChurnU.w == 0xFFFFFFFFu) return 1.0f;
    const float2 uv = CsWindowUv(SeaPlanetDir(xz));
    if (any(uv < 0.0f) || any(uv > 1.0f)) return 1.0f;
    const uint slice = gCsU6.z;
    const int2 rc = int2(clamp(uv * 128.0f, 0.0f, 127.0f));
    const float haveB = gTexArr[gChurnU.w].Load(int4(rc, int(slice), 0)).x * 15.9375f;
    if (haveB > 7.5f) return 1.0f;
    const int mip = int(max(round(haveB), 3.0f));
    const float dim = 16384.0f / exp2(float(mip));
    const float2 tf = uv * dim - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    const int2 i0 = clamp(int2(t0), int2(0, 0), int2(dim - 1.0f, dim - 1.0f));
    const int2 i1 = clamp(int2(t0) + 1, int2(0, 0), int2(dim - 1.0f, dim - 1.0f));
    const float a = gTexArr[gSweU.w].Load(int4(i0.x, i0.y, int(slice), mip)).x;
    const float b = gTexArr[gSweU.w].Load(int4(i1.x, i0.y, int(slice), mip)).x;
    const float c = gTexArr[gSweU.w].Load(int4(i0.x, i1.y, int(slice), mip)).x;
    const float d = gTexArr[gSweU.w].Load(int4(i1.x, i1.y, int(slice), mip)).x;
    return lerp(lerp(a, b, fr.x), lerp(c, d, fr.x), fr.y);
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
    // M9bg: the surface's finished colour, shaded in the DOMAIN shader (this mesh's per-vertex
    // stage) and INTERPOLATED across the triangle. PsMain only writes it out.
    float3 col : TEXCOORD5;
};

// ---- M9bg: THE SEA, VERTEX-SHADED --------------------------------------------------------
// The user's contract: "old school vertex shaded water keeping the nice geometry". This is
// the SeaLayer's half of it (the globe's half is Globe.hlsl WaterVertexColor); it runs in the
// DOMAIN shader, which is this surface's per-vertex stage once the hardware has tessellated.
//
// The geometry is untouched -- the same screen-space edge factors, the same three cascades,
// the same depth-limited breaking clamp. What retired is everything that used to PAINT the
// water per pixel: the composed imagery bed, the churn atlas, the globe-convergence block
// with its ComposedHeight/ComposedColor lod cascade, the residency visualiser. The only reads
// left are the cascade DERIVATIVE textures, which is where this vertex's wave NORMAL comes
// from -- the geometry's own source, sampled exactly where the displacement was.
float3 SeaVertexColor(float2 xz, float3 rel, float2 U, float2 adv, float depth, float dryGuard,
                      float shadow, float att, float brk, float distCam) {
    const float kFoamW[3] = {0.3f, 1.0f, 1.0f};
    float hx = 0, hz = 0, foam = 0, chopFoam = 0, blockC2 = 0;
    float sig2 = gBandSig.w;
    [unroll] for (uint c = 0; c < 3; ++c) {
        const float2 uvc = (xz - adv) / gPatchL[c];
        const float w = CascadeFade(c, distCam) * att;
        const float2 ab = BandScale(c, U, depth, shadow);
        const float4 dv = gTex[gDerivSrv[c]].SampleLevel(sLinearWrap, uvc, 0);
        hx += dv.x * w * ab.x;
        hz += dv.y * w * ab.x;
        sig2 += gBandSig[c] * (1.0f - w) * ab.x * ab.x * dryGuard * dryGuard;
        foam += dv.w * w * kFoamW[c] * saturate(ab.x);
        if (c == 2) {
            chopFoam = dv.w * w;
            blockC2 = ab.y;
        }
    }
    hx *= dryGuard * gWaveC.x;
    hz *= dryGuard * gWaveC.x;
    const float sm0 = length(float2(hx, hz));
    if (sm0 > 1.1f) {
        hx *= 1.1f / sm0;
        hz *= 1.1f / sm0;
    }
    // THE FOLD, AT VERTEX DENSITY: a glint lobe narrower than the triangle it lands on
    // interpolates into hard white facets. The sub-vertex sharpness sheds into the variance
    // (the same law the bands already obey), floored at the open-ocean Cox-Munk value.
    sig2 = max(sig2, 0.0260f);
    const float3 n = normalize(float3(-hx, 1.0f, -hz));
    const float3 v = normalize(-rel);

    // The body: the scattering asymptote, one Lambert term. No bed, no imagery.
    const float3 deep = gBscat.rgb / max(gSigmaW.rgb, 1e-4f);
    const float ndl = saturate(dot(n, gSunDir.xyz));
    float3 col = deep * (0.30f + 0.70f * ndl) * SUN_IRR_C;

    // The sky mirror, horizon-clamped (a steep shoaling face must read as horizon sky, not
    // the near-black a below-horizon ray would give).
    float3 refl = reflect(-v, n);
    refl.y = max(refl.y, 0.02f);
    const float f = 0.02f + 0.98f * pow(1.0f - saturate(dot(n, v)), 5.0f);
    // Daylight 1: this layer never told its gradient the hour (the globe's water does).
    col = lerp(col, SkyRadianceDirDiscless(refl, 1.0f), f);

    // The one glint: the same Cox-Munk lobe, evaluated once per vertex.
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

    // Whitecaps: the Jacobian foam, current blocking of the chop, and the depth-limited
    // breaking this vertex's own clamp reported. The churn atlas (a texture) is out.
    const float slopeMag = saturate(length(float2(hx, hz)) * 1.5f);
    const float currentFoam = blockC2 * (0.10f * CascadeFade(2, distCam) +
                                         0.55f * saturate(chopFoam * 3.0f + slopeMag * 0.6f));
    float shoreFoam = 0.0f;
    if (gBathyU.x != 0xFFFFFFFFu) {
        const float surf = saturate(brk * 6.0f);
        shoreFoam = saturate(brk * 1.5f + smoothstep(0.9f, 0.25f, depth) * 0.4f * surf) *
                    dryGuard * saturate(0.35f + chopFoam * 2.0f + slopeMag * 0.8f);
    }
    const float fm = min(saturate(foam * gSea.z + currentFoam + shoreFoam), 0.88f);
    col = lerp(col, float3(1.05f, 1.10f, 1.15f) * (0.55f + 0.6f * ndl), fm);

    return AerialPerspective(col, normalize(rel), distCam);
}

// ---- M9bh: THE ESTUARY SURFACE, PIXEL-SHADED -- glass over the bed --------------------------
// SeaVertexColor above is the shipped default and stays it. This is the --pixel-water look
// (gSnap.z), and it is the SeaLayer's half of the same argument the globe's WaterPixelColor
// makes: a colour interpolates, a RAY does not. What the eye sees through this water is the
// bed, reached by a refracted ray that leaves every pixel in its own direction -- and over the
// bar, where the bed is a metre away and the wave face is steep, two neighbouring pixels look
// at bed a metre apart. Gouraud cannot express that; it smears it, and the smear slides.
//
//   * the REFRACTED ray, Snell's rotor in closed form (ALGEBRA "cl3"), marched by 2 secant
//     steps against the CUDEM bed in the PHYSICS frame -- the rendered surface carries the
//     one-world curvature drop and the bed does not, so the drop is added back before the
//     cast or the ray lands metres deep in the wrong place;
//   * the bed wears the composed imagery AT THE RAY'S OWN LANDING POINT, not at the pixel's
//     -- that offset IS the refraction, and it is what makes a sandbar wobble under a wave;
//   * Beer-Lambert per channel over the real path (down along the ray + diffuse up) at the
//     scene's water extinction, so thin water is transparent and deep water collapses to the
//     b/sigma scattering asymptote the layer always drew;
//   * the REFLECTED ray, horizon-clamped, and the one Cox-Munk glint on the pixel normal, at
//     the SUB-RESOLVED floor -- SeaVertexColor has to floor sigma^2 at the open-ocean value
//     because a triangle cannot carry a sharper lobe. A pixel can. That is the glassiness.
//
// NO FOAM, by contract: the Jacobian whitecaps, the current-blocked chop, the depth-limited
// breaking foam and the churn atlas are all absent here. `brk` still limits the GEOMETRY in
// DsMain exactly as before -- the wave still stops standing up past 0.55h -- it simply no
// longer pays out as white. Calm or storm, this surface is glass.
float3 SeaPixelColor(float2 xz, float3 rel, float att, float depth, float dryGuard,
                     float distCam) {
    const float2 U = JetU(xz);
    const float2 adv = JetUAdvect(xz) * gWaveC.w;   // must match the DS sampling exactly
    const float shadow = SweShadow(xz);

    // The surface, at PIXEL rate: the same three cascades the domain shader displaced from,
    // sampled here instead of interpolated. Every band the footprint has folded away still
    // sheds into sigma^2 (M6t) -- the energy ledger does not care which stage reads it.
    float hx = 0.0f, hz = 0.0f;
    float sig2 = gBandSig.w;
    [unroll] for (uint c = 0; c < 3; ++c) {
        const float2 uvc = (xz - adv) / gPatchL[c];
        const float w = CascadeFade(c, distCam) * att;
        const float ab = BandScale(c, U, depth, shadow).x;
        const float4 dv = gTex[gDerivSrv[c]].SampleLevel(sLinearWrap, uvc, 0);
        hx += dv.x * w * ab;
        hz += dv.y * w * ab;
        sig2 += gBandSig[c] * (1.0f - w) * ab * ab * dryGuard * dryGuard;
    }
    hx *= dryGuard * gWaveC.x;
    hz *= dryGuard * gWaveC.x;
    // Shoaling gain can push sampled slopes past any real wave face; the over-steep facets
    // render as dark back-face speckle (M6t).
    const float sm0 = length(float2(hx, hz));
    if (sm0 > 1.1f) { hx *= 1.1f / sm0; hz *= 1.1f / sm0; }
    const float3 n = normalize(float3(-hx, 1.0f, -hz));
    const float3 v = normalize(-rel);
    const float3 dIn = -v;
    const float ndl = saturate(dot(n, gSunDir.xyz));

    // ---- RAY 2, REFRACTED: the rotor's closed form, then the cast onto the bed.
    const float etaR = 1.0f / 1.34f;
    const float ci = saturate(-dot(dIn, n));
    const float st2 = etaR * etaR * max(1.0f - ci * ci, 0.0f);
    const float3 tDir = normalize(etaR * dIn + (etaR * ci - sqrt(max(1.0f - st2, 0.0f))) * n);
    const float dd = max(depth, 0.04f);
    const float drop = dot(xz, xz) / (2.0f * 6371000.0f);   // back to the PHYSICS frame
    const float3 Ps = rel + gEyeRel.xyz + float3(0.0f, drop, 0.0f);
    const float muD = max(-tDir.y, 0.10f);
    float sP = dd / muD;
    [unroll] for (int it = 0; it < 2; ++it) {
        const float3 Pb = Ps + tDir * sP;
        sP = clamp(sP + (Pb.y - BedAt(Pb.xz)) / muD, 0.05f, 120.0f);
    }
    const float2 bedXZ = (Ps + tDir * sP).xz;

    // The body: the b/sigma asymptote, opened up toward the sunlit bed as the two-way
    // extinction thins. Where there is no survey there is no bed to see -- the asymptote is
    // the whole answer, which is exactly what open ocean looks like.
    float3 col = (gBscat.rgb / max(gSigmaW.rgb, 1e-4f)) * (0.30f + 0.70f * ndl) * SUN_IRR_C;
    if (gBathyU.x != 0xFFFFFFFFu) {
        const float3 T = exp(-gSigmaW.rgb * (sP + dd));
        float3 bedAlb = float3(0.42f, 0.38f, 0.28f);
        if (ComposedColorOn()) bedAlb = ComposedColor(SeaPlanetDir(bedXZ) CS_WALK_AT(SeaPlanetDir(bedXZ)));
        col = lerp(col, bedAlb * (0.35f + 0.75f * ndl) * SUN_IRR_C, T);
    }

    // ---- RAY 1, REFLECTED: horizon-clamped (a steep shoaling face must read as horizon sky,
    // not the near-black a below-horizon ray gives), DISCLESS -- the sun belongs to the lobe.
    // M9bj: THE MIRROR SEES SEA, NOT ONLY SKY -- the same law as Globe.hlsl WaterPixelColor.
    // A below-horizon reflected ray clamped back up to the horizon returns the sky's brightest
    // band, and at the helm that painted the whole sea white. The pixel is not one facet: sig2
    // is the fold's shed slope variance, a slope spread of sigma spreads the reflected ray by
    // 2 sigma, and the share of that spread pointing below the horizon sees WATER -- whose
    // radiance is `col`, right here, already computed. One expression, no branch; 1/2 exactly
    // at the horizon, 0 from overhead, so the orbital view is untouched.
    const float3 refl = reflect(dIn, n);
    const float spread = 2.0f * sqrt(max(sig2, 1e-6f));
    const float seaward = saturate(0.5f - 0.5f * refl.y / spread);
    const float f = 0.02f + 0.98f * pow(1.0f - saturate(dot(n, v)), 5.0f);
    const float3 rSky = normalize(refl + float3(0.0f, max(0.02f - refl.y, 0.0f), 0.0f));
    // ...and what the seaward share hits is another wave, which at ITS grazing angle is itself
    // a mirror: one more bounce of the same Schlick, |refl.y| being that hit's cosine. At the
    // horizon it returns 1 (the band stays pale, not green); straight down it returns 0.02 and
    // the endpoint is the water.
    const float3 skyLit = SkyRadianceDirDiscless(rSky, 1.0f);
    const float fresHit = 0.02f + 0.98f * pow(1.0f - saturate(-refl.y), 5.0f);
    col = lerp(col, lerp(skyLit, lerp(col, skyLit, fresHit), seaward), f);

    // ---- THE ONE GLINT, on the resolved normal at the sub-resolved floor.
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

    return AerialPerspective(col, normalize(rel), distCam);
}

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
    // comes back as foam through `brk`. An AMPLITUDE cap, so the whole displacement scales by the
    // one ratio (WaterBank.hlsl's law: continuous at the cap).
    const float hmax = 0.55f * max(depth, 0.05f);
    const float yAbs = abs(d.y);
    o.brk = 0.0f;
    if (yAbs > hmax) {
        o.brk = saturate((yAbs - hmax) / max(hmax, 0.2f));
        d *= hmax / yAbs;
    }

    // M6g one-world: the flat frame is the tangent at the estuary origin; the sea rides the
    // SPHERE, so far vertices take the curvature drop (28 m by the window edge). Physics stays
    // flat -- only the rendered position bends.
    const float drop = dot(xz, xz) / (2.0f * 6371000.0f);
    const float3 world = float3(xz.x + d.x, lvl + d.y - drop, xz.y + d.z);
    o.worldXZ = xz;
    o.sh = float2(depth, dryGuard);
    o.rel = world - gEyeRel.xyz;
    o.col = SeaVertexColor(xz, o.rel, U, adv, depth, dryGuard, shadow, o.att, o.brk, distCam);
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    return o;
}

// ------------------------------------------------------------------ shading

float4 PsMain(VsOut i) : SV_Target {
    // ---- M9bg: THE PIXEL STAGE DOES NO SHADING. The colour was computed at this triangle's
    // vertices (SeaVertexColor, in the domain shader) and interpolated here -- old-school
    // vertex shading, with the tessellated geometry left exactly as it was.
    //
    // The one thing that stays per pixel is not paint, it is the land/water QUESTION: outside
    // the surveyed window the ocean sheet simply must not be drawn over land, and that cut is
    // a classification, at the pixel's own resolution (M6i). Inside the window the SWE wet/dry
    // and the terrain depth own it, so the gate stays out of the estuary's way.
#if GA_BLOCK_RANKS
    // HIERARCHY 4.17: the chain, once for the pixel, for its land question (the bed's colour under
    // --pixel-water walks from the bed, where its ray lands).
    const WalkChain wc = CsWalk(SeaPlanetDir(i.worldXZ), CsPointOfDir(SeaPlanetDir(i.worldXZ)));
#endif
    {
        const float2 buv = (i.worldXZ - gBathyGeo.xy) * gBathyGeo.zw;
        const bool surveyed =
            gBathyU.x != 0xFFFFFFFFu && all(buv > 0.002f) && all(buv < 0.998f);
        if (!surveyed && ComposedHeightOn() &&
            ComposedIsLand(SeaPlanetDir(i.worldXZ) CS_WC, gSea.x - i.sh.x, gSea.x) &&
            i.sh.x < 0.75f) {
            discard;
        }
    }
    // M9bh: --pixel-water (gSnap.z) shades HERE instead, per pixel -- the refracted ray
    // that makes this water translucent cannot ride an interpolator. Default is the
    // interpolated domain-shader colour, byte for byte.
    if (gSnap.z > 0.5f) {
        return float4(SeaPixelColor(i.worldXZ, i.rel, i.att, i.sh.x, i.sh.y, length(i.rel)),
                      1.0f);
    }
    return float4(i.col, 1.0f);
}
