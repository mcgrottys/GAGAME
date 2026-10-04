// ================================================================================================
//  Common.hlsli - bindings shared by every product, plus the field-sampling indirection.
//  Carried from vqview-inlet, including its geometric-algebra block, which was written as
//  scaffolding there and becomes load-bearing here (M3 activates Mv2 / OkuboWeiss on real
//  current fields; the atlas kernels grow out of the same conventions).
//
//  NOTHING IN A SHADER SHOULD EVER NAME A TEXTURE DIRECTLY. Textures live in one unbounded array
//  indexed by a heap slot, and a product refers to data by FieldDesc index. That is what lets a
//  tiled-resource residency manager move data underneath the shaders without touching them.
// ================================================================================================
#ifndef GA_COMMON_HLSLI
#define GA_COMMON_HLSLI

// Mirrors ga::SceneConstants in src/render/Renderer.h. Keep the two in step.
cbuffer SceneCb : register(b0) {
    // ** row_major IS LOAD-BEARING. ** HLSL packs a float4x4 in a constant buffer COLUMN-major by
    // default, so a matrix uploaded in DirectXMath's native row-major layout arrives transposed and
    // mul(rowVector, M) silently computes the wrong product. The symptom is not garbage -- it is a
    // scene that still points roughly the right way but with the wrong field of view.
    row_major float4x4 gViewProj;
    float4   gSunDir;        // xyz unit toward the sun
    float4   gSigmaW;        // per-channel water extinction, 1/m
    float4   gBscat;         // per-channel backscatter, 1/m
    float4   gParams0;       // x time, y heightScale, z patchWidth m, w patchHeight m
    float4   gParams1;       // x depthScale, y aspect, z nearZ, w exposure
    float4   gEyeRel;        // camera position in scene-local metres
    float4   gCamRight;      // pre-scaled by tan(fovY/2) * aspect
    float4   gCamUp;         // pre-scaled by tan(fovY/2)
    float4   gCamFwd;
    float4   gViewport;      // w, h, 1/w, 1/h
    float4   gMisc;          // x water level (m above datum -- the tide), yz = the SUN's disc
                             // (cos of 1.15x and 0.85x its true angular radius, M9bi), w spare
    float4   gSkyLut;        // M13: the AIR's table and this eye's place in it:
                             // x = multiple-scattering slot (a number; -1 = none),
                             // y = the planet's radius, z = the eye's (metres), w spare
    // THE PLANET'S AIR (src/scene/Air.h, Atmosphere.hlsli AtmAirRows), appended at the END:
    float4   gAirRay;        // Rayleigh scattering rgb (1/m at the ground), scale height (m)
    float4   gAirMieS;       // aerosol scattering rgb, scale height
    float4   gAirMieE;       // aerosol extinction rgb, Henyey-Greenstein g
    float4   gAirOzo;        // ozone absorption rgb (at the tent's peak), the tent's centre (m)
    float4   gAirTop;        // the tent's half-width, the top of the air (m), ground albedo, gain
};

#define gTime        (gParams0.x)
#define gHeightScale (gParams0.y)
#define gPatchM      (gParams0.zw)
#define gNearZ       (gParams1.z)
#define gExposure    (gParams1.w)
#define WATER_Y_M    (gMisc.x)

// M10 THE DROSTE GAUGE. The sky and haze helpers below read the sun and the eye's height
// through these two names, so a shader that draws a level OTHER than the camera's own (Globe.hlsl:
// the inner and outer globes are the root seen from S^-k(C), shaded in their own frame) can hand
// them that level's values before including this file. Every other shader gets the scene's own,
// byte for byte.
#ifndef GA_SUN_DIR
#define GA_SUN_DIR (gSunDir.xyz)
#endif
#ifndef GA_EYE_Y
#define GA_EYE_Y (gEyeRel.y)
#endif
// ...and WHOSE sky it is: the zenith the sky gradient is measured from. Inside a twisted Droste
// level under realistic lighting the sky above is the ROOT's (the inner planet's own air is thin
// and, on its night side, dark), so the gradient runs from the root's zenith, not the level's.
// Every other shader: +y, and dot(d, (0,1,0)) is d.y exactly.
#ifndef GA_SKY_UP
#define GA_SKY_UP (float3(0.0f, 1.0f, 0.0f))
#endif

// Mirrors ga::FieldDesc in src/scene/FieldSet.h (64 bytes).
struct FieldDesc {
    float4 worldToUv;    // uv = worldXZ * xy + zw
    float4 valueScale;
    float4 valueBias;
    uint   srvIndex;
    uint   layout;
    uint2  _pad;
};

// FieldLayout, mirroring the C++ enum.
#define FIELD_SCALAR        0
#define FIELD_VECTOR2       1
#define FIELD_RGBA          2
#define FIELD_MULTIVECTOR2  3
#define FIELD_ROTOR3        4

static const float3 SUN_IRR_C = float3(1.350f, 1.283f, 1.161f);
// THE ONE SUN. SUN_IRR_C is the engine's unit of light: the radiance a white Lambertian surface
// facing the sun returns (E / pi). The sun's irradiance at the top of the air is therefore
// pi SUN_IRR_C, and it is the only light: the sky is the air's integral times it (Common.hlsli
// SkyAir), a surface is lit by it through the air to its point (SunAt) and by the sky's irradiance
// (SkyAmbient). Sky and ground in one unit, by construction.
static const float3 kSunE = 3.14159265f * SUN_IRR_C;

// A shader that needs root param 2's SRV for its OWN per-draw list (the globe's CDLOD node
// buffer) defines GA_NO_FIELD_BUFFER before including this file and binds t0/space0 itself;
// the FieldSet helpers below then compile out with it.
#ifndef GA_NO_FIELD_BUFFER
StructuredBuffer<FieldDesc> gFields : register(t0, space0);
#endif
Texture2D gTex[] : register(t0, space1);      // unbounded; needs resource binding tier 3
// M9h: the same descriptors seen as UINT. A residency map is R8_UINT -- "the finest level
// resident here" is an index, not a quantity, and reading it through the float view would
// return a normalized fraction instead of the level. Same heap, same slots, different
// interpretation; only the maps are ever read through this one.
Texture2D<uint> gTexU[] : register(t0, space4);
// M9j: the heap a FIFTH time, as Texture2DArray. A paged GA bank is one reserved array whose
// SLICES are pages of the shared (level, x, y) space, so a consumer samples page and level from
// ONE view -- which is what lets two pages composite without a second SRV or a branch on which
// source owns the pixel.
Texture2DArray gTexArr[] : register(t0, space5);
Texture3D gTex3D[] : register(t0, space2);    // M6c: the SAME heap as volumes (cloud banks)
TextureCubeArray gTexCubeArr[] : register(t0, space6);   // M9ap: slices 0..5 of a page tenant
TextureCube gTexCube[] : register(t0, space3);   // M6e: streamed planet surfaces + their
                                                 // residency-map cubes (read only cube slots)

SamplerState sLinearClamp : register(s0);
SamplerState sLinearWrap  : register(s1);
SamplerState sPointClamp  : register(s2);     // for fields that must NOT be filtered (GA.hlsli
                                              // NlerpRotor: rotor fields are point-sampled)
// M9z: anisotropic, for the streamed SURFACE at grazing angles. Used with Sample()'s min-LOD
// clamp form so the hardware picks the footprint while the residency floor still holds -- a
// miss must still degrade to the best RESIDENT ancestor, never to unmapped garbage.
// PIXEL SHADERS ONLY: Sample() needs derivatives. The height path stays on SampleLevel.
SamplerState sAniso       : register(s3);
// PHASE A1: s3 that WRAPS, for the eye's windows (placed modulo 16384, HIERARCHY 4.1). Their
// residency gathers take s1, the trilinear that wraps.
SamplerState sAnisoWrap   : register(s4);

#ifndef GA_NO_FIELD_BUFFER
// Bilinear. Correct for scalars, vectors and independent-channel packings.
float4 SampleField(uint idx, float2 worldXZ) {
    FieldDesc f = gFields[idx];
    float2 uv = worldXZ * f.worldToUv.xy + f.worldToUv.zw;
    float4 v = gTex[f.srvIndex].SampleLevel(sLinearClamp, saturate(uv), 0);
    return v * f.valueScale + f.valueBias;
}

float SampleField1(uint idx, float2 worldXZ) { return SampleField(idx, worldXZ).x; }
#endif  // GA_NO_FIELD_BUFFER

// Reversed-Z with an infinite far plane: ndcZ = nearZ / viewZ, so viewZ = nearZ / ndcZ.
float LinearDepthFromReversedZ(float ndcZ) { return gNearZ / max(ndcZ, 1e-9f); }

// World-space view ray for a pixel, from NDC in [-1,1]. No matrix inverse needed.
float3 ViewRay(float2 ndc) {
    return normalize(gCamFwd.xyz + ndc.x * gCamRight.xyz + ndc.y * gCamUp.xyz);
}

// ---- THE GATES' SLAB, ONCE (scene/Gateway.cpp SeenThroughFrom, line for line) -----------------
// The segment from the eye (the origin of the true camera frame) to p, walked from tStart on,
// enters the box at or before p: p is seen THROUGH the window. One window of a chain, packed as
// scene/WindowBox.h packs it: rows 0..2 carry a camera-frame vector into the box's own frame with
// the half extent in w, row 3 is the box's centre relative to the eye. Declared here because THREE
// passes ask it -- the globe (which world a surface pixel belongs to), the sky (whose sky a backdrop
// pixel belongs to) and the hulls -- and passes that disagreed about an edge would show a seam.
// With tStart 0 this is the single window's test exactly: tEnter clamped at 0 is <= tExit iff
// tEnter <= tExit and tExit >= 0.
bool GateSlabFrom(float3 p, float4 r0, float4 r1, float4 r2, float4 c, float tStart,
                  out float tIn) {
    const float3x3 R = float3x3(r0.xyz, r1.xyz, r2.xyz);
    const float3 h = float3(r0.w, r1.w, r2.w);
    const float3 e = mul(R, -c.xyz);
    float3 d = mul(R, p);
    d = lerp(d, float3(1e-12f, 1e-12f, 1e-12f), float3(abs(d) < 1e-12f));
    const float3 t1 = (-h - e) / d;
    const float3 t2 = (h - e) / d;
    const float3 tn = min(t1, t2);
    const float3 tf = max(t1, t2);
    // The ray is in this window's world only from where it entered the last one.
    const float tEnter = max(max(max(tn.x, tn.y), tn.z), tStart);
    const float tExit = min(min(tf.x, tf.y), tf.z);
    tIn = tEnter;
    return tEnter <= tExit && tEnter <= 1.0f;
}

// THE DEPTH OF A POINT: how many windows of the view's chain the segment from the eye to p passes,
// in order (scene/Gateway.cpp ChainDepth) -- the world p belongs to, 0 being the eye's own. A world
// keeps exactly the pixels of its own depth, so the windows within windows are one rule, and the
// view through each is the world behind glass: rasterized from the true eye, never a picture.
static const uint kWindowChain = 7u;
uint WindowChainDepth(float3 p, float4 boxes[28], uint n) {
    float t = 0.0f;
    uint k = 0u;
    const uint m = min(n, kWindowChain);
    [loop] for (; k < m; ++k) {
        const uint b = k * 4u;
        float tIn;
        if (!GateSlabFrom(p, boxes[b], boxes[b + 1u], boxes[b + 2u], boxes[b + 3u], t, tIn)) break;
        t = tIn;
    }
    return k;
}

// ---- THE SKY, ONCE (M13; one integral since 2026-10-05) ------------------------------------------
// Defined here so the sky layer, the globe's backdrop and every reflection cannot disagree. The
// sky is ONE line integral (Atmosphere.hlsli AtmRay): the light the air scatters toward the eye
// along the ray, from where the ray enters the air -- the eye itself, inside it -- to where it
// leaves it or meets the ground, under the one sun. The eye's position says only where the ray
// starts. There is no ceiling, no fallback and no second march: the helm's dome, the limb seen
// from orbit and the air over the disc are this one function, and the planet's air is rows
// (gAir*, src/scene/Air.h), never a branch.
//
// A shader that draws another viewpoint (the globe's gate window and Droste levels) hands its own
// zenith, sun and eye radius through the three macros, exactly as it already did for the sun; the
// integral then IS that place's sky, with nothing else to tell it.
#ifndef GA_SKY_EYE_R
#define GA_SKY_EYE_R (gSkyLut.z)
#endif
#include "Atmosphere.hlsli"

#define GA_AIR (AtmAirRows(gAirRay, gAirMieS, gAirMieE, gAirOzo, gAirTop))
static const uint kSkySteps = 10u;   // exponential from the segment's start (Atmosphere.hlsli AtmRay)

// THE AIR along a ray from a viewpoint -- its zenith `up`, its sun, its distance eyeR from the
// planet's centre -- to the exit, the ground or tMax, in the engine's radiance; T what survives
// over that segment, tGround where the ray met the ground (-1: it did not). With no table yet the
// slot is -1 and the orders past the first are not added: the same integral, one term fewer.
float3 SkyAir(float3 dir, float3 up, float3 sunDir, float eyeR, float tMax, out float3 T,
              out float tGround) {
    const AtmAir air = GA_AIR;
    float3 fms;
    return AtmRay(air, eyeR, dir, up, sunDir, kSkySteps, int(gSkyLut.x), gSkyLut.y, tMax, T, fms,
                  tGround) * kSunE;
}

// THE SKY along a ray, which has no end: the air; where it meets the ground, the ground's bounce
// seen through the air in front of it; and -- weighted by `disc` -- the SUN, drawn once at its
// true angular size (gMisc.yz, M9bi) and coloured by the same transmittance, which is why a
// setting sun is red. A ray the planet stops sees no sun.
float3 SkyAlong(float3 dir, float3 up, float3 sunDir, float eyeR, float disc) {
    float3 T;
    float tG;
    float3 L = SkyAir(dir, up, sunDir, eyeR, 3.0e38f, T, tG);
    if (tG >= 0.0f) {
        const AtmAir air = GA_AIR;
        L += T * AtmGroundBounce(air, eyeR, dir, up, sunDir, tG, gSkyLut.y) * kSunE;
    } else {
        L += disc * SUN_IRR_C * T * smoothstep(gMisc.y, gMisc.z, dot(dir, sunDir)) * 12.0f;
    }
    return L;
}

// ---- A SURFACE'S LIGHT, FROM THE SAME SUN AND THE SAME AIR --------------------------------------
// In the engine's unit (SUN_IRR_C = E / pi): a Lambertian albedo a under these returns
// a (SunAt cos + SkyAmbient). `up` is the point's own zenith, heightM its height above the sphere.
// The SUN at the point: the one sun through the air to it (Atmosphere.hlsli AtmSunT, the closed
// form every step of the sky's integral is lit by) -- why a sunset ground is red and dim.
float3 SunAt(float3 up, float heightM) {
    const float Rb = gSkyLut.y;
    return SUN_IRR_C * AtmSunT(GA_AIR, Rb + max(heightM, 0.0f),
                               clamp(dot(GA_SUN_DIR, up), -1.0f, 1.0f), Rb);
}
// The SKY's irradiance on a surface of normal n: the integral's radiance over the hemisphere,
// tabulated with the air (SkyLut.hlsl CsMultiScatter, the table's second half: a level surface at
// a height under a sun angle), tilted by the split-hemisphere (1 + n.up)/2. No table yet: none.
float3 SkyAmbient(float3 n, float3 up, float heightM) {
    if (gSkyLut.x < 0.0f) return float3(0.0f, 0.0f, 0.0f);
    const float Rb = gSkyLut.y;
    const float r = Rb + max(heightM, 0.0f);
    const float3 eHat = AtmFetch(uint(gSkyLut.x),
                                 AtmMsUv(r, clamp(dot(GA_SUN_DIR, up), -1.0f, 1.0f), Rb,
                                         Rb + GA_AIR.top), kAtmMsDims, 32);
    return SUN_IRR_C * eHat * (0.5f + 0.5f * dot(n, up));
}

// ---- THE PLANET'S OWN SHADOW, ONCE ----------------------------------------------------------
// The sun is one light, and at this scale the only thing that stands between it and the ground
// is the planet itself. So whether a surface point is lit is a question about the PLANET at that
// point, never about the surface's own tilt. The surfaces asked their tilt: a wave face leaning a
// few degrees toward a sun that set two degrees ago still read "lit", and the sea drew a white
// glint under a sky that had already put its sun out (--sun 236,-2 at the helm). The sky asks
// the planet (Atmosphere.hlsli AtmSunT); every sun term on a surface now asks it too.
//
// Seen from a point h above the sphere, the planet fills the sky below the dipped horizon, whose
// sine is sqrt(h (2R + h)) / (R + h) -- zero on the sphere. The sun is a disc, so the answer is
// not a switch: SunDiscClear is the share of the sun's disc clear of an occluder's rim when the
// disc's centre stands `sep` radians beyond it -- 1/2 with the centre on the rim, all of it one
// disc radius above, none of it one below. Globe.hlsl's PortalShadow asks exactly this of the
// Droste inner globe; PlanetShadow asks it of the sphere the point stands on. The separation is
// summed in sines, which differs from the angle by under 0.2 % within three degrees of the
// horizon -- and the ramp is 0 or 1 everywhere further out.
static const float kSunDiscR = 0.0047f;   // the sun's angular radius, ~0.27 deg
float SunDiscClear(float sep) { return saturate(sep / (2.0f * kSunDiscR) + 0.5f); }
// `up` is the point's own zenith and `heightM` its ground above the sphere (the sea stands at
// 0), both in the frame `sunDir` is in.
float PlanetShadow(float3 up, float3 sunDir, float heightM, float planetR) {
    const float h = max(heightM, 0.0f);
    return SunDiscClear(dot(up, sunDir) + sqrt(h * (2.0f * planetR + h)) / (planetR + h));
}

// The sky, asked with an EXPLICIT viewpoint: its zenith, its sun, its distance from the planet's
// centre, all in the frame the ray is in. The gate's window is the destination's sky -- a ray the
// gate carried to the other side of the planet, the one sun asked at ITS OWN PLACE (the ephemeris
// is evaluated there, in doubles, and handed here already in that frame).
// ** THE SUN MUST BE IN HERE ** -- vqview measured an entire HDR frame under 1.0 luminance when
// the reflection used a sky without it; no tonemapper can invent a missing highlight.
float3 SkyRadianceAt(float3 dir, float3 up, float3 sunDir, float eyeR) {
    return SkyAlong(dir, up, sunDir, eyeR, 1.0f);
}

// The sky from this shader's own eye (the dome; anything that mirrors it whole).
float3 SkyRadianceDir(float3 dir) {
    return SkyAlong(dir, GA_SKY_UP, GA_SUN_DIR, GA_SKY_EYE_R, 1.0f);
}

// With only an elevation to give (the haze's colour, a slope's skylight): the sky along the sun's
// own meridian at that elevation, from the eye.
float3 SkyRadiance(float ey) {
    const float3 up = GA_SKY_UP;
    const float3 m = GA_SUN_DIR - up * dot(GA_SUN_DIR, up);
    const float ml = length(m);
    const float3 sh = (ml > 1e-6f) ? m / ml : float3(1.0f, 0.0f, 0.0f);
    const float c = clamp(ey, -1.0f, 1.0f);
    return SkyAlong(normalize(up * c + sh * sqrt(max(1.0f - c * c, 0.0f))), up, GA_SUN_DIR,
                    GA_SKY_EYE_R, 0.0f);
}

// M6t: the sky WITHOUT the specular sun disc, for surfaces that carry their own explicit sun lobe
// (the unified water BRDF owns the sun through Cox-Munk at every scale; the mirror disc on top of
// a helm-tight lobe would count the sun twice). The aureole stays: the Mie lobe IS the halo.
//
// A REFLECTION IS SEEN FROM THE WATER, so the integral starts at the sea's surface (the planet's
// radius), under the zenith the shader hands it: from the helm that is the eye's own sky to a few
// metres, and from orbit it is still the sky above the sea -- the eye's own radius would put the
// mirror's ray above the air, and the sea would reflect black space.
//
// `day` is kept for its callers and no longer read: the gradient it dimmed is gone, and the
// integral carries the hour by itself -- the sky over a set sun is dark, or glowing, because the
// air is (ALGEBRA priors 45).
float3 SkyRadianceDirDiscless(float3 dir, float day) {
    return SkyAlong(dir, GA_SKY_UP, GA_SUN_DIR, gSkyLut.y, 0.0f);
}

// THE AIR IN FRONT OF A SURFACE: the sky's own integral (SkyAir) along the view ray from the eye to
// the surface point `range` away; the surface is seen through what survives of it. It replaces
// the haze's closed form (a 1.3 km scale height, a 6 km extinction and the sky's colour lerped in,
// with the hour passed beside it): the haze near the eye and the limb from orbit are one function
// of the ray, and the hour is the air's (a sun under the planet lights none of it).
float3 AerialPerspective(float3 col, float3 viewDir, float range) {
    float3 T;
    float tG;
    const float3 L = SkyAir(viewDir, GA_SKY_UP, GA_SUN_DIR, GA_SKY_EYE_R, range, T, tG);
    return col * T + L;
}

// M6i: the composed-surface constants -- the rows every shader samples a planet's composed
// channels through (Compose.hlsli). Mirrors ga::ComposedSurfaceCb (count rows on BOTH sides
// after any edit); filled by SurfaceFrame::Fill ALONE so no two layers can disagree about the
// math. M12 step 4g: declared ONCE, in SurfaceCb (b2) below, where the globe, the terrain,
// the sea and the GIS vectors each embedded them in their own cbuffer.
#define GA_COMPOSED_CB_ROWS \
    uint4  gCsU;    /* color cube SRV, color cube residency, window SRV, window residency */ \
    uint4  gCsU2;   /* height cube SRV + residency, height WINDOW SRV + residency */ \
    uint4  gCsU3;   /* M9ay survey MASK PAGES: array SRV, array residency, cube SRV, cube \
                       residency (r = water coverage, b = edited, a = surveyed) */ \
    float4 gCsF;    /* color cube on, window on, height on, planet radius (m) */ \
    float4 gCsG;    /* height cube max lod, height texel arc (rad), height window max lod, \
                       stencil overlay on */ \
    float4 gCsR0;   /* planet->tangent rotation rows (east / up / north) */ \
    float4 gCsR1; \
    float4 gCsR2; \
    float4 gCsGround; /* M12 step 4f: ground texel (m) at the cube's mip 0 (Lattice::GroundRes(0)); \
                         yzw spare (PHASE B3: the pages' grounds are gone) */ \
    uint4  gCsU5;   /* M9ap PAGES: colour array SRV, array residency SRV; zw spare (PHASE B3: \
                       the page slices). x == ~0 means the old three-tenant path. */ \
    uint4  gCsU6;   /* M9aq HEIGHT PAGES: height array SRV, array residency SRV; zw spare \
                       (PHASE B3). x == ~0: a height cube alone (Mars). */
// THE CAMERA'S EYE (plan_address.md), appended at the END: for a stage with only a direction
// (CsPointOfDir), the point VsOut.geo is relative to, in doubles on the CPU (SurfaceFrame::Fill).
// PHASE B3: the Mercator anchor's other rows (eyeA/E/N/U, eyePx) are deleted.
#define GA_COMPOSED_CB_EYE_ROWS \
    float4 gCsEyeT;  /* the eye in the tangent axes less (0, R, 0): the flat camera; w spare */
// PHASE A2: THE EYE'S WINDOWS, PER LEVEL (SurfaceFrame::Fill; ComposedSurfaceCb's last rows):
// slot s's rank k + 1 at row 5 s + k.
#define GA_COMPOSED_CB_WINDOW_ROWS \
    float4 gCsWinU[40]; /* PageTexelUv's planes U, V, W about slot s's own eye, anchored on the \
                           multiple of 16384 texels nearest it */ \
    float4 gCsWinV[40]; \
    float4 gCsWinW[40]; \
    float4 gCsWinO[20]; /* the box's origin less the anchor, in 16384s, two (slot, rank) a row */ \
    uint4  gCsWinS[10]; /* the slice of the colour and the mask, four a row */ \
    uint4  gCsWinK[2];  /* K, the ranks live, per slot, four a row */ \
    float4 gCsRankG[2]; /* rank k + 1's ground texel (m) at mip 0 */

// M12 step 4g: THE ONE SURFACE CONSTANT BUFFER, on the shared layout's b2 (Renderer.h): the
// frame loop fills ga::ComposedSurfaceCb once a frame through SurfaceFrame::Fill, RenderFrame
// pushes it once and binds it before any layer records, and every shader on the shared layout
// reads these rows from that one buffer. Include Compose.hlsli after Common.hlsli, as before;
// DxTest's parity gate holds this cbuffer against the C++ struct, row by row.
cbuffer SurfaceCb : register(b2) {
    GA_COMPOSED_CB_ROWS
    GA_COMPOSED_CB_EYE_ROWS
    GA_COMPOSED_CB_WINDOW_ROWS
};

// The geometric-algebra toolkit lives in GA.hlsli (M3 moved it out so compute shaders with
// their own root signatures can share it). Note for surface fields: the grade-2 part of
// grad(eta) is identically zero because curl(grad(f)) == 0 -- the algebra pays on CURRENT
// fields, which is exactly where M3 spends it.
#include "GA.hlsli"

#endif  // GA_COMMON_HLSLI
