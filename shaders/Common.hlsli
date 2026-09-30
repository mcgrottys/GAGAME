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

static const float3 SKY_HI_C = float3(0.200f, 0.360f, 0.600f);   // zenith radiance
static const float3 SKY_LO_C = float3(0.720f, 0.790f, 0.860f);   // horizon radiance
static const float3 SUN_IRR_C = float3(1.350f, 1.283f, 1.161f);

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

// ---- THE SKY, ONCE (M13) --------------------------------------------------------------------
// Defined here so the sky layer and every reflection cannot disagree -- and MARCHED, not looked
// up: the ray integrates the air from its own viewpoint under the one sun (Atmosphere.hlsli
// AtmRay). A shader that draws another viewpoint (the globe's gate window and Droste levels) hands
// its own zenith, sun and eye radius through the three macros, exactly as it already did for the
// sun; the march then IS that place's sky, with nothing else to tell it.
//
// Without the air's table -- a tool, a test, a pass that runs before the sky layer, or an eye
// above the air, where the limb shell owns the backdrop -- the shipped gradient answers, byte for
// byte. A missing sky is a wrong sky, not a black one.
#ifndef GA_SKY_EYE_R
#define GA_SKY_EYE_R (gSkyLut.z)
#endif
#include "Atmosphere.hlsli"

static const uint kSkySteps = 6u;   // exponentially spaced (Atmosphere.hlsli AtmRay)

// The marched sky belongs to an eye INSIDE the air. Past 60 km the from-space rim has faded in and
// the limb shell owns the backdrop (the atmosphere ledger: four disjoint terms), so the cut is
// that altitude and not the top of the air -- a viewpoint between the two would otherwise draw
// both.
static const float kSkyAirCeilingM = 60000.0f;
bool SkyAirOn(float eyeR) {
    return gSkyLut.x >= 0.0f && eyeR < gSkyLut.y + kSkyAirCeilingM;
}

// The air along a ray from a viewpoint: its zenith, its sun, its distance from the planet centre.
float3 SkyAirAt(float3 dir, float3 up, float3 sunDir, float eyeR) {
    float3 fms;
    const float Rb = gSkyLut.y;
    const float r = clamp(eyeR, Rb + 1.0f, Rb + kAtmTopM - 1000.0f);
    return AtmRay(r, dir, up, sunDir, kSkySteps, int(gSkyLut.x), Rb, Rb + kAtmTopM, fms) *
           kAtmGain;
}

// THE SUN'S OWN COLOUR through the air from a viewpoint -- why a setting sun is red.
float3 SunThroughAirAt(float3 up, float3 sunDir, float eyeR) {
    const float Rb = gSkyLut.y;
    const float r = clamp(eyeR, Rb + 1.0f, Rb + kAtmTopM - 1000.0f);
    return AtmSunT(r, clamp(dot(sunDir, up), -1.0f, 1.0f), Rb);
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

// The gradient that shipped, kept as the fallback and for callers with only an elevation to give
// (asked, when the air is on, along the sun's own meridian at that elevation).
float3 SkyRadiance(float ey) {
    if (SkyAirOn(GA_SKY_EYE_R)) {
        const float3 up = GA_SKY_UP;
        const float3 m = GA_SUN_DIR - up * dot(GA_SUN_DIR, up);
        const float ml = length(m);
        const float3 sh = (ml > 1e-6f) ? m / ml : float3(1.0f, 0.0f, 0.0f);
        const float c = clamp(ey, -1.0f, 1.0f);
        return SkyAirAt(normalize(up * c + sh * sqrt(max(1.0f - c * c, 0.0f))), up, GA_SUN_DIR,
                        GA_SKY_EYE_R);
    }
    return lerp(SKY_LO_C, SKY_HI_C, smoothstep(0.0f, 0.30f, ey));
}

// The FULL directional form, sun included. Use this for anything that reflects the sky.
// ** THE SUN MUST BE IN HERE ** -- vqview measured an entire HDR frame under 1.0 luminance when
// the reflection used the gradient-only form; no tonemapper can invent a missing highlight.
// The same radiance, asked with an EXPLICIT sun: the gate's window is the destination's sky, and
// the destination's sun is the one sun asked at ITS OWN PLACE (the ephemeris is evaluated there,
// in doubles, and handed here already in that frame) rather than this frame's sun turned around.
// A viewpoint that is not this frame's -- a ray the gate carried to the other side of the
// planet: its own zenith, sun and eye radius, all in the frame the ray is in. The same march.
float3 SkyRadianceAt(float3 dir, float3 up, float3 sunDir, float eyeR) {
    if (!SkyAirOn(eyeR)) {
        const float ey = dot(dir, up);
        float3 col = SkyRadiance(ey);
        const float cosA = dot(dir, sunDir);
        col += SUN_IRR_C * (smoothstep(gMisc.y, gMisc.z, cosA) * 12.0f +
                            pow(saturate(cosA), 350.0f) * 0.35f + pow(saturate(cosA), 12.0f) * 0.05f);
        return lerp(col, SKY_LO_C * 0.45f, smoothstep(0.0f, -0.06f, ey));
    }
    return SkyAirAt(dir, up, sunDir, eyeR) +
           SUN_IRR_C * SunThroughAirAt(up, sunDir, eyeR) *
               smoothstep(gMisc.y, gMisc.z, dot(dir, sunDir)) * 12.0f;
}

float3 SkyRadianceDir(float3 dir) {
    const float ey = dot(dir, GA_SKY_UP);
    if (SkyAirOn(GA_SKY_EYE_R)) return SkyRadianceAt(dir, GA_SKY_UP, GA_SUN_DIR, GA_SKY_EYE_R);
    float3 col = SkyRadiance(ey);
    const float cosA = dot(dir, GA_SUN_DIR);
    // M9bi: the disc is the sun's ACTUAL angular size (gMisc.yz, from the Earth-Sun distance
    // of this frame), not the two hand-picked cosines that stood here -- those spanned 0.44 to
    // 0.99 degrees against a true radius of 0.2666, so the sun was drawn 1.7x to 3.7x too wide.
    const float disc = smoothstep(gMisc.y, gMisc.z, cosA);
    const float halo = pow(saturate(cosA), 350.0f) * 0.35f + pow(saturate(cosA), 12.0f) * 0.05f;
    col += SUN_IRR_C * (disc * 12.0f + halo);
    // Below the horizon there is no sky, so darken rather than mirroring the horizon band.
    return lerp(col, SKY_LO_C * 0.45f, smoothstep(0.0f, -0.06f, ey));
}

// M6t: the sky WITHOUT the specular sun disc (halo kept -- that is scattered skylight, not
// the mirror image). For surfaces that carry their own explicit sun lobe: the unified water
// BRDF owns the sun through Cox-Munk at every scale, and the mirror disc here on top of a
// helm-tight lobe would count the sun twice.
//
// `day` is the caller's daylight, and only the GRADIENT takes it. The march already carries the
// hour -- the sky over a set sun is dark, or glowing, because the air is -- and the water that
// multiplied the march by its ramp of the sun's height (3 sin(el) + 0.12) darkened a twilight
// sea twice: it reflected 0.015 of the glow at -2 deg and 0.17 of it at +1. The gradient is the
// same two colours at noon and at midnight, so it is still told the hour: an eye above the air
// (the globe, a Droste level) keeps its night-side sea dark.
float3 SkyRadianceDirDiscless(float3 dir, float day) {
    const float ey = dot(dir, GA_SKY_UP);
    // The march carries the aureole (the Mie lobe IS the halo); what this form must not add is the
    // specular disc, and it does not.
    if (SkyAirOn(GA_SKY_EYE_R)) return SkyAirAt(dir, GA_SKY_UP, GA_SUN_DIR, GA_SKY_EYE_R);
    float3 col = SkyRadiance(ey);
    const float cosA = dot(dir, GA_SUN_DIR);
    col += SUN_IRR_C * (pow(saturate(cosA), 350.0f) * 0.35f + pow(saturate(cosA), 12.0f) * 0.05f);
    return lerp(col, SKY_LO_C * 0.45f, smoothstep(0.0f, -0.06f, ey)) * day;
}

// Aerial perspective: exponential extinction toward the sky colour along the view ray. The 6 km
// scale keeps mid-field wave contrast alive on the open sea; vqview's 2.5 km suited a 470 m
// scene.
// M10: AerialPerspectiveDay is the same haze with the hour in it. The light the haze scatters
// toward the eye is the SKY's, so it dims with the sky: at day = 1 it is AerialPerspective
// exactly (every existing caller keeps its bytes), at night the air still attenuates but adds
// no daylight.
float3 AerialPerspectiveDay(float3 col, float3 viewDir, float range, float day) {
    // Height-integrated airmass (M6b): haze density falls off exp(-y/H), so the effective path
    // is the integral of density along the ray, not its raw length -- a helm-height horizontal
    // view keeps the sea-level look, while a view DOWN from 2.6 km no longer drowns the estuary
    // in fog (the pre-M6b constant-density version did exactly that the moment the camera could
    // fly). Closed form: L_eff = H/dy * (exp(-y0/H) - exp(-(y0+L*dy)/H)), dy != 0.
    const float H = 1300.0f;                       // haze scale height, m
    const float y0 = max(GA_EYE_Y, 0.0f);
    const float dy = viewDir.y;
    float leff;
    if (abs(dy) < 1e-3f) {
        leff = range * exp(-y0 / H);
    } else {
        leff = (H / dy) * (exp(-y0 / H) - exp(-(y0 + range * dy) / H));
    }
    const float t = 1.0f - exp(-max(leff, 0.0f) / 6000.0f);
    return lerp(col, SkyRadiance(viewDir.y) * day, saturate(t));
}
float3 AerialPerspective(float3 col, float3 viewDir, float range) {
    return AerialPerspectiveDay(col, viewDir, range, 1.0f);
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
    float4 gCsMerc; /* window org px x, org px y, 1/sizePx, full-world px at window zoom */ \
    float4 gCsG;    /* height cube max lod, height texel arc (rad), height window max lod, \
                       stencil overlay on */ \
    float4 gCsR0;   /* planet->tangent rotation rows (east / up / north) */ \
    float4 gCsR1; \
    float4 gCsR2; \
    uint4  gCsU4;   /* M7f: DETAIL color window (z17) SRV + residency, fine edit mask SRV */ \
    float4 gCsDet;  /* detail uv from window uv: offset xy, scale z; w = fine edit mask on */ \
    float4 gCsGround; /* M12 step 4f: ground texel (m) at mip 0 -- cube, z14 window, z17 \
                         detail (Lattice::GroundRes(0)); w spare. Was gCsEd, dead since M9ay */ \
    uint4  gCsU5;   /* M9ap PAGES: colour array SRV, array residency SRV, window slice, \
                       detail slice. x == ~0 means the old three-tenant path. */ \
    uint4  gCsU6;   /* M9aq HEIGHT PAGES: height array SRV, array residency SRV, window \
                       slice. x == ~0 means the old cube + window tenants. */

// M12 step 4g: THE ONE SURFACE CONSTANT BUFFER, on the shared layout's b2 (Renderer.h): the
// frame loop fills ga::ComposedSurfaceCb once a frame through SurfaceFrame::Fill, RenderFrame
// pushes it once and binds it before any layer records, and every shader on the shared layout
// reads these rows from that one buffer. Include Compose.hlsli after Common.hlsli, as before;
// DxTest's parity gate holds this cbuffer against the C++ struct, row by row.
cbuffer SurfaceCb : register(b2) {
    GA_COMPOSED_CB_ROWS
};

// The geometric-algebra toolkit lives in GA.hlsli (M3 moved it out so compute shaders with
// their own root signatures can share it). Note for surface fields: the grade-2 part of
// grad(eta) is identically zero because curl(grad(f)) == 0 -- the algebra pays on CURRENT
// fields, which is exactly where M3 spends it.
#include "GA.hlsli"

#endif  // GA_COMMON_HLSLI
