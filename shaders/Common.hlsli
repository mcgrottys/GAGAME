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
    float4   gMisc;          // x water level (m above datum -- the tide), yzw spare
};

#define gTime        (gParams0.x)
#define gHeightScale (gParams0.y)
#define gPatchM      (gParams0.zw)
#define gNearZ       (gParams1.z)
#define gExposure    (gParams1.w)
#define WATER_Y_M    (gMisc.x)

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
TextureCube gTexCube[] : register(t0, space3);   // M6e: streamed planet surfaces + their
                                                 // residency-map cubes (read only cube slots)

SamplerState sLinearClamp : register(s0);
SamplerState sLinearWrap  : register(s1);
SamplerState sPointClamp  : register(s2);     // for fields that must NOT be filtered; see below

#ifndef GA_NO_FIELD_BUFFER
float2 FieldUv(uint idx, float2 worldXZ) {
    FieldDesc f = gFields[idx];
    return worldXZ * f.worldToUv.xy + f.worldToUv.zw;
}

// Bilinear. Correct for scalars, vectors and independent-channel packings.
float4 SampleField(uint idx, float2 worldXZ) {
    FieldDesc f = gFields[idx];
    float2 uv = worldXZ * f.worldToUv.xy + f.worldToUv.zw;
    float4 v = gTex[f.srvIndex].SampleLevel(sLinearClamp, saturate(uv), 0);
    return v * f.valueScale + f.valueBias;
}

// Point-sampled. Use for rotor fields -- see the warning on NlerpRotor below.
float4 SampleFieldPoint(uint idx, float2 worldXZ) {
    FieldDesc f = gFields[idx];
    float2 uv = worldXZ * f.worldToUv.xy + f.worldToUv.zw;
    float4 v = gTex[f.srvIndex].SampleLevel(sPointClamp, saturate(uv), 0);
    return v * f.valueScale + f.valueBias;
}

float SampleField1(uint idx, float2 worldXZ) { return SampleField(idx, worldXZ).x; }

// Metres by which a world position lies OUTSIDE the field's extent, per axis, signed.
// Clamping a PHASE field at the edge is not a neutral extrapolation: the phase stops advancing.
// Amplitude and wavenumber clamp harmlessly; phase must be CONTINUED.
float2 FieldOverrun(uint idx, float2 worldXZ) {
    FieldDesc f = gFields[idx];
    const float2 uv = worldXZ * f.worldToUv.xy + f.worldToUv.zw;
    const float2 over = uv - saturate(uv);
    return over / f.worldToUv.xy;   // back to metres
}
#endif  // GA_NO_FIELD_BUFFER

// Reversed-Z with an infinite far plane: ndcZ = nearZ / viewZ, so viewZ = nearZ / ndcZ.
float LinearDepthFromReversedZ(float ndcZ) { return gNearZ / max(ndcZ, 1e-9f); }

// World-space view ray for a pixel, from NDC in [-1,1]. No matrix inverse needed.
float3 ViewRay(float2 ndc) {
    return normalize(gCamFwd.xyz + ndc.x * gCamRight.xyz + ndc.y * gCamUp.xyz);
}

// Sky radiance. Defined ONCE here so the sky layer and every reflection cannot disagree.
float3 SkyRadiance(float ey) { return lerp(SKY_LO_C, SKY_HI_C, smoothstep(0.0f, 0.30f, ey)); }

// The FULL directional form, sun included. Use this for anything that reflects the sky.
// ** THE SUN MUST BE IN HERE ** -- vqview measured an entire HDR frame under 1.0 luminance when
// the reflection used the gradient-only form; no tonemapper can invent a missing highlight.
float3 SkyRadianceDir(float3 dir) {
    float3 col = SkyRadiance(dir.y);
    const float cosA = dot(dir, gSunDir.xyz);
    const float disc = smoothstep(0.99985f, 0.99997f, cosA);
    const float halo = pow(saturate(cosA), 350.0f) * 0.35f + pow(saturate(cosA), 12.0f) * 0.05f;
    col += SUN_IRR_C * (disc * 12.0f + halo);
    // Below the horizon there is no sky, so darken rather than mirroring the horizon band.
    return lerp(col, SKY_LO_C * 0.45f, smoothstep(0.0f, -0.06f, dir.y));
}

// M6t: the sky WITHOUT the specular sun disc (halo kept -- that is scattered skylight, not
// the mirror image). For surfaces that carry their own explicit sun lobe: the unified water
// BRDF owns the sun through Cox-Munk at every scale, and the mirror disc here on top of a
// helm-tight lobe would count the sun twice.
float3 SkyRadianceDirDiscless(float3 dir) {
    float3 col = SkyRadiance(dir.y);
    const float cosA = dot(dir, gSunDir.xyz);
    col += SUN_IRR_C * (pow(saturate(cosA), 350.0f) * 0.35f + pow(saturate(cosA), 12.0f) * 0.05f);
    return lerp(col, SKY_LO_C * 0.45f, smoothstep(0.0f, -0.06f, dir.y));
}

// Aerial perspective: exponential extinction toward the sky colour along the view ray. The 6 km
// scale keeps mid-field wave contrast alive on the open sea; vqview's 2.5 km suited a 470 m
// scene.
float3 AerialPerspective(float3 col, float3 viewDir, float range) {
    // Height-integrated airmass (M6b): haze density falls off exp(-y/H), so the effective path
    // is the integral of density along the ray, not its raw length -- a helm-height horizontal
    // view keeps the sea-level look, while a view DOWN from 2.6 km no longer drowns the estuary
    // in fog (the pre-M6b constant-density version did exactly that the moment the camera could
    // fly). Closed form: L_eff = H/dy * (exp(-y0/H) - exp(-(y0+L*dy)/H)), dy != 0.
    const float H = 1300.0f;                       // haze scale height, m
    const float y0 = max(gEyeRel.y, 0.0f);
    const float dy = viewDir.y;
    float leff;
    if (abs(dy) < 1e-3f) {
        leff = range * exp(-y0 / H);
    } else {
        leff = (H / dy) * (exp(-y0 / H) - exp(-(y0 + range * dy) / H));
    }
    const float t = 1.0f - exp(-max(leff, 0.0f) / 6000.0f);
    return lerp(col, SkyRadiance(viewDir.y), saturate(t));
}

// M6i: the composed-surface constants -- 9 float4 rows a layer embeds in its OWN cbuffer to
// sample a planet's composed channels through Compose.hlsli. Mirrors ga::ComposedSurfaceCb
// (count rows on BOTH sides after any edit); filled by FillComposedCb ALONE so the globe and
// the terrain cannot disagree about the math. Include Compose.hlsli AFTER the cbuffer.
#define GA_COMPOSED_CB_ROWS \
    uint4  gCsU;    /* color cube SRV, color cube residency, window SRV, window residency */ \
    uint4  gCsU2;   /* height cube SRV + residency, height WINDOW SRV + residency */ \
    uint4  gCsU3;   /* GIS land masks (raster realizations of the survey vectors): x = window \
                       R8 in the shared Mercator frame, y = global R8 equirect */ \
    float4 gCsF;    /* color cube on, window on, height on, planet radius (m) */ \
    float4 gCsMerc; /* window org px x, org px y, 1/sizePx, full-world px at window zoom */ \
    float4 gCsG;    /* height cube max lod, height texel arc (rad), height window max lod, \
                       stencil overlay on */ \
    float4 gCsR0;   /* planet->tangent rotation rows (east / up / north) */ \
    float4 gCsR1; \
    float4 gCsR2; \
    uint4  gCsU4;   /* M7f: DETAIL color window (z17) SRV + residency, fine edit mask SRV */ \
    float4 gCsDet;  /* detail uv from window uv: offset xy, scale z; w = fine edit mask on */ \
    float4 gCsEd;   /* fine edit mask box in window uv: offset xy, scale zw */

// The geometric-algebra toolkit lives in GA.hlsli (M3 moved it out so compute shaders with
// their own root signatures can share it). Note for surface fields: the grade-2 part of
// grad(eta) is identically zero because curl(grad(f)) == 0 -- the algebra pays on CURRENT
// fields, which is exactly where M3 spends it.
#include "GA.hlsli"

#endif  // GA_COMMON_HLSLI
