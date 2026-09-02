// ================================================================================================
//  HeightPages.hlsli -- M9ax: THE BED FROM THE HEIGHT PAGE TENANT, FOR COMPUTE.
//
//  The pixel stage resolves the planet's height with ComposedHeightPages (Compose.hlsli): the
//  z14 Mercator page by containment where its resident texel is at least as fine as the cube's,
//  else the cube face by direction, the residency map clamping the mip. The three kernels that
//  simulate on the bed -- the SWE solver, the churn, the wave bank -- each carried their own copy
//  of HALF that rule: the page only, with a wall / -30 m / a CPU corner lerp outside it
//  (AUDIT_WATER item 5, "the keyhole"). This is the whole rule, once, for any stage: manual
//  bilinear Loads, because a bindless SampleLevel outside the pixel stage returns zero
//  (ALGEBRA priors 1), on the tenant's full array view (slices 0..5 the cube faces, 6 the z14
//  page) and its residency-map array.
//
//  Frames (GaAst: height.pages -> {swe.solver, churn.kernel, water.bank}, merc-uv, no flip):
//  lat/lon -> Mercator px -> page uv is CsWindowUv's closed form; lat/lon -> direction ->
//  cube face uv is ComposeCubeDir's inverse (the D3D cube convention the tiles were painted in).
// ================================================================================================
#ifndef HEIGHT_PAGES_HLSLI
#define HEIGHT_PAGES_HLSLI

static const float kHpDim = 16384.0f;          // the tenant's page dim (cube faces and pages)
static const float kHpResDim = 128.0f;         // its residency map (R8, byte = finest mip * 16)
static const float kHpCubeTexelM = 611.0f;     // cube mip 0 at the equator
static const float kHpPageTexelM = 9.55f;      // z14 mip 0
static const float kHpMaxMip = 6.0f;           // 7 mips

// Direction (x = cos lat cos lon, y = sin lat, z = cos lat sin lon -- Compose.hlsli's
// convention) -> D3D cube face and its texture uv, the inverse of Compositor::ComposeCubeDir.
uint HpCubeFace(float3 d, out float2 uv) {
    const float3 a = abs(d);
    float s, t;
    uint face;
    if (a.x >= a.y && a.x >= a.z) {
        if (d.x > 0.0f) { face = 0; s = -d.z / a.x; t = -d.y / a.x; }
        else            { face = 1; s =  d.z / a.x; t = -d.y / a.x; }
    } else if (a.y >= a.z) {
        if (d.y > 0.0f) { face = 2; s =  d.x / a.y; t =  d.z / a.y; }
        else            { face = 3; s =  d.x / a.y; t = -d.z / a.y; }
    } else {
        if (d.z > 0.0f) { face = 4; s =  d.x / a.z; t = -d.y / a.z; }
        else            { face = 5; s = -d.x / a.z; t = -d.y / a.z; }
    }
    uv = float2(s, t) * 0.5f + 0.5f;
    return face;
}

// The finest resident mip at a uv of one slice (the map is conservative by construction: a
// byte per 128th of the page, the coarsest answer of the tiles it covers).
float HpHaveMip(Texture2DArray<float4> res, float2 uv, uint slice) {
    const int2 rc = int2(clamp(uv * kHpResDim, 0.0f, kHpResDim - 1.0f));
    return res.Load(int4(rc, int(slice), 0)).x * 15.9375f;
}

// Bilinear at one mip of one slice, by Loads.
float HpLoadBilinear(Texture2DArray<float4> arr, float2 uv, uint slice, float mip) {
    const float dim = kHpDim / exp2(mip);
    const float2 tf = uv * dim - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    float acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = clamp(int2(t0) + int2(k & 1, k >> 1), int2(0, 0),
                              int2(dim - 1.0f, dim - 1.0f));
        acc += ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y) *
               arr.Load(int4(tc, int(slice), int(mip))).x;
    }
    return acc;
}

// The planet's height (m NAVD) at lat/lon (degrees). winA = (page org px x, org px y,
// 1/kHpDim, world px at z14); winSlice the page's slice; pageMipMin the coarsest-allowed
// page mip a consumer wants to be held to (the bank rings ask 2, the solver 0).
float HpHeightAt(Texture2DArray<float4> arr, Texture2DArray<float4> res, float latDeg,
                 float lonDeg, float4 winA, uint winSlice, float pageMipMin) {
    const float latR = latDeg * 0.01745329252f;
    const float lonR = lonDeg * 0.01745329252f;
    const float cl = cos(latR);
    const float3 dir = float3(cl * cos(lonR), sin(latR), cl * sin(lonR));
    float2 cuv;
    const uint face = HpCubeFace(dir, cuv);
    const float haveC = clamp(round(HpHaveMip(res, cuv, face)), 0.0f, kHpMaxMip);
    // The page, by containment, where it is at least as fine as the cube. Two residency
    // reads decide; only ONE bilinear is paid (the bank kernel runs this per texel per ring).
    const float mx = (lonDeg + 180.0f) / 360.0f * winA.w;
    const float my = (0.5f - log(tan(0.7853981634f + latR * 0.5f)) * 0.15915494309f) * winA.w;
    const float2 wuv = float2(mx - winA.x, my - winA.y) * winA.z;
    if (all(wuv > 0.0f) && all(wuv < 1.0f)) {
        const float haveW = clamp(round(HpHaveMip(res, wuv, winSlice)), pageMipMin, kHpMaxMip);
        if (kHpPageTexelM * exp2(haveW) <= kHpCubeTexelM * exp2(haveC)) {
            return HpLoadBilinear(arr, wuv, winSlice, haveW);
        }
    }
    return HpLoadBilinear(arr, cuv, face, haveC);
}

#endif
