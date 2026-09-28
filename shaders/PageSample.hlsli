// ================================================================================================
//  PageSample.hlsli - M12 step 4e: THE ONE CONTRACT FOR READING A PAGE TENANT.
//
//  A page tenant (hal::Tenant) is one reserved Texture2DArray whose slices are pages of the
//  shared (level, x, y) space -- the cube's six faces (slices 0..5, read through the cube views,
//  hardware-seamless across faces) and the Mercator pages (the z14 window, the z17 detail) --
//  with a residency-map array beside it: R8, a byte per 128th of the page, the finest resident
//  mip times 16, conservative by construction (the coarsest answer of the tiles it covers).
//  Every consumer of one -- Compose.hlsli's pixel-stage colour, height and survey-mask reads;
//  the kernels that simulate on the bed (Swe, SeaChurn and WaterBank through HeightPages.hlsli)
//  and WaterBank's own exposure and wave-page reads -- shares what is stated here and nothing
//  else:
//
//    PageUv          the window uv of a planet DIRECTION (Compose.hlsli's CsWindowUv, moved);
//    PageUvLatLon    the same Mercator closed form from lat/lon in DEGREES -- the kernels'
//                    spelling (HeightPages.hlsli's, which WaterBank.hlsl repeated twice). TWO
//                    spellings and not one, because they are NOT bit-identical. Measured in
//                    float32, the shaders' ops in the shaders' order, over the solver's
//                    1863x1174 cells, the bank's six rings at the helm and a 4096^2 grid of the
//                    whole z14 page (step 4e, scratchpad/step4e_uv_ab.py): 10-13 % of the texels
//                    differ, by one to three EIGHTHS of a mip-0 texel -- the asin / atan2 round
//                    trip of the direction, and beneath it the `/ (2 pi)` against
//                    `* 0.15915494309f` (5 % on its own). A window pixel near 1.5e6 has a
//                    float32 ulp of 1/8 px, so every rounding difference lands on that grid.
//                    A consumer takes the spelling of what it HAS (a direction, or lat/lon); a
//                    kernel switched to the other would move its bed by up to 3.6 m of ground.
//    PageTexel       the texel of a FACE-PLANE window (HIERARCHY step 3), relative to its
//                    anchor: two planes through the body's centre over a third, evaluated at a
//                    point given relative to the eye, in whatever frame the CPU pulled the
//                    planes into (FaceWindow::PlanesIn, core/Lattice.h). The one large
//                    cancellation is the planes' w, taken in doubles there; everything added
//                    here is eye-relative, which is how float32 finds a rung-15 texel (1.9 cm)
//                    to 0.002 of itself within reach of the helm (tiletest, this GPU), where
//                    the direction's spelling is 29 texels off. Its CPU twin, op for op, is
//                    FaceWindow::PageTexel. Nothing reads it yet;
//    PageTexelUv     the same over the page's texels: a window's uv. The anchor is a multiple
//                    of the page, so a WRAP sampler's modulo puts every texel where the global
//                    lattice has it;
//    PageHave        the residency floor at a uv of one slice, PIXEL stage: a conservative
//                    GATHER + max -- never sample finer than any texel under the filter
//                    footprint (M6h: the bilinear ramp read unmapped tiles and mottled the marsh);
//    PageHaveLoad    the same floor by one Load, ANY stage (a bindless SampleLevel outside the
//                    pixel stage returns zero on this GPU: ALGEBRA priors 1);
//    PageLoad        bilinear at one mip of one slice by four Loads -- the kernels' read
//                    (PageLoad4: all four channels, the wave pages' spinor planes);
//    PageSample      the pixel-stage sample: the hardware's own footprint (anisotropic, from
//                    derivatives) with the residency floor as its min-LOD clamp, so a miss
//                    degrades to the best RESIDENT ancestor, never to unmapped garbage;
//    PageSampleLevel the explicit-lod sample at max(want, have) -- vertex- and mesh-safe;
//    PageGroundM     the ground texel (metres) a slice resolves at a resident mip: its mip-0
//                    ground times 2^have;
//    PageWins        THE CHOICE: a rung wins where its resident texel is at least as fine as
//                    the one held (the cube's, or a coarser page's), so the finest resident
//                    ground wins the ladder. No feather: the pages of one tenant are the same
//                    field at different ground resolutions (M9ap), and where two are resident
//                    at the same resolution they hold the same pixels.
//    ...Cube         the same three reads through a page tenant's CUBE views (TextureCubeArray,
//                    slices 0..5 by direction).
//
//  This file names no cbuffer row and no bound resource: a consumer passes its own merc row,
//  texture, sampler and slice, which is what lets one contract serve a pixel shader carrying
//  GA_COMPOSED_CB_ROWS and a compute kernel with its own root signature alike. The ground
//  resolutions are the consumers' (Compose.hlsli's CsGroundM, HeightPages.hlsli's kHp*TexelM):
//  this file states the law, not the numbers.
// ================================================================================================
#ifndef GA_PAGE_SAMPLE_HLSLI
#define GA_PAGE_SAMPLE_HLSLI

static const float kPagePi = 3.14159265358979f;
static const float kPageDim = 16384.0f;      // a page's texels a side (Lattice::kFaceDim)
static const float kPageResDim = 128.0f;     // its residency map: a byte per 128 texels (one tile)

// The window uv of a planet direction: every window realization (colour AND height, the masks,
// the exposure) shares ONE frame -- the merc row: origin px x, origin px y, 1 / page texels,
// world px at the window's zoom (Lattice::Rows) -- so their texels describe the same ground by
// construction.
float2 PageUv(float3 dir, float4 merc) {
    const float lat = asin(clamp(dir.y, -1.0f, 1.0f));
    const float lonDeg = degrees(atan2(dir.z, dir.x));
    const float mx = (lonDeg + 180.0f) / 360.0f * merc.w;
    const float my = (0.5f - log(tan(0.785398163f + lat * 0.5f)) / (2.0f * kPagePi)) * merc.w;
    return (float2(mx, my) - merc.xy) * merc.z;
}
// The kernels' spelling of the same frame, from lat/lon in degrees (the banner says why it is
// its own).
float2 PageUvLatLon(float latDeg, float lonDeg, float4 merc) {
    const float latR = latDeg * 0.01745329252f;
    const float mx = (lonDeg + 180.0f) / 360.0f * merc.w;
    const float my = (0.5f - log(tan(0.7853981634f + latR * 0.5f)) * 0.15915494309f) * merc.w;
    return float2(mx - merc.x, my - merc.y) * merc.z;
}

// A face-plane window's texel relative to its anchor (HIERARCHY 4.4): (U . p + U.w, V . p + V.w)
// / (W . p + W.w), p the point relative to the eye in the frame the rows were pulled into. The
// rows are FaceWindow::PlanesIn's; their w holds the cancellation, so nothing here is large.
float2 PageTexel(float3 p, float4 planeU, float4 planeV, float4 planeW) {
    return float2(dot(p, planeU.xyz) + planeU.w, dot(p, planeV.xyz) + planeV.w) /
           (dot(p, planeW.xyz) + planeW.w);
}
// ...and its uv, for a WRAP sampler to take modulo the page.
float2 PageTexelUv(float3 p, float4 planeU, float4 planeV, float4 planeW) {
    return PageTexel(p, planeU, planeV, planeW) / kPageDim;
}

// The residency floor, pixel stage: CONSERVATIVE (gather + max). Smoothness comes from
// trilinear WITHIN resident data, never from interpolating the map below what is resident.
float PageHave(Texture2DArray map, SamplerState s, float2 uv, uint slice) {
    const float4 g = map.GatherRed(s, float3(uv, slice));
    return max(max(g.x, g.y), max(g.z, g.w)) * 255.0f / 16.0f;
}
float PageHaveCube(TextureCubeArray map, SamplerState s, float3 dir) {
    const float4 g = map.GatherRed(s, float4(dir, 0.0f));
    return max(max(g.x, g.y), max(g.z, g.w)) * 255.0f / 16.0f;
}
// The residency floor, any stage: one Load of the byte under the uv (the map is conservative
// by construction: the coarsest answer of the tiles a byte covers).
float PageHaveLoad(Texture2DArray<float4> map, float2 uv, uint slice) {
    const int2 rc = int2(clamp(uv * kPageResDim, 0.0f, kPageResDim - 1.0f));
    return map.Load(int4(rc, int(slice), 0)).x * 15.9375f;
}

// Bilinear at one mip of one slice, by Loads: the kernels' read.
float4 PageLoad4(Texture2DArray<float4> arr, float2 uv, uint slice, float mip) {
    const float dim = kPageDim / exp2(mip);
    const float2 tf = uv * dim - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    float4 acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = clamp(int2(t0) + int2(k & 1, k >> 1), int2(0, 0),
                              int2(dim - 1.0f, dim - 1.0f));
        acc += ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y) *
               arr.Load(int4(tc, int(slice), int(mip)));
    }
    return acc;
}
float PageLoad(Texture2DArray<float4> arr, float2 uv, uint slice, float mip) {
    return PageLoad4(arr, uv, slice, mip).x;
}

// The pixel-stage sample: the hardware's footprint, the residency floor as the min-LOD clamp.
// PIXEL SHADERS ONLY (Sample needs derivatives); everything else takes PageSampleLevel.
float4 PageSample(Texture2DArray arr, SamplerState s, float2 uv, uint slice, float have) {
    return arr.Sample(s, float3(uv, slice), int2(0, 0), have);
}
float4 PageSampleCube(TextureCubeArray arr, SamplerState s, float3 dir, float have) {
    return arr.Sample(s, float4(dir, 0.0f), have);
}
// The explicit-lod sample at max(want, have): the level the caller wants, floored to what is
// resident. want = 0 asks for the finest resident level.
float4 PageSampleLevel(Texture2DArray arr, SamplerState s, float2 uv, uint slice, float want,
                       float have) {
    return arr.SampleLevel(s, float3(uv, slice), max(want, have));
}
float4 PageSampleLevelCube(TextureCubeArray arr, SamplerState s, float3 dir, float want,
                           float have) {
    return arr.SampleLevel(s, float4(dir, 0.0f), max(want, have));
}

// The ground texel (m) a slice resolves at: its mip-0 ground, doubled per resident mip.
float PageGroundM(float ground0, float have) { return ground0 * exp2(have); }
// The choice: a rung wins where its resident texel is at least as fine as the one held.
bool PageWins(float rungGroundM, float heldGroundM) { return rungGroundM <= heldGroundM; }
// The two-rung form: the page against the cube, from their resident mips and their mip-0
// grounds (x = the cube's, y = the page's).
bool PageWins(float haveCube, float havePage, float2 ground) {
    return PageWins(PageGroundM(ground.y, havePage), PageGroundM(ground.x, haveCube));
}

#endif  // GA_PAGE_SAMPLE_HLSLI
