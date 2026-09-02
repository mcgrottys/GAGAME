// ================================================================================================
//  M4: the churn field -- the engine's FIRST genuinely stateful quantity, living where state
//  belongs: in resident tiles of a sparse atlas over a 16 x 16 km virtual domain.
//
//  Where the ebb blocks the chop band, breaking sheds aerated water whose whiteness LINGERS --
//  that is memory, so it cannot be stateless. But it only exists along the entrance bar, so
//  99% of the domain stays NULL: reads-as-zero everywhere else, dispatches walk the resident
//  tile list only, and writes never touch unmapped tiles by construction (the residency policy
//  maps ahead of the jet, and freshly mapped tiles are CLEARED before first use -- their pool
//  memory is undefined).
//
//  Kernels are list-driven: Dispatch(tileW/16, tileH/16, listCount); id.xy spans one tile,
//  id.z indexes gTileList.
// ================================================================================================
#include "Jet.hlsli"

cbuffer ChurnCb : register(b0) {
    float gOriginX, gOriginZ, gTexelM, gDomainM;
    uint  gTilesX, gTileW, gTileH, gListCount;
    float gDt, gTau, gPad0, gPad1;
    float4 gJetA;    // signedMs, halfWidth, seawardDecay, enabled
    float4 gJetB;    // floodDir xy, ebbDir xy
    float4 gMiscC;   // x = chop-band WAVENUMBER (rad/m; M5c -- was deep phase speed),
                     // y = patchL2, z = advWrapT, w unused
    float4 gWaveD;   // xy = peak propagation dir, z = deriv texture valid, w = source gain
    float4 gBathyG;  // M5c: world x0, z0, 1/sizeX, 1/sizeZ of the CUDEM (row 0 = north)
    float4 gSweM;    // M5c: x = solved-field on, y = current gain, zw = seaward blend x-range
    float4 gGeoA;    // M9ar: world -> lat/lon: orgLat, orgLon, 1/mPerLat, 1/mPerLon
    float4 gWinA;    // M9ar: height page frame: org px x, org px y, 1/16384, world px at z14
    float4 gPageB;   // M9ax: x = the z14 page's slice in the tenant's array
};

#include "HeightPages.hlsli"

StructuredBuffer<uint> gTileList : register(t0);
Texture2D<float4> gChopDeriv : register(t1);   // cascade-2 derivatives (foam pattern source)
Texture2D<float4> gSweUv : register(t2);       // M5c: solved currents (u, v, |U|, valid)
// M9ar: THE BED IS THE HEIGHT MEGATEXTURE. M9ax: the tenant's whole array (cube faces and the
// z14 page) with its residency-map array, resolved by HeightPages.hlsli -- no -30 m past the
// page any more. The CUDEM-private texture this read is gone.
Texture2DArray<float4> gBathy    : register(t3);   // NAVD88 m; R16F loads as .x
Texture2DArray<float4> gBathyRes : register(t4);   // R8: finest resident mip * 16

RWTexture2D<float> gChurnTex : register(u0);

SamplerState sWrap : register(s0);
SamplerState sClamp : register(s1);

// The bed at a world point: the flat-one-world map to lat/lon, then the page-or-cube rule.
float PageBedAt(float2 world) {
    const float lat = gGeoA.x + world.y * gGeoA.z;
    const float lon = gGeoA.y + world.x * gGeoA.w;
    return HpHeightAt(gBathy, gBathyRes, lat, lon, gWinA, uint(gPageB.x), 0.0f);
}

uint2 TileTexel(uint3 id) {
    const uint t = gTileList[id.z];
    return uint2(t % gTilesX, t / gTilesX) * uint2(gTileW, gTileH) + id.xy;
}

[numthreads(16, 16, 1)]
void CsChurnClear(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gTileW || id.y >= gTileH) return;
    gChurnTex[TileTexel(id)] = 0.0f;
}

// Self-test kernel: stamps tileIndex+1 so the CPU can verify writes land exactly where
// residency says they should (and nowhere else).
[numthreads(16, 16, 1)]
void CsChurnTestPattern(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gTileW || id.y >= gTileH) return;
    gChurnTex[TileTexel(id)] = (float)(gTileList[id.z] + 1u);
}

[numthreads(16, 16, 1)]
void CsChurnUpdate(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gTileW || id.y >= gTileH) return;
    const uint2 texel = TileTexel(id);
    const float2 world = float2(gOriginX, gOriginZ) + (float2(texel) + 0.5f) * gTexelM;

    float src = 0.0f;
    if (gJetA.w > 0.5f) {
        // M5c: the same current the WAVES feel -- the solved field inside the estuary (real
        // channel shape), the analytic jet seaward of the tips, blended on the same x-ramp as
        // Sea.hlsl's JetU. The M4 analytic-only version deposited over its whole geography-blind
        // Gaussian band: a tile-shaped white blanket across the flats at every strong ebb.
        float2 U = JetVelocity(world, gJetA.x, gJetA.y, gJetA.z, gJetB.xy, gJetB.zw);
        float depth = 30.0f;
        if (gSweM.x > 0.5f) {
            const float2 buv = (world - gBathyG.xy) * gBathyG.zw;
            if (all(buv > 0.001f) && all(buv < 0.999f)) {
                const float2 suv = float2(buv.x, 1.0f - buv.y);
                depth = -PageBedAt(world);   // refined below by the level
                const float4 s = gSweUv.SampleLevel(sClamp, suv, 0);
                if (s.w > 0.5f) {
                    U = lerp(s.xy * gSweM.y, U, smoothstep(gSweM.z, gSweM.w, world.x));
                }
            }
        }
        // Chop phase speed at the LOCAL depth (gMiscC.x is the band wavenumber since M5c).
        // gJetA.x carries the analytic water level offset... the bed is NAVD; treat the water
        // level as ~0 NAVD for the churn's coarse purposes (sub-metre level error moves the
        // blocking threshold negligibly against a 2 m/s jet).
        const float cChop = BandPhaseSpeed(gMiscC.x, max(depth, 0.4f));
        const float blocked = (depth > 0.4f) ? WaveCurrentAmp(U, gWaveD.xy, cChop).y : 0.0f;
        if (blocked > 0.0f) {
            // Pattern the deposition by the chop's own Jacobian foam so the memory has the
            // streaky texture of the breaking that produced it. The floor is low on purpose:
            // deposit follows the streaks, or the wide solved-field blocking area fills as an
            // even white sheet.
            float pattern = 0.6f;
            if (gWaveD.z > 0.5f) {
                const float2 uv = (world - U * gMiscC.z) / gMiscC.y;
                pattern = 0.08f + 0.92f * saturate(gChopDeriv.SampleLevel(sWrap, uv, 0).w * 4.0f);
            }
            src = blocked * pattern * gWaveD.w;
        }
    }

    const float old = gChurnTex[texel];
    // Decay toward zero, refresh from breaking: max keeps freshly churned streaks crisp while
    // the old wash fades on the tau clock.
    gChurnTex[texel] = max(old * exp(-gDt / max(gTau, 1.0f)), src);
}
