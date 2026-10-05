// ================================================================================================
//  M4: the churn field -- the engine's FIRST genuinely stateful quantity, living where state
//  belongs: in resident tiles of a sparse atlas over a 16 x 16 km virtual domain.
//  M9az: the domain is a toroidal clipmap WINDOW around the camera on a world-anchored tile
//  lattice (slot = world tile mod atlas tiles); it no longer sits on the station.
//
//  Where the ebb blocks the chop band, breaking sheds aerated water whose whiteness LINGERS --
//  that is memory, so it cannot be stateless. But it only exists along the entrance bar, so
//  99% of the domain stays NULL: reads-as-zero everywhere else, dispatches walk the resident
//  tile list only, and writes never touch unmapped tiles by construction (the residency policy
//  maps ahead of the current, and freshly mapped tiles are CLEARED before first use -- their pool
//  memory is undefined).
//
//  Kernels are list-driven: Dispatch(tileW/16, tileH/16, listCount); id.xy spans one tile,
//  id.z indexes gTileList.
// ================================================================================================
#include "Jet.hlsli"

#include "WindowRows.hlsli"

cbuffer ChurnCb : register(b0) {
    float gOriginX, gOriginZ, gTexelM, gDomainM;
    uint  gTilesX, gTileW, gTileH, gListCount;
    float gDt, gTau, gPad0, gPad1;
    float4 gMiscC;   // x = chop-band WAVENUMBER (rad/m; M5c -- was deep phase speed),
                     // y = patchL2, z = advWrapT, w unused
    float4 gWaveD;   // xy = peak propagation dir, z = deriv texture valid, w = source gain
    float4 gSweM;    // M5c: x = solved-field on, y = current gain, zw unused (C3b: no jet to blend)
    float4 gWindow;  // M9az: x, y = world tile index of the window's origin; z = atlas tiles in y in the tenant's array
    // PHASE B2: the camera's world's windows -- its eye in the tangent axes less (0, R, 0), w = R,
    // then its rows (WindowRows.hlsli). Appended at the END on both sides.
    float4 gEyeT;
    HP_WINDOW_ROWS_DECL
    HP_SOLVER_ROWS_DECL     // the solver's chart about the churn's frame
    // PHASE C5: the world.flat chart's rows in the planet frame (gTanU.w = R), appended LAST.
    float4 gTanE;
    float4 gTanU;
    float4 gTanN;
};

#define HP_WINDOW_ROWS 1
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

// The bed at a world point: its direction by the chart's rows, then the page-or-cube rule.
// PHASE B2: the windows' chain at the texel's own point -- the flat world point (the sea's world IS
// the tangent frame about the anchor's ground: Sea.hlsl's SeaPoint) about the camera's eye -- read at
// the churn's own texel; the direction (for the cube) by the chart's rows.
float3 ChurnPoint(float2 world) {
    const float drop = dot(world, world) / (2.0f * gEyeT.w);
    return float3(world.x - gEyeT.x, -drop - gEyeT.y, world.y - gEyeT.z);
}
float PageBedAt(float2 world) {
    // The direction of the flat point, by the chart's rows (the exact map, Space::Anchor::PlaceOf).
    const float3 dir = normalize(gTanU.xyz + (gTanE.xyz * world.x + gTanN.xyz * world.y) / gTanU.w);
    return HpHeightChain(gBathy, gBathyRes, dir, WindowChain(ChurnPoint(world), 0u), gTexelM);
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
    // M9az: the slot's WORLD tile is the one inside the window that aliases to it -- unwrap
    // from the window's origin tile. World then follows from the lattice, not from an origin.
    const uint t = gTileList[id.z];
    const int2 slot = int2(t % gTilesX, t / gTilesX);
    const int2 N = int2(int(gTilesX), int(gWindow.z));
    const int2 T0 = int2(gWindow.xy);
    int2 d = slot - T0;
    d = ((d % N) + N) % N;
    const int2 T = T0 + d;
    const float2 world =
        (float2(T) * float2(gTileW, gTileH) + float2(id.xy) + 0.5f) * gTexelM;

    float src = 0.0f;
    {
        // THE ONE WATER (PHASE C3b): the current the waves feel is the solver's own, the channel's
        // ebb and flood plume included; outside its domain's cells there is none.
        float2 U = 0.0f;
        float depth = 30.0f;
        const float3 cp = ChurnPoint(world);
        if (gSweM.x > 0.5f && gSvO.z > 0.0f && SolverDen(cp) > 0.0f) {   // the cell's ground in the solver's chart
            const float2 suv = SolverCell(cp) / gSvO.xy;   // row 0 north, as the bank's texture
            if (all(suv > 0.001f) && all(suv < 0.999f)) {
                depth = -PageBedAt(world);   // refined below by the level
                const float4 s = gSweUv.SampleLevel(sClamp, suv, 0);
                if (s.w > 0.5f) U = s.xy * gSweM.y;
            }
        }
        // Chop phase speed at the LOCAL depth (gMiscC.x is the band wavenumber since M5c). The bed
        // is NAVD; the water level is taken as ~0 NAVD for the churn's coarse purposes.
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
