// PHASE B1 (out/integration/plan_phase_b.md): THE KERNELS' BED BY THE CHAIN on the GPU, for the
// selftest's readback (the harness is ProbeBedChain in src/hal/TileAtlas.cpp). Each pixel runs
// HeightPages.hlsli's HpHeightChain -- the one body the bank, the churn and the solver will call --
// for one point, its chain found by Window.hlsli from the kernels' own rows (HP_WINDOW_ROWS_DECL in
// a cbuffer, filled by SurfaceFrame::KernelRows), against a SYNTHETIC tenant: a texel's value and a
// tile's residency byte are hashes of their address, which the C++ twin computes the same way, so
// the address, the chain, the choice of rank and mip and the modulo taps are all that is tested.
// Point j writes (the bed, 16 slice + mip, the chain's length, 1) at x = j % 64, row j / 64.
// Compiled with HP_TRACE (the rule's own record of its choice: HP_CHOSE).
#include "WindowRows.hlsli"

cbuffer BedRows : register(b0) {
    uint gPts;     // the points (RGBA32F): xyz relative to the eye in the rows' frame, w = texelM
    uint gDirs;    // their planet directions (RGBA32F)
    uint gShift;   // how far below its points' rows this draw's target rows sit
    uint gPad;
    HP_WINDOW_ROWS_DECL
};

Texture2D<float4> gTex[] : register(t0, space1);         // the heap
Texture2DArray<float4> gTA[] : register(t0, space5);    // (the rule's parameters; never read here)

// THE SYNTHETIC TENANT (the C++ twin in TileAtlas.cpp says the same, op for op).
uint BedHash(uint a, uint b, uint c, uint d) {
    uint h = a * 73856093u ^ b * 19349663u ^ c * 83492791u ^ d * 2654435761u;
    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return h;
}
float BedTexel(uint slice, uint mip, int x, int y) {
    return float(BedHash(slice, mip, uint(x), uint(y)) & 0xFFFFu) / 65536.0f;
}
float BedBilinear(float2 uv, uint slice, float mip, bool wrap) {
    const float dim = 16384.0f / exp2(mip);
    const int idim = int(dim);
    const float2 tf = (wrap ? frac(uv) : uv) * dim - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    float acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        int2 tc = int2(t0) + int2(k & 1, k >> 1);
        tc = wrap ? (tc + idim) % idim : clamp(tc, int2(0, 0), int2(idim - 1, idim - 1));
        acc += ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y) *
               BedTexel(slice, uint(mip), tc.x, tc.y);
    }
    return acc;
}
float BedHave(float2 uv, uint slice, bool wrap) {
    const int2 rc = int2(clamp((wrap ? frac(uv) : uv) * 128.0f, 0.0f, 127.0f));
    return (slice < 6u) ? float((slice + uint(rc.x) + uint(rc.y)) % 4u)
                        : float((slice * 7u + uint(rc.x) * 3u + uint(rc.y) * 5u) % 6u);
}
#define HP_FETCH_WIN(arr, uv, slice, mip) BedBilinear(uv, slice, mip, true)
#define HP_HAVE_WIN(res, uv, slice) BedHave(uv, slice, true)
#define HP_FETCH_CUBE(arr, uv, face, mip) BedBilinear(uv, face, mip, false)
#define HP_HAVE_CUBE(res, uv, face) BedHave(uv, face, false)
#define HP_WINDOW_ROWS 1
#include "HeightPages.hlsli"

float4 VsBed(uint id : SV_VertexID) : SV_Position {
    const float2 t = float2((id << 1) & 2, id & 2);
    return float4(t * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), 0.0f, 1.0f);
}

float4 PsBed(float4 pos : SV_Position) : SV_Target {
    const int2 at = int2(pos.xy) - int2(0, int(gShift));
    const float4 pt = gTex[gPts].Load(int3(at, 0));
    const float3 dir = gTex[gDirs].Load(int3(at, 0)).xyz;
    const WalkChain wc = WindowChain(pt.xyz, 0u);
    const float bed = HpHeightChain(gTA[0], gTA[0], dir, wc, pt.w);
    return float4(bed, float(gHpTraceSlice) * 16.0f + gHpTraceMip, float(wc.n), 1.0f);
}
