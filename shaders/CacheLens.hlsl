// ================================================================================================
//  CacheLens.hlsl - the cache lens's pixel shader (the minimap's Cache toggle, docs/FIDELITY.md 7),
//  compiled only for a scene with a minimap and drawn only by an eye whose toggle is on
//  (GlobeLayer::SetCacheLens, m_msPsoCache / m_psoCache). A file of its own for ResidencyLens.hlsl's
//  measured reason: an extra entry point inside Globe.hlsl moved the shipped picture.
//
//  Not the picture: WHAT THE HARVEST HOLDS OFFLINE under this pixel, read from the storage index
//  (compose/CacheIndex.h) reduced to the cube lattice at level 10 and stacked face by face in one
//  2-D texture. The texel is found by the lattice's own law -- the pixel's planet direction, its
//  D3D cube face and uv (HpCubeFace, core/Lattice.h CubeFaceOfDir), row face * 1024 + y -- and Loaded
//  whole, so a cell is one colour and no filter mixes two. The rows (the eye's own, GlobeLayer):
//      gLensU.z  the map's SRV (0xFFFFFFFE: asked for, no index on disk -> grey)
//      gLensU.w  the eye's rectangle's top-left on the target, x | y << 16 (for the key)
//  The texel: R = the finest lattice level stored over it, x 20 (0..12); G = log2(1 + bytes per
//  texel) / 40. The hue is the level -- dark blue nothing, blue coarse, cyan, yellow, red, white at
//  level 12 (about 2.4 km cells: whatever is finer than that, centimetre imagery included, is
//  counted in its level-12 cell) -- and the bytes brighten it a little. PsMeshlet's lambert keeps
//  the terminator readable. The key at the bottom left of the rectangle, above the altitude text,
//  is drawn by this same function: nothing, then levels 0..12, left to right.
// ================================================================================================
#include "Globe.hlsl"

#define CACHE_LENS_KEY_Y 30.0f   // the key's bottom edge above the rectangle's (the altitude text is below it)

float3 CacheLensHue(float level, float bytes01, bool have) {
    if (!have) return float3(0.015f, 0.02f, 0.08f);
    // One ramp over the level, stops every three levels: 0 blue, 3 cyan-blue, 6 cyan, 9 yellow,
    // 11 red, 12 white.
    static const float3 kStop[6] = {float3(0.05f, 0.08f, 0.45f), float3(0.05f, 0.40f, 0.85f),
                                    float3(0.10f, 0.85f, 0.90f), float3(1.00f, 0.90f, 0.15f),
                                    float3(1.00f, 0.20f, 0.05f), float3(1.00f, 1.00f, 1.00f)};
    static const float kAt[6] = {0.0f, 3.0f, 6.0f, 9.0f, 11.0f, 12.0f};
    float3 c = kStop[0];
    [unroll] for (int k = 1; k < 6; ++k) {
        c = lerp(c, kStop[k], saturate((level - kAt[k - 1]) / (kAt[k] - kAt[k - 1])));
    }
    return c * (0.7f + 0.3f * bytes01);
}

float4 PsCacheLens(VsOut i) : SV_Target {
    LoadLevel(i.lvl);
    if (gGateA.x >= 0.0f) {
        const uint depth = LevelGateDepth(i.lvl);
        if (GateDepth(TrueRel(i.rel), depth + 1u) != depth) discard;
    }
    const float3 up = normalize(i.dir);
    // The key: 16 px cells, 14 px swatches, one row, nothing then levels 0..12.
    const float2 org = float2(float(gLensU.w & 0xFFFFu), float(gLensU.w >> 16));
    const float2 kp = float2(i.pos.x - org.x - 8.0f, (org.y + gViewport.y - CACHE_LENS_KEY_Y) - i.pos.y);
    if (all(kp >= 0.0f) && kp.x < 14.0f * 16.0f && kp.y < 16.0f) {
        if (frac(kp.x / 16.0f) >= 0.875f || kp.y >= 14.0f) return float4(0, 0, 0, 1);
        const int col = int(kp.x / 16.0f);
        return float4(CacheLensHue(float(col - 1), 0.5f, col > 0), 1);
    }
    float3 c = float3(0.3f, 0.3f, 0.3f);   // no index on disk: grey
    if (gLensU.z < 0xFFFFFFF0u) {
        float2 uv;
        const uint face = HpCubeFace(up, uv);
        const uint n = 1024u;   // CacheIndex::kSide
        const uint2 t = min(uint2(uv * float(n)), uint2(n - 1u, n - 1u));
        const float2 rg = gTex[gLensU.z].Load(int3(t.x, face * n + t.y, 0)).rg;
        c = CacheLensHue(rg.r * 255.0f / 20.0f, rg.g, rg.g > 0.0f);
    }
    const float lam = saturate(dot(CsToTangent(up), gSunDir.xyz) * 0.5f + 0.5f);
    return float4(c * (0.25f + 0.75f * lam), 1.0f);
}
