// ================================================================================================
//  ImpactRead.hlsli -- THE IMPACTS' WAVE AT A POINT OF THE PLANET, from the events' buffer
//  (2026-10-25; sim/ImpactField.h packs it, laws/Impact.hlsli is the law). Included by Globe.hlsl
//  (the vertices, the far sea's pixel) and WaterBank.hlsl (the texel's level) after gBufU and the law.
//
//  The buffer (uints, read as floats): events x 8 words of header -- the impact's direction (the
//  planet's frame), its zero less the field's base (s), the basin's depth, the display exaggeration,
//  the first word of its terms, their count -- then each event's terms (k, omega, A, 0). The sum at
//  a point: for every event begun, the great-circle distance, the table's Bessel sum, the DISPLAY
//  EXAGGERATION ON THE DEEP WAVE, then the shoaling to the depth here (so the bound is the depth's,
//  whatever the exaggeration: a wave over a shelf stands no taller than the shelf's water, and the
//  exaggeration shows the deep sea's wave alone). The slope is accumulated as a vector in the
//  planet's frame (each event's is along its own radial): the caller takes it into its tangent frame.
// ================================================================================================
#ifndef GA_IMPACT_READ_HLSLI
#define GA_IMPACT_READ_HLSLI

float ImpactSumAt(uint slot, uint events, float tNow, float R, float3 dir, float depth, out float3 slopePlanet) {
    slopePlanet = 0.0f;
    float eta = 0.0f;
    if (events == 0u || slot == 0xFFFFFFFFu) return 0.0f;
    for (uint e = 0u; e < events; ++e) {
        const uint h = e * 8u;
        const uint n = uint(asfloat(gBufU[slot][h + 7u]));
        if (n == 0u) continue;
        const float t = tNow - asfloat(gBufU[slot][h + 3u]);
        if (t < 0.0f) continue;
        const float3 pI = float3(asfloat(gBufU[slot][h]), asfloat(gBufU[slot][h + 1u]), asfloat(gBufU[slot][h + 2u]));
        const float h0 = asfloat(gBufU[slot][h + 4u]);
        const float exag = asfloat(gBufU[slot][h + 5u]);
        const uint off = uint(asfloat(gBufU[slot][h + 6u]));
        const float c = clamp(dot(dir, pI), -1.0f, 1.0f);
        const float r = R * acos(c);
        float ed = 0.0f, sd = 0.0f;
        for (uint i = 0u; i < n; ++i) {
            const uint w = off + 4u * i;
            const float k = asfloat(gBufU[slot][w]);
            const float om = asfloat(gBufU[slot][w + 1u]);
            const float A = asfloat(gBufU[slot][w + 2u]);
            ed += ImpactTerm(A, k, om, r, t);
            sd += ImpactTermSlope(A, k, om, r, t);
        }
        ed *= exag;
        sd *= exag;
        eta += ImpactShoal(ed, h0, depth);
        const float slS = ImpactShoalSlope(ed, sd, h0, depth);
        // the slope is along r: away from the impact, in the tangent plane at dir
        float3 away = dir * c - pI;
        const float la = length(away);
        if (la > 1e-6f) slopePlanet += away * (slS / la);
    }
    return eta;
}
#endif
