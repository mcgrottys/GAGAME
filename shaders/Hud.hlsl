// THE WINDSHIELD (scene/HudLayer.h): rectangles on the glass, drawn after the tonemap into the
// display-referred target. One instance per element, six vertices each; the kind says how the
// rectangle fills. Positions are target pixels, top-left origin.
#define GA_NO_FIELD_BUFFER
#include "Common.hlsli"

cbuffer HudCb : register(b1) {
    float4 gHudTarget;   // w, h, 1/w, 1/h of the target
};

struct HudEl {           // mirrored in HudLayer.h (Element), 48 bytes
    float4 rect;         // x0, y0, x1, y1 px
    float4 color;        // display rgb, alpha
    uint kind;           // 0 glyph, 1 fill, 2 frame, 3 ring
    uint glyphLo;        // the 5x7 glyph's columns 0-3, a byte each (bit 0 = top row)
    uint glyphHi;        // column 4
    float edge;          // frame / ring thickness px (ring: 0 = a filled disc)
};
StructuredBuffer<HudEl> gHud : register(t0, space0);

struct VsOut {
    float4 pos : SV_Position;
    float2 uv : TEXCOORD0;                     // 0..1 across the rectangle
    nointerpolation uint id : TEXCOORD1;
};

VsOut VsMain(uint vid : SV_VertexID, uint iid : SV_InstanceID) {
    static const float2 kQuad[6] = {float2(0, 0), float2(1, 0), float2(0, 1),
                                    float2(0, 1), float2(1, 0), float2(1, 1)};
    const HudEl e = gHud[iid];
    const float2 uv = kQuad[vid];
    const float2 px = lerp(e.rect.xy, e.rect.zw, uv);
    VsOut o;
    o.pos = float4(px.x * gHudTarget.z * 2.0f - 1.0f, 1.0f - px.y * gHudTarget.w * 2.0f, 0.0f, 1.0f);
    o.uv = uv;
    o.id = iid;
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    const HudEl e = gHud[i.id];
    const float2 size = e.rect.zw - e.rect.xy;
    float cover = 1.0f;
    if (e.kind == 0) {
        const uint col = min(uint(i.uv.x * 5.0f), 4u);
        const uint row = min(uint(i.uv.y * 7.0f), 6u);
        const uint bits = col < 4 ? (e.glyphLo >> (col * 8u)) & 0xFFu : e.glyphHi & 0xFFu;
        cover = ((bits >> row) & 1u) ? 1.0f : 0.0f;
    } else if (e.kind == 2) {
        const float2 p = i.uv * size;   // px from the top-left corner
        const float d = min(min(p.x, p.y), min(size.x - p.x, size.y - p.y));
        cover = d < e.edge ? 1.0f : 0.0f;
    } else if (e.kind == 3) {
        // The inscribed disc, antialiased over one pixel; a ring keeps the outer `edge` px.
        const float r = 0.5f * min(size.x, size.y);
        const float d = length((i.uv - 0.5f) * size);
        cover = saturate(r - d + 0.5f);
        if (e.edge > 0.0f) cover *= saturate(d - (r - e.edge) + 0.5f);
    }
    if (cover <= 0.0f) discard;
    return float4(e.color.rgb, e.color.a * cover);
}
