// ================================================================================================
//  Buildings.hlsl - the building solids as prisms (scene/BuildingLayer.h).
//
//  Each vertex is metres from its CELL's origin; the constants carry that origin relative to the
//  eye, taken in doubles on the CPU and cast once (the vessel layer's boundary). So the position a
//  pixel sees is eye-relative and small wherever the cell stands.
//
//  The colours are neutral and say only what the solid is: wall or roof, building or part. What a
//  roof looks like is the imagery's business; a lens over these solids can colour them by any
//  value a source carries (height tagged or assumed, which source won) without this file knowing.
// ================================================================================================
#define GA_NO_FIELD_BUFFER
#include "Common.hlsli"

cbuffer BuildingCb : register(b1) {
    float4 gBdOrigin;   // xyz = the cell's origin relative to the eye (m), w = brightness
};

struct BuildingVertex {
    float3 pos;    // metres from the cell's origin, flat frame
    float kind;    // 0 wall, 1 roof; +2 for a building:part
    float3 n;      // outward unit normal, flat frame
    float pad;
};
StructuredBuffer<BuildingVertex> gVerts : register(t0, space0);

struct VsOut {
    float4 pos : SV_Position;
    float3 col : COLOR0;
    float3 n : NORMAL0;
    float3 rel : TEXCOORD0;   // eye-relative: the window chain's slab test
    float alpha : TEXCOORD1;  // a fold's coverage (1 for a building)
};

static const float3 kPalette[4] = {
    float3(0.80, 0.78, 0.74),   // wall
    float3(0.52, 0.50, 0.49),   // roof
    float3(0.76, 0.74, 0.72),   // part wall
    float3(0.48, 0.47, 0.47)    // part roof
};

VsOut VsMain(uint vid : SV_VertexID) {
    const BuildingVertex v = gVerts[vid];
    VsOut o;
    o.rel = gBdOrigin.xyz + v.pos;
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    o.n = v.n;
    o.col = kPalette[uint(v.kind) & 3u] * gBdOrigin.w;
    o.alpha = 1.0f;
    return o;
}

// THE FAR BOXES (docs/BUILDING_LOD.md): a building past the detail cells as its moment box, three
// half-axes about a centre (metres from its TILE's origin), 36 vertices a box from SV_VertexID. The
// buffer is bound at the same slot as the prisms' (one draw reads one of them).
struct BuildingBox {
    float3 c; float alpha;  // the share of its spread a fold's footprints cover (1 for a building)
    float3 u; float pad1;   // long half-axis
    float3 v; float pad2;   // short half-axis
    float3 w; float pad3;   // up half-axis
};
StructuredBuffer<BuildingBox> gBoxes : register(t0, space0);

VsOut VsBox(uint vid : SV_VertexID) {
    const BuildingBox b = gBoxes[vid / 36u];
    const uint f = (vid % 36u) / 6u, t = vid % 6u;   // face, then its two triangles
    const float s = (f & 1u) ? -1.0f : 1.0f;         // faces in pairs along u, v, w
    const uint ax = f >> 1;
    const float3 A = ax == 0u ? b.u : (ax == 1u ? b.v : b.w);
    const float3 B = ax == 0u ? b.v : (ax == 1u ? b.w : b.u);
    const float3 C = ax == 0u ? b.w : (ax == 1u ? b.u : b.v);
    static const float2 kQuad[6] = {float2(-1, -1), float2(1, -1), float2(1, 1), float2(-1, -1), float2(1, 1), float2(-1, 1)};
    const float2 q = kQuad[t];
    VsOut o;
    o.rel = gBdOrigin.xyz + b.c + s * A + q.x * B + q.y * C;
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    o.n = s * A / max(length(A), 1e-6f);   // a box with no width keeps a finite normal
    const bool roof = ax == 2u && s > 0.0f;
    o.col = kPalette[roof ? 1u : 0u] * gBdOrigin.w;
    o.alpha = b.alpha;
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    // The solids stand in the eye's own world: cut away wherever a window shows another place.
    if (WindowChainDepth(i.rel, 1u) != 0u) discard;
    // THE LAND'S LIGHT (Globe.hlsl's law, Common.hlsli's terms): the sky's irradiance on the face
    // plus the sun through the air, which the PLANET decides is up at this point -- never the face's
    // own tilt. The point's zenith and height are asked of the sphere (its centre at flat
    // (0, -R, 0)), so a wall at night is dark however it faces.
    const float Rb = gSkyLut.y;
    const float3 fromCentre = gEyeRel.xyz + i.rel + float3(0.0f, Rb, 0.0f);
    const float r = length(fromCentre);
    const float3 up = fromCentre / r;
    const float h = r - Rb;
    const float3 n = normalize(i.n);
    const float ndl = saturate(dot(n, GA_SUN_DIR)) * PlanetShadow(up, GA_SUN_DIR, h, Rb);
    const float3 lit = i.col * (SkyAmbient(n, up, h) + ndl * SunAt(up, h) * 1.15f);
    // ...and seen through THE AIR IN FRONT OF IT, the ground's own law: a box 60 km off fades into
    // the same haze as the land under it, where it stood out at full contrast.
    const float range = length(i.rel);
    return float4(AerialPerspective(lit, i.rel / max(range, 1e-3f), range), i.alpha);
}
