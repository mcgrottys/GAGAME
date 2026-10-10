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
    float4 gBdTime;     // x = the layer's now (s), y = a page's fade-in (s), z = a pixel's angle (rad)
    uint4 gBdMesh;      // the shapes' draw: x first instance, y first task (float4 rows of the walk
                        // buffer), z tasks, w the shape pool's heap slot (compose/BuildingShape.h)
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
    float alpha : TEXCOORD1;  // a far box fading in (1 for a prism)
};

static const float3 kPalette[6] = {
    float3(0.80, 0.78, 0.74),   // wall
    float3(0.52, 0.50, 0.49),   // roof
    float3(0.76, 0.74, 0.72),   // part wall
    float3(0.48, 0.47, 0.47),   // part roof
    float3(0.30, 0.30, 0.31),   // a road's surface (asphalt)
    float3(0.56, 0.53, 0.47)    // a path's
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

// THE FAR BOXES (docs/BUILDING_LOD.md): a building past the detail cells as its own box, three
// half-axes about a centre (metres from its TILE's origin), 36 vertices a box from SV_VertexID. The
// buffer is bound at the same slot as the prisms' (one draw reads one of them).
struct BuildingBox {
    float3 c; float alpha;  // its fade-in: (pixels - lodPixels) / lodPixels, up to 1
    float3 u; float landed; // long half-axis; its page's landing on the layer's clock (s)
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
    // A page arrives whole; its buildings come in over gBdTime.y from its landing, not in one frame.
    o.alpha = b.alpha * saturate((gBdTime.x - b.landed) / max(gBdTime.y, 1e-3f));
    return o;
}

// THE BUILDINGS' OWN FORMS (compose/BuildingShape.h, GALOD04): one group a TASK of the walk's --
// up to 32 edges of a building's walls, or up to 64 of its roof's triangles -- extruded here from the
// footprint the shape pool holds, so a building is the solid it is at every distance, never a box.
// The walk buffer (t0): instances of 4 rows (c ground under the centroid, alpha | east, landed |
// north, shape address | up, -) about the walk's origin, then the tasks two a row (instance, code:
// bit 31 roof, the rest the chunk).
// Both read as INTEGERS: a word of two small 16-bit numbers is a denormal as a float, and a float
// load may flush it to zero -- a roof triangle on vertex 0 collapsed, an instance index lost.
StructuredBuffer<uint4> gWalk : register(t0, space0);
StructuredBuffer<uint4> gShapePool[] : register(t0, space7);   // the heap's buffers (gWorlds' range)
uint ShapeU32(uint b) {
    const uint4 q = gShapePool[gBdMesh.w][b >> 4];
    const uint k = (b >> 2) & 3u;
    return k == 0u ? q.x : (k == 1u ? q.y : (k == 2u ? q.z : q.w));
}
uint ShapeU16(uint b) {
    const uint w = ShapeU32(b & ~3u);
    return (b & 2u) ? (w >> 16) : (w & 0xFFFFu);
}
float ShapeI16(uint b) { return float(int(ShapeU16(b) << 16) >> 16); }

static const uint kTaskEdges = 32u, kTaskTris = 64u;   // MIRRORS BuildingLayer::kTaskEdges, kTaskTris
[outputtopology("triangle")]
[numthreads(64, 1, 1)]
void MsShape(uint gi : SV_GroupIndex, uint3 gid : SV_GroupID, out vertices VsOut verts[192],
             out indices uint3 tris[64]) {
    const uint t = gid.x + gid.y * 65535u;
    const bool live = t < gBdMesh.z;
    const uint4 tk = gWalk[gBdMesh.y + (min(t, max(gBdMesh.z, 1u) - 1u) >> 1)];
    const uint inst = (t & 1u) ? tk.z : tk.x, code = (t & 1u) ? tk.w : tk.y;
    const uint ib = gBdMesh.x + inst * 4u;
    const float4 r0 = asfloat(gWalk[ib]), r1 = asfloat(gWalk[ib + 1u]), r2 = asfloat(gWalk[ib + 2u]),
                 r3 = asfloat(gWalk[ib + 3u]);
    const float3 c = r0.xyz, E = r1.xyz, N = r2.xyz, U = r3.xyz;
    const uint addr = gWalk[ib + 2u].w;
    const uint w0 = ShapeU32(addr), w1 = ShapeU32(addr + 4u);
    const uint nVerts = w0 & 0xFFFFu, nTris = w0 >> 16, nRings = w1 & 0xFFu, flags = (w1 >> 8) & 0xFFu;
    const float bottom = asfloat(ShapeU32(addr + 8u)), top = asfloat(ShapeU32(addr + 12u));
    const uint ringAt = addr + 16u, xyAt = ringAt + 2u * nRings, triAt = xyAt + 4u * nVerts;
    const float unit = (flags & 2u) ? 1.0f : 0.1f;
    const float part = (flags & 1u) ? 2.0f : 0.0f;
    const bool roof = (code >> 31) != 0u;
    const bool ribbon = (flags & 4u) != 0u;   // shape::kRibbon: a road piece, one quad a segment
    const uint first = (code & 0x7FFFFFFFu) * (roof ? kTaskTris : kTaskEdges);
    const uint total = ribbon ? nVerts - 1u : (roof ? nTris : nVerts);
    const uint n = live && first < total ? min(roof ? kTaskTris : kTaskEdges, total - first) : 0u;
    // One count for the group, set before any output (the validator's rule): 4 vertices and 2
    // triangles an edge (a wall, or a ribbon's segment), or 3 and 1 a roof triangle.
    SetMeshOutputCounts(roof ? n * 3u : n * 4u, roof ? n : n * 2u);
    if (gi >= n) return;
    VsOut o;
    o.alpha = r0.w * saturate((gBdTime.x - r1.w) / max(gBdTime.y, 1e-3f));   // the page's fade-in, as VsBox
    if (ribbon) {   // THE RIBBON: segment e of the polyline as a quad, mitred at both vertices
        const uint zAt = triAt + 6u * nTris;                    // the heights the page's reader filled
        const float w = float(ShapeU16(addr + 6u)) * 0.1f;      // ShapeHead.pad: the width, decimetres
        const uint e = first + gi;
        float3 P[4];   // prev, a, b, next (an end repeats itself)
        [unroll] for (uint k = 0u; k < 4u; ++k) {
            const uint v = uint(clamp(int(e) - 1 + int(k), 0, int(nVerts) - 1));
            P[k] = float3(ShapeI16(xyAt + 4u * v) * unit, ShapeI16(xyAt + 4u * v + 2u) * unit, ShapeI16(zAt + 2u * v) * 0.1f);
        }
        const float2 s = P[2].xy - P[1].xy;
        const float sl = length(s);
        const float2 sn = sl > 1e-4f ? float2(s.y, -s.x) / sl : float2(0.0f, 0.0f);   // the segment's right
        // The miter at each end: the right of the bisector tangent, lengthened by 1 / cos of the
        // half turn (held to 2: a hairpin does not spike), so neighbouring quads share their edge.
        float2 m[2];
        [unroll] for (uint q = 0u; q < 2u; ++q) {
            const float2 t = P[q + 2u].xy - P[q].xy;
            const float tl = length(t);
            const float2 tn = tl > 1e-4f ? float2(t.y, -t.x) / tl : sn;
            m[q] = tn / max(dot(tn, sn), 0.5f);
        }
        // The width on screen is at least a pixel, and the alpha its true share of that pixel: a
        // road 20 km off is a faint line, never a sparkle. A pixel's metres at this distance is the
        // distance times the pixel's angle (gBdTime.z, the walk's own).
        const float3 mid = gBdOrigin.xyz + c + P[1].x * E + P[1].y * N;
        const float px = length(mid) * gBdTime.z;
        const float hw = 0.5f * max(w, px);
        o.alpha *= saturate(w / max(px, 1e-3f));
        o.n = U;
        o.col = kPalette[(flags & 8u) ? 5u : 4u] * gBdOrigin.w;
        const float2 xy[4] = {P[1].xy + m[0] * hw, P[1].xy - m[0] * hw, P[2].xy + m[1] * hw, P[2].xy - m[1] * hw};
        const float zz[4] = {P[1].z + top, P[1].z + top, P[2].z + top, P[2].z + top};
        [unroll] for (uint k = 0u; k < 4u; ++k) {
            o.rel = gBdOrigin.xyz + c + xy[k].x * E + xy[k].y * N + zz[k] * U;
            o.pos = mul(float4(o.rel, 1.0f), gViewProj);
            verts[gi * 4u + k] = o;
        }
        tris[gi * 2u] = uint3(gi * 4u, gi * 4u + 2u, gi * 4u + 1u);
        tris[gi * 2u + 1u] = uint3(gi * 4u + 1u, gi * 4u + 2u, gi * 4u + 3u);
    } else if (!roof) {   // the walls: edge e from its vertex to its ring's next
        const uint e = first + gi;
        uint start = 0u, len = 0u;
        for (uint r = 0u; r < nRings; ++r) {
            len = ShapeU16(ringAt + 2u * r);
            if (e < start + len) break;
            start += len;
        }
        const uint f = (e + 1u < start + len) ? e + 1u : start;
        const float2 a = float2(ShapeI16(xyAt + 4u * e), ShapeI16(xyAt + 4u * e + 2u)) * unit;
        const float2 b = float2(ShapeI16(xyAt + 4u * f), ShapeI16(xyAt + 4u * f + 2u)) * unit;
        const float2 d = b - a;
        const float l = length(d);
        const float2 out2 = l > 1e-4f ? float2(d.y, -d.x) / l : float2(0.0f, 0.0f);   // the edge's right: out
        o.n = out2.x * E + out2.y * N;
        o.col = kPalette[uint(part)] * gBdOrigin.w;
        const float2 xy[4] = {a, b, b, a};
        const float zz[4] = {bottom, bottom, top, top};
        [unroll] for (uint k = 0u; k < 4u; ++k) {
            o.rel = gBdOrigin.xyz + c + xy[k].x * E + xy[k].y * N + zz[k] * U;
            o.pos = mul(float4(o.rel, 1.0f), gViewProj);
            verts[gi * 4u + k] = o;
        }
        tris[gi * 2u] = uint3(gi * 4u, gi * 4u + 1u, gi * 4u + 2u);
        tris[gi * 2u + 1u] = uint3(gi * 4u, gi * 4u + 2u, gi * 4u + 3u);
    } else {       // the roof: its triangle at the top
        o.n = U;
        o.col = kPalette[uint(part) + 1u] * gBdOrigin.w;
        [unroll] for (uint k = 0u; k < 3u; ++k) {
            const uint v = ShapeU16(triAt + 6u * (first + gi) + 2u * k);
            const float2 q = float2(ShapeI16(xyAt + 4u * v), ShapeI16(xyAt + 4u * v + 2u)) * unit;
            o.rel = gBdOrigin.xyz + c + q.x * E + q.y * N + top * U;
            o.pos = mul(float4(o.rel, 1.0f), gViewProj);
            verts[gi * 3u + k] = o;
        }
        tris[gi] = uint3(gi * 3u, gi * 3u + 1u, gi * 3u + 2u);
    }
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
