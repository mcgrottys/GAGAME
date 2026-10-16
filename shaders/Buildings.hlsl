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
// THE WORLD BEING DRAWN (Globe.hlsl's law for its levels): the sky and haze helpers read the sun,
// the zenith and the eye's radius through these names, so a window world's buildings are lit by
// that world's sun and sky (the view's world table, loaded at the top of PsMain), not the eye's own.
static float3 sLvlSun = float3(0.0f, 1.0f, 0.0f);
static float3 sLvlSkyUp = float3(0.0f, 1.0f, 0.0f);
static float sLvlSkyEyeR = 6371000.0f;
#define GA_SKY_EYE_R (sLvlSkyEyeR)
#define GA_SUN_DIR (sLvlSun)
#define GA_SKY_UP (sLvlSkyUp)
#include "Common.hlsli"
// THE GROUND UNDER THE SUN (Compose.hlsli, TerrainRay.hlsli): a face asks the composed height field
// whether a hill stands between it and the sun, and the sun map whether another building does --
// the same two questions the globe's surface asks of the same rows (b2 is bound for every layer).
#include "Compose.hlsli"
#include "TerrainRay.hlsli"

cbuffer BuildingCb : register(b1) {
    float4 gBdOrigin;   // xyz = the cell's origin relative to the eye (m), w = brightness
    float4 gBdTime;     // x = the layer's now (s), y = a page's fade-in (s), z = a pixel's angle (rad)
    uint4 gBdMesh;      // z the depth of the world this draw is in (0 the eye's own: the hulls' rule);
                        // w the shape pool's heap slot (compose/BuildingShape.h); x, y unused -- a form's
                        // first row and count are the walk buffer's own header (FormBase, FormCount)
    float4 gBdWorld;    // THE WORLD'S TURN in the true frame (a rotor, identity for the eye's own): a window
                        // world's instances are composed with it, their centres turned by it, about gBdOrigin
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

// A rotor on a vector (q = (xyz, w)), and the rotor applying b then a (the Hamilton product).
float3 Rotate(float4 q, float3 v) { return v + 2.0f * cross(q.xyz, cross(q.xyz, v) + q.w * v); }
// The rotor applying b then a (the Hamilton product, w last).
float4 QMul(float4 a, float4 b) { return float4(a.w * b.xyz + b.w * a.xyz + cross(a.xyz, b.xyz), a.w * b.w - dot(a.xyz, b.xyz)); }

VsOut VsMain(uint vid : SV_VertexID) {
    const BuildingVertex v = gVerts[vid];
    VsOut o;
    o.rel = gBdOrigin.xyz + Rotate(gBdWorld, v.pos);
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    o.n = Rotate(gBdWorld, v.n);
    o.col = kPalette[uint(v.kind) & 3u] * gBdOrigin.w;
    o.alpha = 1.0f;
    return o;
}

// THE WALK'S INSTANCES (scene/BuildingLayer.h BuildingLayer::Box, 48 bytes, three float4 rows),
// WRITTEN BY THE GPU (BuildingsClassify.hlsl, 2026-10-15): the walk names its nodes, the classify
// pass expands each node's stand into the three forms' lists -- boxes, then shapes, then points,
// packed -- and writes their counts and the draws' indirect arguments into the buffer's own
// HEADER (kWalkHeader rows), so a draw reads its first row and its count from here and the CPU
// never knows how many:
//   row 0  nBoxes, nShapes, nPoints, cap     the counts (cap: the records the walk sent)
//   row 1  the boxes' mesh dispatch (x, y, 1)     ExecuteIndirect's arguments
//   row 2  the shapes' mesh dispatch (x, 1, 1)
//   row 3  the points' draw (n, 1, 0, 0)
//   row 4  the boxes' vertex draw (36 n, 1, 0, 0)   (a device without mesh shaders)
//   row 5  the write pass's running cursors
// A record:
//   row 0  c.xyz, alpha      the centre (a box) or the ground under the centroid (a shape), metres
//                            from the walk's origin; alpha 1 (opaque)
//   row 1  q                 THE ATTITUDE AS ONE ROTOR: east, up and north are q acting on the flat
//                            frame's x, y and z (a right-handed triple) -- one object in place of
//                            three unit vectors
//   row 2  a box: a1, a2, hz, landed      the half-extents (long, short, up) and the page's landing
//          a shape: addr, landed, 0, 0    the record's byte address in the shape pool (an integer)
// Read as INTEGERS (a word of two 16-bit numbers is a denormal as a float, and a float load may
// flush it to zero) and reinterpreted where a float is meant.
StructuredBuffer<uint4> gWalk : register(t0, space0);
StructuredBuffer<uint4> gShapePool[] : register(t0, space7);   // the heap's buffers (gWorlds' range)
static const uint kWalkHeader = 8u;
// A form's count and its first row: 0 the boxes, 1 the shapes, 2 the points.
uint FormCount(uint f) { const uint3 n = gWalk[0].xyz; return f == 0u ? n.x : (f == 1u ? n.y : n.z); }
uint FormBase(uint f) { const uint3 n = gWalk[0].xyz; return kWalkHeader + (f == 0u ? 0u : (f == 1u ? n.x : n.x + n.y)) * 3u; }

struct Inst {
    float3 c; float alpha;
    float4 q;
    uint4 x;   // row 2 as read
};
Inst ReadInst(uint row) {
    Inst o;
    const float4 r0 = asfloat(gWalk[row]);
    o.c = r0.xyz;
    o.alpha = r0.w;
    o.q = asfloat(gWalk[row + 1u]);
    o.x = gWalk[row + 2u];
    return o;
}

// THE FAR BOXES (docs/BUILDING_LOD.md): a building past its own form as its moment box, 8 vertices
// and 12 triangles a box, 16 boxes a mesh group. The faces share their corners, so no vertex carries
// a normal: the pixel shader takes the face's from the surface's own derivatives (a flat face has
// one), and reads the top as a roof by its tilt. Counter-clockwise from outside on every face.
static const uint3 kBoxTris[12] = {
    uint3(1, 3, 7), uint3(1, 7, 5),   // +east
    uint3(0, 4, 6), uint3(0, 6, 2),   // -east
    uint3(2, 6, 7), uint3(2, 7, 3),   // +north
    uint3(0, 1, 5), uint3(0, 5, 4),   // -north
    uint3(4, 5, 7), uint3(4, 7, 6),   // +up: the roof
    uint3(0, 2, 3), uint3(0, 3, 1)    // -up
};
[outputtopology("triangle")]
[numthreads(128, 1, 1)]
void MsBox(uint gi : SV_GroupIndex, uint3 gid : SV_GroupID, out vertices VsOut verts[128], out indices uint3 tris[192]) {
    const uint b0 = (gid.x + gid.y * 65535u) * 16u;   // past 65535 groups the dispatch takes a second row
    const uint nB = FormCount(0u);
    const uint n = b0 < nB ? min(16u, nB - b0) : 0u;
    SetMeshOutputCounts(n * 8u, n * 12u);
    const uint bi = gi >> 3, corner = gi & 7u;
    if (bi >= n) return;
    Inst b = ReadInst(FormBase(0u) + (b0 + bi) * 3u);
    b.c = Rotate(gBdWorld, b.c);
    b.q = QMul(gBdWorld, b.q);
    const float3 E = Rotate(b.q, float3(1, 0, 0)), U = Rotate(b.q, float3(0, 1, 0)), N = Rotate(b.q, float3(0, 0, 1));
    const float3 ext = asfloat(b.x.xyz);
    const float3 sgn = float3((corner & 1u) ? 1.0f : -1.0f, (corner & 2u) ? 1.0f : -1.0f, (corner & 4u) ? 1.0f : -1.0f);
    VsOut o;
    o.rel = gBdOrigin.xyz + b.c + sgn.x * ext.x * E + sgn.y * ext.y * N + sgn.z * ext.z * U;
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    o.n = float3(0, 0, 0);   // flat: the pixel shader's derivatives
    o.col = kPalette[0] * gBdOrigin.w;
    o.alpha = 1.0f;   // opaque: a box drawn is drawn whole (the owner saw the transparent ones)
    verts[gi] = o;
    // 12 triangles over 8 threads: each writes one, the first four write a second.
    const uint tb = bi * 12u, vb = bi * 8u;
    tris[tb + corner] = kBoxTris[corner] + vb;
    if (corner < 4u) tris[tb + 8u + corner] = kBoxTris[8u + corner] + vb;
}

// ...and the same boxes through the vertex stage where there are no mesh shaders (36 vertices a box).
VsOut VsBox(uint vid : SV_VertexID) {
    Inst b = ReadInst(FormBase(0u) + (vid / 36u) * 3u);
    b.c = Rotate(gBdWorld, b.c);
    b.q = QMul(gBdWorld, b.q);
    const float3 E = Rotate(b.q, float3(1, 0, 0)), U = Rotate(b.q, float3(0, 1, 0)), N = Rotate(b.q, float3(0, 0, 1));
    const float3 ext = asfloat(b.x.xyz);
    const uint f = (vid % 36u) / 6u, t = vid % 6u;   // face, then its two triangles
    const float s = (f & 1u) ? -1.0f : 1.0f;         // faces in pairs along E, N, U
    const uint ax = f >> 1;
    const float3 A = ax == 0u ? E * ext.x : (ax == 1u ? N * ext.y : U * ext.z);
    const float3 B = ax == 0u ? N * ext.y : (ax == 1u ? U * ext.z : E * ext.x);
    const float3 C = ax == 0u ? U * ext.z : (ax == 1u ? E * ext.x : N * ext.y);
    static const float2 kQuad[6] = {float2(-1, -1), float2(1, -1), float2(1, 1), float2(-1, -1), float2(1, 1), float2(-1, 1)};
    const float2 q = float2(kQuad[t].x * s, kQuad[t].y);   // counter-clockwise from outside on both faces
    VsOut o;
    o.rel = gBdOrigin.xyz + b.c + s * A + q.x * B + q.y * C;
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    o.n = s * A / max(length(A), 1e-6f);
    o.col = kPalette[(ax == 2u && s > 0.0f) ? 1u : 0u] * gBdOrigin.w;
    o.alpha = 1.0f;
    return o;
}

// THE POINT (the owner, 2026-10-15): the smallest of the three forms, a building under boxPixels
// across -- one vertex at its centre, and its ROTOR is what makes it a building and not a dot:
// the eye's elevation against the rotor's up, sin e, is the share of the roof the eye sees and
// cos e the share of the walls, so the point's colour is that mix of the palette and its normal
// the mix of the roof's up and the wall that faces the eye -- the same law the building field
// gives the walls it does not draw (Globe.hlsl PsSurface), so a suburb is lit the same whether it
// stands as points, as boxes or as the field. PsMain lights it as any face.
VsOut VsPoint(uint vid : SV_VertexID) {
    Inst b = ReadInst(FormBase(2u) + vid * 3u);
    b.c = Rotate(gBdWorld, b.c);
    b.q = QMul(gBdWorld, b.q);
    const float3 U = Rotate(b.q, float3(0, 1, 0));
    VsOut o;
    o.rel = gBdOrigin.xyz + b.c;
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    const float3 v = -o.rel / max(length(o.rel), 1e-3f);   // toward the eye
    const float sinE = abs(dot(v, U));
    float3 nw = v - dot(v, U) * U;                           // the wall that faces the eye
    nw = dot(nw, nw) > 1e-8f ? normalize(nw) : U;
    o.n = normalize(lerp(nw, U, sinE));
    o.col = lerp(kPalette[0], kPalette[1], sinE) * gBdOrigin.w;
    o.alpha = 1.0f;
    return o;
}

// THE BUILDINGS' OWN FORMS (compose/BuildingShape.h, GALOD04), extruded from the footprints the
// shape pool holds, so a building is the solid it is at every distance, never a box.
//
// THE STREAMS (2026-10-10). An amplification group takes 64 instances, reads each record's counts
// from the pool, and lays the group's walls end to end as one stream of EDGES and its roofs as one
// stream of TRIANGLES; it launches a mesh group for every 64 of either, full whatever the buildings'
// sizes (a four-edged house no longer takes a group of its own). A mesh thread finds its instance
// by where its stream index falls among the payload's running counts.
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

struct ShapePayload {
    uint instBase, edgeTotal, triTotal, pad;
    uint edgeStart[64];   // edges before instance i of the group (exclusive)
    uint triStart[64];    // roof triangles before it
};
groupshared ShapePayload sPay;
groupshared uint sScanE[64], sScanT[64];

[numthreads(64, 1, 1)]
void AsShape(uint gi : SV_GroupIndex, uint3 gid : SV_GroupID) {
    const uint inst = gid.x * 64u + gi;
    uint edges = 0u, tris = 0u;
    if (inst < FormCount(1u)) {
        const uint addr = gWalk[FormBase(1u) + inst * 3u + 2u].x;
        const uint w0 = ShapeU32(addr), w1 = ShapeU32(addr + 4u);
        const uint nV = w0 & 0xFFFFu, flags = (w1 >> 8) & 0xFFu;
        edges = (flags & 4u) ? 4u * (max(nV, 1u) - 1u) : nV;   // a ribbon of n points: n - 1 segments, four faces each
        tris = w0 >> 16;
    }
    // Exclusive running counts over the 64, a Hillis-Steele scan in groupshared (two waves or one).
    sScanE[gi] = edges;
    sScanT[gi] = tris;
    GroupMemoryBarrierWithGroupSync();
    [unroll] for (uint d = 1u; d < 64u; d <<= 1) {
        const uint e = gi >= d ? sScanE[gi - d] : 0u, t = gi >= d ? sScanT[gi - d] : 0u;
        GroupMemoryBarrierWithGroupSync();
        sScanE[gi] += e;
        sScanT[gi] += t;
        GroupMemoryBarrierWithGroupSync();
    }
    sPay.edgeStart[gi] = sScanE[gi] - edges;   // inclusive -> exclusive
    sPay.triStart[gi] = sScanT[gi] - tris;
    if (gi == 63u) {
        sPay.instBase = gid.x * 64u;
        sPay.edgeTotal = sScanE[63];
        sPay.triTotal = sScanT[63];
        sPay.pad = 0u;
    }
    GroupMemoryBarrierWithGroupSync();
    const uint groups = (sPay.edgeTotal + 63u) / 64u + (sPay.triTotal + 63u) / 64u;
    DispatchMesh(groups, 1, 1, sPay);
}

// The instance whose stream holds index s: the last i with start[i] <= s (six halvings over 64).
uint InstanceOf(in ShapePayload pay, bool roof, uint s) {
    uint lo = 0u, hi = 63u;
    [unroll] for (uint k = 0u; k < 6u; ++k) {
        const uint mid = (lo + hi + 1u) >> 1;
        const uint st = roof ? pay.triStart[mid] : pay.edgeStart[mid];
        if (st <= s) lo = mid; else hi = mid - 1u;
    }
    return lo;
}

[outputtopology("triangle")]
[numthreads(64, 1, 1)]
void MsShape(uint gi : SV_GroupIndex, uint3 gid : SV_GroupID, in payload ShapePayload pay,
             out vertices VsOut verts[256], out indices uint3 tris[128]) {
    const uint edgeGroups = (pay.edgeTotal + 63u) / 64u;
    const bool roof = gid.x >= edgeGroups;
    const uint s0 = (roof ? gid.x - edgeGroups : gid.x) * 64u;
    const uint total = roof ? pay.triTotal : pay.edgeTotal;
    const uint n = s0 < total ? min(64u, total - s0) : 0u;
    // One count for the group, set before any output (the validator's rule): 4 vertices and 2
    // triangles an edge (a wall, or a ribbon's segment), or 3 and 1 a roof triangle.
    SetMeshOutputCounts(roof ? n * 3u : n * 4u, roof ? n : n * 2u);
    if (gi >= n) return;
    const uint s = s0 + gi;
    const uint k = InstanceOf(pay, roof, s);
    const uint local = s - (roof ? pay.triStart[k] : pay.edgeStart[k]);
    Inst b = ReadInst(FormBase(1u) + (pay.instBase + k) * 3u);
    b.c = Rotate(gBdWorld, b.c);
    b.q = QMul(gBdWorld, b.q);
    const float3 c = b.c, E = Rotate(b.q, float3(1, 0, 0)), U = Rotate(b.q, float3(0, 1, 0)), N = Rotate(b.q, float3(0, 0, 1));
    const uint addr = b.x.x;
    const uint w0 = ShapeU32(addr), w1 = ShapeU32(addr + 4u);
    const uint nVerts = w0 & 0xFFFFu, nTris = w0 >> 16, nRings = w1 & 0xFFu, flags = (w1 >> 8) & 0xFFu;
    const float bottom = asfloat(ShapeU32(addr + 8u)), top = asfloat(ShapeU32(addr + 12u));
    const uint ringAt = addr + 16u, xyAt = ringAt + 2u * nRings, triAt = xyAt + 4u * nVerts;
    const float unit = (flags & 2u) ? 1.0f : 0.1f;
    const float part = (flags & 1u) ? 2.0f : 0.0f;
    const bool ribbon = (flags & 4u) != 0u;   // shape::kRibbon: a road piece, one quad a segment
    VsOut o;
    o.alpha = 1.0f;   // opaque (a ribbon's width under a pixel is the one alpha, below)
    if (ribbon) {   // THE RIBBON IS A SLAB: segment e of the polyline, mitred at both vertices, as four
        // one-sided faces -- top, underside, right and left -- so a bridge's deck has an underside for
        // the eye on the water and the cull stays (the owner, 2026-10-11: keep the cull, lose no face).
        const uint zAt = triAt + 6u * nTris;                    // the heights the page's reader filled
        const float w = float(ShapeU16(addr + 6u)) * 0.1f;      // ShapeHead.pad: the width, decimetres
        const uint e = local >> 2, face = local & 3u;
        float3 P[4];   // prev, a, b, next (an end repeats itself)
        [unroll] for (uint j = 0u; j < 4u; ++j) {
            const uint v = uint(clamp(int(e) - 1 + int(j), 0, int(nVerts) - 1));
            P[j] = float3(ShapeI16(xyAt + 4u * v) * unit, ShapeI16(xyAt + 4u * v + 2u) * unit, ShapeI16(zAt + 2u * v) * 0.1f);
        }
        const float2 sg = P[2].xy - P[1].xy;
        const float sl = length(sg);
        const float2 sn = sl > 1e-4f ? float2(sg.y, -sg.x) / sl : float2(0.0f, 0.0f);   // the segment's right
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
        o.col = kPalette[(flags & 8u) ? 5u : 4u] * gBdOrigin.w;
        const float2 R1 = P[1].xy + m[0] * hw, L1 = P[1].xy - m[0] * hw, R2 = P[2].xy + m[1] * hw, L2 = P[2].xy - m[1] * hw;
        const float t1 = P[1].z + top, t2 = P[2].z + top, b1 = P[1].z + bottom, b2 = P[2].z + bottom;
        float2 xy[4];
        float zz[4];
        uint3 ta, tb;
        if (face == 0u) {          // the top, as it was: seen from above
            xy[0] = R1; xy[1] = L1; xy[2] = R2; xy[3] = L2;
            zz[0] = t1; zz[1] = t1; zz[2] = t2; zz[3] = t2;
            o.n = U;
            ta = uint3(0u, 2u, 1u); tb = uint3(1u, 2u, 3u);
        } else if (face == 1u) {   // the underside: the top's quad at the bottom, wound the other way
            xy[0] = R1; xy[1] = L1; xy[2] = R2; xy[3] = L2;
            zz[0] = b1; zz[1] = b1; zz[2] = b2; zz[3] = b2;
            o.n = -U;
            ta = uint3(0u, 1u, 2u); tb = uint3(1u, 3u, 2u);
        } else {                   // a side: the wall's own law, its outward the edge's right
            const bool right = face == 2u;
            const float2 A = right ? R1 : L2, B = right ? R2 : L1;
            const float zA0 = right ? b1 : b2, zB0 = right ? b2 : b1, zA1 = right ? t1 : t2, zB1 = right ? t2 : t1;
            xy[0] = A; xy[1] = B; xy[2] = B; xy[3] = A;
            zz[0] = zA0; zz[1] = zB0; zz[2] = zB1; zz[3] = zA1;
            o.n = (right ? sn.x : -sn.x) * E + (right ? sn.y : -sn.y) * N;
            ta = uint3(0u, 1u, 2u); tb = uint3(0u, 2u, 3u);
        }
        [unroll] for (uint j = 0u; j < 4u; ++j) {
            o.rel = gBdOrigin.xyz + c + xy[j].x * E + xy[j].y * N + zz[j] * U;
            o.pos = mul(float4(o.rel, 1.0f), gViewProj);
            verts[gi * 4u + j] = o;
        }
        tris[gi * 2u] = ta + gi * 4u;
        tris[gi * 2u + 1u] = tb + gi * 4u;
    } else if (!roof) {   // the walls: edge e from its vertex to its ring's next
        const uint e = local;
        uint start = 0u, len = 0u;
        for (uint r = 0u; r < nRings; ++r) {
            len = ShapeU16(ringAt + 2u * r);
            if (e < start + len) break;
            start += len;
        }
        const uint f = (e + 1u < start + len) ? e + 1u : start;
        const float2 a = float2(ShapeI16(xyAt + 4u * e), ShapeI16(xyAt + 4u * e + 2u)) * unit;
        const float2 bb = float2(ShapeI16(xyAt + 4u * f), ShapeI16(xyAt + 4u * f + 2u)) * unit;
        const float2 d = bb - a;
        const float l = length(d);
        const float2 out2 = l > 1e-4f ? float2(d.y, -d.x) / l : float2(0.0f, 0.0f);   // the edge's right: out
        o.n = out2.x * E + out2.y * N;
        o.col = kPalette[uint(part)] * gBdOrigin.w;
        const float2 xy[4] = {a, bb, bb, a};
        const float zz[4] = {bottom, bottom, top, top};
        [unroll] for (uint j = 0u; j < 4u; ++j) {
            o.rel = gBdOrigin.xyz + c + xy[j].x * E + xy[j].y * N + zz[j] * U;
            o.pos = mul(float4(o.rel, 1.0f), gViewProj);
            verts[gi * 4u + j] = o;
        }
        tris[gi * 2u] = uint3(gi * 4u, gi * 4u + 1u, gi * 4u + 2u);
        tris[gi * 2u + 1u] = uint3(gi * 4u, gi * 4u + 2u, gi * 4u + 3u);
    } else {       // the roof: its triangle at the top
        o.n = U;
        o.col = kPalette[uint(part) + 1u] * gBdOrigin.w;
        [unroll] for (uint j = 0u; j < 3u; ++j) {
            const uint v = ShapeU16(triAt + 6u * local + 2u * j);
            const float2 q = float2(ShapeI16(xyAt + 4u * v), ShapeI16(xyAt + 4u * v + 2u)) * unit;
            o.rel = gBdOrigin.xyz + c + q.x * E + q.y * N + top * U;
            o.pos = mul(float4(o.rel, 1.0f), gViewProj);
            verts[gi * 3u + j] = o;
        }
        tris[gi] = uint3(gi * 3u, gi * 3u + 1u, gi * 3u + 2u);
    }
}

float4 PsMain(VsOut i) : SV_Target {
    // THE HULLS' RULE (Vessel.hlsl): a pixel belongs to the world whose depth its ray reaches, and
    // to no other -- the eye's own cut away where a window shows another place, a window world's
    // kept only where the eye reaches it through every box before it.
    const uint depth = gBdMesh.z;
    if (WindowChainDepth(i.rel, depth + 1u) != depth) discard;
    // THAT WORLD'S LIGHT: its sun, its zenith and its eye's radius from the view's world table
    // (Globe.hlsl LoadLevel), so the sky's irradiance, the sun through the air and the haze are the
    // far place's through its window, not the eye's own.
    {
        const uint s = min(depth, max(WtLevels(), 1u) - 1u);
        sLvlSun = (depth > 0u) ? normalize(WtLevelRow(s, 4u).xyz) : gSunDir.xyz;
        sLvlSkyUp = (depth > 0u) ? normalize(WtLevelRow(s, 5u).xyz) : float3(0.0f, 1.0f, 0.0f);
        sLvlSkyEyeR = length(WtLevelRow(s, 0u).xyz);
    }
    // THE LAND'S LIGHT (Globe.hlsl's law, Common.hlsli's terms): the sky's irradiance on the face
    // plus the sun through the air, which the PLANET decides is up at this point -- never the face's
    // own tilt. The point's zenith and height are asked of the sphere (its centre at flat
    // (0, -R, 0)), so a wall at night is dark however it faces.
    const float Rb = gSkyLut.y;
    const float3 fromCentre = gEyeRel.xyz + i.rel + float3(0.0f, Rb, 0.0f);
    const float r = length(fromCentre);
    const float3 up = (depth > 0u) ? sLvlSkyUp : fromCentre / r;
    const float h = (depth > 0u) ? 0.0f : r - Rb;
    // THE AIR'S ZENITH IS THE PLANET'S, NEVER THE FRAME'S (2026-10-15): the haze below took the
    // flat frame's y as up, which is the zenith only at the frame's anchor -- at Tokyo, 7,000 km
    // from it, the sky's integral ran a ray through the wrong air and a building 50 km off stayed
    // black against the hazed ground under it. The same up the light above uses.
    sLvlSkyUp = up;
    // A box's faces share their corners and carry no normal: the face's own, from the surface's
    // derivatives, turned to face the eye (the box is convex and culled, so the eye sees outsides);
    // its top is a roof by its tilt.
    float3 n;
    float3 col = i.col;
    if (dot(i.n, i.n) > 0.25f) {
        n = normalize(i.n);
    } else {
        n = normalize(cross(ddy(i.rel), ddx(i.rel)));
        if (dot(n, i.rel) > 0.0f) n = -n;
        if (dot(n, up) > 0.9f) col *= 0.65f;   // kPalette[1] / kPalette[0]: the roof's shade of the wall's
    }
    float ndl = saturate(dot(n, GA_SUN_DIR)) * PlanetShadow(up, GA_SUN_DIR, h, Rb);
    // THE GROUND AND THE OTHER BUILDINGS between this face and the sun, by the globe's own law
    // (Globe.hlsl PsSurface): the terrain march from the face's point at its pixel's grain, and the
    // sun map of the drawn solids. Asked where the sun still reaches (a set sun or a face turned
    // away needs no march), and of the eye's own world only: a window world's point is here in the
    // true frame, pulled through its window, and its ground is addressed in its own -- the step that
    // carries the march through a window is not taken yet.
    if (depth == 0u && ndl > 0.0f) {
        const float range = length(i.rel);
        const float footM = range * gBdTime.z;   // a pixel's metres at this face
        TerrainHit hit;
        ndl *= TerrainRay(up, h, i.rel, 0u, GA_SUN_DIR, footM, ComposedHeightLod(range, gBdTime.z), Rb, false, hit) *
               SunMapClear(i.rel, n);
    }
    const float3 lit = col * (SkyAmbient(n, up, h) + ndl * SunAt(up, h) * 1.15f);
    // ...and seen through THE AIR IN FRONT OF IT, the ground's own law: a box 60 km off fades into
    // the same haze as the land under it, where it stood out at full contrast.
    const float range = length(i.rel);
    return float4(AerialPerspective(lit, i.rel / max(range, 1e-3f), range), i.alpha);
}
