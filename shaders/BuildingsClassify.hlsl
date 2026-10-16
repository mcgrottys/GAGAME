// ================================================================================================
//  BuildingsClassify.hlsl - THE WALK'S PER-BUILDING WORK, ON THE GPU (2026-10-15; the owner: the
//  walk should be O(log n) where it is O(N)).
//
//  The walk (scene/BuildingLayer.cpp Walk) visits the tree's nodes and, for each node it reaches,
//  names ONE RANGE of the stands' record pool: its buildings, as its stand wrote them once when it
//  landed (the records stated about the stand's own centre -- a translator per node). No fold:
//  a page that has not landed shows nothing until it does (the owner, 2026-10-15). This kernel expands those ranges: one thread a record,
//  the SIZE LAW the walk applied per building before, moved here unchanged --
//      a record is drawn when its diameter through the air covers lodPixels (sz >= tau);
//      it is its own form (the mesh) from shapePixels, else its moment box, and under boxPixels
//      a point; a building whose detail cell's prisms are drawn is not drawn again;
//  -- and appends the 48-byte instance each draw stage reads (Buildings.hlsl ReadInst) to that
//  form's list. Four passes on one buffer: clear the header; count each form; write the three
//  lists' bases and the draws' indirect arguments; write the records. The two classifying passes
//  are the same pure function of the same inputs, so their verdicts agree record for record.
//
//  Every threshold is the node's through the air: tau = lodPixels x air, where air = 1 / T(r) at
//  the node's nearest point (the walk's own number, carried in the node). A record that stands
//  nothing claims no size (rho = 0): one law, no special case.
// ================================================================================================
cbuffer ClassifyCb : register(b0) {
    float4 gClsEye;    // xyz the eye about the walk's origin (m), w a pixel's angle (rad)
    float4 gClsTau;    // x lodPixels, y shapePixels, z boxPixels, w 1 when the shape pool stands (else 0)
    uint4 gClsN;       // x nodes, y cap (records sent), z the cells' first row, w how many cells
    uint4 gClsPass;    // x the pass: 0 clear, 1 count, 2 arguments, 3 write
};
// A NODE (two rows): the stand's centre about the walk's origin and its first record in the
// pool; the node's first thread, its thread count (its buildings) and its air factor.
StructuredBuffer<uint4> gNodes : register(t0);
// A STAND'S RECORD (four rows, scene/BuildingLayer.h StoodRec): p.xyz about the stand's centre
// and rho; the box's rotor; a1, a2, hz, zc (the centroid's height over the ground); the shape's
// pool address (~0: none), the heading, the detail cell (two int16), the page's landing.
StructuredBuffer<uint4> gStood : register(t1);
RWStructuredBuffer<uint4> gOut : register(u0);   // the walk's buffer: the header, then the lists

static const uint kHeader = 8u;
static const uint kNone = 3u;

float3 Rotate(float4 q, float3 v) { return v + 2.0f * cross(q.xyz, cross(q.xyz, v) + q.w * v); }
float4 QMul(float4 a, float4 b) { return float4(a.w * b.xyz + b.w * a.xyz + cross(a.xyz, b.xyz), a.w * b.w - dot(a.xyz, b.xyz)); }

// The thread's node: the last whose first thread is at or before t (a binary search over the
// walk's list, log2(nodes) reads).
uint NodeOf(uint t) {
    uint lo = 0u, hi = gClsN.x - 1u;
    while (lo < hi) {
        const uint mid = (lo + hi + 1u) >> 1;
        if (gNodes[mid * 2u + 1u].x <= t) lo = mid; else hi = mid - 1u;
    }
    return lo;
}

struct Verdict {
    uint form;      // 0 box, 1 shape, 2 point, kNone
    uint4 r0, r1, r2;
};
Verdict Classify(uint t) {
    Verdict v;
    v.form = kNone;
    v.r0 = v.r1 = v.r2 = uint4(0, 0, 0, 0);
    if (t >= gClsN.y || gClsN.x == 0u) return v;
    const uint n = NodeOf(t);
    const uint4 n0 = gNodes[n * 2u], n1 = gNodes[n * 2u + 1u];
    const uint i = t - n1.x;
    if (i >= n1.y) return v;
    const uint at = (n0.w + i) * 4u;
    const uint4 s0 = gStood[at], s1 = gStood[at + 1u], s2 = gStood[at + 2u], s3 = gStood[at + 3u];
    const float3 p = asfloat(n0.xyz) + asfloat(s0.xyz);   // about the walk's origin
    const float rho = asfloat(s0.w), air = asfloat(n1.z);
    const float d = max(length(p - gClsEye.xyz), 1.0f);
    const float sz = 2.0f * rho / (d * gClsEye.w);
    if (!(sz >= gClsTau.x * air)) return v;   // under a pixel through the air (and a record of no size)
    for (uint c = 0u; c < gClsN.w; ++c) {   // its detail cell's prisms are drawn: not again
        if (gNodes[gClsN.z + c].x == s3.z) return v;
    }
    const float4 q = asfloat(s1);
    const float4 ext = asfloat(s2);   // a1, a2, hz, zc
    const float landed = asfloat(s3.w);
    if (gClsTau.w > 0.0f && s3.x != 0xFFFFFFFFu && sz >= gClsTau.y * air) {
        // ITS OWN FORM: the ground under the centroid (p less zc up), and the ground frame's rotor --
        // the box's with the heading taken back out (a rotor about up by -heading, applied first).
        const float3 U = Rotate(q, float3(0.0f, 1.0f, 0.0f));
        const float hd = asfloat(s3.y);
        const float4 back = float4(0.0f, sin(0.5f * hd), 0.0f, cos(0.5f * hd));
        v.form = 1u;
        v.r0 = uint4(asuint(p - U * ext.w), asuint(1.0f));
        v.r1 = asuint(QMul(q, back));
        v.r2 = uint4(s3.x, asuint(landed), 0u, 0u);
        return v;
    }
    const float wpx = 2.0f * max(ext.x, ext.y) / (d * gClsEye.w);
    v.form = wpx < gClsTau.z * air ? 2u : 0u;
    v.r0 = uint4(asuint(p), asuint(1.0f));
    v.r1 = s1;
    v.r2 = uint4(asuint(ext.x), asuint(ext.y), asuint(ext.z), asuint(landed));
    return v;
}

[numthreads(64, 1, 1)]
void CsClassify(uint3 id : SV_DispatchThreadID) {
    const uint pass = gClsPass.x;
    if (pass == 0u) {   // the header: counts and cursors to zero, the cap said
        if (id.x == 0u) {
            gOut[0] = uint4(0u, 0u, 0u, gClsN.y);
            gOut[5] = uint4(0u, 0u, 0u, 0u);
        }
        return;
    }
    if (pass == 2u) {   // the draws' arguments from the counts; the cursors start at zero
        if (id.x == 0u) {
            const uint3 n = gOut[0].xyz;
            const uint g = (n.x + 15u) / 16u;   // 16 boxes a mesh group; past 65535 groups a second row
            gOut[1] = uint4(min(g, 65535u), (g + 65534u) / 65535u, 1u, 0u);
            gOut[2] = uint4((n.y + 63u) / 64u, 1u, 1u, 0u);
            gOut[3] = uint4(n.z, 1u, 0u, 0u);
            gOut[4] = uint4(n.x * 36u, 1u, 0u, 0u);
            gOut[5] = uint4(0u, 0u, 0u, 0u);
        }
        return;
    }
    const Verdict v = Classify(id.x);
    if (v.form == kNone) return;
    uint old;
    if (pass == 1u) {
        if (v.form == 0u) InterlockedAdd(gOut[0].x, 1u, old);
        else if (v.form == 1u) InterlockedAdd(gOut[0].y, 1u, old);
        else InterlockedAdd(gOut[0].z, 1u, old);
        return;
    }
    // pass 3: the record into its form's list, the lists packed in form order
    const uint3 n = gOut[0].xyz;
    uint base;
    if (v.form == 0u) { InterlockedAdd(gOut[5].x, 1u, old); base = 0u; }
    else if (v.form == 1u) { InterlockedAdd(gOut[5].y, 1u, old); base = n.x; }
    else { InterlockedAdd(gOut[5].z, 1u, old); base = n.x + n.y; }
    const uint row = kHeader + (base + old) * 3u;
    gOut[row] = v.r0;
    gOut[row + 1u] = v.r1;
    gOut[row + 2u] = v.r2;
}
