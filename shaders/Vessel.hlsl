// ================================================================================================
//  Vessel.hlsl - M9bq: hulls, drawn from their own SPEC.
//
//  There is no vessel art in this engine and there is not going to be any yet. A VesselSpec
//  already describes where every part of a boat is and how big it is -- the buoyancy stations,
//  each tube chamber's extent and radius, where the engines hang -- because the PHYSICS needs
//  exactly that. So the layer derives a blocky solid per element and draws those. A new hull
//  costs a spec and gets a body for free, which is the same bargain GradeField makes for a new
//  dataset, and it means the picture cannot drift from the thing being simulated: if a tube is
//  drawn in the wrong place, the tube IS in the wrong place.
//
//  Geometry is procedural and indexed by SV_VertexID. This engine has no input-assembler path at
//  all (there is not one IASetVertexBuffers in src/), and a vessel is not the reason to add one.
//
//  The sandwich is applied HERE, on the GPU, from the same dual-quaternion coordinates ga::Motor
//  uses on the CPU -- GA.hlsli's MotorPoint is a line-for-line translation of Pga.h, so the two
//  sides cannot drift. Each part's motor is already hull * attach, composed on the CPU where the
//  articulation lives (a steered outboard is a rotation about a LINE, which is one motor).
// ================================================================================================
#define GA_NO_FIELD_BUFFER
#include "Common.hlsli"

cbuffer VesselCb : register(b1) {
    float4 gVsParams;   // x = part count (informational), y = brightness, zw = spare
    // ---- THE VIEW'S WINDOWS (M13). A hull is drawn once for every world the view reaches
    // through the gates -- at the pose that world's geometry is drawn by (its windows' motors,
    // inverted) -- and each copy keeps only the pixels of its own depth. So a boat that has gone
    // through a gate the eye has not is seen through that window, and a boat whose place a window
    // shows again, further down the corridor, is seen there too. The chain and the light at each
    // depth are the view's world table's (Common.hlsli Wt*).
};

struct VesselPart {
    float4 re;      // motor real part: s, r23, r31, r12   (world = hull * attach)
    float4 du;      // motor dual part: q, t01, t02, t03
    float4 half;    // xyz = half extents (m), w = palette index
    float4 opt;     // x = the depth of the world this part is drawn in (0 = the eye's own)
};
StructuredBuffer<VesselPart> gParts : register(t0, space0);

struct VsOut {
    float4 pos : SV_Position;
    float3 col : COLOR0;
    float3 n : NORMAL0;
    float3 rel : TEXCOORD0;             // the point, relative to the eye: the slab test's input
    nointerpolation float depth : TEXCOORD1;   // which world of the view's chain this part is in
};

// A box CENTRED on its motor -- unlike Markers.hlsl's pylon, which stands on its origin. A part's
// mount is its middle: that is where a spec puts a tube's axis and an engine's mass.
static const float3 kCorners[8] = {
    float3(-1, -1, -1), float3(1, -1, -1), float3(1, -1, 1), float3(-1, -1, 1),
    float3(-1,  1, -1), float3(1,  1, -1), float3(1,  1, 1), float3(-1,  1, 1)
};
// CLOCKWISE SEEN FROM OUTSIDE, which is D3D's front face -- and the table used to be the other
// way round. Culling BACK then removed every face turned toward the camera and kept only the far
// walls, so a box drew as its own inside and anything sitting within one showed straight through
// (the RHIB's ballast, visible through its hull). The rule, checkable by hand: with the
// LookToLH view and this projection there is no mirror anywhere in the chain, so a triangle winds
// clockwise on screen exactly when (v1 - v0) x (v2 - v0) points TOWARD the eye -- i.e. every
// triangle here must have that cross product along its OUTWARD normal, kFaceN[tri]. Swapping the
// last two indices of each triangle is the whole fix; kFaceN is per-triangle and does not move.
static const uint kIdx[36] = {0,5,1, 0,4,5, 1,6,2, 1,5,6, 2,7,3, 2,6,7,
                              3,4,0, 3,7,4, 4,6,5, 4,7,6, 0,1,2, 0,2,3};
static const float3 kFaceN[12] = {
    float3(0,0,-1), float3(0,0,-1), float3(1,0,0), float3(1,0,0),
    float3(0,0,1),  float3(0,0,1),  float3(-1,0,0), float3(-1,0,0),
    float3(0,1,0),  float3(0,1,0),  float3(0,-1,0), float3(0,-1,0)
};
// One colour per element KIND, so the picture reads as the physics: hull, tube, planing surface,
// thruster, foil, windage. Reading a screenshot tells you which laws are acting.
static const float3 kPalette[7] = {
    float3(0.82, 0.82, 0.86),   // Buoyancy  -- hull, light grey
    float3(0.15, 0.16, 0.19),   // Collar    -- tube, near-black like real hypalon
    float3(0.45, 0.55, 0.70),   // Planing   -- the running surface
    float3(0.90, 0.90, 0.92),   // Thruster  -- outboard cowling
    float3(0.75, 0.35, 0.20),   // Foil      -- rudder / skeg / sail
    float3(0.35, 0.65, 0.45),   // Drag      -- windage box
    float3(0.85, 0.65, 0.10)    // Ballast   -- fuel, crew, gear: where the weight actually is
};

VsOut VsMain(uint vid : SV_VertexID) {
    const uint inst = vid / 36u;
    const uint tri = (vid % 36u) / 3u;
    const VesselPart p = gParts[inst];
    const float3 local = kCorners[kIdx[vid % 36u]] * p.half.xyz;
    const float3 world = MotorPoint(p.re, p.du, local);   // the sandwich, on the GPU
    VsOut o;
    o.n = MotorDir(p.re, kFaceN[tri]);
    o.col = kPalette[uint(p.half.w) % 7u] * gVsParams.y;
    // `world` is already RELATIVE TO THE EYE: VesselLayer::Render composes the eye's reverse
    // translation onto each box's motor in doubles before the float cast, so a hull 2000 km from
    // the origin is as exact here as one at the Merrimack.
    o.pos = mul(float4(world, 1.0f), gViewProj);
    o.rel = world;
    o.depth = p.opt.x;
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    // THE GLOBE'S OWN RULE FOR ITS WORLDS (Globe.hlsl), applied to hulls: a pixel belongs to the
    // world whose depth its ray reaches, and to no other. So a hull seen through a window shows
    // only where the eye reaches it through every box before it, and a hull on this side is cut
    // away where a window shows another place -- without that half, a boat standing behind the
    // portal would be visible through it, which is the one thing a window must never do. With no
    // chain every ray is at depth 0: what is through a window cannot be seen at all.
    const uint depth = (uint)round(i.depth);
    if (WindowChainDepth(i.rel, depth + 1u) != depth) discard;
    const float3 n = normalize(i.n);
    const float3 sunHere = (depth > 0u) ? normalize(WtWinSun(depth - 1u).xyz) : gSunDir.xyz;
    const float ndl = saturate(dot(n, sunHere));
    const float sky = 0.5f + 0.5f * n.y;
    // The sky term is generous on purpose. A hull read against bright water with only a sun
    // term goes to black on every face pointing away from it, and a black silhouette on water
    // reads as a HOLE in the sea rather than as an object floating on it.
    return float4(i.col * (0.38f * sky + 0.85f * ndl), 1.0f);
}
