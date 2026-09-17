// Sky. A fullscreen pass with depth test and depth write both OFF, drawn before anything else.
// It uses the SAME SkyRadianceDir() every reflection uses, from Common.hlsli -- with two copies
// of a sky model, a reflection stops matching the sky above it.
//
// M10: the dome is evaluated in ITS OWN world's frame (SkyLayer::SetSkyFrame). The rows take the
// view ray from the camera's frame into the sky's, and the sun is the sky's sun. Identity and
// the scene's sun are the default -- the old pass exactly; inside a twisted Droste level under
// realistic lighting the backdrop is the root's sky, turned.
cbuffer SkyFrameCb : register(b1) {
    float4 gSkyR0;
    float4 gSkyR1;
    float4 gSkyR2;
    float4 gSkySun;
    // ---- THE GATE'S WINDOW HAS A SKY TOO (M13). The gate carries the destination's GEOMETRY
    // through one motor and clips it per pixel to the box; the backdrop knew nothing about the
    // box, so above the destination's horizon the window showed the observer's own sky -- a hole
    // in the illusion exactly where the sea met the air. A window is a transform on the whole
    // view, not on its surfaces, so the same slab test runs here and the pixels inside it are
    // answered in the DESTINATION's frame: its rows, and its sun.
    //
    // Not a second camera: a camera would be a frustum placed once, right from one angle and
    // wrong the moment the eye slid along the box. This is per pixel, so it holds from any angle.
    float4 gWinA;         // x = how many windows deep the view's chain goes (0 = none)
    float4 gWinBox[28];   // the chain, packed as scene/WindowBox.h packs it (Globe.hlsl's boxes)
    // THE RAY, CARRIED. A backdrop pixel whose ray passes k windows has gone through them, so its
    // sky is the air marched from where the k-th lands -- that place's zenith, the one sun as seen
    // from there, and its distance from the planet's centre -- all said in this frame, because the
    // gates draw each far place in this frame's coordinates.
    float4 gWinUp[7];     // per depth: xyz = the zenith there, in this frame; w = its eye radius (m)
    float4 gWinSun[7];    // per depth: the one sun as seen from there, in this frame
};
#define GA_SUN_DIR (gSkySun.xyz)
#include "Common.hlsli"

struct VsOut {
    float4 pos : SV_Position;
    float2 ndc : TEXCOORD0;
};

VsOut VsMain(uint vid : SV_VertexID) {
    VsOut o;
    const float2 xy = float2((vid == 1) ? 3.0f : -1.0f, (vid == 2) ? 3.0f : -1.0f);
    o.pos = float4(xy, 0.0f, 1.0f);
    o.ndc = xy;
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    const float3 r = ViewRay(i.ndc);
    // Through k windows, the sky is the k-th place's. The walk is the one in Common.hlsli that the
    // globe's surface pixels use, asked along this ray -- a backdrop ray has no end, so it is asked
    // at a point far down it, which is the same statement for finite boxes.
    const uint k = WindowChainDepth(r * 1e9f, gWinBox, (uint)gWinA.x);
    if (k > 0u) {
        const uint j = k - 1u;
        return float4(SkyRadianceAt(r, normalize(gWinUp[j].xyz), normalize(gWinSun[j].xyz),
                                    gWinUp[j].w), 1.0f);
    }
    const float3 d = float3(dot(gSkyR0.xyz, r), dot(gSkyR1.xyz, r), dot(gSkyR2.xyz, r));
    return float4(SkyRadianceDir(d), 1.0f);
}
