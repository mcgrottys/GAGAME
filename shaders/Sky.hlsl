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
    float4 gWinBoxR0;   // rows: true camera frame -> the box's own frame; w = half extent
    float4 gWinBoxR1;
    float4 gWinBoxR2;
    float4 gWinBoxC;    // the box's centre relative to the eye; w != 0 = a window is in view
    // The gate draws the far place in THIS frame's coordinates, so a backdrop ray inside the box
    // needs no motor -- only the other place's UP. These rows are gSkyR0..R2's own law (the
    // shortest arc from the zenith onto +y) asked at the carried eye, and the sun is the scene's
    // one sun said in that frame: parallel rays, one vector, a different ground under it.
    float4 gWinSkyR0;   // rows: this frame -> the DESTINATION's dome
    float4 gWinSkyR1;
    float4 gWinSkyR2;
    float4 gWinSun;     // the one sun, in the destination's dome frame
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
    // Through the window, the sky is the destination's. The slab test is the one in Common.hlsli
    // that the globe's surface pixels use, asked along this ray -- a backdrop ray has no end, so
    // it is asked at a point far down it, which is the same statement for a finite box.
    if (gWinBoxC.w != 0.0f &&
        GateSlabThrough(r * 1e9f, float3x3(gWinBoxR0.xyz, gWinBoxR1.xyz, gWinBoxR2.xyz),
                        float3(gWinBoxR0.w, gWinBoxR1.w, gWinBoxR2.w), gWinBoxC.xyz)) {
        const float3 dw =
            float3(dot(gWinSkyR0.xyz, r), dot(gWinSkyR1.xyz, r), dot(gWinSkyR2.xyz, r));
        return float4(SkyRadianceDirSun(dw, gWinSun.xyz), 1.0f);
    }
    const float3 d = float3(dot(gSkyR0.xyz, r), dot(gSkyR1.xyz, r), dot(gSkyR2.xyz, r));
    return float4(SkyRadianceDir(d), 1.0f);
}
