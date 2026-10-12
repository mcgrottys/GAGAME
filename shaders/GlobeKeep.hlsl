// ================================================================================================
//  THE GLOBE KEPT (GlobeLayer.h Keep): the composite of a view's cached land.
//
//  The globe's surface is a pure function of its inputs -- the records the walk made, the rows it
//  draws with, the residency, the sun and the tide -- so while those stand its picture stands. A
//  full draw renders into the frame as ever and, beside it, into the view's cache colour (the
//  pixel's class in the alpha: 1 land whole, 0.5 water has a share, 0 not drawn) and the meshlet
//  that drew each pixel; the frame's depth is copied beside them. On a kept frame this pass puts
//  the cached LAND back into the frame at the globe's place in the layer order, depth-tested and
//  written as the surface would have been; the meshlets that had water are then drawn anew over it
//  (GlobeMesh.hlsl MsMain, by the classes GlobeClass.hlsl wrote), their land pixels rejected by the
//  depth they already have.
// ================================================================================================
#include "Common.hlsli"

cbuffer KeepCb : register(b1) {
    uint4 gKeep;   // x = the cache's colour SRV, y = its depth SRV
};

struct KeepVsOut {
    float4 pos : SV_Position;
};

KeepVsOut VsKeep(uint vid : SV_VertexID) {
    const float2 xy = float2((vid == 1) ? 3.0f : -1.0f, (vid == 2) ? 3.0f : -1.0f);
    KeepVsOut o;
    o.pos = float4(xy, 0.0f, 1.0f);
    return o;
}

struct KeepPsOut {
    float4 c : SV_Target;
    float d : SV_Depth;
};

KeepPsOut PsKeep(KeepVsOut i) {
    const int3 p = int3((int2)i.pos.xy, 0);
    const float d = gTex[gKeep.y].Load(p).x;   // reversed-Z: 0 = nothing drawn there
    const float4 c = gTex[gKeep.x].Load(p);
    if (d <= 0.0f || c.a < 0.75f) discard;     // the land alone; the water is drawn anew
    KeepPsOut o;
    o.c = float4(c.rgb, 1.0f);
    o.d = d;
    return o;
}
