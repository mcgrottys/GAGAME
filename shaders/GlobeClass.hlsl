// ================================================================================================
//  THE MESHLET CLASSES (GlobeLayer.h Keep): after a full draw, each meshlet is land whole or has
//  water, by its PIXELS. The draw wrote, beside the frame, the view's cache colour (its alpha 1
//  where the pixel is land whole, 0.5 where water has a share) and the meshlet that drew each pixel
//  (its record index + 1; 0 where none did). This pass reads both over the view's rectangle and
//  marks every meshlet a water pixel belongs to; a kept frame draws those again and leaves the
//  land to the cache (GlobeMesh.hlsl MsMain). Its own root: b0 these words, the heap as
//  Texture2D<uint> (space1) and Texture2D (space2), the heap as RWStructuredBuffer<uint> (space3).
// ================================================================================================
cbuffer ClassCb : register(b0) {
    uint4 gA;   // x = the meshlet target's SRV, y = the cache colour's SRV, z = the classes' UAV
    uint4 gB;   // the view's rectangle: x0, y0, w, h
};

Texture2D<uint> gId[] : register(t0, space1);
Texture2D gCol[] : register(t0, space2);
RWStructuredBuffer<uint> gOut[] : register(u0, space3);

[numthreads(8, 8, 1)]
void CsClassify(uint3 t : SV_DispatchThreadID) {
    if (t.x >= gB.z || t.y >= gB.w) return;
    const int3 p = int3(int(gB.x + t.x), int(gB.y + t.y), 0);
    const uint m = gId[gA.x].Load(p).x;
    if (m == 0u) return;   // no meshlet drew this pixel
    const float a = gCol[gA.y].Load(p).a;
    if (a < 0.75f) gOut[gA.z][m - 1u] = 1u;   // water has a share here: drawn again on a kept frame
}
