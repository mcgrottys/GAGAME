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
    uint4 gA;   // x = the meshlet target's SRV, y = the cache colour's SRV, z = the classes' UAV, w = this view's first
    uint4 gB;   // the view's rectangle: x0, y0, w, h
};

Texture2D<uint> gId[] : register(t0, space1);
Texture2D gCol[] : register(t0, space2);
RWStructuredBuffer<uint> gOut[] : register(u0, space3);

// TWO BITS A MESHLET (2026-10-14): 1, a pixel of it is land the water cannot touch; 2, a pixel of it
// has water, or is land beside water. A kept frame leaves to the cache only a meshlet that is 1 alone.
// The water's mesh moves with the waves while the cache stands: a crest at the full draw hides the
// meshlets behind it, and when it falls on a kept frame what it uncovers must be drawn. A meshlet that
// won no pixel (0) or whose land borders water (2) is drawn again; the land beside the water is drawn
// over its own cached depth, the rest rejected by the depth the composite wrote. MEASURED: classed by
// its own pixels alone, the far flats showed holes of what lay under the globe, the plates that came
// and went with the sea (Mark, 2026-10-14; gone with every meshlet drawn, render.keep false likewise).
bool WaterAt(int3 q) {
    if (q.x < int(gB.x) || q.y < int(gB.y) || q.x >= int(gB.x + gB.z) || q.y >= int(gB.y + gB.w)) return false;
    return gId[gA.x].Load(q).x != 0u && gCol[gA.y].Load(q).a < 0.75f;
}

[numthreads(8, 8, 1)]
void CsClassify(uint3 t : SV_DispatchThreadID) {
    if (t.x >= gB.z || t.y >= gB.w) return;
    const int3 p = int3(int(gB.x + t.x), int(gB.y + t.y), 0);
    const uint m = gId[gA.x].Load(p).x;
    if (m == 0u) return;   // no meshlet drew this pixel
    const bool wet = gCol[gA.y].Load(p).a < 0.75f || WaterAt(p + int3(1, 0, 0)) || WaterAt(p - int3(1, 0, 0)) ||
                     WaterAt(p + int3(0, 1, 0)) || WaterAt(p - int3(0, 1, 0));
    uint was;
    InterlockedOr(gOut[gA.z][gA.w + m - 1u], wet ? 2u : 1u, was);
}
