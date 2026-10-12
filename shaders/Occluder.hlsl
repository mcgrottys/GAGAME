// ================================================================================================
//  Occluder.hlsl - THE FRAME'S FARTHEST SURFACE, BY CELL, for the walks that decide what to load.
//
//  The owner, 2026-10-11: "avoid loading buildings that get blocked by terrain and closer
//  buildings". The depth the frame just drew holds every occluder there is -- the ground, the
//  sea, the hulls, the buildings themselves -- so the test needs no second model of any of them:
//  a view's rectangle is reduced to a small grid of the FARTHEST view-depth drawn in each cell
//  (reversed Z: the least z; nothing drawn = the sky = no limit), read back through the frame
//  ring without a stall (hal::RegionReadback), and a walk hides a node whose nearest point lies
//  beyond every cell it covers. Conservative by construction: a cell's farthest surface is the
//  most generous limit any pixel in it has.
// ================================================================================================
cbuffer OccCb : register(b0) {
    uint4 gOccA;    // x the depth's srv, y the grid's uav, z the view rect's x0, w its y0
    uint4 gOccB;    // x the rect's w, y its h, z the grid's w, w its h
    float4 gOccC;   // x nearZ (view depth = nearZ / z)
};
Texture2D gTex[] : register(t0, space1);
RWTexture2D<float> gOut[] : register(u0, space2);

[numthreads(8, 8, 1)]
void CsOccluder(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gOccB.z || id.y >= gOccB.w) return;
    const uint x0 = gOccA.z + (id.x * gOccB.x) / gOccB.z, x1 = gOccA.z + ((id.x + 1u) * gOccB.x) / gOccB.z;
    const uint y0 = gOccA.w + (id.y * gOccB.y) / gOccB.w, y1 = gOccA.w + ((id.y + 1u) * gOccB.y) / gOccB.w;
    float zMin = 1e30f;
    for (uint y = y0; y < y1; ++y) {
        for (uint x = x0; x < x1; ++x) zMin = min(zMin, gTex[gOccA.x].Load(int3(int(x), int(y), 0)).x);
    }
    gOut[gOccA.y][id.xy] = (zMin > 0.0f && zMin < 1e30f) ? gOccC.x / zMin : 1e30f;
}
