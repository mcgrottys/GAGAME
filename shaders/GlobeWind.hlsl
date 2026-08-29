// ================================================================================================
//  GlobeWind.hlsl - M6d: the M3 velocity-gradient decomposition goes PLANETARY.
//
//  Source: the live GFS 10 m wind vector (dense, small). Output: a sparse Mv2 bank over the
//  globe -- (divergence, u, v, curl) per texel, the same grade layout the gulf view has used
//  since M3 -- where tiles are resident only where the atmosphere is DOING something (storms,
//  jets). A NULL tile is calm air: it contributes algebraic zero to every product, which is
//  the whole point of grade-banked sparsity. Storage scale: div and curl are held x1e4 (raw
//  synoptic values straddle fp16's subnormal floor -- the M3 lesson, third appearance).
//
//  Derivatives use the spherical metric: dx = R cos(lat) dlon (guarded at the poles), dy = R dlat.
// ================================================================================================

cbuffer WindCb : register(b0) {
    uint  gNx, gNy, gListCount, gTilesX;
    uint  gTileW, gTileH, gPad0, gPad1;
    float gLat1, gDLatDeg, gRadius, gScale;   // row-0 latitude (deg), step (deg, +down), R, 1e4
};

StructuredBuffer<uint> gTileList : register(t0);
Texture2D<float2> gWindSrc : register(t1);    // u east, v north (m/s)
RWTexture2D<float4> gBank : register(u0);

[numthreads(16, 16, 1)]
void CsWindGrad(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gTileW || id.y >= gTileH) return;
    const uint t = gTileList[id.z];
    const int2 texel = int2((t % gTilesX) * gTileW + id.x, (t / gTilesX) * gTileH + id.y);
    if (texel.x >= (int)gNx || texel.y >= (int)gNy) return;

    const float latDeg = gLat1 - (texel.y + 0.5f) * gDLatDeg;
    const float cosLat = max(cos(radians(latDeg)), 0.05f);
    const float dxM = gRadius * cosLat * radians(gDLatDeg);   // dlon == dlat on this grid
    const float dyM = gRadius * radians(gDLatDeg);

    // Neighbours: x wraps (it is a globe); y clamps at the poles.
    const int xm = (texel.x + gNx - 1) % gNx;
    const int xp = (texel.x + 1) % gNx;
    const int ym = max(texel.y - 1, 0);
    const int yp = min(texel.y + 1, (int)gNy - 1);
    const float2 c = gWindSrc[texel];
    const float2 e = gWindSrc[int2(xp, texel.y)];
    const float2 w = gWindSrc[int2(xm, texel.y)];
    const float2 n = gWindSrc[int2(texel.x, ym)];
    const float2 s = gWindSrc[int2(texel.x, yp)];

    // Row index grows SOUTH: d/dy(north) = -(south - north)/(2 dy).
    const float dudx = (e.x - w.x) / (2.0f * dxM);
    const float dvdx = (e.y - w.y) / (2.0f * dxM);
    const float dudy = (n.x - s.x) / (2.0f * dyM);
    const float dvdy = (n.y - s.y) / (2.0f * dyM);

    const float div = dudx + dvdy;
    const float curl = dvdx - dudy;    // + = counter-clockwise = NH cyclonic

    gBank[texel] = float4(div * gScale, c.x, c.y, curl * gScale);
}
