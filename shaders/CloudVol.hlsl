// ================================================================================================
//  CloudVol.hlsl - M6c: the sky becomes a VOLUME grade bank.
//
//  The cloud density volume (1024 x 512 x 32 R16F over lon x lat x altitude 0..12.8 km) is the
//  first tenant of TileAtlas3D: tiles are resident only where GFS says clouds exist, and a
//  NULL tile IS clear air -- the same hardware zero that means "flat tide plane" in the eta
//  bank means "nothing to scatter" here. Weather says WHERE (isobaric cloud fraction, 10
//  levels); a little value noise says what the texture looks like inside the truth.
//
//  List-driven like every atlas kernel: Dispatch(tileW/8, tileH/8, listCount * tileD/8) would
//  waste the z groups; instead z spans (listCount) and each thread loops the tile's depth.
// ================================================================================================

cbuffer CloudCb : register(b0) {
    uint  gVolNx, gVolNy, gVolNz, gListCount;
    uint  gTilesX, gTilesY, gTileW, gTileH;
    uint  gTileD, gSrcNx, gSrcNy, gSrcNz;
    float4 gAltA;      // source level altitudes 0-3 (m)
    float4 gAltB;      // 4-7
    float4 gAltC;      // 8-9, z = volume top altitude (m), w = density gamma
};

StructuredBuffer<uint> gTileList : register(t0);
Texture3D<float> gSrc : register(t1);        // isobaric cloud fraction, %, w = level index
RWTexture3D<float> gVol : register(u0);

SamplerState sClamp : register(s0);

float Hash(float3 p) {
    return frac(sin(dot(p, float3(12.9898f, 78.233f, 45.164f))) * 43758.5453f);
}

float ValueNoise(float3 p) {
    const float3 i = floor(p);
    const float3 f = frac(p);
    const float3 u = f * f * (3.0f - 2.0f * f);
    float v = 0.0f;
    [unroll] for (uint k = 0; k < 8; ++k) {
        const float3 c = float3(k & 1, (k >> 1) & 1, (k >> 2) & 1);
        const float w = lerp(1.0f - u.x, u.x, c.x) * lerp(1.0f - u.y, u.y, c.y) *
                        lerp(1.0f - u.z, u.z, c.z);
        v += w * Hash(i + c);
    }
    return v;
}

[numthreads(8, 8, 1)]
void CsCloudBuild(uint3 id : SV_DispatchThreadID) {
    if (id.x >= gTileW || id.y >= gTileH || id.z >= gListCount) return;
    const uint t = gTileList[id.z];
    const uint tx = t % gTilesX;
    const uint ty = (t / gTilesX) % gTilesY;
    const uint tz = t / (gTilesX * gTilesY);

    const float alts[10] = {gAltA.x, gAltA.y, gAltA.z, gAltA.w, gAltB.x,
                            gAltB.y, gAltB.z, gAltB.w, gAltC.x, gAltC.y};

    for (uint dz = 0; dz < gTileD; ++dz) {
        const uint3 texel = uint3(tx * gTileW + id.x, ty * gTileH + id.y, tz * gTileD + dz);
        if (texel.x >= gVolNx || texel.y >= gVolNy || texel.z >= gVolNz) continue;

        const float altM = (texel.z + 0.5f) / gVolNz * gAltC.z;
        const float2 suvXY = float2((texel.x + 0.5f) / gVolNx, (texel.y + 0.5f) / gVolNy);

        // A 55 km GFS cell is not a razor sheet: each level contributes a Gaussian slab (700 m
        // deep low, 1.4 km high) so the sheets span several voxels and a planet-scale march
        // cannot step over them (the first cut point-sampled and rendered SPECKLE).
        float frac01 = 0.0f;
        [unroll] for (uint L = 0; L < 10; ++L) {
            const float sigma = lerp(700.0f, 1400.0f, saturate(alts[L] / 11000.0f));
            const float w = exp(-0.5f * pow((altM - alts[L]) / sigma, 2.0f));
            if (w < 0.02f) continue;
            const float c =
                gSrc.SampleLevel(sClamp, float3(suvXY, (L + 0.5f) / gSrcNz), 0) / 100.0f;
            frac01 = max(frac01, saturate(c) * w);
        }

        // Weather is the envelope; noise is the texture inside it (soft, not a threshold --
        // hard carving at 20 km texels reads as polka dots from orbit).
        const float3 np = float3(texel.x * 0.13f, texel.y * 0.13f, texel.z * 0.09f);
        const float n = 0.65f * ValueNoise(np) + 0.35f * ValueNoise(np * 2.6f);
        gVol[texel] = pow(frac01, gAltC.w) * (0.70f + 0.50f * n);
    }
}
