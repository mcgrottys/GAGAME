// ================================================================================================
//  WaterBank.hlsl - M7: THE WAVE VERTEX BANK -- the water's geometry as ONE tiled resource.
//
//  A camera-anchored window of mip rings (finest tiles at the camera, coarser rings outward)
//  over TWO RGBA16F banks:
//      disp  = (dx, dy, dz, foam)          -- the wave vertex, metres about the local level
//      param = (levelNavd, sigma2, u, v)   -- the mean surface + shed variance + current
//
//  Each resident tile is RECOMPUTED every frame by this kernel from the state the weather
//  manager federates: the FFT cascades (folded by the tile's own texel footprint -- the M6t
//  grade shedding, so a coarse ring carries variance where a fine ring carries geometry),
//  the SWE window banks where resident (eta + solved currents), and per-tile corner params
//  the CPU sampled from the atlas stacks (tide level via the constituent rotors, local Hs
//  from the global wave grid, bed for the dry guard). Land tiles are NULL -- never mapped,
//  never dispatched: no water here costs nothing, which is the atlas thesis.
//
//  In the state-diagram reading (the user's architecture): this kernel is an EDGE -- the
//  geometric product carrying the weather-manager node's state into the renderer node. The
//  accepting state samples the bank; it never asks who computed what.
// ================================================================================================

cbuffer BankCb : register(b0) {
    float4 gOrg;        // xy = window origin (world m, snapped), z = base texel m, w = time s
    float4 gPatch;      // xyz = cascade patch sizes m, w = height exaggeration
    float4 gBandK;      // xyz = representative wavenumber per cascade, w = list count
    float4 gSwe;        // xy = swe world x0/z0, zw = 1/sizeX, 1/sizeZ (0 = solver absent)
    float4 gSweDims;    // xy = swe grid nx/ny, zw = 1 / eta-atlas padded dims
    float4 gMisc;       // x = tile texels, y = seaLevel fallback, zw unused
    uint4  gSlotsA;     // cascade disp SRV slots x3, swe eta SRV slot
    uint4  gSlotsB;     // swe uv SRV slot, disp/param/detail bank UAV slots
};

struct BankTile {
    float2 orgXZ;       // this tile's window-frame origin, world m
    float texelM;       // this tile's texel size (mip ladder)
    uint dstX;          // atlas texel origin of the tile slot
    uint dstY;
    float lvl00, lvl10, lvl01, lvl11;   // tide level at corners (atlas stack, CPU rotors)
    float bed00, bed10, bed01, bed11;   // bed at corners (the one height stack)
    float hsScale;      // local Hs / reference Hs (the global wave grid modulates the sea)
    float pad0, pad1, pad2;
};
StructuredBuffer<BankTile> gTiles : register(t0);

// Bindless over the shared heap (the renderer's doctrine, compute-side): textures by SLOT,
// never by name -- the residency machinery can move data under this kernel freely. All
// reads are manual-bilinear LOADS: the static-sampler SampleLevel path silently returns
// zero on this driver for bindless arrays outside the pixel stage (stage-bisected in M7
// bring-up; the mesh stage showed the same).
Texture2D gT[] : register(t0, space1);
RWTexture2D<float4> gU[] : register(u0, space2);

float4 LoadBilinearWrap(uint slot, float2 uv, float dim) {
    const float2 tf = uv * dim - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    float4 acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = (int2(t0) + int2(k & 1, k >> 1) + int(dim)) % int(dim);
        acc += ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y) *
               gT[slot][tc];
    }
    return acc;
}

float4 LoadBilinearClamp(uint slot, float2 texel, float2 dims) {
    const float2 tf = texel - 0.5f;
    const float2 t0 = floor(tf);
    const float2 fr = tf - t0;
    float4 acc = 0.0f;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = clamp(int2(t0) + int2(k & 1, k >> 1), int2(0, 0),
                              int2(dims) - int2(1, 1));
        acc += ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y) *
               gT[slot][tc];
    }
    return acc;
}

[numthreads(16, 16, 1)]
void CsBankFill(uint3 id : SV_DispatchThreadID) {
    const uint texels = (uint)gMisc.x;
    if (id.x >= texels || id.y >= texels) return;
    const BankTile t = gTiles[id.z];
    const float2 f = (float2(id.xy) + 0.5) / gMisc.x;
    const float2 xz = t.orgXZ + float2(id.xy) * t.texelM;

    // Corner-lerped spatial context (the CPU sampled the atlas stacks at the corners; a tile
    // spans well under the tide's or the wave grid's own resolution, so bilinear is honest).
    const float level = lerp(lerp(t.lvl00, t.lvl10, f.x), lerp(t.lvl01, t.lvl11, f.x), f.y);
    const float bed = lerp(lerp(t.bed00, t.bed10, f.x), lerp(t.bed01, t.bed11, f.x), f.y);

    // The SWE refinement where the solver is resident: dEta on the level, solved currents.
    float dEta = 0.0f;
    float2 cur = 0.0f;
    if (gSwe.z > 0.0f) {
        const float2 uv = (xz - gSwe.xy) * gSwe.zw;
        if (all(uv > 0.001f) && all(uv < 0.999f)) {
            const float2 texel = float2(uv.x * gSweDims.x, (1.0f - uv.y) * gSweDims.y);
            dEta = LoadBilinearClamp(gSlotsA.w, texel, gSweDims.xy).x;
            const float4 s = LoadBilinearClamp(
                gSlotsB.x, float2(uv.x * gSweDims.x, (1.0f - uv.y) * gSweDims.y),
                gSweDims.xy);
            if (s.w > 0.5f) cur = s.xy;
        }
    }
    const float lvl = level + dEta;
    const float depth = lvl - bed;
    const float dry = smoothstep(0.05f, 0.65f, depth);

    // THE FOLD, per ring (M6t): a band is geometry while THIS tile's texels resolve its
    // phase; past its Nyquist it sheds to sigma^2. Coarse rings carry the same energy as
    // statistics that fine rings carry as vertexes -- no popping between rings possible.
    float3 d = 0.0f;
    float sig2 = 0.0015f;
    float foam = 0.0f;
    [unroll] for (uint c = 0; c < 3; ++c) {
        const float lam = 6.2831853f / gBandK[c];
        const float w = 1.0f - smoothstep(lam * 0.12f, lam * 0.5f, t.texelM);
        const float2 cuv = frac(xz / gPatch[c]);
        const float4 s = LoadBilinearWrap(gSlotsA[c], cuv, 256.0f);
        d += s.xyz * (w * t.hsScale);
        foam += s.w * w * (c == 2 ? 1.0f : 0.4f);
        // shed variance rides the local sea state too (amp^2)
        sig2 += (1.0f - w) * t.hsScale * t.hsScale *
                (c == 0 ? 0.0004f : (c == 1 ? 0.0018f : 0.0060f));
    }
    d *= dry * gPatch.w;

    // Depth-limited breaking (the Sea.hlsl clamp, bank-side): the excess becomes foam.
    const float hmax = 0.55f * max(depth, 0.05f);
    if (abs(d.y) > hmax) {
        foam += saturate((abs(d.y) - hmax) / max(hmax, 0.2f));
        d.y *= hmax / abs(d.y);
        d.xz *= 0.85f;
    }

    const uint2 dst = uint2(t.dstX + id.x, t.dstY + id.y);
    gU[gSlotsB.y][dst] = float4(d, saturate(foam * dry));
    gU[gSlotsB.z][dst] = float4(lvl, sig2, cur.x, cur.y);
    // The DETAIL plane: what the PS needs to recover sub-ring sparkle -- the tile's local
    // sea-state scale (cascade derivs are unit-sea) and the dry guard (no sparkle on the
    // flats). Churn memory joins this fiber next.
    gU[gSlotsB.w][dst] = float4(t.hsScale, dry, 0.0f, 0.0f);
}
