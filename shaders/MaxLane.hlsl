// ================================================================================================
//  MaxLane.hlsl - THE MAX LANE OF A SCALAR PAGE TENANT (hal/MaxLane.h; TerrainRay.hlsli's cast).
//
//  A fold keeps what its readers must not lose. The height's mips are coverage-weighted MEANS --
//  the right fold for a picture, and no bound at all for a ray that asks "can anything under this
//  cell reach me": a ridge narrower than a coarse texel is averaged away. So beside every resident
//  mip of every page the residency manager keeps a lane at 1/16 of its side: one texel for each
//  16 x 16 block of the page, the MAXIMUM any bilinear read at that mip, or at a finer resident one,
//  can return inside the block -- and beside it the MINIMUM, the same bound from below: a ray under a
//  block's minimum is certainly under the ground there. The maximum of a set of maxima is their maximum, so the lane folds
//  up the mips exactly, with no weighting (an absent child does not vote; a present one votes its
//  maximum), and its fold crosses from a rank's floor to the rank above, which holds the same
//  pyramid level (hal/Tenant.h: rank k's mip 3 is rank k - 1's mip 0).
//
//  THE BILINEAR FOOTPRINT. A read at a point inside block b reads the texels floor(x - 1/2) and the
//  one after, which reach one texel past the block on each side; the block's own maximum is taken
//  over [16 b - 1, 16 b + 16] on each axis, and its children's lanes (each dilated the same way at
//  their own mip) cover the finer reads. A neighbour tile landing later changes a block's dilated
//  rim, which is why the manager rebuilds the tiles beside every tile that lands.
//
//  ONE GROUP A TILE: a page tile is the tenant's 64 KB tile (128 x 128 floats: 8 x 8 blocks; 256 x 128
//  half floats: 16 x 8), a thread a block; one dispatch
//  a pyramid level, a group a job of the turn's list (gJobs from gTurn.x). The job's rows name the tile, its slice and mip, how the page is addressed (a window wraps modulo its page,
//  HIERARCHY 4.1; a cube face clamps to its face) and the four tiles that hold its children's lanes
//  (a slot of ~0 has none: a child not held cannot be read, so its lane is not asked).
//
//  Loads only: a bindless SampleLevel outside the pixel stage returns zero on this GPU (ALGEBRA
//  priors 1). An unmapped tile reads 0 (tiled resources tier 2+), which is a value the reader never
//  meets there: a tile not held is never read.
// ================================================================================================

struct LaneJob {
    uint4 job;        // slice, mip, tile x, tile y
    uint4 page;       // wrap (1 = a window's modulo page, 0 = a cube face), page side at the mip, the
                      // tenant's array SRV, the tile's texels (w = width | height << 16)
    uint4 child[4];   // the four children's lanes: slice, mip, tile x, tile y (slice ~0 = none);
                      // child (i, j) covers the quarter i + 2 j of the tile
};
StructuredBuffer<LaneJob> gJobs : register(t0, space0);
cbuffer LaneTurn : register(b0) {
    uint4 gTurn;      // x = the level's first job
};
Texture2DArray<float> gPages[] : register(t0, space1);
RWTexture2DArray<float2> gLane[8] : register(u0);   // x = the block's maximum, y = its minimum

static const int kBlock = 16;   // MIRRORS hal::MaxLane::kBlock and TerrainRay.hlsli kCastBlock

float2 LaneMip(uint mip, int3 at) {
    // A dynamic index over the eight UAVs is uniform across the dispatch (one job's mip).
    [branch] switch (mip) {
        case 0: return gLane[0][at];
        case 1: return gLane[1][at];
        case 2: return gLane[2][at];
        case 3: return gLane[3][at];
        case 4: return gLane[4][at];
        case 5: return gLane[5][at];
        case 6: return gLane[6][at];
        default: return gLane[7][at];
    }
}
void LaneStore(uint mip, int3 at, float2 v) {
    [branch] switch (mip) {
        case 0: gLane[0][at] = v; break;
        case 1: gLane[1][at] = v; break;
        case 2: gLane[2][at] = v; break;
        case 3: gLane[3][at] = v; break;
        case 4: gLane[4][at] = v; break;
        case 5: gLane[5][at] = v; break;
        case 6: gLane[6][at] = v; break;
        default: gLane[7][at] = v; break;
    }
}
// The lane is half floats; a page may be full floats. A bound stays a bound only rounded OUTWARD: the
// maximum to the nearest half at or above it, the minimum at or below (round-to-nearest could pull a
// maximum under the page's own value by half a half-float step -- a metre at altitude).
float HalfAtOrAbove(float x) {
    uint hb = f32tof16(x);
    if (f16tof32(hb) < x) hb = (x >= 0.0f) ? hb + 1u : hb - 1u;
    return f16tof32(hb);
}
float HalfAtOrBelow(float x) { return -HalfAtOrAbove(-x); }

[numthreads(16, 8, 1)]
void CsLane(uint3 tid : SV_GroupThreadID, uint3 gid : SV_GroupID) {
    const LaneJob J = gJobs[gTurn.x + gid.x];
    const uint4 gJob = J.job, gPage = J.page;
    const uint slice = gJob.x, mip = gJob.y;
    const int2 tile = int2(gPage.w & 0xFFFFu, gPage.w >> 16);   // the tenant's tile in texels
    const int2 nb = tile / kBlock;                                // its blocks
    const int2 bl = int2(tid.xy);                                 // the block within the tile
    if (any(bl >= nb)) return;
    const int2 t0 = int2(gJob.zw) * tile + bl * kBlock;          // its first texel on the page
    const int side = int(gPage.y);
    float2 m = float2(-3.0e38f, 3.0e38f);   // (max, min)
    // The block and its one-texel rim, at the job's mip.
    for (int y = -1; y <= kBlock; ++y) {
        for (int x = -1; x <= kBlock; ++x) {
            int2 p = t0 + int2(x, y);
            p = (gPage.x != 0u) ? (p & (side - 1)) : clamp(p, 0, side - 1);
            const float h = gPages[gPage.z].Load(int4(p, int(slice), int(mip)));
            m = float2(max(m.x, h), min(m.y, h));
        }
    }
    m = float2(HalfAtOrAbove(m.x), HalfAtOrBelow(m.y));
// The children: block b covers child texels [32 b, 32 b + 32), the child blocks 2 b and 2 b + 1,
    // in the child tile of its quarter.
    const int2 hb = nb / 2;
    const int2 q = int2(bl.x >= hb.x ? 1 : 0, bl.y >= hb.y ? 1 : 0);
    const uint4 c = J.child[q.x + 2 * q.y];
    if (c.x != 0xFFFFFFFFu) {
        const int2 cb = int2(c.zw) * nb + (bl - q * hb) * 2;
        [unroll] for (int j = 0; j < 2; ++j) {
            [unroll] for (int i = 0; i < 2; ++i) {
                const float2 k = LaneMip(c.y, int3(cb + int2(i, j), int(c.x)));
                m = float2(max(m.x, k.x), min(m.y, k.y));
            }
        }
    }
    LaneStore(mip, int3(int2(gJob.zw) * nb + bl, int(slice)), m);
}
