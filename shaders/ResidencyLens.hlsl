// ================================================================================================
//  ResidencyLens.hlsl - the residency lens's pixel shader (--lens residency | residency.height |
//  residency.landsea), compiled ONLY when the lens is asked for (GlobeLayer::BuildPso and
//  BuildMeshPso). It reads Globe.hlsl's rows and helpers, so it includes the whole file; Globe.hlsl
//  itself does not know the lens exists.
//
//  WHY A FILE OF ITS OWN. MEASURED (2026-09-28, the six settled stills on 35a9eb7, every instrument
//  off): with the lens inside Globe.hlsl -- as a branch of PsMain never taken, and again as an entry
//  point of its own beside PsMain -- the helm still moved 53 to 60 pixels of its horizon strip
//  (rows 430-431, max |d| 25) against three bit-identical renders of the unmodified file, and two
//  renders of the edited file differed from each other there by 25 pixels. The compiled output of
//  a file's other entry points is not independent of what else the file holds, so an instrument
//  that must change no picture while it is off lives outside the file the picture is built from.
// ================================================================================================
#include "Globe.hlsl"

// ---- THE RESIDENCY LENS (--lens residency | residency.height | residency.landsea; HIERARCHY
// 4.13's first lens, "the residency of the scene's earth"). Not the picture: what the sampler is
// ALLOWED to read at this pixel. The page that answers -- the cube (violet), the z14 window
// (green), the z17 detail (red) -- is chosen exactly as ComposedColorPages, ComposedHeightPages and
// CsMaskSample choose it, and that page's residency floor (PageHave: the gather + max of the
// byte, the min-LOD its sample is clamped by) is the brightness: mip 0 full, each coarser mip a
// ninth darker. Magenta: the answering page holds nothing at all (the byte is past its chain).
// Dark grey: no page of the survey has an opinion here (landsea only). A black line outlines the
// answering tile at its floor mip where the tile spans 5 to 500 pixels (hard-edged, so a pixel is
// either a key colour or the outline). The swatches at the bottom left are the key, drawn by this
// same function -- rows cube / z14 / z17 upward, columns mip 0..7, then magenta and grey -- so a
// PNG is read back against its own colours whatever the exposure and the tonemap did to them.
// The cube is violet, not blue, so the sky can never be read as the cube.
// PHASE A1: the colour and the mask read the eye's windows: rank 1's (76 m) is cyan, a rank 2
// window is painted as the z14 page was (green) and a rank 3 window as the z17 (red); rank 4
// (15 cm) is yellow and rank 5 (1.9 cm) white, rows five and six of the key. The height's pages
// read the same ranks (PHASE B3: the Mercator pages are gone).
static const float3 kResLensHue[6] = {float3(0.60f, 0.25f, 1.00f), float3(0.15f, 1.00f, 0.30f),
                                      float3(1.00f, 0.30f, 0.10f), float3(0.10f, 0.75f, 1.00f),
                                      float3(1.00f, 0.92f, 0.15f), float3(1.00f, 1.00f, 1.00f)};
#define GA_RES_LENS_LAST 5
#define GA_RES_LENS_ROWS 6.0f
int ResLensRankPage(uint k) { return (k == 0u) ? 3 : (k == 1u) ? 1 : (k == 2u) ? 2 : int(k) + 1; }
float3 ResLensColor(int page, float have, float mips) {
    if (have > mips - 0.5f) return float3(1.0f, 0.0f, 1.0f);
    return kResLensHue[clamp(page, 0, GA_RES_LENS_LAST)] * (1.0f - floor(have + 0.5f) / 9.0f);
}
// The texel grid a cube view addresses: the D3D cube face's (u, v) of a direction.
float2 ResLensCubeUv(float3 d) {
    const float3 a = abs(d);
    float2 st;
    float ma;
    if (a.x >= a.y && a.x >= a.z) {
        ma = a.x;
        st = float2(d.x > 0.0f ? -d.z : d.z, -d.y);
    } else if (a.y >= a.z) {
        ma = a.y;
        st = float2(d.x, d.y > 0.0f ? d.z : -d.z);
    } else {
        ma = a.z;
        st = float2(d.z > 0.0f ? d.x : -d.x, -d.y);
    }
    return st / ma * 0.5f + 0.5f;
}
float3 ResidencyLens(float3 dir, float3 p CS_WC_PARAM, int tenant, float2 px) {
    // The key, bottom left: 16 px cells, 14 px swatches.
    const float2 kp = float2(px.x - 6.0f, (gViewport.y - 6.0f) - px.y);
    if (all(kp >= 0.0f) && kp.x < 9.0f * 16.0f && kp.y < GA_RES_LENS_ROWS * 16.0f) {
        const int col = int(kp.x / 16.0f), row = int(kp.y / 16.0f);
        if (frac(kp.x / 16.0f) >= 0.875f || frac(kp.y / 16.0f) >= 0.875f) return float3(0, 0, 0);
        if (col < 8) return ResLensColor(row, float(col), 99.0f);
        return (row == 0) ? float3(1.0f, 0.0f, 1.0f) : float3(0.12f, 0.12f, 0.12f);
    }
    // Every page's uv and its screen derivative, taken here under uniform control flow: the
    // tile outline needs the derivative of whichever page answers.
    const float2 cuv = ResLensCubeUv(dir);
    const float2 fwC = fwidth(cuv);
    // The chain's derivative, rank 1's; a rank's window is an eighth of its parent's, so rank
    // k + 1's derivative is 8^k of it.
    const float2 fwB = fwidth(wc.uv0);
    int page = 0;
    float have = 99.0f, mips = 8.0f;
    float2 uv = cuv, fw = fwC, tiles = float2(128.0f, 128.0f);   // tiles a side at mip 0
    if (tenant == 0) {
        // earth.color: ComposedColorPages' ladder.
        if (gCsU5.x == 0xFFFFFFFFu || gCsF.x < 0.5f) return float3(0.3f, 0.3f, 0.3f);
        const float3 g0 = CsGroundM();
        have = CsHaveCubeArr(gCsU.y, dir);
        float ground = PageGroundM(g0.x, have);
        // The eye's windows: the pixel's chain, read exactly as ComposedColorPages reads it.
        [unroll] for (uint k = 0; k < GA_BLOCK_RANKS; ++k) {
            if (k >= wc.n) break;
            const uint sl = WalkSlice(wc, k);
            const float2 buv = WalkUv(wc, k);
            if (gTexArr[gCsU5.x].CalculateLevelOfDetail(sAnisoWrap, buv) > kCsWindowFloor) break;
            const float hB = CsHaveWindow(gCsU5.y, buv, sl);
            if (hB > kCsWindowFloor) continue;
            const float gB = PageGroundM(CsBlockGround(k), hB);
            if (PageWins(gB, ground)) {
                page = ResLensRankPage(k);
                have = hB;
                ground = gB;
                uv = buv;
                fw = fwB * exp2(3.0f * float(k));
            }
        }
    } else if (tenant == 1) {
        // earth.height: ComposedHeightPages' choice; its tiles are 256 x 128 texels, 7 mips.
        if (gCsU6.x == 0xFFFFFFFFu || gCsF.z < 0.5f) return float3(0.3f, 0.3f, 0.3f);
        mips = 7.0f;
        tiles = float2(64.0f, 128.0f);
        have = CsHaveCubeArr(gCsU2.y, dir);
        const float3 g0 = CsGroundM();
        float ground = PageGroundM(g0.x, have);
        // PHASE B2: the height's windows, the pixel's chain, as ComposedHeightChain holds them (the
        // lens shows what is held to the floor, the finest rank answering).
        [unroll] for (uint k = 0; k < GA_BLOCK_RANKS; ++k) {
            if (k >= wc.n) break;
            const uint sl = WalkSlice(wc, k);
            const float2 buv = WalkUv(wc, k);
            const float hB = CsHaveWindow(gCsU6.y, buv, sl);
            if (hB > kCsHeightWindowFloor) continue;
            const float gB = PageGroundM(CsBlockGround(k), hB);
            if (PageWins(gB, ground)) {
                page = ResLensRankPage(k);
                have = hB;
                ground = gB;
                uv = buv;
                fw = fwB * exp2(3.0f * float(k));
            }
        }
    } else {
        // gis.landsea: CsMaskSample's order -- the finest page with an OPINION answers.
        if (gCsU3.x == 0xFFFFFFFFu) return float3(0.3f, 0.3f, 0.3f);
        page = -1;
        // The eye's windows: the pixel's chain in CsMaskSample's order, the finest rank first.
        [unroll] for (int k = GA_BLOCK_RANKS - 1; k >= 0; --k) {
            if (page >= 0 || uint(k) >= wc.n) continue;
            const uint sl = WalkSlice(wc, uint(k));
            const float2 buv = WalkUv(wc, uint(k));
            const float hB = CsHaveWindow(gCsU3.y, buv, sl);
            if (hB <= kCsWindowFloor &&
                PageSampleLevel(gTexArr[gCsU3.x], sLinearWrap, buv, sl, 0.0f, hB).a > 0.001f) {
                page = ResLensRankPage(uint(k));
                have = hB;
                uv = buv;
                fw = fwB * exp2(3.0f * float(k));
            }
        }
        if (page < 0 && gCsU3.z != 0xFFFFFFFFu) {
            const float hC = CsHaveCubeArr(gCsU3.w, dir);
            if (hC <= 7.5f &&
                PageSampleLevelCube(gTexCubeArr[gCsU3.z], sLinearClamp, dir, 0.0f, hC).a > 0.001f) {
                page = 0;
                have = hC;
            }
        }
        if (page < 0) return float3(0.12f, 0.12f, 0.12f);
    }
    float3 c = ResLensColor(page, have, mips);
    if (have <= mips - 0.5f) {
        // The answering tile's outline: its edges at the floor mip, drawn only where a tile spans
        // 5 to 500 pixels -- a vanishing derivative (the storm-displaced water under the eye) would
        // otherwise paint a whole band as "on the edge".
        const float2 n = tiles / exp2(floor(have + 0.5f));
        const float2 fq = fw * n;   // tiles per pixel, per axis
        const float fqMax = max(fq.x, fq.y);
        if (fqMax > 0.002f && fqMax < 0.2f) {
            const float2 e = abs(frac(uv * n - 0.5f) - 0.5f) / max(fq, 1e-6f);   // pixels to an edge
            if (min(e.x, e.y) < 0.5f) c = float3(0.0f, 0.0f, 0.0f);
        }
    }
    return c;
}


// THE HEIGHT READ (--lens residency.height, --dump-hdr's alpha): the rank and the mip ComposedHeightChain
// actually SAMPLES at the pixel's own lod (PsMain's hp: ComposedHeightLod), max(want, have) of the
// rank that answers -- the read, not the hold. alpha = 10 x rank (0 the cube) + that mip.
float HeightReadLens(float3 dir, float lod CS_WC_PARAM) {
    // PHASE B2: ComposedHeightChain's read, said: alpha = 10 x (rank) + the mip it samples.
    if (gCsU6.x == 0xFFFFFFFFu || gCsF.z < 0.5f) return -1.0f;
    const float haveC = CsHaveCubeArr(gCsU2.y, dir);
    float ground = PageGroundM(CsGroundM().x, haveC), r = max(clamp(lod, 0.0f, gCsG.x), haveC);
    [unroll] for (uint k = 0; k < GA_BLOCK_RANKS; ++k) {
        if (k >= wc.n) break;
        const float want = lod + 3.0f * float(k + 1u);
        if (want > kCsHeightWindowFloor) break;
        const float hB = CsHaveWindow(gCsU6.y, WalkUv(wc, k), WalkSlice(wc, k));
        if (hB > kCsHeightWindowFloor) continue;
        const float gB = PageGroundM(CsBlockGround(k), hB);
        if (PageWins(gB, ground)) {
            r = 10.0f * float(k + 1u) + max(max(want, 0.0f), hB);
            ground = gB;
        }
    }
    return r;
}

// THE CLOUD MARCH'S FIRST SAMPLE (--lens cloudalt; --dump-hdr holds the numbers): r = its altitude
// by CloudMarch's small numbers, g = by the old sphere-centred form (length(eye + rd t) - R), both
// at the jittered t; b = old less new at the step's un-jittered middle (the float error alone);
// a = the march's opacity 1 - T (where there is cloud). Zero where the ray misses the shell.
float4 CloudAltLens(float3 rel, float2 pix) {
    const float3 ro = sLvlCamAbs;
    const float3 rd = normalize(rel);
    const float R = gGlo.x, H = gCloudA.y;
    const float3 c = float3(ro.x, sLvlEyeY, ro.z);
    const float a0 = (dot(c, c) + 2.0f * R * c.y) / (length(ro) + R);
    const float r0 = R + a0;
    const float mu = dot(rd, normalize(ro));
    const float b = r0 * mu, cc = (a0 - H) * (2.0f * R + a0 + H), disc = b * b - cc;
    if (disc <= 0.0f) return 0.0f;
    const float t0 = max(-b - sqrt(disc), 0.0f);
    const float dt = (length(rel) - t0) / 14.0f;
    const float jit = frac(sin(dot(pix, float2(12.9898f, 78.233f))) * 43758.5f);
    float3 scat;
    const float T = (gTexIdx.w != 0xFFFFFFFFu) ? CloudMarch(rel, pix, scat) : 1.0f;
    float2 tt = float2(t0 + jit * dt, t0 + 0.5f * dt), an, ao;
    [unroll] for (int k = 0; k < 2; ++k) {
        const float t = tt[k];
        const float q = t * (2.0f * r0 * mu + t);
        an[k] = a0 + q / (sqrt(r0 * r0 + q) + r0);
        ao[k] = length(ro + rd * t) - R;
    }
    return float4(an.x, ao.x, ao.y - an.y, 1.0f - T);
}

// ---- Phase A0: THE BLOCK LENS (--lens blocks; plan_eye_windows.md). Per pixel, per level: which
// rank and which block answers the colour, chosen exactly as ComposedColorPages chooses it. The hue
// is the RANK -- violet the cube (rank 0), cyan rank 1 (76 m), green rank 2 (9.6 m: the z14 page
// stands here when there are no blocks), red rank 3 (1.2 m: the z17 page), yellow rank 4, white
// rank 5 -- and the brightness the BLOCK (its slice: two blocks of one rank differ); a black line
// is a block's edge. A pixel of any level but the camera's own is hatched, one stripe in (lvl + 1),
// so the levels are told apart. The key is at the bottom right: ranks 0..5 upward. --dump-hdr holds
// the numbers raw: rgb the colour above, a = slice + 128 x level.
// THE TWO-WORLDS PROBE (--ground-probe lat,lon), the bottom-left strip, row H - 8: six pixels a
// slot from x = 8, slot s at 8 + 6 s. Each reads the ground point G as slot s's own pixels address
// it TODAY -- the camera's slot by its point (G less its eye, from the CPU's doubles: what geo is),
// every other slot by its direction (CsPointOfDir, PsMain's own rule for them) -- with the colour's
// own ladder and the height's. HDR, every number exact in half:
//   +0  answering colour slice, its residency floor have, its rank, the chain's length (blocks)
//   +1  the texel it holds at mip ceil(have): x / 1024, x mod 1024, y / 1024, y mod 1024
//   +2  that texel, Loaded (the bytes: UNORM8 / 255, exact in half)
//   +3  ComposedHeight at the floor lod, as hi + lo halves; the land mask; 1 = the height pages on
//   +4  the colour address's fraction: frac(u 16384), frac(v 16384) at mip 0 of the answering
//       slice; the slots filled; the slot
//   +5  1 where the slot is filled, its slot, 0, 1
//   +6..+10 (PHASE A2) rank 1..5's texel at G at its floor (mip 3), Loaded; -1 where the chain does
//       not hold G in that rank, -2 where its floor is not resident there.
//   +11..+15 (PHASE B0) the HEIGHT's rank 1..5 at G at the height window's floor (gProbeX.z), Loaded:
//       (the R16F value, the residency floor there, the slice, 1); -1 / -2 as above, -3 where the
//       height has no windows (gProbeX.z = ~0).
//   +16..+20 (PHASE B0) the EXPOSURE's, the same, at its floor (gProbeX.w; array gProbeX.x, map .y).
//       Twenty-one pixels a slot (PHASE B3: B0's +21, the Mercator pages, is deleted with them).
static const float3 kBlkLensHue[6] = {float3(0.60f, 0.25f, 1.00f), float3(0.10f, 0.75f, 1.00f),
                                      float3(0.15f, 1.00f, 0.30f), float3(1.00f, 0.30f, 0.10f),
                                      float3(1.00f, 0.92f, 0.15f), float3(1.00f, 1.00f, 1.00f)};
// ComposedColorPages' choice, said: the rank (0 the cube, k + 1 the eye's window of rank k + 1),
// its slice, its address and its residency floor.
void BlockLensAnswer(float3 dir, float3 p CS_WC_PARAM, out int rank, out uint slice, out float2 uv,
                     out float have, bool footprint = true) {
    float2 fuv;
    slice = HpCubeFace(dir, fuv);
    uv = fuv;
    rank = 0;
    have = CsHaveCubeArr(gCsU.y, dir);
    if (gCsU5.x == 0xFFFFFFFFu || gCsF.x < 0.5f) return;
    const float3 g0 = CsGroundM();
    float ground = PageGroundM(g0.x, have);
    [unroll] for (uint k = 0; k < GA_BLOCK_RANKS; ++k) {
        if (k >= wc.n) break;
        const uint sl = WalkSlice(wc, k);
        const float2 buv = WalkUv(wc, k);
        // (The probe has no footprint of its own: it reads the finest resident.)
        if (footprint && gTexArr[gCsU5.x].CalculateLevelOfDetail(sAnisoWrap, buv) > kCsWindowFloor) break;
        const float hB = CsHaveWindow(gCsU5.y, buv, sl);
        if (hB > kCsWindowFloor) continue;
        const float gB = PageGroundM(CsBlockGround(k), hB);
        if (PageWins(gB, ground)) {
            rank = int(k) + 1; slice = sl; uv = buv; have = hB; ground = gB;
        }
    }
}
// One probe pixel: slot s reads G its own way. PHASE A2: the colour and the mask at G less slot s's
// own eye (gProbeP, the CPU's doubles) through slot s's own windows; the height (still the
// Mercator pages', Phase B) as PsMain reads it, by the camera's eye.
// PHASE B0: one rank's texel of a tenant at G at that tenant's window floor, Loaded: (value, its
// residency floor there, the slice, 1); -1 not in the chain, -2 the floor not resident, -3 the
// tenant has no windows. Two slots holding one rank at G hold one global tile: the bytes agree.
float4 ProbeRankTexel(WalkChain wc, uint k, uint arr, uint res, uint floorMip) {
    if (floorMip == 0xFFFFFFFFu || arr == 0xFFFFFFFFu) return float4(-3.0f, -3.0f, -3.0f, -3.0f);
    if (k >= wc.n) return float4(-1.0f, -1.0f, -1.0f, -1.0f);
    const uint sl = WalkSlice(wc, k);
    const float2 buv = WalkUv(wc, k);
    const float have = CsHaveWindow(res, buv, sl);
    if (have > float(floorMip)) return float4(-2.0f, have, float(sl), -2.0f);
    const int2 t = int2(floor(frac(buv) * (kPageDim / exp2(float(floorMip)))));
    return float4(gTexArr[arr].Load(int4(t, int(sl), int(floorMip))).x, have, float(sl), 1.0f);
}
float4 BlockProbe(uint s, uint j) {
    if (float(s) + 0.5f > gProbeG.w) return float4(0.0f, float(s), 0.0f, -1.0f);
    const float3 dirG = normalize(gProbeG.xyz);
    const float3 pW = gProbeP[s].xyz;
    const WalkChain wc = CsChain(pW, s);
    const float nChain = float(wc.n);
    // +11..+20 (PHASE B0): the height's and the exposure's rank k + 1 at G, at their floors.
    if (j >= 11u && j <= 15u) return ProbeRankTexel(wc, j - 11u, gCsU6.x, gCsU6.y, gProbeX.z);
    if (j >= 16u && j <= 20u) return ProbeRankTexel(wc, j - 16u, gProbeX.x, gProbeX.y, gProbeX.w);
    // +6..+10: rank k + 1's texel at G at its floor (mip 3), Loaded, where the chain holds G in that
    // rank and the floor is resident there; (-1) elsewhere. Two slots holding one rank at G hold
    // one global tile in their own slices, at the same slot (modulo 16384): the bytes must agree.
    if (j >= 6u) {
        const uint k = j - 6u;
        if (k >= wc.n || gCsU5.x == 0xFFFFFFFFu) return float4(-1.0f, -1.0f, -1.0f, -1.0f);
        const uint sl = WalkSlice(wc, k);
        const float2 buv = WalkUv(wc, k);
        if (CsHaveWindow(gCsU5.y, buv, sl) > kCsWindowFloor) return float4(-2.0f, -2.0f, -2.0f, -2.0f);
        const int2 t3 = int2(floor(frac(buv) * (kPageDim / 8.0f)));
        return gTexArr[gCsU5.x].Load(int4(t3, int(sl), 3));
    }
    int rank;
    uint slice;
    float2 uv;
    float have;
    BlockLensAnswer(dirG, pW CS_WC, rank, slice, uv, have, false);
    const float m = min(ceil(have), 7.0f);
    const float dim = kPageDim / exp2(m);
    // A window's address is modulo 16384 (frac); the cube's uv is its own.
    const int2 tx = int2(clamp(floor(frac(uv) * dim), 0.0f, dim - 1.0f));
    if (j == 0u) return float4(float(slice), have, float(rank), nChain);
    if (j == 1u) return float4(float(tx.x >> 10), float(tx.x & 1023), float(tx.y >> 10), float(tx.y & 1023));
    if (j == 2u && gCsU5.x == 0xFFFFFFFFu) return float4(-1.0f, -1.0f, -1.0f, -1.0f);
    if (j == 2u) return gTexArr[gCsU5.x].Load(int4(tx, int(slice), int(m)));
    if (j == 3u) {
        const float h = ComposedHeightAt(dirG, pW, kCsHeightLodFloor, s);   // PHASE B2: its own windows
        const float hi = f16tof32(f32tof16(h));
        return float4(hi, h - hi, ComposedLandMask(dirG, pW CS_WC), gCsU6.x != 0xFFFFFFFFu ? 1.0f : 0.0f);
    }
    if (j == 4u) return float4(frac(uv * kPageDim), gProbeG.w, float(s));
    return float4(gProbeP[s].w, float(s), 0.0f, 1.0f);
}
float4 BlockLens(float3 dir, float3 p CS_WC_PARAM, uint lvl, float2 px) {
    // The probe strip, bottom left.
    const int2 ip = int2(px);
    const int py = int(gViewport.y) - 8;
    if (gProbeG.w > 0.5f && ip.y == py && ip.x >= 8 && ip.x < 8 + 21 * 8) {
        return BlockProbe(uint(ip.x - 8) / 21u, uint(ip.x - 8) % 21u);
    }
    // The key, bottom right: ranks 0..5 upward, 16 px swatches.
    const float2 kp = float2((gViewport.x - 6.0f) - px.x, (gViewport.y - 6.0f) - px.y);
    if (all(kp >= 0.0f) && kp.x < 16.0f && kp.y < 6.0f * 16.0f) {
        if (frac(kp.y / 16.0f) >= 0.875f || kp.x >= 14.0f) return float4(0, 0, 0, -1.0f);
        return float4(kBlkLensHue[int(kp.y / 16.0f)], -1.0f);
    }
    int rank;
    uint slice;
    float2 uv;
    float have;
    BlockLensAnswer(dir, p CS_WC, rank, slice, uv, have);
    float3 c = kBlkLensHue[clamp(rank, 0, 5)];
    if (rank > 0) c *= 0.45f + 0.55f * frac(float(slice) * 0.381966f + 0.25f);
    // A window's edge: its box coordinate (address + blkO) within a pixel of 0 or 1 (its own
    // footprint, per axis).
    const float2 bo = (rank > 0) ? uv + CsWinOff(lvl, uint(rank - 1)) : uv;
    const float2 fw = max(fwidth(bo), 1e-7f);
    const float2 e = min(bo, 1.0f - bo) / fw;
    if (rank > 0 && min(e.x, e.y) < 1.0f) c = float3(0.0f, 0.0f, 0.0f);
    if (lvl > 0u && ((ip.x + ip.y) / 3) % int(lvl + 1u) == 0) c *= 0.35f;
    return float4(c, float(slice) + 128.0f * float(lvl));
}

// The lens's entry point: the globe draws with it (GlobeLayer: m_msPsoLens / m_psoLens) only when
// --lens residency* or --lens addr is on. The gauge, the gate and the slice-plane discards are PsMain's own, so
// the lens paints exactly the fragments the picture would.
float4 PsResidencyLens(VsOut i) : SV_Target {
    LoadLevel(i.lvl);
    if (gGateA.x >= 0.0f) {
        const uint depth = LevelGateDepth(i.lvl);
        if (GateDepth(TrueRel(i.rel), depth + 1u) != depth) discard;
    }
    const float3 up = normalize(i.dir);
    if (gBankA.w > 0.5f) {
        if ((CsToTangent(up) * gGlo.x).z > gBankC.w) discard;
    }
    // HIERARCHY 4.17: the pixel's chain at PsMain's own point, so the lens shows what it reads.
    const float3 pA = (i.lvl == 0u) ? i.geo : CsPointOfDir(up);
    const WalkChain wc = CsChain(i.geo, i.lvl);   // the level's own windows, at its own point
    if (gBankA.z > 15.5f) return BlockLens(up, pA CS_WC, i.lvl, i.pos.xy);
    if (gBankA.z > 14.5f) return CloudAltLens(i.rel, i.pos.xy);
    const int tenant = int(gBankA.z + 0.5f) - 9;
    const float a = (tenant == 1)
                        ? HeightReadLens(up, ComposedHeightLod(length(i.rel), gWavesB.z) CS_WC)
                        : 1.0f;
    return float4(ResidencyLens(up, pA CS_WC, tenant, i.pos.xy), a);
}
