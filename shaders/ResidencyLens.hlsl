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
static const float3 kResLensHue[3] = {float3(0.60f, 0.25f, 1.00f), float3(0.15f, 1.00f, 0.30f),
                                      float3(1.00f, 0.30f, 0.10f)};
float3 ResLensColor(int page, float have, float mips) {
    if (have > mips - 0.5f) return float3(1.0f, 0.0f, 1.0f);
    return kResLensHue[clamp(page, 0, 2)] * (1.0f - floor(have + 0.5f) / 9.0f);
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
float3 ResidencyLens(float3 dir, int tenant, float2 px) {
    // The key, bottom left: 16 px cells, 14 px swatches.
    const float2 kp = float2(px.x - 6.0f, (gViewport.y - 6.0f) - px.y);
    if (all(kp >= 0.0f) && kp.x < 9.0f * 16.0f && kp.y < 3.0f * 16.0f) {
        const int col = int(kp.x / 16.0f), row = int(kp.y / 16.0f);
        if (frac(kp.x / 16.0f) >= 0.875f || frac(kp.y / 16.0f) >= 0.875f) return float3(0, 0, 0);
        if (col < 8) return ResLensColor(row, float(col), 99.0f);
        return (row == 0) ? float3(1.0f, 0.0f, 1.0f) : float3(0.12f, 0.12f, 0.12f);
    }
    // Every page's uv and its screen derivative, taken here under uniform control flow: the
    // tile outline needs the derivative of whichever page answers.
    const float2 cuv = ResLensCubeUv(dir);
    const float2 duv = CsWindowUv(dir);
    const float2 tuv = duv * gCsDet.z + gCsDet.xy;
    const float2 fwC = fwidth(cuv), fwW = fwidth(duv), fwT = fwidth(tuv);
    // HIERARCHY 4.17 commit 2: each standing block's uv and derivative, here too. A block is
    // painted as the page of its rank (rung / 3): rank 2 as the z14 page, rank 3 as the z17.
    float2 buv[4], fwB[4];
    [unroll] for (uint k = 0; k < 4; ++k) {
        buv[k] = PageTexelUv(dir, gCsBlkU[k], gCsBlkV[k], gCsBlkW[k]);
        fwB[k] = fwidth(buv[k]);
    }
    int page = 0;
    float have = 99.0f, mips = 8.0f;
    float2 uv = cuv, fw = fwC, tiles = float2(128.0f, 128.0f);   // tiles a side at mip 0
    const bool inW = all(duv > 0.0f) && all(duv < 1.0f);
    if (tenant == 0) {
        // earth.color: ComposedColorPages' ladder.
        if (gCsU5.x == 0xFFFFFFFFu || gCsF.x < 0.5f) return float3(0.3f, 0.3f, 0.3f);
        const float3 g0 = CsGroundM();
        have = CsHaveCubeArr(gCsU.y, dir);
        float ground = PageGroundM(g0.x, have);
        if (gCsF.y > 0.5f && inW) {
            const float hW = CsHavePage(gCsU5.y, duv, gCsU5.z);
            const float gW = PageGroundM(g0.y, hW);
            if (PageWins(gW, ground)) {
                page = 1;
                have = hW;
                ground = gW;
                uv = duv;
                fw = fwW;
            }
            if (gCsU5.w != 0xFFFFFFFFu && all(tuv > 0.0f) && all(tuv < 1.0f)) {
                const float hD = CsHavePage(gCsU5.y, tuv, gCsU5.w);
                if (PageWins(PageGroundM(g0.z, hD), ground)) {
                    page = 2;
                    have = hD;
                    uv = tuv;
                    fw = fwT;
                }
            }
        }
        for (uint i = 0; i < gCsBlkN.x; ++i) {   // the standing blocks, ComposedColorPages' ladder
            if (!(all(buv[i] > 0.0f) && all(buv[i] < 1.0f))) continue;
            const float hB = CsHavePage(gCsU5.y, buv[i], gCsBlkS[i]);
            const float gB = PageGroundM(gCsBlkG[i], hB);
            if (PageWins(gB, ground)) {
                page = clamp(int(round(log2(g0.x / gCsBlkG[i]) / 3.0f)) - 1, 1, 2);
                have = hB;
                ground = gB;
                uv = buv[i];
                fw = fwB[i];
            }
        }
    } else if (tenant == 1) {
        // earth.height: ComposedHeightPages' choice; its tiles are 256 x 128 texels, 7 mips.
        if (gCsU6.x == 0xFFFFFFFFu || gCsF.z < 0.5f) return float3(0.3f, 0.3f, 0.3f);
        mips = 7.0f;
        tiles = float2(64.0f, 128.0f);
        have = CsHaveCubeArr(gCsU2.y, dir);
        if (inW) {
            const float hW = CsHavePage(gCsU6.y, duv, gCsU6.z);
            if (PageWins(have, hW, CsGroundM().xy)) {
                page = 1;
                have = hW;
                uv = duv;
                fw = fwW;
            }
        }
    } else {
        // gis.landsea: CsMaskSample's order -- the finest page with an OPINION answers.
        if (gCsU3.x == 0xFFFFFFFFu) return float3(0.3f, 0.3f, 0.3f);
        page = -1;
        for (int i = int(gCsBlkN.x) - 1; i >= 0 && page < 0; --i) {   // the standing blocks
            if (!(all(buv[i] > 0.001f) && all(buv[i] < 0.999f))) continue;
            const float hB = CsHavePage(gCsU3.y, buv[i], gCsBlkS[i]);
            if (hB <= 7.5f &&
                PageSampleLevel(gTexArr[gCsU3.x], sLinearClamp, buv[i], gCsBlkS[i], 0.0f, hB).a >
                    0.001f) {
                page = clamp(int(round(log2(CsGroundM().x / gCsBlkG[i]) / 3.0f)) - 1, 1, 2);
                have = hB;
                uv = buv[i];
                fw = fwB[i];
            }
        }
        if (inW && gCsBlkN.x == 0) {
            if (gCsU5.w != 0xFFFFFFFFu && all(tuv > 0.001f) && all(tuv < 0.999f)) {
                const float hD = CsHavePage(gCsU3.y, tuv, 7u);
                if (hD <= 7.5f &&
                    PageSampleLevel(gTexArr[gCsU3.x], sLinearClamp, tuv, 7u, 0.0f, hD).a > 0.001f) {
                    page = 2;
                    have = hD;
                    uv = tuv;
                    fw = fwT;
                }
            }
            if (page < 0) {
                const float hW = CsHavePage(gCsU3.y, duv, 6u);
                if (hW <= 7.5f &&
                    PageSampleLevel(gTexArr[gCsU3.x], sLinearClamp, duv, 6u, 0.0f, hW).a > 0.001f) {
                    page = 1;
                    have = hW;
                    uv = duv;
                    fw = fwW;
                }
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

// The lens's entry point: the globe draws with it (GlobeLayer: m_msPsoLens / m_psoLens) only when
// --lens residency* is on. The gauge, the gate and the slice-plane discards are PsMain's own, so
// the lens paints exactly the fragments the picture would.
float4 PsResidencyLens(VsOut i) : SV_Target {
    LoadLevel(i.lvl);
    if (gGateA.x >= 0.0f) {
        if (GateDepth(TrueRel(i.rel)) != LevelGateDepth(i.lvl)) discard;
    }
    const float3 up = normalize(i.dir);
    if (gBankA.w > 0.5f) {
        if ((CsToTangent(up) * gGlo.x).z > gBankC.w) discard;
    }
    return float4(ResidencyLens(up, int(gBankA.z + 0.5f) - 9, i.pos.xy), 1.0f);
}
