// ================================================================================================
//  TailRead.hlsli -- THE LEVELS' DEPARTURE AT A POINT OF THE PLANET (stages 2b/2c, 2026-10-27; docs/WATER_V2.md
//  14). The readers of the far field -- the bank kernel's texel, the globe's vertex -- read the finest level that
//  holds the point, in place of the impacts' analytic law (ImpactRead.hlsli), which stays for points no level
//  holds. Included after the laws and the heap's Texture2D array, with TAIL_TEX naming it.
//
//  The constants (both cbuffers, appended at the end; sim/ImpactGpu.h LevelsGpu, SweSolver::LevelsGpuDesc):
//  gLvlChart = the chart's anchor up (planet frame), R; gLvlE = its east, the level count; gLvlN = its north, the
//  display exaggeration; gLvlA[k] = a level's first cell centre on the chart (s0, t0, over R) and its steps (ds, dt);
//  gLvlU[k] = its h and bed SRVs on the heap, its cells across and down. Coarsest first.
//
//  THE CELL under a direction is the chart's: s = (d . E) / (d . A), t = (d . N) / (d . A), the cell
//  ((s - s0) / ds, (t - t0) / dt). THE DEPARTURE is the column less mean sea's over the bed, h + b where b < 0 (a
//  tile the level does not hold reads 0: no sea, nothing). A land cell takes its nearest sea neighbour's (the coast
//  within a cell). THE SLOPE is the neighbours' departures over the chart's true metric, along the chart's east and
//  north projected onto the tangent plane at the point.
// ================================================================================================
#ifndef GA_TAIL_READ_HLSLI
#define GA_TAIL_READ_HLSLI

bool TailHolds() { return uint(gLvlE.w) != 0u; }

// the departure at a level's cell (m), and the water the level stands there (0: no sea at that cell)
float LvlDepCell(uint k, int2 c, out float depth) {
    depth = 0.0f;
    if (any(c < 0) || c.x >= int(gLvlU[k].z) || c.y >= int(gLvlU[k].w)) return 0.0f;
    const float b = TAIL_TEX[gLvlU[k].y].Load(int3(c, 0)).x;
    if (b >= 0.0f) return 0.0f;
    depth = -b;
    return TAIL_TEX[gLvlU[k].x].Load(int3(c, 0)).x + b;
}

// THE FIELD AT A DIRECTION: the finest level's departure there (m, exaggerated), the water it stands in (m), its
// slope in the planet's frame (per metre, exaggerated). False where no level holds the point.
bool TailAt(float3 dir, out float dep, out float depth, out float3 slopePlanet) {
    dep = 0.0f;
    depth = 0.0f;
    slopePlanet = 0.0f;
    const float a = dot(dir, gLvlChart.xyz);
    if (!TailHolds() || a <= 0.3f) return false;   // the near hemisphere, well inside
    const float2 st = float2(dot(dir, gLvlE.xyz) / a, dot(dir, gLvlN.xyz) / a);
    const uint n = uint(gLvlE.w);
    [loop] for (int kk = int(n) - 1; kk >= 0; --kk) {   // the finest first
        const uint k = uint(kk);
        if (gLvlU[k].x == 0xFFFFFFFFu) continue;
        const float2 cxy = (st - gLvlA[k].xy) / gLvlA[k].zw;
        if (any(cxy < 0.0f) || cxy.x >= float(gLvlU[k].z) || cxy.y >= float(gLvlU[k].w)) continue;
        const int2 c = int2(floor(cxy));
        float d0 = LvlDepCell(k, c, depth);
        if (depth <= 0.0f) {   // the coast: the nearest sea cell of the eight about it
            [unroll] for (int j = 0; j < 9; ++j) {
                const int2 o = int2(j % 3 - 1, j / 3 - 1);
                if (all(o == 0)) continue;
                float dk;
                const float v = LvlDepCell(k, c + o, dk);
                if (dk > 0.0f) {
                    d0 = v;
                    depth = dk;
                    break;
                }
            }
            if (depth <= 0.0f) continue;   // no sea at this level here: the coarser may hold some
        }
        float dd;
        float dE = LvlDepCell(k, c + int2(1, 0), dd); if (dd <= 0.0f) dE = d0;
        float dW = LvlDepCell(k, c - int2(1, 0), dd); if (dd <= 0.0f) dW = d0;
        float dN = LvlDepCell(k, c - int2(0, 1), dd); if (dd <= 0.0f) dN = d0;   // north is -y
        float dS = LvlDepCell(k, c + int2(0, 1), dd); if (dd <= 0.0f) dS = d0;
        const float q = 1.0f + st.x * st.x + st.y * st.y;   // the chart's metric at the point (Swe.hlsl CellM)
        const float dxT = gLvlChart.w * gLvlA[k].z * sqrt(1.0f + st.y * st.y) / q;
        const float dyT = gLvlChart.w * (-gLvlA[k].w) * sqrt(1.0f + st.x * st.x) / q;
        const float gs = (dE - dW) / (2.0f * dxT), gt = (dN - dS) / (2.0f * dyT);
        float3 es = gLvlE.xyz - dot(gLvlE.xyz, dir) * dir, en = gLvlN.xyz - dot(gLvlN.xyz, dir) * dir;
        es = (dot(es, es) > 1e-8f) ? normalize(es) : 0.0f;
        en = (dot(en, en) > 1e-8f) ? normalize(en) : 0.0f;
        dep = d0 * gLvlN.w;
        slopePlanet = (gs * es + gt * en) * gLvlN.w;
        return true;
    }
    return false;
}
#endif
