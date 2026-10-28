// ================================================================================================
//  TailRead.hlsli -- THE TAIL'S DEPARTURE AT A POINT OF THE PLANET (stage 2b, 2026-10-27; docs/WATER_V2.md 14).
//  The readers of the far field -- the bank kernel's texel, the globe's vertex -- read the tail's column where
//  the tail holds the point, in place of the impacts' analytic law (ImpactRead.hlsli), which stays for points
//  the tail does not hold. Included after the laws and the heap's Texture2D array, with TAIL_TEX naming it.
//
//  The constants (both cbuffers, appended at the end; sim/ImpactGpu.h TailGpu, SweSolver::TailGpuDesc):
//  gTailA = the tail's anchor up (planet frame), R; gTailE = its east, the cell's angle east (ds, over R);
//  gTailN = its north, the cell's angle north (dt); gTailU = the tail's h and bed SRVs (the heap), its cells a
//  side, 0; gTailB = the display exaggeration (the scene's, on the drawn departure as on the drawn law), 0, 0, 0.
//
//  THE CELL under a direction is the gnomonic chart's: s = (d . E) / (d . A), t = (d . N) / (d . A), the cell
//  (s / ds + n/2, n/2 - t / dt). THE DEPARTURE is the column less mean sea's over the bed, h + b where b < 0
//  (a tile the tail does not hold reads 0: no sea, nothing). A land cell takes its nearest sea neighbour's
//  (the coast within a tail cell). THE SLOPE is the neighbours' departures over the chart's true metric,
//  along the chart's east and north projected onto the tangent plane at the point.
// ================================================================================================
#ifndef GA_TAIL_READ_HLSLI
#define GA_TAIL_READ_HLSLI

bool TailHolds() { return gTailU.z != 0u && gTailU.x != 0xFFFFFFFFu && gTailU.y != 0xFFFFFFFFu; }

bool TailCellOfDir(float3 dir, out int2 c, out float2 st) {
    c = int2(0, 0);
    st = 0.0f;
    const float a = dot(dir, gTailA.xyz);
    if (!TailHolds() || a <= 0.3f) return false;   // the near hemisphere, well inside
    st = float2(dot(dir, gTailE.xyz) / a, dot(dir, gTailN.xyz) / a);
    const float n = float(gTailU.z);
    const float2 cxy = float2(st.x / gTailE.w + 0.5f * n, 0.5f * n - st.y / gTailN.w);
    if (any(cxy < 0.0f) || any(cxy >= n)) return false;
    c = int2(floor(cxy));
    return true;
}

// the departure at a tail cell (m), and the water the tail stands there (0: no sea at that cell)
float TailDepCell(int2 c, out float depth) {
    depth = 0.0f;
    if (any(c < 0) || any(c >= int(gTailU.z))) return 0.0f;
    const float b = TAIL_TEX[gTailU.y].Load(int3(c, 0)).x;
    if (b >= 0.0f) return 0.0f;
    depth = -b;
    return TAIL_TEX[gTailU.x].Load(int3(c, 0)).x + b;
}

// THE TAIL AT A DIRECTION: its departure there (m, exaggerated), the water it stands in (m), its slope in the
// planet's frame (per metre, exaggerated). False where the tail does not hold the point.
bool TailAt(float3 dir, out float dep, out float depth, out float3 slopePlanet) {
    dep = 0.0f;
    depth = 0.0f;
    slopePlanet = 0.0f;
    int2 c;
    float2 st;
    if (!TailCellOfDir(dir, c, st)) return false;
    float d0 = TailDepCell(c, depth);
    if (depth <= 0.0f) {   // the coast: the nearest sea cell of the eight about it
        [unroll] for (int k = 0; k < 9; ++k) {
            const int2 o = int2(k % 3 - 1, k / 3 - 1);
            if (all(o == 0)) continue;
            float dk;
            const float v = TailDepCell(c + o, dk);
            if (dk > 0.0f) {
                d0 = v;
                depth = dk;
                break;
            }
        }
        if (depth <= 0.0f) return true;   // held, with nothing: the plane
    }
    float dd;
    float dE = TailDepCell(c + int2(1, 0), dd); if (dd <= 0.0f) dE = d0;
    float dW = TailDepCell(c - int2(1, 0), dd); if (dd <= 0.0f) dW = d0;
    float dN = TailDepCell(c - int2(0, 1), dd); if (dd <= 0.0f) dN = d0;   // north is -y
    float dS = TailDepCell(c + int2(0, 1), dd); if (dd <= 0.0f) dS = d0;
    const float q = 1.0f + st.x * st.x + st.y * st.y;   // the chart's metric at the point (Swe.hlsl CellM)
    const float dxT = gTailA.w * gTailE.w * sqrt(1.0f + st.y * st.y) / q;
    const float dyT = gTailA.w * gTailN.w * sqrt(1.0f + st.x * st.x) / q;
    const float gs = (dE - dW) / (2.0f * dxT), gt = (dN - dS) / (2.0f * dyT);
    float3 es = gTailE.xyz - dot(gTailE.xyz, dir) * dir, en = gTailN.xyz - dot(gTailN.xyz, dir) * dir;
    es = (dot(es, es) > 1e-8f) ? normalize(es) : 0.0f;
    en = (dot(en, en) > 1e-8f) ? normalize(en) : 0.0f;
    dep = d0 * gTailB.x;
    slopePlanet = (gs * es + gt * en) * gTailB.x;
    return true;
}
#endif
