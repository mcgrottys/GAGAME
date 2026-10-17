// ================================================================================================
//  TerrainRay.hlsli - THE SUN'S RAY FROM A LIT POINT, asked of the ground and of the buildings.
//
//  Two occluders stand between a point and the sun besides the planet itself (PlanetShadow): the
//  GROUND around it, read along the ray from the composed height field (TerrainRay), and the
//  BUILDINGS drawn near the eye, seen from the sun in one depth map (SunMapClear). Both answer in
//  the same unit -- the share of the sun's disc left clear, SunDiscClear's ramp over the disc's
//  diameter -- so a ridge's shadow, a tower's shadow and the planet's terminator have the one
//  penumbra, the sun's own, and no edge of their own. Included after Compose.hlsli by every stage
//  that lights a point of the eye's own world: the globe's surface (Globe.hlsl PsSurface and the
//  water's pixel), and the buildings' faces (Buildings.hlsl PsMain). The knobs are b2's
//  (Common.hlsli gCsShadow, gCsSunMapU, gCsSunVP: scene render.shadowSteps, shadowRangeM,
//  reflectLand, buildingShadowM), one buffer for every asker.
// ================================================================================================
#ifndef GA_TERRAIN_RAY_HLSLI
#define GA_TERRAIN_RAY_HLSLI

// THE GROUND'S OWN SHADOW, AND THE MIRROR'S LAND (scene render.shadowSteps / shadowRangeM /
// reflectLand). PlanetShadow asks whether the SPHERE stands between a point and the sun; this asks
// the same of the GROUND on it -- the composed height field along the ray from the point -- and one
// cast serves two askers. The sun's asker wants the worst of the terrain's rise above the ray, the
// rim of the hill that eclipses the disc, in the sine units SunDiscClear takes, so a ridge's shadow
// has the sun's own penumbra and no edge of its own; the mirror's asker wants the FIRST place the
// ground stands above the ray, refined by one secant, and the land there is what the water reflects.
//
// The ray is made in small numbers, as the refracted cast is (WaterPixelColor): from the point's
// altitude a0 at its zenith upT, the ray's point at t is an altitude
//     alt(t) = a0 + t mu + t^2 |tPerp|^2 / 2 (R + a0),
// the sphere to second order in t / R (the third-order term is 2 mm at 8 km), and the ground under
// it is addressed through the level's own windows at the level's own point (geo + upT a0 + t ray).
// The cast starts two footprints out, so the pixel's own texel cannot shade itself. R is the
// asker's planet radius (the globe's gGlo.x, the buildings' gSkyLut.y: each stage's own name for
// the one sphere).
struct TerrainHit {
    float s;       // the ray's length to the ground, < 0 when it meets none within the range
    float3 dirP;   // the planet direction of the hit
    float3 pL;     // the hit in the level's own frame (the windows' address)
    float alt;     // its altitude above the geoid
};

// ---- THE CAST: the question asked of the height's MAX LANE (MaxLane.hlsl). It replaced a march that
// read the height at twelve log-spaced points, each at a mip a step's length wanted: a coarse mip is a
// MEAN, so a ridge between two samples, or narrower than the texel a far sample read, was not there
// for it. The lane holds, for each 16 x 16 block of every resident mip of every page, the maximum
// any bilinear read at that mip or a finer resident one can return there; so a block the ray passes
// above can raise nothing, whatever lies inside it, and is passed whole. The cast is the maximum-
// mipmap traversal (Tevs, Ihrke, Seidel 2008): pass a block whose lane cannot reach the rim, step up
// a level after each pass; descend where it can; at the finest level the pixel may read, read the
// height itself, texel by texel, across the block. The result is the march's own quantity -- the
// worst rise of the ground above the ray, in sines, through SunDiscClear -- at every texel of the
// resident level instead of twelve of them, so the penumbra and every asker are unchanged.
//
// THE LADDER is the pages' own: the cube's mips G = top..0, then rank k's mips 2..0 at G = -3k - 1 ..
// -3k - 3 (rank k's mip 3 is rank k - 1's mip 0, the same pyramid level, and is never read here). A
// level is taken at the ray's point where its slice holds the point (a rank's box) and has it resident
// (the map's conservative floor, the read the picture takes): only there is its lane a bound.
//
// THE BLOCK'S EXIT IS EXACT: every cell boundary of a face-plane window, and of a cube face, is a
// plane through the body's centre, and the ray is straight in the level's frame, so the texel
// coordinate along it is a ratio of two linear functions of the ray's length, X(t) = N(t) / D(t),
// and the length at which it reaches a boundary Xb is one division: t + (Xb D - N) / (dN - Xb dD).
//
// THE BLOCK'S BOUND IS CLOSED: the rise of height M above the ray at t, in the march's sines, is
// f(t) cos e with f(t) = (M - a0) / t - mu - k t, k = |tPerp|^2 / 2 (R + a0). Above the point's own
// altitude f falls with t, so its largest value over a crossing [ta, tb] is at ta; below, f is concave
// with its top at t* = sqrt((a0 - M) / k), clamped into the crossing.
float CastBound(float M, float ta, float tb, float a0, float mu, float k, float cosE) {
    const float dM = M - a0;
    float tq = ta;
    if (dM < 0.0f) tq = (k > 0.0f) ? clamp(sqrt(-dM / k), ta, tb) : tb;
    return (dM / tq - mu - k * tq) * cosE;
}
// The length along the ray at which a projective coordinate X = N / D (N = n + t dn, D = d + t dd, at
// the current t: n, d) leaves the block [b S, (b + 1) S) it is in; 3e38 when it never does.
float CastExit(float t, float n, float dn, float d, float dd, float S) {
    const float X = n / d;
    const float b = floor(X / S);
    const float rate = dn * d - n * dd;   // the sign of dX/dt (D > 0)
    if (abs(rate) < 1e-20f) return 3.0e38f;
    const float Xb = (rate > 0.0f ? b + 1.0f : b) * S;
    const float den = dn - Xb * dd;
    if (abs(den) < 1e-20f) return 3.0e38f;
    const float dt = (Xb * d - n) / den;
    return (dt > 0.0f) ? t + dt : 3.0e38f;
}
// A level's cell at the ray's point: whether the level may be read there, its slice and mip, the
// block, the block's exit, the ground texel (m) and what a fine read needs.
struct CastCell {
    bool ok;
    bool cube;
    uint slice;
    uint mip;
    int2 blk;
    float tExit;
    float texM;
    float2 uv;     // a window's address (wrap)
    float3 dirP;   // a cube read's direction (planet frame)
    float have;    // the finest mip of the slot held under the point (the map's conservative floor)
};
// The cube face's three rows (HpCubeFace's ratios): s = cu.P / cw.P, t = cv.P / cw.P.
void CastFaceRows(uint f, out float3 cw, out float3 cu, out float3 cv) {
    cw = float3(1, 0, 0); cu = float3(0, 0, -1); cv = float3(0, -1, 0);
    if (f == 1u) { cw = float3(-1, 0, 0); cu = float3(0, 0, 1); cv = float3(0, -1, 0); }
    else if (f == 2u) { cw = float3(0, 1, 0); cu = float3(1, 0, 0); cv = float3(0, 0, 1); }
    else if (f == 3u) { cw = float3(0, -1, 0); cu = float3(1, 0, 0); cv = float3(0, 0, -1); }
    else if (f == 4u) { cw = float3(0, 0, 1); cu = float3(1, 0, 0); cv = float3(0, -1, 0); }
    else if (f == 5u) { cw = float3(0, 0, -1); cu = float3(-1, 0, 0); cv = float3(0, -1, 0); }
}
static const float kCastBlock = 16.0f;   // MIRRORS MaxLane.hlsl kBlock and hal::MaxLane::kBlock
CastCell CastCellAt(int G, float t, uint s, float3 p0, float3 P0, float3 rayT, float3 rayP) {
    CastCell c = (CastCell)0;
    c.tExit = 3.0e38f;
    if (G >= 0) {
        // THE CUBE: the direction of the ray's point, its face, the face's gnomonic ratios.
        const float3 P = CsToPlanet(P0 + t * rayT);
        c.dirP = normalize(P);
        float2 cuv;
        const uint f = HpCubeFace(c.dirP, cuv);
        const float have = CsHaveCubeArr(gCsU2.y, c.dirP);
        c.have = have;
        c.ok = have <= float(G) + 0.01f;
        c.cube = true;
        c.slice = f;
        c.mip = uint(G);
        const float dim = kPageDim / exp2(float(G));
        float3 cw, cu, cv;
        CastFaceRows(f, cw, cu, cv);
        // X = dim/2 (1 + cu.P / cw.P) = (dim/2)(cw + cu).P / cw.P; Y likewise with cv.
        const float d = dot(cw, P), dd = dot(cw, rayP);
        const float nx = 0.5f * dim * dot(cw + cu, P), dnx = 0.5f * dim * dot(cw + cu, rayP);
        const float ny = 0.5f * dim * dot(cw + cv, P), dny = 0.5f * dim * dot(cw + cv, rayP);
        c.blk = int2(floor(cuv * dim / kCastBlock));
        c.tExit = min(CastExit(t, nx, dnx, d, dd, kCastBlock), CastExit(t, ny, dny, d, dd, kCastBlock));
        c.texM = gCsGround.x * exp2(float(G));
        return c;
    }
    // A WINDOW: rank k's mip m, at the ray's point in the level's own frame.
    const uint k = uint(-G - 1) / 3u;
    const int m = G + 3 * int(k + 1u);
    if (k >= CsWinK(s)) return c;
    const uint r = 1u + 4u * k;
    const float4 U = WtWinRow(s, r), V = WtWinRow(s, r + 1u), W = WtWinRow(s, r + 2u);
    const float3 p = p0 + t * rayT;
    const float nx = dot(p, U.xyz) + U.w, ny = dot(p, V.xyz) + V.w, d = dot(p, W.xyz) + W.w;
    c.uv = float2(nx, ny) / (d * kPageDim);
    const float2 b = c.uv + CsWinOff(s, k);
    if (any(b < 0.0f) || any(b >= 1.0f)) return c;   // past the rank's box: its level is not here
    c.slice = CsWinSlice(s, k);
    c.mip = uint(m);
    const float have = CsHaveWindow(gCsU6.y, c.uv, c.slice);
    c.have = have;
    c.ok = have <= float(m) + 0.01f;
    const float S0 = kCastBlock * exp2(float(m));   // a block, in mip-0 texels
    c.blk = int2(floor(float2(nx, ny) / (d * S0)));
    c.tExit = min(CastExit(t, nx, dot(rayT, U.xyz), d, dot(rayT, W.xyz), S0),
                  CastExit(t, ny, dot(rayT, V.xyz), d, dot(rayT, W.xyz), S0));
    c.texM = CsBlockGround(k) * exp2(float(m));
    return c;
}
// The block's (maximum, minimum) from the lane.
float2 CastLane(CastCell c) {
    const int dimB = int(kPageDim / kCastBlock) >> c.mip;
    const int2 b = c.blk & (dimB - 1);   // a window's page is the lattice modulo itself
    return gTexArr[gCsU2.z].Load(int4(b, int(c.slice), int(c.mip))).xy;
}
// The cube's coarsest mip, the ladder's top (gCsG.x: 7 for the float pages' 8 mips, 6 for a half cube's 7).
int CastTop() { return int(gCsG.x + 0.5f); }
// The level read at length t (cube lod: the windows are below 0): the pixel's own, or the sun's disc
// there, whichever is coarser; and the level the traversal stops at for it, the one just coarser.
float CastRead(float lodPix, float discK, float t, int gMin) {
    return clamp(max(lodPix, log2(max(discK * t, 1e-30f))), float(gMin), float(CastTop()));
}
int CastStop(float lodPix, float discK, float t, int gMin) {
    return clamp(int(ceil(CastRead(lodPix, discK, t, gMin) - 1e-3f)), gMin, CastTop());
}
float TerrainRay(float3 upT, float a0, float3 geo, uint s, float3 rayT, float footM, float lodPix, float R,
                 bool wantHit, out TerrainHit hit) {
    hit.s = -1.0f;
    hit.dirP = CsToPlanet(upT);
    hit.pL = geo;
    hit.alt = a0;
    // Off (render.shadowSteps 0), or no lane to cast on: a height cube alone, as Mars has, carries
    // none, and its ground casts no shadow.
    if (gCsShadow.x < 0.5f || gCsU2.z == 0xFFFFFFFFu || gCsU6.x == 0xFFFFFFFFu) return 1.0f;
    const float mu = dot(rayT, upT);
    const float3 tPerp = rayT - mu * upT;
    const float tPerp2 = dot(tPerp, tPerp);
    const float cosE = sqrt(tPerp2);
    const float kq = tPerp2 / (2.0f * (R + a0));
    const float t0 = max(2.0f * footM, 0.5f);
    const float tMax = gCsShadow.y;
    if (t0 >= tMax) return 1.0f;
    const float3 p0 = geo + upT * a0;            // the point in the level's frame (the windows' address)
    const float3 P0 = upT * (R + a0);            // ...and about the planet's centre, tangent axes (the cube's)
    const float3 rayP = CsToPlanet(rayT);
    const int gMin = max(int(kCsHeightLodFloor), -3 * int(CsWinK(s)));
    const float lodF0 = clamp(lodPix, float(gMin), float(CastTop()));
    // THE GRAIN THE SUN CAN SEE: a rise narrower than the sun's disc at its distance covers only part
    // of the disc, which is the penumbra's own business, so the ground at length t is read at the
    // finest level whose texel is no wider than the disc there, t 2 kSunDiscR (0.94 % of t: 9.4 m at a
    // kilometre). One law, continuous in t: the pixel's own grain near the point, the disc's beyond.
    // The mirror's ray sees no disc and keeps the pixel's grain.
    const float discK = wantHit ? 1e-30f : 2.0f * kSunDiscR / gCsGround.x;   // the disc over the cube's texel, a metre out
    // The level read at length t, CONTINUOUS (a trilinear read between two mips, as the picture's own):
    // the traversal stops at the level just coarser, whose lane folds the finer one's and so bounds
    // the blend; an integer level here stepped the shade where a pixel's level crossed a half.
    // Start where a block is about the first sample's length: finer is pointless, coarser fails.
    int gHere = CastStop(lodF0, discK, t0, gMin);   // the level the traversal stops at, here
    int G = max(clamp(int(floor(log2(t0 / (kCastBlock * gCsGround.x)))), gMin, CastTop()), gHere);
    float worst = -1.0f;
    float t = t0;
    float prevT = 0.0f, prevGap = 1.0e9f;
    const uint cap = max(uint(gCsShadow.x), 8u);   // render.shadowSteps: the traversal's cap
    [loop] for (uint it = 0u; it < cap; ++it) {
        if (t >= tMax) break;
        const CastCell c = CastCellAt(G, t, s, p0, P0, rayT, rayP);
        if (!c.ok) {
            if (G >= CastTop()) break;   // nothing resident at all here: nothing to cast on
            ++G;
            gHere = G;
            continue;
        }
        const float tb = min(c.tExit, tMax);
        const float2 lane = CastLane(c);
        const float laneM = lane.x;
        // THE MINIMUM, the bound from below: the ground in the block stands at least this high, so its
        // rise over the crossing at least this much -- a value the true worst cannot fall under. It
        // raises the threshold every later block is passed against, and where it alone covers the
        // whole disc nothing further can give the sun back.
        if (!wantHit) {
            worst = max(worst, CastBound(lane.y, t, tb, a0, mu, kq, cosE));
            if (worst >= kSunDiscR) return 0.0f;
        }
        if (CastBound(laneM, t, tb, a0, mu, kq, cosE) <= (wantHit ? 0.0f : max(worst, -kSunDiscR))) {
            t = tb + max(1e-4f * tb, 1e-3f);
            G = min(G + 1, CastTop());
            gHere = CastStop(lodF0, discK, t, gMin);
            continue;
        }
        if (G > gHere) { --G; continue; }
        // THE FINEST LEVEL THIS PIXEL MAY READ: the height itself, a texel at a time across the block.
        // The read's mip within the slot: the continuous level less the slot's offset, never finer than
        // what is held under the point, nor than the slot's own mip 0 (a finer rank is another slice).
        const float mOff = float(c.mip) - float(G);
        const float mLo = max(c.have, max(float(c.mip) - 1.0f, 0.0f));
        const float dt = c.texM * exp2(clamp(CastRead(lodF0, discK, t, gMin) + mOff, mLo, float(c.mip)) - float(c.mip)) /
                         max(cosE, 0.05f);
        const uint r = c.cube ? 1u : 1u + 4u * (uint(-G - 1) / 3u);   // the rank's planes (unused on the cube)
        const float4 U = WtWinRow(s, r), V = WtWinRow(s, r + 1u), W = WtWinRow(s, r + 2u);
        [loop] for (float tf = t; tf < tb; tf += dt) {
            float H;
            const float mip = clamp(CastRead(lodF0, discK, tf, gMin) + mOff, mLo, float(c.mip));
            if (c.cube) {
                const float3 dirP = CsToPlanet(normalize(P0 + tf * rayT));
                H = gTexCubeArr[gCsU2.x].SampleLevel(sLinearClamp, float4(dirP, 0.0f), mip).x;
            } else {
                const float2 uv = PageTexelUv(p0 + tf * rayT, U, V, W);
                H = gTexArr[gCsU6.x].SampleLevel(sLinearWrap, float3(uv, c.slice), mip).x;
            }
            const float alt = a0 + tf * mu + tf * tf * kq;
            const float gap = alt - H;
            worst = max(worst, -gap * cosE / tf);
            if (wantHit && gap < 0.0f) {
                const float sH = prevT + (tf - prevT) * prevGap / max(prevGap - gap, 1e-6f);
                hit.s = sH;
                hit.alt = a0 + sH * mu + sH * sH * kq;
                hit.dirP = CsToPlanet(normalize(upT + (sH / (R + hit.alt)) * tPerp));
                hit.pL = geo + upT * a0 + sH * rayT;
                return SunDiscClear(-worst);
            }
            if (!wantHit && worst >= kSunDiscR) return 0.0f;   // the disc is gone: nothing can give it back
            prevT = tf;
            prevGap = gap;
            // The rest of the block, from here: the same closed bound, now from a later t (the ray has
            // risen, and the threshold may have too). Once it cannot reach, nothing left in the block can.
            if (CastBound(laneM, tf, tb, a0, mu, kq, cosE) <= (wantHit ? 0.0f : max(worst, -kSunDiscR))) break;
        }
        t = tb + max(1e-4f * tb, 1e-3f);
        G = min(G + 1, CastTop());
        gHere = CastStop(lodF0, discK, t, gMin);
    }
    return SunDiscClear(-worst);
}

// THE BUILDINGS' SHADOW (scene render.buildingShadowM, buildingShadowLevels; BuildingLayer::
// RenderSunMap). The solids the buildings layer draws are drawn once more each frame as the SUN
// sees them: orthographic casts along the sun over cells of the eye's own world, each a cell of the
// origin lattice, its depth the distance toward the sun over the cell's span (gCsSunVP[k]), so a
// texel holds the nearest solid the sun meets there. A lit point asks whether anything on its map
// stands nearer the sun than itself along its own ray.
//
// THE SUN IS A DISC, here as everywhere (SunDiscClear): the shadow's edge is not a line but the
// disc's width at the occluder's distance. The point sees the disc as a cone of half-angle
// kSunDiscR; a solid at distance D along the sun covers part of it, and on the map that part is
// the texels within D kSunDiscR of the point's own -- in map units, the depth difference times
// kSunDiscR, because the map's side is its depth range. So: the blockers nearer the sun within the
// widest cone any occluder on the map could fill (the far wall of the map) are found and averaged
// (their mean distance D), and the share of the disc they leave clear is the share of taps over a
// disc of radius D kSunDiscR that find nothing nearer the sun than the point -- the same ramp over
// the disc's diameter TerrainRay's worst rise gives, read from a map instead of a march. The taps
// are a Vogel spiral turned by a hash of the map texel (the ground's own, so the pattern stands
// still under a moving eye and a still reproduces).
//
// THE RECEIVER IS A PLANE (its normal n): a face lit at a slant stores its own depth in the map,
// and a tap beside the point reads the face a little nearer or farther along the sun. The plane's
// depth is extrapolated to the tap (its slope on the map from n through the map's own axes, the
// columns of gCsSunVP) and the comparison is made against that, so a wall does not shade itself
// in stripes and no bias has to guess the slope. The map holds each texel's depth at the texel's
// CENTRE, so the plane is evaluated there too (SunMapTap), not at the tap's own point: on a roof
// seen at 25 degrees the half texel between them is a texel of depth, and comparing at the tap's
// point speckled every roof with its own shadow. One texel of depth is the margin left for the
// map's own precision. The slope is held to eight depth texels a texel: a face past 83 degrees to
// the sun takes almost no sun and is left to its own margin.
//
// THE MAPS ARE A CASCADE ON THE ORIGIN LATTICE (VISIBILITY_FIELD 9; BuildingLayer::PlanSunMap): one
// cast per CELL the eye's ground address names -- its cell at the finest level, then that cell's
// parents, each twice the span at the same texel count. A lit point takes the FINEST map that holds
// it (the floor of its address), and from there the one law below runs on that level's matrix
// alone: the uv, the blocker search, the disc, the receiver plane and the texel's margin are all in
// the map's own units, so every quantity scales with the cell's grain and no level has a case of its
// own. The seam between two levels is a change of grain, not of law.
// A tap: the texel under uv + d of map `srv`, its stored depth against the plane at its centre.
bool SunMapTap(uint srv, float2 uv, float2 d, float z, float2 grad, float margin, float N, out float zb) {
    const float2 tx = clamp(floor((uv + d) * N), 0.0f, N - 1.0f);
    zb = gTex[srv].Load(int3(int(tx.x), int(tx.y), 0)).x;
    const float2 dc = (tx + 0.5f) / N - uv;   // the texel's centre, from the point
    return zb > z + dot(grad, dc) + margin;
}
float SunMapClear(float3 rel, float3 n) {
    const uint levels = min(gCsSunMapU.x, GA_SUN_LEVELS);
    if (levels == 0u) return 1.0f;
    // THE FLOOR OF THE POINT'S ADDRESS: the finest cell whose map holds the point.
    uint k = 0u;
    float4 q = float4(0.0f, 0.0f, 0.0f, 1.0f);
    bool held = false;
    [loop] for (k = 0u; k < levels && !held; ++k) {
        q = mul(float4(rel, 1.0f), gCsSunVP[k]);   // orthographic: w is 1
        held = all(abs(q.xy) < 1.0f) && q.z > 0.0f && q.z < 1.0f;
    }
    if (!held) return 1.0f;   // off every map: the sun is the planet's
    k -= 1u;
    const uint srv = gCsSunSrv[k >> 2u][k & 3u];
    if (srv == 0xFFFFFFFFu) return 1.0f;
    const float4x4 M = gCsSunVP[k];
    const float2 uv = float2(0.5f + 0.5f * q.x, 0.5f - 0.5f * q.y);
    const float N = float(gCsSunMapU.y);
    const float texel = 1.0f / N;
    // The map's axes in the eye's frame are the matrix's columns (scaled); the plane's normal in
    // map coordinates is n against each, over the column's own length squared.
    const float3 c0 = float3(M._m00, M._m10, M._m20);
    const float3 c1 = float3(M._m01, M._m11, M._m21);
    const float3 c2 = float3(M._m02, M._m12, M._m22);
    const float3 nq = float3(dot(n, c0) / dot(c0, c0), dot(n, c1) / dot(c1, c1), dot(n, c2) / dot(c2, c2));
    const float nz = (abs(nq.z) > 1e-6f) ? nq.z : 1e-6f;
    // Depth per uv step on the plane: uv.x runs with clip x (2 per side), uv.y against clip y.
    const float2 grad = clamp(float2(-nq.x, nq.y) / nz * 2.0f, -8.0f, 8.0f);
    const float margin = 1.0f * texel;
    // The spiral's turn, from the map texel: Jimenez's interleaved gradient noise on its index.
    const float2 tx = floor(uv * N);
    const float turn = 6.2831853f * frac(52.9829189f * frac(dot(tx, float2(0.06711056f, 0.00583715f))));
    const float ca = cos(turn), sa = sin(turn);
    const float2x2 R = float2x2(ca, -sa, sa, ca);
    // THE BLOCKERS: taps over the widest cone the map can hold an occluder in (its far wall), each
    // a texel at least so a neighbour is seen.
    const float rSearch = max((1.0f - q.z) * kSunDiscR, texel);
    float zSum = 0.0f;
    float nBlock = 0.0f;
    [unroll] for (uint k = 0u; k < 8u; ++k) {
        const float r = rSearch * sqrt((float(k) + 0.5f) / 8.0f);
        const float a = float(k) * 2.39996323f;
        const float2 d = mul(R, float2(cos(a), sin(a))) * r;
        float zb;
        if (SunMapTap(srv, uv, d, q.z, grad, margin, N, zb)) {
            zSum += zb;
            nBlock += 1.0f;
        }
    }
    if (nBlock == 0.0f) return 1.0f;
    // THE DISC: the occluders' mean distance sets its radius on the map; the share of its taps
    // that see nothing nearer the sun is the share of the disc left clear.
    const float rDisc = max((zSum / nBlock - q.z) * kSunDiscR, 0.5f * texel);
    float clear = 0.0f;
    [unroll] for (uint j = 0u; j < 16u; ++j) {
        const float r = rDisc * sqrt((float(j) + 0.5f) / 16.0f);
        const float a = float(j) * 2.39996323f;
        const float2 d = mul(R, float2(cos(a), sin(a))) * r;
        float zb;
        clear += SunMapTap(srv, uv, d, q.z, grad, margin, N, zb) ? 0.0f : 1.0f;
    }
    return clear / 16.0f;
}

#endif  // GA_TERRAIN_RAY_HLSLI
