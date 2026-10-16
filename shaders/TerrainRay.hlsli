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
// the same of the GROUND on it -- the composed height field, read along the ray from the point --
// and one march serves two askers. The sun's asker wants the worst of the terrain's rise above the
// ray, the rim of the hill that eclipses the disc, in the sine units SunDiscClear takes, so a
// ridge's shadow has the sun's own penumbra and no edge of its own; the mirror's asker wants the
// FIRST sample where the ground stands above the ray, refined by one secant between the two
// samples that bracket it, and the land there is what the water reflects.
//
// The ray is made in small numbers, as the refracted cast is (WaterPixelColor): from the point's
// altitude a0 at its zenith upT, the ray's point at s is an altitude and a direction,
//     alt(s) = a0 + s mu + s^2 |tPerp|^2 / 2 (R + a0),   dir(s) = normalize(upT + s / (R + alt) tPerp),
// the sphere to second order in s / R (the third-order term is 2 mm at 8 km). The ground under it
// is read at the level's own point through that level's windows (ComposedHeightAt), at the level a
// step's length wants (two texels a step) and never finer than the pixel's own, so the march reads
// what the picture reads. The samples are log-spaced from twice the pixel's footprint to the
// range: a dune a metre off is seen as surely as a ridge eight kilometres off, and the march costs
// the same at every altitude. Its first sample stands two footprints out, so the pixel's own
// texel cannot shade itself. R is the asker's planet radius (the globe's gGlo.x, the buildings'
// gSkyLut.y: each stage's own name for the one sphere).
struct TerrainHit {
    float s;       // the ray's length to the ground, < 0 when it meets none within the range
    float3 dirP;   // the planet direction of the hit
    float3 pL;     // the hit in the level's own frame (the windows' address)
    float alt;     // its altitude above the geoid
};
float TerrainRay(float3 upT, float a0, float3 geo, uint s, float3 rayT, float footM, float lodPix, float R,
                 bool wantHit, out TerrainHit hit) {
    hit.s = -1.0f;
    hit.dirP = CsToPlanet(upT);
    hit.pL = geo;
    hit.alt = a0;
    const uint steps = uint(gCsShadow.x);
    if (steps == 0u) return 1.0f;
    const float mu = dot(rayT, upT);
    const float3 tPerp = rayT - mu * upT;
    const float tPerp2 = dot(tPerp, tPerp);
    const float cosE = sqrt(tPerp2);
    const float twoR = 2.0f * (R + a0);
    const float s0 = max(2.0f * footM, 0.5f);
    if (s0 >= gCsShadow.y) return 1.0f;   // from orbit the first sample is past the range: no ground to march
    const float ratio = pow(max(gCsShadow.y / s0, 1.0f), 1.0f / max(float(steps) - 1.0f, 1.0f));
    float worst = -1.0f;   // in sines: nothing stands above the ray yet
    float sP = s0, prevS = 0.0f, prevGap = 1.0e9f;
    [loop] for (uint k = 0u; k < steps; ++k) {
        const float altP = (a0 + sP * mu) + sP * sP * tPerp2 / twoR;
        const float3 dirP = normalize(upT + (sP / (R + altP)) * tPerp);
        const float lodP = max(lodPix, ComposedHeightLod(0.5f * sP * (ratio - 1.0f), 1.0f));
        const float3 pL = geo + upT * a0 + sP * rayT;
        const float gap = altP - ComposedHeightAt(CsToPlanet(dirP), pL, lodP, s);   // the ray's clearance
        worst = max(worst, -gap * cosE / sP);
        if (wantHit && gap < 0.0f) {
            const float sH = prevS + (sP - prevS) * prevGap / max(prevGap - gap, 1e-6f);
            hit.s = sH;
            hit.alt = (a0 + sH * mu) + sH * sH * tPerp2 / twoR;
            hit.dirP = CsToPlanet(normalize(upT + (sH / (R + hit.alt)) * tPerp));
            hit.pL = geo + upT * a0 + sH * rayT;
            break;
        }
        prevS = sP;
        prevGap = gap;
        sP *= ratio;
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
