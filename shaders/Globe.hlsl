// ================================================================================================
//  Globe.hlsl - M6: the planet.
//
//  A quad-sphere CDLOD earth: the CPU walks a quadtree per cube face and emits visible nodes;
//  every node draws the same 32x32 grid, morphing odd vertices toward their even neighbours as
//  a node approaches the distance where its parent takes over (Strugar's CDLOD), so LOD rings
//  cross-fade with no cracks and no stitching.
//
//  M6i: surface DATA arrives through the layer compositor's channels (Compose.hlsli): color is
//  ComposedColor (Google cube + Mercator window, painted per stack), height is ComposedHeight
//  (ETOPO + NE 15s + CUDEM for Earth, MOLA for Mars -- painted per stack). The equirect relief
//  texture, the NE-window blend (NeWeight/ReliefBlended) and the inline detail-window block are
//  all GONE from render time; the ocean is still shaded with the live GFS-Wave field (physics a
//  baked mosaic cannot know). Mars's native BC1/BC5 pyramids stay direct streams (gStreamU).
//
//  Precision contract: positions are dir * (R + h) - camAbs in float. Both terms are ~6.4e6 m,
//  so the difference carries ~0.5-1 m of noise -- static per vertex, invisible above the 2 km
//  minimum altitude this mode allows. The metre-precise near field is the estuary's job; both
//  render in ONE shared tangent frame (M6g).
// ================================================================================================
#define GA_NO_FIELD_BUFFER
#include "Common.hlsli"

cbuffer GlobeCb : register(b1) {
    float4 gGlo;        // x = R (m), y = relief exaggeration, z = node count, w = sim time s
    float4 gCamAbs;     // xyz = sphere-centred TANGENT-frame camera (flat cam + (0,R,0), the
                        // doubles cancelled on the CPU)
    uint4  gTexIdx;     // x = unused (was equirect relief; the composed height cube retired
                        // it), y = Hs SRV, z = wind SRV, w = CLOUD VOLUME SRV (0xFFFFFFFF =
                        // absent; the volume is a TileAtlas3D -- null = clear air)
    float4 gWavesA;     // Hs/wind grid: lat1 (deg, row 0), lon1, 1/dlat, 1/dlon (rows N->S)
    float4 gWavesB;     // x = nx, y = ny, z = pixel angular size (rad), w = unused
    float4 gCloudA;     // x = extinction /m at density 1, y = shell top (m), z = sun boost,
                        // w = ground-shadow strength
    uint4  gTexIdx2;    // x = unused (was NE 15s relief; composed away), y = wind Mv2 bank
                        // SRV, z = overlay on, w = --albedo texture-work lens
    float4 gWindGeo;    // wind grid: lat1, lon1, 1/dlat, 1/dlon
    float4 gWindB;      // x = nx, y = ny
    uint4  gStreamU;    // Mars native streams (cube SRVs): x = surface (BC1), y = normal
                        // (BC5), z = surface residency map, w = normal residency map
    float4 gStreamF;    // x = surface on, y = normal on, z = planet is Mars, w = unused
    // M6i: the composed channels (color cube + window, height cube) + the M6g one-world frame
    // rows, shared VERBATIM with every other layer that samples this planet's surface.
    GA_COMPOSED_CB_ROWS
    float4 gEstGeo;     // estuary CUDEM window (deg): lon0, lat1, 1/lonSpan, 1/latSpan
                        // (w also gates: 0 = absent). The globe FOUNDATION-SINKS a few metres
                        // inside it so the sharp CUDEM surface owns the depth buffer there.
    // M7: THE WAVE VERTEX BANK -- water geometry + params from ONE tiled resource: a camera-
    // anchored mip ladder of rings (m of 6, ring m at atlas x offset m*512). one-water mode
    // displaces water VERTEXES from it and shades from its params; a NULL tile reads zero =
    // the calm plane, which is the correct absence.
    uint4  gBankU;      // disp SRV, param SRV, one-water on, ring texels
    float4 gBankA;      // base texel m, mip count, unused, unused
    float4 gBankOrg01;  // ring origins (world m): r0.xy, r1.zw
    float4 gBankOrg23;
    float4 gBankOrg45;
    // M7a: THE SPARKLE ROWS -- the detail plane + the cascade derivative textures, so the
    // pixel stage can recover bands its footprint resolves but the ring texel does not.
    uint4  gBankU2;     // detail bank SRV, cascade deriv SRVs x3
    float4 gBankB;      // cascade patch sizes x3, height exaggeration
    float4 gBankC;      // representative wavenumber per cascade, w = slice offset
    float4 gBankD;      // M8: unit-sea rms envelope per band (xyz), w = foam opacity
    float4 gBankE;      // M8: x = ring cross-fade width (texels), yzw spare (scene cfg)
    // M9: THE WATER'S QUALITY (docs/ALGEBRA.md "optics"). One RGBA plane carries the NOAA
    // gap-filled retrievals -- (log10 chl-a, Kd490, log10 SPM, retrieved?) -- and one scalar
    // plane the GFS sea-ice concentration on the WAVE grid. gOptU.z == 0 restores the M7c
    // constants byte for byte, so the A/B is one flag and the old look is never lost.
    uint4  gOptU;       // ocean-colour SRV, ice SRV, optics on, spare
    float4 gOptA;       // ocean grid: lat1, lon1, 1/dlat, 1/dlon
    float4 gOptB;       // nx, ny, deep-albedo gain g, spare
};

// Sample the bank at a world-frame XZ: finest ring containing the point wins. Returns false
// beyond every ring (the far field: sub-pixel waves, level ~ the plane).
// One ring's manual-bilinear fetch (the static-sampler SampleLevel path reads ZERO from
// the MESH stage on this driver -- Load is stage-proof). edgeD = distance to the ring's
// valid border in texels, for the cross-fade below.
bool BankFetch(uint m, float2 worldXZ, out float4 disp, out float4 param,
               out float4 detail, out float texelOut, out float edgeD) {
    disp = 0.0f;
    param = 0.0f;
    detail = 0.0f;
    texelOut = 0.0f;
    edgeD = 0.0f;
    const float texel = gBankA.x * (float)(1u << m);
    const float2 org = (m == 0) ? gBankOrg01.xy
                      : (m == 1) ? gBankOrg01.zw
                      : (m == 2) ? gBankOrg23.xy
                      : (m == 3) ? gBankOrg23.zw
                      : (m == 4) ? gBankOrg45.xy
                                 : gBankOrg45.zw;
    const float2 local = (worldXZ - org) / texel;
    if (any(local < 1.0f) || any(local > 511.0f)) return false;
    edgeD = min(min(local.x - 1.0f, 511.0f - local.x),
                min(local.y - 1.0f, 511.0f - local.y));
    const float2 tf = local - 0.5f;
    const int2 t0 = int2(floor(tf));
    const float2 fr = tf - float2(t0);
    const int xoff = (int)m * 512;
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = t0 + int2(k & 1, k >> 1);
        const float wgt = ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y);
        disp += wgt * gTex[gBankU.x][int2(xoff + tc.x, tc.y)];
        param += wgt * gTex[gBankU.y][int2(xoff + tc.x, tc.y)];
        detail += wgt * gTex[gBankU2.x][int2(xoff + tc.x, tc.y)];
    }
    texelOut = texel;
    return true;
}

// ---- M8 FOAM BREAKUP (ALGEBRA.md foamlaw; the reference's range-faded octaves) ----
// Per-pixel value noise in three IRRATIONALLY-ROTATED octaves (axis-aligned octaves at
// 2x spacing share lattice seams and sum to rectangular blocks); each octave fades once
// its features stop covering several pixels of RANGE (never a bank texel -- a shading
// texture lives at shading resolution). The renormalization keeps the mean exactly 1/2
// in every fade state, so foam coverage never becomes a function of camera distance.
float FoamHash21(float2 p) {
    p = frac(p * float2(123.34f, 456.21f));
    p += dot(p, p + 45.32f);
    return frac(p.x * p.y);
}

float FoamValueNoise(float2 p) {
    const float2 i0 = floor(p);
    const float2 f = frac(p);
    const float2 u = f * f * (3.0f - 2.0f * f);
    return lerp(lerp(FoamHash21(i0), FoamHash21(i0 + float2(1, 0)), u.x),
                lerp(FoamHash21(i0 + float2(0, 1)), FoamHash21(i0 + float2(1, 1)), u.x),
                u.y);
}

float FoamBreakup(float2 posM, float range) {
    const float2x2 r1 = float2x2(0.8776f, -0.4794f, 0.4794f, 0.8776f);
    const float2x2 r2 = float2x2(0.6062f, -0.7952f, 0.7952f, 0.6062f);
    const float w1 = smoothstep(0.035f, 0.130f, 2.4f / max(range, 1e-3f));
    const float w2 = smoothstep(0.035f, 0.130f, 6.5f / max(range, 1e-3f));
    const float w3 = smoothstep(0.035f, 0.130f, 17.0f / max(range, 1e-3f));
    const float n = 0.45f * FoamValueNoise(posM * 0.42f) * w1 +
                    0.34f * FoamValueNoise(mul(r1, posM) * 0.155f) * w2 +
                    0.21f * FoamValueNoise(mul(r2, posM) * 0.059f) * w3;
    const float wsum = 0.45f * w1 + 0.34f * w2 + 0.21f * w3;
    return (wsum > 1e-3f) ? (n / wsum) : 0.5f;
}

bool BankSample(float2 worldXZ, out float4 disp, out float4 param, out float4 detail,
                out float texelOut) {
    disp = 0.0f;
    param = 0.0f;
    detail = 0.0f;
    texelOut = 0.0f;
    if (gBankU.z == 0u) return false;
    [unroll] for (uint m = 0; m < 6; ++m) {
        float eD;
        if (!BankFetch(m, worldXZ, disp, param, detail, texelOut, eD)) continue;
        // M8 THE RING CROSS-FADE (the user's "interpolation param", data/wave_scene.json
        // ringBlendTexels): near this ring's border, blend toward the next coarser ring
        // so the texture handover never pops -- the M6t fold already conserves the
        // ENERGY across rings; this interpolates the REALIZATION too.
        const float blendW = max(gBankE.x, 1.0f);
        if (m < 5u && eD < blendW) {
            float4 d2, p2, det2;
            float t2, e2;
            if (BankFetch(m + 1u, worldXZ, d2, p2, det2, t2, e2)) {
                const float w = saturate(eD / blendW);
                disp = lerp(d2, disp, w);
                param = lerp(p2, param, w);
                detail = lerp(det2, detail, w);
                texelOut = lerp(t2, texelOut, w);
            }
        }
        return true;
    }
    return false;
}

#include "Compose.hlsli"

// M6e: sample a streamed cube with the classic residency clamp -- the R8 residency-map cube
// carries (finest resident mip * 16) per base tile; clamping the LOD there means a miss
// degrades to the best RESIDENT ancestor (blur), never to unmapped garbage. (Earth's composed
// channels do the same inside Compose.hlsli; these direct forms serve Mars's native pyramids.)
float3 StreamedSample(uint texSrv, uint mapSrv, float3 dir) {
    const float want = gTexCube[texSrv].CalculateLevelOfDetail(sLinearClamp, dir);
    const float have = CsHaveCube(mapSrv, dir);   // conservative: see Compose.hlsli
    return gTexCube[texSrv].SampleLevel(sLinearClamp, dir, max(want, have)).rgb;
}
float2 StreamedSampleRg(uint texSrv, uint mapSrv, float3 dir) {
    const float want = gTexCube[texSrv].CalculateLevelOfDetail(sLinearClamp, dir);
    const float have = CsHaveCube(mapSrv, dir);
    return gTexCube[texSrv].SampleLevel(sLinearClamp, dir, max(want, have)).rg;
}

#ifndef GA_MESH_PATH
struct GlobeNode {      // mirrors GlobeLayer::NodeData
    float2 uv0;         // face-uv rect origin
    float2 uvStep;      // face-uv per grid CELL (rect size / 32)
    uint  face;
    float morphStart;   // camera distances (m) over which this LOD cross-fades to its parent
    float morphEnd;
    float pad;
};
StructuredBuffer<GlobeNode> gNodes : register(t0, space0);
#endif

static const uint kGrid = 32;
static const float kPi = 3.14159265358979f;

// Cube face -> unit sphere. Face order: +x -x +y(N pole) -y(S pole) +z -z.
float3 CubeDir(uint face, float2 uv) {
    const float2 c = uv * 2.0f - 1.0f;
    float3 p;
    if      (face == 0) p = float3(1.0f, c.y, -c.x);
    else if (face == 1) p = float3(-1.0f, c.y, c.x);
    else if (face == 2) p = float3(c.x, 1.0f, -c.y);
    else if (face == 3) p = float3(c.x, -1.0f, c.y);
    else if (face == 4) p = float3(c.x, c.y, 1.0f);
    else                p = float3(-c.x, c.y, -1.0f);
    return normalize(p);
}

// Equirect uv for a planet-frame direction -- the CLOUD VOLUME's addressing (the relief
// texture that shared it is gone; the volume bank keeps the scheme).
float2 ReliefUv(float3 dir) {
    const float lat = asin(clamp(dir.y, -1.0f, 1.0f));
    const float lon = atan2(dir.z, dir.x);
    return float2(lon / (2.0f * kPi) + 0.5f,
                  clamp(0.5f - lat / kPi, 0.5f / 4096.0f, 1.0f - 0.5f / 4096.0f));
}

// ------------------------------------------------------------------ vertex

struct VsOut {
    float4 pos : SV_Position;
    float3 rel : TEXCOORD0;    // camera-relative position
    float3 dir : TEXCOORD1;    // unit radial (the sphere normal)
    float  h   : TEXCOORD2;    // relief metres (negative = ocean floor)
};

#ifndef GA_MESH_PATH
VsOut VsMain(uint vid : SV_VertexID, uint inst : SV_InstanceID) {
    const GlobeNode nd = gNodes[inst];
    const uint quad = vid / 6u;
    const uint corner = vid % 6u;
    const uint qx = quad % kGrid;
    const uint qy = quad / kGrid;
    const uint2 kOff[6] = {uint2(0, 0), uint2(1, 0), uint2(0, 1),
                           uint2(1, 0), uint2(1, 1), uint2(0, 1)};
    float2 g = float2(qx + kOff[corner].x, qy + kOff[corner].y);

    // CDLOD morph: estimate the vertex's distance pre-morph, then slide odd vertices onto
    // their even neighbours as this LOD hands over to its parent. Both sides of every seam
    // evaluate the same per-LEVEL ramp, which is what makes it crack-free.
    const float3 dir0 = CubeDir(nd.face, nd.uv0 + g * nd.uvStep);
    const float d0 = length(CsToTangent(dir0) * gGlo.x - gCamAbs.xyz);   // M6g: one frame
    const float k = saturate((d0 - nd.morphStart) / max(nd.morphEnd - nd.morphStart, 1.0f));
    g -= frac(g * 0.5f) * 2.0f * k;

    const float3 dir = CubeDir(nd.face, nd.uv0 + g * nd.uvStep);
    // M6i: displacement from the composed height cube, both planets, one code path. The lod
    // floor of 3 keeps the displacement footprint at the ~5 km the 32x32 grids can actually
    // articulate (the deepest vertex spacing); finer height texels feed PIXEL normals instead.
    const float vlod = max(ComposedHeightLod(d0, gWavesB.z), 3.0f);
    float h = ComposedHeight(dir, vlod);

    // M6g foundation sink: inside the CUDEM window the ESTUARY mesh is this same surface at
    // 13.7 m; the globe dips a few metres under it (feathered -- continuous) so the sharp data
    // owns the depth buffer and there is no z-fight between two descriptions of one earth.
    if (gEstGeo.w > 0.5f) {
        const float latDegV = degrees(asin(clamp(dir.y, -1.0f, 1.0f)));
        const float lonDegV = degrees(atan2(dir.z, dir.x));
        const float2 wuv = float2((lonDegV - gEstGeo.x) * gEstGeo.z,
                                  (gEstGeo.y - latDegV) * abs(gEstGeo.w));
        if (all(wuv > 0.0f) && all(wuv < 1.0f)) {
            const float2 fe = smoothstep(0.0f, 0.12f, wuv) * smoothstep(1.0f, 0.88f, wuv);
            h -= 14.0f * fe.x * fe.y;
        }
    }

    VsOut o;
    o.dir = dir;   // PLANET frame: texturing (cube samples, lat/lon) stays untouched
    o.h = h;
    // The ocean surface renders AT the geoid; land rides the (altitude-scaled) exaggeration.
    // M6g: rotate the unit direction (exact in float), scale, subtract the sphere-centred
    // camera -- the same cancellation profile the planet frame had, now in ONE shared frame.
    const float3 dirT = CsToTangent(dir);
    o.rel = dirT * (gGlo.x + max(h, 0.0f) * gGlo.y) - gCamAbs.xyz;
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    return o;
}
#endif  // GA_MESH_PATH

// ------------------------------------------------------------------ pixel

float3 Hypsometric(float h, float lat) {
    // Land tints by elevation, biased toward rock/snow at high latitude. (The fallback and
    // the no-imagery look; with the color channel on, the mosaic owns the albedo.)
    const float3 shore = float3(0.62f, 0.60f, 0.42f);
    const float3 low   = float3(0.22f, 0.38f, 0.18f);
    const float3 mid   = float3(0.48f, 0.42f, 0.28f);
    const float3 high  = float3(0.52f, 0.50f, 0.48f);
    float3 c = lerp(shore, low, saturate(h / 55.0f));
    c = lerp(c, mid, saturate((h - 700.0f) / 1200.0f));
    c = lerp(c, high, saturate((h - 1900.0f) / 1400.0f));
    const float snowLine = lerp(4800.0f, 200.0f, saturate((abs(lat) - 0.72f) / 0.55f));
    c = lerp(c, float3(0.93f, 0.95f, 0.99f), saturate((h - snowLine) / 500.0f));
    return c;
}

// ---- M9: THE WATER'S QUALITY (docs/ALGEBRA.md "optics"; proofs/water_optics.py) --------------
// M7c left two constants in the ray path -- the K_d triple and the shelf/deep scatter colour.
// Both are MADE by chlorophyll, sediment and CDOM, so both are measurements. Two closed forms
// replace them, and every constant below is printed by the proof's "numbers the HLSL port must
// reproduce" block; change them there first, never here.
//
//   K_d(l) = max( Kdw(l) + M(l)*[K490 - Kdw(490)], Kdw(l) )          the spectral transfer
//   R(l)   = g * (f/mu_d) * bb(l) / K_d(l)                            the two-flux endpoint
//
// The SAME K_d serves as the absorption proxy in the second form, so extinction and colour
// cannot drift apart. The max() is load-bearing: the retrieval's valid_min (0.01) sits BELOW
// the pure-water anchor (0.0224) and blue's slope is the steepest, so blue crosses zero first.
// Water cannot be clearer than water.
static const float3 kKdw = float3(0.285f, 0.064f, 0.019f);   // pure seawater at 620/550/460 nm
static const float kKdw490 = 0.0224f;                        // ...and at the 490 nm anchor
static const float3 kMspec = float3(0.60f, 0.40f, 1.10f);    // M(490) == 1 by construction
static const float3 kBbw = float3(0.00056857f, 0.00095399f, 0.00206442f);
static const float3 kBbpSpec = float3(0.915211f, 1.007266f, 1.162059f);   // (555/lambda)^0.80
static const float kFoverMu = 0.44f;                         // f = 0.33, mu_d = 0.75
static const float kBetaSpm = 0.010f;                        // m^-1 per mg/L
static const float kBetaChl = 0.0038f;                       // m^-1 per (mg/m^3)^0.63

struct WaterOptics {
    float3 kd;      // per-channel diffuse attenuation, m^-1
    float3 deep;    // the deep-water albedo endpoint (< 0 in x = "use the M7c constants")
    float ice;      // sea-ice concentration, 0-1
};

WaterOptics SampleWaterOptics(float latDeg, float lonDeg) {
    WaterOptics o;
    o.kd = float3(0.36f, 0.105f, 0.06f);   // M7c, byte for byte, when the plane is absent
    o.deep = float3(-1.0f, 0.0f, 0.0f);
    o.ice = 0.0f;
    // Sea ice rides the WAVE grid (node-centred, 0..360) -- the same mapping the Hs fetch uses.
    if (gOptU.y != 0xFFFFFFFFu) {
        const float2 iuv = float2(
            frac((lonDeg - gWavesA.y) * gWavesA.w / gWavesB.x),
            saturate(((gWavesA.x - latDeg) * gWavesA.z + 0.5f) / gWavesB.y));
        o.ice = saturate(gTex[gOptU.y].SampleLevel(sLinearClamp, iuv, 0).x);
    }
    if (gOptU.z == 0u) return o;
    // Ocean colour is its OWN grid: cell-centred, 1440x720, on -180..180 -- not the wave grid
    // shifted. Half-texel centres on BOTH axes here (the wave fetch above omits it on lon; that
    // is a pre-existing 0.125 deg offset, left alone rather than silently changed under it).
    float dLon = lonDeg - gOptA.y;
    dLon -= 360.0f * floor(dLon / 360.0f);
    const float2 ouv = float2(
        frac((dLon * gOptA.w + 0.5f) / gOptB.x),
        saturate(((latDeg - gOptA.x) * gOptA.z + 0.5f) / gOptB.y));
    // .w is the retrieval mask (0 where polar night or the gap fill stops). The texel already
    // carries the PURE-WATER limit there, so the hardware lerp degrades toward clear water at
    // the data's edge; the mask is kept for the coverage lens and the climatology rung.
    const float4 oc = gTex[gOptU.x].SampleLevel(sLinearWrap, ouv, 0);
    const float chl = pow(10.0f, oc.x);        // stored as log10: the FILTER wants the
    const float spm = pow(10.0f, oc.z);        // geometric mean across a coastal front
    o.kd = max(kKdw + kMspec * (oc.y - kKdw490), kKdw);
    // Backscatter arbitrates two retrievals by AUTHORITY -- coastal water is SPM-carried, open
    // ocean is chlorophyll-carried, and whichever holds signal wins. The compositor's idiom.
    const float3 bb =
        kBbw + max(kBetaSpm * spm, kBetaChl * pow(max(chl, 1e-4f), 0.63f)) * kBbpSpec;
    o.deep = gOptB.z * kFoverMu * bb / o.kd;
    return o;
}

float4 PsMain(VsOut i) : SV_Target {
    const float3 up = normalize(i.dir);   // PLANET frame: lat/lon + every texture fetch
    const float3 v = normalize(-i.rel);   // TANGENT frame: geometry + lighting (M6g)
    const float lat = asin(clamp(up.y, -1.0f, 1.0f));
    const float lonDeg = degrees(atan2(up.z, up.x));

    // M7o: THE SLICE PLANE -- the algebra-first demo node (proofs/slice_plane.py,
    // gatest-pinned). The cut is the plane pi = (north, d); a pixel lives on the kept
    // side iff the signed distance s = P.n - d <= 0 -- one inner product, one discard.
    // The exposed silhouette along the cut IS the terrain/bathymetry profile: verify
    // against --bathy-map.
    if (gBankA.w > 0.5f) {
        if ((CsToTangent(up) * gGlo.x).z > gBankC.w) discard;
    }

    // M6g: lighting happens in the tangent frame so the globe shares the sea's ONE sun.
    const float3 upT = CsToTangent(up);
    const float3 axisT = float3(gCsR0.y, gCsR1.y, gCsR2.y);   // planet north pole
    float3 east = cross(axisT, upT);
    east = (dot(east, east) < 1e-8f) ? float3(1.0f, 0.0f, 0.0f) : normalize(east);
    const float3 north = cross(upT, east);
    const float day = saturate(dot(gSunDir.xyz, upT) * 3.0f + 0.12f);

    const float lod = ComposedHeightLod(length(i.rel), gWavesB.z);
    // M6i: land/sea CLASSIFICATION resamples the height channel PER PIXEL at screen lod. The
    // vertex height (i.h) is footprint-floored for displacement (~5 km) and foundation-sunk
    // in the estuary window -- gating on it smeared whole towns below sea level. hp is what
    // the data says HERE, at this pixel's own resolution.
    const float hp = ComposedHeightOn() ? ComposedHeight(up, lod) : i.h;
    // M6i/M6j/M6n: classification by SURVEY far afield; inside the window the LIVE waterline
    // decides through an ANALOG shore band (ComposedLandness) -- the binary cut flickered
    // tile-shaped speckle over the flats whenever the height data's resident level changed
    // mid-stream. Both shading sides are evaluated and MIXED by landness: a half-emerged flat
    // is half wet, in color, in glint, and (in the mesh shader) in geometry.
    // The classifier's INPUT is residency-stable: a fixed ~38 m level (window mip 2, fully
    // warmed) rather than "finest resident" -- neighbouring tiles streaming at different
    // depths were reading heights that disagreed by more than the shore band and cutting
    // hard seams. Fine data still drives shading; only the land/water QUESTION reads the
    // stable level.
    // M7f: classification input sharpens one rung, 38 m -> 19 m (window mip 1): the fine
    // edit mask owns the structures now, so the height-driven shoreline can afford the
    // finer level -- half the staircase, same residency-stable contract.
    const float hpC = ComposedHeightOn() ? ComposedHeight(up, max(lod, -5.0f)) : i.h;
    const float landness =
        (gStreamF.z > 0.5f) ? ((hp > 0.0f) ? 1.0f : 0.0f)
                            : ComposedLandness(up, hpC, gWavesB.w);
    float3 n = upT;
    float3 alb;
    float spec = 0.0f;
    float3 skyReflAdd = 0.0f;   // M7c: Fresnel-weighted sky on the reflected ray (water only)
    if (gStreamF.z > 0.5f) {
        // MARS: the rescued sample pyramids ARE the planet -- residency-clamped diffuse +
        // BC5 surface normals in the local ENU frame.
        alb = StreamedSample(gStreamU.x, gStreamU.z, up);   // M6j: BC1_SRGB hardware decode,
                                                            // no lift -- pixels as authored
        if (gStreamF.y > 0.5f) {
            const float2 nxy = StreamedSampleRg(gStreamU.y, gStreamU.w, up);   // SNORM -1..1
            n = normalize(upT + east * nxy.x * 1.2f + north * nxy.y * 1.2f);
        }
    } else {
        // LAND side: normals from the composed height cube -- one gradient, no equirect/NE
        // fork, no pole singularity. Modest slope gain (the vertical exaggeration is a
        // display choice; shading at x25 would posterize the continents).
        const float2 gr = ComposedHeightGrad(up, lod);
        const float kSlopeGain = 4.0f;
        const float3 nLand =
            normalize(upT - east * (gr.x * kSlopeGain) - north * (gr.y * kSlopeGain));
        const float3 albLand = Hypsometric(max(hp, 0.0f), lat);

        // SEA side: depth tints the shelves; GFS-Wave whitens the storms.
        // M7e: the reference photos say the green band is NARROW -- open water reads navy
        // just past the bar, and the emerald belongs to the shallows alone.
        // M9: the scatter endpoint is now a MEASUREMENT (ALGEBRA "optics"). The depth lerp it
        // replaces was double-counting bed brightness that lerp(albSea, bedAlb, Tw) already
        // supplies below -- with real optics the bed carries the shallows and the water carries
        // its own colour. The M7c pair survives verbatim on the flag-off branch.
        const WaterOptics wq = SampleWaterOptics(degrees(lat), lonDeg);
        const float shelf = saturate(1.0f + hp / 45.0f);        // 1 at the beach, 0 by -45 m
        float3 albSea = wq.deep.x >= 0.0f
                            ? wq.deep
                            : lerp(float3(0.008f, 0.030f, 0.080f), float3(0.055f, 0.28f, 0.31f),
                                   shelf * shelf);
        float hs = 0.0f, wind = 6.0f;
        if (gTexIdx.y != 0xFFFFFFFFu) {
            const float2 wuv = float2(
                frac((lonDeg - gWavesA.y) * gWavesA.w / gWavesB.x),
                saturate(((gWavesA.x - degrees(lat)) * gWavesA.z + 0.5f) / gWavesB.y));
            const float hsS = gTex[gTexIdx.y].SampleLevel(sLinearClamp, wuv, 0).x;
            if (hsS >= 0.0f) hs = hsS;                          // -1 = land in the wave grid
            if (gTexIdx.z != 0xFFFFFFFFu) {
                const float w = gTex[gTexIdx.z].SampleLevel(sLinearClamp, wuv, 0).x;
                if (w >= 0.0f) wind = w;
            }
        }
        // Dupuy-Bruneton in spirit: unresolved whitecap coverage brightens the storm belts.
        albSea = lerp(albSea, float3(0.55f, 0.62f, 0.68f), saturate((hs - 2.5f) / 9.0f) * 0.55f);

        // Cox-Munk: slope variance from wind speed; the glint lobe IS the far-field BRDF.
        // M7: inside the wave bank's rings the sigma^2 comes from the BANK (the M6t fold's
        // shed variance, locally sea-state true), and its foam whitens the water -- the
        // shading now reads the same tiled resource the geometry displaces from.
        // M9: ice damps the capillary-gravity waves the Cox-Munk lobe is MADE of, so the glint
        // dies with the same field that whitens the surface below -- one number, both terms.
        float s2 = (0.003f + 0.00512f * wind) * (1.0f - 0.95f * wq.ice);
        float3 nWater = upT;    // band-limited to the PIXEL (ring slopes + cascade detail,
                                // each band under its per-axis Gaussian prefilter): feeds
                                // the glint AND the Fresnel split / both rays. Fresnel on
                                // the ring-texel normal rendered a 4.8 m-blurred sea --
                                // molten "Vaseline" at helm height (seen, fixed M8g); the
                                // prefilter law makes this normal safe at every footprint,
                                // because gAx/gAy fold each band away as the pixel stops
                                // resolving it (sub-pixel energy stays in sigma^2).
        float3 nSmooth = upT;   // band-limited to the ring texel: feeds DIFFUSE only --
                                // per-chop diffuse rendered 2 m wavelets as dark flecks
                                // from altitude (the leopard, rolled back by the user)
        float lvlW = 0.0f;    // live water level here (bank: tide + solver); 0 = geoid far afield
        float foamW = 0.0f;   // whitening accumulator -- painted AFTER the refraction mix so
                              // foam rides ON the water, not under it
        const float footPx = length(i.rel) * gWavesB.z;
        // M8 ripple prefilter (ALGEBRA.md ripple; proofs/ripple_prefilter.py): the pixel
        // footprint's world-space edges, computed OUTSIDE the bank branch so the screen
        // derivatives ride uniform control flow. The footprint is a FRAME {fpx, fpz};
        // its per-axis Gaussian at a band's wavenumber is the exact expected attenuation
        // -- the anisotropy no scalar footPx can express (a grazing sliver resolves
        // across-view ripples while along-view ones alias into crawling shimmer).
        const float2 fpxW = ddx((CsToTangent(up) * gGlo.x).xz);
        const float2 fpzW = ddy((CsToTangent(up) * gGlo.x).xz);
        float3 bankGains = 0.0f;   // per-band sea-state gains (x=b0, y=b1, z=b2) for the
        float bankEta = 0.0f;      // caustic assembly + peak shaping below; hoisted out
        bool bankOn = false;       // of the bank scope
        {
            float4 bD, bP, bDet, bDx, bDz, tmp, tmp2;
            float bT, t2u;
            const float2 wxz = (CsToTangent(up) * gGlo.x).xz;
            if (BankSample(wxz, bD, bP, bDet, bT)) {
                bankOn = true;
                bankGains = float3(bDet.z, bDet.x, bDet.w);
                bankEta = bD.y;
                s2 = max(bP.y, 0.0015f);
                lvlW = bP.x;
                foamW = saturate(bD.w);   // M8: the bank's foam is already disciplined
                                          // (crest-gated union, noise-broken, memory-max)
                // Per-pixel wave normals, finite-differenced from the bank at its own ring
                // texel: geometry alone is nearly invisible from above -- the normal is what
                // makes the sea READ (the M6t glint then rides real wave faces).
                BankSample(wxz + float2(bT, 0.0f), bDx, tmp, tmp2, t2u);
                BankSample(wxz + float2(0.0f, bT), bDz, tmp, tmp2, t2u);
                float sx = (bDx.y - bD.y) / bT;
                float sz = (bDz.y - bD.y) / bT;
                // Shoaled slopes can exceed any real wave face; cap at 1.1 (the Sea.hlsl
                // guard) or over-steep facets render as dark back-face speckle and pin
                // Fresnel at its ceiling (grey plateaus -- the two-sided failure mode).
                const float slS = length(float2(sx, sz));
                if (slS > 1.1f) { sx *= 1.1f / slS; sz *= 1.1f / slS; }
                nSmooth = normalize(upT - east * sx - north * sz);
                // M7a: THE SPARKLE -- bands this PIXEL resolves but the ring texel does not,
                // read straight from the cascade DERIVATIVE textures at full FFT resolution,
                // weighted by the fold difference (wPix - wRing) and the tile's local sea
                // state (detail plane: hsScale, dry). Sigma^2 hands the same energy back, so
                // the three tiers stay telescoped; at altitude wPix <= wRing and every term
                // vanishes -- the far field is untouched.
                float env = 0.0f;
                [unroll] for (uint c = 0; c < 3; ++c) {
                    const float lam = 6.2831853f / gBankC[c];
                    const float wRing = 1.0f - smoothstep(lam * 0.12f, lam * 0.5f, bT);
                    const float wPix = 1.0f - smoothstep(lam * 0.12f, lam * 0.5f, footPx);
                    const float wDet = saturate(wPix - wRing) * bDet.y;
                    if (wPix <= 0.01f) continue;
                    const float2 duv = wxz / gBankB[c];
                    const float4 dv =
                        gTex[gBankU2[c + 1u]].SampleLevel(sLinearWrap, duv, 0);
                    // M7e: THE GROUPS. From altitude the eye never sees the wave -- it sees
                    // the ENVELOPE: groups, streaks, glitter grain. The fold's flat sigma^2
                    // erased that spatial variance (the "lame" aerial water). The local
                    // slope magnitude, weighted by what the PIXEL resolves, restores it --
                    // and it telescopes: by ~10 km footprints wPix folds it away again.
                    // Per-axis Gaussian prefilter at this band's wavenumber: the x-slope
                    // channel carries content along x-hat, the y-slope along y-hat; each
                    // attenuates by exp(-|J^T k|^2/8) in ITS direction (sigma = half the
                    // footprint extent). Under-recovered energy stays in sigma^2 below --
                    // shed, never aliased. Isotropic-max here erred x3000 at grazing.
                    const float kC = gBankC[c];
                    const float gAx = exp(-0.125f * kC * kC *
                                          (fpxW.x * fpxW.x + fpzW.x * fpzW.x));
                    const float gAy = exp(-0.125f * kC * kC *
                                          (fpxW.y * fpxW.y + fpzW.y * fpzW.y));
                    env += length(dv.xy) * wPix * (0.5f * (gAx + gAy));
                    if (wDet <= 0.002f) continue;
                    sx += dv.x * wDet * bDet.x * gBankB.w * gAx;
                    sz += dv.y * wDet * bDet.x * gBankB.w * gAy;
                    foamW = max(foamW, saturate(dv.w * wDet) * saturate(bDet.x) *
                                           max(gAx, gAy) * gBankE.y *
                                           (c == 2 ? 0.30f : 0.15f));
                    s2 = max(s2 - wDet * bDet.x * bDet.x *
                                      (0.5f * (gAx * gAx + gAy * gAy)) *
                                      (c == 0 ? 0.0004f : (c == 1 ? 0.0018f : 0.0060f)),
                             0.0015f);
                }
                const float envN = saturate(env * bDet.x * 4.0f);
                s2 = max(s2 * (0.70f + 0.60f * envN), 0.0015f);
                albSea = lerp(albSea, float3(0.52f, 0.58f, 0.60f), envN * envN * 0.08f);
                const float slW = length(float2(sx, sz));
                if (slW > 1.1f) { sx *= 1.1f / slW; sz *= 1.1f / slW; }
                nWater = normalize(upT - east * sx - north * sz);
            }
        }
        // ---- M7c: THE TWO RAYS. A water pixel is a Fresnel split between two rays, and
        // both are answered by data this atlas already realizes -- ray tracing against our
        // own quadtrees, no BLAS, no second scene description. The REFLECTED ray asks the
        // analytic sky (below, on the true wave normal: the sandwich r = -n d n). The
        // REFRACTED ray bends by Snell -- a ROTOR in the incidence bivector (d ^ n); the
        // closed form below IS R d ~R expanded -- then marches into the water and lands on
        // the BED: the composed height quadtree, found by secant cast, wearing the composed
        // IMAGERY as its albedo. Beer-Lambert attenuates per channel over the REAL path
        // (down along the ray + diffuse up), so deep water collapses to the shelf scatter
        // color the globe always drew and the far field stays converged, while shallow
        // water shows the bar through the surface -- scaled by the LIVE tide, because
        // depth = level - bed at this pixel. Space and helm are the same formula; the
        // tiers telescope by physics, not by altitude branches.
        const float depthW = max(lvlW - hp, 0.0f);
        const float3 dIn = -v;                                   // camera -> surface
        const float ci = saturate(-dot(dIn, nWater));
        const float etaR = 1.0f / 1.34f;
        const float st2 = etaR * etaR * max(1.0f - ci * ci, 0.0f);
        const float3 tDir = normalize(
            etaR * dIn + (etaR * ci - sqrt(max(1.0f - st2, 0.0f))) * nWater);
        float sDown = depthW;    // vertical closed form: exact where parallax is subpixel
        float3 bedDir = up;
        if (footPx < 30.0f && depthW > 0.01f && depthW < 90.0f) {
            // The cast: 2 secant steps against ComposedHeight. Under 30 m footprints the
            // march matters (looking through a wave face shifts the bar); past that the
            // refracted hit is the pixel's own bed and the closed form takes over -- the
            // two agree where they meet, so there is no seam to hide.
            const float3 Pw = gCamAbs.xyz + i.rel;
            const float muD = max(-dot(tDir, upT), 0.10f);
            float sP = depthW / muD;
            [unroll] for (int itr = 0; itr < 2; ++itr) {
                const float3 Pb = Pw + tDir * sP;
                const float gap =
                    (length(Pb) - gGlo.x) - ComposedHeight(CsToPlanet(normalize(Pb)), lod);
                sP = clamp(sP + gap / muD, 0.3f, 140.0f);
            }
            bedDir = CsToPlanet(normalize(Pw + tDir * sP));
            sDown = sP;
        }
        // Per-channel diffuse attenuation. M7c shipped one coastal constant for the whole
        // planet, with BLUE penetrating deepest -- the open-ocean ordering, which is why the
        // shoals read turquoise everywhere. M9 reads it from the retrieval instead: open ocean
        // keeps that ordering, but past K490* = 0.0823 /m the crossover flips and GREEN becomes
        // the deepest-penetrating channel, which is what makes coastal water green. One field,
        // one crossover, no second tuning (priors ledger 13).
        const float3 Kd = wq.kd;
        const float3 Tw = exp(-Kd * (sDown + depthW));
        float3 bedAlb = (ComposedColorOn() && gStreamF.z < 0.5f)
                            ? ComposedColor(bedDir)
                            : float3(0.44f, 0.40f, 0.31f);
        // ---- M8 CAUSTICS (ALGEBRA.md caustics; proofs/caustic_jacobian.py). Sunlight
        // refracting at the wave surface converges on the bed; the gain is the inverse
        // ray-map Jacobian in its PHYSICAL form, gain = 1/(1 + h K lap_phys) with the
        // crest identity lap_phys = lap/areaJac^2 and K = 1 - 1/n ~ 0.25 -- adjudicated
        // by 2^22-ray ground truth (corr 0.999; the reference's shipped sign renders
        // troughs bright). areaJac and lap assemble from the cascade derivative fibers
        // (J in dv.z; lap = FD of the slope channels at cascade resolution), scaled by
        // the detail plane's per-band gains, and sampled where the SUN entered the water
        // (bed - sunRun): sampling at the view entry pins the pattern to the camera.
        // The sun's 0.53-degree disc blurs the pattern by ~h*0.0093, so it fades by 20 m
        // (smoothstep 4..20) and clamps [0.35, 2.6] -- closures, pinned by the proof.
        if (bankOn && footPx < 30.0f && depthW > 0.05f && depthW < 20.0f &&
            gSunDir.y > 0.02f) {
            const float2 wxzC = (CsToTangent(up) * gGlo.x).xz;
            const float2 bedXZ =
                wxzC + float2(dot(tDir, east), dot(tDir, north)) * sDown;
            const float3 sunT = refract(normalize(-gSunDir.xyz), upT, etaR);
            const float sunDn = max(-dot(sunT, upT), 0.15f);
            const float3 sunH = sunT + sunDn * upT;
            const float2 causticXZ =
                bedXZ - float2(dot(sunH, east), dot(sunH, north)) / sunDn * depthW;
            float ajF = 1.0f;
            float lapF = 0.0f;
            [unroll] for (uint cc = 0; cc < 3; ++cc) {
                const float lamC = 6.2831853f / gBankC[cc];
                const float wPixC = 1.0f - smoothstep(lamC * 0.12f, lamC * 0.5f, footPx);
                if (wPixC <= 0.01f) continue;
                const float ampC = (cc == 0) ? bankGains.x
                                             : ((cc == 1) ? bankGains.y : bankGains.z);
                const float2 duvC = causticXZ / gBankB[cc];
                const float du1 = 1.0f / 256.0f;                  // one cascade texel
                const float texM = gBankB[cc] * du1;              // its metres
                const float4 dc =
                    gTex[gBankU2[cc + 1u]].SampleLevel(sLinearWrap, duvC, 0);
                const float hxp = gTex[gBankU2[cc + 1u]]
                                      .SampleLevel(sLinearWrap, duvC + float2(du1, 0), 0).x;
                const float hxm = gTex[gBankU2[cc + 1u]]
                                      .SampleLevel(sLinearWrap, duvC - float2(du1, 0), 0).x;
                const float hzp = gTex[gBankU2[cc + 1u]]
                                      .SampleLevel(sLinearWrap, duvC + float2(0, du1), 0).y;
                const float hzm = gTex[gBankU2[cc + 1u]]
                                      .SampleLevel(sLinearWrap, duvC - float2(0, du1), 0).y;
                const float lapC =
                    (hxp - hxm + hzp - hzm) / (2.0f * texM);
                ajF += wPixC * ampC * (dc.z - 1.0f);
                lapF += wPixC * ampC * lapC;
            }
            const float ajC = max(ajF, 0.05f);
            const float lapPhys = lapF * gBankB.w / (ajC * ajC);
            float cg = 1.0f / max(1.0f + depthW * 0.25f * lapPhys, 0.05f);
            cg = lerp(1.0f, cg, 1.0f - smoothstep(4.0f, 20.0f, depthW));
            cg = clamp(cg, 0.60f, 1.8f);
            cg = lerp(1.0f, cg, gBankE.z);   // scene causticStrength
            bedAlb *= lerp(1.0f, cg, saturate(gSunDir.y * 3.0f));
        }
        albSea = lerp(albSea, bedAlb, Tw);
        // M9: sea ice sits ON the water -- after every water term, before foam (foam on top of
        // ice is still foam). The concentration is the coverage fraction, so the lerp IS the
        // physics; no threshold, no ice "texture" the data does not contain.
        albSea = lerp(albSea, float3(0.78f, 0.82f, 0.85f), saturate(wq.ice));

        const float3 hv = normalize(v + gSunDir.xyz);
        const float ch = saturate(dot(hv, nWater));
        const float t2 = max(1.0f - ch * ch, 0.0f) / max(ch * ch, 1e-4f);
        spec = exp(-t2 / s2) / (4.0f * kPi * s2 * max(ch * ch * ch * ch, 1e-4f));
        const float fres = 0.02f + 0.98f * pow(1.0f - saturate(dot(v, hv)), 5.0f);
        spec *= fres * saturate(dot(gSunDir.xyz, nWater));

        // The reflected ray: sky radiance on the sandwich -n d n, Fresnel-weighted on the
        // TRUE wave normal; the refracted side scales by 1-F so energy splits, not doubles.
        // From straight above F ~ 0.02 (space view untouched); toward the horizon the sea
        // mirrors the sky, which is the term every previous tuning pass was missing.
        const float fresN = 0.02f + 0.98f * pow(1.0f - saturate(dot(v, nWater)), 5.0f);
        float3 rDir = normalize(dIn - 2.0f * dot(dIn, nWater) * nWater);
        // A front-facing facet on the back of a steep wave reflects BELOW the horizon;
        // the sky model correctly darkens there, which painted grey patches over wave
        // backs. What such a facet actually sees is more sea and the sky above it --
        // clamp the ray to the horizon (the Sea.hlsl refl.y guard, tangent-frame form).
        const float rUp = dot(rDir, upT);
        if (rUp < 0.02f) rDir = normalize(rDir + (0.02f - rUp) * upT);
        skyReflAdd = SkyRadianceDirDiscless(rDir) * (fresN * 0.9f * (1.0f - landness));
        albSea *= 1.0f - fresN;

        // M8 foamlaw: foam rides ON the water -- painted AFTER the Fresnel split (it no
        // longer dims toward the horizon) at the scene's peak opacity over water AND
        // reflection: whitewater is a thin aerated layer, not paint. The kernel shipped
        // PURE physics coverage; the per-pixel range-faded breakup textures it here
        // (strength 0.55 + 0.75 n: a triggered crest always shows SOME foam; noise
        // modulates amount, never existence -- 30.7% gouache -> 2.1%, the reference's
        // own measurement).
        const float fnB =
            FoamBreakup((CsToTangent(up) * gGlo.x).xz, length(i.rel));
        const float foamOp =
            saturate(saturate(foamW) * (0.55f + 0.75f * fnB)) * gBankD.w;
        albSea = lerp(albSea, float3(0.945f, 0.965f, 0.975f), foamOp);
        skyReflAdd *= 1.0f - foamOp;

        // M8: peak-normalized shaping -- crests catch more light than troughs simply by
        // being nearer the sky. tanh never saturates, so no flat-bottomed troughs; the
        // peak proxy is kappa*envRms (kappa 2.8, the Gaussian expected-max closure).
        if (bankOn) {
            const float envR = max(
                sqrt(dot(bankGains * bankGains, gBankD.xyz * gBankD.xyz)), 1e-4f);
            const float shape = tanh(1.6f * bankEta / max(2.8f * envR, 1e-3f));
            albSea *= 1.0f + 0.09f * shape;
        }

        // The analog mix: glint dies as the flat emerges, the land normal takes over.
        // M8 the sparkle normal feeds ONLY the glint (the reference law, one step
        // further than M7c): per-chop diffuse shading on nWater rendered every 2 m
        // wavelet as a dark fleck from altitude -- the leopard the user rolled back.
        // The body of the water shades on the band-limited normal; glitter is the
        // microfacet lobe's job.
        n = normalize(lerp(nSmooth, nLand, landness));
        alb = lerp(albSea, albLand, landness);
        spec *= 1.0f - landness;
    }

    // ---- M6i/M7c: the composed color channel. Land takes the imagery outright; the water
    // side already carries it in through the REFRACTED ray (bed albedo, attenuated by the
    // real underwater path) -- the old fixed 0.6 shallow blend is retired.
    if (ComposedColorOn() && gStreamF.z < 0.5f) {
        const float3 img = ComposedColor(up);
        alb = lerp(alb, img, landness);
        // M7e: a surveyed STRUCTURE wears rock, not the photo under it -- beneath a jetty
        // footprint the imagery is a smear of foam and water, which rendered the jetties as
        // grey blobs. Boulder-scale hash grain, cell size folded to the pixel footprint so
        // the rock never shimmers from altitude.
        const float elp = ComposedEditLand(up);
        if (elp > 0.01f && landness > 0.0f) {
            const float2 rxz = (CsToTangent(up) * gGlo.x).xz;
            const float cellM = max(0.7f, length(i.rel) * gWavesB.z);
            const float2 rc = floor(rxz / cellM);
            const float rn = frac(sin(dot(rc, float2(127.1f, 311.7f))) * 43758.5453f);
            const float3 rock =
                lerp(float3(0.15f, 0.14f, 0.13f), float3(0.33f, 0.30f, 0.26f), rn);
            const float rockW = saturate(elp * 1.5f) * landness;
            alb = lerp(alb, rock, rockW);
            // M8g: riprap FACETS. Albedo grain alone still lit as one continuous grease
            // (the Vaseline jetty) -- a boulder pile's signature is per-block NORMALS.
            // Each hash cell gets a fixed random tilt; amplitude folds away as the pixel
            // footprint approaches the boulder size (same law as every detail band: fold,
            // never alias), so from altitude the jetty relaxes to the smooth ridge.
            const float fp = length(i.rel) * gWavesB.z;
            const float facetW = rockW * (1.0f - smoothstep(0.35f, 1.4f, fp));
            if (facetW > 0.01f) {
                const float fx = frac(sin(dot(rc, float2(269.5f, 183.3f))) * 43758.5453f);
                const float fz = frac(sin(dot(rc, float2(419.2f, 371.9f))) * 43758.5453f);
                n = normalize(n + (east * (fx - 0.5f) + north * (fz - 0.5f)) *
                                      (1.1f * facetW));
            }
        }
    }

    // ---- M6j --albedo: the TEXTURE-WORK lens. Raw composed color (or Mars's raw pyramid) --
    // no lighting, no atmosphere, no materials, no clouds, and NOT the shallow-water mix
    // either (classification is presentation too; the lens must show what the TEXTURES say).
    // The stencil still draws. Everything below this line is presentation.
    if (gTexIdx2.w != 0u) {
        const float3 lens =
            (ComposedColorOn() && gStreamF.z < 0.5f) ? ComposedColor(up) : alb;
        return float4(ApplyComposedStencil(lens, up), 1.0f);
    }

    // ---- M7m: THE SANITY LENSES -- values as color, so a domain error is a broken
    // pattern instead of an argument. worldxz: a 100 m world checker (any frame/scale
    // error shows as a seam across LOD or ring boundaries). winuv: the window uv gradient
    // (its v is SOUTH -- the gradient direction proves it). mip: the height window's
    // residency heat. ring: the bank's rings with a 4-texel checker (bank addressing on
    // screen). These test the GA the cheap way: patterns survive correct products.
    if (gBankA.z > 0.5f) {
        const float2 wxzL = (CsToTangent(up) * gGlo.x).xz;
        const int lensId = (int)(gBankA.z + 0.5f);
        float3 lc = float3(0.05f, 0.05f, 0.08f);
        if (lensId == 1) {
            const float chk = fmod(floor(wxzL.x / 100.0f) + floor(wxzL.y / 100.0f) +
                                       200000.0f,
                                   2.0f);
            lc = float3(frac(wxzL / 100.0f) * 0.75f + 0.1f, chk * 0.7f);
        } else if (lensId == 2) {
            const float2 duvL = CsWindowUv(up);
            if (all(duvL > 0.0f) && all(duvL < 1.0f)) lc = float3(duvL, 0.0f);
        } else if (lensId == 3) {
            const float2 duvL = CsWindowUv(up);
            if (gCsU2.w != 0xFFFFFFFFu && all(duvL > 0.0f) && all(duvL < 1.0f)) {
                const float mL = CsHave2D(gCsU2.w, duvL);
                lc = lerp(float3(0.1f, 0.85f, 0.25f), float3(0.9f, 0.12f, 0.1f),
                          saturate(mL / 7.0f));
            }
        } else if (lensId == 6) {
            // M7p: WATER AS DATA -- flat, unlit, comparable 1:1 with the 2D proof figure
            // proofs/inlet_storm.png: R = band-1 amplitude gain (detail.x, /2),
            // G = foam, B = current speed (/2.5 m/s).
            float4 bDL6, bPL6, bDetL6;
            float bTL6;
            if (BankSample(wxzL, bDL6, bPL6, bDetL6, bTL6)) {
                lc = float3(saturate(bDetL6.x * 0.5f), saturate(bDL6.w),
                            saturate(length(bPL6.zw) * 0.4f));
            }
        } else if (lensId == 5) {
            // M7n: cascade coincidence -- red = the BANK's foam (the kernel's card, if
            // --inject cascade is on), green = the SAME card from the PS's own cascade-1
            // mapping. Yellow everywhere = the two consumers of the wave tiles agree on
            // the domain; fringes = a flip/offset/scale between them.
            float4 bDL, bPL, bDetL;
            float bTL;
            const float2 fL = frac(wxzL / gBankB[1]);
            float cardL = 0.20f + 0.30f * step(0.5f, fL.x) + 0.40f * step(0.5f, fL.y);
            if (any(fL < 0.03f) || any(fL > 0.97f)) cardL = 1.0f;
            if (BankSample(wxzL, bDL, bPL, bDetL, bTL)) {
                lc = float3(saturate(bDL.w), cardL, 0.0f);
            } else {
                lc = float3(0.0f, cardL * 0.3f, 0.15f);
            }
        } else if (lensId == 4) {
            [unroll] for (uint mR = 0u; mR < 6u; ++mR) {
                const float texelL = gBankA.x * float(1u << mR);
                const float2 orgL =
                    (mR == 0u)   ? gBankOrg01.xy
                    : (mR == 1u) ? gBankOrg01.zw
                    : (mR == 2u) ? gBankOrg23.xy
                    : (mR == 3u) ? gBankOrg23.zw
                    : (mR == 4u) ? gBankOrg45.xy
                                 : gBankOrg45.zw;
                const float2 localL = (wxzL - orgL) / texelL;
                if (all(localL >= 1.0f) && all(localL < 511.0f)) {
                    const float chk =
                        fmod(floor(localL.x / 4.0f) + floor(localL.y / 4.0f), 2.0f);
                    lc = lerp(float3(0.15f, 0.35f, 1.0f), float3(1.0f, 0.75f, 0.15f),
                              float(mR) / 5.0f) *
                         (0.45f + 0.55f * chk);
                    break;
                }
            }
        }
        return float4(lc, 1.0f);
    }

    // ---- M6c: the live 3D sky. One column sample shades the ground under weather...
    float overhead = 0.0f;
    if (gTexIdx.w != 0xFFFFFFFFu) {
        const float2 cuv = ReliefUv(up);
        overhead = gTex3D[gTexIdx.w].SampleLevel(sLinearClamp, float3(cuv, 0.18f), 0).x +
                   gTex3D[gTexIdx.w].SampleLevel(sLinearClamp, float3(cuv, 0.55f), 0).x;
        overhead = saturate(overhead * gCloudA.w);
    }

    const float ndl = saturate(dot(n, gSunDir.xyz)) * (1.0f - 0.75f * overhead);
    float3 col = alb * (0.030f + ndl * SUN_IRR_C * 1.15f);
    col += spec * SUN_IRR_C * 0.85f * (1.0f - overhead);
    col += skyReflAdd * day * (1.0f - 0.6f * overhead);   // M7c: the reflected ray, skyward
    col += alb * float3(0.010f, 0.014f, 0.028f) * (1.0f - day);   // moonlit-blue night side

    // ---- M6j: the CLOSE-UP material model, now in the planet shader -- wet sand at the LIVE
    // waterline, dunes, riprap on steep rock, lit the way the terrain layer lit them (sun +
    // sky ambient + near haze). This block is what let TerrainLayer retire as a renderer:
    // physics that a mosaic cannot know owns the last few hundred metres, the mosaic owns the
    // aerial, and the crossfade between them is the ONE distance ramp.
    const float distC = length(i.rel);
    if (gStreamF.z < 0.5f && landness > 0.0f && distC < 2700.0f && gCsU2.z != 0xFFFFFFFFu) {
        const float2 wuv = CsWindowUv(up);
        if (all(wuv > 0.0f) && all(wuv < 1.0f)) {
            const float2 grF = ComposedHeightGrad(up, -8.0f);   // true slope, finest resident
            const float slope = length(grF);
            const float3 nM = normalize(upT - east * grF.x - north * grF.y);
            const float water = gWavesB.w;
            float3 matAlb;
            if (hp - water < 0.35f) matAlb = float3(0.38f, 0.34f, 0.27f);   // wet sand band
            else if (hp < 3.6f) matAlb = float3(0.70f, 0.64f, 0.50f);       // beach and flats
            else matAlb = float3(0.28f, 0.37f, 0.20f);                      // dune grass
            if (slope > 0.42f && hp > water - 1.5f) {
                matAlb = lerp(matAlb, float3(0.36f, 0.35f, 0.34f),
                              saturate((slope - 0.42f) * 3.0f));            // riprap
            }
            // M7e: a surveyed structure is DARK rock at every distance -- the near-material
            // pale riprap was overriding the edit-land boulder paint inside 2.7 km.
            const float elm = ComposedEditLand(up);
            if (elm > 0.01f) {
                const float2 rxz2 = (CsToTangent(up) * gGlo.x).xz;
                const float cell2 = max(0.7f, distC * gWavesB.z);
                const float rn2 =
                    frac(sin(dot(floor(rxz2 / cell2), float2(127.1f, 311.7f))) * 43758.5453f);
                matAlb = lerp(matAlb,
                              lerp(float3(0.15f, 0.14f, 0.13f), float3(0.33f, 0.30f, 0.26f),
                                   rn2),
                              saturate(elm * 1.5f));
            }
            // M7g: the generic beach/dune constants were painted when the near field was
            // a 9.5 m blur; with the z17 rung resident the ORTHO is the better beach. The
            // tide's wet band and the surveyed rock keep full authority -- they know what
            // no photo can (the live waterline, the edit polygons).
            const float imgTexM = ComposedColorTexelM(up);
            const float imgFine = 1.0f - smoothstep(1.5f, 6.0f, imgTexM);
            const float generic =
                (hp - water < 0.35f || elm > 0.01f) ? 0.0f : 1.0f;
            matAlb = lerp(matAlb, alb, imgFine * generic * 0.85f);
            const float ndlM = saturate(dot(nM, gSunDir.xyz));
            float3 colNear = matAlb * (SUN_IRR_C * ndlM + SkyRadiance(nM.y) * 0.55f);
            colNear = AerialPerspective(colNear, normalize(i.rel), distC);
            // M6n: the material weight rides landness too -- a half-emerged flat takes half
            // the wet-sand treatment, and the shore band grades instead of popping.
            col = lerp(col, colNear,
                       (1.0f - saturate((distC - 500.0f) / 2200.0f)) * 0.92f * landness);
        }
    }

    // ...and a short march through the volume bank renders the clouds themselves. NULL tiles
    // read zero: over clear air every sample is the hardware's answer, not a branch's.
    if (gTexIdx.w != 0xFFFFFFFFu) {
        const float3 ro = gCamAbs.xyz;
        const float3 rd = normalize(i.rel);
        const float top = gGlo.x + gCloudA.y;
        const float b = dot(ro, rd);
        const float cc = dot(ro, ro) - top * top;
        const float disc = b * b - cc;
        if (disc > 0.0f) {
            const float tShell = -b - sqrt(disc);                  // entering the cloud shell
            const float t0 = max(tShell, 0.0f);
            const float t1 = length(i.rel);                        // the ground
            const float span = t1 - t0;
            if (span > 1.0f) {
                const uint kSteps = 14;
                const float dt = span / kSteps;
                // Per-pixel jitter turns residual step-banding into noise the eye forgives.
                const float jit = frac(sin(dot(i.pos.xy, float2(12.9898f, 78.233f))) * 43758.5f);
                float T = 1.0f;
                float3 scat = 0.0f;
                const float mu = dot(rd, gSunDir.xyz);
                const float phase = 0.55f + 0.45f * mu;            // cheap forward lobe
                [loop] for (uint s = 0; s < kSteps && T > 0.02f; ++s) {
                    const float3 p = ro + rd * (t0 + (s + jit) * dt);
                    const float pr = length(p);
                    const float alt = pr - gGlo.x;
                    if (alt < 0.0f || alt > gCloudA.y) continue;
                    const float3 pd = CsToPlanet(p / pr);   // texturing needs planet lat/lon
                    const float3 uvw = float3(ReliefUv(pd), alt / gCloudA.y);
                    const float dens = gTex3D[gTexIdx.w].SampleLevel(sLinearClamp, uvw, 0).x;
                    if (dens <= 0.0f) continue;
                    const float sigma = dens * gCloudA.x;
                    const float stepT = exp(-sigma * dt);
                    // One sun-ward sample above approximates self-shadowing. Geometry stays
                    // tangent (shared sun); only the texture lookup rotates to planet.
                    const float3 pdT = p / pr;
                    const float3 lpT = pdT * (pr + 900.0f) + gSunDir.xyz * 900.0f;
                    const float3 luvw = float3(ReliefUv(CsToPlanet(normalize(lpT))),
                                               (length(lpT) - gGlo.x) / gCloudA.y);
                    const float lDens = gTex3D[gTexIdx.w].SampleLevel(sLinearClamp, luvw, 0).x;
                    const float sunT = exp(-lDens * gCloudA.x * 1500.0f) *
                                       saturate(dot(pdT, gSunDir.xyz) * 3.0f + 0.1f);
                    const float3 cloudCol =
                        SUN_IRR_C * (sunT * phase * gCloudA.z + 0.06f) + 0.02f;
                    scat += T * (1.0f - stepT) * cloudCol;
                    T *= stepT;
                }
                col = col * T + scat;
            }
        }
    }

    // The atmosphere, as seen ON the disc: grazing rays cross a long air path. M6j: this is a
    // FROM-SPACE effect and now fades in above 60 km -- inside the atmosphere it was pouring
    // grey-blue over every oblique view of the imagery (the "washed out when zooming in"
    // report; the near-field haze budget belongs to AerialPerspective alone).
    const float rim = pow(1.0f - saturate(dot(upT, v)), 3.0f) *
                      smoothstep(60000.0f, 250000.0f, length(gCamAbs.xyz) - gGlo.x);
    // Mars wears a THIN dusty shell, not Earth's blue one.
    const float3 rimCol = (gStreamF.z > 0.5f) ? float3(0.72f, 0.42f, 0.24f)
                                              : float3(0.42f, 0.58f, 0.92f);
    col = lerp(col, rimCol * (0.15f + 1.05f * day), rim * (gStreamF.z > 0.5f ? 0.18f : 0.55f));

    // M6d: the Mv2 wind bank's curl, on demand (V). Violet = NH-cyclonic (+), amber = anti-
    // cyclonic. NULL tiles read zero and tint nothing: calm air costs neither memory nor a
    // branch -- sparse structure carried as algebra, which is the atlas's whole thesis.
    if (gTexIdx2.z != 0u && gTexIdx2.y != 0xFFFFFFFFu) {
        const float2 wuv = float2(
            frac((lonDeg - gWindGeo.y) * gWindGeo.w / gWindB.x),
            saturate(((gWindGeo.x - degrees(lat)) * gWindGeo.z + 0.5f) / gWindB.y));
        const float4 mv = gTex[gTexIdx2.y].SampleLevel(sLinearClamp, wuv, 0);
        const float s = smoothstep(0.35f, 2.2f, abs(mv.w));   // curl stored x1e4; synoptic
                                                              // systems sit ~0.3-1, cores 2+
        const float3 tint = (mv.w > 0.0f) ? float3(0.45f, 0.20f, 0.95f)
                                          : float3(0.95f, 0.55f, 0.15f);
        col = lerp(col, tint * (0.25f + day), s * 0.4f);
    }

    col = ApplyComposedStencil(col, up);   // M6i: --stencil alignment overlay (off = no-op)
    return float4(col, 1.0f);
}

// ------------------------------------------------------------------ the atmosphere shell
//
// Fullscreen backdrop drawn BEFORE the surface (depth off): rays that MISS the planet march a
// thin Rayleigh shell, which is what puts the blue limb past the edge of the disc and the
// sunrise ring on the terminator. Rays that hit the planet output space black and let the
// surface overdraw. Single scattering, 6 steps, scale height 8 km -- a sketch of Bruneton with
// the same phase conventions, not a claim to be him.

static const float3 kBetaR = float3(5.8e-6f, 13.5e-6f, 33.1e-6f);
static const float kAtmTop = 60000.0f;
static const float kRayleighH = 8000.0f;

struct SkyVsOut {
    float4 pos : SV_Position;
    float3 dir : TEXCOORD0;
};

SkyVsOut VsSky(uint vid : SV_VertexID) {
    // Fullscreen triangle. The pixel ray is rebuilt in PsSky from the camera basis handed in
    // through b2 (the shared-surface CBV slot, which the globe never uses otherwise); only the
    // NDC coordinate rides through here.
    const float2 xy = float2((vid == 1) ? 3.0f : -1.0f, (vid == 2) ? 3.0f : -1.0f);
    SkyVsOut o;
    // M6g: z = 0 (reversed-Z infinity) + a GREATER_EQUAL depth test in the PSO means this
    // backdrop touches ONLY pixels nothing has drawn -- it can no longer stomp the estuary
    // layers that render before the globe in the one-world scene.
    o.pos = float4(xy, 0.0f, 1.0f);
    o.dir = float3(xy, 1.0f);
    return o;
}

cbuffer GlobeSkyCb : register(b2) {
    float4 gSkyFwd;     // camera forward, w = tan(fovY/2)
    float4 gSkyRight;   // camera right,  w = aspect
    float4 gSkyUp;      // camera up
};

float4 PsSky(SkyVsOut i) : SV_Target {
    const float2 ndc = i.dir.xy;
    const float3 rd = normalize(gSkyFwd.xyz + gSkyRight.xyz * (ndc.x * gSkyFwd.w * gSkyRight.w)
                                + gSkyUp.xyz * (ndc.y * gSkyFwd.w));
    const float3 ro = gCamAbs.xyz;

    // Planet hit? Space stays black; the surface pass owns the disc.
    const float b = dot(ro, rd);
    const float cPlan = dot(ro, ro) - gGlo.x * gGlo.x;
    if (b * b - cPlan > 0.0f && -b - sqrt(max(b * b - cPlan, 0.0f)) > 0.0f) {
        return float4(0, 0, 0, 1);
    }

    // Atmosphere shell chord.
    const float top = gGlo.x + kAtmTop;
    const float cTop = dot(ro, ro) - top * top;
    const float disc = b * b - cTop;
    if (disc <= 0.0f) return float4(0, 0, 0, 1);
    const float t0 = max(-b - sqrt(disc), 0.0f);
    const float t1 = -b + sqrt(disc);
    const float span = t1 - t0;
    if (span <= 0.0f) return float4(0, 0, 0, 1);

    const uint kSteps = 6;
    const float dt = span / kSteps;
    float3 sum = 0.0f;
    float odView = 0.0f;
    [unroll] for (uint s = 0; s < kSteps; ++s) {
        const float3 p = ro + rd * (t0 + (s + 0.5f) * dt);
        const float h = max(length(p) - gGlo.x, 0.0f);
        const float dens = exp(-h / kRayleighH);
        odView += dens * dt;
        // Sun transmittance out of the shell from p: one closed-form-ish estimate via the
        // grazing airmass (Chapman-lite).
        const float cosSun = dot(normalize(p), gSunDir.xyz);
        const float am = dens * kRayleighH * 2.2f / max(cosSun + 0.18f, 0.02f);
        const float3 sunT = exp(-kBetaR * max(am, 0.0f));
        sum += dens * dt * sunT * exp(-kBetaR * odView);
    }
    const float mu = dot(rd, gSunDir.xyz);
    const float phaseR = 0.0596831f * (1.0f + mu * mu);   // 3/(16 pi)
    float3 col = sum * kBetaR * phaseR * 22.0f * SUN_IRR_C;
    if (gStreamF.z > 0.5f) {   // Mars: 1% of Earth's air, dust-toned
        col = dot(col, float3(0.33f, 0.34f, 0.33f)) * float3(1.15f, 0.55f, 0.30f) * 0.30f;
    }

    // The sun itself, when the ray leaves the shell toward it.
    col += SUN_IRR_C * smoothstep(0.9998f, 0.99995f, mu) * 4.0f;
    return float4(col, 1.0f);
}
