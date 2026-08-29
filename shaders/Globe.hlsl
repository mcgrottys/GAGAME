// ================================================================================================
//  Globe.hlsl - M6: the planet.
//
//  A quad-sphere CDLOD earth: the CPU walks a quadtree per cube face and emits visible nodes;
//  every node draws the same 32x32 grid, morphing odd vertices toward their even neighbours as
//  a node approaches the distance where its parent takes over (Strugar's CDLOD), so LOD rings
//  cross-fade with no cracks and no stitching. Relief is ETOPO 2022 in one equirect texture;
//  the ocean is shaded with the live GFS-Wave field: Cox-Munk slope variance from 10 m wind
//  drives the sun-glint lobe (the BRDF-LOD far field -- waves too small to resolve become
//  ROUGHNESS), significant height drives the storm-whitening.
//
//  Precision contract: positions are dir * (R + h) - camAbs in float. Both terms are ~6.4e6 m,
//  so the difference carries ~0.5-1 m of noise -- static per vertex, invisible above the 2 km
//  minimum altitude this mode allows. The metre-precise near field is the estuary's job (M6b
//  hands off); the planet frame is its own mode with the planet centre at the world origin,
//  y through the north pole, x through (0N, 0E).
// ================================================================================================
#define GA_NO_FIELD_BUFFER
#include "Common.hlsli"

cbuffer GlobeCb : register(b1) {
    float4 gGlo;        // x = R (m), y = relief exaggeration, z = node count, w = sim time s
    float4 gCamAbs;     // xyz = camera position in the planet frame (float: constant sub-metre
                        // offset only -- the jitter analysis lives in the header comment)
    uint4  gTexIdx;     // x = relief SRV, y = Hs SRV, z = wind SRV, w = CLOUD VOLUME SRV
                        // (0xFFFFFFFF = absent; the volume is a TileAtlas3D -- null = clear air)
    float4 gWavesA;     // Hs/wind grid: lat1 (deg, row 0), lon1, 1/dlat, 1/dlon (rows N->S)
    float4 gWavesB;     // x = nx, y = ny, z = pixel angular size (rad), w = max relief mip
    float4 gBeacon;     // xyz = unit direction to the Merrimack entrance, w = enabled
    float4 gCloudA;     // x = extinction /m at density 1, y = shell top (m), z = sun boost,
                        // w = ground-shadow strength
    uint4  gTexIdx2;    // M6d: x = NE 15s relief SRV, y = wind Mv2 bank SRV, z = overlay on
    float4 gNeGeo;      // NE window: lon0, lat1 (deg), 1/lonSpan, 1/latSpan
    float4 gWindGeo;    // wind grid: lat1, lon1, 1/dlat, 1/dlon
    float4 gWindB;      // x = nx, y = ny
    uint4  gStreamU;    // M6e streamed surfaces (cube SRVs): x = surface, y = normal (Mars BC5),
                        // z = surface residency map, w = normal residency map
    float4 gStreamF;    // x = surface on, y = normal on, z = planet is Mars, w = MOLA present
    uint4  gDetU;       // M6f detail window: x = texture SRV (2D), y = residency map, z = on
    float4 gDetGeo;     // Mercator z14-pixel window: org x, org y, 1/sizePx, unused
    // M6g: ONE WORLD. The globe renders in the estuary's tangent frame (x east, y up at the
    // origin, z north; sphere centre at flat (0,-R,0)). gFrameR* are rows of the
    // planet->tangent rotation; gCamAbs is REDEFINED as the sphere-CENTRED tangent-frame
    // camera (flat camera + (0,R,0), the doubles cancelled on the CPU). Texturing keeps the
    // PLANET-frame direction; lighting rotates into the tangent frame so the globe shares the
    // sea's sun -- the terminator is the real local sun, not a second one.
    float4 gFrameR0;    // rotation row 0 (east basis)
    float4 gFrameR1;    // rotation row 1 (up/radial-at-origin basis)
    float4 gFrameR2;    // rotation row 2 (north basis)
    float4 gEstGeo;     // estuary CUDEM window (deg): lon0, lat1, 1/lonSpan, 1/latSpan
                        // (w also gates: 0 = absent). The globe FOUNDATION-SINKS a few metres
                        // inside it so the sharp CUDEM surface owns the depth buffer there.
};

float3 ToTangent(float3 p) {
    return float3(dot(gFrameR0.xyz, p), dot(gFrameR1.xyz, p), dot(gFrameR2.xyz, p));
}
float3 ToPlanet(float3 t) {   // transpose of the orthonormal rotation
    return gFrameR0.xyz * t.x + gFrameR1.xyz * t.y + gFrameR2.xyz * t.z;
}

// M6e: sample a streamed cube with the classic residency clamp -- the R8 residency-map cube
// carries (finest resident mip * 16) per base tile; clamping the LOD there means a miss
// degrades to the best RESIDENT ancestor (blur), never to unmapped garbage.
float3 StreamedSample(uint texSrv, uint mapSrv, float3 dir) {
    const float want = gTexCube[texSrv].CalculateLevelOfDetail(sLinearClamp, dir);
    const float have = gTexCube[mapSrv].SampleLevel(sPointClamp, dir, 0).x * 255.0f / 16.0f;
    return gTexCube[texSrv].SampleLevel(sLinearClamp, dir, max(want, have)).rgb;
}
float2 StreamedSampleRg(uint texSrv, uint mapSrv, float3 dir) {
    const float want = gTexCube[texSrv].CalculateLevelOfDetail(sLinearClamp, dir);
    const float have = gTexCube[mapSrv].SampleLevel(sPointClamp, dir, 0).x * 255.0f / 16.0f;
    return gTexCube[texSrv].SampleLevel(sLinearClamp, dir, max(want, have)).rg;
}

struct GlobeNode {      // mirrors GlobeLayer::NodeData
    float2 uv0;         // face-uv rect origin
    float2 uvStep;      // face-uv per grid CELL (rect size / 32)
    uint  face;
    float morphStart;   // camera distances (m) over which this LOD cross-fades to its parent
    float morphEnd;
    float pad;
};
StructuredBuffer<GlobeNode> gNodes : register(t0, space0);

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

float2 ReliefUv(float3 dir) {
    const float lat = asin(clamp(dir.y, -1.0f, 1.0f));
    const float lon = atan2(dir.z, dir.x);
    // v is clamped a half-texel short of the poles so the WRAP sampler (which fixes the
    // dateline seam) can never drag the poles across each other.
    return float2(lon / (2.0f * kPi) + 0.5f,
                  clamp(0.5f - lat / kPi, 0.5f / 4096.0f, 1.0f - 0.5f / 4096.0f));
}

float ReliefAt(float3 dir, float lod) {
    return gTex[gTexIdx.x].SampleLevel(sLinearWrap, ReliefUv(dir), lod).x;
}

// Mip level so a relief texel never shrinks much below a screen pixel: kills the far-zoom
// coastline shimmer the single-mip first slice had.
float ReliefLod(float dist) {
    const float texelM = 2.0f * kPi * gGlo.x / 8192.0f;
    const float pixM = dist * gWavesB.z;
    return clamp(log2(max(pixM / texelM, 1.0f)), 0.0f, gWavesB.w);
}

// M6d: the New England 15-arc-second ring. Inside its window and close enough that the global
// texture is running out of texels, the regional relief takes over (edge-feathered, lod-faded).
float NeWeight(float lonDeg, float latDeg, float lod) {
    if (gTexIdx2.x == 0xFFFFFFFFu) return 0.0f;
    const float u = (lonDeg - gNeGeo.x) * gNeGeo.z;
    const float v = (gNeGeo.y - latDeg) * gNeGeo.w;
    if (any(float2(u, v) < 0.0f) || any(float2(u, v) > 1.0f)) return 0.0f;
    const float edge = min(min(u, 1.0f - u), min(v, 1.0f - v));
    return smoothstep(0.0f, 0.04f, edge) * saturate(2.5f - lod);
}

float ReliefBlended(float3 dir, float lod, out float wNe) {
    const float latDeg = degrees(asin(clamp(dir.y, -1.0f, 1.0f)));
    const float lonDeg = degrees(atan2(dir.z, dir.x));
    wNe = NeWeight(lonDeg, latDeg, lod);
    const float g = ReliefAt(dir, lod);
    if (wNe <= 0.0f) return g;
    const float2 uv = float2((lonDeg - gNeGeo.x) * gNeGeo.z, (gNeGeo.y - latDeg) * gNeGeo.w);
    const float n = gTex[gTexIdx2.x].SampleLevel(sLinearClamp, uv, 0).x;
    return lerp(g, n, wNe);
}

// ------------------------------------------------------------------ vertex

struct VsOut {
    float4 pos : SV_Position;
    float3 rel : TEXCOORD0;    // camera-relative position
    float3 dir : TEXCOORD1;    // unit radial (the sphere normal)
    float  h   : TEXCOORD2;    // relief metres (negative = ocean floor)
};

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
    const float d0 = length(ToTangent(dir0) * gGlo.x - gCamAbs.xyz);   // M6g: one frame
    const float k = saturate((d0 - nd.morphStart) / max(nd.morphEnd - nd.morphStart, 1.0f));
    g -= frac(g * 0.5f) * 2.0f * k;

    const float3 dir = CubeDir(nd.face, nd.uv0 + g * nd.uvStep);
    float wNeUnused;
    // M6f: Mars displaces MOLA when the harvester has delivered it (streamF.w); a Mars run
    // without MOLA stays a textured sphere rather than wearing Earth's relief.
    float h = (gStreamF.z > 0.5f && gStreamF.w < 0.5f)
                  ? 0.0f
                  : ReliefBlended(dir, 0.0f, wNeUnused);

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
    const float3 dirT = ToTangent(dir);
    o.rel = dirT * (gGlo.x + max(h, 0.0f) * gGlo.y) - gCamAbs.xyz;
    o.pos = mul(float4(o.rel, 1.0f), gViewProj);
    return o;
}

// ------------------------------------------------------------------ pixel

float3 Hypsometric(float h, float lat) {
    // Land tints by elevation, biased toward rock/snow at high latitude.
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

float4 PsMain(VsOut i) : SV_Target {
    const float3 up = normalize(i.dir);   // PLANET frame: lat/lon + every texture fetch
    const float3 v = normalize(-i.rel);   // TANGENT frame: geometry + lighting (M6g)
    const float lat = asin(clamp(up.y, -1.0f, 1.0f));
    const float lonDeg = degrees(atan2(up.z, up.x));

    // M6g: lighting happens in the tangent frame so the globe shares the sea's ONE sun.
    const float3 upT = ToTangent(up);
    const float3 axisT = float3(gFrameR0.y, gFrameR1.y, gFrameR2.y);   // planet north pole
    float3 east = cross(axisT, upT);
    east = (dot(east, east) < 1e-8f) ? float3(1.0f, 0.0f, 0.0f) : normalize(east);
    const float3 north = cross(upT, east);
    const float day = saturate(dot(gSunDir.xyz, upT) * 3.0f + 0.12f);

    const float lod = ReliefLod(length(i.rel));
    const float latDegPx = degrees(lat);
    const float lonDegPx = degrees(atan2(up.z, up.x));
    const float wNe = NeWeight(lonDegPx, latDegPx, lod);
    float3 n = upT;
    float3 alb;
    float spec = 0.0f;
    if (gStreamF.z > 0.5f) {
        // MARS: the rescued sample pyramids ARE the planet -- residency-clamped diffuse +
        // BC5 surface normals in the local ENU frame.
        alb = StreamedSample(gStreamU.x, gStreamU.z, up);
        alb = alb * alb * 1.1f;   // sRGB-ish to linear
        if (gStreamF.y > 0.5f) {
            const float2 nxy = StreamedSampleRg(gStreamU.y, gStreamU.w, up);   // SNORM -1..1
            n = normalize(upT + east * nxy.x * 1.2f + north * nxy.y * 1.2f);
        }
    } else if (i.h > 0.0f) {
        // Relief normal from the height texture, at a fixed modest slope gain (the vertical
        // exaggeration is a display choice; shading at x25 would posterize the continents).
        // Inside the NE ring the 460 m texture supplies the derivatives instead (M6d).
        float hE, hW, hN, hS, texMx, texMy;
        if (wNe > 0.5f) {
            const float2 uv = float2((lonDegPx - gNeGeo.x) * gNeGeo.z,
                                     (gNeGeo.y - latDegPx) * gNeGeo.w);
            const float2 ts = float2(1.0f / 1440.0f, 1.0f / 1200.0f);
            texMx = 461.0f * max(cos(lat), 0.05f);
            texMy = 461.0f;
            hE = gTex[gTexIdx2.x].SampleLevel(sLinearClamp, uv + float2(ts.x, 0), 0).x;
            hW = gTex[gTexIdx2.x].SampleLevel(sLinearClamp, uv - float2(ts.x, 0), 0).x;
            hN = gTex[gTexIdx2.x].SampleLevel(sLinearClamp, uv - float2(0, ts.y), 0).x;
            hS = gTex[gTexIdx2.x].SampleLevel(sLinearClamp, uv + float2(0, ts.y), 0).x;
        } else {
            const float mipScale = exp2(lod);
            const float2 ts = float2(mipScale / 8192.0f, mipScale / 4096.0f);
            const float2 uv = ReliefUv(up);
            texMx = 2.0f * kPi * gGlo.x * max(cos(lat), 0.05f) * mipScale / 8192.0f;
            texMy = kPi * gGlo.x * mipScale / 4096.0f;
            hE = gTex[gTexIdx.x].SampleLevel(sLinearWrap, uv + float2(ts.x, 0), lod).x;
            hW = gTex[gTexIdx.x].SampleLevel(sLinearWrap, uv - float2(ts.x, 0), lod).x;
            hN = gTex[gTexIdx.x].SampleLevel(sLinearWrap, uv - float2(0, ts.y), lod).x;
            hS = gTex[gTexIdx.x].SampleLevel(sLinearWrap, uv + float2(0, ts.y), lod).x;
        }
        const float kSlopeGain = 4.0f;
        n = normalize(upT - east * ((hE - hW) * kSlopeGain / (2.0f * texMx))
                          - north * ((hN - hS) * kSlopeGain / (2.0f * texMy)));
        alb = Hypsometric(i.h, lat);
    } else {
        // The live sea. Depth tints the shelves; GFS-Wave whitens the storms.
        const float shelf = saturate(1.0f + i.h / 160.0f);      // 1 at the beach, 0 by -160 m
        alb = lerp(float3(0.013f, 0.055f, 0.115f), float3(0.06f, 0.30f, 0.34f),
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
        alb = lerp(alb, float3(0.55f, 0.62f, 0.68f), saturate((hs - 2.5f) / 9.0f) * 0.55f);

        // Cox-Munk: slope variance from wind speed; the glint lobe IS the far-field BRDF.
        const float s2 = 0.003f + 0.00512f * wind;
        const float3 hv = normalize(v + gSunDir.xyz);
        const float ch = saturate(dot(hv, upT));
        const float t2 = max(1.0f - ch * ch, 0.0f) / max(ch * ch, 1e-4f);
        spec = exp(-t2 / s2) / (4.0f * kPi * s2 * max(ch * ch * ch * ch, 1e-4f));
        const float fres = 0.02f + 0.98f * pow(1.0f - saturate(dot(v, hv)), 5.0f);
        spec *= fres * saturate(dot(gSunDir.xyz, upT));
    }

    // ---- M6e: streamed Earth imagery (residency-clamped). Land takes the imagery outright;
    // water blends it into the shallows only, so the LIVE ocean (Hs whitening, Cox-Munk glint)
    // keeps doing physics that a baked mosaic cannot.
    if (gStreamF.x > 0.5f && gStreamF.z < 0.5f) {
        float3 img = StreamedSample(gStreamU.x, gStreamU.z, up);
        img = img * img * 1.2f;
        alb = (i.h > 0.0f) ? img : lerp(alb, img, 0.6f * saturate(1.0f + i.h / 80.0f));
    }

    // ---- M6f: the Merrimack detail window -- deeper imagery inside a Mercator-aligned rect,
    // CONTINUOUS by construction: a feathered edge blend, and the residency clamp inside (its
    // coarsest always-resident mip matches the global cube's quality, so an unstreamed region
    // just looks like the globe). No LOD branches; the manager makes every sample defined.
    if (gDetU.z != 0u && gStreamF.z < 0.5f) {
        const float n14 = 16384.0f * 256.0f;
        const float mx = (lonDegPx + 180.0f) / 360.0f * n14;
        const float my =
            (0.5f - log(tan(0.785398163f + lat * 0.5f)) / (2.0f * kPi)) * n14;
        const float2 duv = (float2(mx, my) - gDetGeo.xy) * gDetGeo.z;
        if (all(duv > 0.0f) && all(duv < 1.0f)) {
            const float2 fe = smoothstep(0.0f, 0.06f, duv) * smoothstep(1.0f, 0.94f, duv);
            const float wantD = gTex[gDetU.x].CalculateLevelOfDetail(sLinearClamp, duv);
            const float haveD =
                gTex[gDetU.y].SampleLevel(sPointClamp, duv, 0).x * 255.0f / 16.0f;
            float3 img = gTex[gDetU.x].SampleLevel(sLinearClamp, duv, max(wantD, haveD)).rgb;
            img = img * img * 1.2f;
            const float w = fe.x * fe.y;
            const float3 target =
                (i.h > 0.0f) ? img : lerp(alb, img, 0.6f * saturate(1.0f + i.h / 80.0f));
            alb = lerp(alb, target, w);
        }
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
    col += alb * float3(0.010f, 0.014f, 0.028f) * (1.0f - day);   // moonlit-blue night side

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
                    const float3 pd = ToPlanet(p / pr);   // texturing needs planet lat/lon
                    const float3 uvw = float3(ReliefUv(pd), alt / gCloudA.y);
                    const float dens = gTex3D[gTexIdx.w].SampleLevel(sLinearClamp, uvw, 0).x;
                    if (dens <= 0.0f) continue;
                    const float sigma = dens * gCloudA.x;
                    const float stepT = exp(-sigma * dt);
                    // One sun-ward sample above approximates self-shadowing. Geometry stays
                    // tangent (shared sun); only the texture lookup rotates to planet.
                    const float3 pdT = p / pr;
                    const float3 lpT = pdT * (pr + 900.0f) + gSunDir.xyz * 900.0f;
                    const float3 luvw = float3(ReliefUv(ToPlanet(normalize(lpT))),
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

    // The atmosphere, as seen ON the disc: grazing rays cross a long air path. (The limb glow
    // BEYOND the edge is a later shell pass.)
    const float rim = pow(1.0f - saturate(dot(upT, v)), 3.0f);
    // Mars wears a THIN dusty shell, not Earth's blue one.
    const float3 rimCol = (gStreamF.z > 0.5f) ? float3(0.72f, 0.42f, 0.24f)
                                              : float3(0.42f, 0.58f, 0.92f);
    col = lerp(col, rimCol * (0.15f + 1.05f * day), rim * (gStreamF.z > 0.5f ? 0.18f : 0.55f));

    // M6d: the Mv2 wind bank's curl, on demand (V). Violet = NH-cyclonic (+), amber = anti-
    // cyclonic. NULL tiles read zero and tint nothing: calm air costs neither memory nor a
    // branch -- sparse structure carried as algebra, which is the atlas's whole thesis.
    if (gTexIdx2.z != 0u && gTexIdx2.y != 0xFFFFFFFFu) {
        const float2 wuv = float2(
            frac((lonDegPx - gWindGeo.y) * gWindGeo.w / gWindB.x),
            saturate(((gWindGeo.x - latDegPx) * gWindGeo.z + 0.5f) / gWindB.y));
        const float4 mv = gTex[gTexIdx2.y].SampleLevel(sLinearClamp, wuv, 0);
        const float s = smoothstep(0.35f, 2.2f, abs(mv.w));   // curl stored x1e4; synoptic
                                                              // systems sit ~0.3-1, cores 2+
        const float3 tint = (mv.w > 0.0f) ? float3(0.45f, 0.20f, 0.95f)
                                          : float3(0.95f, 0.55f, 0.15f);
        col = lerp(col, tint * (0.25f + day), s * 0.4f);
    }

    // Home beacon: the Merrimack entrance, so the M6b zoom has a destination.
    if (gBeacon.w > 0.5f) {
        const float chord2 = dot(up - gBeacon.xyz, up - gBeacon.xyz);
        const float distM2 = chord2 * gGlo.x * gGlo.x;
        const float glow = exp(-distM2 / (2.0f * 9000.0f * 9000.0f));
        col += float3(1.4f, 0.55f, 0.10f) * glow *
               (0.6f + 0.4f * sin(gGlo.w * 2.5f));
    }
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
