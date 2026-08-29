// ================================================================================================
//  Compose.hlsli - M6i: the ONE render path for composed planet channels.
//
//  The renderer knows CHANNELS, not sources: earth color is earth color, earth height is earth
//  height. Whatever stack of imagery, bathymetry and regional grids produced a tile happened at
//  PAINT time on the compositor's workers; here there is exactly one residency-clamped fetch
//  per channel (plus the one window overlay, which is the same composed color at a depth the
//  16k cube cannot carry -- a resolution ramp of identical data, not a second source).
//
//  Every layer that includes this file embeds GA_COMPOSED_CB_ROWS (Common.hlsli) in its own
//  cbuffer FIRST, and fills those rows through ga::FillComposedCb alone. The globe and the
//  terrain therefore run literally the same code on literally the same constants -- the class
//  of bug where two layers disagree about the planet's surface is structurally gone.
// ================================================================================================
#ifndef GA_COMPOSE_HLSLI
#define GA_COMPOSE_HLSLI

static const float kCsPi = 3.14159265358979f;

float3 CsToTangent(float3 p) {
    return float3(dot(gCsR0.xyz, p), dot(gCsR1.xyz, p), dot(gCsR2.xyz, p));
}
float3 CsToPlanet(float3 t) {   // transpose of the orthonormal rotation
    return gCsR0.xyz * t.x + gCsR1.xyz * t.y + gCsR2.xyz * t.z;
}

// Residency-map reads are CONSERVATIVE (gather + max): the M6h bilinear ramp interpolated
// the map BELOW the locally-resident mip near boundaries, so the sampler read UNMAPPED tiles
// -- Tier-2 null zeros -- which diluted the window's alpha toward 0 and let the coarse cube
// bleed through in bilinear-isoline blobs (the "mottled marsh"). Never sample finer than any
// texel under the filter footprint; smoothness comes from trilinear WITHIN resident data.
float CsHaveCube(uint mapSrv, float3 dir) {
    const float4 g = gTexCube[mapSrv].GatherRed(sLinearClamp, dir);
    return max(max(g.x, g.y), max(g.z, g.w)) * 255.0f / 16.0f;
}
float CsHave2D(uint mapSrv, float2 uv) {
    const float4 g = gTex[mapSrv].GatherRed(sLinearClamp, uv);
    return max(max(g.x, g.y), max(g.z, g.w)) * 255.0f / 16.0f;
}

// The Mercator-window uv for a planet direction: every window realization (color AND height)
// shares ONE frame (gCsMerc), so their texels describe the same ground by construction.
float2 CsWindowUv(float3 dir) {
    const float lat = asin(clamp(dir.y, -1.0f, 1.0f));
    const float lonDeg = degrees(atan2(dir.z, dir.x));
    const float mx = (lonDeg + 180.0f) / 360.0f * gCsMerc.w;
    const float my = (0.5f - log(tan(0.785398163f + lat * 0.5f)) / (2.0f * kCsPi)) * gCsMerc.w;
    return (float2(mx, my) - gCsMerc.xy) * gCsMerc.z;
}

// The planet's composed color along a PLANET-frame unit radial. Residency maps sample
// BILINEAR so mip seams ramp instead of snapping (M6h); the window overlay feathers over the
// cube across 6% of its span AND rides its own per-texel ALPHA -- a texel the paint did not
// cover keeps the cube underneath, pixel by pixel, never tile by tile.
// M6j: the tenants are *_SRGB now -- the HARDWARE decodes to linear exactly (the old
// img*img*1.2 curve hack is gone; the user called the conversion, and the user was right).
// One LINEAR exposure constant remains: display-referred mosaics sit darker than the scene
// lighting expects, and scaling exposure is honest where bending the curve was not.
float3 ComposedColor(float3 dir) {
    float3 c = float3(0.5f, 0.5f, 0.5f);
    if (gCsF.x > 0.5f) {
        const float want = gTexCube[gCsU.x].CalculateLevelOfDetail(sLinearClamp, dir);
        const float have = CsHaveCube(gCsU.y, dir);
        c = gTexCube[gCsU.x].SampleLevel(sLinearClamp, dir, max(want, have)).rgb;
    }
    if (gCsF.y > 0.5f) {
        const float2 duv = CsWindowUv(dir);
        if (all(duv > 0.0f) && all(duv < 1.0f)) {
            const float2 fe = smoothstep(0.0f, 0.06f, duv) * smoothstep(1.0f, 0.94f, duv);
            const float want = gTex[gCsU.z].CalculateLevelOfDetail(sLinearClamp, duv);
            const float have = CsHave2D(gCsU.w, duv);
            const float4 w = gTex[gCsU.z].SampleLevel(sLinearClamp, duv, max(want, have));
            c = lerp(c, w.rgb, fe.x * fe.y * w.a);
        }
    }
    // M6j final word on "conversion": NO lift at all. The 1.35 exposure compensation matched
    // the old curve hack at mid-tones but pushed bright land cover (marsh tan) over the
    // tonemapper's shoulder into cream. Pixels ship exactly as the hardware sRGB decode
    // delivers them; scene brightness belongs to the lighting and gExposure alone.
    return c;
}
bool ComposedColorOn() { return gCsF.x > 0.5f; }

// The planet's composed height (metres), residency-clamped at the caller's lod. Usable from a
// VERTEX shader (no gradient intrinsics). The height WINDOW (same Mercator frame as the color
// window; CUDEM-fine near the estuary) overlays the cube exactly the way color does, so the
// land/sea gate and the shading normals stop being 611 m/px approximations where finer truth
// exists. Off -> 0 (a smooth sphere).
float ComposedHeight(float3 dir, float lod) {
    if (gCsF.z < 0.5f) return 0.0f;
    const float have = CsHaveCube(gCsU2.y, dir);
    float h = gTexCube[gCsU2.x].SampleLevel(sLinearClamp, dir, max(lod, have)).x;
    if (gCsU2.z != 0xFFFFFFFFu) {
        const float2 duv = CsWindowUv(dir);
        if (all(duv > 0.0f) && all(duv < 1.0f)) {
            const float2 fe = smoothstep(0.0f, 0.06f, duv) * smoothstep(1.0f, 0.94f, duv);
            // The window pyramid runs ~6 mips finer than the cube at the same footprint
            // (611 m cube texels vs 9.55 m z14 pixels), so the matching window mip is lod+6.
            const float wantW = clamp(lod + 6.0f, 0.0f, gCsG.z);
            const float haveW = CsHave2D(gCsU2.w, duv);
            const float hw = gTex[gCsU2.z].SampleLevel(sLinearClamp, duv,
                                                       max(wantW, haveW)).x;
            h = lerp(h, hw, fe.x * fe.y);
        }
    }
    return h;
}
bool ComposedHeightOn() { return gCsF.z > 0.5f; }

// M6i: the survey land mask -- raster realization of the GSHHG polygons, sampled per pixel.
// Returns land coverage 0..1, or -1 where no mask exists (a planet without GIS yet).
float ComposedLandMask(float3 dir) {
    const float2 duv = CsWindowUv(dir);
    if (gCsU3.x != 0xFFFFFFFFu && all(duv > 0.0f) && all(duv < 1.0f)) {
        return gTex[gCsU3.x].SampleLevel(sLinearClamp, duv, 0).x;
    }
    if (gCsU3.y != 0xFFFFFFFFu) {
        const float lat = asin(clamp(dir.y, -1.0f, 1.0f));
        const float lon = atan2(dir.z, dir.x);
        const float2 uvG = float2(lon / (2.0f * kCsPi) + 0.5f, 0.5f - lat / kCsPi);
        return gTex[gCsU3.y].SampleLevel(sLinearClamp, uvG, 0).x;
    }
    return -1.0f;
}

// THE land/sea classifier: survey polygons decide by default; inside the fine z14 window the
// height channel takes over COMPLETELY against the LIVE waterline -- tidal flats emerge and
// drown with the actual tide, which no static survey polygon can know. No mask, no window ->
// height sign vs the waterline (Mars: waterLevel 0).
bool ComposedIsLand(float3 dir, float hp, float waterLevel) {
    const float lm = ComposedLandMask(dir);
    bool land = (lm >= 0.0f) ? (lm > 0.5f) : (hp > waterLevel);
    if (gCsU2.z != 0xFFFFFFFFu) {
        const float2 duv = CsWindowUv(dir);
        if (all(duv > 0.0f) && all(duv < 1.0f)) {
            land = hp > waterLevel + 0.05f;
        }
    }
    return land;
}

// M6i debug: the alignment overlay (--stencil). The survey VECTORS render as real line
// geometry (GisLayer) -- this shader-side part draws what must be compared against them:
//   red     OUR composed height channel's zero-crossing (thin, fwidth-scaled)
//   blue    the shared Mercator window frame;  white  0.05-degree graticule
float3 ApplyComposedStencil(float3 col, float3 dir) {
    if (gCsG.w < 0.5f) return col;
    const float h = ComposedHeight(dir, -8.0f);   // finest RESIDENT height everywhere
    const float fw = max(fwidth(h), 0.05f);
    const float coastH = 1.0f - smoothstep(1.0f * fw, 2.5f * fw, abs(h));
    col = lerp(col, float3(1.0f, 0.12f, 0.10f), coastH * 0.8f);
    const float2 duv = CsWindowUv(dir);
    if (all(duv > -0.01f) && all(duv < 1.01f)) {
        const float2 e = min(abs(duv), abs(1.0f - duv));
        const float frame = 1.0f - smoothstep(0.0f, 0.003f, min(e.x, e.y));
        col = lerp(col, float3(0.2f, 0.4f, 1.0f), frame * 0.9f);
    }
    const float latDeg = degrees(asin(clamp(dir.y, -1.0f, 1.0f)));
    const float lonDeg = degrees(atan2(dir.z, dir.x));
    const float2 g = abs(frac(float2(lonDeg, latDeg) / 0.05f + 0.5f) - 0.5f);
    const float grat = 1.0f - smoothstep(0.0f, 0.006f, min(g.x, g.y));
    return lerp(col, float3(1.0f, 1.0f, 1.0f), grat * 0.30f);
}

// Mip so a height texel never shrinks much below a screen pixel (pixAngRad = one pixel's
// angular size; the caller owns that). gCsG.y is one texel's arc in radians. CONTINUOUS and
// allowed NEGATIVE: the cube clamps at its mip 0, and the window realization picks up exactly
// where the cube runs out (lod -6 = z14 texels) -- the climb-the-quadtree rule, in the
// sampler, with no branch anywhere.
float ComposedHeightLod(float dist, float pixAngRad) {
    const float texelM = gCsG.y * gCsF.w;
    const float pixM = dist * pixAngRad;
    return clamp(log2(max(pixM / texelM, 0.00390625f)), -8.0f, gCsG.x);
}

// Central-difference height gradient by eps-rotated directions: uniform-METRE steps at any
// latitude, no pole singularity, no per-source branch -- the equirect/NE-window fork this
// replaces needed both. Returns d(height)/d(metres) east and north; eps scales with lod so
// derivatives ride the same footprint the height fetch does.
float2 ComposedHeightGrad(float3 dir, float lod) {
    const float eps = gCsG.y * exp2(lod);
    float3 eP = cross(float3(0.0f, 1.0f, 0.0f), dir);
    eP = (dot(eP, eP) < 1e-8f) ? float3(1.0f, 0.0f, 0.0f) : normalize(eP);
    const float3 nP = cross(dir, eP);
    const float hE = ComposedHeight(normalize(dir + eP * eps), lod);
    const float hW = ComposedHeight(normalize(dir - eP * eps), lod);
    const float hN = ComposedHeight(normalize(dir + nP * eps), lod);
    const float hS = ComposedHeight(normalize(dir - nP * eps), lod);
    const float texM = eps * gCsF.w;
    return float2(hE - hW, hN - hS) / (2.0f * texM);
}

#endif  // GA_COMPOSE_HLSLI
