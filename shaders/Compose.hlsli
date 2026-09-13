// ================================================================================================
//  Compose.hlsli - M6i: the ONE render path for composed planet channels.
//
//  The renderer knows CHANNELS, not sources: earth color is earth color, earth height is earth
//  height. Whatever stack of imagery, bathymetry and regional grids produced a tile happened at
//  PAINT time on the compositor's workers; here there is exactly one residency-clamped fetch
//  per channel (plus the one window overlay, which is the same composed color at a depth the
//  16k cube cannot carry -- a resolution ramp of identical data, not a second source).
//
//  The rows this file reads are Common.hlsli's SurfaceCb (b2): ONE constant buffer, filled
//  through SurfaceFrame::Fill alone (M12 step 4a) and bound once a frame for every layer (M12
//  step 4g, where each layer embedded a copy in its own cbuffer). The globe and the terrain
//  therefore run literally the same code on literally the same constants -- the class of bug
//  where two layers disagree about the planet's surface is structurally gone.
//  M12 step 4e: the reads themselves -- the window uv, the residency floor, the residency-
//  clamped sample, the cube-versus-page choice -- are PageSample.hlsli's contract, shared with
//  the kernels; the functions here bind this cbuffer's rows and views to it and nothing else.
// ================================================================================================
#ifndef GA_COMPOSE_HLSLI
#define GA_COMPOSE_HLSLI

#include "PageSample.hlsli"

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
// A page tenant's floor is PageHave / PageHaveCube (PageSample.hlsli); these two serve the old
// three-tenant path's TextureCube / Texture2D realizations (and Mars's native pyramids) with
// the same law.
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
float2 CsWindowUv(float3 dir) { return PageUv(dir, gCsMerc); }

// The planet's composed color along a PLANET-frame unit radial. Residency maps sample
// BILINEAR so mip seams ramp instead of snapping (M6h); the window overlay feathers over the
// cube across 6% of its span AND rides its own per-texel ALPHA -- a texel the paint did not
// cover keeps the cube underneath, pixel by pixel, never tile by tile.
// M6j: the tenants are *_SRGB now -- the HARDWARE decodes to linear exactly (the old
// img*img*1.2 curve hack is gone; the user called the conversion, and the user was right).
// One LINEAR exposure constant remains: display-referred mosaics sit darker than the scene
// lighting expects, and scaling exposure is honest where bending the curve was not.
// M9ap: residency of a PAGE (a slice of the page tenant's array) and of a face through the
// tenant's cube view, conservative like the rest (PageSample.hlsli).
float CsHavePage(uint mapSrv, float2 uv, uint slice) {
    return PageHave(gTexArr[mapSrv], sLinearClamp, uv, slice);
}
float CsHaveCubeArr(uint mapSrv, float3 dir) {
    return PageHaveCube(gTexCubeArr[mapSrv], sLinearClamp, dir);
}
// The ground texel (metres) at mip 0 of the three rungs -- the cube, the z14 window, the z17
// detail: the surface's own row (gCsGround, SurfaceFrame::Fill from Lattice::GroundRes(0):
// 611.496.., 9.5546.., 1.1943..), M12 step 4f, where the literals 611 / 9.55 / 1.19 stood.
// Exact, the page's mip 6 IS the cube's mip 0 and the z17's mip 3 the z14's mip 0, and
// PageWins takes the page there.
float3 CsGroundM() { return gCsGround.xyz; }

// M9ap: THE PAGES PATH. One texture, pages selected by CONTAINMENT and by what is actually
// resident: every page is the same megatexture at a different ground resolution, so the page
// whose resident mip gives the finest ground texel at this pixel is the right answer and needs
// no fade against its neighbour -- where two pages are resident at the same resolution they
// hold the same pixels. `hand` and `finer` are gone; there is nothing to hand off between.
// M12 step 4e: the ladder is PageWins on PageGroundM, the rung's resident ground against the
// ground held -- "at least as fine" (the height path's spelling), which with these literals is
// the strict `<` this path used to write: no two rungs' literal grounds are ever equal in float
// (9.55 * 64 = 611.2, not 611; 1.19 * 8 = 9.52, not 9.55). Measured, not assumed.
float3 ComposedColorPages(float3 dir) {
    // The cube, through the cube views over slices 0..5 (hardware-seamless across faces).
    const float haveC = CsHaveCubeArr(gCsU.y, dir);
    float3 c = PageSampleCube(gTexCubeArr[gCsU.x], sAniso, dir, haveC).rgb;
    const float3 g0 = CsGroundM();
    float ground = PageGroundM(g0.x, haveC);
    if (gCsF.y > 0.5f) {
        const float2 duv = CsWindowUv(dir);
        if (all(duv > 0.0f) && all(duv < 1.0f)) {
            const float haveW = CsHavePage(gCsU5.y, duv, gCsU5.z);
            const float gW = PageGroundM(g0.y, haveW);
            if (PageWins(gW, ground)) {
                c = PageSample(gTexArr[gCsU5.x], sAniso, duv, gCsU5.z, haveW).rgb;
                ground = gW;
            }
            if (gCsU5.w != 0xFFFFFFFFu) {
                const float2 tuv = duv * gCsDet.z + gCsDet.xy;
                if (all(tuv > 0.0f) && all(tuv < 1.0f)) {
                    const float haveD = CsHavePage(gCsU5.y, tuv, gCsU5.w);
                    const float gD = PageGroundM(g0.z, haveD);
                    if (PageWins(gD, ground)) {
                        c = PageSample(gTexArr[gCsU5.x], sAniso, tuv, gCsU5.w, haveD).rgb;
                        ground = gD;
                    }
                }
            }
        }
    }
    return c;
}

float3 ComposedColor(float3 dir) {
    if (gCsU5.x != 0xFFFFFFFFu && gCsF.x > 0.5f) return ComposedColorPages(dir);
    float3 c = float3(0.5f, 0.5f, 0.5f);
    if (gCsF.x > 0.5f) {
        // M9z: ANISOTROPIC, and the CalculateLevelOfDetail call is GONE. It collapsed the
        // screen-space Jacobian to ONE number -- the longest derivative -- and then took a
        // single trilinear tap there, which is why the surface smeared along the compressed
        // axis wherever the globe curves away. Sample()'s min-LOD clamp form hands the whole
        // Jacobian to the hardware and still honours the residency floor, so a miss degrades
        // to the best RESIDENT ancestor exactly as before. Fewer instructions AND the right
        // footprint: `have` was the only value the old form needed to keep.
        const float have = CsHaveCube(gCsU.y, dir);
        c = gTexCube[gCsU.x].Sample(sAniso, dir, have).rgb;
    }
    if (gCsF.y > 0.5f) {
        const float2 duv = CsWindowUv(dir);
        if (all(duv > 0.0f) && all(duv < 1.0f)) {
            const float2 fe = smoothstep(0.0f, 0.06f, duv) * smoothstep(1.0f, 0.94f, duv);
            // `want` survives here because it GATES the hand-off below, not just the fetch --
            // a scalar decision genuinely needs a scalar. The fetch itself goes anisotropic.
            const float want = gTex[gCsU.z].CalculateLevelOfDetail(sLinearClamp, duv);
            const float have = CsHave2D(gCsU.w, duv);
            const float4 w = gTex[gCsU.z].Sample(sAniso, duv, int2(0, 0), have);
            // M7h: the window HANDS OFF to the cube when the view outresolves even its
            // pinned floor (want past ~mip 6): a rung that cannot add detail must vanish,
            // or its different-zoom capture sits as a vintage RECTANGLE on the planet.
            // Symmetric with the z17 rung's finer-only gate below.
            const float hand = 1.0f - smoothstep(5.5f, 7.0f, want);
            c = lerp(c, w.rgb, fe.x * fe.y * w.a * hand);
            // M7f: the DETAIL window (z17, ~1.2 m px) -- the ladder's third rung, in the
            // same Mercator frame, so the near field stops being capped at 9.5 m texels.
            if (gCsU4.x != 0xFFFFFFFFu) {
                const float2 tuv = duv * gCsDet.z + gCsDet.xy;
                if (all(tuv > 0.0f) && all(tuv < 1.0f)) {
                    const float2 fd =
                        smoothstep(0.0f, 0.04f, tuv) * smoothstep(1.0f, 0.96f, tuv);
                    const float wantD =
                        gTex[gCsU4.x].CalculateLevelOfDetail(sLinearClamp, tuv);
                    const float haveD = CsHave2D(gCsU4.y, tuv);
                    const float lodD = max(wantD, haveD);
                    const float4 d = gTex[gCsU4.x].Sample(sAniso, tuv, int2(0, 0), haveD);
                    // Take the detail rung only where it is actually FINER than what the
                    // z14 window just delivered (z17 mip m == z14 mip m-3): a half-warmed
                    // detail tile must never replace sharper coarse truth with mush.
                    const float finer = saturate(max(want, have) + 3.0f - lodD);
                    c = lerp(c, d.rgb, fd.x * fd.y * d.a * finer);
                }
            }
        }
    }
    // M6j final word on "conversion": NO lift at all. The 1.35 exposure compensation matched
    // the old curve hack at mid-tones but pushed bright land cover (marsh tan) over the
    // tonemapper's shoulder into cream. Pixels ship exactly as the hardware sRGB decode
    // delivers them; scene brightness belongs to the lighting and gExposure alone.
    return c;
}
bool ComposedColorOn() { return gCsF.x > 0.5f; }

// M9av: THE SEAFLOOR THROUGH OPAQUE WATER. The megatexture's ocean texels are DRY seafloor
// albedo (synth.seafloor.relief: the bathymetry's hillshade times a sediment ramp keyed on
// datum depth). Beer-Lambert over the measured K_d makes water past a few tens of metres
// opaque, and the two-flux endpoint (chlorophyll, SPM) IS its colour -- that stays. The floor's
// SHADING is carried as a modulation of that endpoint's brightness: divide the ramp's own
// luminance back out of the texel and what remains is the hillshade, flat bed = 1. A map
// convention, declared: the hue is the measurement, the relief is the floor's, and
// kSeafloorRelief = 0 removes it.
static const float kSeafloorRelief = 1.0f;          // 0 = the physical endpoint alone
static const float kSeafloorReliefContrast = 1.0f;  // hillshade gain over the painted swing (the
                                                    // user chose the painted swing as is; a
                                                    // brightened "map ocean" was tried and rejected)
float SeafloorRampLuma(float depthM) {
    // MIRRORS src/compose/Sources.cpp SeafloorRamp: change both.
    const float d = clamp(depthM, 0.0f, 4000.0f);
    const float3 c0 = float3(0.66f, 0.60f, 0.46f), c1 = float3(0.56f, 0.52f, 0.42f),
                 c2 = float3(0.46f, 0.44f, 0.39f), c3 = float3(0.39f, 0.37f, 0.34f),
                 c4 = float3(0.32f, 0.31f, 0.30f);
    float3 c;
    if (d < 40.0f) c = lerp(c0, c1, d / 40.0f);
    else if (d < 200.0f) c = lerp(c1, c2, (d - 40.0f) / 160.0f);
    else if (d < 1000.0f) c = lerp(c2, c3, (d - 200.0f) / 800.0f);
    else c = lerp(c3, c4, (d - 1000.0f) / 3000.0f);
    return dot(c, float3(0.299f, 0.587f, 0.114f));
}
// The endpoint's HUE and brightness are the measurement's (albWater is kept in the signature
// for the record: a version that redrew the floor through water of that hue at a declared
// visibility was brighter, greener, and rejected -- the physical water is the look). Only the
// floor's hillshade rides the endpoint, as a brightness modulation, where the bed term has died.
float3 SeafloorReliefMod(float3 albSea, float3 albWater, float3 floorAlb, float hp, float opaque) {
    const float3 L = float3(0.299f, 0.587f, 0.114f);
    const float ref = SeafloorRampLuma(max(-hp, 0.0f));
    const float shade0 = dot(floorAlb, L) / max(ref, 1e-3f);          // flat bed = 1
    const float shade = clamp(1.0f + kSeafloorReliefContrast * (shade0 - 1.0f), 0.30f, 2.2f);
    return albSea * lerp(1.0f, shade, kSeafloorRelief * saturate(opaque));
}

// M7g: the effective composed-color texel (metres) RESIDENT at this pixel. Consumers that
// historically replaced the mosaic outright (the close-up material constants, born when
// the near field was a 9.5 m blur) ask this and YIELD where the imagery outresolves them.
float ComposedColorTexelM(float3 dir) {
    const float3 g0 = CsGroundM();
    float t = g0.x;   // cube-only worst case
    if (gCsU5.x != 0xFFFFFFFFu) {
        // M9ap: the pages path reports the same choice ComposedColorPages makes -- the finest
        // resident ground of the ladder (PageWins, as a value).
        t = PageGroundM(g0.x, CsHaveCubeArr(gCsU.y, dir));
        if (gCsF.y > 0.5f) {
            const float2 duv = CsWindowUv(dir);
            if (all(duv > 0.0f) && all(duv < 1.0f)) {
                t = min(t, PageGroundM(g0.y, CsHavePage(gCsU5.y, duv, gCsU5.z)));
                if (gCsU5.w != 0xFFFFFFFFu) {
                    const float2 tuv = duv * gCsDet.z + gCsDet.xy;
                    if (all(tuv > 0.001f) && all(tuv < 0.999f)) {
                        t = min(t, PageGroundM(g0.z, CsHavePage(gCsU5.y, tuv, gCsU5.w)));
                    }
                }
            }
        }
        return t;
    }
    if (gCsF.y > 0.5f && gCsU.z != 0xFFFFFFFFu) {
        const float2 duv = CsWindowUv(dir);
        if (all(duv > 0.0f) && all(duv < 1.0f)) {
            t = PageGroundM(g0.y, CsHave2D(gCsU.w, duv));
            if (gCsU4.x != 0xFFFFFFFFu) {
                const float2 tuv = duv * gCsDet.z + gCsDet.xy;
                if (all(tuv > 0.001f) && all(tuv < 0.999f)) {
                    t = min(t, PageGroundM(g0.z, CsHave2D(gCsU4.y, tuv)));
                }
            }
        }
    }
    return t;
}

// The planet's composed height (metres), residency-clamped at the caller's lod. Usable from a
// VERTEX shader (no gradient intrinsics). The height WINDOW (same Mercator frame as the color
// window; CUDEM-fine near the estuary) overlays the cube exactly the way color does, so the
// land/sea gate and the shading normals stop being 611 m/px approximations where finer truth
// exists. Off -> 0 (a smooth sphere).
// M9aq: is there a height window at all, on either path? (Globe.hlsl gates its near-field
// material on this.)
bool CsHeightWindowOn() { return gCsU6.x != 0xFFFFFFFFu || gCsU2.z != 0xFFFFFFFFu; }
// The height window's resident mip at a window uv, on either path.
float CsHaveHeightWin(float2 duv) {
    if (gCsU6.x != 0xFFFFFFFFu) return CsHavePage(gCsU6.y, duv, gCsU6.z);
    return CsHave2D(gCsU2.w, duv);
}

// M9aq: THE HEIGHT PAGES PATH. Same rule as colour: the cube through its cube views, the z14
// page through the array view, the page chosen by containment and by what is resident --
// no feather, because both pages are the same height field at different ground resolutions.
float ComposedHeightPages(float3 dir, float lod) {
    const float haveC = CsHaveCubeArr(gCsU2.y, dir);
    float h = PageSampleLevelCube(gTexCubeArr[gCsU2.x], sLinearClamp, dir, lod, haveC).x;
    const float2 duv = CsWindowUv(dir);
    if (all(duv > 0.0f) && all(duv < 1.0f)) {
        // The window pyramid runs ~6 mips finer than the cube at the same footprint.
        const float wantW = clamp(lod + 6.0f, 0.0f, gCsG.z);
        const float haveW = CsHavePage(gCsU6.y, duv, gCsU6.z);
        // Take the page where its resident texel is at least as fine as the cube's.
        if (PageWins(haveC, haveW, CsGroundM().xy)) {
            h = PageSampleLevel(gTexArr[gCsU6.x], sLinearClamp, duv, gCsU6.z, wantW, haveW).x;
        }
    }
    return h;
}

float ComposedHeight(float3 dir, float lod) {
    if (gCsF.z < 0.5f) return 0.0f;
    if (gCsU6.x != 0xFFFFFFFFu) return ComposedHeightPages(dir, lod);
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

// M9ay: THE SURVEY AS PAGES. gis.landsea's own tree -- the vector rings swept per tile on the
// same addresses as the imagery and the bed -- is a page tenant: r = water coverage (1 water,
// 0 land), b = edited (a hand ring decided this texel), a = surveyed (an opinion exists).
// The finest page with an opinion answers: z17 (1.19 m, where the old ~1 m edit raster was),
// z14, then the cube face. No page with an opinion here -> false: the classifier falls back
// to the height sign. The three committed rasters this replaces (window R8G8, global R8,
// fine edit R8G2) read the .raw parity fills the vector mask refuses; they are gone.
bool CsMaskSample(float3 dir, out float4 m) {
    m = float4(0, 0, 0, 0);
    if (gCsU3.x == 0xFFFFFFFFu) return false;
    const float2 duv = CsWindowUv(dir);
    if (all(duv > 0.0f) && all(duv < 1.0f)) {
        if (gCsU5.w != 0xFFFFFFFFu) {   // the z17 page exists in the colour ladder -> ours too
            const float2 tuv = duv * gCsDet.z + gCsDet.xy;
            if (all(tuv > 0.001f) && all(tuv < 0.999f)) {
                const float haveD = CsHavePage(gCsU3.y, tuv, 7u);
                if (haveD <= 7.5f) {   // the finest resident level: want 0, floored to have
                    m = PageSampleLevel(gTexArr[gCsU3.x], sLinearClamp, tuv, 7u, 0.0f, haveD);
                    if (m.a > 0.001f) return true;
                }
            }
        }
        const float haveW = CsHavePage(gCsU3.y, duv, 6u);
        if (haveW <= 7.5f) {
            m = PageSampleLevel(gTexArr[gCsU3.x], sLinearClamp, duv, 6u, 0.0f, haveW);
            if (m.a > 0.001f) return true;
        }
    }
    if (gCsU3.z != 0xFFFFFFFFu) {
        const float haveC = CsHaveCubeArr(gCsU3.w, dir);
        if (haveC <= 7.5f) {
            m = PageSampleLevelCube(gTexCubeArr[gCsU3.z], sLinearClamp, dir, 0.0f, haveC);
            if (m.a > 0.001f) return true;
        }
    }
    return false;
}

// The survey land mask, per pixel: land coverage 0..1, or -1 where the survey has no opinion
// (outside its rings' box, or a planet without GIS). Coverage is un-premultiplied: a texel
// half surveyed still reports its water fraction, not half of it.
float ComposedLandMask(float3 dir) {
    float4 m;
    if (!CsMaskSample(dir, m)) return -1.0f;
    return 1.0f - m.r / max(m.a, 0.001f);
}

// THE land/sea classifier: survey polygons decide by default; inside the fine z14 window the
// height channel takes over against the LIVE waterline -- tidal flats emerge and drown with
// the actual tide, which no static survey polygon can know.
//
// M6n: the in-window answer is ANALOG. Flats sit within centimetres of the tide line, and a
// hard threshold there turned every residency change of the height data into land/water
// SPECKLE (tile-shaped patches flickering during streaming). A ~40 cm shore band -- 15 cm
// below the waterline to 25 cm above -- classifies a half-emerged flat as half-emerged:
// shading mixes, geometry blends, and data-level changes modulate a gradient the eye reads
// as wet sand instead of flipping a bit. Outside the window the mask stays binary (a survey
// polygon IS a bit); no mask, no window -> height sign vs the waterline (Mars: 0).
// M7f/M9ay: the hand edits -- (land, edited) at this pixel from the mask pages; the z17 page
// keeps the jetties at 1.19 m where the ~1 m fine raster used to. (0, 0) with no opinion.
float2 CsEditMask(float3 dir) {
    float4 m;
    if (!CsMaskSample(dir, m)) return float2(0.0f, 0.0f);
    const float a = max(m.a, 0.001f);
    return float2(1.0f - m.r / a, m.b / a);
}

float ComposedLandness(float3 dir, float hp, float waterLevel) {
    const float lm = ComposedLandMask(dir);
    float land = (lm >= 0.0f) ? ((lm > 0.5f) ? 1.0f : 0.0f)
                              : ((hp > waterLevel) ? 1.0f : 0.0f);
    if (gCsU2.z != 0xFFFFFFFFu) {
        const float2 duv = CsWindowUv(dir);
        if (all(duv > 0.0f) && all(duv < 1.0f)) {
            land = smoothstep(waterLevel - 0.15f, waterLevel + 0.25f, hp);
        }
    }
    // M6p: HAND EDITS ARE LAW. The window mask is R8G8 -- g flags texels painted by
    // data/gis/edits.geojson, and a flagged texel's mask value overrides survey and the
    // live tide alike (the survey shoreline predates the jetties, and the stabilized height
    // classifier smears their thin ridges -- the operator's polygon settles it). Bilinear g
    // blends the override's own edge.
    if (gCsU3.x != 0xFFFFFFFFu) {
        const float2 me = CsEditMask(dir);
        land = lerp(land, (me.x > 0.5f) ? 1.0f : 0.0f, smoothstep(0.2f, 0.8f, me.y));
    }
    return land;
}
// The binary view, for consumers that ARE bits (the sea's discard).
bool ComposedIsLand(float3 dir, float hp, float waterLevel) {
    return ComposedLandness(dir, hp, waterLevel) > 0.5f;
}

// M6p: strength of a hand-edit declaring LAND here (0 where unedited or edited to water).
// Geometry consumers floor their display height with it: an operator's jetty stands as a
// continuous ridge even where the smeared height channel dips under the tide.
float ComposedEditLand(float3 dir) {
    if (gCsU3.x == 0xFFFFFFFFu) return 0.0f;
    const float2 me = CsEditMask(dir);
    return smoothstep(0.2f, 0.8f, me.y) * ((me.x > 0.5f) ? 1.0f : 0.0f);
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
