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
//  step 4g, where each layer embedded a copy in its own cbuffer). Every layer that reads them
//  therefore runs literally the same code on literally the same constants -- the class of bug
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

// THE POINT a window is addressed by: the ground point relative to the level's eye in the tangent
// axes -- the frame VsOut.geo lives in. A stage with only a direction makes it from that, about the
// camera's eye, at the direction's grain (0.43 texel at rung 9, uv_precision.py) and no better --
// CsPointOfDir. (PHASE B3: the Mercator windows and their anchor rows are deleted.)
float3 CsPointOfDir(float3 dir) {
    return (CsToTangent(dir) * gCsF.w - float3(0.0f, gCsF.w, 0.0f)) - gCsEyeT.xyz;
}

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
// The ground texel (metres) at the cube's mip 0: the surface's own row (gCsGround.x, SurfaceFrame::Fill
// from Lattice::GroundRes(0): 611.496..); a rank's is CsBlockGround.
float3 CsGroundM() { return gCsGround.xyz; }
// The height ladder's lod floor (cube lod): rung 9's mip 0 (1.1943 m = 611.496 m / 2^9), rank 3's,
// where the z17 page stood. Every reader that asks for the finest resident height asks for this.
static const float kCsHeightLodFloor = -9.0f;

// M9ap: THE PAGES PATH. One texture, pages selected by CONTAINMENT and by what is actually
// resident: every page is the same megatexture at a different ground resolution, so the page
// whose resident mip gives the finest ground texel at this pixel is the right answer and needs
// no fade against its neighbour -- where two pages are resident at the same resolution they
// hold the same pixels. `hand` and `finer` are gone; there is nothing to hand off between.
// M12 step 4e: the ladder is PageWins on PageGroundM, the rung's resident ground against the
// ground held -- "at least as fine" (the height path's spelling), which with these literals is
// the strict `<` this path used to write: no two rungs' literal grounds are ever equal in float
// (9.55 * 64 = 611.2, not 611; 1.19 * 8 = 9.52, not 9.55). Measured, not assumed.
// PHASE A1 (out/integration/plan_eye_windows.md): THE EYE'S WINDOWS, the one compiled path.
// Rank k + 1's window is a box of 16384 texels of rung 3 (k + 1) about the eye, placed MODULO
// 16384 (HIERARCHY 4.1; hal/Tenant.h's banner). Its rows are PageTexelUv's planes about the eye's
// own tangent frame anchored on the multiple of 16384 nearest the eye, so the address is the
// point's texel less that anchor over 16384 -- the uv a WRAP sampler reads modulo 16384 at every
// mip, whatever the box's origin -- and gCsBlkO the box's origin less the anchor: a point is in
// the box where address + gCsBlkO lies in [0, 1). The point p is the UNDISPLACED ground point
// relative to the eye in that frame -- the mesh stage's geo -- so every number here is small.
// PHASE A2: the eye is the LEVEL's (below).
// A window is read at its mips 0..3 alone (kCsWindowFloor): its own three rungs and its floor.
#ifndef GA_BLOCK_RANKS
#define GA_BLOCK_RANKS 5
#endif
static const float kCsWindowFloor = 3.0f;
// PHASE A2: every level has its own windows, about ITS eye -- slot s of the level table (0 the
// camera, then the Droste levels and the gate worlds), its rank k + 1 at row 5 s + k -- and its
// readers address them by the level's own point (relative to its eye: the mesh stage's geo in that
// level), so two worlds that see one piece of ground read one tile.
float CsBlockGround(uint k) {   // the rank's ground at mip 0, the pyramid's: by selects, no local array
    const float4 g = gCsRankG[k >> 2];
    const uint c = k & 3u;
    return (c == 0u) ? g.x : (c == 1u) ? g.y : (c == 2u) ? g.z : g.w;
}
float2 CsWinUv(uint s, uint k, float3 p) {
    const uint j = 5u * s + k;
    return PageTexelUv(p, gCsWinU[j], gCsWinV[j], gCsWinW[j]);
}
float2 CsWinOff(uint s, uint k) {
    const uint j = 5u * s + k;
    const float4 o = gCsWinO[j >> 1];
    return (j & 1u) != 0u ? o.zw : o.xy;
}
uint CsWinSlice(uint s, uint k) {
    const uint j = 5u * s + k;
    return gCsWinS[j >> 2][j & 3u];
}
// THE CHAIN, ARITHMETIC (Window.hlsli, one body with the selftest's): one ratio and one compare a
// rank, the ranks from 1 up to the first whose box does not hold p.
#define WIN_UV(s, i, p) CsWinUv(s, i, p)
#define WIN_OFF(s, i) CsWinOff(s, i)
#define WIN_SLICE(s, i) CsWinSlice(s, i)
#define WIN_K(s) gCsWinK[(s) >> 2][(s) & 3u]
#include "Window.hlsli"
// The level's chain at its own point: p relative to slot s's eye, in the tangent axes.
WalkChain CsChain(float3 p, uint s) { return WindowChain(p, min(s, 7u)); }
// A rank's residency floor at an address: the gather wraps as the sample does.
float CsHaveWindow(uint mapSrv, float2 uv, uint slice) {
    return PageHave(gTexArr[mapSrv], sLinearWrap, uv, slice);
}
#define CS_WC_PARAM , WalkChain wc
#define CS_WC , wc
// A read at ANOTHER point (the pixel water's bed, where its refracted ray lands) takes that
// point's own chain, in its level (the point relative to that level's eye).
#define CS_WALK_AT(p, s) , CsChain(p, s)

float3 ComposedColorPages(float3 dir, float3 p CS_WC_PARAM) {
    // The cube, through the cube views over slices 0..5 (hardware-seamless across faces).
    const float haveC = CsHaveCubeArr(gCsU.y, dir);
    float3 c = PageSampleCube(gTexCubeArr[gCsU.x], sAniso, dir, haveC).rgb;
    const float3 g0 = CsGroundM();
    float ground = PageGroundM(g0.x, haveC);
    // PHASE A1: the eye's windows, the chain coarsest rank first, by the same ladder: a rank
    // answers where its resident ground is at least as fine as the ground held, read at its mips
    // 0..3 alone -- a rank whose footprint or floor is past its mip 3 leaves the pixel to the rank
    // above, which holds that ground at its mip 0 (the window's FLOOR, HIERARCHY 4.1).
    [unroll] for (uint k = 0; k < GA_BLOCK_RANKS; ++k) {
        if (k >= wc.n) break;
        const uint sl = WalkSlice(wc, k);
        const float2 buv = WalkUv(wc, k);
        const float lodB = gTexArr[gCsU5.x].CalculateLevelOfDetail(sAnisoWrap, buv);
        if (lodB > kCsWindowFloor) break;
        const float haveB = CsHaveWindow(gCsU5.y, buv, sl);
        if (haveB > kCsWindowFloor) continue;
        const float gB = PageGroundM(CsBlockGround(k), haveB);
        if (PageWins(gB, ground)) {
            c = PageSample(gTexArr[gCsU5.x], sAnisoWrap, buv, sl, haveB).rgb;
            ground = gB;
        }
    }
    return c;
}

float3 ComposedColor(float3 dir, float3 p CS_WC_PARAM) {
    if (gCsU5.x != 0xFFFFFFFFu && gCsF.x > 0.5f) return ComposedColorPages(dir, p CS_WC);
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
    // M6j final word on "conversion": NO lift at all. The 1.35 exposure compensation matched
    // the old curve hack at mid-tones but pushed bright land cover (marsh tan) over the
    // tonemapper's shoulder into cream. Pixels ship exactly as the hardware sRGB decode
    // delivers them; scene brightness belongs to the lighting and gExposure alone.
    return c;
}
bool ComposedColorOn() { return gCsF.x > 0.5f; }

// M9av: the luminance of the seafloor's dry sediment ramp. The megatexture's ocean texels are
// DRY seafloor albedo (synth.seafloor.relief: the bathymetry's hillshade times this ramp, keyed
// on datum depth).
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



bool ComposedHeightOn() { return gCsF.z > 0.5f; }

// ---- PHASE B1 (out/integration/plan_phase_b.md): THE COMPOSED HEIGHT OVER THE CHAIN. The height
// on the eye's windows, read as ComposedColorPages reads the colour: the cube through its cube
// views, then the ranks of the reader's own chain coarsest first -- each at the level the reader's
// footprint wants of it, lod + 3 (k + 1) (rank k + 1 is 3 (k + 1) rungs finer than the cube), and
// only to the window's floor: a footprint past it leaves the point to the rank above, which holds
// that ground at its mip 0 (HIERARCHY 4.1) -- taken where it is resident to its floor and at least
// as fine as the ground held. Every height reader takes its level's chain at its OWN point (the
// vertex's, the pixel's, a gradient's four, the bed's cast), so two worlds that see one ground read
// one tile. A sample at an explicit level: vertex-, mesh- and pixel-safe.
static const float kCsHeightWindowFloor = 3.0f;
float ComposedHeightChain(float3 dir, float lod CS_WC_PARAM) {
    if (gCsF.z < 0.5f) return 0.0f;
    if (gCsU6.x == 0xFFFFFFFFu) {   // a height cube alone, no pages (Mars's MOLA)
        return gTexCube[gCsU2.x].SampleLevel(sLinearClamp, dir, max(lod, CsHaveCube(gCsU2.y, dir))).x;
    }
    const float haveC = CsHaveCubeArr(gCsU2.y, dir);
    float h = PageSampleLevelCube(gTexCubeArr[gCsU2.x], sLinearClamp, dir, lod, haveC).x;
    float ground = PageGroundM(CsGroundM().x, haveC);
    [unroll] for (uint k = 0; k < GA_BLOCK_RANKS; ++k) {
        if (k >= wc.n) break;
        const float want = lod + 3.0f * float(k + 1u);
        if (want > kCsHeightWindowFloor) break;
        const uint sl = WalkSlice(wc, k);
        const float2 buv = WalkUv(wc, k);
        const float haveB = CsHaveWindow(gCsU6.y, buv, sl);
        if (haveB > kCsHeightWindowFloor) continue;
        const float gB = PageGroundM(CsBlockGround(k), haveB);
        if (PageWins(gB, ground)) {
            h = PageSampleLevel(gTexArr[gCsU6.x], sLinearWrap, buv, sl, max(want, 0.0f), haveB).x;
            ground = gB;
        }
    }
    return h;
}
// ...at a point of level s: its own chain there.
float ComposedHeightAt(float3 dir, float3 p, float lod, uint s) {
    return ComposedHeightChain(dir, lod, CsChain(p, s));
}
// ComposedHeightGrad's law on the chain: each of the four points walks its own chain in its level.
float2 ComposedHeightGradAt(float3 dir, float3 p, float lod, uint s) {
    const float eps = gCsG.y * exp2(lod);
    float3 eP = cross(float3(0.0f, 1.0f, 0.0f), dir);
    eP = (dot(eP, eP) < 1e-8f) ? float3(1.0f, 0.0f, 0.0f) : normalize(eP);
    const float3 nP = cross(dir, eP);
    const float texM = eps * gCsF.w;
    const float3 sE = CsToTangent(eP) * texM, sN = CsToTangent(nP) * texM;
    const float hE = ComposedHeightAt(normalize(dir + eP * eps), p + sE, lod, s);
    const float hW = ComposedHeightAt(normalize(dir - eP * eps), p - sE, lod, s);
    const float hN = ComposedHeightAt(normalize(dir + nP * eps), p + sN, lod, s);
    const float hS = ComposedHeightAt(normalize(dir - nP * eps), p - sN, lod, s);
    return float2(hE - hW, hN - hS) / (2.0f * texM);
}

// M9ay: THE SURVEY AS PAGES. gis.landsea's own tree -- the vector rings swept per tile on the
// same addresses as the imagery and the bed -- is a page tenant: r = water coverage (1 water,
// 0 land), b = edited (a hand ring decided this texel), a = surveyed (an opinion exists).
// The finest page with an opinion answers: z17 (1.19 m, where the old ~1 m edit raster was),
// z14, then the cube face. No page with an opinion here -> false: the classifier falls back
// to the height sign. The three committed rasters this replaces (window R8G8, global R8,
// fine edit R8G2) read the .raw parity fills the vector mask refuses; they are gone.
bool CsMaskSample(float3 dir, float3 p CS_WC_PARAM, out float4 m) {
    m = float4(0, 0, 0, 0);
    if (gCsU3.x == 0xFFFFFFFFu) return false;
    // PHASE A1: the eye's windows in the pages' order -- the finest with an OPINION answers, the
    // finest rank first -- at their mips 0..3 alone.
    [unroll] for (int k = GA_BLOCK_RANKS - 1; k >= 0; --k) {
        if (uint(k) >= wc.n) continue;
        const uint sl = WalkSlice(wc, uint(k));
        const float2 buv = WalkUv(wc, uint(k));
        const float haveB = CsHaveWindow(gCsU3.y, buv, sl);
        if (haveB <= kCsWindowFloor) {
            m = PageSampleLevel(gTexArr[gCsU3.x], sLinearWrap, buv, sl, 0.0f, haveB);
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
float ComposedLandMask(float3 dir, float3 p CS_WC_PARAM) {
    float4 m;
    if (!CsMaskSample(dir, p CS_WC, m)) return -1.0f;
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
float2 CsEditMask(float3 dir, float3 p CS_WC_PARAM) {
    float4 m;
    if (!CsMaskSample(dir, p CS_WC, m)) return float2(0.0f, 0.0f);
    const float a = max(m.a, 0.001f);
    return float2(1.0f - m.r / a, m.b / a);
}

float ComposedLandness(float3 dir, float3 p CS_WC_PARAM, float hp, float waterLevel) {
    const float lm = ComposedLandMask(dir, p CS_WC);
    float land = (lm >= 0.0f) ? ((lm > 0.5f) ? 1.0f : 0.0f)
                              : ((hp > waterLevel) ? 1.0f : 0.0f);
    // M6p: HAND EDITS ARE LAW. The window mask is R8G8 -- g flags texels painted by
    // data/gis/edits.geojson, and a flagged texel's mask value overrides survey and the
    // live tide alike (the survey shoreline predates the jetties, and the stabilized height
    // classifier smears their thin ridges -- the operator's polygon settles it). Bilinear g
    // blends the override's own edge.
    if (gCsU3.x != 0xFFFFFFFFu) {
        const float2 me = CsEditMask(dir, p CS_WC);
        land = lerp(land, (me.x > 0.5f) ? 1.0f : 0.0f, smoothstep(0.2f, 0.8f, me.y));
    }
    return land;
}
// The binary view, for consumers that ARE bits (the sea's discard).
bool ComposedIsLand(float3 dir, float3 p CS_WC_PARAM, float hp, float waterLevel) {
    return ComposedLandness(dir, p CS_WC, hp, waterLevel) > 0.5f;
}

// M6p: strength of a hand-edit declaring LAND here (0 where unedited or edited to water).
// Geometry consumers floor their display height with it: an operator's jetty stands as a
// continuous ridge even where the smeared height channel dips under the tide.
float ComposedEditLand(float3 dir, float3 p CS_WC_PARAM) {
    if (gCsU3.x == 0xFFFFFFFFu) return 0.0f;
    const float2 me = CsEditMask(dir, p CS_WC);
    return smoothstep(0.2f, 0.8f, me.y) * ((me.x > 0.5f) ? 1.0f : 0.0f);
}

// M6i debug: the alignment overlay (--stencil). The survey VECTORS render as real line
// geometry (GisLayer) -- this shader-side part draws what must be compared against them:
//   red     OUR composed height channel's zero-crossing (thin, fwidth-scaled)
//   white   0.05-degree graticule (PHASE B3: the Mercator window's frame is gone)
float3 ApplyComposedStencil(float3 col, float3 dir, float3 p) {
    if (gCsG.w < 0.5f) return col;
    const float h = ComposedHeightAt(dir, p, kCsHeightLodFloor, 0u);   // finest RESIDENT height everywhere
    const float fw = max(fwidth(h), 0.05f);
    const float coastH = 1.0f - smoothstep(1.0f * fw, 2.5f * fw, abs(h));
    col = lerp(col, float3(1.0f, 0.12f, 0.10f), coastH * 0.8f);
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
// sampler, with no branch anywhere. The floor is kCsHeightLodFloor (above).
float ComposedHeightLod(float dist, float pixAngRad) {
    const float texelM = gCsG.y * gCsF.w;
    const float pixM = dist * pixAngRad;
    return clamp(log2(max(pixM / texelM, exp2(kCsHeightLodFloor))), kCsHeightLodFloor, gCsG.x);
}


#endif  // GA_COMPOSE_HLSLI
