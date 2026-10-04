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
// M10 THE DROSTE GAUGE (src/core/Droste.h). A record of level k is the ROOT seen from the eye
// S^-k(C): the mesh and pixel stages run the unchanged root shading in the level's OWN frame,
// with that level's eye and sun, and only the clip position is mapped back to the true frame
// (s^k Q^k). These statics carry the level being drawn; LoadLevel below fills them from the
// level table at the top of every entry point. Declared before Common.hlsli so its sky and haze
// helpers read the level's sun and eye height, not the scene's.
static float3 sLvlSun = float3(0.0f, 1.0f, 0.0f);
static float sLvlEyeY = 0.0f;
static float3 sLvlSkyUp = float3(0.0f, 1.0f, 0.0f);
// M13: and the distance of the level's own eye from its planet's centre, which is where its sky
// is marched from (Common.hlsli SkyAirAt). The gate's window is a level: its rays land at the far
// place, so its surfaces reflect the far place's sky with nothing more said.
static float sLvlSkyEyeR = 6371000.0f;
#define GA_SKY_EYE_R (sLvlSkyEyeR)
#define GA_SUN_DIR (sLvlSun)
#define GA_EYE_Y (sLvlEyeY)
#define GA_SKY_UP (sLvlSkyUp)
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
    // rows, shared VERBATIM with every other layer that samples this planet's surface, are
    // Common.hlsli's SurfaceCb (b2) since M12 step 4g -- one buffer, not a copy in this one.
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
    // M9c: the wavenumber the FOLD judges each band by (energy-weighted over the live
    // spectrum). gBankC keeps the band's geometric midpoint for the prefilter and the
    // caustic assembly, whose own proofs pin that number. Appended at the END.
    float4 gBankFold;
    // M9h: the grad(flow) lens. gLensU.x = the derived (div, curl) bank's SRV; gLensA maps
    // world XZ onto the bathy grid the SWE solver runs on (row 0 NORTH, so v flips at the
    // sample -- the same convention Sea.hlsl's BathyUv declares).
    uint4  gLensU;
    float4 gLensA;      // org x, org z, 1/sizeX, 1/sizeZ
    float4 gLensB;      // M9h: x = bank texel metres, yz = residency-map dims, w spare
    float4 gLensR;      // M9j: region page -- lon0, lat0, 1/spanLon, 1/spanLat
    // M10 THE DROSTE LEVELS -- appended at the END, per the layout law (priors 22).
    float4 gDrosteA;      // x = levels in the table, y = the camera's absolute level,
                          // z = lighting (0 realistic, 1 appealing), w = portal shadow on
    float4 gDrostePortal; // the inner globe in ANY level's own frame: centre xyz, radius (m)
    uint4  gBankBU;       // set B (the outer level's rings): disp, param, detail SRVs, on
    float4 gBankBOrg01;   // set B ring origins, as gBankOrg01..45
    float4 gBankBOrg23;
    float4 gBankBOrg45;
    float4 gDroste[48];   // 8 levels x 6 rows -- see LoadLevel
    // THE VIEW'S WINDOWS (scene/Gateway.h WindowChain): x = the first window level's slot (-1
    // none), y = how many windows deep; the chain, packed as scene/WindowBox.h packs it.
    float4 gGateA;
    float4 gGateBox[28];
    // M13 step 2: the cascade sea's plane at the eye (sim/WaveChart.h) -- see GlobeLayer.h.
    // Said in the TANGENT frame about the tangent point, per cascade (ChartUOf below).
    float4 gChartOrg;   // xyz = the constant along east, wrapped to cascade 0 / 1 / 2's patch;
                        // w = 1 when these rows are live
    float4 gChartE;     // the cell's east in the tangent frame; w = the constant along north,
                        // wrapped to cascade 0's patch
    float4 gChartN;     // its north in the tangent frame; w = the same, cascade 1
    // Appended at the END (priors 22): the constant along north, wrapped to cascade 2; yzw spare.
    float4 gChartCn;
};

// A point's coordinate in that plane, for cascade c: WaveChart::UOf, u = (P - org) . e + off,
// said about the TANGENT POINT A = R up. The chart's law is written in the planet frame and this
// stage holds its points in the tangent frame, so the CPU turns the chart's axes into the tangent
// frame and takes the constants (A - org) . e + off there, in doubles, wrapped to each cascade's
// patch before the cast (the patches are periodic, and a constant of 1e5 m has a float grain of
// 8 mm). `q` is the point less A, in tangent axes. (REVIEW finding 7: handed a tangent-frame point
// against the planet-frame rows, this read a coordinate unrelated to the one the bank's kernel
// filled its texels by -- tools/hierarchy/ripple_chart.py.)
float2 ChartUOf(float3 q, uint c) {
    const float cn = (c == 0u) ? gChartE.w : ((c == 1u) ? gChartN.w : gChartCn.x);
    return float2(dot(q, gChartE.xyz) + gChartOrg[c], dot(q, gChartN.xyz) + cn);
}

// ---- M10: the level being drawn (LoadLevel) --------------------------------------------------
// Six rows per level, filled by GlobeLayer::SetView:
//   0  the eye in the level's own frame, sphere-centred (replaces gCamAbs)   | s^k
//   1  Q^k row 0 (own -> true rotation)                                      | relief exaggeration
//   2  Q^k row 1                                                             | the eye's flat y
//   3  Q^k row 2                                                             | bank set (0, 1, -1)
//   4  the sun in the level's own frame                                      | spare
//   5  the zenith of the sky this level SEES, own frame                      | that sky's daylight
//      (< 0: the level's own day -- every level but a realistic inner one)
static float3 sLvlCamAbs = float3(0.0f, 0.0f, 0.0f);
static float sLvlSigma = 1.0f;
static float3x3 sLvlQ = float3x3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
static float sLvlExag = 1.0f;
static int sLvlBank = 0;
static float sLvlSkyDay = -1.0f;

void LoadLevel(uint slot) {
    const uint b = min(slot, 7u) * 6u;
    const float4 r0 = gDroste[b], r1 = gDroste[b + 1u], r2 = gDroste[b + 2u];
    const float4 r3 = gDroste[b + 3u], r4 = gDroste[b + 4u], r5 = gDroste[b + 5u];
    sLvlCamAbs = r0.xyz;
    sLvlSigma = r0.w;
    sLvlQ = float3x3(r1.xyz, r2.xyz, r3.xyz);
    sLvlExag = r1.w;
    sLvlEyeY = r2.w;
    sLvlBank = (int)round(r3.w);
    sLvlSun = r4.xyz;
    sLvlSkyUp = r5.xyz;
    sLvlSkyEyeR = length(r0.xyz);   // the level's eye, sphere-centred, in its own units
    sLvlSkyDay = r5.w;
}

// THE SKY'S DAYLIGHT, as the mirror and the skylight see it. A planet's sky is lit by that
// planet's sun, so its brightness goes with the local `day` -- except inside a realistic Droste
// tower, where the sky above an inner level is the ROOT's, lit by the root's day, whatever the
// inner planet's own terminator says. A mirror reflects the sky it sees: the inner night-side sea
// under the root's noon is a mirror of a bright sky, not a black one.
float SkyDay(float localDay) { return (sLvlSkyDay < 0.0f) ? localDay : sLvlSkyDay; }

// The gauge's one outward step: a camera-relative vector in the level's own frame, as the true
// camera sees it. s^k Q^k -- the similarity with its translation already cancelled by the eye.
float3 TrueRel(float3 relOwn) { return sLvlSigma * mul(sLvlQ, relOwn); }

// THE VIEW'S WINDOWS. A level's depth: 0 for the eye's own world and every Droste level, k for the
// world seen through k windows. A surface pixel is kept by the level whose depth its ray reaches
// (Common.hlsli WindowChainDepth) -- the slab walk the sky and the hulls ask too, so no two passes
// can disagree about where a window's edge is.
uint LevelGateDepth(uint lvl) {
    if (gGateA.x < 0.0f) return 0u;
    const uint first = (uint)gGateA.x;
    const uint n = (uint)gGateA.y;
    return (lvl >= first && lvl < first + n) ? lvl - first + 1u : 0u;
}
uint GateDepth(float3 p) { return WindowChainDepth(p, gGateBox, (uint)gGateA.y); }

// THE RIM: how near the ray's entry into window k lies to an EDGE of the face it enters through,
// 1 on the edge falling to 0 a rim-width in. A window between two open seas is otherwise hard to
// find; the rim is drawn on the pixels seen through that window only, so its far side never
// shows it -- and down a corridor of windows every frame has its own.
float GateRim(float3 p, uint k) {
    const uint b = min(k, kWindowChain - 1u) * 4u;
    const float3x3 R = float3x3(gGateBox[b].xyz, gGateBox[b + 1u].xyz, gGateBox[b + 2u].xyz);
    const float3 h = float3(gGateBox[b].w, gGateBox[b + 1u].w, gGateBox[b + 2u].w);
    const float3 e = mul(R, -gGateBox[b + 3u].xyz);
    float3 d = mul(R, p);
    d = lerp(d, float3(1e-12f, 1e-12f, 1e-12f), float3(abs(d) < 1e-12f));
    const float3 t1 = (-h - e) / d;
    const float3 t2 = (h - e) / d;
    const float3 tn = min(t1, t2);
    const float tEnter = max(max(max(tn.x, tn.y), tn.z), 0.0f);
    const float3 q = e + tEnter * d;                   // the entry point, box frame
    const float3 inset = h - abs(q);                   // >= 0 inside; ~0 on the entered face
    // The entered face's axis is the smallest inset; the rim is the next smallest.
    const float lo = min(min(inset.x, inset.y), inset.z);
    const float hi = max(max(inset.x, inset.y), inset.z);
    const float mid = inset.x + inset.y + inset.z - lo - hi;
    const float width = max(0.35f, 0.004f * length(tEnter * p));
    return 1.0f - saturate(mid / width);
}

// THE INNER GLOBE'S SHADOW. The next level down sits in every level at the same place in that
// level's own frame (gDrostePortal), so one analytic sphere test shades the whole tower: the sun
// seen from P is eclipsed by the globe's disc, softened over the sun's own angular radius. One
// expression -- at the contact (d -> r) the globe's disc fills the half-sky and the ground is in
// shade; far away the disc shrinks to nothing and so does the shadow. The disc's share past the
// rim is Common.hlsli's SunDiscClear, the same ramp the planet's own shadow uses (PlanetShadow):
// two spheres, one law.
float PortalShadow(float3 pOwnFlat) {
    const float rad = gDrostePortal.w;
    const float3 toC = gDrostePortal.xyz - pOwnFlat;
    const float d = max(length(toC), 1e-6f);
    const float alpha = asin(saturate(rad / d));                  // the globe's angular radius
    const float theta = acos(clamp(dot(toC / d, GA_SUN_DIR), -1.0f, 1.0f));
    return (rad > 0.0f) ? SunDiscClear(theta - alpha) : 1.0f;
}

// Sample the bank at a world-frame XZ: finest ring containing the point wins. Returns false
// beyond every ring (the far field: sub-pixel waves, level ~ the plane).
// ---- THE BANK'S RECONSTRUCTION KERNEL (ALGEBRA.md "caustics": the tangent bivector) -------
//
// The bank stores the wave surface on a ring lattice. What the mesh and the pixels actually
// need is the surface BETWEEN those texels, and the kernel that fills the gap decides whether
// the water reads as water or as a heightfield.
//
// WHAT WAS WRONG. This was a tent (manual bilinear): C0. The value is continuous across a
// texel border and the GRADIENT is not, so the surface carries a slope discontinuity along
// every texel edge -- and the normal was then taken as a FORWARD DIFFERENCE over one whole
// texel, which is piecewise constant per cell. Geometry creased on the lattice and shading
// stepped with it: the comb of parallel ridges down a storm wave's face and the dead-straight
// slope crease at a ring handover (rail_1199, the storm rail's last frame). Neither is in the
// data; both are the kernel.
//
// THE LAW. One cubic (Catmull-Rom) convolution, applied uniformly -- no case split, no
// threshold, no special near field. It is INTERPOLATING, so the sampled surface is preserved
// exactly at texel centres and the rendered Hs gate does not move (a B-spline would smooth the
// data itself and shed amplitude the fold has already accounted for). Its first derivative is
// continuous, which is precisely what the facets were the absence of.
//
// The derivative is ANALYTIC and comes from the SAME 16 taps -- the kernel differentiated, not
// the surface re-probed. That retires the two extra ring probes each shading path used to take
// (priors 21's "a derivative at the DATA's grain" is honoured better here than by a difference:
// the kernel's support IS the grain, and it is continuous). Net taps per shaded point FALL,
// 36 -> 24, because one evaluation now answers what three probes used to.
//
// Loads, not SampleLevel: priors 1 -- a bindless SampleLevel reads ZERO outside the pixel
// stage on this driver, and this runs in the mesh stage.
//
// M10: TWO RING SETS. The bank is a camera-anchored ladder, and under the Droste gauge each
// level has its own eye. Set A is anchored at the camera level's eye (as it always was); set B
// at the OUTER level's eye (S(C)), so the sea the camera's planet floats in carries its own
// waves. A level with no set (-1: every globe smaller than the camera's own) is seen from far
// enough that the fold has already shed its waves into sigma^2 -- its rings would sit beyond
// the last one anyway, which is the same answer.
float2 BankRingOrg(uint m) {
    const bool b = sLvlBank == 1;
    const float4 o01 = b ? gBankBOrg01 : gBankOrg01;
    const float4 o23 = b ? gBankBOrg23 : gBankOrg23;
    const float4 o45 = b ? gBankBOrg45 : gBankOrg45;
    return (m == 0) ? o01.xy : (m == 1) ? o01.zw : (m == 2) ? o23.xy
         : (m == 3) ? o23.zw : (m == 4) ? o45.xy : o45.zw;
}
uint BankDispSrv() { return (sLvlBank == 1) ? gBankBU.x : gBankU.x; }
uint BankParamSrv() { return (sLvlBank == 1) ? gBankBU.y : gBankU.y; }
uint BankDetailSrv() { return (sLvlBank == 1) ? gBankBU.z : gBankU2.x; }
bool BankSetLive() {
    return gBankU.z != 0u && sLvlBank >= 0 && (sLvlBank == 0 || gBankBU.w != 0u);
}

float4 CatmullW(float t) {
    const float t2 = t * t, t3 = t2 * t;
    return float4(-0.5f * t3 + t2 - 0.5f * t,
                   1.5f * t3 - 2.5f * t2 + 1.0f,
                  -1.5f * t3 + 2.0f * t2 + 0.5f * t,
                   0.5f * t3 - 0.5f * t2);
}
// d/dt of the above: the tangent, exact, same support.
float4 CatmullDW(float t) {
    const float t2 = t * t;
    return float4(-1.5f * t2 + 2.0f * t - 0.5f,
                   4.5f * t2 - 5.0f * t,
                  -4.5f * t2 + 4.0f * t + 0.5f,
                   1.5f * t2 - 1.0f * t);
}

// One ring's fetch. disp is reconstructed by the cubic above and carries its own world-space
// Jacobian columns dDdx = dD/dx, dDdz = dD/dz (metres per metre, all three components -- the
// lateral Gerstner terms included, which is what makes the wedge below a real tangent
// bivector and not a heightfield approximation). param/detail stay on the tent: they are
// shading scalars read at their own grain, and no geometry hangs off their gradient.
// edgeD = distance to the ring's valid border in texels, for the cross-fade below.
bool BankFetch(uint m, float2 worldXZ, out float4 disp, out float4 param,
               out float4 detail, out float texelOut, out float edgeD,
               out float3 dDdx, out float3 dDdz) {
    disp = 0.0f;
    param = 0.0f;
    detail = 0.0f;
    texelOut = 0.0f;
    edgeD = 0.0f;
    dDdx = 0.0f;
    dDdz = 0.0f;
    const float texel = gBankA.x * (float)(1u << m);
    const float2 org = BankRingOrg(m);
    const float2 local = (worldXZ - org) / texel;
    // The cubic reaches one texel further than the tent on each side, so the valid window
    // loses one texel at each border. The ring cross-fade already lives 48 texels inside it.
    if (any(local < 2.0f) || any(local > 510.0f)) return false;
    edgeD = min(min(local.x - 2.0f, 510.0f - local.x),
                min(local.y - 2.0f, 510.0f - local.y));
    const float2 tf = local - 0.5f;
    const int2 t0 = int2(floor(tf));
    const float2 fr = tf - float2(t0);
    const int xoff = (int)m * 512;

    // param/detail: the tent, unchanged (4 taps).
    const uint srvD = BankDispSrv(), srvP = BankParamSrv(), srvT = BankDetailSrv();
    [unroll] for (int k = 0; k < 4; ++k) {
        const int2 tc = t0 + int2(k & 1, k >> 1);
        const float wgt = ((k & 1) ? fr.x : 1.0f - fr.x) * ((k >> 1) ? fr.y : 1.0f - fr.y);
        param += wgt * gTex[srvP][int2(xoff + tc.x, tc.y)];
        detail += wgt * gTex[srvT][int2(xoff + tc.x, tc.y)];
    }

    // disp: the cubic, value and both tangents from one 4x4 support.
    const float4 wx = CatmullW(fr.x), wz = CatmullW(fr.y);
    const float4 dx = CatmullDW(fr.x), dz = CatmullDW(fr.y);
    [unroll] for (int j = 0; j < 4; ++j) {
        float4 row = 0.0f, rowDx = 0.0f;
        [unroll] for (int i = 0; i < 4; ++i) {
            const int2 tc = t0 + int2(i - 1, j - 1);
            const float4 s = gTex[srvD][int2(xoff + tc.x, tc.y)];
            row += wx[i] * s;
            rowDx += dx[i] * s;
        }
        disp += wz[j] * row;
        dDdx += wz[j] * rowDx.xyz;
        dDdz += dz[j] * row.xyz;
    }
    // d/d(texel) -> d/d(metre).
    dDdx /= texel;
    dDdz /= texel;
    texelOut = texel;
    return true;
}

// ---- M8 FOAM BREAKUP (ALGEBRA.md foamlaw; the reference's range-faded octaves) ----
// M9bg: CURRENTLY UNREFERENCED. This textured the bank's physics coverage per pixel; the
// vertex-shaded water takes the bank's foam number as it comes. Kept because foamlaw and the
// reference's 30.7% -> 2.1% gouache measurement still describe the shipped bank kernel.
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

bool BankSampleT(float2 worldXZ, out float4 disp, out float4 param, out float4 detail,
                 out float texelOut, out float3 dDdx, out float3 dDdz) {
    disp = 0.0f;
    param = 0.0f;
    detail = 0.0f;
    texelOut = 0.0f;
    dDdx = 0.0f;
    dDdz = 0.0f;
    if (!BankSetLive()) return false;
    [unroll] for (uint m = 0; m < 6; ++m) {
        float eD;
        if (!BankFetch(m, worldXZ, disp, param, detail, texelOut, eD, dDdx, dDdz)) continue;
        // M8 THE RING CROSS-FADE (the user's "interpolation param", data/wave_scene.json
        // ringBlendTexels): near this ring's border, blend toward the next coarser ring
        // so the texture handover never pops -- the M6t fold already conserves the
        // ENERGY across rings; this interpolates the REALIZATION too.
        const float blendW = max(gBankE.x, 1.0f);
        if (m < 5u && eD < blendW) {
            float4 d2, p2, det2;
            float t2, e2;
            float3 dx2, dz2;
            if (BankFetch(m + 1u, worldXZ, d2, p2, det2, t2, e2, dx2, dz2)) {
                const float w = saturate(eD / blendW);
                disp = lerp(d2, disp, w);
                param = lerp(p2, param, w);
                detail = lerp(det2, detail, w);
                texelOut = lerp(t2, texelOut, w);
                // The tangents are already in metres per metre on BOTH rings, so the same
                // weight carries them: the handover is C0 in the slope instead of a step,
                // which is what the dead-straight crease at a ring border actually was.
                dDdx = lerp(dx2, dDdx, w);
                dDdz = lerp(dz2, dDdz, w);
            }
        }
        return true;
    }
    return false;
}

// The old signature, for the call sites that want the value and no frame.
bool BankSample(float2 worldXZ, out float4 disp, out float4 param, out float4 detail,
                out float texelOut) {
    float3 dx, dz;
    return BankSampleT(worldXZ, disp, param, detail, texelOut, dx, dz);
}

// ---- THE TANGENT BIVECTOR ------------------------------------------------------------------
// T = t_x ^ t_z of the displaced surface, where the displacement's Jacobian columns come from
// the kernel above. In the local {east, up, north} frame the two tangents are
//     t_x = (1 + dDx/dx,  dDy/dx,      dDz/dx)
//     t_z = (dDx/dz,      dDy/dz,  1 + dDz/dz)
// and the lighting normal is the bivector's DUAL. One object, one evaluation -- ALGEBRA.md
// "caustics": the same grade-2 quantity whose horizontal part is areaJac, so when the caustic
// path wants the ray map's first factor it is already here rather than re-derived.
//
// The 1.1 slope cap stays exactly where it was: shoaling gain can exceed any real wave face,
// and uncapped facets render as dark back-face speckle (the two-sided failure).
// The bivector reduced to the slope pair the shading paths carry (the pixel path then ADDS
// the sub-texel cascade bands onto it before building its own normal, so the slope -- not the
// finished normal -- is the shared currency).
float2 BankSlope(float3 dDdx, float3 dDdz) {
    const float3 tx = float3(1.0f + dDdx.x, dDdx.y, dDdx.z);
    const float3 tz = float3(dDdz.x, dDdz.y, 1.0f + dDdz.z);
    // dual(t_x ^ t_z) in the tangent basis; x,z are the horizontal axes and y is up.
    const float3 nL = cross(tz, tx);
    const float uy = max(abs(nL.y), 1e-4f);
    float2 s = float2(-nL.x / uy, -nL.z / uy);
    const float sl = length(s);
    if (sl > 1.1f) s *= 1.1f / sl;
    return s;
}
float3 BankNormal(float3 dDdx, float3 dDdz, float3 east, float3 upT, float3 north) {
    const float2 s = BankSlope(dDdx, dDdz);
    return normalize(upT - east * s.x - north * s.y);
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

// ---- M9bg: THE WATER IS VERTEX-SHADED (the user's contract: "old school vertex shaded water
// keeping the nice geometry"). The sea's colour is computed ONCE PER VERTEX here, from the
// vertex's own wave normal and the analytic sky, and the triangle interpolates it. Nothing
// paints the water in the pixel stage any more: no imagery, no ocean-colour retrieval, no
// bed cast, no caustics, no foam-breakup octaves, no per-pixel normals -- PsMain's whole
// water branch retired into these forty lines.
//
// THE GEOMETRY IS UNTOUCHED. The wave bank still displaces every vertex (level + folded
// cascades + the lateral Gerstner term) and the meshlet/CDLOD density is what it was; the
// only bank reads here are the two finite-difference probes that give this vertex its wave
// NORMAL, and those are Loads (BankFetch), which is what makes this legal outside the pixel
// stage at all -- priors 1: a bindless SampleLevel returns ZERO in the mesh stage, so a
// vertex-shaded sea may not sample. It does not need to: every term below is a closed form.
//
// h is the vertex's own composed height, already fetched for displacement -- the shelf tint
// rides it for free, so shallows still read green without a single extra fetch.
float3 WaterVertexColor(float3 dir, float3 rel, float h) {
    const float3 upT = CsToTangent(dir);
    const float3 v = normalize(-rel);
    const float3 axisT = float3(gCsR0.y, gCsR1.y, gCsR2.y);
    float3 east = cross(axisT, upT);
    east = (dot(east, east) < 1e-8f) ? float3(1.0f, 0.0f, 0.0f) : normalize(east);
    const float3 north = cross(upT, east);

    // The wave normal at THIS VERTEX: two finite differences of the bank's own displacement,
    // at the ring's texel. Same arithmetic the pixel shader used to run per pixel, and the
    // same 1.1 slope cap (shoaling gain can exceed any real wave face; uncapped facets
    // render as dark back-face speckle).
    float3 nW = upT;
    float s2 = 0.0300f;    // plain wind-sea slope variance where no ring covers this vertex
    float foam = 0.0f;
    {
        float4 bD, bP, bDet;
        float bT;
        float3 bDdx, bDdz;
        const float2 wxz = (upT * gGlo.x).xz;
        if (BankSampleT(wxz, bD, bP, bDet, bT, bDdx, bDdz)) {
            // THE FOLD, AT VERTEX DENSITY (ALGEBRA.md "fold"). The bank's sigma^2 floors at
            // 0.0015 because a PIXEL could resolve a lobe that sharp. A VERTEX cannot: a
            // highlight narrower than the triangle it lands on interpolates into hard white
            // facets (seen at the helm -- the sun's path came out as a staircase of blocks).
            // Same law as everywhere else in this engine: what the sampling cannot resolve is
            // not deleted, it sheds into the variance. The floor is the open-ocean Cox-Munk
            // value, so the sun's path stays a broad sheen the mesh can actually carry.
            s2 = max(bP.y, 0.0260f);
            foam = saturate(bD.w);
            // The wave normal at THIS VERTEX: the tangent bivector's dual, analytic from the
            // reconstruction kernel's own derivative. The two extra ring probes this used to
            // take are gone -- and with them the piecewise-constant normal that made a storm
            // face read as a staircase.
            nW = BankNormal(bDdx, bDdz, east, upT, north);
        }
    }

    // Depth tint from the vertex's own height: navy offshore, green over the shallows.
    const float shelf = saturate(1.0f + h / 45.0f);
    float3 alb = lerp(float3(0.008f, 0.030f, 0.080f), float3(0.055f, 0.28f, 0.31f),
                      shelf * shelf);

    // The sky mirror, on the reflected ray, horizon-clamped: a facet on the back of a steep
    // wave reflects BELOW the horizon and the sky model darkens there, which used to paint
    // grey patches over wave backs.
    const float fres = 0.02f + 0.98f * pow(1.0f - saturate(dot(v, nW)), 5.0f);
    const float3 dIn = -v;
    float3 rDir = normalize(dIn - 2.0f * dot(dIn, nW) * nW);
    const float rUp = dot(rDir, upT);
    if (rUp < 0.02f) rDir = normalize(rDir + (0.02f - rUp) * upT);

    // The sun's highlight: the Cox-Munk lobe on the vertex normal, sigma^2 from the bank.
    // M10: the level's own sun, eclipsed by the inner globe where its disc covers the sun --
    // and by the planet under this vertex once it has set there (PlanetShadow): the wave
    // normal shapes the lobe, it does not decide whether there is a sun to shape.
    const float sunVis = PortalShadow(sLvlCamAbs + rel - float3(0.0f, gGlo.x, 0.0f)) *
                         PlanetShadow(upT, GA_SUN_DIR, h, gGlo.x);
    const float3 hv = normalize(v + GA_SUN_DIR);
    const float ch = saturate(dot(hv, nW));
    const float tt = max(1.0f - ch * ch, 0.0f) / max(ch * ch, 1e-4f);
    float spec = exp(-tt / s2) / (4.0f * kPi * s2 * max(ch * ch * ch * ch, 1e-4f));
    spec *= (0.02f + 0.98f * pow(1.0f - saturate(dot(v, hv)), 5.0f)) *
            saturate(dot(GA_SUN_DIR, nW)) * sunVis;

    // Energy splits, not doubles: the body dims by the Fresnel the mirror takes. Foam rides
    // ON the water, after the split.
    alb *= 1.0f - fres;
    alb = lerp(alb, float3(0.945f, 0.965f, 0.975f), foam * gBankD.w);

    const float day = saturate(dot(GA_SUN_DIR, upT) * 3.0f + 0.12f);
    const float ndl = saturate(dot(nW, GA_SUN_DIR)) * sunVis;
    float3 col = alb * (0.030f + ndl * SUN_IRR_C * 1.15f);
    col += spec * SUN_IRR_C * 0.85f;
    col += SkyRadianceDirDiscless(rDir, SkyDay(day)) * (fres * 0.9f * (1.0f - foam));
    col += alb * float3(0.010f, 0.014f, 0.028f) * (1.0f - day);   // moonlit-blue night side
    return col;
}

struct VsOut {
    float4 pos : SV_Position;
    float3 rel : TEXCOORD0;    // camera-relative position
    float3 dir : TEXCOORD1;    // unit radial (the sphere normal)
    float  h   : TEXCOORD2;    // relief metres (negative = ocean floor)
    // M9b: which amplification unit drew this pixel -- the meshlet record on the MS path,
    // the CDLOD node instance on the VS fallback. nointerpolation: it is an identity, not a
    // quantity. Read only by PsMeshlet; PsMain ignores it, so the shipped shading is
    // byte-identical.
    nointerpolation uint mid : TEXCOORD3;
    // M9bg: the water's finished colour, shaded at this vertex and INTERPOLATED across the
    // triangle -- old-school Gouraud. PsMain reads it; it never recomputes it.
    float3 wcol : TEXCOORD4;
    // M10: which Droste level drew this fragment -- a slot in the level table. rel above is in
    // THAT level's own frame (the gauge), so PsMain must load the level before it reads rel.
    nointerpolation uint lvl : TEXCOORD5;
    // THE WATER'S SAMPLE POINT (REVIEW finding 42): the undisplaced point of the geoid under this
    // fragment, eye-relative in the level's own tangent frame. The water is read at the level's
    // eye plus this -- the bank's rings and the ripples -- and not at a point made from dir, a
    // float32 direction whose own grain is about 0.27 m of ground at the Merrimack: turning it
    // into the tangent frame cancels two numbers near one half to find one near 1e-5, and a helm
    // pixel is 1 to 3 cm across. The mesh stage makes this from a fine meshlet's
    // double-precision anchor and small offsets, as it makes rel.
    float3 geo : TEXCOORD6;
};

#ifndef GA_MESH_PATH
VsOut VsMain(uint vid : SV_VertexID, uint inst : SV_InstanceID) {
    LoadLevel(0u);   // M10: the fallback path draws the camera's own level only
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
    // The address: this path has no double-precision anchor, so its point is the direction's
    // (CsPointOfDir, the grain the blocks' fallback takes too).
    float h = ComposedHeight(dir, CsPointOfDir(dir), vlod);

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
    o.mid = inst;
    o.lvl = 0u;
    o.dir = dir;   // PLANET frame: texturing (cube samples, lat/lon) stays untouched
    o.h = h;
    // The ocean surface renders AT the geoid; land rides the (altitude-scaled) exaggeration.
    // M6g: rotate the unit direction (exact in float), scale, subtract the sphere-centred
    // camera -- the same cancellation profile the planet frame had, now in ONE shared frame.
    const float3 dirT = CsToTangent(dir);
    o.rel = dirT * (gGlo.x + max(h, 0.0f) * gGlo.y) - gCamAbs.xyz;
    // The water's sample point, undisplaced: made the way rel is made, with rel's precision --
    // this path has no double-precision anchor to make it from.
    o.geo = dirT * gGlo.x - gCamAbs.xyz;
    o.wcol = WaterVertexColor(dir, o.rel, h);   // M9bg: the sea, shaded here and nowhere else
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
// M9bh: REFERENCED AGAIN, by WaterPixelColor alone (--pixel-water). The vertex-shaded default
// still does not read it -- a Gouraud sea carries no texture and no ray to attenuate -- so this
// is the retrieval's only renderer consumer, and it is one a flag can switch off.
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

// ---- M9bh: THE WATER, PIXEL-SHADED -- the two rays, and nothing else -------------------------
// The shipped sea is VERTEX-shaded (WaterVertexColor above, M9bg): one colour per vertex,
// Gouraud across the triangle, and it stays the default. This is the other look, selected by
// --pixel-water (gOptU.w). It exists per PIXEL because the thing it draws cannot be
// interpolated: a RAY. A vertex can carry a colour; it cannot carry what the eye sees THROUGH
// the water, because the refracted ray leaves every pixel in its own direction and lands
// somewhere else on the bed. Interpolating two vertices' bed hits does not give the bed
// between them -- it gives a smear that slides when the wave moves.
//
// docs/ALGEBRA.md "radiometry": a water pixel is a Fresnel split between two rays, and both
// are answered by quadtrees this engine already realizes -- no BLAS, no second scene
// description, no acceleration structure to keep in sync with the water it describes.
//
//   L = F L_sky(r) + (1-F)[ T_w (*) rho_bed E_bed + (1 - T_w) (*) C_scatter ] + L_glint
//
//   * the REFLECTED ray r = -n d n -- the Cl(3) versor sandwich (ALGEBRA "cl3"), answered by
//     the analytic sky, horizon-clamped (a facet on the back of a steep wave reflects BELOW
//     the horizon, where the sky model correctly darkens, and that painted grey patches);
//   * the REFRACTED ray t = R d ~R -- the Snell rotor in the incidence bivector (d ^ n); the
//     closed form below IS that sandwich expanded. It marches into the water and lands on the
//     BED by 2 secant steps against the composed height quadtree, and the bed wears the
//     composed IMAGERY as its albedo;
//   * Beer-Lambert over the REAL path (down along the ray + diffuse up) at the MEASURED K_d
//     (ALGEBRA "optics"; priors 13 -- the channel ordering is a retrieval, not a constant),
//     so the water is TRANSLUCENT where it is thin and collapses to the scattering endpoint
//     where it is deep. The far field is untouched by construction: past 30 m footprints the
//     cast retires to its own vertical closed form, and the two agree where they meet.
//
// WHAT IS DELIBERATELY ABSENT -- the contract for this path is "just glassy waves with no
// foam": no foam of any kind (no bank foam channel, no cascade foam channel, no FoamBreakup
// octaves, no churn atlas, no whitecap albedo, no storm-belt whitening), no sea ice, no
// seafloor relief modulation, no caustics, no peak shaping. Glass, over water, over a bed.
//
// TWO NORMALS, and the split matters (M8, the "leopard" the user rolled back): nSmooth is
// band-limited to the ring TEXEL and shades the BODY -- per-chop diffuse rendered every 2 m
// wavelet as a dark fleck from altitude. nPix carries the cascade bands this pixel resolves
// and feeds the glint, the Fresnel split and BOTH rays. Glitter is the microfacet lobe's job.
//
// Every screen derivative this needs is taken in PsMain under uniform control flow and handed
// in: the footprint FRAME {fpxW, fpzW}, whose per-axis Gaussian at a band's wavenumber is that
// band's exact expected attenuation (ALGEBRA "ripple"; an isotropic scalar erred x3000 at
// grazing). Under-recovered energy sheds into sigma^2 -- never aliased, never deleted. The
// point the water is read at, wxz, is handed in too: PsMain forms it once from VsOut.geo, and
// the frame is its screen derivative.
// pA is the pixel's address point (PsMain's), `own` whether it is geo (the camera's own level):
// the bed's point is then the ray's own, pA lifted to the surface the ray leaves plus s along it;
// another level's bed has only its direction.
float3 WaterPixelColor(float3 up, float3 upT, float3 east, float3 north, float3 rel, float2 wxz,
                       float hp, float latDeg, float lonDeg, float lod, float day, float footPx,
                       float2 fpxW, float2 fpzW, float3 pA, bool own) {
    const float3 v = normalize(-rel);

    // ---- THE VOLUME'S OPTICS. K_d and the scattering endpoint are MADE by chlorophyll,
    // sediment and CDOM, so both are measurements (ALGEBRA "optics"). Where the retrieval is
    // absent, the M7c constants and the shelf tint stand in, byte for byte.
    const WaterOptics wq = SampleWaterOptics(latDeg, lonDeg);
    const float shelf = saturate(1.0f + hp / 45.0f);        // 1 at the beach, 0 by -45 m
    const float3 cScatter =
        (wq.deep.x >= 0.0f) ? wq.deep
                            : lerp(float3(0.008f, 0.030f, 0.080f),
                                   float3(0.055f, 0.28f, 0.31f), shelf * shelf);

    // ---- THE SURFACE. Far afield the slope variance is Cox-Munk on the live wind; inside the
    // bank's rings the fold's own shed sigma^2 replaces it, floored at the PIXEL's resolving
    // limit (0.0015 -- not the vertex path's 0.0260, which exists only because a triangle
    // cannot carry a lobe that sharp; a pixel can, and that sharpness IS the glassiness).
    float s2 = 0.0300f;
    if (gTexIdx.z != 0xFFFFFFFFu) {
        const float2 wuv = float2(
            frac((lonDeg - gWavesA.y) * gWavesA.w / gWavesB.x),
            saturate(((gWavesA.x - latDeg) * gWavesA.z + 0.5f) / gWavesB.y));
        const float w10 = gTex[gTexIdx.z].SampleLevel(sLinearClamp, wuv, 0).x;
        if (w10 >= 0.0f) s2 = 0.003f + 0.00512f * w10;
    }
    float3 nSmooth = upT;    // ring-texel band limit -> the body's diffuse
    float3 nPix = upT;       // + the bands this pixel resolves -> glint, Fresnel, both rays
    float lvlW = 0.0f;       // live water level here (tide + solver); 0 = geoid far afield
    float dispW = 0.0f;      // the wave's own vertical displacement here; 0 where no ring reads
    {
        float4 bD, bP, bDet;
        float bT;
        float3 bDdx, bDdz;
        if (BankSampleT(wxz, bD, bP, bDet, bT, bDdx, bDdz)) {
            s2 = max(bP.y, 0.0015f);
            lvlW = bP.x;
            dispW = bD.y;
            // The wave normal: the tangent bivector's dual, analytic from the reconstruction
            // kernel (priors 21 -- the derivative is still taken at the DATA's grain, because
            // the kernel's 4x4 support IS that grain; what changed is that it is now
            // continuous across a texel border instead of stepping at it). The two extra ring
            // probes are gone, and the 1.1 cap rides inside BankSlope.
            const float2 sRing = BankSlope(bDdx, bDdz);
            float sx = sRing.x;
            float sz = sRing.y;
            nSmooth = normalize(upT - east * sx - north * sz);

            // The bands the PIXEL resolves but the ring texel does not, read from the cascade
            // DERIVATIVE textures at full FFT resolution and weighted by the fold difference
            // (wPix - wRing). At altitude wPix <= wRing and every term below vanishes: the
            // far field is untouched, and the tiers stay telescoped because sigma^2 takes
            // back exactly what the slope does not.
            //
            // Where the chart reads this point: the geoid point less the tangent point, in
            // tangent axes (ChartUOf). Its drop below the plane is R (upT.y - 1), written as
            // -(x^2 + z^2) / (R (1 + upT.y)) so that nothing near R cancels.
            const float3 qA = float3(wxz.x, -dot(wxz, wxz) / (gGlo.x * (1.0f + upT.y)), wxz.y);
            [unroll] for (uint c = 0; c < 3; ++c) {
                const float lam = 6.2831853f / gBankFold[c];    // M9c: the band's ENERGY, not
                const float wRing =                             // its geometric midpoint
                    1.0f - smoothstep(lam * 0.12f, lam * 0.5f, bT);
                const float wPix = 1.0f - smoothstep(lam * 0.12f, lam * 0.5f, footPx);
                const float wDet = saturate(wPix - wRing) * bDet.y;
                if (wDet <= 0.002f) continue;
                const float kC = gBankC[c];
                const float gAx =
                    exp(-0.125f * kC * kC * (fpxW.x * fpxW.x + fpzW.x * fpzW.x));
                const float gAy =
                    exp(-0.125f * kC * kC * (fpxW.y * fpxW.y + fpzW.y * fpzW.y));
                // M13 step 2: read in the SAME PLANE the bank filled its texels from (the
                // lattice's chart at the eye), not on the root's tangent plane -- otherwise
                // these sub-ring bands are a second realization laid over the first, and at a
                // carried place they are a different sea entirely. The slopes come back in the
                // chart's axes and are turned into this pixel's east/north below. The chart is
                // the CAMERA's own cell for every level: a world seen through a gate still reads
                // it, which is the wrong cell there -- known, and not mended here.
                const float2 cuvP = (gChartOrg.w != 0.0f)
                                        ? ChartUOf(qA, c) / gBankB[c]
                                        : wxz / gBankB[c];
                const float4 dv = gTex[gBankU2[c + 1u]].SampleLevel(sLinearWrap, cuvP, 0);
                float2 slope = float2(dv.x, dv.y);
                if (gChartOrg.w != 0.0f) {
                    // The chart's axes said in this pixel's own east/north (a rotation under two
                    // degrees: the two frames are a cell apart at most). Both are tangent-frame
                    // vectors, the rows since the CPU turned them.
                    const float2 R0 = float2(dot(gChartE.xyz, east), dot(gChartN.xyz, east));
                    const float2 R1 = float2(dot(gChartE.xyz, north), dot(gChartN.xyz, north));
                    slope = float2(dot(R0, float2(dv.x, dv.y)), dot(R1, float2(dv.x, dv.y)));
                }
                sx += slope.x * wDet * bDet.x * gBankB.w * gAx;
                sz += slope.y * wDet * bDet.x * gBankB.w * gAy;
                s2 = max(s2 - wDet * bDet.x * bDet.x *
                                  (0.5f * (gAx * gAx + gAy * gAy)) *
                                  (c == 0 ? 0.0004f : (c == 1 ? 0.0018f : 0.0060f)),
                         0.0015f);
            }
            const float slW = length(float2(sx, sz));
            if (slW > 1.1f) { sx *= 1.1f / slW; sz *= 1.1f / slW; }
            nPix = normalize(upT - east * sx - north * sz);
        }
    }

    // ---- THE SPLIT. Schlick on the TRUE per-pixel normal: from straight above F ~ 0.02 and
    // the space view is the bed's; toward the horizon the sea becomes a mirror.
    const float3 dIn = -v;                                   // camera -> surface
    const float cosV = saturate(dot(v, nPix));
    const float fres = 0.02f + 0.98f * pow(1.0f - cosV, 5.0f);

    // ---- RAY 1, REFLECTED: the Cl(3) sandwich. What it HITS is the question the old horizon
    // clamp dodged.
    float3 rDir = normalize(dIn - 2.0f * dot(dIn, nPix) * nPix);
    const float rUp = dot(rDir, upT);

    // ---- M9bj: THE MIRROR SEES SEA, NOT ONLY SKY. Clamping every below-horizon ray back up to
    // the horizon laundered it into the BRIGHTEST band of the sky, and at the helm that turned
    // a storm into a white sheet: the mid-field normal is band-limited to the bank's ring texel
    // (5-15 m), so it is nearly flat, its mirror ray leaves at +2 deg, and every pixel returned
    // the same pale horizon.
    //
    // But the pixel is not one facet. sigma^2 is the slope variance the FOLD sheds the moment a
    // band stops being resolved (ALGEBRA "fold"), and a slope spread of sigma spreads the
    // REFLECTED direction by 2 sigma. The fraction of that distribution whose ray leaves below
    // the horizon does not see sky at all -- it sees more sea, whose radiance this function has
    // already computed. So the mirror is a blend, in units of the spread itself:
    //
    //     seaward = saturate(1/2 - rUp / (2 * 2 sigma))
    //
    // ONE continuous expression, no branch, no threshold, and its limits are the physics: at
    // rUp = 0 exactly half the facets point down, so seaward = 1/2; from space rUp ~ 1 and it
    // saturates to 0, so the orbital view is untouched to the last bit; at the helm sigma ~ 0.17
    // gives a 20 deg spread and roughly half the sheet turns back into water you can see into.
    // The fold's shed grades used to reach the GLINT and not the MIRROR -- that asymmetry was
    // the artefact, and this is the same sigma^2 arriving where it was always due.
    const float spread = 2.0f * sqrt(max(s2, 1e-6f));
    const float seaward = saturate(0.5f - 0.5f * rUp / spread);
    // The sky is still sampled on a horizon-clamped ray -- below it the model has no sky to
    // give -- but it now only carries the (1 - seaward) share.
    const float3 rSky = normalize(rDir + max(0.02f - rUp, 0.0f) * upT);

    // ---- RAY 2, REFRACTED: the rotor's closed form, then the cast. depth = level - bed at
    // THIS pixel, so the translucency follows the LIVE tide.
    const float depthW = max(lvlW - hp, 0.0f);
    const float ci = saturate(-dot(dIn, nPix));
    const float etaR = 1.0f / 1.34f;
    const float st2 = etaR * etaR * max(1.0f - ci * ci, 0.0f);
    const float3 tDir =
        normalize(etaR * dIn + (etaR * ci - sqrt(max(1.0f - st2, 0.0f))) * nPix);
    float sDown = depthW;      // the vertical closed form: exact where parallax is subpixel
    float3 bedDir = up;
    float3 bedP = pA;
    if (footPx < 30.0f && depthW > 0.01f && depthW < 90.0f) {
        // Under 30 m footprints the march MATTERS -- looking through a wave face shifts the
        // bar, and that shift is the whole reason this path is per pixel.
        //
        // THE CAST IS MADE IN SMALL NUMBERS (REVIEW finding 6). It used to march the drawn
        // point sphere-centred -- the level's eye plus rel, 6.4e6 m in float, whose ulp is half a
        // metre -- and take the radius off its length, which put the ray's length out by up to
        // 1.0 m and its landing by 0.76 m (tools/hierarchy/refraction_cast.py). Split along this
        // pixel's radial and across it, the ray's point is an altitude and a direction, and both
        // are small numbers:
        //     alt(s) = a0 + s mu + s^2 |tPerp|^2 / (2 (R + a0))
        //     dir(s) = normalize(upT + s / (R + alt(s)) tPerp)
        // -- the sphere to second order in s / R, which inside the clamp's 140 m leaves out less
        // than a micrometre. a0 is the altitude of the surface the ray leaves: the live level plus
        // the wave's own vertical displacement here, the two things the drawn point carried. The
        // same start, the same two secant steps and the same clamp; no eye enters, and the gauge
        // is kept because upT and a0 are the level's own. The harness puts the ray's length within
        // 5 micrometres of doubles and its landing within 22.
        const float mu = dot(tDir, upT);            // along this pixel's radial; negative is down
        const float3 tPerp = tDir - mu * upT;       // and across it
        const float tPerp2 = dot(tPerp, tPerp);
        const float a0 = lvlW + dispW;
        const float twoR = 2.0f * (gGlo.x + a0);    // 2 (R + a0): the sphere's fall under the ray
        const float muD = max(-mu, 0.10f);
        float sP = depthW / muD;
        [unroll] for (int itr = 0; itr < 2; ++itr) {
            const float altP = (a0 + sP * mu) + sP * sP * tPerp2 / twoR;
            const float3 dirP = normalize(upT + (sP / (gGlo.x + altP)) * tPerp);
            const float3 pP = own ? pA + upT * a0 + sP * tDir : CsPointOfDir(CsToPlanet(dirP));
            const float gap = altP - ComposedHeight(CsToPlanet(dirP), pP, lod);
            sP = clamp(sP + gap / muD, 0.3f, 140.0f);
        }
        const float altB = (a0 + sP * mu) + sP * sP * tPerp2 / twoR;   // at the landing
        bedDir = CsToPlanet(normalize(upT + (sP / (gGlo.x + altB)) * tPerp));
        bedP = own ? pA + upT * a0 + sP * tDir : CsPointOfDir(bedDir);
        sDown = sP;
    }
    const float3 Tw = exp(-wq.kd * (sDown + depthW));
    const float3 bedAlb = (ComposedColorOn() && gStreamF.z < 0.5f)
                              ? ComposedColor(bedDir, bedP CS_WALK_AT(bedDir))
                              : float3(0.44f, 0.40f, 0.31f);
    // THE TRANSLUCENCY. Albedos mix and the surface lights ONCE -- the engine's radiometry
    // everywhere else -- so this path cannot disagree with the vertex path about EXPOSURE,
    // only about what is under the water.
    float3 albW = lerp(cScatter, bedAlb, Tw);

    // ---- THE GLINT: the Cox-Munk lobe on the pixel normal, sigma^2 as folded above.
    // M10: the level's own sun, eclipsed by the inner globe (PortalShadow) and by the planet
    // under this pixel (PlanetShadow) -- asked of upT, never of nPix.
    const float sunVis = PortalShadow(sLvlCamAbs + rel - float3(0.0f, gGlo.x, 0.0f)) *
                         PlanetShadow(upT, GA_SUN_DIR, hp, gGlo.x);
    const float3 hv = normalize(v + GA_SUN_DIR);
    const float ch = saturate(dot(hv, nPix));
    const float tt = max(1.0f - ch * ch, 0.0f) / max(ch * ch, 1e-4f);
    float spec = exp(-tt / s2) / (4.0f * kPi * s2 * max(ch * ch * ch * ch, 1e-4f));
    spec *= (0.02f + 0.98f * pow(1.0f - saturate(dot(v, hv)), 5.0f)) *
            saturate(dot(GA_SUN_DIR, nPix)) * sunVis;

    // ---- THE COMBINE. Energy SPLITS: the body dims by exactly the Fresnel the mirror takes,
    // and the mirror itself is part sky, part the very water it stands on.
    const float ndl = saturate(dot(nSmooth, GA_SUN_DIR)) * sunVis;   // the body, smooth normal
    const float3 bodyLit = albW * (0.030f + ndl * SUN_IRR_C * 1.15f);
    // What the seaward share HITS is another wave, and that wave is water too: at its own
    // grazing angle it is mostly a mirror, and only steeply-down rays see into it. So the
    // seaward endpoint is one more bounce of the SAME Schlick, on the flat sea's normal --
    // |rUp| is that hit's cosine. At the horizon it returns 1 and the endpoint is sky again
    // (which is why the horizon band stays pale silver instead of turning green); pointing
    // straight down it returns 0.02 and the endpoint is the water body. No new constant, no
    // branch -- the law already in this function, applied once more.
    // M10: the sky it sees -- the hour rides the gradient only; the march already has it.
    const float3 skyLit = SkyRadianceDirDiscless(rSky, SkyDay(day));
    const float fresHit = 0.02f + 0.98f * pow(1.0f - saturate(-rUp), 5.0f);
    const float3 mirror = lerp(skyLit, lerp(bodyLit, skyLit, fresHit), seaward);
    float3 col = bodyLit * (1.0f - fres);
    col += spec * SUN_IRR_C * 0.85f;
    col += mirror * (fres * 0.9f);
    col += albW * (1.0f - fres) * float3(0.010f, 0.014f, 0.028f) * (1.0f - day);   // night side
    return col;
}

// M9b: THE AMPLIFICATION UNIT, made visible. Wireframe answers "is the geometry moving";
// this answers "what drew it" -- one flat colour per meshlet record (per CDLOD node
// instance on the VS fallback), so the 8x8-cell blocks and the CDLOD ring handovers read
// at altitudes where every-triangle wireframe collapses into moire. A hash, not a ramp:
// neighbours must not share a colour. Selected by its own PSO, so PsMain is untouched.
float4 PsMeshlet(VsOut i) : SV_Target {
    const uint h = (i.mid * 2654435761u) ^ ((i.mid * 40503u) << 13);
    const float3 c = float3(float((h >> 16) & 255u), float((h >> 8) & 255u),
                            float(h & 255u)) / 255.0f;
    // Keep the terminator readable so the planet still looks like a planet under the tint.
    // gSunDir lives in the TANGENT frame (PsMain dots it with upT), so the surface normal
    // must cross frames too -- CsToTangent, exactly as PsMain does it. A planet-frame dir
    // here would light the wrong hemisphere.
    const float lam = saturate(dot(CsToTangent(normalize(i.dir)), gSunDir.xyz) * 0.5f + 0.5f);
    return float4(c * (0.25f + 0.75f * lam), 1.0f);
}

// M9bk: THE FLAT WIRE (--wireflat). The --wireframe pass draws its lines with PsMain, so
// every edge is painted with finished water -- specular, foam, sky mirror -- and the mesh you
// are trying to READ is buried in the shading of the thing it describes. That cost this
// session two wrong diagnoses off wireframe crops. This is the same geometry with NO shading:
// one grey, depth-cued only so near and far stay separable. What you see is where the vertices
// are, and nothing else.
float4 PsWireFlat(VsOut i) : SV_Target {
    const float d = length(i.rel);
    const float f = saturate(260.0f / (260.0f + d));
    return float4(f * 2.2f, f * 2.2f, f * 2.3f, 1.0f);
}

float4 PsMain(VsOut i) : SV_Target {
    // M10: THE GAUGE FIRST. i.rel, the eye, the sun and the bank set are all the drawing
    // level's own; everything below is the root's shading, unchanged, run in that frame.
    LoadLevel(i.lvl);
    // THE VIEW'S WINDOWS: seen through k boxes is the level of depth k, and only that; every
    // other level is everything else. The same test from every eye, so every view agrees.
    if (gGateA.x >= 0.0f) {
        if (GateDepth(TrueRel(i.rel)) != LevelGateDepth(i.lvl)) discard;
    }
    const float3 up = normalize(i.dir);   // PLANET frame: lat/lon + every texture fetch
    // THE ADDRESS (plan_address.md): every Mercator read below is addressed by this pixel's
    // undisplaced ground point, eye-relative in the tangent axes -- the mesh stage's geo in the
    // camera's own level (slot 0, whose eye the surface rows are taken about); a fragment of
    // another level, a Droste globe or a window's world, has only its direction.
    const bool ownLvl = (i.lvl == 0u);
    const float3 pA = ownLvl ? i.geo : CsPointOfDir(up);
#if GA_BLOCK_RANKS
    // HIERARCHY 4.17: THE CHAIN, found once for the pixel and handed to every read below (CS_WC):
    // the blocks under its undisplaced ground point, which the mesh stage carried eye-relative in
    // the camera's own level (slot 0, whose frame the rows are about). A fragment of another level,
    // a Droste globe or a window's world, has only its direction, taken into the root's frame at
    // the direction's grain.
    const WalkChain wc = CsWalk(up, pA);
#endif
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
    const float day = saturate(dot(GA_SUN_DIR, upT) * 3.0f + 0.12f);

    // ---- M9bh: THE PIXEL'S FOOTPRINT, taken HERE and nowhere else. Screen derivatives are
    // defined only under UNIFORM control flow, so the water's prefilter frame is computed at
    // the top of the shader and handed to WaterPixelColor -- never inside its branches. The
    // footprint is a FRAME {fpxW, fpzW}, not a scalar: a grazing sliver resolves across-view
    // ripples while along-view ones alias into crawling shimmer, and only the frame's per-axis
    // Gaussian can say so (ALGEBRA "ripple"; an isotropic max erred x3000 at grazing).
    // THE WATER'S SAMPLE POINT, formed once (REVIEW finding 42): the level's eye plus the
    // undisplaced geoid point the mesh stage carried. It is the same point as the (upT * R).xz
    // it replaces -- for a point G of the geoid, (upT R).xz = G.xz = eye.xz + (G - eye).xz --
    // made from the record's doubles instead of from dir's third of a metre of grain, so the
    // footprint frame, its screen derivative, is the pixel's own ground and not that staircase.
    const float2 wxzW = sLvlCamAbs.xz + i.geo.xz;
    const float2 fpxW = ddx(wxzW);
    const float2 fpzW = ddy(wxzW);
    const float footPxW = length(i.rel) * gWavesB.z;

    const float lod = ComposedHeightLod(length(i.rel), gWavesB.z);
    // M6i: land/sea CLASSIFICATION resamples the height channel PER PIXEL at screen lod. The
    // vertex height (i.h) is footprint-floored for displacement (~5 km) and foundation-sunk
    // in the estuary window -- gating on it smeared whole towns below sea level. hp is what
    // the data says HERE, at this pixel's own resolution.
    const float hp = ComposedHeightOn() ? ComposedHeight(up, pA, lod) : i.h;
    // The land's sun: the inner globe's eclipse and the planet's own shadow at this pixel's
    // ground -- the two factors the water takes, so a shoreline cannot disagree about whether
    // the sun is up. A hillside leans toward a set sun exactly as a wave face does.
    const float sunVis = PortalShadow(sLvlCamAbs + i.rel - float3(0.0f, gGlo.x, 0.0f)) *
                         PlanetShadow(upT, GA_SUN_DIR, hp, gGlo.x);
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
    const float hpC = ComposedHeightOn() ? ComposedHeight(up, pA, max(lod, -5.0f)) : i.h;
    const float landness =
        (gStreamF.z > 0.5f) ? ((hp > 0.0f) ? 1.0f : 0.0f)
                            : ComposedLandness(up, pA CS_WC, hpC, gWavesB.w);
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
        const float2 gr = ComposedHeightGrad(up, pA, lod);
        const float kSlopeGain = 4.0f;
        const float3 nLand =
            normalize(upT - east * (gr.x * kSlopeGain) - north * (gr.y * kSlopeGain));
        const float3 albLand = Hypsometric(max(hp, 0.0f), lat);
        // ---- M9bg: THE SEA SIDE IS GONE FROM THIS SHADER. Everything that used to paint
        // water here -- the ocean-colour retrieval and its K_d transfer, the wave-grid Hs and
        // wind fetch, the bank's per-pixel normals and the cascade sparkle, the two rays with
        // their secant cast onto the imagery bed, Beer-Lambert, the caustic Jacobian, the
        // seafloor relief, the ice lerp, the foam-breakup octaves and the peak shaping --
        // retired to WaterVertexColor, which runs once per VERTEX. What arrives here is
        // i.wcol, already lit, and it is mixed in AFTER the lighting combine below. A water
        // pixel now costs this shader nothing at all: no sample, no march, no branch.
        //
        // The geometry that carried all of it is untouched: the bank still displaces every
        // vertex and the meshlets are as dense as they ever were.
        n = nLand;
        alb = albLand;
    }

    // ---- M6i/M7c: the composed color channel. Land takes the imagery outright.
    // M9bg: gated on landness as well -- the imagery is now a LAND albedo only (the water
    // carries no texture at all), so an open-water pixel must not pay for the fetch.
    if (ComposedColorOn() && gStreamF.z < 0.5f && landness > 0.0f) {
        const float3 img = ComposedColor(up, pA CS_WC);
        alb = lerp(alb, img, landness);
        // The land is the imagery (the owner's law): the surveyed structures' rock -- a hash per
        // 0.7 m cell addressed through the float32 direction, painted over the photo with facet
        // normals -- is gone; the edit mask still decides landness and the structures' geometry.
    }

    // ---- M6j --albedo: the TEXTURE-WORK lens. Raw composed color (or Mars's raw pyramid) --
    // no lighting, no atmosphere, no materials, no clouds, and NOT the shallow-water mix
    // either (classification is presentation too; the lens must show what the TEXTURES say).
    // The stencil still draws. Everything below this line is presentation.
    if (gTexIdx2.w != 0u) {
        const float3 lens =
            (ComposedColorOn() && gStreamF.z < 0.5f) ? ComposedColor(up, pA CS_WC) : alb;
        return float4(ApplyComposedStencil(lens, up, pA), 1.0f);
    }

    // ---- M7m: THE SANITY LENSES -- values as color, so a domain error is a broken
    // pattern instead of an argument. worldxz: a 100 m world checker (any frame/scale
    // error shows as a seam across LOD or ring boundaries). winuv: the window uv gradient
    // (its v is SOUTH -- the gradient direction proves it). mip: the height window's
    // residency heat. ring: the bank's rings with a 4-texel checker (bank addressing on
    // screen). These test the GA the cheap way: patterns survive correct products.
#if GA_BLOCK_RANKS
    if (gBankA.z > 0.5f && gBankA.z < 11.5f) {   // 12 and 13 are the mix lenses, at the end
#else
    if (gBankA.z > 0.5f) {
#endif
        const float2 wxzL = (CsToTangent(up) * gGlo.x).xz;
        const int lensId = (int)(gBankA.z + 0.5f);
        float3 lc = float3(0.05f, 0.05f, 0.08f);
        if (lensId == 1) {
            const float chk = fmod(floor(wxzL.x / 100.0f) + floor(wxzL.y / 100.0f) +
                                       200000.0f,
                                   2.0f);
            lc = float3(frac(wxzL / 100.0f) * 0.75f + 0.1f, chk * 0.7f);
        } else if (lensId == 7) {
            // GRADE PICKS THE VISUALIZATION (docs/SPARSE_GA.md 9). One bank, two grades, and
            // they want different ramps: the divergence is a signed SCALAR, the vorticity is a
            // signed BIVECTOR whose sign is a handedness. So curl drives a diverging red/blue
            // -- the eddy's sense of rotation reads directly -- and |div| rides the green,
            // where convergence and divergence both brighten because the magnitude is what
            // the confluence looks like. Black is not "no data": a NULL tile reads zero, and
            // zero here means the flow is irrotational and divergence-free, which is the
            // honest answer for open water.
            // TWO LODs, ONE LENS. The low-LOD global default is the wind Mv2 bank -- global,
            // already sparse, resident only where storms live; the high-LOD inset is the SWE
            // grad(flow) over the inlet. Whichever covers this pixel wins, finest first, which
            // is the whole heterogeneous story in four lines of shader.
            //
            // HONEST CAVEAT, and it matters for every lens that spans LODs: the two sources
            // are NOT on one absolute scale. Synoptic wind curl lives at ~1e-5 /s over 25 km
            // cells; a tidal jet's curl is ~1e-2 /s over 10 m cells -- three orders apart.
            // Each is normalized against its OWN dynamic range, so colour compares structure
            // within a source and NOT magnitude across them. Putting them on one absolute
            // ramp would make the global field look dead, which would be a lie told by a
            // colour map rather than by the data.
            // TWO PAGES OF ONE BANK. Both are slices of the same reserved array, read
            // through one Texture2DArray view, and composited on coverage exactly as before.
            // What changed is what the coarse operand IS: it was the global WIND -- a
            // different field in different units, three orders away, only made to look
            // comparable by normalizing each against its own range. Now it is the same
            // quantity (grad of a current, 1/s) over wider ground, so the composite is a
            // statement about one field at two resolutions instead of a colour map pretending
            // two things are alike.
            //
            // No branch decides which source owns a pixel. Slice 1 answers everywhere it has
            // coverage, slice 0 answers better where the solve reaches, and the weight sorts
            // them out per pixel -- the compositor's rule, on the GPU.
            float2 fine = float2(0, 0), coarse = float2(0, 0);
            float wFine = 0.0f, wCoarse = 0.0f;
            if (gLensU.x != 0xFFFFFFFFu) {
                const float footL = length(i.rel) * gWavesB.z;

                // --- slice 0: this window's page, at the solve's resolution
                const float2 uvL = (wxzL - gLensA.xy) * gLensA.zw;
                if (all(uvL > 0.0f) && all(uvL < 1.0f)) {
                    float lodL = log2(max(footL / max(gLensB.x, 1e-3f), 1.0f));
                    if (gLensU.y != 0xFFFFFFFFu) {
                        const uint2 rt = uint2(uvL.x * gLensB.y, (1.0f - uvL.y) * gLensB.z);
                        const uint have = gTexU[gLensU.y].Load(uint3(rt, 0)).x;
                        lodL = max(lodL, (have == 0xFFu) ? 0.0f : float(have));
                    }
                    lodL = clamp(lodL, 0.0f, max(gLensB.w - 1.0f, 0.0f));
                    const float4 hq = gTexArr[gLensU.x].SampleLevel(
                        sLinearClamp, float3(uvL.x, 1.0f - uvL.y, 0.0f), lodL);
                    fine = float2(abs(hq.x) * 60.0f, hq.y * 60.0f);
                    const float2 fe = smoothstep(0.0f, 0.06f, uvL) *
                                      smoothstep(1.0f, 0.94f, uvL);
                    wFine = saturate(hq.z) * fe.x * fe.y;
                }

                // --- slice 1: the region's page, on its OWN geography
                if (gLensR.z != 0.0f) {
                    const float2 uvR = float2((lonDeg - gLensR.x) * gLensR.z,
                                              (degrees(lat) - gLensR.y) * gLensR.w);
                    if (all(uvR > 0.0f) && all(uvR < 1.0f)) {
                        // Level 3 is the finest the region page OWNS -- below that it was
                        // never written, and sampling there would read a level that is NULL.
                        const float4 rq = gTexArr[gLensU.x].SampleLevel(
                            sLinearClamp, float3(uvR, 1.0f), 3.0f);
                        // The region is a coarser description of the SAME quantity, so it
                        // shares the fine gain -- no second normalization, which is the whole
                        // reason this can be one composite.
                        coarse = float2(abs(rq.x) * 60.0f, rq.y * 60.0f);
                        const float2 fr = smoothstep(0.0f, 0.04f, uvR) *
                                          smoothstep(1.0f, 0.96f, uvR);
                        wCoarse = saturate(rq.z) * fr.x * fr.y;
                    }
                }
            }
            // Weighted, not switched: where the solve has coverage it wins by weight;
            // where it does not, the region carries the pixel; where neither does, the texel
            // is absent and stays black -- which is now honest, because "no data" is a real
            // answer rather than a missing source.
            const float wSum = wFine + wCoarse * (1.0f - wFine);
            const float2 mvL = (wSum > 0.0f)
                                   ? (fine * wFine + coarse * wCoarse * (1.0f - wFine)) / wSum
                                   : float2(0, 0);
            {

                const float divN = saturate(mvL.x);
                const float curlN = clamp(mvL.y, -1.0f, 1.0f);
                lc = float3(saturate(curlN), divN * 0.9f, saturate(-curlN));
            }
        } else if (lensId == 8) {
            // Step 23 -- THE SHELL LENS: R = the fragment's central angle from the eye's
            // sub-point (radians), G/B = the meshlet record (hi/lo byte), A = 7 marks a
            // surface fragment against the sky pass and 9 a SEAM BAND fragment
            // (GlobeMesh.hlsl BandDepth). A closed shell never shows a fragment beyond the
            // horizon angle acos(R/r) plus the relief's reach -- a pixel that does is a ray
            // that left the shell through a crack and landed on the far side -- and a band
            // is visible only where the shell had no coverage of its own: the hole map.
            const float caL = acos(clamp(dot(upT, normalize(sLvlCamAbs)), -1.0f, 1.0f));
            const uint midL = i.mid & 0x1FFFFu;   // M10: 17-bit records
            return float4(caL, float(midL >> 8u), float(midL & 255u),
                          (i.mid & 0x80000000u) ? 9.0f : 7.0f);
        } else if (lensId == 2) {
            const float2 duvL = CsWindowUvAt(pA);
            if (all(duvL > 0.0f) && all(duvL < 1.0f)) lc = float3(duvL, 0.0f);
        } else if (lensId == 3) {
            const float2 duvL = CsWindowUvAt(pA);
            if (CsHeightWindowOn() && all(duvL > 0.0f) && all(duvL < 1.0f)) {
                const float mL = CsHaveHeightWin(duvL, pA);
                lc = lerp(float3(0.1f, 0.85f, 0.25f), float3(0.9f, 0.12f, 0.1f),
                          saturate(mL / 7.0f));
                lc.b = max(lc.b, saturate(-mL / 3.0f));   // the z17 page: blue 1/3 a mip finer
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

    const float ndl = saturate(dot(n, GA_SUN_DIR)) * (1.0f - 0.75f * overhead) * sunVis;
    float3 col = alb * (0.030f + ndl * SUN_IRR_C * 1.15f);
    col += spec * SUN_IRR_C * 0.85f * (1.0f - overhead);
    col += skyReflAdd * day * (1.0f - 0.6f * overhead);   // M7c: the reflected ray, skyward
    col += alb * float3(0.010f, 0.014f, 0.028f) * (1.0f - day);   // moonlit-blue night side

    // ---- M9bg: THE MIX. Land was lit above, per pixel; the water was lit at its VERTICES and
    // interpolated here. A shoreline pixel is landness of the one and (1 - landness) of the
    // other -- the same analog band that mixes the GEOMETRY in SurfaceVertex, so colour and
    // mesh cannot disagree about where the shore is. Over open water landness is 0, the land
    // half above contributed nothing, and the sea's pixel is a pure interpolation.
    // Everything below is ATMOSPHERE (cloud shadow, the volume march, the space rim) -- that
    // is the air between the eye and the surface, not paint on the water, so it still applies.
    // M9bh: --pixel-water (gOptU.w) replaces the interpolated colour with WaterPixelColor,
    // evaluated HERE, per pixel -- the two rays cannot ride an interpolator. Both paths return
    // a FINISHED water colour, so the mix below is the same line either way, and the land half
    // of a shoreline pixel is untouched by the choice.
    float3 wcol = i.wcol;
    if (gOptU.w != 0u && gStreamF.z < 0.5f && landness < 0.999f) {
        wcol = WaterPixelColor(up, upT, east, north, i.rel, wxzW, hp, degrees(lat), lonDeg, lod,
                               day, footPxW, fpxW, fpzW, pA, ownLvl);
    }
    if (gStreamF.z < 0.5f) col = lerp(wcol, col, landness);

    // ---- M6j: the CLOSE-UP material model, now in the planet shader -- the land's albedo is
    // the imagery's at whatever grain it has, and over it what no photo can know: wet sand at
    // the LIVE waterline and the surveyed rock of the edit polygons, lit by the relief's own
    // normal the way the terrain layer lit them (sun + sky ambient + near haze). The generic
    // beach, dune-grass and riprap constants are gone: they were guesses by height and slope
    // that painted every land within 2.7 km of the eye, a farm upriver as a dune.
    const float distC = length(i.rel);
#if GA_BLOCK_RANKS
    float lensNearW = 0.0f;   // the mix lens's record of this block's mix
#endif
    if (gStreamF.z < 0.5f && landness > 0.0f && distC < 2700.0f && CsHeightWindowOn()) {
        const float2 wuv = CsWindowUvAt(pA);
        if (all(wuv > 0.0f) && all(wuv < 1.0f)) {
            const float2 grF = ComposedHeightGrad(up, pA, kCsHeightLodFloor);   // true slope, finest resident
            const float3 nM = normalize(upT - east * grF.x - north * grF.y);
            const float water = gWavesB.w;
            float3 matAlb = alb;
            if (hp - water < 0.35f) matAlb = float3(0.38f, 0.34f, 0.27f);   // wet sand band
            const float ndlM = saturate(dot(nM, GA_SUN_DIR)) * sunVis;
            // M10 (a pre-existing bug the Droste night found): the sky's ambient here ignored
            // the hour. Every other term in this shader dims its skylight by `day`; this one did
            // not, so a night-side beach glowed daylight-grey within 2.7 km of the eye -- never
            // seen, because the root helm is always in daylight. Now it takes the same `day` and
            // the same moonlit floor the far-field mix below uses: one law, day or night.
            const float skyD = SkyDay(day);
            float3 colNear = matAlb * (SUN_IRR_C * ndlM + SkyRadiance(dot(nM, GA_SKY_UP)) * (0.55f * skyD) +
                                       float3(0.010f, 0.014f, 0.028f) * (1.0f - skyD));
            colNear = AerialPerspectiveDay(colNear, normalize(i.rel), distC, skyD);
            // M6n: the material weight rides landness too -- a half-emerged flat takes half
            // the wet-sand treatment, and the shore band grades instead of popping.
            col = lerp(col, colNear,
                       (1.0f - saturate((distC - 500.0f) / 2200.0f)) * 0.92f * landness);
#if GA_BLOCK_RANKS
            lensNearW = (1.0f - saturate((distC - 500.0f) / 2200.0f)) * 0.92f * landness;
#endif
        }
    }

    // ...and a short march through the volume bank renders the clouds themselves. NULL tiles
    // read zero: over clear air every sample is the hardware's answer, not a branch's.
    if (gTexIdx.w != 0xFFFFFFFFu) {
        const float3 ro = sLvlCamAbs;   // M10: the level's own eye marches its own sky
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
                const float mu = dot(rd, GA_SUN_DIR);
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
                    const float3 lpT = pdT * (pr + 900.0f) + GA_SUN_DIR * 900.0f;
                    const float3 luvw = float3(ReliefUv(CsToPlanet(normalize(lpT))),
                                               (length(lpT) - gGlo.x) / gCloudA.y);
                    const float lDens = gTex3D[gTexIdx.w].SampleLevel(sLinearClamp, luvw, 0).x;
                    const float sunT = exp(-lDens * gCloudA.x * 1500.0f) *
                                       saturate(dot(pdT, GA_SUN_DIR) * 3.0f + 0.1f);
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
                      smoothstep(60000.0f, 250000.0f, length(sLvlCamAbs) - gGlo.x);
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

    // THE GATE'S RIM, on the window's own pixels only (the destination never draws one).
    if (LevelGateDepth(i.lvl) > 0u) {
        col = lerp(col, float3(0.75f, 0.92f, 1.0f),
                   0.55f * GateRim(TrueRel(i.rel), LevelGateDepth(i.lvl) - 1u));
    }
    col = ApplyComposedStencil(col, up, pA);   // M6i: --stencil alignment overlay (off = no-op)
#if GA_BLOCK_RANKS
    // THE MIX LENSES, with the key, off unless asked for: what this stage mixes the land by, where it decides.
    // --lens mix (12): r = landness (the water's colour comes in by 1 - landness), g = the edit
    // mask's land, b = its edited weight. --lens mix.near (13): r = the close-up material's
    // weight, which is the wet band's alone now that the land's constants are out (the
    // imagery's share is whole, and no texel is judged): g and b are 0.
    if (gBankA.z > 11.5f) {
        const float2 meL = CsEditMask(up, pA CS_WC);
        col = (gBankA.z < 12.5f) ? float3(landness, meL.x, meL.y)
                                 : float3(lensNearW, 0.0f, 0.0f);
    }
#endif
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
    // through b3 (the globe's own root CBV; M12 step 4g moved it off b2, which is the surface's
    // now); only the NDC coordinate rides through here.
    const float2 xy = float2((vid == 1) ? 3.0f : -1.0f, (vid == 2) ? 3.0f : -1.0f);
    SkyVsOut o;
    // M6g: z = 0 (reversed-Z infinity) + a GREATER_EQUAL depth test in the PSO means this
    // backdrop touches ONLY pixels nothing has drawn -- it can no longer stomp the estuary
    // layers that render before the globe in the one-world scene.
    o.pos = float4(xy, 0.0f, 1.0f);
    o.dir = float3(xy, 1.0f);
    return o;
}

cbuffer GlobeSkyCb : register(b3) {
    float4 gSkyFwd;     // camera forward, w = tan(fovY/2)
    float4 gSkyRight;   // camera right,  w = aspect
    float4 gSkyUp;      // camera up
    // M10: x = the level slot PsLimb draws; y = the share of the space backdrop that is the
    // camera level's OWN air (1 = the old backdrop; 0 when the eye is outside that air, whose
    // limb is then PsLimb's); z = 1 under Droste: the sun shows wherever the planet does not;
    // w = one pixel's angle (rad), the footprint PsLimb filters over.
    float4 gSkyLvl;
    float4 gSkySpaceSun;   // M10: the space backdrop's sun (Droste only; camera frame)
};

// The shell's single scatter along [t0, t0 + span] of a ray from ro (sphere-centred, in the
// planet's own units) lit by `sun`: the light it adds, and in odView the optical depth it puts
// in front of whatever lies behind it. 6 steps, Chapman-lite sun transmittance.
float3 ShellScatter(float3 ro, float3 rd, float t0, float span, float3 sun, out float odView) {
    const uint kSteps = 6;
    const float dt = span / kSteps;
    float3 sum = 0.0f;
    odView = 0.0f;
    [unroll] for (uint s = 0; s < kSteps; ++s) {
        const float3 p = ro + rd * (t0 + (s + 0.5f) * dt);
        const float h = max(length(p) - gGlo.x, 0.0f);
        const float dens = exp(-h / kRayleighH);
        odView += dens * dt;
        // Sun transmittance out of the shell from p: one closed-form-ish estimate via the
        // grazing airmass (Chapman-lite).
        const float cosSun = dot(normalize(p), sun);
        const float am = dens * kRayleighH * 2.2f / max(cosSun + 0.18f, 0.02f);
        const float3 sunT = exp(-kBetaR * max(am, 0.0f));
        sum += dens * dt * sunT * exp(-kBetaR * odView);
    }
    const float mu = dot(rd, sun);
    const float phaseR = 0.0596831f * (1.0f + mu * mu);   // 3/(16 pi)
    float3 col = sum * kBetaR * phaseR * 22.0f * SUN_IRR_C;
    if (gStreamF.z > 0.5f) {   // Mars: 1% of Earth's air, dust-toned
        col = dot(col, float3(0.33f, 0.34f, 0.33f)) * float3(1.15f, 0.55f, 0.30f) * 0.30f;
    }
    return col;
}

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

    // The sun itself. Without Droste it shows only where the ray leaves the shell toward it (the
    // shipped backdrop); under Droste the backdrop is space, and space has the sun in it
    // wherever the planet is not -- a limb in front of it (PsLimb) dims it by its own air. Whose
    // sun: under appealing lighting every level has one, and space shows the one of the level
    // whose orbit called for it (gSkySpaceSun) -- not the camera's, which is the gauge.
    const float3 sunSky = (gSkyLvl.z > 0.5f) ? gSkySpaceSun.xyz : gSunDir.xyz;
    const float mu = dot(rd, sunSky);
    const float3 sunDisc = SUN_IRR_C * smoothstep(0.9998f, 0.99995f, mu) * 4.0f;
    const float3 bare = (gSkyLvl.z > 0.5f) ? sunDisc : float3(0.0f, 0.0f, 0.0f);

    // Atmosphere shell chord.
    const float top = gGlo.x + kAtmTop;
    const float cTop = dot(ro, ro) - top * top;
    const float disc = b * b - cTop;
    if (disc <= 0.0f) return float4(bare, 1);
    const float t0 = max(-b - sqrt(disc), 0.0f);
    const float t1 = -b + sqrt(disc);
    const float span = t1 - t0;
    if (span <= 0.0f) return float4(bare, 1);

    // M10: the camera level's own air, at its share of the space backdrop (gSkyLvl.y: exactly 1
    // without Droste).
    float odView;
    float3 col = ShellScatter(ro, rd, t0, span, gSunDir.xyz, odView) * gSkyLvl.y;

    // The sun itself, when the ray leaves the shell toward it.
    col += sunDisc;
    return float4(col, 1.0f);
}

// ------------------------------------------------------------------ M10: the limbs
//
// Every planet in the Droste tower whose air the eye is OUTSIDE of wears its limb: the same
// shell, scattered by the same function, in that level's own frame (its eye, its sun), drawn
// after the surface over whatever lies behind it. The backdrop above could not do this -- it
// touches only empty pixels, and an inner globe's limb lies over the outer world's sea. Blended
// as light added plus the light behind carried through (dual source: dst = scatter + dst * T),
// and depth-tested at the shell's own entry (SV_Depth), so a jetty in front hides it and the sea
// behind shows through it. Which levels: CPU side, one rule for every level including the
// camera's own -- the eye outside the shell -- so the re-root (a gauge change) moves no limb.
struct LimbOut {
    float4 scatter : SV_Target0;   // the in-scatter the chord adds
    float4 trans : SV_Target1;     // what survives of the scene behind it
    float depth : SV_Depth;        // the shell's entry: nearer surfaces hide the limb
};

LimbOut PsLimb(SkyVsOut i) {
    LoadLevel((uint)gSkyLvl.x);
    const float2 ndc = i.dir.xy;
    const float3 rdT = normalize(gSkyFwd.xyz + gSkyRight.xyz * (ndc.x * gSkyFwd.w * gSkyRight.w)
                                 + gSkyUp.xyz * (ndc.y * gSkyFwd.w));
    const float3 rd = mul(rdT, sLvlQ);   // Q^T: the same ray in the level's own frame
    const float3 ro = sLvlCamAbs;
    const float R = gGlo.x;
    const float top = R + kAtmTop;
    // The closest approach as a vector (ro - rd b), not b^2 - c: an inner globe's eye sits
    // ~1.7e7 own-metres out, where b^2 - c cancels away kilometres of the limb.
    const float b = dot(ro, rd);
    const float3 perp = ro - rd * b;
    const float rc = length(perp);                 // the ray's closest approach to the centre
    if (b >= 0.0f || rc >= top) discard;           // the shell is behind, or the ray misses it

    // THE PIXEL, NOT ITS CENTRE. A limb seen from afar is thinner than a pixel -- from the
    // jetties one pixel spans ~60 km of an inner planet's own air, the whole shell -- and a
    // centre sample of it is a bright dotted outline that crawls as the eye moves. The value
    // the pixel owes is the limb AVERAGED over its footprint, and the limb varies across a
    // pixel only with the tangent height h, so average 8 rays whose tangent heights span the
    // footprint (pixel angle x distance to the tangent point). Resolved up close, the 8 rays
    // coincide and this is the centre sample; far away it is the thin faint glow a real limb
    // is. Rays that strike the planet are the disc's (nothing added, nothing dimmed) -- which
    // also feathers the limb onto the disc's edge instead of clipping it there.
    const float foot = gSkyLvl.w * (-b);
    const float hc = rc - R;
    if (hc < -0.5f * foot) discard;                // wholly the disc: the surface's, rim and all
    const float3 nrm = perp / max(rc, 1e-3f);      // the direction h grows at the tangent
    const float3 tanPt = ro + rd * (-b);
    float3 scatter = 0.0f;
    float3 trans = 0.0f;
    const float t0c = -b - sqrt(max(top * top - rc * rc, 0.0f));
    const uint kSub = 8u;
    [unroll] for (uint s = 0; s < kSub; ++s) {
        const float h = hc + foot * ((float(s) + 0.5f) / float(kSub) - 0.5f);
        const float3 pSub = tanPt + nrm * (h - hc);     // the tangent point, moved to height h
        const float3 rdS = normalize(pSub - ro);
        const float bS = dot(ro, rdS);
        const float rS = length(ro - rdS * bS);
        if (rS < R) { trans += 1.0f; continue; }         // the disc's part of the pixel
        if (rS >= top) { trans += 1.0f; continue; }      // clear of the air
        const float hwS = sqrt(top * top - rS * rS);
        const float t0S = max(-bS - hwS, 0.0f);
        float od;
        scatter += ShellScatter(ro, rdS, t0S, (hwS - bS) - t0S, GA_SUN_DIR, od);
        trans += exp(-kBetaR * od);
    }
    LimbOut o;
    o.scatter = float4(scatter / float(kSub), 0.0f);
    o.trans = float4(trans / float(kSub), 1.0f);
    // Depth: the shell's entry along the pixel's own ray (a jetty in front hides the limb).
    const float4 clip = mul(float4(rdT * max(sLvlSigma * max(t0c, 0.0f), 1e-3f), 1.0f), gViewProj);
    o.depth = saturate(clip.z / clip.w);
    return o;
}
