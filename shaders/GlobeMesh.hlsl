// ================================================================================================
//  GlobeMesh.hlsl - M6j: THE UNIFIED PLANET SURFACE. One mesh-shader pipeline draws the Earth
//  (and Mars) from orbit to the helm, geometry amplified straight from the composed height
//  channel -- the terrain layer stops being a second description of the same planet.
//
//  How the float wall falls: the CPU CDLOD walk (which also drives residency wants) emits one
//  RECORD per 8x8-cell meshlet. For fine meshlets (arc <= 650 m, i.e. level >= 14) the record
//  carries a DOUBLE-precision camera-relative anchor and the position Jacobian d(pos)/d(uv) at
//  the meshlet centre; the mesh shader reconstructs vertices as anchor + J.du with du built
//  from small integer cell offsets -- no 6.4e6-magnitude float subtraction anywhere, so the
//  surface is millimetre-stable at walking height. Coarse meshlets use the classic exact
//  float path (their >= 2 km viewing distance hides the ~0.6 m dir jitter below a pixel).
//  Linearization error at 650 m span is  span^2/2R  ~ 3 cm -- under the fine texel noise.
//
//  Heights and colors come from Compose.hlsli at CONTINUOUS lod (the z14 window takes over
//  from the cube in the sampler): CUDEM detail at the beach, ETOPO at the horizon, MOLA on
//  Mars, one code path. PsMain is shared verbatim with the fallback VS pipeline.
// ================================================================================================
#define GA_MESH_PATH
#include "Globe.hlsl"

// Mirrors GlobeLayer::MeshletRec (96 B).
struct MeshletRec {
    float2 uv0;          // node face-uv origin
    float2 uvStepCell;   // face-uv per node CELL (node size / 32)
    uint face;
    uint cell0;          // (cellY0 << 8) | cellX0 -- this meshlet's 8x8 window in node cells
    float morphStart;    // the node's CDLOD cross-fade band (camera distance, m)
    float morphEnd;
    float3 anchorRel;    // camera-relative meshlet centre at the geoid (CPU doubles)
    float arc;           // node ground span (m): path pick + vertex-density lod
    float3 dPdu;         // tangent-frame position per face-uv unit, at the centre
    uint seamX;          // step 23: the node-boundary seam across the W (mx 0) / E (mx 3) edge
    float3 dPdv;
    uint seamY;          // step 23: the same across the N (my 0) / S (my 3) edge
    float3 upT;          // tangent-frame up at the centre
    uint level;          // M10: the Droste level slot -- anchorRel, dPdu/dPdv and upT are all in
                         // THAT level's own frame, relative to ITS eye (Droste.h, the gauge)
};
StructuredBuffer<MeshletRec> gMeshlets : register(t0, space0);

// Step 23 (docs/PERF_EXPERIMENT.md): THE SEAM WORD, packed by GlobeLayer::SeamTable.
//   bits 0..16  the coarse neighbour's meshlet record across the seam (M10: 17 bits -- the
//               record budget is 2^17 since the Droste levels)
//   bits 17..18 relation: 0 none (a same-level or finer neighbour, another face, or no
//               neighbour), 1 one level coarser
//   bit 19      this meshlet meets the upper half (cells 4..7) of that record's edge
static const uint kSeamNone = 0u, kSeamCoarser = 1u;

// M9bm: the lateral term is fully applied above this areaJac and rolls off to zero at the
// overturn. 0.35 sits under foamlaw's saturation knee (J = 0.45), so the guard only ever acts
// where the foam law has already declared the crest broken.
static const float kLatFoldFloor = 0.35f;

// ONE vertex of the surface: the record's grid position g (node cells, 0..32), CDLOD-morphed
// and displaced, as the M6j path always computed it. MsMain's 81 vertices and the seam bands
// below both come from here, so a band vertex is the neighbour's own arithmetic on the
// neighbour's own record (the same doubles the neighbour amplifies from), not an estimate.
VsOut SurfaceVertex(const MeshletRec rec, uint gid, float2 g) {
    const float cx0 = float(rec.cell0 & 0xFFu);
    const float cy0 = float(rec.cell0 >> 8);
    const bool fine = rec.arc <= 650.0f;

    // Pre-morph distance estimate for the CDLOD ramp.
    float d0;
    if (fine) {
        const float2 duC = (g - float2(cx0 + 4.0f, cy0 + 4.0f)) * rec.uvStepCell;
        d0 = length(rec.anchorRel + rec.dPdu * duC.x + rec.dPdv * duC.y);
    } else {
        const float3 dir0 = CubeDir(rec.face, rec.uv0 + g * rec.uvStepCell);
        d0 = length(CsToTangent(dir0) * gGlo.x - sLvlCamAbs);   // M10: the level's own eye
    }
    const float k = saturate((d0 - rec.morphStart) / max(rec.morphEnd - rec.morphStart, 1.0f));
    g -= frac(g * 0.5f) * 2.0f * k;

    const float2 uv = rec.uv0 + g * rec.uvStepCell;
    const float3 dir = CubeDir(rec.face, uv);   // float dir: fine for sampling the channels
    // THE WATER'S SAMPLE POINT (VsOut.geo): the undisplaced point of the geoid under this vertex,
    // eye-relative in the level's own tangent frame. A fine meshlet has it exactly -- the record's
    // double-precision anchor plus small offsets, the float wall's own arithmetic, and the base
    // its position is displaced from -- and the coarse path forms it as it forms its position,
    // without the displacement.
    float3 geo;
    if (fine) {
        const float2 du = (g - float2(cx0 + 4.0f, cy0 + 4.0f)) * rec.uvStepCell;
        geo = rec.anchorRel + rec.dPdu * du.x + rec.dPdv * du.y;
    } else {
        geo = CsToTangent(dir) * gGlo.x - sLvlCamAbs;
    }

    // Height at the vertex's own density: texels no finer than the vertex spacing feed
    // displacement; anything finer feeds pixel normals instead.
    // Height lod: NEAR nodes (<= 2.6 km) ride the FINEST RESIDENT data -- the same the
    // per-pixel classifier reads, so geometry and classification cannot disagree (the
    // vertex-land/pixel-water "plates"). Far nodes match data to vertex density, and the
    // CDLOD morph blends the height SOURCE along with the grid so ring handovers cannot
    // step into cliffs.
    const float texelM = gCsG.y * gGlo.x;
    const float vlod = clamp(log2(max(rec.arc / 32.0f, 0.01f) / texelM), -8.0f, gCsG.x);
    const float vl = (rec.arc <= 2600.0f) ? -8.0f : vlod;
    const float vlP = (rec.arc <= 1300.0f) ? -8.0f : vlod + 1.0f;
    float h = lerp(ComposedHeight(dir, vl), ComposedHeight(dir, vlP), k);
    // Geometry obeys the same classifier the pixels use: WATER rides ~2 m BELOW the live
    // waterline (not the geoid -- at low tide the geoid stands PROUD of the real sea and
    // buries the FFT surface; this was the M6j flat-sea bug). Land keeps its height.
    // M6n: the mix is ANALOG (ComposedLandness): a half-emerged flat sits halfway between
    // the drowned plane and its true height, so streaming height data slides shorelines
    // smoothly instead of popping plateau edges (the flats speckle, geometry side).
    // HIERARCHY 4.17 commit 3: the standing blocks are addressed by the undisplaced point geo
    // (above) in the camera's own level; another level's vertex has its direction only.
    const float3 blkP = (rec.level == 0u) ? geo : CsPointOfDir(dir);
    const float landness =
        ComposedLandness(dir, blkP, ComposedHeight(dir, max(vl, -4.0f)), gWavesB.w);
    // M6p/M8g: an operator's LAND edit floors the display height where the height
    // channel's SMEAR dips -- but at an ABSOLUTE crest elevation (NAVD, scene
    // jettyCrestNavd), never relative to the live tide. The old floor tracked the
    // waterline (+1.2 m), which made the jetty unsinkable by construction; the real
    // north jetty goes awash at high water (the user's catch). Surveyed data taller
    // than the floor still wins through the max below.
    const float editFloor = ComposedEditLand(dir, blkP) * gBankE.w;
    // M10: the relief exaggeration is a display choice made at the level's OWN altitude.
    const float dispLand = max(max(h, 0.0f) * sLvlExag, editFloor * sLvlExag);
    // M7: ONE WATER. In one-water mode the vertex samples THE WAVE VERTEX BANK -- level
    // (tide + solver) plus the folded cascade displacement, one tiled resource, ring LOD.
    // Beyond every ring (or bank off) the M6t sunk plane remains: it exists only so the
    // SeaLayer grid can cover it, and in one-water mode that grid is retired.
    float dispWater = min(gWavesB.w - 8.0f, -8.0f);
    // M8g: the FULL displacement vector. The vertex used to take level + dy only;
    // in shallow water the ORBITAL (horizontal) displacement dominates the vertical
    // (coth(kh) -- the trace showed dx -0.48 m against dy +0.06 m), and it is what
    // sharpens crests and hollows troughs. Normals carried the waves, the mesh
    // stayed flat (seen, fixed). Lateral fades with landness so shoreline verts
    // never slide onto the rocks; the bank's world xz stays the SAMPLE point --
    // Gerstner convention, same as the reference.
    float2 latW = 0.0f;
    if (gBankU.z != 0u) {
        float4 bD, bP, bDet;
        // A fine meshlet reads the bank at its exact point, the level's eye plus geo: the
        // coordinate made from dir has a third of a metre of grain at the helm (REVIEW finding
        // 42). A coarse one reads it as it always has; at its distances that grain is under a
        // pixel.
        const float2 bankXZ = fine ? sLvlCamAbs.xz + geo.xz : (CsToTangent(dir) * gGlo.x).xz;
        float bT;
        float3 dDdx, dDdz;
        dispWater = BankSampleT(bankXZ, bD, bP, bDet, bT, dDdx, dDdz) ? (bP.x + bD.y) : 0.0f;
        // M9bm: THE FOLD GUARD.
        //
        // The lateral term was applied with nothing bounding the Jacobian it induces. Where
        // d(Dx)/dx approaches -1 neighbouring vertices cross, the surface turns over, and the
        // mesh renders a near-vertical WALL with the water visible behind it -- the user's
        // "gap ... it looks like the water behind it has the section of wave it's missing".
        // Seen only under --wireflat: shaded, the crease reads as a wave face.
        //
        // areaJac = det(I + J_h) IS the tangent bivector's own magnitude (ALGEBRA
        // "caustics"), and it reaches zero exactly at the overturn -- "breaking is the
        // crest's area 2-blade degenerating". So the object already computed for the normal
        // is the one that detects the fold; no new quantity, no threshold pulled from the air.
        //
        // Scaling the lateral term to zero as the blade degenerates loses NOTHING physical:
        // foamlaw's Jacobian trigger, saturate((0.80 - J) * 4), has already been paying that
        // excess out as foam since M8. This is the same law the depth-limited breaking clamp
        // states -- "converts the excess into foam instead of geometry" -- applied to the
        // horizontal component, which was the one term still exempt from it.
        const float areaJac =
            (1.0f + dDdx.x) * (1.0f + dDdz.z) - dDdz.x * dDdx.z;
        latW = float2(bD.x, bD.z) * (1.0f - landness) *
               smoothstep(0.0f, kLatFoldFloor, areaJac);
    }
    const float disp = lerp(dispWater, dispLand, landness);

    VsOut o;
    o.mid = gid;   // M9b: this record's index, for PsMeshlet's tint
    o.lvl = rec.level;
    o.dir = dir;
    o.h = h;
    if (fine) {
        o.rel = geo + rec.upT * disp;
    } else {
        o.rel = CsToTangent(dir) * (gGlo.x + disp) - sLvlCamAbs;
    }
    o.rel += float3(latW.x, 0.0f, latW.y);
    o.geo = geo;   // the label: the lateral term moves the drawn point, never where it is read
    // M9bg: the water's colour, shaded AT THIS VERTEX from its own wave normal (the bank's
    // Loads -- stage-proof) and the analytic sky. The pixel stage does no water work.
    // EVERY vertex is shaded, including dry ones: a shoreline triangle's land corner is still
    // READ by the pixels partway across it (landness there is < 1), so skipping it to save the
    // three bank probes would interpolate a black corner into the surf. Over land the bank's
    // tiles are NULL and read zero, so a dry vertex costs three cheap Loads and lands on the
    // shallow tint -- the right colour for exactly the pixels that mix it.
    o.wcol = WaterVertexColor(dir, o.rel, h);
    // M10: the one outward step of the gauge -- rel stays in the level's own frame for the
    // pixel stage; the rasterizer gets the true camera-relative position (s^k Q^k rel). Level 0
    // is the identity, so the camera's own planet rasterizes exactly as it always did.
    o.pos = mul(float4(TrueRel(o.rel), 1.0f), gViewProj);
    return o;
}

// ------------------------------------------------------------------------------------------
// Step 23: THE SEAM BANDS -- the shell made watertight (docs/ALGEBRA.md priors 28).
//
// MEASURED (--lens shell, --settle-exact): the surface had holes. At the 7 km key pose
// seven pixels, at the bird thirteen, each a ray that left the shell through a seam and
// landed on the far side of the planet. Two classes, one mechanism:
//   (a) LEVEL seams. A leaf splits on its CENTRE distance (WalkNode: dist < 3 arc) and its
//       finer neighbour morphs out over [4.05, 5.85] x ITS arc, i.e. up to 2.93 x the coarse
//       arc -- but a coarse leaf only promises its centre is past 3 arc, so its near corner
//       can sit at 2.29 arc, where the fine side's morph is still k ~ 0.3..0.9: odd seam
//       vertices off the coarse edge (T-junctions) and even ones on a blended height the
//       coarse side never samples. All seven key7km cracks are this.
//   (b) FINE-PATH seams. Fine meshlets (arc <= 650 m) reconstruct anchor + J.du from THEIR
//       OWN record, so two records sharing a vertex round it to ~0.3 mm apart; at 2 km that
//       is a 1e-4 px hairline and a pixel centre lands in one every ~100k px of seam. Eleven
//       of the bird's thirteen.
// The fix cannot move a vertex: on this GPU the interpolated dir's ulp is 0.4 m of ground, so
// any retessellation flips 8-bit pixels far from the crack (step 23's declared difference is
// the crack pixels ONLY). So the seam is CLOSED, not re-cut: the owning meshlet emits a thin
// band along each seam, pushed kSeamDepth behind in depth (pos.z scaled: reversed-Z, GREATER
// wins). Behind by 0.5 % of the distance it loses to every real fragment it overlaps and wins
// only where nothing else was drawn -- the crack -- against the far side or the sky.
//   Hairline bands (b): every fine record's W and N seams, unconditionally (each same-level
//   seam is then closed once; a finer or coarser neighbour has its own level band). The
//   inner side is the record's own seam vertex with ONLY its depth changed (bit-identical
//   screen xy, so the edge it shares with the record's own triangles stays single-covered);
//   the outer side is the same vertex nudged kSeamOut cells outward in the tangent plane.
//   No texture work and no neighbour record: the band is flat over 0.002 cells, and the
//   neighbour's surface is within slope x 0.002 cells of it.
//   Level bands (a): the fine record on any of its four edges that meets a coarser leaf (the
//   seam word from GlobeLayer::SeamTable). The fine side is again its own seam vertices; the
//   coarse side is the COARSE record's own vertex (SurfaceVertex on that record: the same
//   doubles it amplifies from) at our five even positions, nudged into the coarse leaf by
//   kSeamIn cells or a metre, whichever is more -- the classic path rounds a position to
//   0.5 m, and a nudge past that keeps the band's edge inside the coarse triangle whatever
//   the codegen of the recomputation. The band spans the height step between the two
//   surfaces whatever its size.
// Why the margins are what they are (MEASURED at the helm, v1 of this band): a band that
// straddled the seam by 0.02 cells on both sides, flat, sat ~7 mm off the sloping wave
// surface, and at a 3.8 deg depression 7 mm of height is 10 cm along the ray = a 0.1 %
// margin at 100 m, so the band won over real fragments on 39 pixels. One-sided at 0.002
// cells and 0.5 % behind, the same mismatch is 0.7 mm against a 50 cm margin.
// Classic-path same-level seams need nothing: both sides evaluate one formula on
// bit-identical uv and are coincident by construction.
// ------------------------------------------------------------------------------------------
static const float kSeamOut = 0.002f;     // hairline band width, cells, outward of the seam
static const float kSeamIn = 0.01f;       // level band: the coarse side's inset, cells
static const float kSeamInMinM = 1.0f;    // ... and at least this, metres (the float wall)
static const float kSeamDepth = 5.0e-3f;  // relative push behind, in ndc depth (near/dist)
static const uint kBandNone = 0u, kBandHairline = 1u, kBandLevel = 2u;
static const uint kBandVerts[3] = {0u, 18u, 14u};   // none, hairline strip, level T-band
static const uint kBandTris[3] = {0u, 16u, 12u};

struct Band {
    uint kind;      // kBandNone / kBandHairline / kBandLevel
    uint nb;        // level: the coarse neighbour's record
    uint off;       // level: the coarse record's cell offset along the seam (0 or 4)
    uint vbase;     // this band's first vertex, relative to 81
    uint tbase;     // this band's first triangle, relative to 128
};

// The four potential bands of a meshlet, in edge order W, E, N, S.
void SeamBands(const MeshletRec rec, out Band b[4], out uint nv, out uint nt) {
    const uint mx = (rec.cell0 & 0xFFu) >> 3;
    const uint my = (rec.cell0 >> 8) >> 3;
    const bool fine = rec.arc <= 650.0f;
    uint kind[4] = {kBandNone, kBandNone, kBandNone, kBandNone};
    uint nb[4] = {0u, 0u, 0u, 0u};
    uint off[4] = {0u, 0u, 0u, 0u};
    // Level seams first: the seam words name a coarser neighbour across the node's W/E edge
    // (records with mx 0/3) and N/S edge (my 0/3).
    const uint relX = (rec.seamX >> 17) & 3u, relY = (rec.seamY >> 17) & 3u;
    if (mx == 0u && relX == kSeamCoarser) {
        kind[0] = kBandLevel; nb[0] = rec.seamX & 0x1FFFFu; off[0] = ((rec.seamX >> 19) & 1u) * 4u;
    }
    if (mx == 3u && relX == kSeamCoarser) {
        kind[1] = kBandLevel; nb[1] = rec.seamX & 0x1FFFFu; off[1] = ((rec.seamX >> 19) & 1u) * 4u;
    }
    if (my == 0u && relY == kSeamCoarser) {
        kind[2] = kBandLevel; nb[2] = rec.seamY & 0x1FFFFu; off[2] = ((rec.seamY >> 19) & 1u) * 4u;
    }
    if (my == 3u && relY == kSeamCoarser) {
        kind[3] = kBandLevel; nb[3] = rec.seamY & 0x1FFFFu; off[3] = ((rec.seamY >> 19) & 1u) * 4u;
    }
    // Hairline bands on the fine path's W and N seams where no level band already sits.
    if (fine) {
        if (kind[0] == kBandNone) kind[0] = kBandHairline;
        if (kind[2] == kBandNone) kind[2] = kBandHairline;
    }
    nv = 0u;
    nt = 0u;
    [unroll] for (uint e = 0u; e < 4u; ++e) {
        b[e].kind = kind[e];
        b[e].nb = nb[e];
        b[e].off = off[e];
        b[e].vbase = nv;
        b[e].tbase = nt;
        nv += kBandVerts[kind[e]];
        nt += kBandTris[kind[e]];
    }
}

// A tangent-plane nudge of `cells` cells across edge e (0 W, 1 E, 2 N, 3 S), pointing INTO
// the record for inward = true and out of it otherwise.
float3 SeamNudge(const MeshletRec rec, uint e, bool inward, float cells) {
    const float s = ((e == 0u || e == 2u) == inward) ? cells : -cells;
    return (e < 2u) ? rec.dPdu * (rec.uvStepCell.x * s) : rec.dPdv * (rec.uvStepCell.y * s);
}

// The band's depth: the vertex pushed kSeamDepth behind (ndcZ = near / viewZ, so a smaller
// clip z is farther). The record id's top bit marks a band vertex: PsMain never reads mid,
// the --lens shell reports a band fragment as the hole it filled (a band, behind the seam
// surface by kSeamDepth, can only be seen where no real fragment was drawn within that
// margin in front of it), and PsMeshlet's tint hashes it like any record.
VsOut BandDepth(VsOut o) {
    o.pos.z *= (1.0f - kSeamDepth);
    o.mid |= 0x80000000u;
    return o;
}

// The same vertex moved in the tangent plane, then pushed behind. Its water sample point moves
// with it, by the same nudge.
VsOut BandMoved(VsOut o, float3 nudge) {
    o.rel += nudge;
    o.geo += nudge;
    o.pos = mul(float4(TrueRel(o.rel), 1.0f), gViewProj);   // M10: the gauge, as SurfaceVertex
    return BandDepth(o);
}

// The coarse side of level band e: the coarse neighbour's own vertex at our even seam
// position 2i (its cells run half as fine), nudged into the coarse leaf.
VsOut CoarseBandVertex(uint gid, uint e, const Band bd, uint i) {
    const MeshletRec nr = gMeshlets[bd.nb];
    const float ncx0 = float(nr.cell0 & 0xFFu);
    const float ncy0 = float(nr.cell0 >> 8);
    // The coarse record's seam edge faces ours: its E edge for our W seam, and so on.
    const float fixedN = (e == 0u) ? ncx0 + 8.0f : (e == 1u) ? ncx0
                       : (e == 2u) ? ncy0 + 8.0f : ncy0;
    const float along = ((e < 2u) ? ncy0 : ncx0) + float(bd.off) + float(i);
    const float2 g = (e < 2u) ? float2(fixedN, along) : float2(along, fixedN);
    const float cells = max(kSeamIn, kSeamInMinM / max(nr.arc / 32.0f, 1.0f));
    // Into the coarse leaf: across ITS facing edge (e ^ 1), inward.
    return BandMoved(SurfaceVertex(nr, gid, g), SeamNudge(nr, e ^ 1u, true, cells));
}

// Band triangle t of band bd, as indices into verts[] (the band's vertices start at
// 81 + bd.vbase; hairline: 9 seam vertices then 9 outward copies, a strip; level: 9 seam
// vertices then 5 coarse vertices, three triangles per pair of our cells reaching their one).
uint3 BandTri(const Band bd, uint t) {
    const uint v0 = 81u + bd.vbase;
    if (bd.kind == kBandHairline) {
        const uint j = t >> 1;
        return (t & 1u) ? uint3(v0 + 9u + j, v0 + 9u + j + 1u, v0 + j + 1u)
                        : uint3(v0 + j, v0 + 9u + j, v0 + j + 1u);
    }
    const uint i = t / 3u;
    const uint w = t - i * 3u;
    const uint c0 = v0 + 9u + i, c1 = c0 + 1u;
    const uint f0 = v0 + 2u * i, f1 = f0 + 1u, f2 = f0 + 2u;
    return (w == 0u) ? uint3(c0, c1, f1) : (w == 1u) ? uint3(c0, f1, f0) : uint3(c1, f2, f1);
}

// One thread block per meshlet: 9x9 = 81 vertices, 8x8x2 = 128 triangles, plus up to four
// seam bands (step 23: at most 64 vertices and 56 triangles more).
[outputtopology("triangle")]
[numthreads(128, 1, 1)]
void MsMain(uint gtid : SV_GroupThreadID, uint3 gid3 : SV_GroupID,
            out vertices VsOut verts[145], out indices uint3 tris[184]) {
    // M10: the dispatch is 2-D (one dimension caps at 65535 groups); fold it back to the record.
    // The tail groups of the last row emit nothing (gGlo.z = the record count) -- through the ONE
    // SetMeshOutputCounts the validator allows (two call sites fail validation even on exclusive
    // paths, and a failed MsMain silently drops the whole mesh pipeline), with the record read
    // clamped in bounds (a root SRV has no bounds check).
    const uint count = (uint)gGlo.z;
    const uint gid = gid3.y * 65535u + gid3.x;
    const bool live = gid < count;
    const MeshletRec rec = gMeshlets[min(gid, max(count, 1u) - 1u)];
    // M10: this record's Droste level -- its eye, sun, exaggeration, bank set and the gauge's
    // outward map. A seam band's coarse neighbour is always a record of the SAME level (the
    // seam table keys on it), so one load serves every vertex this group writes.
    LoadLevel(rec.level);
    Band bands[4];
    uint nv, nt;
    SeamBands(rec, bands, nv, nt);
    SetMeshOutputCounts(live ? 81u + nv : 0u, live ? 128u + nt : 0u);
    if (!live) return;
    const float cx0 = float(rec.cell0 & 0xFFu);
    const float cy0 = float(rec.cell0 >> 8);

    if (gtid < 81u) {
        // Node-grid coordinates (0..32): the morph operates here, so shared edges between
        // meshlets and between nodes evaluate identically -- crack-free exactly as the VS
        // path was.
        const uint x = gtid % 9u, y = gtid / 9u;
        const float2 g = float2(cx0 + float(x), cy0 + float(y));
        const VsOut o = SurfaceVertex(rec, gid, g);
        verts[gtid] = o;
        // Step 23: a seam vertex also writes its band copies -- itself, pushed behind, for
        // any band on its edge, and its outward copy for a hairline band. A corner vertex
        // sits on two edges.
        [unroll] for (uint e = 0u; e < 4u; ++e) {
            const bool onEdge = (e == 0u) ? x == 0u : (e == 1u) ? x == 8u : (e == 2u) ? y == 0u : y == 8u;
            if (bands[e].kind == kBandNone || !onEdge) continue;
            const uint j = (e < 2u) ? y : x;
            verts[81u + bands[e].vbase + j] = BandDepth(o);
            if (bands[e].kind == kBandHairline) {
                verts[81u + bands[e].vbase + 9u + j] =
                    BandMoved(o, SeamNudge(rec, e, false, kSeamOut));
            }
        }
    }

    if (gtid < 128u) {
        const uint cell = gtid / 2u;
        const uint x = cell % 8u, y = cell / 8u;
        const uint v0 = y * 9u + x;
        tris[gtid] = (gtid & 1u) ? uint3(v0 + 1u, v0 + 10u, v0 + 9u)
                                 : uint3(v0, v0 + 1u, v0 + 9u);
    }

    // Step 23: the coarse vertices of the level bands (five per band, threads 0..19 at most)
    // and every band triangle (threads 0..nt-1).
    {
        uint nc = 0u;
        [unroll] for (uint e = 0u; e < 4u; ++e) {
            if (bands[e].kind != kBandLevel) continue;
            if (gtid >= nc && gtid < nc + 5u) {
                verts[81u + bands[e].vbase + 9u + (gtid - nc)] =
                    CoarseBandVertex(gid, e, bands[e], gtid - nc);
            }
            nc += 5u;
        }
    }
    if (gtid < nt) {
        uint e = 0u;
        [unroll] for (uint q = 1u; q < 4u; ++q) {
            if (bands[q].kind != kBandNone && gtid >= bands[q].tbase) e = q;
        }
        tris[128u + gtid] = BandTri(bands[e], gtid - bands[e].tbase);
    }
}
