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
    float pad0;
    float3 dPdv;
    float pad1;
    float3 upT;          // tangent-frame up at the centre
    float pad2;
};
StructuredBuffer<MeshletRec> gMeshlets : register(t0, space0);

// One thread block per meshlet: 9x9 = 81 vertices, 8x8x2 = 128 triangles.
[outputtopology("triangle")]
[numthreads(128, 1, 1)]
void MsMain(uint gtid : SV_GroupThreadID, uint gid : SV_GroupID,
            out vertices VsOut verts[81], out indices uint3 tris[128]) {
    SetMeshOutputCounts(81, 128);
    const MeshletRec rec = gMeshlets[gid];
    const float cx0 = float(rec.cell0 & 0xFFu);
    const float cy0 = float(rec.cell0 >> 8);
    const bool fine = rec.arc <= 650.0f;

    if (gtid < 81u) {
        // Node-grid coordinates (0..32): the morph operates here, so shared edges between
        // meshlets and between nodes evaluate identically -- crack-free exactly as the VS
        // path was.
        float2 g = float2(cx0 + float(gtid % 9u), cy0 + float(gtid / 9u));

        // Pre-morph distance estimate for the CDLOD ramp.
        float d0;
        if (fine) {
            const float2 duC = (g - float2(cx0 + 4.0f, cy0 + 4.0f)) * rec.uvStepCell;
            d0 = length(rec.anchorRel + rec.dPdu * duC.x + rec.dPdv * duC.y);
        } else {
            const float3 dir0 = CubeDir(rec.face, rec.uv0 + g * rec.uvStepCell);
            d0 = length(CsToTangent(dir0) * gGlo.x - gCamAbs.xyz);
        }
        const float k = saturate((d0 - rec.morphStart) / max(rec.morphEnd - rec.morphStart, 1.0f));
        g -= frac(g * 0.5f) * 2.0f * k;

        const float2 uv = rec.uv0 + g * rec.uvStepCell;
        const float3 dir = CubeDir(rec.face, uv);   // float dir: fine for SAMPLING always

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
        const float landness =
            ComposedLandness(dir, ComposedHeight(dir, max(vl, -4.0f)), gWavesB.w);
        // M6p: an operator's LAND edit floors the display height -- the jetty stands as a
        // continuous ridge above the tide even where the height channel's smear dips.
        const float editFloor = ComposedEditLand(dir) * max(gWavesB.w + 1.2f, 1.2f);
        const float dispLand = max(max(h, 0.0f) * gGlo.y, editFloor * gGlo.y);
        const float dispWater = min(gWavesB.w - 2.0f, -2.0f);
        const float disp = lerp(dispWater, dispLand, landness);

        VsOut o;
        o.dir = dir;
        o.h = h;
        if (fine) {
            const float2 du = (g - float2(cx0 + 4.0f, cy0 + 4.0f)) * rec.uvStepCell;
            o.rel = rec.anchorRel + rec.dPdu * du.x + rec.dPdv * du.y + rec.upT * disp;
        } else {
            o.rel = CsToTangent(dir) * (gGlo.x + disp) - gCamAbs.xyz;
        }
        o.pos = mul(float4(o.rel, 1.0f), gViewProj);
        verts[gtid] = o;
    }

    if (gtid < 128u) {
        const uint cell = gtid / 2u;
        const uint x = cell % 8u, y = cell / 8u;
        const uint v0 = y * 9u + x;
        tris[gtid] = (gtid & 1u) ? uint3(v0 + 1u, v0 + 10u, v0 + 9u)
                                 : uint3(v0, v0 + 1u, v0 + 9u);
    }
}
