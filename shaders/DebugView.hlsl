// ================================================================================================
//  DebugView.hlsl - THE DEBUG VIEWS of the globe's surface (the scene's `debug` section,
//  scene/SceneSchema.h DebugSection): what the mesh is and what it carries, read as geometry and
//  as numbers rather than through the look. Compiled ONLY when a view or the wire is asked for
//  (GlobeLayer::BuildDebugPso), in a file of its own for ResidencyLens.hlsl's measured reason:
//  an entry point beside PsMain moved the shipped picture, so an instrument that must change no
//  picture while it is off lives outside the file the picture is built from.
//
//  THE ROWS. A debug view excludes the residency lens, so it takes the lens's rows for its own:
//    gLensU.x  the view: 0 shaded (PsMain), 1 flat grey, 2 edges, 3 normals, 4 height, 5 depth
//    gLensU.y  the wire's colour: 0 its own (gLensA.rgb), 1 by edge length, 2 by depth
//    gLensA    the wire's rgb and its width in pixels (0 = no wire)
//    gLensB    x the height's contour interval (m), y z the depth's near and far (m), w the edge
//              length the edge hues are centred on (m)
//
//  ONE LAW FOR THE TRIANGLE'S SIZE. The rasterizer gives the barycentrics b and their screen
//  derivatives; the fragment's point is p = p0 + b1 e1 + b2 e2, so the derivatives of p are
//  [ddx p; ddy p] = J [e1; e2] with J the 2x2 of the barycentrics' derivatives, and the edges are
//  J^-1 times them -- the triangle's own edges in metres, whichever way it is seen, with no
//  buffer read. The wire is the same barycentrics: a fragment within w pixels of an edge is
//  the edge whose barycentric, over its own screen rate, is below w.
// ================================================================================================
#include "Globe.hlsl"

// A hue from a scalar on a log ladder: one octave of x is one sixth of the wheel, so a factor of
// two in size is always the same step in colour, at any size.
float3 DvHue(float octaves) {
    const float h = frac(octaves / 6.0f) * 6.0f;
    return saturate(float3(abs(h - 3.0f) - 1.0f, 2.0f - abs(h - 2.0f), 2.0f - abs(h - 4.0f)));
}

// The triangle's two edges from the fragment, in the level's tangent metres (the header's law).
void DvEdges(float3 rel, float3 b, out float3 e1, out float3 e2) {
    const float3 px = ddx(rel), py = ddy(rel);
    const float a = ddx(b.y), c = ddx(b.z), d = ddy(b.y), f = ddy(b.z);
    const float det = a * f - c * d;
    const float inv = abs(det) > 1e-20f ? 1.0f / det : 0.0f;
    e1 = (f * px - c * py) * inv;
    e2 = (a * py - d * px) * inv;
}

// East, north and up at a planet direction, in the tangent frame: up is the direction, east is
// up times the pole (d/dlon of (cos lat cos lon, sin lat, cos lat sin lon) is pole x up, negated),
// north completes the right-handed triple.
void DvEnu(float3 dirPlanet, out float3 east, out float3 north, out float3 up) {
    up = CsToTangent(normalize(dirPlanet));
    const float3 pole = CsToTangent(float3(0.0f, 1.0f, 0.0f));
    const float3 ex = cross(up, pole);
    east = dot(ex, ex) > 1e-12f ? normalize(-ex) : CsToTangent(float3(1.0f, 0.0f, 0.0f));
    north = cross(up, east);
}

float4 PsDebugView(VsOut i, float3 bary : SV_Barycentrics) : SV_Target {
    LoadLevel(i.lvl);
    if (gGateA.x >= 0.0f) {
        const uint depth = LevelGateDepth(i.lvl);
        if (GateDepth(TrueRel(i.rel), depth + 1u) != depth) discard;
    }
    if (gBankA.w > 0.5f) {
        if ((CsToTangent(normalize(i.dir)) * gGlo.x).z > gBankC.w) discard;
    }
    const uint view = gLensU.x;
    const float dist = length(i.rel);
    float3 e1, e2;
    DvEdges(i.rel, bary, e1, e2);
    const float edgeM = max(length(e1), length(e2));
    const float edgeOct = log2(max(edgeM, 1e-6f) / max(gLensB.w, 1e-6f));
    const float depthT = saturate(log2(max(dist, gLensB.y) / max(gLensB.y, 1e-3f)) /
                                  max(log2(max(gLensB.z, gLensB.y * 2.0f) / max(gLensB.y, 1e-3f)), 1e-3f));

    float3 c;
    if (view == 0u) {
        c = PsMain(i).rgb;
    } else if (view == 2u) {
        c = DvHue(edgeOct);
    } else if (view == 3u) {
        float3 east, north, up;
        DvEnu(i.dir, east, north, up);
        float3 n = normalize(cross(ddy(i.rel), ddx(i.rel)));
        if (dot(n, up) < 0.0f) n = -n;
        c = float3(dot(n, east), dot(n, north), dot(n, up)) * 0.5f + 0.5f;
    } else if (view == 4u) {
        // Hypsometric: the sea's blues darken with depth, the land runs green to brown to white
        // over 0..2000 m; a line every interval, a heavier one every fifth.
        const float h = i.h;
        c = h < 0.0f ? lerp(float3(0.55f, 0.80f, 0.95f), float3(0.02f, 0.06f, 0.30f), saturate(-h / 200.0f))
                     : (h < 300.0f ? lerp(float3(0.20f, 0.55f, 0.20f), float3(0.75f, 0.70f, 0.35f), h / 300.0f)
                                   : lerp(float3(0.75f, 0.70f, 0.35f), float3(0.97f, 0.97f, 0.97f), saturate((h - 300.0f) / 1700.0f)));
        const float stepM = max(gLensB.x, 1e-4f);
        const float k = h / stepM, w = max(fwidth(k), 1e-6f);
        const float ln1 = 1.0f - saturate(abs(frac(k + 0.5f) - 0.5f) / w);
        const float k5 = h / (5.0f * stepM), w5 = max(fwidth(k5), 1e-6f);
        const float ln5 = 1.0f - saturate(abs(frac(k5 + 0.5f) - 0.5f) / (1.6f * w5));
        c *= 1.0f - 0.45f * max(ln1, ln5);
    } else if (view == 5u) {
        c = DvHue(depthT * 5.0f) * (1.0f - 0.6f * depthT);
    } else {
        c = PsWireFlat(i).rgb;
    }

    const float wPx = gLensA.w;
    if (wPx > 0.0f) {
        const float3 r = bary / max(fwidth(bary), 1e-6f);
        const float nearPx = min(r.x, min(r.y, r.z));
        const float cover = 1.0f - smoothstep(wPx - 0.5f, wPx + 0.5f, nearPx);
        const float3 wc = gLensU.y == 1u ? DvHue(edgeOct) : gLensU.y == 2u ? DvHue(depthT * 5.0f) : gLensA.rgb;
        c = lerp(c, wc, cover);
    }
    return float4(c, 1.0f);
}
