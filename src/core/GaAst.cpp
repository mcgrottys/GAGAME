#include "GaAst.h"

#include <cstdio>
#include <cstring>

#include "Common.h"

namespace ga::ast {

namespace {
std::vector<Edge>& Reg() {
    static std::vector<Edge> r;
    return r;
}
}  // namespace

void Register(const Edge& e) {
    for (Edge& x : Reg()) {
        if (!std::strcmp(x.from, e.from) && !std::strcmp(x.to, e.to) &&
            !std::strcmp(x.field, e.field)) {
            x = e;
            return;
        }
    }
    Reg().push_back(e);
}

void SetActive(const char* to, bool active) {
    for (Edge& x : Reg()) {
        if (!std::strcmp(x.to, to)) x.active = active;
    }
}

const std::vector<Edge>& Edges() { return Reg(); }

void Print() {
    Log("[gaast] ---- THE STATE DIAGRAM (%zu edges; frames are law) ----", Reg().size());
    const char* last = "";
    for (const Edge& e : Reg()) {
        if (std::strcmp(last, e.from)) {
            Log("[gaast] node %s", e.from);
            last = e.from;
        }
        char sf[96], df[96];
        snprintf(sf, sizeof(sf), "%s%s", e.src.space, e.src.vNorth ? "(+v=N)" : "(+v=S)");
        snprintf(df, sizeof(df), "%s%s", e.dst.space, e.dst.vNorth ? "(+v=N)" : "(+v=S)");
        char geo[128] = "";
        if (e.src.metersPerUnit > 0.0) {
            snprintf(geo, sizeof(geo), "  org(%.0f,%.0f) %.4gm/u", e.src.orgX, e.src.orgY,
                     e.src.metersPerUnit);
        }
        Log("[gaast]   -> %-12s %-14s %-22s->%-22s%s%s  %s in %s  x%.3g%s  [%s]", e.to,
            e.field, sf, df, e.flip ? "  FLIP" : "      ", geo, e.range, e.units, e.gain,
            e.active ? "" : "  (INACTIVE)", e.code);
    }
}

bool Validate() {
    bool ok = true;
    for (const Edge& e : Reg()) {
        // THE FLIP RULE: frames that disagree about +v need exactly one flip in the code.
        const bool need = e.src.vNorth != e.dst.vNorth;
        if (need != e.flip) {
            Log("[gaast] FAIL flip rule: %s -> %s '%s': src %s dst %s but code %s  [%s]",
                e.from, e.to, e.field, e.src.vNorth ? "+v=N" : "+v=S",
                e.dst.vNorth ? "+v=N" : "+v=S", e.flip ? "FLIPS" : "does not flip", e.code);
            ok = false;
        }
    }
    // M7n: THE CONVENTION RULE -- both sides of an edge must agree where samples LIVE
    // (cell centers vs lattice corners); disagreement is an undeclared half-lattice
    // translation, the exact class the coincidence card caught in the bank kernel.
    for (const Edge& e : Reg()) {
        if (e.src.centers != e.dst.centers) {
            Log("[gaast] FAIL convention rule: %s -> %s '%s': src %s dst %s -- undeclared "
                "half-lattice translation  [%s]",
                e.from, e.to, e.field, e.src.centers ? "centers" : "corners",
                e.dst.centers ? "centers" : "corners", e.code);
            ok = false;
        }
    }
    // THE ORPHAN RULE: a field someone produces that nothing active consumes is dead physics
    // walking -- the swell shadow fed only the retired SeaLayer draw for a whole milestone.
    for (const Edge& e : Reg()) {
        if (e.active) continue;
        bool live = false;
        for (const Edge& x : Reg()) {
            if (!std::strcmp(x.from, e.from) && !std::strcmp(x.field, e.field) && x.active) {
                live = true;
                break;
            }
        }
        if (!live) {
            Log("[gaast] WARN orphan: %s '%s' has no ACTIVE consumer (only %s, inactive)",
                e.from, e.field, e.to);
        }
    }
    if (ok) Log("[gaast] validate: %zu edges, flip rule holds on every one", Reg().size());
    return ok;
}

void WriteMarkdown(const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    const char nl = '\n';
    fprintf(f, "# THE GA AST -- the state diagram's edges (generated every run; do not "
               "hand-edit)%c%c", nl, nl);
    fprintf(f, "Nodes are GA engines; edges carry geometric products. `+v=N` / `+v=S` is "
               "the second axis' compass direction; FLIP marks where the sampling code "
               "inverts v. The flip rule (frames that disagree need exactly one flip) and "
               "the orphan rule are enforced by gatest and at every boot.%c%c", nl, nl);
    fprintf(f, "| from | field | to | src frame | dst frame | flip | units | range | gain "
               "| code anchor |%c|---|---|---|---|---|---|---|---|---|---|%c", nl, nl);
    for (const Edge& e : Reg()) {
        fprintf(f, "| %s | %s | %s%s | %s %s | %s %s | %s | %s | %s | x%.3g | %s |%c",
                e.from, e.field, e.to, e.active ? "" : " (inactive)", e.src.space,
                e.src.vNorth ? "+v=N" : "+v=S", e.dst.space, e.dst.vNorth ? "+v=N" : "+v=S",
                e.flip ? "FLIP" : "-", e.units, e.range, e.gain, e.code, nl);
    }
    fclose(f);
}

void RegisterKnownComposeEdges() {
    const Frame latlon{"latlon.deg", true, 0, 0, 0};
    const Frame mercPx{"mercator.px", false, 0, 0, 0};   // web-mercator y grows SOUTH
    const Frame uvS{"uv01.vS", false, 0, 0, 0};
    const Frame cube{"cube.face", true, 0, 0, 0};        // per-face D3D spec dirs
    const Frame resMap{"resmap.texel", false, 0, 0, 0};
    // THE COMPOSITOR PILLAR. The paint loop iterates raster rows (merc y south) and
    // resolves each texel to lat/lon -- that inversion IS the flip. The shader-side window
    // uv keeps mercator orientation, so sampling needs NO flip (unlike the water atlases:
    // this asymmetry is exactly what the table exists to keep straight).
    Register({"compose.stack", "window.z14", "paint", latlon, mercPx, true,
              "sRGB bytes / m NAVD", "tile 128^2", 1.0,
              "Compositor::WindowColor/WindowHeight (merc inverse per texel)"});
    Register({"compose.stack", "window.z17", "paint", latlon, mercPx, true, "sRGB bytes",
              "tile 128^2", 1.0, "Compositor::WindowColor zBase 17"});
    Register({"compose.stack", "cube.color", "paint", latlon, cube, false, "sRGB bytes",
              "16k faces", 1.0, "ComposeCubeDir (D3D cube convention, composetest-pinned)"});
    Register({"window.z14", "globe.ps", "window-sample", mercPx, uvS, false,
              "sRGB / m NAVD", "uv 0..1", 1.0, "Compose.hlsli CsWindowUv (no flip: both vS)"});
    Register({"window.z17", "globe.ps", "detail-sample", mercPx, uvS, false, "sRGB",
              "finer-only gate", 1.0, "Compose.hlsli detail rung (M7f/M7h handoff)"});
    Register({"google.tiles", "window.z14", "fetch", mercPx, mercPx, false, "sRGB bytes",
              "zoom = f(groundResM)", 1.0, "GoogleColorSource::ZoomFor"});
    Register({"massgis.ortho", "window.z14", "fetch", latlon, mercPx, true, "sRGB bytes",
              "EPSG:6348 UTM19N declared", 1.0, "AerialOrthoSource (TM forward)"});
    Register({"height.stack", "synth.bed", "classify", latlon, latlon, false,
              "m NAVD -> dry albedo", "3 samples/texel", 1.0,
              "BedSynthSource::Sample (M7d cross-channel edge)"});
    // THE RESIDENCY PILLAR. Wants come from the CDLOD walk in each frame's uv boxes; the
    // CPU map (byte = mip*16) is uploaded and sampled by every composed consumer as the
    // resolution CLAMP -- want vs have divergence is the vintage-patchwork mechanism the
    // M7g mip floor bounded.
    Register({"globe.walk", "residency.mgr", "wants", uvS, uvS, false, "mip requests",
              "mips 0..7 + floor 4..7", 1.0, "GlobeLayer node walk + M7g mip floor"});
    Register({"residency.mgr", "globe.ps", "have-map", resMap, uvS, false,
              "finest mip * 16 (R8)", "0..7*16", 1.0, "CsHave2D residency clamp"});
}

void RegisterKnownWaterEdges() {
    RegisterKnownComposeEdges();
    const Frame worldM{"world.m", true, 0, 0, 0};
    const Frame latlonW{"latlon.deg", true, 0, 0, 0};
    const Frame uvSW{"uv01.vS", false, 0, 0, 0};
    // M7r: the M7q per-texel bed chain, with its PRECISION contract. The kernel repeats
    // CsWindowUv's formulation (absolute float mercator px minus org): ~0.25 px ulp at
    // z14 = ~2.4 m ground -- bounded by gatest's merc-chain test, invisible under 9.55 m
    // texels. The world->latlon step is the FLAT-ONE-WORLD map (mPerLon frozen at the
    // anchor): absolute georegistration drifts ~0.2 km at the window's far corners, but
    // every water consumer shares the same map, so the water cannot disagree with itself.
    Register({"world.flat", "latlon.deg", "anchor-linear map", worldM, latlonW, false,
              "deg", "mPerLon frozen at anchor; shared by ALL water consumers", 1.0,
              "BathyModel::kOrgLat/kMPerLat convention"});
    Register({"height.window", "water.bank", "bed per texel", latlonW, uvSW, false,
              "m NAVD", "float merc ~0.25 px ulp (gatest-bounded); residency-clamped "
              "mips 2..7",
              1.0, "WaterBank.hlsl M7q (same formulation as CsWindowUv)"});
    const Frame wrap{"patch.wrap", true, 0, 0, 0};
    const Frame atlasN{"atlas.texel", true, 0, 0, 0};
    const Frame rowS{"raster.row0N", false, 0, 0, 0};
    // The one-water chain. Row-0-north rasters (SWE atlases, CUDEM, the shadow) demand a
    // flip into +v=north consumers; the wrap cascades and the bank/churn atlases agree
    // with world +z and demand none. These lines ARE the orientation ledger, as code.
    Register({"ocean.fft", "water.bank", "cascade.disp", wrap, atlasN, false,
              "m disp + jacobian foam", "+-Hs/2", 1.0, "WaterBank.hlsl CsBankFill wrap"});
    Register({"ocean.fft", "globe.ps", "cascade.deriv", wrap, atlasN, false, "slope",
              "+-0.3", 1.0, "Globe.hlsl detail loop"});
    Register({"swe.solver", "water.bank", "eta", rowS, atlasN, true, "m dEta", "+-1.5", 1.0,
              "WaterBank.hlsl CsBankFill (1-uv.y)"});
    Register({"swe.solver", "water.bank", "uv", rowS, atlasN, true, "m/s", "+-2.5", 1.0,
              "WaterBank.hlsl CsBankFill (1-uv.y)"});
    Register({"swe.solver", "water.bank", "shadow", rowS, atlasN, true, "0..1 exposure",
              "0.12..1", 1.0, "WaterBank.hlsl CsBankFill (1-uv.y), floor 0.18"});
    Register({"churn.kernel", "water.bank", "churn", atlasN, atlasN, false,
              "0..1 aeration", "0..1", 1.05, "WaterBank.hlsl CsBankFill flat"});
    Register({"bathy.cudem", "churn.kernel", "bed", rowS, atlasN, true, "m NAVD", "-40..15",
              1.0, "SeaChurn.hlsl suv flip"});
    Register({"bathy.cudem", "sea.ps", "bed", rowS, atlasN, true, "m NAVD", "-40..15", 1.0,
              "Sea.hlsl:71 (uv.x, 1-uv.y)"});
    Register({"swe.solver", "sea.ps", "eta", rowS, atlasN, true, "m dEta", "+-1.5", 1.0,
              "Sea.hlsl SweDEta (1-uv.y)"});
    Register({"swe.solver", "sea.ps", "shadow", rowS, atlasN, true, "0..1 exposure",
              "0.12..1", 1.0, "Sea.hlsl SweShadow (uv.x, 1-uv.y)"});
    Register({"churn.kernel", "sea.ps", "churn", atlasN, atlasN, false, "0..1 aeration",
              "0..1", 1.05, "Sea.hlsl cuv flat"});
    Register({"compose.stack", "water.bank", "corners", worldM, worldM, false,
              "m NAVD level/bed + hsScale", "hsScale 0.15..3", 1.0,
              "WaterBankLayer CornerParams (CPU)"});
    Register({"water.bank", "globe.ps", "disp/param/detail", atlasN, atlasN, false,
              "m / sigma2 / m/s / hsScale*expo", "rings 4.8..154 m/texel", 1.0,
              "Globe.hlsl BankSample manual bilinear"});
    Register({"water.bank", "globe.mesh", "disp+level", atlasN, atlasN, false, "m NAVD",
              "+-4", 1.0, "GlobeMesh.hlsl BankSample"});
}

}  // namespace ga::ast
