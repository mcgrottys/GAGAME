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
    // M9aw: THE PAGE TENANTS. Since M9ap/M9aq the imagery and the heights are ONE reserved
    // Texture2DArray each: slices 0..5 the cube faces (painted in cube.face, no flip), the
    // rest Mercator pages (painted merc-y-south from lat/lon -- the flip IS the inverse
    // projection, as the M7l window edge documented). Sampling takes the finest page that
    // contains the direction and the residency map clamps the mip (ComposedColorPages /
    // ComposedHeightPages). The window tenants these edges used to name -- window.z14,
    // window.z17, height.window, bathy.cudem -- were deleted in M9ap..M9ar; this registry
    // kept describing them for a month (AUDIT_WATER item 1): self-consistent edges about
    // resources that did not exist, which is the one rot the validator cannot see.
    Register({"compose.stack", "color.pages", "paint cube faces", latlon, cube, false,
              "sRGB bytes", "slices 0..5, 16k faces", 1.0,
              "TileTree::Provider(ColorFrame::Cube) / ComposeCubeDir (composetest-pinned)"});
    Register({"compose.stack", "color.pages", "paint mercator pages", latlon, mercPx, true,
              "sRGB bytes", "slice 6 = z14, slice 7 = z17; tile 128^2", 1.0,
              "TileTree::Provider(ColorFrame::Window) (merc inverse per texel)"});
    Register({"compose.stack", "height.pages", "paint cube faces", latlon, cube, false,
              "m NAVD (R16F)", "slices 0..5, 16k faces", 1.0,
              "TileTree::Provider(ColorFrame::Cube), the height root"});
    Register({"compose.stack", "height.pages", "paint mercator page", latlon, mercPx, true,
              "m NAVD (R16F)", "slice 6 = z14 at 1263360,1538048; tile 256x128", 1.0,
              "TileTree::Provider(ColorFrame::Window), the height root"});
    Register({"color.pages", "globe.ps", "page-sample", mercPx, uvS, false, "sRGB",
              "finest containing page, residency-clamped mip", 1.0,
              "Compose.hlsli ComposedColorPages (no flip: both vS)"});
    // M9bg: `color.pages -> {globe,sea}.ps "bed albedo"` retired with the refracted ray. The
    // water is vertex-shaded now and takes NO imagery at all; the colour pages feed land only.
    Register({"height.pages", "globe.ps", "height", mercPx, uvS, false, "m NAVD",
              "vertex + pixel classification (M9bg: the refracted cast retired)", 1.0,
              "Compose.hlsli ComposedHeightPages"});
    Register({"google.tiles", "compose.stack", "fetch", mercPx, mercPx, false, "sRGB bytes",
              "zoom = f(groundResM)", 1.0, "GoogleColorSource::ZoomFor"});
    Register({"massgis.ortho", "compose.stack", "fetch", latlon, mercPx, true, "sRGB bytes",
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
    const Frame mercPxW{"mercator.px", false, 0, 0, 0};
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
    // Declared merc->uv (both +v=SOUTH, no flip): the north/south inversion happens
    // INSIDE the mercator closed form (the latlon->merc edge above carries flip=true),
    // exactly as the M7l window-sample edge documents. First registration of this edge
    // said latlon->uv/no-flip and the validator rightly refused it -- the checker's
    // first catch of its own author.
    Register({"height.pages", "water.bank", "bed per texel", mercPxW, uvSW, false,
              "m NAVD", "z14 slice only (AUDIT_WATER item 5); float merc ~0.25 px ulp "
              "(gatest-bounded); residency-clamped mips 2..7",
              1.0, "WaterBank.hlsl gTA[slice 6] (same formulation as CsWindowUv)"});
    const Frame wrap{"patch.wrap", true, 0, 0, 0};
    const Frame atlasN{"atlas.texel", true, 0, 0, 0};
    const Frame rowS{"raster.row0N", false, 0, 0, 0};
    // The one-water chain. Row-0-north rasters (SWE atlases, CUDEM, the shadow) demand a
    // flip into +v=north consumers; the wrap cascades and the bank/churn atlases agree
    // with world +z and demand none. These lines ARE the orientation ledger, as code.
    Register({"ocean.fft", "water.bank", "cascade.disp", wrap, atlasN, false,
              "m displacement", "+-Hs/2", 1.0, "WaterBank.hlsl CsBankFill wrap"});
    // M8 foamlaw: the Jacobian foam lives in the DERIV fiber (the disp fiber's w is
    // zero) -- the kernel takes the crest-gated UNION of the per-band answers, never
    // their sum (one physical event, detected in two representations).
    Register({"ocean.fft", "water.bank", "cascade.deriv (foam union)", wrap, atlasN,
              false, "jacobian foam 0..1", "0..1", 1.0,
              "WaterBank.hlsl CsBankFill foam discipline"});
    // M9bg: `ocean.fft -> globe.ps` lost BOTH its edges -- the per-pixel cascade sparkle and
    // the M8 caustic Jacobian. A vertex-shaded sea reads no cascade fibers in the pixel stage;
    // the FFT still reaches the water through the bank kernel, which is where it belongs.
    Register({"swe.solver", "water.bank", "eta", rowS, atlasN, true, "m dEta", "+-1.5", 1.0,
              "WaterBank.hlsl CsBankFill (1-uv.y)"});
    Register({"swe.solver", "water.bank", "uv", rowS, atlasN, true, "m/s", "+-2.5", 1.0,
              "WaterBank.hlsl CsBankFill (1-uv.y)"});
    Register({"exposure.node", "water.bank", "exposure", mercPxW, uvSW, false, "0..1 exposure",
              "0.12..1", 1.0, "WaterBank.hlsl CsBankFill (1-uv.y), floor 0.18"});
    Register({"churn.kernel", "water.bank", "churn", atlasN, atlasN, false,
              "0..1 aeration (remembered foam, MAX-composited)", "0..1", 1.0,
              "WaterBank.hlsl CsBankFill flat"});
    // M9ar: ONE BED. The committed CUDEM copy is gone; the churn, the sea and the solver
    // read the height page tenant through the same lat/lon -> Mercator-uv step the bank
    // takes (both +v south, no flip). The z14 slice is still the only one these kernels
    // resolve (AUDIT_WATER item 5).
    Register({"height.pages", "churn.kernel", "bed", mercPxW, uvSW, false, "m NAVD",
              "z14 slice; -30 m off the page", 1.0, "SeaChurn.hlsl PageBedAt"});
    Register({"height.pages", "sea.ps", "bed", mercPxW, uvSW, false, "m NAVD",
              "cube + z14 page", 1.0, "Sea.hlsl BedAt -> ComposedHeight(SeaPlanetDir)"});
    Register({"height.pages", "swe.solver", "bed", mercPxW, uvSW, false, "m NAVD",
              "z14 slice; +100 m wall off the page", 1.0,
              "Swe.hlsl BedAt (lattice -> lat/lon -> page uv, residency-clamped)"});
    Register({"swe.solver", "sea.ps", "eta", rowS, atlasN, true, "m dEta", "+-1.5", 1.0,
              "Sea.hlsl SweDEta (1-uv.y)"});
    Register({"exposure.node", "sea.ps", "exposure", mercPxW, uvSW, false, "0..1 exposure",
              "0.12..1", 1.0, "Sea.hlsl SweShadow (uv.x, 1-uv.y)"});
    // M9bg: `churn.kernel -> sea.ps` retired -- the churn atlas was a per-pixel foam texture.
    Register({"compose.stack", "water.bank", "corners", worldM, worldM, false,
              "m NAVD level/bed + hsScale", "hsScale 0.15..3", 1.0,
              "WaterBankLayer CornerParams (CPU)"});
    // M8 THE SOLVED WAVE FIELD (ALGEBRA.md wavefield). The solver's grid is row-0-SOUTH
    // (+v = north, the patch.wrap family) so the bank kernel samples it with NO flip;
    // the phase gauge (phi = 0 at the SW corner texel center, x west->east, y south->
    // north row-mean) is declared in the whitepaper -- an undeclared gauge is an
    // ambiguous field. Inputs: the one bed (stack, CPU, v-N), the tide level bucket,
    // the ACT current proxy; output: per-component (a, k, cos phi, sin phi) planes.
    Register({"compose.stack", "wave.solver", "bed (per cell)", worldM, atlasN, false,
              "m NAVD", "-40..15", 1.0, "WaveField.h SolveNow (SampleHeightStack)"});
    Register({"water.atlas", "wave.solver", "level bucket", worldM, worldM, false,
              "m NAVD", "0.25 m buckets", 1.0, "WaveField.h BucketKey"});
    Register({"act.currents", "wave.solver", "current proxy (fallback)", worldM, atlasN,
              false, "m/s (conveyance jet, x3.0 closure)", "0..2", 3.0,
              "WaveField.h (ebb toward 105, flood 285)"});
    // M8 flows into waves: the SOLVED current when the SWE window is resident -- read
    // back row-0-north (FLIP), resampled to the solve grid, x3.2 prism gain, quantized
    // 0.05 m/s, content-hashed into the bucket key.
    Register({"swe.solver", "wave.solver", "current (solved)", rowS, atlasN, true,
              "m/s (live SeaLayer gain), 0.05 buckets", "+-2.5", 1.0,
              "WaveField.h RefreshSweCurrent (1-v flip)"});
    Register({"wave.solver", "water.bank", "a/k/phase-spinor planes", mercPxW, uvSW,
              false, "m / rad/m / unit spinor (RGBA8 pages, per-comp aMax kMax)",
              "17 planes of the wave.field page tenant (z16), mip 0 pinned", 1.0,
              "WaterBank.hlsl WavePageSample (M9bc; no flip: both vS)"});
    // M9bg: the pixel stage reads the bank ONLY for the --bank-lens sanity overlay now; the
    // shading edge moved to globe.mesh below, where WaterVertexColor probes it for the wave
    // normal, sigma^2 and foam.
    Register({"water.bank", "globe.ps", "disp/param/detail (sanity lens only)", atlasN, atlasN,
              false, "m / sigma2 / m/s / band gains (g1,dry,g0,g2)", "rings 1.2..38 m/texel",
              1.0, "Globe.hlsl BankSample cubic (Catmull-Rom) + tangent bivector"});
    Register({"water.bank", "globe.mesh", "shading: normal/sigma2/foam", atlasN, atlasN, false,
              "m / sigma2 / 0..1 foam", "rings 1.2..38 m/texel", 1.0,
              "Globe.hlsl WaterVertexColor (one BankSampleT, analytic normal)"});
    Register({"water.bank", "globe.mesh", "disp+level", atlasN, atlasN, false, "m NAVD",
              "+-4", 1.0, "GlobeMesh.hlsl BankSample"});

    // ---- M7u: THE CATALOG COMPLETION -- the atlas, the weather federation, the churn's
    // own inputs, the shadow builder, the air, and the accepting state. With these the
    // graph covers the engine's data flow end to end; tools/astdiagram.py draws it.
    const Frame params{"scalar.params", true, 0, 0, 0};
    Register({"noaa.stations", "water.atlas", "harmonic fit", latlonW, latlonW, false,
              "phasor re/im per constituent", "sub-mm RMS (watertest)", 1.0,
              "harvest_tides.py -> StationFieldSource IDW p=2"});
    Register({"eot20.grid", "water.atlas", "phasor grid", latlonW, latlonW, false,
              "phasor re/im", "|P| clamp a2>100 (Fundy)", 1.0,
              "Eot20Source (epoch-rotated arg sum P conj Q)"});
    Register({"water.atlas", "window.field", "tide phasors x18 (M8i)", latlonW, mercPxW,
              true, "phasor re/im", "RG16F tiles", 1.0, "Compositor::WindowField paint"});
    Register({"water.atlas", "weather.mgr", "level rotors", latlonW, latlonW, false,
              "m NAVD", "+-3", 1.0, "WeatherManager::Query h(t)=msl+Re[P e^iwt]"});
    // M8g THE ORIGIN PLANES: the datum envelope (synodic-month min/max of the same
    // rotor sum, NAVD) -- MLLW/MHHW generalized between gauges. First consumer: the
    // edit-land geometry floor (absolute crest, so high water drowns the outer jetty).
    Register({"water.atlas", "globe.mesh", "datum envelope (origin planes)", latlonW,
              atlasN, false, "m NAVD lo/hi", "containment + width 2.4..3.6 (watertest 7)",
              1.0, "WaterAtlas::EnvelopeNavd -> gBankE.w edit floor"});
    Register({"gfswave.grid", "weather.mgr", "hs/tp/dir", rowS, latlonW, true,
              "m / s / deg", "0..15 m", 1.0, "WeatherManager wave grid (lat1-lat row)"});
    Register({"weather.mgr", "compose.stack", "corner params feed", latlonW, worldM,
              false, "level/bed/hs", "query rungs", 1.0,
              "WeatherManager::Query -> CornerParams"});
    // M9bg: `gfswave.grid -> globe.ps` (far-field whitening) and `gfs.wind -> globe.ps` (the
    // Cox-Munk sigma^2 from U10) retired. Both still reach the water, but through the BANK:
    // Hs sets the corner params below, the wind raises the FFT's sea, and the vertex reads
    // sigma^2 out of the bank's own param plane.
    // M9a: the wind's SECOND consumer. The same U10 that sets the far-field glint lobe also
    // RAISES a sea on hours where GFS-Wave's partitioning reports none -- without it those
    // hours carry swell only (a 4 mHz Gaussian, no tail) and cascades 1-2 synthesise exactly
    // zero. Point scalar out of seastate.json: no raster, no frame change, no flip.
    Register({"gfs.wind", "ocean.fft", "wind-sea fill (PM, when partitions have none)",
              latlonW, latlonW, false, "m Hs / s Tp", "Hs 0..2 over U10 0..9", 1.0,
              "SeaLayer::SetTime -> SeaState::WindSeaPm (closure windSeaFill)"});
    // M9bg: the M9 OPTICS EDGES ARE GONE FROM THE RENDERER. `ocean.colour -> globe.ps` (both
    // the Austin-Petzold K_d transfer and the Gordon deep albedo) and `gfs.icec -> globe.ps`
    // fed the two-ray water; a vertex-shaded sea has no ray path to put them in. The
    // retrievals, the closed forms and proofs/water_optics.py all stand -- nothing downstream
    // consumes them at present, which is exactly what this diagram should say.
    Register({"gfs.cloud", "cloud.volume", "density bake", rowS, atlasN, true, "0..1",
              "3D tiles 320 km col", 1.0, "GlobeLayer cloud bake (ReliefUv family)"});
    Register({"cloud.volume", "globe.ps", "density march", rowS, atlasN, true, "sigma_t",
              "14 steps + sun tap", 1.0, "Globe.hlsl ReliefUv (row0 north)"});
    Register({"mv2.windbank", "globe.ps", "curl overlay", atlasN, atlasN, false,
              "curl x1e4", "+-2.2 synoptic", 1.0, "Globe.hlsl wind overlay (V)"});
    Register({"ocean.fft", "churn.kernel", "chop deriv (pattern)", wrap, atlasN, false,
              "jacobian foam", "0..1", 1.0, "SeaChurn.hlsl (world - U dt)/patch"});
    Register({"swe.solver", "churn.kernel", "uv (blocking)", rowS, atlasN, true, "m/s",
              "+-2.5", 1.0, "SeaChurn.hlsl suv flip"});
    Register({"sea.peakdir", "exposure.node", "LOS march over the height stack (M9ba)", worldM, worldM, false,
              "0..1 exposure", "0.12..1; rebuilt on dir/level move", 1.0,
              "SeaLayer::BuildShadowMask (CPU)"});
    // M9ay: the survey is PAGES of its own tree (gis.landsea: rings swept per tile on the
    // shared addresses; r = water coverage, b = edited, a = surveyed), a third page tenant.
    Register({"compose.stack", "mask.pages", "paint survey mask", latlonW, mercPxW, true,
              "water coverage / edited / surveyed (bytes)", "cube + z14 + z17 pages", 1.0,
              "TileTree::Provider over gis.landsea (GisMaskSource sweep)"});
    Register({"mask.pages", "globe.ps", "classifier + edit override", mercPxW, uvSW, false,
              "land 0..1, edited 0..1, or no opinion", "finest page with an opinion", 1.0,
              "Compose.hlsli CsMaskSample / ComposedLandness"});
    Register({"mask.pages", "sea.ps", "classifier + edit override", mercPxW, uvSW, false, "land bit",
              "ComposedIsLand", 1.0, "Sea.hlsl ComposedIsLand"});
    Register({"globe.ps", "frame.out", "radiance (accepting state)", worldM, worldM,
              false, "linear RGB -> tonemap", "the render", 1.0, "Renderer tonemap"});
}

// M7u: the machine-readable graph -- the contract a future Blueprint-style node editor
// loads/saves, and the input of tools/astdiagram.py (the SVG catalog).
void WriteJson(const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    auto esc = [](const char* s) {
        std::string o;
        for (const char* p = s; *p; ++p) {
            if (*p == '"' || *p == '\\') { o += '\\'; o += *p; }
            else if (*p == '\n') o += "\\n";
            else o += *p;
        }
        return o;
    };
    fprintf(f, "{\n  \"version\": 1,\n  \"edges\": [\n");
    const auto& es = Edges();
    for (size_t i = 0; i < es.size(); ++i) {
        const Edge& e = es[i];
        fprintf(f,
                "    { \"from\": \"%s\", \"to\": \"%s\", \"field\": \"%s\", "
                "\"srcSpace\": \"%s\", \"srcVNorth\": %s, \"dstSpace\": \"%s\", "
                "\"dstVNorth\": %s, \"flip\": %s, \"units\": \"%s\", "
                "\"range\": \"%s\", \"gain\": %.3f, \"code\": \"%s\", "
                "\"active\": %s }%s\n",
                esc(e.from).c_str(), esc(e.to).c_str(), esc(e.field).c_str(),
                esc(e.src.space).c_str(), e.src.vNorth ? "true" : "false",
                esc(e.dst.space).c_str(), e.dst.vNorth ? "true" : "false",
                e.flip ? "true" : "false", esc(e.units).c_str(), esc(e.range).c_str(),
                e.gain, esc(e.code).c_str(), e.active ? "true" : "false",
                i + 1 < es.size() ? "," : "");
    }
    fprintf(f, "  ]\n}\n");
    fclose(f);
}

// ---- M9bi: THE SUN'S OWN EDGES ---------------------------------------------------------------
// Before this, the sun appeared in the diagram nowhere at all -- it was two floats on the
// Renderer with no producer, which is precisely the shape of graph rot the orphan rule was
// written to catch, except that a field with no producer is invisible to it. Now the clock
// produces a PLACE and the place produces a direction, and both edges are in the table.
//
// Registered only when the ephemeris is actually driving (main.cpp gates on --sun, which pins
// the old art direction and takes these edges out with it), for the same reason the
// --pixel-water edges are gated: an edge for a mode the run is not in is rot wearing the other
// sign (priors 6).
void RegisterSolarEdges() {
    // solar.au: the heliocentric space, unit length 1 AU, equatorial mean-of-date axes with the
    // celestial pole as +v. planet.re: the engine's planet frame at unit length R_earth. Both
    // are +v = north, so nothing in this chain flips -- and saying so is the point of the rule.
    const Frame clock{"scalar.params", true, 0, 0, 0};
    const Frame solarAu{"solar.au", true, 0, 0, 1.495978707e11};
    const Frame tangent{"world.m", true, 0, 0, 0};

    Register({"sim.clock", "solar.sun", "unix -> apparent RA/dec/distance", clock, solarAu,
              false, "deg / deg / AU", "dec +-23.44, 0.98329..1.01671 AU", 1.0,
              "Ephemeris.h Solar (Astronomical Almanac low-precision series, ~0.01 deg)"});
    Register({"solar.sun", "globe.ps", "sun direction (versor chain, tangent frame)", solarAu,
              tangent, false, "unit vector",
              "T,R,M,D then CsToTangent; topocentric, 8.8 arcsec of parallax", 1.0,
              "Ephemeris.h Build + SunDirFromPlanetPoint -> Renderer sunDirTangent (gSunDir)"});
    Register({"solar.sun", "sea.ps", "sun direction (versor chain, tangent frame)", solarAu,
              tangent, false, "unit vector", "the same gSunDir -- one place, every layer", 1.0,
              "Renderer SceneConstants sunDir (Sea.hlsl gSunDir)"});
    Register({"solar.sun", "globe.mesh", "sun direction (per-vertex water shading)", solarAu,
              tangent, false, "unit vector", "WaterVertexColor's lambert + Cox-Munk lobe", 1.0,
              "Globe.hlsl WaterVertexColor (gSunDir)"});
    // The second thing only a PLACED sun can supply: its own angular size, which is what the
    // disc in the sky is drawn from. A direction has no distance and therefore no disc.
    Register({"solar.sun", "globe.ps", "angular radius -> the sky's disc", solarAu, tangent,
              false, "deg", "0.2621..0.2710 over a year (Earth-Sun distance)", 1.0,
              "Common.hlsli SkyRadianceDir smoothstep(gMisc.y, gMisc.z, cosA)"});
}

// ---- M9bh: THE EDGES --pixel-water PUTS BACK ------------------------------------------------
// M9bg retired eleven edges when the water went Gouraud, and that was the truth of it: a
// vertex-shaded sea reads no imagery, no retrieval, no cascade fiber in the pixel stage. This
// function declares the ones the PIXEL path restores, and it is called ONLY when the flag is
// on -- so the printed diagram is exactly true in both modes rather than true in one of them
// (priors 6: graph rot is edges that describe a path the renderer no longer walks; an edge
// registered for a mode the run is not in is the same rot, wearing the other sign).
//
// EIGHT return; four stay retired, on purpose. The M8 caustic Jacobian, the GFS-Wave Hs
// whitening, the sea-ice albedo and the churn atlas are NOT here, because the pixel path
// carries no foam, no ice and no bed dapple. The look contract is visible as graph shape,
// which is most of the point of keeping this table.
void RegisterPixelWaterEdges() {
    const Frame mercPx{"mercator.px", false, 0, 0, 0};   // web-mercator y grows SOUTH
    const Frame uvS{"uv01.vS", false, 0, 0, 0};
    const Frame wrap{"patch.wrap", true, 0, 0, 0};
    const Frame atlasN{"atlas.texel", true, 0, 0, 0};
    const Frame rowS{"raster.row0N", false, 0, 0, 0};

    // THE REFRACTED RAY'S LANDING POINT wears the composed imagery -- in both water shaders,
    // and in each one sampled at the RAY's own bed hit, not the pixel's surface position.
    Register({"color.pages", "globe.ps", "bed albedo (--pixel-water)", mercPx, uvS, false,
              "sRGB", "at the refracted ray's bed hit, 2 secant steps", 1.0,
              "Globe.hlsl WaterPixelColor ComposedColor(bedDir)"});
    Register({"color.pages", "sea.ps", "bed albedo (--pixel-water)", mercPx, uvS, false,
              "sRGB", "at the refracted ray's bed hit in the flat frame", 1.0,
              "Sea.hlsl SeaPixelColor ComposedColor(SeaPlanetDir(bedXZ))"});
    // The height quadtree IS the scene description the refracted ray traces against; the
    // secant loop reads it at the pixel's own lod. (The vertex+pixel classification edge
    // already registered in RegisterKnownComposeEdges covers the same producer/field, so
    // this one names the CAST specifically.)
    Register({"height.pages", "globe.ps", "refracted cast (--pixel-water)", mercPx, uvS, false,
              "m NAVD", "2 secant steps, s clamped 0.3..140 m", 1.0,
              "Globe.hlsl WaterPixelColor ComposedHeight(CsToPlanet(Pb))"});
    // The cascade sparkle: bands the PIXEL resolves but the ring texel does not, each under
    // its own per-axis footprint Gaussian. No foam channel is read -- only the slope pair.
    Register({"ocean.fft", "globe.ps", "cascade.deriv slope (--pixel-water)", wrap, atlasN,
              false, "slope", "+-0.3, prefiltered per axis", 1.0,
              "Globe.hlsl WaterPixelColor detail loop"});
    // The far-field Cox-Munk lobe, where no bank ring covers the pixel.
    Register({"gfs.wind", "globe.ps", "wind10 (far sigma2, --pixel-water)", rowS, atlasN, true,
              "m/s", "0..40", 1.0,
              "Globe.hlsl WaterPixelColor wuv; sigma2 = 0.003+0.00512 U"});
    // M9 optics (docs/ALGEBRA.md "optics"): two retrievals, two questions about the same
    // pixel -- how fast light dies in this water, and what colour comes back out of it. Both
    // land in the ray path, which is the only place they have ever had a consumer.
    Register({"ocean.colour", "globe.ps", "Kd490 -> Kd(RGB) transfer (--pixel-water)", rowS,
              atlasN, true, "1/m", "0.019..6 (Kdw floor)", 1.0,
              "Globe.hlsl SampleWaterOptics (Austin-Petzold; M(490)=1)"});
    Register({"ocean.colour", "globe.ps", "chl/SPM -> deep albedo (--pixel-water)", rowS,
              atlasN, true, "albedo", "0.001..0.5", 1.0,
              "Globe.hlsl SampleWaterOptics (Gordon two-flux, gain 2.0331)"});
    // The bank stops being a sanity overlay and becomes a SHADING read again: the two ring
    // finite differences for the wave normal, the fold's shed sigma^2, the live water level
    // the translucency's depth is measured against. (The M9bg "sanity lens only" edge stays
    // registered beside this one -- --bank-lens is still its own consumer.)
    Register({"water.bank", "globe.ps", "shading: normal/sigma2/level (--pixel-water)", atlasN,
              atlasN, false, "m / sigma2 / m NAVD", "rings 1.2..38 m/texel", 1.0,
              "Globe.hlsl WaterPixelColor (one BankSampleT, analytic normal)"});
}

}  // namespace ga::ast
