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
    Register({"color.pages", "sea.ps", "bed albedo", mercPx, uvS, false, "sRGB",
              "through the refracted ray; the seafloor relief past the survey", 1.0,
              "Sea.hlsl ComposedColor(SeaPlanetDir)"});
    Register({"color.pages", "globe.ps", "bed albedo", mercPx, uvS, false, "sRGB",
              "at the refracted ray's bed hit", 1.0, "Globe.hlsl ComposedColor(bedDir)"});
    Register({"height.pages", "globe.ps", "height", mercPx, uvS, false, "m NAVD",
              "vertex, pixel, refracted cast", 1.0, "Compose.hlsli ComposedHeightPages"});
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
    Register({"ocean.fft", "globe.ps", "cascade.deriv", wrap, atlasN, false, "slope",
              "+-0.3", 1.0, "Globe.hlsl detail loop"});
    // M8 caustics: the SAME deriv fibers, sampled at the SUN ray's water entry
    // (bed - sunRun) and assembled into the ray-map Jacobian -- J in dv.z, the
    // Laplacian finite-differenced from the slope channels at cascade resolution,
    // amplitude-scaled by the detail plane's per-band gains (ALGEBRA.md caustics;
    // proofs/caustic_jacobian.py adjudicated the PHYSICAL gain form).
    Register({"ocean.fft", "globe.ps", "caustic jacobian", wrap, atlasN, false,
              "J / 1/m lap", "gain 0.35..2.6", 1.0, "Globe.hlsl M8 caustic block"});
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
    Register({"churn.kernel", "sea.ps", "churn", atlasN, atlasN, false, "0..1 aeration",
              "0..1", 1.05, "Sea.hlsl cuv flat"});
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
    Register({"water.bank", "globe.ps", "disp/param/detail", atlasN, atlasN, false,
              "m / sigma2 / m/s / band gains (g1,dry,g0,g2)", "rings 4.8..154 m/texel", 1.0,
              "Globe.hlsl BankSample manual bilinear"});
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
    Register({"gfswave.grid", "globe.ps", "hs whitening", rowS, atlasN, true, "m", "0..15",
              1.0, "Globe.hlsl wuv (lat1-lat formula)"});
    Register({"gfs.wind", "globe.ps", "wind10 (far sigma2)", rowS, atlasN, true, "m/s",
              "0..40", 1.0, "Globe.hlsl wuv; sigma2 = 0.003+0.00512 U"});
    // M9a: the wind's SECOND consumer. The same U10 that sets the far-field glint lobe also
    // RAISES a sea on hours where GFS-Wave's partitioning reports none -- without it those
    // hours carry swell only (a 4 mHz Gaussian, no tail) and cascades 1-2 synthesise exactly
    // zero. Point scalar out of seastate.json: no raster, no frame change, no flip.
    Register({"gfs.wind", "ocean.fft", "wind-sea fill (PM, when partitions have none)",
              latlonW, latlonW, false, "m Hs / s Tp", "Hs 0..2 over U10 0..9", 1.0,
              "SeaLayer::SetTime -> SeaState::WindSeaPm (closure windSeaFill)"});
    // M9 (docs/ALGEBRA.md "optics"): the water's QUALITY. Two edges into the same consumer,
    // because the two retrievals answer two different questions about the same pixel -- how
    // fast light dies in it, and what colour comes back out. Both land in Globe.hlsl's ray
    // path where M7c had constants.
    Register({"ocean.colour", "globe.ps", "Kd490 -> Kd(RGB) transfer", rowS, atlasN, true,
              "1/m", "0.019..6 (Kdw floor)", 1.0,
              "Globe.hlsl SampleWaterOptics (Austin-Petzold; M(490)=1)"});
    Register({"ocean.colour", "globe.ps", "chl/SPM -> deep albedo", rowS, atlasN, true,
              "albedo", "0.001..0.5", 1.0,
              "Globe.hlsl SampleWaterOptics (Gordon two-flux, gain 2.0331)"});
    Register({"gfs.icec", "globe.ps", "ice albedo + glint damp", rowS, atlasN, true, "0..1",
              "concentration", 1.0, "Globe.hlsl wuv (wave grid); sigma2 *= 1-0.95c"});
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

}  // namespace ga::ast
