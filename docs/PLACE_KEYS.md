# Where the engine names one place: the work list for step 8

A read-only map made on 2026-09-29 by an Opus agent for `docs/HIERARCHY.md` step 8, which
turns what names the mouth of the Merrimack in code into keys of the scene. Nothing was
changed, built or run to make it. Fable opened six of its rows against the source (a1, a5,
c1, c3, c6, d3) and found each as written; the rest stand on the agent's reading.

**The lines are not `main`'s.** The tree that was read is `main` at `35a9eb7` with six pieces
applied that are not committed yet (the harvest, the prune tool, step 4a, step 1a, step 1
and step 2's probe), so a line number in a file those pieces touch, `Assembly.cpp` and
`FrameLoop.cpp` among them, stands a few lines from `main`'s. The quoted text is the thing to
search for. A row marked **inferred** names something not opened at that line.

What HIERARCHY 4.15 had counted is inside this map: its anchor is row c1 (the header is
`src\sim\BathyModel.h`), its windows e1 and e2, its paths a1 to a18, its stations b1 to b4,
its compass d1 and d2, its zone c2 and c3, its box c4.

The classes:

- (a) a path or a name of data
- (b) a station's or a buoy's identifier
- (c) a coordinate, an anchor, a zone or a box
- (d) an assumption about geography written as code, in C++ or in a shader
- (e) a window or a lattice built for one place
- (f) a default that is this place's, in the schema, in the options, or in a member's
  initialiser that no key reaches

"Key today" names a key of the schema that already carries the value. "Proposed key" follows
the schema's existing names (`data.bathy`, `sea.datum.mllwToNavd`, `water.swe.riverQ`,
`water.wavefield.orgX`, `views[].at`, `gates[].toLat`, `tools[]`). "Default" is today's
literal, so that a scene that says nothing behaves as today.

---

## 1. The table

### (a) Paths and names of data: 23 rows

| # | class | file:line | literal or assumption | who reads it | key today | proposed key | default |
|---|---|---|---|---|---|---|---|
| a1 | a | src\app\Assembly.cpp:485 | `if (auto bld = breg.Open("data/bathy/merrimack.json")) {` | the bed of the boot's depth tree `water.depth = tide - bed` (Assembly.cpp:486-497) and its 24 hourly equivalence probes, a log (:507-538). Opens this file whatever the scene's `data.bathy` says | `data.bathy` | (use the key) | `data/bathy/merrimack.json` |
| a2 | a | src\app\Assembly.cpp:1111 | `if (auto cld = freg.Open("data/currents/currents.json")) {` | the current field composed into the solver's velocity-gradient bank (swe.velgrad, levels 3 and up, :1119-1138). The current model itself reads `S.data.currents` (:336); this line does not | `data.currents` | (use the key) | `data/currents/currents.json` |
| a3 | a | src\app\Assembly.cpp:360, 397-400 | `bathyCapeAnn.Load("data/bathy/capeann.json");` and the layer name `"noaa.cudem.capeann"` | a CUDEM layer of the height stack (ETOPO < NE-15s < capeann < boston < the scene's bed < edits, :388-414) -> the solver's bed (RealizeFromChannel, :433), the height pages, the globe; the name enters the layer's cache identity | none | `data.insets[]` {file, name} | `[{"file": "data/bathy/capeann.json", "name": "noaa.cudem.capeann"}, {"file": "data/bathy/boston.json", "name": "noaa.cudem.boston"}]` |
| a4 | a | src\app\Assembly.cpp:361, 402-405 | `bathyBoston.Load("data/bathy/boston.json");` and `"noaa.cudem.boston"` | the same height stack, and the Boston dormant solver's grid (FrameLoop.cpp:1039-1052, row b3) | none | `data.insets[]` (a3) | as a3 |
| a5 | a | src\compose\Sources.h:127; src\app\Assembly.cpp:408 | `const char* name = "noaa.cudem.merrimack");` and `srcCudem = std::make_unique<CudemHeightSource>(&bathyRaw);` (no name given) | the name of the layer built from the scene's `data.bathy`, whatever place it holds -> the layer's name in the height stack -> tile-tree tags and cache identity (**inferred** from the tag `noaa.cudem.merrimack.0f5b0281` in the fixture at TreePruneTest.cpp:889) | none | `data.bathyName` | `"noaa.cudem.merrimack"` |
| a6 | a | src\sim\BathyModel.cpp:45 | `root.Str("file", "merrimack.f32")` | the grid file when the bathy json names none -> BathyModel::Load (the solver's grid, the terrain, the CUDEM layer) | none needed: the json names its file (the harvested jsons are "BathyModel format", **inferred** from the manifests; not opened) | none | `"merrimack.f32"` |
| a7 | a | src\app\Assembly.cpp:411, 435; src\compose\GisMask.cpp:250 | `srcEdits.Load("data/gis/edits.geojson", 2.5f)`; `bathy.ApplyMaskEdits("data/gis/edits.geojson", 2.5f);`; `LoadEdits(dir + "edits.geojson");` | the hand-edit polygons (the jetties): the top height layer "survey.edits" at a crest of 2.5 m NAVD88 (Sources.cpp:366-369, 388), the fallback walls when there is no channel, and the GIS mask's edit rings | none | `data.edits`, `data.editsCrestNavd` | `"data/gis/edits.geojson"`, `2.5` |
| a8 | a | src\app\Assembly.cpp:421 | `waterAtlas.Init(compositor, model, "data/water");` | the water atlas folder: EOT20 phasors `%s/eot20_%s.json` (WaterAtlas.cpp:102) -> the tide channels water.tide.* -> the level everywhere. The content is global; the folder is fixed | none | `data.water` | `"data/water"` |
| a9 | a | src\compose\WaterAtlas.cpp:178 | `info.name = std::string("tide.stations.ne.") + WaterAtlas::kConName[c];` | the name of the station field built from `data.tides`, whatever place it holds -> the compose stack's source names (and cache identity, **inferred**) | none | `data.tidesName` | `"tide.stations.ne"` |
| a10 | a | src\app\Assembly.cpp:544-545 | `: LoadRiverDischarge("data/river/river.json");` | the river discharge when `water.swe.riverQ` is not positive -> the Flather west boundary's transport (FrameLoop.cpp:1014-1019) | `water.swe.riverQ` carries a number, not the file | `data.river` | `"data/river/river.json"` |
| a11 | a | src\app\Assembly.h:158 | `const char* kScenePath = "data/wave_scene.json";` | the water scene (the solved wave field's window and the closures): loaded at boot (Assembly.cpp:564), hot-reloaded (FrameLoop.cpp:1185), written from SceneConfig.h:126-153 if absent. scenes\merrimack.json:15 also includes it at `water`, so it overlays any scene whose base is merrimack.json | none | `data.waterScene` | `"data/wave_scene.json"` |
| a12 | a | src\app\Assembly.cpp:850; src\compose\Sources.cpp:240-242 | `srcAerial.Load("data/aerial/aerial.json")`; `v.Str("name", "massgis.coq2023")`, `v.Str("crs", "EPSG:6348 NAD83(2011)/UTM 19N (~1 m vs WGS84, uncorrected)")` | the MassGIS 15 cm orthos, the colour stack's layer above Google -> the colour tenant | none | `data.aerial` | `"data/aerial/aerial.json"` |
| a13 | a | src\app\Assembly.cpp:863; src\compose\Sources.cpp:445, 501 | `srcBed.Load("data/bed/bed_rules.json", &compositor, hgtCh)`; `"zones": "data/bed/bed_zones.geojson"`; `root.Str("zones", "data/bed/bed_zones.geojson")` | the bed classifier, the water's colour authority, gated by the GIS mask (:881-895); its program carries a box of this coast (row c5) and is written from the code's default if absent (Sources.cpp:459-463) | none | `data.bedRules` | `"data/bed/bed_rules.json"` |
| a14 | a | src\app\Assembly.cpp:867 | `srcOverlay.Load("data/overlay/overlay.json")` | hand overlays at the top of the colour stack | none | `data.overlay` | `"data/overlay/overlay.json"` |
| a15 | a | src\app\Assembly.cpp:883; src\compose\GisMask.cpp:246, 249 | `gisMask.Load("data/gis/")`; `ReadRings(dir + "coast_ne.bin", ...)`; `ReadRings(dir + "nhd_water_ne.bin", m_water);` | the vector land/sea gate that multiplies the bed and relief layers' weight (:882-895). Without coast rings the gate does not exist (GisMask.cpp:251-255) | none | `data.gisMask` {dir, coast, water} | `"data/gis/"`, `"coast_ne.bin"`, `"nhd_water_ne.bin"` |
| a16 | a | src\app\Assembly.cpp:1327, 1329; src\compose\GisStencil.cpp:41-42; src\scene\GisLayer.cpp:46, 55-56, 72-73, 85-86 | `gisStencil.Load("data/gis/gis.json")`; `vectors.Load("data/vectors/vectors.vpack");`; `v.Str("coast_ne")`, `v.Str("rivers_ne")`; `"gis.coast.ne"`, `"gis.rivers.ne"` | the survey pack and the vector overlay layer (GisLayer, drawn under --stencil); also read by the fidelity-map and water-map tools (FidelityMap.cpp:249, WaterMap.cpp:187) | none | `data.gisStencil`, `data.vectors`, `data.vectorLayers` | `"data/gis/gis.json"`, `"data/vectors/vectors.vpack"`, `["coast_ne", "rivers_ne"]` |
| a17 | a | src\app\FrameLoop.cpp:1194 | `if (waterBank) route.Load("data/gis/route_merrimack.json");` | the AIS traffic lane the fleet's boats ride (`water.fleet.boats`) | `water.fleet` has no lane | `water.fleet.route` | `"data/gis/route_merrimack.json"` |
| a18 | a | src\sim\GlobeModel.cpp:107-108, 112; src\app\Assembly.cpp:390-394 | `js.Num("ne_lon0", -72.0)`, `js.Num("ne_lat1", 45.0)`, `js.Str("ne_file", "ne_15s.i16")`; `"noaa.etopo15s.ne", "window-grid int16 1440x1200"` | the New England 15-arcsecond relief window inside data\globe\globe.json -> the height stack's second layer. Its box and file are data with this place's defaults; its name is code | none | `data.reliefInset` {file, name} | `"ne_15s.i16"`, `"noaa.etopo15s.ne"` (box from globe.json, else -72.0 / 45.0) |
| a19 | a | src\sim\CurrentModel.cpp:105; src\core\CurrentFieldLoader.h:160 | `jf->Str("file", "gomofs_uv.f32")` | the current field's file when currents.json's `field` names none; the data carries `field.file` | none needed | none | `"gomofs_uv.f32"` |
| a20 | a | src\core\CurrentFieldLoader.h:49, 176 | `"gomofs uv float32 planar (row 0 south)"`; `m_name = "noaa.gomofs.current";` | every current field loaded through the plugin seam is declared GoMOFS -> the velocity-gradient bank's source (Assembly.cpp:1109-1112) and its logs (`grad(GoMOFS)`, :1171, :1242) | none | `data.currentsName` | `"noaa.gomofs.current"` |
| a21 | a | src\sim\SweSolver.h:45; src\app\FrameLoop.cpp:1038, 1052 | `const char* name = "merrimack";`; `weather.AddExternalWindow("merrimack", ...)`; `weather.AddDormantWindow("boston", ...)` | the solver windows' names: logs and the title's window list (WeatherManager.h:142) | none | `water.swe.name` (Boston's in b3) | `"merrimack"` |
| a22 | a | src\app\FrameLoop.cpp:607 | `m_tangentSpace.name = marsMode ? "tangent.mars" : "tangent.merrimack";` | the tangent space's name (Space::Declare, the diagram) | none | `place.name` | `"merrimack"` |
| a23 | a | src\app\FrameLoop.cpp:142; src\scene\GulfLayer.cpp:92; src\sim\WeatherManager.cpp:313, 344; src\app\Tools\Trace.cpp:88 | `Hs model %.2f m (44013 obs %.2f m)`; `"GoMOFS at 44029: %.2f m/s @%03.0f"`; `"gomofs.surface (700 m)"`; `"gfswave gulf point (Hs/Tp/dir)"`; `"[trace] ==== cross-check: NOAA tides at 8440452, GoMOFS currents, "` | title-bar, sample-source and log words | none | none: they name what b4, b5, a20 and c4 carry | today's words |

### (b) Identifiers of stations, buoys and gauges: 7 rows

| # | class | file:line | literal or assumption | who reads it | key today | proposed key | default |
|---|---|---|---|---|---|---|---|
| b1 | b | src\sim\TideModel.cpp:102 | `if (m_stations[i].id == "8440466") m_focus = static_cast<int>(i);   // Newburyport` | `model.Focus()`: the MLLW -> NAVD88 link of the whole water level (ResolveDatum, Assembly.cpp:141-155 -> datumOff), the start of the boundary-station search (FrameLoop.cpp:969), the depth tree's probe (Assembly.cpp:510), the ribbon's focus (TideLayer.cpp:114). Absent from the file: station 0 (TideModel.cpp:99) | none | `data.tideFocus` | `"8440466"` |
| b2 | b | src\app\FrameLoop.cpp:1103; src\scene\SeaLayer.h:44; src\app\Tools\SweCycle.h:118 | `currents.StationIndex("ACT0816")` | the entrance jet's speed and flood/ebb bearings (SeaLayer.cpp:492-514 -> Sea.hlsl, SeaChurn through Jet.hlsli), the wave field's current (FrameLoop.cpp:1110-1112), the swe-cycle validation. Absent: the jet is off (SeaLayer.cpp:510) | none | `data.currentStation` | `"ACT0816"` |
| b3 | b | src\app\FrameLoop.cpp:1042, 1050-1052 | `if (model.S(i).id == "8443970") iBos = ...`; `bcfg.spongeX0 = -2500.0f;   // Mass Bay, east of the outer harbor islands`; `bcfg.westBoundary = false;  // the Charles is dammed`; `AddDormantWindow("boston", &bathyBoston, bcfg, oceanAtBoston, 0.5)` | Boston Harbor's dormant solver: its ocean clock and datum link, its sponge, its wall, 0.5 h spin-up, bed a4, bed page slice 6 (:1053-1057) | none | `water.swe.dormant[]` {name, bathy, station, spongeX0, westBoundary, spinupH} | `[{"name": "boston", "bathy": "data/bathy/boston.json", "station": "8443970", "spongeX0": -2500, "westBoundary": false, "spinupH": 0.5}]` |
| b4 | b | src\scene\SeaLayer.cpp:30, 216, 423 | `m_sea->Buoy("44013")` | the title's buoy Hs, the assimilation gain on every partition's energy (:215-228, the CPU twin fed from the same call, :231-234), the spectrum plot's dashed curve (SpecPlot.hlsl:3). Absent: no assimilation, silently | none | `sea.buoy` | `"44013"` |
| b5 | b | src\sim\CurrentModel.cpp:91 | `adcpId = adcp->Str("id", "44029");` | the ADCP buoy's id when currents.json's `buoy_adcp` names none; the data carries `buoy_adcp.id` | none needed | none | `"44029"` |
| b6 | b | src\app\Assembly.cpp:95-96, 103-106 | `if (id == "01100000") lowell = ...`; `if (id == "01100500") lawrence = ...` | the USGS gauges read from a10's file -> riverQ -> the Flather west boundary | none | `water.swe.riverGauges` | `["01100500", "01100000"]` |
| b7 | b | src\app\Tools\OceanProbe.cpp:61 | `for (const char* sid : {"8443970", "8441841"}) {` | the ocean-probe tool's station comparison (Boston, Gloucester) | none | `tools[]` entry of ocean-probe: `stations` (ToolSchema at SceneSchema.cpp:553, its fields not read) | `["8443970", "8441841"]` |

### (c) Coordinates, the anchor, zones and boxes: 13 rows

| # | class | file:line | literal or assumption | who reads it | key today | proposed key | default |
|---|---|---|---|---|---|---|---|
| c1 | c | src\sim\BathyModel.h:25-28 | `kOrgLon = -70.81; kOrgLat = 42.81833; kMPerLon = 81660.0;    // 111320 * cos(42.818 deg)` `kMPerLat = 110574.0;` | the flat world frame (x east, z north, metres) of: the bed's placement BathyModel.cpp:60-64 and so the solver's dx, dy (SweSolver.cpp:187-195); the CUDEM layers Sources.cpp:316-322, 397-398; the velgrad page Assembly.cpp:1129-1134; the hull's water WaterSurfaceTree.cpp:22-23, 138-139; the kernels' world.flat chart SurfaceFrame.cpp:23-26; the tangent frame FrameLoop.cpp:568-569; the globe's estuary window FrameLoop.cpp:646-650; camera and Droste conversions FrameLoop.cpp:2598-2599, 3254-3257, 3273-3274, 3357-3360; WaveField.cpp:957-959, 1044-1046, 1093-1100; WaveFieldSource.h:63-82; WeatherManager.cpp:19-20; SeaLayer.h:126-127; ExposureSource.h:146-147; WaterBankLayer.cpp:133-134; tools DumpWaterState.cpp:33-35, Trace.cpp:31-39, TwinSurface.cpp:111-112 | none | `place.anchor` {lat, lon}, `place.mPerLat`, `place.mPerLon` | `42.81833`, `-70.81`, `110574`, `81660`. 111320 cos(42.81833 deg) is about 81,655, not 81,660: a default derived from the anchor moves a point 10 km out by about 0.7 m |
| c2 | c | src\compose\Sources.cpp:253-255 | `static const TransverseMercator kUtm19 = TransverseMercator::Utm(19);` | every aerial tile is found in zone 19 whatever aerial.json declares; the json's `crs` is a label (a12). Haulover is in zone 17, the Chesapeake in 18 | none | `data.aerialZone` | `19` |
| c3 | c | src\compose\DomainSource.h:260-263 | `int zone = 19;` ... `else if (g.epsg == 6348) zone = 19;` | the zone of any transverse-Mercator raster read through the loader: 326xx and 327xx map to their zones, 6348 to 19, every other code (NAD83(2011) zone 17 is EPSG:6346, zone 18 is 6347) to 19 | none | none as a scene key: the zone belongs to the file's declaration (section 4) | `19` |
| c4 | c | src\sim\WeatherManager.cpp:336-337 | `latDeg > 41.5 && latDeg < 44.5 && lonDeg > -71.2 &&` `lonDeg < -68.5` | where the seastate point forecast's Tp and direction (and Hs where the global grid is empty) are believed -> every WeatherManager::Query. Outside it Tp and direction stay unset | none | `sea.pointBox` {lat0, lat1, lon0, lon1} | `{41.5, 44.5, -71.2, -68.5}` |
| c5 | c | src\compose\Sources.cpp:439, 472-473 | `"bounds": {"lon0": -71.15, "lat0": 42.15, "lon1": -70.25, "lat1": 43.10}`; `b->Num("lon0", -71.15)` ... `b->Num("lat1", 43.10)` | the bed classifier's footprint: it paints only inside it, the global relief outside. The box is data in bed_rules.json; the code writes this default program if the file is absent | the file of a13 | none beyond `data.bedRules` | the box |
| c6 | c | src\scene\WaterComponent.cpp:301, 304, 309 | `floorNavd = 1.8f;`; `m_o.atlas->EnvelopeNavd(42.8190, -70.8031, 0.0, &elo, &ehi);`; `floorNavd = ehi - 0.45f;` | the edit-land geometry floor when `water.closures.jettyCrestNavd` <= -90, the default: the tide envelope sampled at the Merrimack's north jetty | `water.closures.jettyCrestNavd` overrides the result, not the point | `water.closures.jettyAt` {lat, lon} | `{42.8190, -70.8031}` |
| c7 | c | src\scene\GulfLayer.h:25 | `static constexpr double kBuoyLon = -70.566, kBuoyLat = 42.523;` | buoy 44029's place: the gulf map's marker ring (GulfLayer.cpp:76-77 -> Gulf.hlsl:68) and the field-against-ADCP gate (GulfLayer.cpp:84-92). currents.json carries the ADCP's id, not its place | none | `data.adcpAt` {lat, lon} | `{42.523, -70.566}` |
| c8 | c | src\app\FrameLoop.cpp:538 | `if (bathy.Ready()) camSea.SetFromCompass(522.0, 7.0, 72.0, 246.0f, -4.0f);` | the sea camera, "the north jetty tip", before the scene's `sea` view overrides it (:544) | `views[].at` of `sea` (merrimack.json:109) | (use the key) | `{"x": 522, "alt": 7, "z": 72, "az": 246, "pitch": -4}` |
| c9 | c | src\app\FrameLoop.cpp:555 | `camGlobe = scene::GlobeCamera(34.0, -52.0, planetR * 2.1, planetR);` | the globe start "with home in view", before the `orbit` view overrides it (:556) | `views[].at` of `orbit` | (use the key) | `{"lat": 34, "lon": -52, "alt": 2.1 R}` |
| c10 | c | src\app\Tools\FidelityMap.cpp:196-198 | `{"GULF OF MAINE  8 DEG", 42.9, -69.6, 8.0}`, `{"THE ESTUARY  0.8 DEG", 42.78, -70.83, 0.8}`, `{"THE INLET  0.08 DEG", 42.8165, -70.8125, 0.08}` | the fidelity-map tool's three frames | none | `tools[]` entry: `frames` | the three |
| c11 | c | src\app\Tools\WaterMap.cpp:44, 48 | `const double lon0 = -71.15, lon1 = -70.30, lat0 = 42.18, lat1 = 43.02;`; `LambertConformalConic lcc{42.35 * kD2R, 42.90 * kD2R, 42.0 * kD2R, -70.725 * kD2R,` | the water-map and bathy-map tool's sheet and its projection | none | `tools[]` entry: `box` | the box and cone |
| c12 | c | src\app\Tools\DumpWaterState.cpp:58 | `weather.Query(42.816, -70.79, simUnix, 500.0);` | the water-state dump's weather sample | none | `tools[]` entry: `at` | `{42.816, -70.79}` |
| c13 | c | src\app\Tools\Export.cpp:121 | `std::cos(42.8 * 3.14159265 / 180.0)` | the .obj export's metres a pixel | none | `place.anchor` (c1) | `42.8` |

### (d) Geography written as code: 11 rows

| # | class | file:line | literal or assumption | who reads it | key today | proposed key | default |
|---|---|---|---|---|---|---|---|
| d1 | d | shaders\Swe.hlsl:102-104, 299; src\sim\SweSolver.h:46; src\app\FrameLoop.cpp:1050 | `return smoothstep(gSpongeX0, gSpongeX0 + 700.0f, worldX);`; `if (h < 0.05f \|\| SpongeAt(t) > 0.5f) {`; `float spongeX0 = 1400.0f;   // world-x where the offshore sponge ramps in (open sea east)` | the solver's open sea is the half-plane of world x beyond a threshold: the sea lies east of every window. The ramp is 700 m. Past half the ramp the render is handed to the stateless tide | none | `water.swe.spongeX0` for the number; the direction has no key (section 4) | `1400` (Boston `-2500`) |
| d2 | d | shaders\Swe.hlsl:223, 273; src\sim\SweSolver.cpp:224-226 | `if (t.x < (int)gRiverBox.x && BedAt(t) < 2.0f) {`; `const float e = elev[static_cast<size_t>(y) * nx];` `if (e > -9000.0f && e < 2.0f) m_westBed.push_back(e);` | the river boundary is the grid's first column, the west edge, wet-capable below +2.0 m NAVD88; its Flather transport is positive east (Swe.hlsl:27-28, 231) | `water.swe.westBoundary` (on or off only) | none (section 4) | west, +2.0 m |
| d3 | d | src\app\FrameLoop.cpp:969-989; src\sim\TideModel.cpp:58 | `kWestKm = bathy.Ready() ? std::min(20.0, std::abs(bathy.WorldX0()) / 1000.0) : 6.0;`; `if (km < bestEnt) { bestEnt = km; entSta = static_cast<int>(i); }`; `s.riverKm = js.Num("river_km", -1);` | the boundary clocks: the ocean clock is the station with the least `river_km` (8440452 at km 0), the west edge's clock the two stations that bracket the window's west edge in km -> `oceanAt`, `westAt` -> the solver, the weather manager's window, the wave field. Assumes the anchor at the mouth, the river running west along x, and straight-line km equal to channel km | none | none (section 4) | the rule |
| d4 | d | src\app\FrameLoop.h:310 | `const double m_kUpriverAreaM2 = 3.9e6;` | `westQAt = riverQ - kUpriverAreaM2 * dh` (FrameLoop.cpp:1014-1019): the Merrimack's reach from the window's west edge to the head of tide at Haverhill | none | `water.swe.upriverAreaM2` | `3.9e6` |
| d5 | d | src\app\Assembly.cpp:78, 86, 99, 105 | `// else Lowell scaled up ~8% for the intervening drainage (Shawsheen, Spicket, Little)`; `return 70.0;`; `double q = 70.0;`; `q = lowell * 1.08;` | the river discharge chain: Lawrence, else Lowell times 1.08, else a summer 70 m3/s -> riverQ | none | `water.swe.riverGaugeScale`, `water.swe.riverQClimate` | `1.08`, `70` |
| d6 | d | shaders\Jet.hlsli:7, 11-14; src\scene\SeaLayer.cpp:494, 508-509 | `// Analytic ebb/flood jet: channel axis through the origin along the ebb direction`; `smoothstep(-4000.0f, -2500.0f, along)`; `double floodDeg = 285.0, ebbDeg = 105.0;`; `m_seaCb.jet[1] = 380.0f;` `m_seaCb.jet[2] = 1600.0f;` | the analytic entrance jet in Sea.hlsl:141-161 and the churn: the waves' current blocking and the foam. The jet stands at the world origin, which is the anchor, which is the ACT0816 station (BathyModel.h:4-7); half-width 380 m, seaward decay 1600 m, upstream plateau from 2.5 to 4 km | none | partial: `sea.jet` {halfWidthM, decayM}; the origin has no key (section 4) | `380`, `1600`, `285`, `105` |
| d7 | d | src\app\FrameLoop.cpp:994-998; src\sim\SweSolver.cpp:233-245; shaders\Swe.hlsl:280; src\app\FrameLoop.h:307 | `return hasSound ? model.Height(entSta, t - 600.0) - model.Height(entSta, t) : 0.0;`; `const bool hasSound = false;`; `if (t.y >= (int)gRiverBox.z && BedAt(t) < -2.0f) {` | Plum Island Sound's south strip: the entrance clock 10 min late, entered on the grid's last rows. Retired (off) but in the code | none | none | off |
| d8 | d | src\app\Tools\SweCycle.h:47-54 | `Log("[swe-cycle] river probe (-5000, %.0f), bed %.1f m", ...)`; `// Throat probe: the deepest channel cell within 150 m of the ACT0816 station`; `for (float wz = -150; wz <= 150; wz += 10) {` | the swe-cycle tool's probes: the river 5 km west of the origin, the throat within 150 m of the origin, taken as the current station | none | `tools[]` entry | as today |
| d9 | d | src\scene\SceneSchema.cpp:183-184, 361, 448, 470, 484 | `"window origin, world x"`; `"the eye: {x, alt, z, az, pitch} ..."`; `"the box's centre and heading in its place's flat frame"`; `"a fixed place in the flat world frame: {x, alt, z}"`; `"the spawn, in the flat world frame (--campos)"` | every {x, z} a scene writes (wave-field window, views, gates, interests, entities) is metres east and north of the anchor c1, measured with its frozen metres a degree. Float precision is best at the anchor (VesselLayer.h:82) | the positions are keys; their origin is not | `place.anchor` (c1) | c1 |
| d10 | d | src\sim\BathyModel.cpp:60-64; src\sim\SweSolver.cpp:187-189; src\compose\DomainSource.h:496-498; src\app\Assembly.cpp:1125-1126 | `m_worldX0 = static_cast<float>((m_lon0 - kOrgLon) * kMPerLon);`; `// dlon*mPerLon = 10.08 east vs dlat*mPerLat = 13.65 north`; `// metres-per-degree frozen at 42.8N is wrong by 40% at the equator`; `// ... the anchor-linear form, which is declared valid only near its anchor.` | any bed is placed in world metres as degrees times the anchor's metres a degree, and the solver's cells take their size from that. At 25.9 N a degree east is 81,660 world metres against about 100,100 true (0.82); at 36.9 N 0.92 | none | none (section 4) | c1 |
| d11 | d | src\scene\TideLayer.h:4, 28; src\scene\TideLayer.cpp:14-16, 51, 103, 108 | `//    1. THE RIBBON: the lower Merrimack as a strip, x = along-channel distance`; `kKmToSceneM = 100.0f;`; `// entrance: cyan`, `// Newburyport: orange`, `// Salisbury Point: violet`; `if (s.riverKm >= 0 && ...)` | the chart mode's ribbon: stations placed along one river's profile by `river_km`; the chart view's camera is "computed from the river's length" (merrimack.json:101). A tide file without `river_km` gives an empty ribbon (**inferred**) | none | none (section 4) | the ribbon |

### (e) Windows and lattices built for one place: 5 rows

| # | class | file:line | literal or assumption | who reads it | key today | proposed key | default |
|---|---|---|---|---|---|---|---|
| e1 | e | src\compose\SurfaceFrame.cpp:16, 31, 33; src\app\Assembly.cpp:669; src\main.cpp:107 | `SurfaceFrame SurfaceFrame::Merrimack(double planetR, bool stencil)`; `s.win = Lattice::Window(1263360, 1538048, 14);`; `s.winH = Lattice::Window(1263360, 1538048, 14, 256, 128);` | the z14 window, tile (4935, 6008), 16384 texels a side, colour and height: the page tenants and their trees (cache tags from Lattice::Tag, **inferred**), the kernels' merc row (SurfaceFrame.cpp:189), the solvers' and the sea's bed (e3), the warm and pack-tiles tools, Export's `earth.*.window` (Export.cpp:46-55) | none | `place.windows[]` {zoom, orgPxX, orgPxY} | `{14, 1263360, 1538048}` |
| e2 | e | src\compose\SurfaceFrame.cpp:37-49 | `const double lonC = -70.8125, latC = 42.8160 * piD / 180.0;` then `std::floor(mx - 8192.0)` | the z17 detail window, 16384 px centred there: the colour tenant's detail slice and the kernels' det row (SurfaceFrame.cpp:210-215) | none | `place.windows[]` (e1), or {zoom 17, centre} | `{17, 42.8160, -70.8125}` |
| e3 | e | src\app\Assembly.cpp:447-451, 828-830; src\app\FrameLoop.cpp:1053-1057; src\app\Tools\WarmInlet.cpp:49 | `// slice 6 = the z14 survey page) is the only bed on the GPU; the solver, the sea` `// shader, the water bank and the globe all read it.`; `sea->SetHeightPage(..., 6u, ..., surface.winH);`; `// M9ar: Boston lies inside the z14 height page; its solver reads slice 6 too.` | every solver reads its bed from the one z14 page, so every solver window must lie inside the one z14 window; slices 6 and 7 are that page and the z17 page. A grep for `6u`, `7u`, `slice 6`, `slice 7` finds 63 hits in 24 files; not each is this | none | none (section 4) | slice 6, slice 7 |
| e4 | e | src\app\Tools\Export.cpp:50-52 | `fn = comp.WindowColor(colCh, 40699567, 49405858, 16384, 19);` | a z19 export-only window "centred on the MassGIS ortho coverage": the export tool's `earth.color.inlet` | none | `tools[]` entry: `window` | `{19, 40699567, 49405858}` |
| e5 | e | src\app\Tools\WarmInlet.cpp:35-36 | `{0.00f, 1.00f, 3}, {0.00f, 1.00f, 2}, {0.38f, 0.62f, 1}, {0.44f, 0.56f, 0}};` | the warm tool's rings "tighten on the inlet": the inlet taken as the z14 page's centre | none | `tools[]` entry: `rings` | the four rings |

### (f) Defaults that are this place's: 10 rows

| # | class | file:line | literal or assumption | who reads it | key today | proposed key | default |
|---|---|---|---|---|---|---|---|
| f1 | f | src\scene\SceneSchema.h:79; docs\scene_schema.json:54 | `std::string name = "merrimack";` | the scene's label | `scene.name` | (use the key) | `"merrimack"` |
| f2 | f | src\scene\SceneSchema.h:91-94; src\app\Options.h:54-56, 150; src\app\Options.cpp:93-95, 396 | `tides = "data/tides/stations.json"; seastate = "data/sea/seastate.json"; currents = "data/currents/currents.json"; bathy = "data/bathy/merrimack.json";` and the flags' `next(...)` defaults | the four data files at boot (Assembly.cpp:270, 316, 336, 359, 431). The first three are unprefixed paths that hold this place's data; the new places' files sit under `<kind>/<place>/` | `data.tides`, `data.seastate`, `data.currents`, `data.bathy` | (use the keys) | the four paths |
| f3 | f | src\scene\SceneSchema.h:120; src\app\Options.h:151 | `float mllwToNavd = -1.30f;`; `float datumOff = -1.30f;` | the MLLW -> NAVD88 link when `sea.datum.fromStation` is false: the Newburyport estimate | `sea.datum.mllwToNavd` | (use the key) | `-1.30` |
| f4 | f | src\scene\SceneSchema.h:139-141; src\core\SceneConfig.h:35-37, 134-135 | `double orgX = -1400.0, orgZ = -800.0;` `int nx = 1600, ny = 1000;` `double cellM = 2.0;` and the same in the text written to data\wave_scene.json | the solved wave field's window around the entrance, world metres from the anchor (FrameLoop.cpp:1078-1105) | `water.wavefield.orgX`, `.orgZ`, `.nx`, `.ny`, `.cellM` | (use the keys) | as quoted |
| f5 | f | src\scene\SceneSchema.h:143; src\core\SceneConfig.h:43, 136; src\sim\WaveField.h:57 | `barNormalDeg = 285.0` "the entrance bar's normal, compass" | the wave field's directions measured from the bar (WaveField.cpp:455) | `water.wavefield.barNormalDeg` | (use the key) | `285` |
| f6 | f | src\scene\SceneSchema.h:241; src\app\Options.h:201-202; src\app\Options.cpp:126 | `double lat = 42.81826, lon = -70.80045;`; `next("42.81826,-70.80045,16")` | the Droste portal's leaf, "the entrance mouth, east of the jetty tips" | `portals[].lat`, `portals[].lon` | (use the keys) | `42.81826`, `-70.80045` |
| f7 | f | src\app\Options.h:49; src\app\Options.cpp:146-148 | `double traceLat = 42.816, traceLon = -70.81;`; `next("42.816,-70.81")` | the --trace tool's point | none | `tools[]` entry: `at` | `{42.816, -70.81}` |
| f8 | f | src\app\Options.cpp:413 | `o.oceanProbe = next("42.35,-70.65");` | the ocean-probe tool's point (near buoy 44013) | none | `tools[]` entry: `at` | `{42.35, -70.65}` |
| f9 | f | src\app\Options.cpp:564-566 | `: (world \|\| o.gulfStart)   ? "scenes/merrimack.json"` `: "scenes/chart.json";` | the scene a launch with flags and no scene file reads | none: it chooses the scene | none | `scenes/merrimack.json`, `scenes/chart.json` |
| f10 | f | src\render\Renderer.h:196-198 | `// Turbid coastal water (vqview's calibrated Merrimack optics, kept as the default palette).` `float sigmaW[3] = {0.330f, 0.1238f, 0.1463f};` `float bscat[3] = {0.0173f, 0.0233f, 0.0248f};` | the water's absorption and backscatter; the consumer was not traced (**inferred**), nor whether `water.closures.waterOptics` replaces it | none | `water.optics.sigmaW`, `water.optics.bscat` | as quoted |

Rows: (a) 23, (b) 7, (c) 13, (d) 11, (e) 5, (f) 10; 69 in all.

---

## 2. What a scene can already say

The schema's sections that name data or a place (SceneSchema.cpp), what reads each, and who
uses it:

- `data.tides` -> TideModel::Load (Assembly.cpp:270; the run stops without it) -> the focus
  (b1), the datum, the water atlas's station field, the boundary clocks.
- `data.seastate` -> SeaState::Load (Assembly.cpp:316) -> the sea's partitions and buoys, the
  weather manager's point.
- `data.currents` -> CurrentModel::Load (Assembly.cpp:336) -> the current stations (the jet),
  the ADCP, the field (the gulf map, the weather manager's current). Not the velocity-gradient
  bank, which reopens the default file (a2).
- `data.bathy` -> the CUDEM layer (Assembly.cpp:359) and the solver's grid, the terrain and the
  sea's bed window (Assembly.cpp:431). Not the depth tree, which reopens the default file (a1).
- `data.shaders` -> the shader folder; not a place (reader not opened).
- `sea.datum.fromStation`, `sea.datum.mllwToNavd` -> datumOff (Assembly.cpp:343).
- `sea.storm.*` -> a sandbox sea state; not a place.
- `water.swe.enabled`, `.westBoundary`, `.spinupH`, `.gain`, `.riverQ` -> the solver
  (Assembly.cpp:470, 544; FrameLoop.cpp:992). `gain` overwrites the `3.2f` member defaults
  (Assembly.cpp:324).
- `water.wavefield.*` -> the solved wave field's window in world metres (FrameLoop.cpp:1078-1105).
- `water.closures.jettyCrestNavd` -> the edit-land floor (WaterComponent.cpp:299).
- `water.fleet.enabled`, `.boats` -> the AIS boats; their lane is a file in code (a17).
- `views[].at` -> {x, alt, z, az, pitch} in the flat frame, or {lat, lon, alt, lookAt} on the
  planet (FrameLoop.cpp:538-556).
- `portals[].lat`, `.lon`, `.toLat`, `.toLon` -> the Droste leaf and its destination.
- `gates[].fromLat`, `.fromLon`, `.at`, `.size`, `.toLat`, `.toLon`, `.toAt`, `.toAz` -> the
  cuboid gates, which carry bodies between two places, each with a chart built at its own place
  (Gateway.cpp:39-40). Today the only keys that set anything at a second place by latitude and
  longitude.
- `interests[].at`, `entities[].at` -> the flat frame (d9).
- `rails.active`, `rails.keys[].at` -> the rails; the shipped rail files look at the anchor
  (scenes\rails\classic.json:21, droste.json:32, 43, 54), and the rail names `jetty` and `flood`
  are this place's features (FrameLoop.cpp:858).
- `streaming.treeRoot` -> the folder of the tile trees; not a place.
- `scene.name`, `scene.mode`, `scene.planet`, `scene.view`, `base`, `include[]` -> merrimack.json
  includes data\wave_scene.json at `water` (merrimack.json:13-16), so that file's wave-field
  window overlays every scene based on merrimack.json.

The shipped recipes: scenes\merrimack.json says every key. chart.json has merrimack.json as its
base. mars.json and each recipe in scenes\recipes (bird, droste, globe, helm, helm_boat,
helm_ebb, key7km, selftest, storm_rail) writes the whole `data` block with the Merrimack's paths
and `sea.datum.mllwToNavd: -1.3`. key7km.json's view stands at lat 42.74, lon -70.87;
droste.json's portal at lat 42.81826 (line 221). scenes\demos\haulover_portal.json (base
merrimack.json) reaches Haulover only through `gates[]` (toLat 25.8997, toLon -80.1239,
lines 44-49); haulover_dusk.json has haulover_portal.json as its base. No recipe points a `data`
key at a second place's files.

---

## 3. What the second place brings

From data\places\haulover.json and data\places\chesapeake.json, and the files they list
(data\tides\<place>\stations.json, data\currents\<place>\currents.json,
data\sea\<place>\seastate.json, opened). The manifests' `anchor` and `sea_point` are
[lon, lat].

| key | Haulover | Chesapeake |
|---|---|---|
| `place.anchor` (c1) | `anchor`: [-80.1225, 25.903] | `anchor`: [-76.01, 36.93] |
| `place.mPerLat`, `place.mPerLon` | nothing: not in the manifest | nothing |
| `place.name` (a22) | `place`: "haulover" | `place`: "chesapeake" |
| `data.bathy` (exists) | data/bathy/haulover.json (1538 x 1619, lon0 -80.22, lat1 26.02) | data/bathy/chesapeake_mouth.json (6075 x 3645, lon0 -76.5, lat1 37.25; the mouth only) |
| `data.bathyName` (a5) | nothing: the manifest gives a `kind`, not a layer name | nothing |
| `data.tides` (exists) | data/tides/haulover/stations.json, 14 stations | data/tides/chesapeake/stations.json, 62 stations |
| `data.tideFocus` (b1) | 8723080, Haulover Pier: first in the file, NAVD-linked (mllw_minus_navd_m -0.687) | 8638863, Chesapeake Bay Bridge Tunnel: first in the file, no NAVD link |
| `data.tidesName` (a9) | only the word `place` | only the word `place` |
| `sea.datum.*` (exists) | the focus publishes its link, so `fromStation` resolves it | the focus has none; the fallback takes the NAVD-linked station with the smallest offset |
| `data.currents` (exists) | data/currents/haulover/currents.json | data/currents/chesapeake/currents.json |
| `data.currentStation` (b2) | ACT8141, "Bakers Haulover Cut" (flood 270, ebb 90) | not said: 8 stations off Cape Henry, first ACT4521 (flood 298, ebb 113) |
| ADCP id (b5, data) and `data.adcpAt` (c7) | id 41122 in `buoy_adcp`; its place is not in the file | none: skipped, "no NDBC station within 60 km of the anchor publishes realtime2 .adcp now" |
| current field (a19, data) and `data.currentsName` (a20) | none: `field` is null | `field.file` cbofs_uv.f32 (360 x 560 at 0.005 deg); `field.source` names CBOFS |
| `data.seastate` (exists) | data/sea/haulover/seastate.json | data/sea/chesapeake/seastate.json |
| `sea.buoy` (b4) | 41122 or 41114 (the file's two buoys; neither is marked for assimilation) | 44099 or 44087 (the same) |
| `sea.pointBox` (c4) | seastate.json `box`: lon 279.25 to 280.25 (-80.75 to -79.75), lat 25.5 to 26.25 | `box`: lon 283.75 to 284.75 (-76.25 to -75.25), lat 36.5 to 37.25. The Merrimack's seastate.json has no `box` |
| `water.swe.spongeX0` (d1) | nothing directly; `sea_point` [-80.03, 25.9] lies east of the anchor, and `box_why` names the shelf edge at -80.03 | nothing directly; `sea_point` [-75.75, 36.95] lies east of the anchor |
| `place.windows[]` (e1, e2) | nothing given; the `box` (about 19 by 22 km) fits one z14 window (about 141 km across at 25.9 N) | nothing given; the `box` (about 157 by 310 km) does not fit one z14 window (about 123 km across at 37 to 38 N); the mouth grid (about 67 by 50 km) does |

Nothing in either manifest for: `data.insets` (a3, a4), `data.edits` and its crest (a7),
`data.water` (a8, global, the same folder), `data.river`, `water.swe.riverGauges`,
`.riverGaugeScale`, `.riverQClimate`, `.upriverAreaM2` (a10, b6, d4, d5: no river was
harvested), `data.waterScene` (a11, authored), `data.aerial` and `data.aerialZone` (a12, c2: no
orthos taken; the zones 17 and 18 come from HIERARCHY, not the manifests), `data.bedRules` and
its box (a13, c5: the default box, lat 42.15 to 43.10, covers neither), `data.overlay` (a14),
`data.gisMask`, `data.gisStencil`, `data.vectors` (a15, a16: Haulover's shoreline was skipped at
247 MB, the Chesapeake's not asked for), `water.fleet.route` (a17), `data.reliefInset` (a18),
`water.swe.name` (a21), `water.swe.dormant[]` (b3), the tools' stations, points, boxes, frames,
windows and rings (b7, c10 to c13, d8, e4, e5, f7, f8), `water.closures.jettyAt` (c6: no
structure surveyed), the cameras (c8, c9), the wave-field window and bar normal (f4, f5: to be
written in metres from the new anchor; ACT8141's bearings give Haulover's channel axis), the
portal (f6), and the optics (f10).

---

## 4. What cannot be a key as it stands

Each item says what the code assumes and where. No design is proposed.

**The anchor is the physics' origin, not only a number (c1, d9, d10).** The world frame is
degrees times the anchor's frozen metres a degree (BathyModel.cpp:60-64), and the solver takes
its cell size from it (SweSolver.cpp:187-195), so a bed far from the anchor is solved on cells
of the wrong size: 0.82 of true east-west at Haulover, 0.92 at the Chesapeake. The tangent frame
the renderer draws in is built at the anchor (FrameLoop.cpp:568-592), and the kernels' geoA row
is the same chart (SurfaceFrame.cpp:20-27). Every {x, z} of a scene is measured from it. The
anchor is also a current station: BathyModel.h:4-7 anchors the world at ACT0816, the jet's axis
passes through the origin (Jet.hlsli:7-11), and the swe-cycle throat probe searches 150 m
around the origin (SweCycle.h:50-54). The gates already build a chart at another place with the
same law at that latitude (Gateway.cpp:39-40, Entity.cpp:244-245).

**The solver's compass (d1, d2, d7).** The open sea is the half-plane where world x exceeds
`spongeX0` (Swe.hlsl:102-104): east. The river is the grid's first column (Swe.hlsl:223, 273;
SweSolver.cpp:224-226), its transport positive east. A south strip for Plum Island Sound is in
the code and off (SweSolver.cpp:241). "Wet-capable" is a fixed +2.0 m NAVD88 (Swe.hlsl:223).
At Haulover the sea is east (the `sea_point` lies east of the anchor) and the water to the west
is Biscayne Bay, a lagoon with no river discharge. At the Chesapeake the sea is east and the
bay reaches north and west from the mouth.

**The boundary clocks by river kilometre (d3, d4, d5).** The ocean clock is the station with the
least `river_km`; the west clock interpolates two stations bracketing |WorldX0| / 1000 km; the
prism area and the gauge chain are the Merrimack's. Neither new tide file has a `river_km`, and
Json.h:32-34 returns the default -1 for a null, so at both places the ocean clock and both west
stations collapse to the focus (FrameLoop.cpp:969, 984).

**The entrance jet (d6).** A Gaussian jet with its axis through the world origin along the ebb
bearing, with fixed width, decay and plateau (Jet.hlsli:9-17, SeaLayer.cpp:506-514). The
station gives only the speed and the two bearings. With no station of that id the jet is off.

**The chart ribbon (d11).** The chart mode draws one river's profile by `river_km`, with colours
by role (TideLayer.cpp:13-16).

**One z14 window, one z17 window (e1, e2, e3).** The height page at slice 6 is the only bed on
the GPU; the solver, the Boston solver, the sea shader, the water bank and the globe read it
(Assembly.cpp:447-451, FrameLoop.cpp:1053-1057), so every solver grid must lie inside the one
z14 window. The colour's z17 page is the one detail window. The export's z19 window (e4) and the
warm rings (e5) repeat the assumption in tools. The Chesapeake box is larger than a z14 window
at its latitude; its mouth grid and Haulover's box are not.

**The zone of a projected raster (c3).** The zone is chosen from the file's EPSG code with 19 as
the fallback (DomainSource.h:260-263); the NAD83(2011) codes of zones 17 and 18 fall to 19. The
aerial source does not consult the code at all (Sources.cpp:253, row c2).

---

## 5. What was not covered

**Folders not read.** src\hal (another agent is editing it; not opened); src\render beyond
Renderer.h:190-205; the shaders beyond Jet.hlsli (read whole) and grep hits in Swe.hlsl,
Sea.hlsl, Gulf.hlsl, WaterBank.hlsl, Globe.hlsl, GlobeMesh.hlsl, Compose.hlsli, TideRibbon.hlsl;
tools\ and proofs\ (Python; counts only); docs other than HIERARCHY 4.11 and 4.15 and
scene_schema.json (grep). Data files not opened: data\bathy\*.json, data\globe\globe.json,
data\gis\gis.json, data\aerial\aerial.json, data\wave_scene.json, data\bed\bed_rules.json,
data\river\river.json. The cache was not listed.

**Patterns not tried.** Local time zones or clock offsets; four-digit world-metre constants in
shaders other than Swe.hlsl and Jet.hlsli; the window tile numbers (1263360, 10168820,
12344774) outside SurfaceFrame.cpp and Assembly.cpp; state-plane feet (EPSG:2249) in shaders;
magnetic declination; the rail files beyond their lookAt lines; the fields of ToolSchema
(SceneSchema.cpp:553).

**Suspected, not confirmed.**
- The bed classifier's alpha band in NAVD88 (Sources.cpp:440, 476-477: full -9 to 0.4, off -14
  to 1.2; "hand land back to the photos above +1.2 m NAVD", Assembly.cpp:846-848), the solver's
  +2.0 m wet test, and WaterComponent.cpp:301 and 309 (1.8 m, 0.45 m) look tuned to this coast's
  tide range.
- ComposeTree.h:214 says "The Merrimack stations sit within a few..." beside a station
  authority radius; the value was not opened.
- Whether Renderer.h:197-198 is replaced where `water.closures.waterOptics` is on (f10).
- Whether globe.json overrides GlobeModel.cpp:107-112's New England defaults (a18).
- That data\bathy\haulover.json names its own `file` (a6).
- Assembly.cpp:544 reads the river file when `water.swe.riverQ` is 0 too (`> 0`), where the
  schema's words say "< 0".

**Seen and left out as not tied to the place.** data\globe\globe.json itself, data\bed\
seafloor_rules.json (a global relief program), data\earth\*.bin (the Mars sample), the cache
paths, data\views.json and scenes\views\builtin.json, data\swe_cycle.csv (a tool's output), the
general metres-a-degree law at a place's own latitude (Entity.cpp:244-245, Gateway.cpp:39-40,
WaterAtlas.cpp:189, 598, GulfLayer.cpp:136-137), and SeaLayer.h:161 and WaveField.h:283
(`3.2f`, calibrated to this window's prism, overwritten by `water.swe.gain` at Assembly.cpp:324
and FrameLoop.cpp:1116, 1539).

**Self-tests pinned at the anchor (not rows).** Droste.h:349-353; ComposeTest.cpp:360-372 (UTM
19 and the Massachusetts state plane, EPSG:26986, at the anchor; Projections.h:56-66
`MassMainland` is used only there); GaTest.cpp:277-278, 990-991, 1012, 1088, 1125;
SpaceTest.cpp:101, 437, 514; SceneTest.cpp:480-482, 531-541, 679, 1358-1641, 1665, 1818, 1840,
1889, 2008, 2266-2267; WaveChartTest.cpp:129-155; WaterAtlas.cpp:373-379, 475-549 (the water
self-test); TreePruneTest.cpp:889.

**The harvester (Python, outside src; step 8's gate does not cover it).** The single-place
scripts write this place's files at the paths the engine names: harvest_waves.py:49 (the point
42.35, 289.35) and :403 (buoys 44013, 44098); harvest_water.py:49-51 (pinned tide stations,
current stations, buoys); harvest_river.py:26 (the two gauges); harvest_currents.py:49
(ACT0816 first); harvest_refbathy.py:98 (focus 8440452 or 8440466); harvest_bathy.py:17 (the
windows merrimack, capeann, boston); harvest_survey.py:374 and harvest_vectors.py:283-285 (the
`_ne` files). Their `--place` modes and harvest_place.py write `data\<kind>\<place>\`.
