# THE GA AST -- the state diagram's edges (generated every run; do not hand-edit)

Nodes are GA engines; edges carry geometric products. `+v=N` / `+v=S` is the second axis' compass direction; FLIP marks where the sampling code inverts v. The flip rule (frames that disagree need exactly one flip) and the orphan rule are enforced by gatest and at every boot.

| from | field | to | src frame | dst frame | flip | units | range | gain | code anchor |
|---|---|---|---|---|---|---|---|---|---|
| compose.stack | paint cube faces | color.pages | latlon.deg +v=N | cube.face +v=N | - | sRGB bytes | slices 0..5, 16k faces | x1 | TileTree::Provider(ColorFrame::Cube) / ComposeCubeDir (composetest-pinned) |
| compose.stack | paint mercator pages | color.pages | latlon.deg +v=N | mercator.px +v=S | FLIP | sRGB bytes | slice 6 = z14, slice 7 = z17; tile 128^2 | x1 | TileTree::Provider(ColorFrame::Window) (merc inverse per texel) |
| compose.stack | paint cube faces | height.pages | latlon.deg +v=N | cube.face +v=N | - | m NAVD (R16F) | slices 0..5, 16k faces | x1 | TileTree::Provider(ColorFrame::Cube), the height root |
| compose.stack | paint mercator page | height.pages | latlon.deg +v=N | mercator.px +v=S | FLIP | m NAVD (R16F) | slice 6 = z14 at 1263360,1538048; tile 256x128 | x1 | TileTree::Provider(ColorFrame::Window), the height root |
| color.pages | page-sample | globe.ps | mercator.px +v=S | uv01.vS +v=S | - | sRGB | finest containing page, residency-clamped mip | x1 | Compose.hlsli ComposedColorPages (no flip: both vS) |
| color.pages | bed albedo | sea.ps (inactive) | mercator.px +v=S | uv01.vS +v=S | - | sRGB | through the refracted ray; the seafloor relief past the survey | x1 | Sea.hlsl ComposedColor(SeaPlanetDir) |
| color.pages | bed albedo | globe.ps | mercator.px +v=S | uv01.vS +v=S | - | sRGB | at the refracted ray's bed hit | x1 | Globe.hlsl ComposedColor(bedDir) |
| height.pages | height | globe.ps | mercator.px +v=S | uv01.vS +v=S | - | m NAVD | vertex, pixel, refracted cast | x1 | Compose.hlsli ComposedHeightPages |
| google.tiles | fetch | compose.stack | mercator.px +v=S | mercator.px +v=S | - | sRGB bytes | zoom = f(groundResM) | x1 | GoogleColorSource::ZoomFor |
| massgis.ortho | fetch | compose.stack | latlon.deg +v=N | mercator.px +v=S | FLIP | sRGB bytes | EPSG:6348 UTM19N declared | x1 | AerialOrthoSource (TM forward) |
| height.stack | classify | synth.bed | latlon.deg +v=N | latlon.deg +v=N | - | m NAVD -> dry albedo | 3 samples/texel | x1 | BedSynthSource::Sample (M7d cross-channel edge) |
| globe.walk | wants | residency.mgr | uv01.vS +v=S | uv01.vS +v=S | - | mip requests | mips 0..7 + floor 4..7 | x1 | GlobeLayer node walk + M7g mip floor |
| residency.mgr | have-map | globe.ps | resmap.texel +v=S | uv01.vS +v=S | - | finest mip * 16 (R8) | 0..7*16 | x1 | CsHave2D residency clamp |
| world.flat | anchor-linear map | latlon.deg | world.m +v=N | latlon.deg +v=N | - | deg | mPerLon frozen at anchor; shared by ALL water consumers | x1 | BathyModel::kOrgLat/kMPerLat convention |
| height.pages | bed per texel | water.bank | mercator.px +v=S | uv01.vS +v=S | - | m NAVD | z14 slice only (AUDIT_WATER item 5); float merc ~0.25 px ulp (gatest-bounded); residency-clamped mips 2..7 | x1 | WaterBank.hlsl gTA[slice 6] (same formulation as CsWindowUv) |
| ocean.fft | cascade.disp | water.bank | patch.wrap +v=N | atlas.texel +v=N | - | m displacement | +-Hs/2 | x1 | WaterBank.hlsl CsBankFill wrap |
| ocean.fft | cascade.deriv (foam union) | water.bank | patch.wrap +v=N | atlas.texel +v=N | - | jacobian foam 0..1 | 0..1 | x1 | WaterBank.hlsl CsBankFill foam discipline |
| ocean.fft | cascade.deriv | globe.ps | patch.wrap +v=N | atlas.texel +v=N | - | slope | +-0.3 | x1 | Globe.hlsl detail loop |
| ocean.fft | caustic jacobian | globe.ps | patch.wrap +v=N | atlas.texel +v=N | - | J / 1/m lap | gain 0.35..2.6 | x1 | Globe.hlsl M8 caustic block |
| swe.solver | eta | water.bank | raster.row0N +v=S | atlas.texel +v=N | FLIP | m dEta | +-1.5 | x1 | WaterBank.hlsl CsBankFill (1-uv.y) |
| swe.solver | uv | water.bank | raster.row0N +v=S | atlas.texel +v=N | FLIP | m/s | +-2.5 | x1 | WaterBank.hlsl CsBankFill (1-uv.y) |
| swe.solver | shadow | water.bank | raster.row0N +v=S | atlas.texel +v=N | FLIP | 0..1 exposure | 0.12..1 | x1 | WaterBank.hlsl CsBankFill (1-uv.y), floor 0.18 |
| churn.kernel | churn | water.bank | atlas.texel +v=N | atlas.texel +v=N | - | 0..1 aeration (remembered foam, MAX-composited) | 0..1 | x1 | WaterBank.hlsl CsBankFill flat |
| height.pages | bed | churn.kernel | mercator.px +v=S | uv01.vS +v=S | - | m NAVD | z14 slice; -30 m off the page | x1 | SeaChurn.hlsl PageBedAt |
| height.pages | bed | sea.ps (inactive) | mercator.px +v=S | uv01.vS +v=S | - | m NAVD | cube + z14 page | x1 | Sea.hlsl BedAt -> ComposedHeight(SeaPlanetDir) |
| height.pages | bed | swe.solver | mercator.px +v=S | uv01.vS +v=S | - | m NAVD | z14 slice; +100 m wall off the page | x1 | Swe.hlsl BedAt (lattice -> lat/lon -> page uv, residency-clamped) |
| swe.solver | eta | sea.ps (inactive) | raster.row0N +v=S | atlas.texel +v=N | FLIP | m dEta | +-1.5 | x1 | Sea.hlsl SweDEta (1-uv.y) |
| swe.solver | shadow | sea.ps (inactive) | raster.row0N +v=S | atlas.texel +v=N | FLIP | 0..1 exposure | 0.12..1 | x1 | Sea.hlsl SweShadow (uv.x, 1-uv.y) |
| churn.kernel | churn | sea.ps (inactive) | atlas.texel +v=N | atlas.texel +v=N | - | 0..1 aeration | 0..1 | x1.05 | Sea.hlsl cuv flat |
| compose.stack | corners | water.bank | world.m +v=N | world.m +v=N | - | m NAVD level/bed + hsScale | hsScale 0.15..3 | x1 | WaterBankLayer CornerParams (CPU) |
| compose.stack | bed (per cell) | wave.solver | world.m +v=N | atlas.texel +v=N | - | m NAVD | -40..15 | x1 | WaveField.h SolveNow (SampleHeightStack) |
| water.atlas | level bucket | wave.solver | world.m +v=N | world.m +v=N | - | m NAVD | 0.25 m buckets | x1 | WaveField.h BucketKey |
| act.currents | current proxy (fallback) | wave.solver | world.m +v=N | atlas.texel +v=N | - | m/s (conveyance jet, x3.0 closure) | 0..2 | x3 | WaveField.h (ebb toward 105, flood 285) |
| swe.solver | current (solved) | wave.solver | raster.row0N +v=S | atlas.texel +v=N | FLIP | m/s (live SeaLayer gain), 0.05 buckets | +-2.5 | x1 | WaveField.h RefreshSweCurrent (1-v flip) |
| wave.solver | a/k/phase-spinor planes | water.bank | atlas.texel +v=N | atlas.texel +v=N | - | m / rad/m / unit spinor (RGBA8, per-comp aMax kMax) | 17 slices, 2-wide grid | x1 | WaterBank.hlsl WaveSample (no flip) |
| water.bank | disp/param/detail | globe.ps | atlas.texel +v=N | atlas.texel +v=N | - | m / sigma2 / m/s / band gains (g1,dry,g0,g2) | rings 4.8..154 m/texel | x1 | Globe.hlsl BankSample manual bilinear |
| water.bank | disp+level | globe.mesh | atlas.texel +v=N | atlas.texel +v=N | - | m NAVD | +-4 | x1 | GlobeMesh.hlsl BankSample |
| noaa.stations | harmonic fit | water.atlas | latlon.deg +v=N | latlon.deg +v=N | - | phasor re/im per constituent | sub-mm RMS (watertest) | x1 | harvest_tides.py -> StationFieldSource IDW p=2 |
| eot20.grid | phasor grid | water.atlas | latlon.deg +v=N | latlon.deg +v=N | - | phasor re/im | |P| clamp a2>100 (Fundy) | x1 | Eot20Source (epoch-rotated arg sum P conj Q) |
| water.atlas | tide phasors x18 (M8i) | window.field | latlon.deg +v=N | mercator.px +v=S | FLIP | phasor re/im | RG16F tiles | x1 | Compositor::WindowField paint |
| water.atlas | level rotors | weather.mgr | latlon.deg +v=N | latlon.deg +v=N | - | m NAVD | +-3 | x1 | WeatherManager::Query h(t)=msl+Re[P e^iwt] |
| water.atlas | datum envelope (origin planes) | globe.mesh | latlon.deg +v=N | atlas.texel +v=N | - | m NAVD lo/hi | containment + width 2.4..3.6 (watertest 7) | x1 | WaterAtlas::EnvelopeNavd -> gBankE.w edit floor |
| gfswave.grid | hs/tp/dir | weather.mgr | raster.row0N +v=S | latlon.deg +v=N | FLIP | m / s / deg | 0..15 m | x1 | WeatherManager wave grid (lat1-lat row) |
| weather.mgr | corner params feed | compose.stack | latlon.deg +v=N | world.m +v=N | - | level/bed/hs | query rungs | x1 | WeatherManager::Query -> CornerParams |
| gfswave.grid | hs whitening | globe.ps | raster.row0N +v=S | atlas.texel +v=N | FLIP | m | 0..15 | x1 | Globe.hlsl wuv (lat1-lat formula) |
| gfs.wind | wind10 (far sigma2) | globe.ps | raster.row0N +v=S | atlas.texel +v=N | FLIP | m/s | 0..40 | x1 | Globe.hlsl wuv; sigma2 = 0.003+0.00512 U |
| gfs.wind | wind-sea fill (PM, when partitions have none) | ocean.fft | latlon.deg +v=N | latlon.deg +v=N | - | m Hs / s Tp | Hs 0..2 over U10 0..9 | x1 | SeaLayer::SetTime -> SeaState::WindSeaPm (closure windSeaFill) |
| ocean.colour | Kd490 -> Kd(RGB) transfer | globe.ps | raster.row0N +v=S | atlas.texel +v=N | FLIP | 1/m | 0.019..6 (Kdw floor) | x1 | Globe.hlsl SampleWaterOptics (Austin-Petzold; M(490)=1) |
| ocean.colour | chl/SPM -> deep albedo | globe.ps | raster.row0N +v=S | atlas.texel +v=N | FLIP | albedo | 0.001..0.5 | x1 | Globe.hlsl SampleWaterOptics (Gordon two-flux, gain 2.0331) |
| gfs.icec | ice albedo + glint damp | globe.ps | raster.row0N +v=S | atlas.texel +v=N | FLIP | 0..1 | concentration | x1 | Globe.hlsl wuv (wave grid); sigma2 *= 1-0.95c |
| gfs.cloud | density bake | cloud.volume | raster.row0N +v=S | atlas.texel +v=N | FLIP | 0..1 | 3D tiles 320 km col | x1 | GlobeLayer cloud bake (ReliefUv family) |
| cloud.volume | density march | globe.ps | raster.row0N +v=S | atlas.texel +v=N | FLIP | sigma_t | 14 steps + sun tap | x1 | Globe.hlsl ReliefUv (row0 north) |
| mv2.windbank | curl overlay | globe.ps | atlas.texel +v=N | atlas.texel +v=N | - | curl x1e4 | +-2.2 synoptic | x1 | Globe.hlsl wind overlay (V) |
| ocean.fft | chop deriv (pattern) | churn.kernel | patch.wrap +v=N | atlas.texel +v=N | - | jacobian foam | 0..1 | x1 | SeaChurn.hlsl (world - U dt)/patch |
| swe.solver | uv (blocking) | churn.kernel | raster.row0N +v=S | atlas.texel +v=N | FLIP | m/s | +-2.5 | x1 | SeaChurn.hlsl suv flip |
| sea.peakdir | shadow build (LOS march) | swe.solver | world.m +v=N | raster.row0N +v=S | FLIP | 0..1 exposure | 0.12..1; rebuilt on dir/level move | x1 | SeaLayer::BuildShadowMask (CPU) |
| compose.stack | paint survey mask | mask.pages | latlon.deg +v=N | mercator.px +v=S | FLIP | water coverage / edited / surveyed (bytes) | cube + z14 + z17 pages | x1 | TileTree::Provider over gis.landsea (GisMaskSource sweep) |
| mask.pages | classifier + edit override | globe.ps | mercator.px +v=S | uv01.vS +v=S | - | land 0..1, edited 0..1, or no opinion | finest page with an opinion | x1 | Compose.hlsli CsMaskSample / ComposedLandness |
| mask.pages | classifier + edit override | sea.ps (inactive) | mercator.px +v=S | uv01.vS +v=S | - | land bit | ComposedIsLand | x1 | Sea.hlsl ComposedIsLand |
| globe.ps | radiance (accepting state) | frame.out | world.m +v=N | world.m +v=N | - | linear RGB -> tonemap | the render | x1 | Renderer tonemap |
