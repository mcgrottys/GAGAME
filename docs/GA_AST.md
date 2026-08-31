# THE GA AST -- the state diagram's edges (generated every run; do not hand-edit)

Nodes are GA engines; edges carry geometric products. `+v=N` / `+v=S` is the second axis' compass direction; FLIP marks where the sampling code inverts v. The flip rule (frames that disagree need exactly one flip) and the orphan rule are enforced by gatest and at every boot.

| from | field | to | src frame | dst frame | flip | units | range | gain | code anchor |
|---|---|---|---|---|---|---|---|---|---|
| compose.stack | paint | window.z14 | latlon.deg +v=N | mercator.px +v=S | FLIP | sRGB bytes / m NAVD | tile 128^2 | x1 | Compositor::WindowColor/WindowHeight (merc inverse per texel) |
| compose.stack | paint | window.z17 | latlon.deg +v=N | mercator.px +v=S | FLIP | sRGB bytes | tile 128^2 | x1 | Compositor::WindowColor zBase 17 |
| compose.stack | paint | cube.color | latlon.deg +v=N | cube.face +v=N | - | sRGB bytes | 16k faces | x1 | ComposeCubeDir (D3D cube convention, composetest-pinned) |
| window.z14 | window-sample | globe.ps | mercator.px +v=S | uv01.vS +v=S | - | sRGB / m NAVD | uv 0..1 | x1 | Compose.hlsli CsWindowUv (no flip: both vS) |
| window.z17 | detail-sample | globe.ps | mercator.px +v=S | uv01.vS +v=S | - | sRGB | finer-only gate | x1 | Compose.hlsli detail rung (M7f/M7h handoff) |
| google.tiles | fetch | window.z14 | mercator.px +v=S | mercator.px +v=S | - | sRGB bytes | zoom = f(groundResM) | x1 | GoogleColorSource::ZoomFor |
| massgis.ortho | fetch | window.z14 | latlon.deg +v=N | mercator.px +v=S | FLIP | sRGB bytes | EPSG:6348 UTM19N declared | x1 | AerialOrthoSource (TM forward) |
| height.stack | classify | synth.bed | latlon.deg +v=N | latlon.deg +v=N | - | m NAVD -> dry albedo | 3 samples/texel | x1 | BedSynthSource::Sample (M7d cross-channel edge) |
| globe.walk | wants | residency.mgr | uv01.vS +v=S | uv01.vS +v=S | - | mip requests | mips 0..7 + floor 4..7 | x1 | GlobeLayer node walk + M7g mip floor |
| residency.mgr | have-map | globe.ps | resmap.texel +v=S | uv01.vS +v=S | - | finest mip * 16 (R8) | 0..7*16 | x1 | CsHave2D residency clamp |
| world.flat | anchor-linear map | latlon.deg | world.m +v=N | latlon.deg +v=N | - | deg | mPerLon frozen at anchor; shared by ALL water consumers | x1 | BathyModel::kOrgLat/kMPerLat convention |
| height.window | bed per texel | water.bank | mercator.px +v=S | uv01.vS +v=S | - | m NAVD | float merc ~0.25 px ulp (gatest-bounded); residency-clamped mips 2..7 | x1 | WaterBank.hlsl M7q (same formulation as CsWindowUv) |
| ocean.fft | cascade.disp | water.bank | patch.wrap +v=N | atlas.texel +v=N | - | m disp + jacobian foam | +-Hs/2 | x1 | WaterBank.hlsl CsBankFill wrap |
| ocean.fft | cascade.deriv | globe.ps | patch.wrap +v=N | atlas.texel +v=N | - | slope | +-0.3 | x1 | Globe.hlsl detail loop |
| swe.solver | eta | water.bank | raster.row0N +v=S | atlas.texel +v=N | FLIP | m dEta | +-1.5 | x1 | WaterBank.hlsl CsBankFill (1-uv.y) |
| swe.solver | uv | water.bank | raster.row0N +v=S | atlas.texel +v=N | FLIP | m/s | +-2.5 | x1 | WaterBank.hlsl CsBankFill (1-uv.y) |
| swe.solver | shadow | water.bank | raster.row0N +v=S | atlas.texel +v=N | FLIP | 0..1 exposure | 0.12..1 | x1 | WaterBank.hlsl CsBankFill (1-uv.y), floor 0.18 |
| churn.kernel | churn | water.bank | atlas.texel +v=N | atlas.texel +v=N | - | 0..1 aeration | 0..1 | x1.05 | WaterBank.hlsl CsBankFill flat |
| bathy.cudem | bed | churn.kernel | raster.row0N +v=S | atlas.texel +v=N | FLIP | m NAVD | -40..15 | x1 | SeaChurn.hlsl suv flip |
| bathy.cudem | bed | sea.ps (inactive) | raster.row0N +v=S | atlas.texel +v=N | FLIP | m NAVD | -40..15 | x1 | Sea.hlsl:71 (uv.x, 1-uv.y) |
| swe.solver | eta | sea.ps (inactive) | raster.row0N +v=S | atlas.texel +v=N | FLIP | m dEta | +-1.5 | x1 | Sea.hlsl SweDEta (1-uv.y) |
| swe.solver | shadow | sea.ps (inactive) | raster.row0N +v=S | atlas.texel +v=N | FLIP | 0..1 exposure | 0.12..1 | x1 | Sea.hlsl SweShadow (uv.x, 1-uv.y) |
| churn.kernel | churn | sea.ps (inactive) | atlas.texel +v=N | atlas.texel +v=N | - | 0..1 aeration | 0..1 | x1.05 | Sea.hlsl cuv flat |
| compose.stack | corners | water.bank | world.m +v=N | world.m +v=N | - | m NAVD level/bed + hsScale | hsScale 0.15..3 | x1 | WaterBankLayer CornerParams (CPU) |
| water.bank | disp/param/detail | globe.ps | atlas.texel +v=N | atlas.texel +v=N | - | m / sigma2 / m/s / hsScale*expo | rings 4.8..154 m/texel | x1 | Globe.hlsl BankSample manual bilinear |
| water.bank | disp+level | globe.mesh | atlas.texel +v=N | atlas.texel +v=N | - | m NAVD | +-4 | x1 | GlobeMesh.hlsl BankSample |
| noaa.stations | harmonic fit | water.atlas | latlon.deg +v=N | latlon.deg +v=N | - | phasor re/im per constituent | sub-mm RMS (watertest) | x1 | harvest_tides.py -> StationFieldSource IDW p=2 |
| eot20.grid | phasor grid | water.atlas | latlon.deg +v=N | latlon.deg +v=N | - | phasor re/im | |P| clamp a2>100 (Fundy) | x1 | Eot20Source (epoch-rotated arg sum P conj Q) |
| water.atlas | tide phasors M2..O1 | window.field | latlon.deg +v=N | mercator.px +v=S | FLIP | phasor re/im | RG16F tiles | x1 | Compositor::WindowField paint |
| water.atlas | level rotors | weather.mgr | latlon.deg +v=N | latlon.deg +v=N | - | m NAVD | +-3 | x1 | WeatherManager::Query h(t)=msl+Re[P e^iwt] |
| gfswave.grid | hs/tp/dir | weather.mgr | raster.row0N +v=S | latlon.deg +v=N | FLIP | m / s / deg | 0..15 m | x1 | WeatherManager wave grid (lat1-lat row) |
| weather.mgr | corner params feed | compose.stack | latlon.deg +v=N | world.m +v=N | - | level/bed/hs | query rungs | x1 | WeatherManager::Query -> CornerParams |
| gfswave.grid | hs whitening | globe.ps | raster.row0N +v=S | atlas.texel +v=N | FLIP | m | 0..15 | x1 | Globe.hlsl wuv (lat1-lat formula) |
| gfs.wind | wind10 (far sigma2) | globe.ps | raster.row0N +v=S | atlas.texel +v=N | FLIP | m/s | 0..40 | x1 | Globe.hlsl wuv; sigma2 = 0.003+0.00512 U |
| gfs.cloud | density bake | cloud.volume | raster.row0N +v=S | atlas.texel +v=N | FLIP | 0..1 | 3D tiles 320 km col | x1 | GlobeLayer cloud bake (ReliefUv family) |
| cloud.volume | density march | globe.ps | raster.row0N +v=S | atlas.texel +v=N | FLIP | sigma_t | 14 steps + sun tap | x1 | Globe.hlsl ReliefUv (row0 north) |
| mv2.windbank | curl overlay | globe.ps | atlas.texel +v=N | atlas.texel +v=N | - | curl x1e4 | +-2.2 synoptic | x1 | Globe.hlsl wind overlay (V) |
| ocean.fft | chop deriv (pattern) | churn.kernel | patch.wrap +v=N | atlas.texel +v=N | - | jacobian foam | 0..1 | x1 | SeaChurn.hlsl (world - U dt)/patch |
| swe.solver | uv (blocking) | churn.kernel | raster.row0N +v=S | atlas.texel +v=N | FLIP | m/s | +-2.5 | x1 | SeaChurn.hlsl suv flip |
| sea.peakdir | shadow build (LOS march) | swe.solver | world.m +v=N | raster.row0N +v=S | FLIP | 0..1 exposure | 0.12..1; rebuilt on dir/level move | x1 | SeaLayer::BuildShadowMask (CPU) |
| survey.edits | polygons + fine mask | gis.masks | latlon.deg +v=N | mercator.px +v=S | FLIP | mask + edit flag | R8G8 4096^2 x2 (0.56 m fine) | x1 | GisStencil::BuildMasks (M7f fine box) |
| gis.masks | classifier override | globe.ps | mercator.px +v=S | uv01.vS +v=S | - | land/water + edit | hand edits are law | x1 | Compose.hlsli CsEditMask (fine box else coarse) |
| globe.ps | radiance (accepting state) | frame.out | world.m +v=N | world.m +v=N | - | linear RGB -> tonemap | the render | x1 | Renderer tonemap |
