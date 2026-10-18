# CONTENT SCALING 2026-10-17: the Merrimack at every grain the public record holds

Mark's ask: "get some drone imagery of the Merrimack river and higher bathymetry to put onto the data
tree and test the scaling of content we can use." This is what exists, what was fetched, what the tree
did with it, and where the ladder bends. Scenes: `scenes/demos/merrimack_hq.json` (the five rasters)
and `scenes/demos/merrimack_hq_a.json` (the A arm: same instant, same views, no rasters).

## 1. What the public record holds at the Merrimack (searched 2026-10-17)

| grain | what | status |
|---|---|---|
| 10 m | CUDEM 1/9 arc-second (data.bathy, the solver's grid) | shipped |
| 1.5 m | **USACE eHydro Newburyport Harbor channel surveys** (2025-09-08 CS, 2025-10-07 XC) as cell means over NOAA BlueTopo 4 m: `data/bathy/inlethq.f32` and its GeoTIFF `D:\DataCache\GAGAME\refbathy\inlethq_navd88.tif` (harvest `hqbathy`, 2026-08-31) | existed, in no scene until today. The eHydro index (ArcGIS FeatureServer, `channelareaidfk='CENAE_MA_01_NEB'`) holds 13 surveys, the newest the two already composited: **nothing newer to fetch.** |
| 1 m | **2018 USACE NCMP CZMIL topobathy lidar, bare-earth DEM** (NOAA Digital Coast project 8691, `noaa-nos-coastal-lidar-pds`), UTM 19N EPSG:6348, NAVD88 by GeoKey 5703, deflate + floating-point predictor, 512 tiles. 25 of its 1 km tiles around the mouth (19TCH 50-53 E, 38-46 N), 52 MB. | **fetched** to `D:\DataCache\GAGAME\lidar\ncmp2018_ma\`. The lidar sees the bottom where the water was clear: the ebb delta, the bars, the beach; the channel is nodata. |
| 0.5 m | **NOAA IOCM ortho-rectified mosaic of 2011-06-19, flown at MEAN LOWER LOW WATER** (NCEI accession 0103944, CC0; also an MHW set and IR bands), 68 tiles of 2.5 km, RGB 8-bit uncompressed, EPSG:32619 (WGS84 UTM 19N). Nine tiles covering 347.5-355.0 km E, 4737.5-4745.0 km N: Salisbury, the mouth, Newburyport, Plum Island north; 910 MB. | **fetched** to `D:\DataCache\GAGAME\iocm\2011_mllw_rgb\`. The three eastern tiles carry black (0,0,0) fill over the sea beyond the flight (5.7 / 26.8 / 46.8 % of the tile); the file declares no alpha, so the black paints. |
| 0.15 m | MassGIS 2023 orthos (two tiles, `data/aerial`) | shipped. MassGIS 2025 (15 cm statewide, 7.5 cm on Cape Cod, 4-band, JP2) is published and not fetched. |
| 0.05 m | **Drone imagery of the Merrimack: none public.** USGS Woods Hole flew UAS over the Plum Island Estuary / Parker River NWR marsh (5 km south of the mouth) on 2017-11-14, 2018-02-27 and 2019-03-28 (DOIs 10.5066/P9O9NSRK, 10.5066/P9PP35F4; bbox -70.824..-70.805, 42.769..42.779) and released the RAW photos (37.9 GB multispectral; the Ricoh true-colour set through EarthExplorer) with RTK ground control, no orthomosaic and no DEM. OpenAerialMap holds nothing in -70.95..-70.70, 42.70..42.90. The Newburyport Conservation Commission's UAVLook flights of Plum Island are not published. | **the nearest public drone orthomosaic** is USGS Woods Hole's at Town Neck Beach, Sandwich MA (Cape Cod Bay, 120 km south): 2025-06-02 flight, Wingtra, Metashape SfM, **5 cm ortho** (20700 x 17200 RGB JPEG-in-TIFF, 69 MB), a 25 cm lidar DSM (NAVD88), UTM 19N EPSG:6348 (ScienceBase 69f4ac22b66b0122e4360ef0). **Fetched** to `D:\DataCache\GAGAME\uas\tnb_2025\` as the drone-grain content; it is not the Merrimack and the scene's readme says so. |

Making the drone photos of Plum Island into an orthomosaic is a structure-from-motion job (OpenDroneMap
in Docker, hours of CPU on a few hundred frames); it is the follow-up if drone imagery AT the Merrimack
is wanted, and it would land 5 km from the jetties on marsh, not on the inlet.

## 2. How each file went onto the tree

No C++ per dataset: each is a `sources` entry of the scene (HIERARCHY 4.20, the raster-is-a-file path),
the file's GeoKeys supply CRS, grain, unit and datum, the entry supplies what the file does not say.

| entry | kind | declared | why |
|---|---|---|---|
| `noaa.inlethq` | height, one file | `unit m`, `datum NAVD88` | EPSG:4269 geographic, no vertical GeoKey (the harvest wrote none) |
| `usace.ncmp2018` | height, folder of 25 | nothing | GeoKeys 3072 = 6348, 4096 = 5703 ("m NAVD88"), 4099 = 9001 |
| `noaa.iocm2011.mllw` | colour, folder of 9 | `feather 10` | EPSG:32619 is in the engine's UTM table |
| `usgs.tnb2025.ortho` | colour, one file | `feather 2` | the RGBA copy (below) |
| `usgs.tnb2025.dsm` | height, one file | `unit m`, `datum NAVD88` | 3072 = 6348, no vertical GeoKey; the metadata says NAVD88 metres. **Refused in every run of the first evening, unnoticed:** USGS wrote it as a BigTIFF (header version 43) and WIC has no decoder for it ("component not found", 0x88982F50). The scene now lists a classic-TIFF copy with the same floats and tags (52 MB raw), and the loader names a BigTIFF before asking WIC. |

**Three file traps, all measured:**

0. The drone DSM is a BigTIFF. WIC refused it in every run with only an HRESULT; the loader now says
   "BigTIFF" by name (`ImageLoader.cpp`, with a self-test), and the classic copy loads. Nothing in
   3a/3b read the DSM; the probe's levels are the bed's and the lidar's.

1. The drone ortho's flight edge is WHITE (255,255,255) over 47.6 % of its rectangle and the file has no
   alpha, so as published it would paint a white rectangle over Cape Cod Bay. `alpha is fiber`: a copy
   was written with alpha 0 where every channel is >= 250, eroded 3 px for the JPEG halo
   (`2025015FA_TNB_MAP61_SfM_Ortho_5cm_rgba.tif`, ExtraSamples 2, valid 52.4 %). Mark's "pixels ship as
   the provider made them" (priors 3) holds: no pixel's colour changed, only its weight.
2. The first RGBA copy was deflate with ONE ROW PER STRIP (what Pillow writes for a row wider than its
   64 KB strip budget). The engine's ingest hung on it for 8 minutes with no progress line (WIC decodes a
   window of a stripped deflate file strip by strip from the top); rewritten uncompressed (one strip,
   1.42 GB) the same ingest ran at the pyramid's pace. The loader comment names "tiled deflate or
   stripped raw" as the measured forms; stripped deflate is now measured too, and it is a trap worth
   a refusal line in `ImageLoader.cpp` (RowsPerStrip x Compression) -- not added today.

## 3. What the tree did (the scaling numbers)

Ingest at boot paints each source's OWN level on each lattice and folds the levels above it. The colour
pyramid is `cube2097152k` (2^31 texels a face edge: 5.9 mm a texel at a face centre at mip 0), the
heights' boot lattice `cube16k` (one tile a face at the coarsest; the finer levels are painted on demand
by `HeightTree` at the grain each reader asks).

| source | grain | source px | own mip | tiles | levels | ingest | per tile | tree on disk |
|---|---|---|---|---|---|---|---|---|
| noaa.iocm2011.mllw (9 tiled TIFFs, 256 x 256 tiles, raw) | 0.5 m | 9 x 25.2 M = 227 M | 6 | 101,211 | 14 | 96.3 s | 0.95 ms | 6.0 GB |
| usgs.tnb2025.ortho (one raw TIFF, ONE strip of 1.42 GB) | 0.05 m | 356 M (52 % valid) | 3 | 96,864 (46,990 of them zero-byte voids: the transparent half) | 15 | 231.9 s | 2.39 ms | 3.1 GB |
| usgs.tnb2025.ortho on cube16k (the coarse lattice, from the file) | | | 0 | 1 | 1 | **40.7 s** | 40.7 s | |
| noaa.inlethq | 1.5 m | 62.6 M | (cube16k) 0 | whole | | | | on demand: 854 files, 140 MB after the ten runs |
| usace.ncmp2018 | 1 m | 25 x 1.0 M | (cube16k) 0 | whole | | | | on demand: 1,206 files, 167 MB after the ten runs |
| usgs.tnb2025.dsm | 0.25 m | 13.1 M | (cube16k) 0 | whole | | | | on demand |

The two colour pyramids, tiles a level (`bin` = painted or folded, `void` = nothing there), from the
tree folders. The own level holds three quarters of the tiles and the fold above it a fifth; the
geometric series is the whole cost of "the tree makes the others":

| level | 2011 orthos (own mip 6) | drone ortho (own mip 3) |
|---|---|---|
| own | 72,237 + 3,251 void | 37,074 + 35,268 void |
| own + 1 | 18,214 + 895 | 9,444 + 8,796 |
| own + 2 | 4,633 + 262 | 2,438 + 2,200 |
| own + 3 | 1,211 + 33 | 649 + 533 |
| own + 4 | 324 + 7 | 181 + 136 |
| own + 5 .. root | 86, 28, 9, 4, 4, 1, 1, 1, 1 | 52, 17, 7, 4, 2, 2, 1, 1, 1, 1 |

The decoder's share, from the exit line of the run that ingested the drone ortho: **44,347 windows decoded,
3,388 s in the decoder** (summed over the pool's threads; the `ingest` parallel-for ran 99 chunks of 35 s
on the Compute lane). Read with the per-lattice instrument added the same evening (section 3c), the
ingest was DECODER-BOUND for a reason the first reading got wrong: the loader serializes its decoder
(one CopyPixels at a time per file), a window of the one-strip raw file decodes in 11 ms, and the
pool's twelve threads, painting neighbouring tiles, asked for the same window at once and each decoded
it again behind that lock -- 26,052 decodes of a 4,959-window file on the pyramid alone, 5.3 times
over, and 4,210 s of threads waiting on 287 s of real decoding. Fixed in 3c.

Read it this way:

- **The pyramid scales with the source's pixels, not its grain.** 227 M source pixels at 0.5 m and 356 M
  at 0.05 m cost the same order of tiles (101 k, 97 k) and the same kind of time: the tree asks the
  file at the source's own level only, so a 5 cm ortho costs what a 50 cm ortho of the same pixel count
  costs. The ladder's depth is free; its width (pixels) is what is paid, at about 1 ms a tile.
- **The 5 cm ortho lands three levels above the lattice floor** (own mip 3 of cube2097152k: 4.7 cm a
  texel there). Content down to 6 mm a texel has a home on today's lattice without any change.
- **A coarse lattice painted straight from a fine file is the expensive case:** the ONE cube16k tile of
  the drone ortho took 40.7 s, the area mean of a 1.4 GB file read window by window, against 0.95 ms a
  tile for the pyramid. The law says the tree makes the coarse levels by folding; the second lattice
  (`surface.cube` for colour, `cube16k`) is realized by asking the source again instead of folding the
  pyramid's tiles. One decode of the file should serve both lattices; this is the first place to spend
  on ingest.
- **Fixed the same evening (3c):** the coarse lattice is now TAKEN from the pyramid's fold (0.0 s, 0
  windows), and a window is decoded once (6,016 decodes for 4,959 windows); the pyramid ingest went
  305 to 82.5 s. The law's consequence became visible on the way: the fold and the direct paint of the
  one 780 m tile differ on the 7 texels the ortho touches, by up to 105/255 in colour and 59/255 in
  coverage (the fold is fourteen byte-rounded levels of children; the direct paint one double-precision
  area mean). The law names the fold; cube16k now obeys it. Whether a coarse texel of a tiny source should
  be the fold or the mean is Mark's to say; at 780 m a texel it is invisible either way.
- Heights at boot paint only the coarsest lattice ("whole"); their finer levels are made as the solver,
  the exposure and the terrain ask. The run logs below count those on-demand windows: at the mouth and the
  bar they were 14 and 36 windows, 0.2 and 0.4 s -- the on-demand path is cheap where the eye is.
- **The first run after a big ingest is not a measurement.** The jetty still that followed the drone
  ortho's ingest held 3,000 frames and never settled (14,521 tiles pending, 0 reads in flight): the
  residency's loads ran at 111 ms each (8,657 of them) behind the 6.3 GB the ingest had just written, and
  the new bed had re-keyed the solved wave field (`wave.rows` 858 jobs, 145 s). The next view settled
  exact in 989 frames with the decoder idle. The jetty still was retaken after the batch (3a).
- **The shoreline is now the coarsest thing in the picture.** At Town Neck Beach the water's edge is a
  stair of the land/sea mask's texel at the eye window's rung (the mask is OSM rings rasterized per mask
  tile, held at rungs 1-4 here), two decades coarser than the 5 cm ortho under it. The 25 cm DSM and the
  water level are both on the tree; dry = height above the level is the shoreline this content deserves.

### 3c. The ingest fixed, measured (the same evening, this branch)

Three changes, each gated on a fresh scratch tree root (`--tool ingest --set streaming.treeRoot=...`)
with the drone ortho alone (the shipped scene's three sources ride along, whole), one take each, other
sessions building on the same machine:

| | before (b5b9ec7 + the instrument) | after |
|---|---|---|
| pyramid (cube2097152k), 96,864 tiles | 305.4 s; 26,052 windows decoded, 4,209.8 s in the decoder | **82.5 s; 6,016 windows, 179.2 s** |
| cube16k, 1 tile | 58.4 s; 4,959 windows, 54.7 s | **0.0 s; 0 windows (1 taken from the pyramid)** |
| the 96,864 pyramid tiles, before against after | | byte-identical, every one; none only on one side |
| the cube16k tile against the pyramid's mip 17 | | byte-identical |
| `--tool rastertest` | | PASS, 56 checks (two new) |

1. **One decode a window** (`RasterFileSource::Window`): a thread that finds the window it needs being
   decoded by another waits for it instead of decoding it again behind the loader's lock. 4,959 unique
   windows now cost 6,016 decodes (the rest are evictions of a 256-window cache under twelve threads).
2. **A coarse realization takes its equal-grid levels from the finer one** (`TileTree::PartnerMip`,
   `TileTree::Adopt`; `Ingest` runs the finest lattice first): cube16k's mip 0 is the pyramid's mip 17,
   the same face, tile shape and texels a face, so the tile is the pyramid's bytes, not an area mean of
   the whole file. The same rule serves any pair of Cube realizations of one tree; the Mercator window
   lattices have no partner and paint as before.
3. **Thin compressed strips are refused by name** (`ImageLoader.cpp`): a compressed TIFF under 16 rows a
   strip over 1,024 rows, with the measured reason in the message; a self-test writes one (deflate as
   stored zlib blocks, one row a strip) and reads the refusal, and the same pixels in 256-row strips.

**The fold against the direct paint, texel by texel** (the old cube16k tile still in the real cache against
the pyramid's mip 17 it now adopts; RGB and the coverage byte):

| texel | direct paint (area mean of the file) | fold (the pyramid's mip 17) |
|---|---|---|
| (39,45) | 76 85 66, cover 73 | 59 71 50, cover 66 |
| (40,45) | 117 118 104, cover **211** | 109 111 98, cover **152** |
| (41,45) | nothing | 99 105 90, cover 1 |
| (39,46) | 128 125 112, cover 209 | 125 122 109, cover 201 |
| (40,46) | 116 113 97, cover 141 | 106 104 87, cover 144 |
| (41,46) | nothing | 76 81 49, cover 2 |
| (39,47) | nothing | 74 76 63, cover 1 |

The direct paint's coverage is at or above the fold's on every texel both hold (211 against 152 on the
one the ortho mostly fills), and it drops the three slivers. Fourteen byte-rounded folds explain a few
counts, not 59. The conjecture, not yet measured: a cube texel is askew on the ground ("0.6 to 1.3 of
GroundRes and askew", Lattice.h), and the direct paint takes the texel's box to the file as an
axis-aligned rectangle in the file's CRS with a margin for the bend, so ortho area that lies in the
neighbouring texel is counted here too, while the fold sums the children's exact 4.7 cm texels and
conserves area. The test is conservation: the sum over texels of coverage times the texel's ground area
(Lattice::TexelGround) against the ortho's valid area, 0.466 km^2, for each path. If the conjecture
holds, every coarse level painted straight from a fine file over-counts at its edges -- the heights'
cube16k at boot still takes that path (the lidar's and the bed's edges), the colour pyramids no longer
do. Proposed as its own session.

Lines: TileTree.h +33, RasterFileSource.cpp +45/-13 (with the instrument), RasterFileSource.h +9/-2,
ImageLoader.cpp +28, ImageLoader.h +3, the self-test +78/-9. Later the same evening: the heights' pyramid
IS ingested at boot (section 3d; `Assembly.cpp`, on the lattice the Float root's children answer in,
`TileTree::KidFrame`'s 256 x 128), and the window cache is 1,024. The own-mip tiles still visit in the
walk's order (the window guard made the order moot for the decoder).

### 3d. The heights at frame time: Mark's "the lighting on the water is like 2 fps"

Mark ran the HQ scene windowed and saw the water's lighting update in half-second steps while the frame
rate read fine. The first jetty still of 3a had shown the mechanism (loads of 111 ms behind a fresh
ingest); the clean measurement is three cold runs of one scene -- the shipped Merrimack plus the 1.5 m
bed and the 1 m lidar, no extra colour -- at the `mouth` view on an empty tree root, frozen clock,
3,000 frames each, other sessions building on the same machine:

| arm | heights at boot | window cache | windows decoded in the frames | residency loads | mapped of 18,703 wanted at 3,000 frames |
|---|---|---|---|---|---|
| before | cube16k only ("whole") | 256 | 12,053 (131 s in the decoder) | 928, **1,222 ms each**, queued 3.3 s | 965 (never settled) |
| pyramid at boot | + the pyramid, 7,849 + 6,964 + 1,666 tiles in 9 + 6 + 2 s | 256 | 12,971 (229 s) | 934, 1,451 ms each | 967 (never settled) |
| + window cache 1,024 | the same | 1,024 | **1,531 (53 s)** | **21,787, 88 ms each**, queued 0.2 s | 13,827 and climbing 2,400 a 600 frames |

Read it this way:

- **The loads were decoding the source files, not reading tiles.** 934 loads of 1.4 s each is the frame
  loop starved: the GPU's height pages near the eye sit at pyramid mips 6-7, below the bed's own level
  (8) and at the lidar's (7), and for a source asked BELOW its own level `TileTree::LeafTile` paints by
  magnifying the source's own texels -- from the file, a window decode a tap -- so the pyramid's folds
  (which fix the levels ABOVE) changed nothing for them: the second arm equals the first.
- **The window cache held a fifth of the working set.** 256 windows against a 980-window bed and 25
  lidar files decoded the same windows fourteen times a load; at 1,024 the decodes fall 8.5x and a load
  costs 88 ms. The cap is 256 MB a source and only what was touched (`RasterFileSource.cpp`).
- **What remains is the law's third clause, unapplied:** below the own level the tree should magnify the
  KEPT own tile, never the file (HIERARCHY 4.20, "finer = the level above magnified"). Proposed as its
  own session; with it a cold view costs the boot ingest and nothing at frame time. Until then the
  first visit of an area streams heights at about 140 tiles a second and the picture fills over a
  minute or two; the second visit is warm. The heights' pyramid at boot is kept (`Assembly.cpp`): cheap
  (17 s for the three sources here), and it is what makes the coarse levels folds.
- The buildings were missing from the HQ scene because its base has none; the layer and sources of
  `newburyport_buildings.json` are now in it. The 19 MassGIS tiles' ingest is section 3e.

**The third clause, built (the same evening, branch `claude/vigorous-wu-bf14f9`; `TileTree::LeafTile`
and `MagnifyOwn`, HIERARCHY 4.20).** Below a source's own level the tile is now the KEPT own-level
tile magnified -- bilinear between its texel centres in the tree's own format, taps past the tile's
edge read from the neighbouring own tile -- and the file is never opened for it; the tile is not kept,
and `Peek` answers for it from the ancestor's entry. The clause's own instrument is the raster
selftest (`--tool rastertest`, 59 checks): the file's `Sample` is called 311,296 times by the pass and
0 times by an ask two levels under the own level; the bytes equal an independent 3x3-mosaic
magnification exactly; a clamp at the own tile's edge is a plant, caught on 280 bytes; a composite
over the leaf is composed once and served once; a height (the hill) asked two and eight levels under
its own level is the hill's function within 0.58 mm at the crown (bound 1.24 mm) with the file asked
0 times. What moved, measured on the test's 4-texel checker (every cell edge a step, the worst case):
the magnified tile against today's paint from the file, max |d| 36/255, mean 0.68/255.

The fourth arm of the table above, the law, cold on a fresh root, one take (`out/law/cold_after.log`):

| arm | windows decoded in the frames | residency loads | earth.height loads | mapped of 18,703 at 3,000 frames |
|---|---|---|---|---|
| + window cache 1,024 (above) | 0 (1,531 = the boot's) | 21,787, 88 ms each | 2,649; 1,676 painted at 101 ms | 13,827 |
| the law | 0 (1,531 = the boot's) | 23,460, 38 ms each | 2,794; 921 painted at 62.5 ms | 14,598 |

Read it this way, and against the probe (`--tool height-tree-probe` before and after, fresh roots,
`out/law/probe_before.log`, `probe_after.log`):

- **The frames' file decodes were already zero at a 1,024-window cache** (1,531 is exactly the boot
  ingest's 225 + 980 + 326): the second arm's 12,971 - 2,798 = 10,173 frame-time decodes were the
  256-window cache thrashing, and how many of them were the tree's and how many the stack's direct
  readers' (the solver's bed realization reads `SampleHeightStack`, not the tree) was never split. The
  law removes the file from the frame by construction rather than by cache size, and with it the
  per-texel `Sample` loop of a file leaf under its own level (32,768 projections and bilinears a tile).
- **What the law did in the frame, from the height tree's own ledger** (`[height-tree] the tree this
  run`, printed at exit from this change on; `out/law/cold_after2.log`, a second cold take on a fresh
  root): the law fired 3,660 times, every one for ETOPO 15-second under its own mip 16, at 1.0 ms a
  call, beside its 155 own-level paints at 3.7 ms; the bed and the lidar were asked under their own
  level 0 times -- the GPU's height pages at this view sit at or above pyramid mip 8 (1,775 and 927
  tiles read, kept by the boot). So the first diagnosis above (the bed at mips 6-7) was wrong for this
  view, and the file reads the law retired here were ETOPO's.
- **A cold height tile still costs some 60 ms, and that cost is the push fold, not the files.** The
  probe's cold sample at 1.5 m is 142 ms before and 140 ms after. The ledger names it: the stack's three
  sources that are not files -- `noaa.cudem.merrimack` (the harvester grid), `noaa.etopo2022`,
  `survey.edits` -- have no own level, so each is painted per texel at EVERY level asked (2,129 / 1,984 /
  2,045 tiles at 2.2-2.6 ms each) and FOLDED UPWARD on every paint (M9bb: 1,684 / 853 / 22 fold-ups at
  103 / 150 / 171 ms each, the chain to the root rewritten under its stripes), then the six leaves are
  composed (4,076 composites). The cold scratch-root run does not settle in 3,000 frames for the same
  reason the band-aid arm did not: those fold-ups, and the colour composite and the survey mask on an
  empty root (8,921 and 4,604 loads at 76-102 ms), fill the queue. The load times across arms are
  confounded by the machine's other work (the sibling's arm ran beside another session's engine; these
  waited for the GPU to be free), so the per-leaf ledger and the selftest's call counts are the clean
  instruments here, not the load means.
- **The deviation the law names, measured where it can be seen.** Above a source's own level nothing
  changes: the probe's lines at 400 m and coarser, and over the sphere, are identical to the metre. Under
  ETOPO 15-second's own level (pyramid mip 16, a 262 m texel at the place), where it is the only painter
  -- outside the bed's, the lidar's and CUDEM's footprints, about a fifth of the probe's 2,000 points --
  the tree's value is the own tile upsampled instead of the file's bilinear between its 340 x 460 m
  texels: |tree - stack| at the place, p95 0.006 -> 0.123 m at 1.5 m, 0.026 -> 0.128 m at 6 m,
  0.157 -> 0.217 m at 25 m, worst 0.69 -> 1.18 m; the texel-centre equality with the stack, which held
  exactly where a leaf painted the level directly, falls from 386 to 12 of 2,000 at level 9. Half a
  texel of position on a 15-second grid is a metre of height on an offshore slope: sub-texel for the
  source, visible to a probe, invisible to the stills (the Merrimack sea view before and after is
  bit-identical; haulover_city differs on 2 pixels past one LSB, within the run-to-run floor, HUD rows
  ignored). The stills could not have seen it anyway: the shared cache serves the tiles the old paint
  kept before it asks the law. Whether a 15-second source should be read at 1.5 m through its own
  260 m tile or through the file's bilinear is the law's one real trade, and it is the same trade the
  GPU already makes for every parent it magnifies.
- The city runs decode 548 (before) and 437 (after) windows at frame time with their trees already
  whole; those are not the tree's reads (its tiles were kept by the earlier run) but the stack's own
  direct readers (`SampleHeightStack`: the solver's bed realization and its kin) -- conjectured from the
  counts, not traced.
- Lines: the engine +158 / -13 (`TileTree.h` +149 / -9: `Peek` 699-707, the branch in `LeafTile`
  1161-1170 before the held-only check, `Ancestor` + `MagnifyOwn` 1387-1508 after `FoldQuadrant`, the
  `below` counter and the `magnify` lap in `Stats`; `RasterFileSource.h` header; `FrameLoop.cpp` the
  height tree's stats at exit), the tests +239 / -1, this document and HIERARCHY 4.20. Nothing was
  deleted: the paint path stays for the own level, and the three legacy sources still need it at every
  level -- and the push fold with it -- until they, too, declare an own level (a file, or `OwnMip` on
  the grid they already are): the remaining clause, and the cold height tile's remaining 60 ms.

### 3e. "Higher res imagery than that house": the MassGIS 15 cm around the mouth

The blurred house in Mark's third screenshot was Google at its zoom, magnified: the first HQ scene had
dropped the MassGIS orthos (3a's correction), and the shipped scene holds only two 1.5 km tiles of them
at the mouth. MassGIS 2023 is 15 cm statewide (2025 the same, 7.5 cm on Cape Cod only), so the answer
is coverage, not a new source. Fetched the same evening: the 4 x 6 USNG grid 19TCH 480-525 E x 370-445 N
(348-354 km E, 4737-4746 km N: the mouth, Newburyport's east end, Salisbury Beach, the north of Plum
Island), 20 zips present of 24 asked (the rest are sea), decoded by `harvest aerial` (Magick.NET,
lossless JP2 to RGB mip chains, a minute or two a tile) into 19 tiles, 7.1 GB under data/aerial, one
manifest. The tree's identity followed the manifest: a sibling session's boot saw it first and paid the
ingest, 17:58 to 18:11, **384,501 tiles on 16 levels at own mip 5, 18 GB** (9.4 cm a texel, 15 cm content),
and this branch's `--tool ingest` found it whole. The house is now 15 cm wherever those tiles reach;
beyond them it is Google again. The stills of 3a were retaken on the whole scene (B2 rows there).

### 3f. The channel that stood dry: the HQ bed's eHydro cells were sign-flipped

Mark, on the corrected mouth still: "the black in the mouth is colour, probably a botched heightmap";
then "the height is broken too"; then "it's not black, it's the water imagery". All three are one fact.
At a point in the entrance channel (42.8172 N, 70.8050 W) the HQ bed file says **+5.74 m NAVD88** where
CUDEM says -8 and the lidar's nearest valid texels -8.25: the channel stands 6 m above the water, so the
engine draws it as DRY GROUND and paints the ground with the colour tree, which there is the ortho's
photograph of dark water. The jagged edge is the eHydro survey's footprint. The arm with the shipped
CUDEM bed never showed it.

The cause is in both harvesters (`HqBathy.cs`, `harvest_hqbathy.py`): eHydro's XYZ carries DEPTHS in US
survey feet below MLLW, positive down (the entrance channel's thinned file runs 0 to 28 ft, median 11.8,
in a channel dredged to 15), and the overlay read them as heights: `z * usft - 1.400`. Over the channel,
12,198 cells of the composite stood above +0.5 m where CUDEM was below -2 (median +3.10 against -6.11);
with the sign right the same cells land within a metre of CUDEM. The composite's own cross-check could
not see it: it sampled only cells below -1 m ("wet cells"), which excluded exactly the cells the error
had lifted above the water -- the instrument's blind spot was the failure's shape. Now `-z * usft -
1.400`, the check samples the eHydro cells whatever their value, and a median farther than 2.5 m from
CUDEM's bed refuses to write (`HarvestStop`). The composite was regenerated from the cached surveys and
the GeoTIFF the scene reads written anew as `inlethq_navd88_2026-10-17.tif` beside the flipped one
(another session's engine held the old file open, so it could not be replaced in place; the scene points
at the new name). The regenerated check: the eHydro cells against CUDEM,
n = 20,000, median +0.011 m, mean +0.073, p5 -1.15, p95 +1.39 (dredging and shoaling since 2014 inside a
metre either way); the channel point that read +5.74 now reads below -8. The ten stills of 3a and the probe of 3b ran on the flipped bed; their
frame-cost reading stands, their bed does not.

Tried and reverted the same evening: `over: 1` on noaa.inlethq (the survey over the lidar). It put
BlueTopo's 4 m ridge over the lidar's 1 m rocks at the jetties, and the jetty tops came out as flat
faceted slabs in the helm view; the grain order (coarser under finer) is right here, the lidar carries
the structures and the bars and the survey fills the channel where the laser found no bottom.

On the corrected bed, the whole scene (A = the shipped Merrimack at the same instant): the mouth from
500 m settled in 2,263 frames at 21.3 ms GPU (A 20.3), 47.6 % of pixels differ, SSIM 0.978 -- the channel
is water again and the bars read as shading; the jetty view 16.1 ms GPU (A 13.9, the buildings on the
skyline are new), SSIM 0.942.

### 3a. Stills and frame cost, A (no rasters) against B (five rasters), one take each

**A correction, found by Mark's eye the same evening.** A scene's `sources` array REPLACES the base's;
it does not merge by name as `views` and `layers` do. The first merrimack_hq.json listed only its five
rasters, so every B run below ran WITHOUT the shipped MassGIS 15 cm orthos, the highlights overlay and
the ETOPO 15-second height ring, while every A run had them: the B stills' Google-only imagery
(blurred houses) and some of the A/B pixel difference are that, not the five rasters. The scene now
repeats the three shipped entries, the 2011 low-tide orthos are out of it (section 1: under today's
water they painted the photographed water as the bed), and the buildings of newburyport_buildings.json
are in. The table stands as the record of what ran; the frame-cost reading (B at or under A) is not
weakened by it -- B carried less imagery, not more -- and the ingest numbers of section 3 are per
source and untouched.

Frozen clock (`--scale 0 --settle-exact --frames 300`, `time.start` 2026-10-17T15:00:00 in both
scenes), `--gpu-time` on, 1600 x 900, the RTX 5060 Laptop. "held" is how many frames the still waited
for the resident set to equal the want set (EXACT); GPU numbers are per-pass means over the run; "px
differ" is `imgdiff`'s n(|d|>1) and SSIM, A against B. The first jetty B was the run right after the
drone ortho's ingest (above); its retake is the row marked so.

| view | arm | held | exact | frame ms (CPU) | GPU whole | globe.water | waterbank | pool MB | px differ A/B |
|---|---|---|---|---|---|---|---|---|---|
| sea (jetty tip) | A | 646 | yes | 14.85 | 13.93 | 9.16 | 2.24 | 1330 | |
| | B, first run | 3000 cap | **no** (14,521 pending) | 23.69 | 23.61 | 17.74 | 3.20 | 467 | 54.8 %, SSIM 0.913 |
| | B, retake | 789 | yes | 14.93 | 13.73 | 6.35 | 2.27 | 1374 | 54.8 %, SSIM 0.913 (the same picture as the first B: the unsettled tiles were not in view) |
| mouth (500 m) | A | 190 | yes | 22.43 | 20.33 | 16.09 | 1.90 | 1137 | |
| | B | 989 | yes | 20.25 | 19.27 | 15.51 | 2.04 | 1170 | 47.4 %, SSIM 0.938 |
| bar (35 m) | A | 186 | yes | 21.68 | 19.70 | 15.09 | 2.33 | 1297 | |
| | B | 780 | yes | 19.55 | 18.42 | 14.36 | 2.40 | 1349 | 73.0 %, SSIM 0.932 |
| tnb_high (600 m) | A | 396 | yes | 14.71 | 13.94 | 10.92 | 0.65 | 540 | |
| | B | 2617 | yes | 12.68 | 12.21 | 9.15 | 0.39 | 556 | 99.0 %, SSIM 0.831 |
| tnb_low (30 m) | A | 127 | yes | 17.85 | 15.81 | 6.71 | 0.65 | 781 | |
| | B | 855 | yes | 14.91 | 13.86 | 4.90 | 0.29 | 942 | 81.6 %, SSIM 0.748 |

- **The frame does not pay for the grain.** At every settled view B's GPU frame is at or under A's
  (13.7 vs 13.9 at the jetty, 19.3 vs 20.3 at the mouth, 18.4 vs 19.7 at the bar, 12.2 vs 13.9 and 13.9
  vs 15.8 over the drone site), one take each and inside what a floor would allow; the residency pool
  holds 30-160 MB more. The water pixel pass is where B is lighter (6.4 vs 9.2 ms at the jetty): the
  water reads the bed, and a bed with a real channel wall shortens the refracted ray's march where the
  CUDEM ramp made it long. Five centimetre content costs what the tile budget costs, not more.
- **B settles slower.** 989 frames against 190 at the mouth, 780 against 186 at the bar, 2617 against
  396 over the beach: the composite tiles under the eye are composed once from the new source trees
  (the colour tree ledger shows them read, not painted), and the on-demand height levels painted. Second
  runs of the same view inherit them.
- The water pixel pass (`globe.water`) is 75-80 % of the frame wherever water fills the view; it is the
  shader every hardware note in docs/REVIEW_2026-10-17.md section 9 points at.

### 3b. The height tree against its stack (`--tool height-tree-probe` on B)

The compose order the probe printed, bottom to top: ETOPO 2022 (4,892 m) < CUDEM (13.7 m) <
noaa.inlethq (1.5 m) < usace.ncmp2018 (1 m) < survey.edits (hand polygons, last). 2,000 points about
the place +-0.05 deg and 500 over the sphere; |tree - stack| in metres, warm; a sample's cost tree cold /
warm against the stack:

| ground asked | level | median | p95 | worst | tree cold / warm | stack |
|---|---|---|---|---|---|---|
| 1.5 m | 8 | 0.000 | 0.006 | 0.692 | 15.5 ms / 2.2 us | 2.57 ms |
| 6 m | 10 | 0.001 | 0.026 | 0.599 | 2.5 ms / 2.6 us | 2.72 ms |
| 25 m | 12 | 0.006 | 0.160 | 1.449 | 0.20 ms / 2.5 us | 2.89 ms |
| 100 m | 14 | 0.017 | 0.659 | 3.995 | 31 us / 2.3 us | 3.74 ms |
| 400 m | 16 | 0.078 | 1.664 | 17.0 | 3.2 us / 2.7 us | 8.8 ms |
| 1600 m | 18 | 0.427 | 3.038 | 33.0 | 6.2 us / 3.1 us | 42.6 ms |
| 6400 m | 20 | 1.837 | 6.397 | 37.9 | 11 us / 3.1 us | 424 ms |

Texel centres: the tree (what the GPU loads) equals the stack's half float at 330 of 2,000 points at
level 9 (worst 0.47 m), 211 at level 11 (0.25 m), 2 at level 13 (2.09 m): the tree is the FOLD of its
children where the stack is a point sample, as the law says (the same ratio the reader's first gate saw).
Rays: 5,645 from the water, |excess(ray) - excess(march)| median 0.000, 99.5 % within 5 cm, 1.1 us a
ray against 153 us a march. Both plants CAUGHT (the ray walked the other way: 57.6 % within 5 cm; three
levels coarse at the place: p95 0.492 against the law's 0.026 m), so the instrument sees a wrong tree.
The cache: 4,096 tiles held, 5,695 reads in 69.6 s. Against the shipped stack's probe (PR 100: level 9
worst 1.7 cm) the worst points grew to decimetres at the fine levels: the 1 m lidar and the 1.5 m
survey put jetty walls and channel banks where the CUDEM had ramps, and a fold edge there differs from
a point sample by the bank's height. The p95 is unchanged in kind: 6 mm at 1.5 m.

The stack's cost is the other number here: 2.6 ms a sample at 1.5 m and 424 ms a sample at 6.4 km,
because the stack asks every source for an area mean over the ground asked, and at 6.4 km that is the
whole lidar set and the whole bed. The tree answers in 3 us at every level. Every CPU reader is on the
tree already (the 2026-10-14 switch); the stack remains the probe's reference and the painter's source.

## 4. The culmination: what this branch holds from the evening's sessions

Mark's word: one branch, one PR, one demo, not pieces. Each session that the day's findings spawned
committed on its own branch and was folded here; their measurements, in their own commits' words:

| piece | branch folded | what it changed | measured |
|---|---|---|---|
| The water's edge by the finest witness | `claude/wizardly-euler-007f5a` (was PR 103, closed into this) | The survey (gis.landsea rings) and the height tree against the live level both say wet or dry; the finer DATA answers, decided per texel at paint time (`Compositor::FinerHeightAt`, the mask texel's g byte), one mask read for landness, wet share and the edit floor; two dead compose functions deleted. | tnb_high with the DSM 8.25 % px, SSIM 0.972, the stair gone along the beach; tnb_low 2.17 %; the shipped sea view bit-identical on the same data; haulover_city at the noise floor. The ~10 m stair was the mesh stage's dry override interpolated across 6-12 m cells, not the mask's texel (ALGEBRA priors 48). |
| Wave intrinsics in the three contended kernels | `claude/jolly-engelbart-f2c7b9` (7ee49ac) | `WaveActiveCountBits` / `WavePrefixCountBits` in the classify, `WaveActiveBitOr` in the meshlet mask, `WavePrefixSum` for the 64-lane scan (13 barriers to 2). | Exact: the city still identical outside the HUD text, the GPU's per-form counts agree record for record; the passes were 0.02-0.10 ms and the frame cannot see it. |
| The water's radiometry in fp16 | the same branch (11072a2) | `-enable-16bit-types` for the globe's mesh-pipeline pixel stage alone; colours, mixes and Fresnel weights in `half`, every address, height and ray in float. | globe.water 6.676 to 6.604 ms, 1.1 % of the pass, for 1 % of pixels one LSB apart (A/A floor bit-identical). ALGEBRA priors 50: this GPU runs non-tensor fp16 at the fp32 rate, and the pass is marches and reads, not arithmetic. Kept as measured; it is Mark's to keep or drop. |
| Priors 48-50 | the same two branches | The shoreline stair's witness; a frozen clock that starts at "now" is a different instant every run (pin `time.start` for any A/B); fp16 on this part. | |
| The seam between two height surveys: feather at the data's edge | `claude/interesting-morse-eaa9c3` (151cd20, four commits) | A source's weight fades over its `feather` from every texel it lacks -- a height's nodata, a colour's alpha 0, ground inside the set's box no file of the set holds -- not only from its bounding box: one exact Euclidean distance transform a window (Felzenszwalb-Huttenlocher, each axis in its own metres), the factor carried in the summed-area table; feather 0 is the hard edge the reads had; a file lacking no texel is byte for byte what it was, and the identity reaches absence only where a file can be absent, so the MassGIS and IOCM trees keep their names. `--tool rastertest:overlap <scene>` reads the heights against each other where both have data; `--tool height-tree-probe:transect=lat0,lon0,lat1,lon1,n` reads every source's weight and value along a line. The scene: noaa.inlethq `over` 1 (in the turbid channel a laser return can be the water's surface; the 2025 sounding is the better witness; the lidar keeps the beach and the bars the survey never saw) and `feather` 10 on the survey and the lidar both. | rastertest 61 checks: the fine weight is the smoothstep of the texel's distance to the nearest absent texel (worst 2.8e-8 against brute force), the stack steps at most 0.740 m a texel (bound 0.75 = 5 m x 1.5 / 10), the plant (feather 0) steps 5.00 m in one texel. The corrected composite over the lidar, 1,371,290 points: median +0.08 m, sd 0.71, the channel +0.08 / 0.38, the beach +0.09 / 0.92 -- no datum bias, `offset` stays 0. Three transects across the north tip of Plum Island read every source within a metre of each other from the dune (+7.7 m) down the beach face to the bed (-5 m): the wall seen there stood on the sign-flipped survey (3f), not on a data edge. The HQ views change only at the jetties; the shipped sea view at one patch, the north jetty's crest, where the MassGIS set's data ends along easting 352,500. The edge build at a quarter of the windows cost 686 s of CPU and never settled until the edge cache grew to `kWindows` entries and a trivial edge was kept empty. |
| An ortho's pixel counts only where the ground was dry when flown | `claude/ortho-capture-level` (4320d23) | A colour source may declare `captureLevel` metres above `captureDatum` ("" or NAVD88 the engine's frame, MLLW by the place's link, MSL by the focus station's fitted mean; any other name refuses the photo by name). Where declared, its weight at a texel is multiplied by a gate on the ground height from the height tree at the texel's grain: 1 above the level, 0 below, between them a smoothstep across the read's own uncertainty, half the spread of the four texels the tree's bilinear stands between. `Sources.h LevelGateSource`, composed per texel by `TileTree::GateTile`, wired in the Assembly's colour tree as GateSource("<photo><ground"); the colour is never touched, an undeclared source keeps its weight byte for byte, the gate's identity folds every height layer's name so an edited bed repaints the gated tiles and nothing else. The 2011 IOCM low-water orthos are back in the demo scene with MLLW declared. | composetest, planted (a synthetic ortho over a synthetic ramp, half above and half below its 1 m level): weight 1.000 at 1.5 m of ground, 0.000 at 0.5 m, 0.502 at the level; at a 100 m grain the tree reads 1.200 +- 0.763 m and the gate weighs 0.6902 = the law's; the tile at mip 7: 5,120 texels kept, 10,240 zeroed, 1,024 within 5 cm of the level, 0 wrong. Stills (serialized on the GPU): the shipped sea view bit-identical; the HQ mouth with no level declared bit-identical, with MLLW declared sub-LSB, 86 px at d = 1 -- the MassGIS orthos cover the view at 15 cm and stand over the 2011 ortho by grain. On the scene without MassGIS the gate removed one wedge along the south jetty's harbour face (8,326 px, luma 74 to 41) and nothing in the channel or on the shoal: under surveyed water `bed_rules.json` already paints the bed over every photo below +0.4 m NAVD88, so a photo's water can show only where the survey's rings say land and the height says below the level. The gate's tiles hold 3.77 M texels at weight 0 under the water. Open: stations.json carries no MHW, so NOAA's MHW set of the same flight cannot declare it yet; the flat channel's gate is a weight gate while the tree's is a value gate (since 2026-10-10), noted, not touched. |
| Below a source's own level, magnify its kept tile, not its file | `claude/vigorous-wu-bf14f9` (6e80451) | HIERARCHY 4.20's third clause made literal: `TileTree::LeafTile` reads a tile finer than one source's own level from the KEPT own-level tile through `MagnifyOwn` -- bilinear between the ancestor's texel centres in the tree's own format (straight alpha for Rgba8, weight-weighted value for FloatW, the four channels for Raw4), a tap past the ancestor's edge read from the neighbouring own tile as the GPU reads across the atlas -- and the file is never opened for it. Not kept; `Peek` answers from the ancestor's entry so a composite above knows its key; the branch stands before the held-only check since it reads only what is kept. The height tree's per-leaf ledger is printed at exit (`[height-tree] the tree this run`). | rastertest 59 checks: the file's `Sample` called 311,296 times by the pass and 0 times by an ask two levels under; the bytes equal an independent 3x3-mosaic magnification exactly; a planted clamp at the own tile's edge caught on 280 bytes; the hill asked eight levels under its own mip within 0.58 mm at the crown (bound 1.24 mm), the file asked 0 times; on the 4-texel checker what moved by design is max 36/255, mean 0.68/255. Stills: the shipped sea view bit-identical, haulover_city within the run-to-run floor, both settle exact. The cold mouth view of 3d, fourth arm: residency.load 38 ms (the 1,024-window arm 88, the original 1,451), frame-time file decodes 0, still not exact at 3,000 frames; the ledger says why: the law fired 3,660 times, every one for ETOPO 15-second under its own mip 16, and the bed and the lidar were asked under their own level 0 times at this view -- 3d's first diagnosis (the bed at mips 6-7) was wrong there -- and what a cold height tile still costs is the three NON-file sources (the CUDEM grid, ETOPO 2022, survey.edits), which have no own level, painted per texel at every level asked and folded upward at 103-171 ms a fold-up: the clause still to apply to them. The probe: above every own level identical to the metre; under ETOPO's own level, where it alone paints, |tree - stack| p95 0.006 to 0.123 m at 1.5 m -- the law's one trade, visible to the probe, not to the stills. |

**The gate on the merged tree** (one take each; the other sessions' engines shared the GPU throughout):
`--tool rastertest` 57 checks; `--selftest` 29 suites PASS, no unplanted FAIL; the demo scene against the
A arm at the same instant: the mouth settled exact in 840 frames, 20.8 ms GPU, 25.8 % px / SSIM 0.984;
Town Neck Beach from 600 m settled in 1,299 frames, 15.8 ms, the stair along the beach gone and the houses
standing as solids, 54 % px / SSIM 0.875 (A has no ortho there); the jetty view did not settle in 3,000
frames under the contention (7,779 of 32,479 tiles pending), 27.5 ms, 13.9 % px / SSIM 0.951 on what it
had. The jetty view also showed a flat, faceted top on the north jetty -- read, it turned out below, off a
still with 7,779 tiles pending; the settled A arm shows the slab without the facets -- and in the shipped
scene since the MassGIS orthos went in: `data/gis/edits.geojson`'s hand polygons (OSM's jetty ways)
put a 2.5 m NAVD88 crest floor under the whole structure, flat, and the 2023 leaf-off ortho paints its dark
riprap with little contrast, so the eye reads a slab where Google's imagery read rocks. The 2018 lidar has
the structure at 1 m (valid on 98 % of the north jetty's outline, crests to 4.4 m): the hand floor should
yield to a finer height where one has data -- the finest-witness law the shoreline took tonight, applied to
the edits. Not done; it predates this branch.

**How the three were folded** (23:00-23:07): each branch stood on this branch's f68c786 and was merged
in turn, seam, capture level, magnify. The dry run (`git merge-tree --write-tree`) named the conflicts
before any merge: the scene twice (the seam's `over` 1 + `feather` 10 on the two heights against the
capture level's 2011 ortho entry and readme paragraph; both kept, the ortho entry after the lidar's)
and this document once (the magnify session's 3d paragraph and 3e-3f inserted at the same line; theirs
first, since it belongs to 3d). Every source file auto-merged: `RasterFileSource.h` carries all three
sessions' additions, `RasterFileTest.cpp` the seam's and the magnify's cases, `Assembly.cpp` the boot
ingest and the capture gate's wiring. The build is clean (70 of 70). The seam's `feather` and the capture
gate give new identities to the feathered heights, the RGBA drone ortho and the gated colour composite;
the sessions' own gates had already ingested every one into the shared cache, so the merged binary's first
boot found all fourteen trees whole (the MassGIS and IOCM trees keep their names and their tiles).
Nothing is left to fold; the coverage-conservation probe (a card) was not started.

**The gate on the whole** (23:08-23:15, the merged binary of 23:07, no other engine on the GPU, one take each):
`--tool rastertest` 65 checks, 0 FAIL (57 + the seam's and the magnify's cases); `--selftest` 29 suites PASS, no
unplanted FAIL. Every one of the fourteen trees was whole at boot. The three stills, `--scale 0 --settle-exact`:

| view | settled | GPU whole frame / water | vs the A arm | vs the stills before the three folds (F0, 19:00) |
|---|---|---|---|---|
| sea (the north jetty tip) | exact in 1,840 frames (F0: not in 3,000, 7,779 pending, under four engines) | 15.4 / 6.5 ms (F0 27.5 under contention; A 14.9) | 54.5 % px, SSIM 0.908 | 53.4 % px, SSIM 0.916 |
| mouth | exact in 309 frames (F0 840) | 20.4 / 15.9 ms (F0 20.8; A 22.4) | 47.8 % px, SSIM 0.977 | 41.3 % px, SSIM 0.975 |
| tnb_high | exact in 986 frames (F0 1,299) | 15.0 / 11.9 ms (F0 15.8; A 14.7) | 54.0 % px, SSIM 0.876 (A has no ortho there) | 0.8 % px, SSIM 0.999 |

Read against the pictures (sent to Mark; his eye decides):

- **Town Neck did not move** (0.8 % of pixels, the RGBA ortho's 2 m feather now reaching its alpha edge): none
  of the three laws has data to act on there.
- **The mouth moved on the bars and in the channel, not on the land.** The dark irregular patches east of the
  jetty tips in F0 were the 2018 lidar's shoals standing over the composite; with noaa.inlethq `over` 1 the
  2025 composite (BlueTopo 4 m where eHydro's polygons end, resampled to 1.5 m) paints them, smoother and
  moved, and the wave field re-solved on the new bed, so the whole water surface differs by a little
  (mean |d| 1.8). What remains in the after-still is a thin dark line running SSE from the south jetty's tip
  and a dark streak east of it: by their straightness, edges INSIDE the composite file (where an eHydro
  survey polygon meets BlueTopo at a 1.5 m cell), which the feather cannot see, since the file lacks no
  texel there -- a reading from the picture, not a measurement; the transect tool is the instrument.
  Where the 2018 lidar is the finer and the truer witness on a bar the composite's own sources never
  sounded, `over` 1 gives it away: a date and a provenance are still not keys the stack knows (section 5).
- **The jetty's crest reads as rock, and the before-still is not the comparison.** F0's slab was taken with
  7,779 tiles pending; its facets were the coarse heights the residency had. The settled after-still shows
  the north jetty's top as the ortho's riprap, magnified into blobs at 1 m from the eye, over a crest the
  transect below shows flat at +2.5 m. The seam session's own measurement of the shipped sea view names what
  changed on the colour: the MassGIS set's data ends along easting 352,500, across this crest, and the ortho
  now fades over its 25 m where it stepped. On the heights, the question this view raises -- does the seam's
  10 m feather on the lidar fade a structure narrower than two feathers? -- the transect answers:

  The transect across the north jetty (`--tool height-tree-probe:transect=42.818805,-70.8085,42.820005,-70.8085,134`,
  a south-north line at 1 m through the structure's middle; 6 s on the CPU, the stack read directly, no
  ingest): the jetty's polygon spans 49-77 m of the line, 29 m, and across every metre of it the stack is
  +2.500 exactly. `survey.edits 1.000 +2.500` is the last source in the compose order and paints the hand
  polygon's crest AS A VALUE at weight 1, over the lidar reading +2.77 m and the composite +3.03 m on the
  same crest (both witnesses have the structure, and agree within 0.3 m). So the slab is the hand edit
  itself, measured, not a floor the finer data could rise above; it stands in A, F0 and F alike, and the
  settled after-still's rock is the ortho's colour on a flat +2.5 m top. The lidar's feather is moot here:
  its weight ramps 0.027 to 1.000 over 50-61 m from the jetty's south face (the channel, where the laser
  saw no bottom), under the edits' value, and at the north foot (78 m on) the lidar has the bottom at
  weight 1 with no edge to fade from. The stack steps -3.97 to +2.50 in one metre at the polygon's south
  edge, since the edits carry no feather: a riprap face, physical. The feather-0 arm was not needed (and
  its probe copy was refused by the schema for a `readme` key, which the shipped scene spells otherwise).
  Where the feather does act, the Plum Island beach, the seam session's own transects in HIERARCHY 4.20
  are the record. The fix named above now has its number: the hand value should yield where a finer
  height has data (the lidar's 1 m, the composite's 1.5 m), as the shoreline's finest-witness law does
  for the mask, so that the edit is a floor under coarse data only. Done the next morning: below.

**The hand edit yields to a finer witness** (2026-10-18 00:40-01:40, this branch, HIERARCHY 4.20's last
block). `EditsHeightSource::Uncovered`: the edit's weight is the product of (1 - w_j) over the channel's
layers finer than its 5 m, the value untouched; the Assembly names the channel as its witnesses; the
layer's tile identity carries the law (`HeightSource::Law`). Engine +60 / -2 lines, the test +97.

| instrument | before | after |
|---|---|---|
| `--tool rastertest` | 65 checks | 69, 0 FAIL: the disc reads the hill with the edit at weight 0.000 (0.03 mm); past the disc the crest at 1.000; the plant (no witnesses named) 50 m over the hill; the identity carries `yields<hill`; across the feathered rim the edit is the hill's complement to 3e-8, the stack steps at most 5.41 m a metre (bound 5.74) where the hard edge steps 72.5 |
| `--selftest` | 29 suites | 29 suites, no unplanted FAIL |
| transect across the north jetty, 49-77 m | +2.500 on every metre (`survey.edits 1.000 +2.500` over lidar +2.77, composite +3.03) | the composite over the lidar on every metre: -3.07 at the south foot, +2.5 to +3.0 on the crest (51-65 m), the north slope down to -2.4; no edits column |
| transect along the polygon's midline (11 segments) | | crest +2.1 to +3.7 m end to end; the composite paints it at weight 1 everywhere, the lidar under it at 0.55-1.0 |
| shipped merrimack.json, view sea, pinned clock, before (the binary of 23:07, a copy) vs after | | 1.3 % px, max 82, SSIM 0.995, all on the jetty's EDGES (diff_ship.png). Not the law: that scene has no height finer than the edits. The edits tree it read, `survey.edits.42dcc867` (2026-10-03), and tonight's `8204eaa5` (same polygons, no witnesses) share 2,579 tiles, 2,527 byte-identical and 52 different, every one at a polygon's edge: the old tree's mip-8 tiles were FOLDED from mip-7 children a deeper view had painted (edge weights in 1/64 steps, the polygon's area in each texel: f5_m8_x10680_y620 holds weights 0.0156, 0.0625, 0.125 ...), the new tree, asked at mip 8 first, painted that level directly (weight 1 or 0 by the texel's centre). First written up here as stale paint; it is the direct paint's area question (the pending card), the same bytes under two legitimate histories. |
| HQ sea view (hq_sea_after.png vs F_sea of 23:12) | the flat +2.5 m slab | 44 % px, SSIM 0.927: the jetty is the composite's rock surface (rolling +2 to +3 m, the slopes into the water), and the wave field re-solved on the changed bed moves the sea; settled exact in 650 frames, 14.9 ms GPU |

**The spike.** The after-still shows one steep 10 m spike on the jetty about 50 m west of the eye. It is
not in any 1 m transect (four crossings near the tip read at most +2.90 m) because it is three cells of
the composite itself, read straight from `data/bathy/inlethq.f32` inside the North Jetty polygon:
+9.99, +7.08, +6.50 m at -70.80386, 42.81894, the polygon's east tip -- the North Jetty light tower,
standing in BlueTopo's surface, which the hand edit used to flatten with everything else. The polygon
holds 9,274 composite cells, median +1.25 m, p95 +3.03, and that is its maximum. Whether a tower is
ground is the surface-against-bare-earth question of the composite's source (the 2018 lidar, bare
earth, reads +2.1 m there), not the law's; a photo would show the tower. Mark's eye decides whether
the demo wants the tower or the lidar on top at the jetties (`over` 0 on noaa.inlethq would give the
bare earth where both have data).

**A grid layer's tiles are keyed by its bytes** (2026-10-18 02:00-02:40, this branch). `noaa.etopo2022` and
`noaa.cudem.<survey>` answer `HeightSource::Law` with the grid's FNV-64, size, datum link, box and
feather (compose/Sources.cpp), so a regenerated grid or another station's link repaints their leaves and
`earth.height`; before, both were keyed by name|unit alone. Engine +44 lines, the test +20. The paint's
form stays `kTileTreeVersion`'s. Gate: `--selftest` composetest plants two equal grids (one key), the
-0.092 m link on one (a new key) and one changed cell (a new key); `--tool rastertest` unchanged. The
cost, the shipped sea view's first boot with every grid leaf and the composite cold against the 01:17
boot (the edits leaf and the composite cold, the grids warm): the shipped sea view's first boot settled exact in 1,792 frames (174 s) against 515 (97 s) with the grids warm; the height tree read 1,053 tiles in 144.9 s against 26.0 s, residency.load 17.8 ms a job against 5.9, the GPU frame 13.4 ms, unchanged; ETOPO 2022 painted 2,720 tiles, CUDEM 2,320 (1,226 void), the composite composed 4,517. The still against the 01:17 one: 0.28 % of pixels, max 53, SSIM 0.9998, specks on the water. **And the repaint from scratch is not the bytes the long-lived trees held.** Against the trees of 2026-10-03, at texels both weigh fully (weights agree to 0.002 everywhere): CUDEM differs by up to 0.13 m at pyramid mip 8, 0.29 at mip 10, 1.08 at mip 11 (a million texels past 1 cm); ETOPO 2022 by up to 0.02 m at pyramid mips 8-13 -- and by 42, 183, 474, 1,022, 1,583 and 2,658 m at cube16k's mips 1 to 6, with mip 0 identical in all 101 tiles. The reading: a source without an own level paints whatever level is asked. The old trees' coarse levels were FOLDED from finer levels earlier views had painted (area means); tonight's fresh trees painted them DIRECTLY at the first ask (the boot walks coarse first), and `EquirectHeightSource::Sample` is a point bilinear that ignores the grain, so a 39 km texel painted directly is one point of a mountain range where the fold is its mean. The tree's bytes for the three grid layers are a function of the order of asks: a fresh cache (a new machine, this commit's first boot) and a lived-in one hold different heights, kilometres apart at the globe's coarse rungs. The identity cannot mend that; the own-level clause for these three (the magnify session's remaining clause: paint at the grid's own level, fold above, magnify below) makes the bytes a function of the data alone, and is the next step.

**The three grid layers declare an own level** (2026-10-18 02:00-, this branch). `OwnMip` (and `Footprint`
for CUDEM and the edits) on the three layers that answered at every grain; the tree's existing paths do the
rest. Engine +40 lines, the test +80 (an equirect grid of ETOPO's size asked cold two levels above its own
mip: the fold of sixteen own-level paints byte for byte, the direct paint the checker's height away, a
fine-first root identical file for file). The gate (02:20-02:40, one take each, one engine at a time): `--tool rastertest` 70 checks, `--selftest` 29
suites. The own level is part of each layer's key (`Law` says "own level"), found necessary on the first
pass: without it the lived-in cache served ETOPO's old point-sampled rungs under the new law. Fresh roots,
before and after: the cold mouth view (the heights-only scene) never settled before (1,575 of 18,703 tiles
pending at 3,000 frames, residency.load 43.9 ms, ETOPO 2022 painted 3,042 tiles, CUDEM 2,867, the edits 2,949,
every one at the level asked) and settles exact after in 1,620 frames (loads 34.5 ms; ETOPO paints its own
level, 8,197 tiles, and magnifies 3,016 below it; CUDEM paints 225 and magnifies 1,731; the edits paint 12);
the picture 0.09 % of pixels apart. The probe (`--tool height-tree-probe`, fresh roots): a cold 1.5 m sample
154 ms before, 3 ms after, the run 451 s to 88 s -- the per-level paint and push fold of the three are gone.
Its |tree - stack| moved where the law says it must and nowhere else: at the place, p95 0.122 to 0.128 m at
1.5 m, 0.126 to 0.135 at 6 m, 0.209 to 0.207 at 25 m, identical at 100 m and above; the worst at 1.5 m from
1.18 to 5.20 m, which is the edits' hard edge become one own texel's ramp (4.8 m at mip 10) against the
stack's hard edge, the crest less the bed; over the sphere p95 0.27 to 13.3 m at 400 m and 7.5 to 13.3 at
1,600 m, which is the stack's doing, not the tree's: above ETOPO 2022's own level the tree now holds area
means and the stack still answers with a point bilinear at every grain (`EquirectHeightSource::Sample`
ignores its grain). The lived-in cache, the three layers' trees fresh under their new keys: the globe's orbit
view settled in 8 frames where it had held one tile short through 3,000, ETOPO painting its 7,171 own-level
tiles once, 0.23 % of pixels apart; the globe's mouth view 1.7 %; the shipped sea view 8.9 % (mean 0.71: the
bed under the water now CUDEM's own level magnified rather than its bilinear at every texel, and the wave
field re-solved on it); the HQ sea view 0.12 %, the drone and survey files owning it. Not done: the stack's
own point sampling above a grid's own level (the probe's sphere numbers), and the edits' ramp width is the
own texel's, not a declared feather.

**"Why does the land look low res?"** (Mark, 2026-10-18 morning, a screenshot of Plum Island's village from
the sea in `scenes/demos/globe_test.json`, view `orbit` flown down by hand: the sand and the ground between
the houses smeared at metres a texel, the roads crisp). The globe scene lists four `sources` of its own --
the buildings, the roads, the two bridge surveys -- and until this morning a scene's `sources` REPLACED its
base's: the globe scene's base (composite_merrimack.json) had already dropped the shipped scene's MassGIS
orthos and highlights for its water composite, and the globe scene dropped the composite and the ETOPO
ring in turn, so its colour tree at the Merrimack was Google alone under the sea-floor gate (`earth.color
= earth.land(google.satellite) + seafloor<gis`), and Google at the Merrimack is held at the zoom the cache
has and magnified past it: at the screenshot's eye 4,967 of 24,511 tiles were served magnified from a
parent. The same trap had taken the first cut of the HQ scene the evening before. Mark's expectation is the
design -- the regional orthos are another shell of the one texture the GPU reads blind, inherited by every
scene at the place -- and the loader already had the law for it: `SceneBuilder::MergeList`, "in place by
name, remove, append", which every named list (`views`, `layers`, `gates`, `entities`) merges by; `sources`,
`stations` and `solvers` were declared unnamed although every entry carries a name, and an unnamed list is a
value an overlay replaces whole (the fleet, the include chain). Now the three are named lists: a derived
scene adds to its base's, an entry of the same name merges in place, `remove: true` drops one. Of the
eleven derived scenes with sources, none removed anything on purpose; eight gain back what the replacement
had silently taken (the globe scene its orthos, highlights, ETOPO ring and water composite; the Haulover
composite scene its own CUDEM survey). Gated (11:37-12:20, one take each): `--selftest` 29 suites (the scene tests among them), `--tool rastertest`
70; `--print-scene` of the globe scene resolves eight sources where it listed four. The screenshot's eye
rebuilt headless (`views.pi`: 42.7985 N -70.7960 E, 260 m, heading 290, tilt 62) in the globe scene: before,
Google alone, exact in 191 frames with 4,967 tiles served magnified from a parent, the land soft; after the
law, the scene inherits its base's base's MassGIS orthos, highlights and ETOPO ring -- and its base's
water-bed composite, five file-backed height rungs it had never actually carried -- and the first boot
mapped 832 of 24,511 tiles in 3,000 frames: 801 residency loads at 3.7 s each, 73,076 windows decoded at
frame time (180 s in the decoder) against 0 before, the loader's 48 slots held by height pages so the colour
pages never got one, the picture unpainted. A second boot on the filled cache was the same (824 loads at
3.5 s, 53,003 decodes) while every leaf of the height tree read in milliseconds and the solver's bed read
its page in 0.3 s: the time is in a reader that samples the STACK through the composite's many small
files at frame time, not in the tree, and which reader was not found tonight (the buildings' stand field
reads the stack by design, but its four page jobs never ran; the height tenant's own paints measured
5.8 ms). The attribution is by experiment: the same eye with `{"name": "water.bed", "remove": true}` and
everything else inherited settles exact in 659 frames at 13.8 ms a load, 209 decodes, the colour tenant
painting its 5,096 MassGIS pages at 34 ms each, the land the orthos. So `globe_test.json` now drops the
composite by name with that reason written beside it, and keeps the rest; the reader that makes a
composite in the stack cost 3.5 s a page is open (section 5).

Nothing else in the merged tree is gated by a still: the seam's, the capture level's and the magnify law's
own instruments (their rastertest and composetest cases, the overlap report, the transect, the per-leaf
ledger) are in the table above and run in `--selftest` and `--tool rastertest` on every build.

## 5. What is open

- The stack order among the heights is now a hand key: noaa.inlethq `over` 1 with its reason in the
  scene's `_over` (the seam session's call, taken). A date is still not a key the stack knows.
- The three eastern 2011 tiles paint black beyond the flight, and black is a colour, not an absence: the
  seam's feather and the capture gate do not see it (under surveyed water the bed classifier covers it).
  A copy with alpha, as the drone ortho got, is one Python line per tile and was not made.
- The three non-file height sources (the CUDEM grid, ETOPO 2022, survey.edits) have no own level, so a
  cold height tile still pays their per-level paint and push fold (the magnify session's ledger, section
  4): the own-level clause applied to them is the next step of 4.20, not started.
- The hand edits now yield to finer data (section 4), so what stands on the jetties is the composite's
  surface, light tower included; the lidar's bare earth is one `over` away. A source could declare which
  it is (surface or bare earth) and a scene choose; not built.
- The grid layers' tree identity is now their bytes (the block below); the paint's FORM is still
  `kTileTreeVersion`'s to bump by hand when the leaf paint changes.
- The three grid layers now declare an own level (section 4), so their bytes no longer depend on the
  order of asks; the 52 edits tiles and the kilometres at the globe's coarse rungs were that. What a cold
  height tile still costs is measured in the same block.
- Drone imagery AT the Merrimack: the PIEPR raw photos through OpenDroneMap (section 1).
- MassGIS 2025 (7.5 cm on Cape Cod, 15 cm here) would put a 2025 plane ortho beside the 2023 one.
- A stale tree folder from the hung first ingest, `cache/trees/usgs.tnb2025.ortho.c94dbf4c` (835 KB,
  2,890 files, 2,886 of them zero-byte), is prunable; `tree-prune` is Mark's to run.
- A scene's `sources` now merge by name (above); the scene files that repeat their base's entries
  (merrimack_hq, haulover_*, newburyport_*) could drop the repeats, and were left as they are.
- A composite's file-backed height rungs in the stack make a height page cost 3.5 s and decode tens of
  thousands of windows at frame time, while the height tree reads them in milliseconds: some reader
  samples the STACK through those files per load, and which one was not found (the buildings' stand field
  reads the stack by design but its page jobs never ran; the height tenant's own paints measured 5.8 ms).
  `globe_test.json` drops the composite by name until it is; the composite scene itself still carries it.
- **A land tile drawn on the sea in live flight** (Mark's second screenshot, 2026-10-18 late morning, the
  globe scene over the Salisbury reservation): one 256 x 128 page of the reservation's parking lot, sharp, standing
  dry some 250 m offshore, no water over it -- so its mask and colour agree it is land at an address that is
  sea: a residency mismatch in his words. Not reproduced headless: a settled still at an eye 100 m from his
  (`views.sal`, 42.8215 N -70.8165 E, 420 m, heading 42, tilt 52) is clean; the jetty rail flown with
  `--res-trace` makes 460 window moves, tells 79,416 slots, keeps 72,352 tiles at their slots and finds 0 at
  another pool slot three turns on (his log: 520 moves, 214 found elsewhere, 7,416 slots mapped to bytes
  already held, 72 colour loads failed or not whole). Read and ruled out by the code: the step lets a told
  slot's bytes go at once (B11), the pool slot is freed only with its retire, the shader's chain reads a rank
  only inside its box (Window.hlsli, the C++ twin agrees), the rows are filled after the steps in the same
  frame. What remains is the live case -- a want whose global key was taken under one binding and mapped or
  aliased after the binding stepped, with the loads slow enough (cold composites that morning) to leave the
  window open -- which no instrument catches today. The instrument that would: at every map and alias,
  re-derive the slot's ground under the current binding and refuse bytes whose ground is another (counted
  in the loader's line); Mark's flight with `--res-trace` and the frame of the screenshot would place it.
- The scenes list absolute `D:/DataCache/...` paths (as `streaming.gisLand` already does); a second
  machine needs the files at those paths or the entries edited.
