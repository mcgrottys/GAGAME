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

**The gate on the merged tree** (one take each; the other sessions' engines shared the GPU throughout):
`--tool rastertest` 57 checks; `--selftest` 29 suites PASS, no unplanted FAIL; the demo scene against the
A arm at the same instant: the mouth settled exact in 840 frames, 20.8 ms GPU, 25.8 % px / SSIM 0.984;
Town Neck Beach from 600 m settled in 1,299 frames, 15.8 ms, the stair along the beach gone and the houses
standing as solids, 54 % px / SSIM 0.875 (A has no ortho there); the jetty view did not settle in 3,000
frames under the contention (7,779 of 32,479 tiles pending), 27.5 ms, 13.9 % px / SSIM 0.951 on what it
had. The jetty view also shows a flat, faceted top on the north jetty -- in the A arm too, and in the
shipped scene since the MassGIS orthos went in: `data/gis/edits.geojson`'s hand polygons (OSM's jetty ways)
put a 2.5 m NAVD88 crest floor under the whole structure, flat, and the 2023 leaf-off ortho paints its dark
riprap with little contrast, so the eye reads a slab where Google's imagery read rocks. The 2018 lidar has
the structure at 1 m (valid on 98 % of the north jetty's outline, crests to 4.4 m): the hand floor should
yield to a finer height where one has data -- the finest-witness law the shoreline took tonight, applied to
the edits. Not done; it predates this branch.

Still to fold, when their sessions report (each told: no PR, commit on your branch): the seam feather at
a height source's data edge (`claude/interesting-morse-eaa9c3`: the law built, 61 checks, the lidar
against the composite with no datum bias at the overlap; its picture gate waits on the corrected bed),
the ortho's capture level as a weight (`claude/distracted-zhukovsky-997ee8`: built, planted case
passing, stills queued), magnify-from-the-kept-tile below a source's own level (`claude/vigorous-wu-bf14f9`,
in progress), and the coverage-conservation probe (a card, not started).

## 5. What is open

- The stack order among the heights is grain (StackOrder's default): the 2018 lidar stands over the 2025
  channel survey where both have a bottom. A date is not a key the stack knows; `over` would say it by
  hand. The channel itself is lidar nodata, so the survey shows there either way.
- The three eastern 2011 tiles paint black beyond the flight; a copy with alpha, as the drone ortho got,
  is one Python line per tile and was not made.
- Drone imagery AT the Merrimack: the PIEPR raw photos through OpenDroneMap (section 1).
- MassGIS 2025 (7.5 cm on Cape Cod, 15 cm here) would put a 2025 plane ortho beside the 2023 one.
- A stale tree folder from the hung first ingest, `cache/trees/usgs.tnb2025.ortho.c94dbf4c` (835 KB,
  2,890 files, 2,886 of them zero-byte), is prunable; `tree-prune` is Mark's to run.
- The scenes list absolute `D:/DataCache/...` paths (as `streaming.gisLand` already does); a second
  machine needs the files at those paths or the entries edited.
