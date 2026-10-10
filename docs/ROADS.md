# ROADS: one road record for every vendor, and the surveys that know a bridge's height

*2026-10-10, on `claude/osm-roads-feasibility-297c45` over `claude/camera-rotor` (PR 74). What is
built: the record, three C++ harvesters, the loaders, the junction table, the bridge match, the
`[roads]` self-test and the `roads-probe` tool. What is not: a layer that draws a road, the profile
law that gives a ramp or a deck its height, a reader for any vendor but OSM. Every number below was
produced on this machine by the harness named beside it; nothing is estimated.*

## 0. In one paragraph

The owner asked whether the planet OSM file holds roads that a car could drive and that can be seen
from a distance, with ramps, bridges and tunnels, and with hooks for TomTom, HERE and the others.
It does, and the vertical world in road data is a LEVEL per end and a STRUCTURE flag, never a
height: OSM, TomTom and HERE all describe stacking this way. The deck heights live in the clearance
surveys: the FHWA National Bridge Inventory (every public road bridge in the US, with navigation and
underclearances) and NOAA's S-57 charts (every bridge over navigable water with its clearance above
a named tide datum, which this engine already models). So the design is one road record that every
vendor reads into (`RoadWay`), one span record that every survey reads into (`BridgeSpan`), both
untagged where a source is silent, and a match law that says which deck a way runs on. The curve
M(s), the profile and the deck are compose's, derived from these as a building's bottom and top are.

## 1. The record (src/compose/RoadWays.h)

The header's block comment is the specification; the formats are GAROAD01 (roads) and GABRDG01
(spans), each sorted by 0.05-degree cell of the record's first point with the buildings' binary cell
index, so a stack of them streams by box as the building stack does.

What each source fills, read from the files (TomTom MultiNet NH read, not harvested):

| member | OSM | TomTom `nw` | HERE / Overture / OpenDRIVE |
|---|---|---|---|
| class, rank | `highway` -> vocabulary + a rank table in the manifest | FRC 0..8, FOW | func class / class |
| ramp | `*_link` (direction untagged) | RAMP 1 exit / 2 entrance | ramp flag |
| structure | `bridge`, `tunnel`, `ford` | PARTSTRUC 1 tunnel 2 bridge | bridge / tunnel |
| level | one `layer`, to both ends | F_ELEV, T_ELEV per end | HERE: per shape point |
| junctions | shared node ids | F_JNCTID, T_JNCTID | connectors as records |
| oneway, lanes, speed, width | tags | ONEWAY, LANES, KPH; no width | ranged over [s0, s1] |
| geometry | nodes, 1e-7 deg | 2D polyline, WGS84 | OpenDRIVE: arcs, clothoids, elevation |

Deck height: none of them. `RoadPoint` carries a level and a height per point for the sources that
have them; `RoadWay::ranged` carries attributes over intervals of the way for the ones that have
those. A member a source lacks is NaN or the enum's untagged value, never a default: the default is
the scene's declared assumption, as `levelHeight` is for buildings.

## 2. The harvesters (harvester/cpp, built by build.bat <name>)

All three are C++ on the standard library (the OSM one on libosmium), by the owner's rule that
harvesting is C++ or C#, not Python. Each refuses what it cannot convert and counts the refusal by
name in its manifest.

**planet_roads.cpp** (686 lines): `highway=*` ways with locations, after
`osmium tags-filter <planet> w/highway` and `osmium add-locations-to-ways`. Massachusetts
(`D:\DataCache\OSM\roads\ma-roads-loc.osm.pbf`): 996,973 ways to 996,973 records in 1,131 cells,
4.2 s, 218 MB. Gate: every per-value count equals `osmium tags-count -t way 'highway=*'` exactly, all
32 vocabulary words and the 9 folded into "other"; an independent byte walk by the index found every
record in its cell. The vocabulary, the rank table and the form rules are in the manifest. The Gillis
Bridge is ways 9111277 and 1044568271 (`US 1`, layer 1, `bridge=movable`, `bridge:movable=bascule`).
The planet extract (`D:\DataCache\OSM\roads\planet_roads_chain.sh`) was running when this was
written; its harvest is a later step.

**nbi_bridges.cpp** (671 lines): the 2025 NBI "all records" file. 743,398 rows to 741,173 records
in 151,373 cells, 9.8 s. Refused 2,225 rows for coordinates the coding guide forbids (blank, zero,
minutes or seconds of 60 or more, outside the US range). Sentinels, each with the guide's wording in
the manifest: items 10 / 53 / 54B 99.99 (30 m or more) -> +inf; items 39 / 40 / 116 coded 0 when
item 38 says no navigation control -> NaN; a blank code -> 255 because NBI's 0 / 00 is a real code
("other"). Record types: '1' the route carried ON the structure (623,377), '2' and 'A'..'Z' routes
UNDER it; `origin` = "<state>|<structure number>|<type>". Categories: 724,530 fixed, 498 bascule,
231 lift, 178 swing, 163 suspension, 15,573 unknown. Two origins collide on `id` (two bridges with
one structure number in MA and VA): the origin is the identity, not the id.

**enc_bridges.cpp** (980 lines): an ISO 8211 / S-57 reader driven by each cell's own DDR (no
hardcoded field layouts), over all 7,323 NOAA base cells (2.10 GB) in 4.4 s, 81 MB peak: 38,039
records in 4,401 cells. Kinds: 20,963 bridges, 10,235 overhead cables, 4,335 pylons (427 towers,
1,067 piers), 715 gates, 635 dams, 615 overhead pipelines, 388 conveyors, 87 tunnels, 66 causeways.
Geometry: 93,489 edges chained, 0 breaks, 0 open rings of 16,506; the check was proven able to see
a failure by disabling edge reversal (54,901 open rings). Vertical datum of the clearances: 30,081
on mean high water, 5,584 on IGLD 1985 (the Great Lakes), the rest local or low water. Refusals: a
cell whose COMF is not 10,000,000 or whose HUNI is not metres (none). Not applied: 5,038 update
files in 2,234 cells, listed per cell in the manifest. A feature appears once per usage band that
charts it (97 ids in two cells), so the finest band is the consumer's choice, not deduplicated here.
INFORM is carried as `note`: it is the only thing that separates "Railway bridge" from the road
bridge at Newburyport, and it names the "John Greenleaf Whittier Bridge" (I-95).

## 3. The Merrimack, as the three sources see it

The US-1 Gillis Bridge at Newburyport (42.8155 N, 70.8705 W):

| source | what it says |
|---|---|
| OSM | ways 9111277 / 1044568271, `bridge=movable`, `bridge:movable=bascule`, layer 1, 1 lane each |
| NBI | `US 1 BRIDGE RD` over `WATER MERRIMACK RIVER`: nav vertical 10.70 m, nav horizontal 30.50 m, bascule (43B = 16), built 1976, 372.8 m long, one point 197 m from the way |
| ENC US5MA1VH | a bascule AREA, closed clearance 10.6 m, horizontal 15.2 m, note "HOR CL 100 FT (CLOSED)", on mean high water; beside it two swing spans "Railway bridge", closed 3.9 m, horizontal 21 / 19.5 m; 8 piers |

NBI's 10.70 and ENC's 10.6 agree. Upstream the ENC names the Whittier Bridge (16.9 m / 141.7 m and
14.6 m / 82.6 m); the Rocks Village swing bridge is 4.8 m closed; the Chain Bridge is one of two
unnamed spans near Deer Island (8.5 m fixed, or 4.2 m swing) and stays unconfirmed. ENC leaves
VERCOP (open clearance) empty on every bascule here: that is NaN, never +inf.

## 4. The match law (MatchBridges)

A span is a DECK: every bridge way on it shares its clearances (US 1's two carriageways and the
cycleway beside them are one Gillis Bridge), so a span pairs with several ways, by OVERLAP: the
length of the way's polyline within the span's reach (inside an area, or within the reach of a
line or a point), every part counted, on the local chart about the span. The reach is the survey's
own positional uncertainty: a point span (NBI's one coordinate per bridge) reaches tens of metres,
a charted line or area a few. Pairs are sorted by span, then overlap, then rank. Measured at
Newburyport by `roads-probe`: with one reach of 60 m the railway's swing spans (3.9 m) landed on
US 1; with the shape reach at 5 m they do not, and the 10.6 m lift span remains on way 9111277.
The NBI point goes to the nearest bridge way, `Bridge Road`, 65 m of overlap.

## 5. What is derived, and not yet built

- **The centreline as motors M(s).** Biarcs through the polyline (arcs are CGA circles) give a G1
  curve with no overshoot at sparse corners; a clothoid refinement is physics, since roads are built
  from arcs and spirals. OpenDRIVE hands these primitives over directly; a polyline is the all-lines
  case.
- **The profile.** The smoothest height along s whose grade stays within road limits, that meets
  the ground at level-0 ends, and whose deck clears the crossed road by the measured clearance
  (NBI 54B, OSM `maxheight` on the lower way) or the design clearance where none is measured;
  over water the deck underside stands at the ENC clearance above its tide datum. Lidar decks
  (USGS 3DEP class 17) pin the curve outright where fetched. One law; the ground is its limit.
- **Tunnels.** Portals known, profile derived under cover constraints (below the ground along the
  way; under a channel below eHydro's dredged depth).
- **Seen from afar.** Wedge importance per vertex (harvest_vectors.py) makes LOD a filter; past a
  pixel of width a road is the imagery's, as the buildings fold.
- **Readers** for TomTom, HERE, Overture, OpenDRIVE, state DOT centrelines: each one file into the
  same record; the engine never names a vendor.
- **Loose ends found on the way:** NBI's 587 rows with seconds coded 60.00 (refused, the point they
  mean is exact); 36,524 item-10 values of 30.47 / 30.48 that look like the old "100 ft" code kept
  as written; ENC updates not applied; the NBI coordinate datum unstated; `harvester/cpp/build.bat`
  rewritten because VsDevCmd fails under delayed expansion and the root `build.bat` may carry the
  same latent fault.

## 6. Lines

| part | lines |
|---|---|
| harvesters (tools, outside the engine) | 2,337 |
| engine: record + loaders + match | 536 + 196 (header) |
| self-test + probe | 460 |
| wiring (CMake, SelfTest, Tools, main, Assembly) | 11 |

Licences travel in the manifests: OSM ODbL 1.0; NBI US public domain; NOAA ENC usable for any
purpose (15 CFR 995), not redistributed as provided.
