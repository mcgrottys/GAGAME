// ================================================================================================
//  RoadWays - ONE ROAD RECORD FOR EVERY VENDOR, AND ONE BRIDGE RECORD FOR EVERY SURVEY OF CLEARANCE.
//
//  The engine knows one road. A source (OSM today; TomTom, HERE, Overture, OpenDRIVE, a state DOT
//  centreline tomorrow) is a READER into this record, and the only question a new vendor asks is
//  which members its reader can fill. Every member a source does not carry stays UNTAGGED (NaN, or
//  the enum's untagged value), never a guess: a default is the scene's declared assumption, visible
//  as one, as `levelHeight` is for buildings (compose/BuildingSolids.h).
//
//  THE SOURCES, and what each one fills (2026-10-10, docs of the survey in GAMEPLAN):
//    OSM        highway=* ways with their node ids and coordinates: class, link (ramp, direction
//               untagged), bridge/tunnel, one `layer` (written to both ends), oneway, lanes, width,
//               maxspeed, maxheight, surface, access, name, ref. Junctions = the shared node ids.
//    TomTom     (MultiNet nw: read, not harvested) FRC rank, FOW form, RAMP exit/entrance, PARTSTRUC
//               bridge/tunnel, F_ELEV/T_ELEV a level PER END, F_JNCTID/T_JNCTID explicit junctions,
//               ONEWAY, LANES, KPH; no width, no elevation.
//    HERE       a level per SHAPE POINT (Navstreets z-levels), lanes per direction, conditions over
//               ranges of the link. Overture: connectors as records, attributes over [s0, s1].
//    OpenDRIVE  the centreline as lines, arcs and clothoids with an elevation polynomial: the
//               fidelity ceiling. A polyline is the all-lines case of `points`.
//  Neither OSM nor TomTom nor HERE carries the DECK HEIGHT of a bridge: the vertical world in road
//  data is a level per end and a structure flag. The heights are in the clearance surveys below.
//
//  THE CLEARANCE SURVEYS (BridgeSpan), one record for every source of them:
//    NBI        FHWA National Bridge Inventory: every public road bridge in the US, one point each,
//               with the navigation vertical / horizontal clearance (items 39/40, over water), the
//               clearance under the deck (54B) and over it (53), the route's minimum (10), length,
//               max span, deck width, spans, kind and type of structure, year, service on / under.
//    ENC        NOAA S-57 charts: every bridge over navigable water as a BRIDGE feature (point, line
//               or area, its real geometry) with VERCLR / VERCCL / VERCOP / VERCSA / HORCLR, CATBRG
//               (fixed, bascule, swing, lift, ...), and VERDAT: the tide datum the clearance is
//               measured from, which the engine's water already models. Overhead cables and
//               pipelines, pylons and piers, tunnels and gates are the same record, by `kind`.
//    lidar      (later) USGS 3DEP point clouds: the deck surface itself, class 17, as `deck`.
//
//  THE FILES, one format a record, both written by harvester/cpp/*.cpp and read here:
//    GAROAD01  <stem>.roads.bin + .roads.json + .roads.idx     roads (planet_roads.cpp from OSM)
//    GABRDG01  <stem>.bridges.bin + .bridges.json + .bridges.idx   spans (nbi_bridges.cpp,
//              enc_bridges.cpp: two harvesters, ONE format, `source` tells them apart)
//  Both are sorted by cell of `cellDeg` (0.05 deg) by the record's FIRST point, with the binary cell
//  index of the buildings harvest (int32 ix, int32 iy, int64 offset, int64 count, by (iy, ix)), so
//  a stack of them streams by box exactly as the building stack does. The .bin opens with its 8-byte
//  format name ("GAROAD01" / "GABRDG01") and the index's offsets count from the file's start, as the
//  buildings' do. A record lives in ONE cell, its first point's, so a reader widens its box by the
//  longest way it expects (a motorway way may run kilometres). Little-endian throughout. Strings are
//  UTF-8 with uint16 lengths (OSM caps a value at 255 characters).
//  Coordinates are int32 in 1e-7 degree, WGS84 lon/lat (OSM's own precision; NOAA ENC's COMF is
//  10,000,000, so an ENC coordinate is the same integer exactly). Lengths are float32 METRES;
//  NaN = untagged / not applicable; +inf = "no restriction" (NBI codes 9999: 30 m or more).
//
//  GAROAD01 record:
//    int64  id                 OSM way id (a vendor's id is a string: see `nativeId`)
//    uint8  highway            index into the manifest's "highway" vocabulary (254 other, 255 untagged)
//    uint8  rank               0 motorway .. 8 (NBI/TomTom FRC-like); 255 untagged
//    uint8  form               RoadForm
//    uint8  ramp               RoadRamp
//    uint8  structure          RoadStructure
//    int8   levelFrom, levelTo level at each end (OSM: `layer` to both); -128 untagged
//    uint8  oneway             RoadOneway
//    uint8  surface            index into the manifest's "surface" vocabulary (254 other, 255 untagged)
//    uint8  access             RoadAccess
//    uint8  pad[2]
//    float  lanes, lanesForward, lanesBackward, widthM, maxspeedKph, maxheightM, maxweightT
//    uint16 nameLen, refLen, nativeIdLen; then name, ref, nativeId as UTF-8 (nativeId "" for OSM)
//    uint32 n; then n x { int64 nodeId; int32 x; int32 y }
//
//  GABRDG01 record:
//    int64  id                 NBI: FNV-1a of the origin string; ENC: the feature's FOID (not unique
//                              across usage bands or within NBI: the origin is the identity)
//    uint8  source             BridgeSource
//    uint8  kind               BridgeKind (bridge, overhead cable, ...)
//    uint8  category           S-57 CATBRG (1 fixed .. 12 suspension) for a bridge; for a pylon
//                              S-57 CATPYL (1 power, 2 telephone, 3 cableway, 4 bridge tower,
//                              5 bridge pier); 0 unknown. NBI maps 43B:
//                              13 -> 12, 15 -> 4, 16 -> 5, 17 -> 3, 18 -> kind tunnel, blank or
//                              00 (other) -> 0, every other design -> 1 fixed
//    uint8  primitive          0 point, 1 line, 2 area
//    uint8  verdat             S-57 VERDAT the clearances are measured from (ENC); 0 not stated
//    uint8  clrUnderRef        NBI 54A: 'H' highway, 'R' railroad, 'N' other; 255 blank
//    uint8  serviceOn, serviceUnder   NBI 42A / 42B codes (0 is a real code, "other"); 255 blank
//    uint8  nbiKind, nbiType   NBI 43A / 43B raw codes (0 / 00 real: "other"); 255 blank
//    uint8  navigation         NBI 38: 0 no control, 1 yes, 2 not applicable ('N'); 255 blank
//    uint8  pad
//    float  vertClr            ENC VERCLR; NBI 39 navigation vertical clearance
//    float  vertClrClosed      ENC VERCCL; NBI 116 (vertical lift bridge, closed)
//    float  vertClrOpen        ENC VERCOP
//    float  vertClrSafe        ENC VERCSA
//    float  horClr             ENC HORCLR; NBI 40
//    float  clrOverDeck        NBI 53
//    float  clrUnder           NBI 54B (see clrUnderRef)
//    float  minVertClrRoute    NBI 10
//    float  lengthM, maxSpanM, deckWidthM     NBI 49, 48, 52
//    float  heightM            ENC HEIGHT (a pylon's, above VERDAT); NBI NaN
//    uint16 spansMain, spansApproach, yearBuilt   NBI 45, 46, 27; 0 none
//    uint16 nameLen, crossesLen, originLen, noteLen; then name (ENC OBJNAM; NBI facility carried,
//           item 7), crosses (NBI features intersected, item 6A), origin (ENC cell name; NBI
//           "<state>|<structure number>|<record type 5A>": '1' the route carried ON the structure,
//           '2' and 'A'..'Z' routes passing UNDER it), note (ENC INFORM, e.g. "Railway bridge";
//           NBI "") as UTF-8
//    uint32 parts; per part: uint32 n, uint8 outer, uint8 pad[3], n x { int32 x; int32 y }
//           (a point: one part of one; a line: one part, more if the chart's line breaks; an
//           area: rings with the closing vertex REPEATED (first == last), the first outer)
//    An ENC leaves an attribute EMPTY for unknown (VERCOP on every bascule here): that is NaN, never
//    +inf; +inf is only a survey's explicit "no restriction" code (NBI 9999).
//
//  LICENCES travel in the manifests: OSM is ODbL 1.0 "(c) OpenStreetMap contributors"; NBI is US
//  public domain; NOAA ENC may be used for any purpose (15 CFR 995) but not redistributed as is.
// ================================================================================================
#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include <limits>
#include <string>
#include <vector>

namespace ga {

enum class RoadForm : uint8_t {
    Carriageway = 0, DualCarriageway = 1, Ramp = 2, Roundabout = 3, Service = 4, ParkingAisle = 5,
    Path = 6, Track = 7, Steps = 8, Other = 9, Untagged = 255
};
enum class RoadRamp : uint8_t { None = 0, Exit = 1, Entrance = 2, DirectionUntagged = 3 };
enum class RoadStructure : uint8_t { None = 0, Bridge = 1, Tunnel = 2, Ford = 3, Culvert = 4, Untagged = 255 };
enum class RoadOneway : uint8_t { Both = 0, Forward = 1, Backward = 2, Closed = 3, Untagged = 255 };
enum class RoadAccess : uint8_t { Public = 0, Private = 1, No = 2, Permissive = 3, Destination = 4, Other = 5, Untagged = 255 };
enum class BridgeSource : uint8_t { Nbi = 0, Enc = 1, Osm = 2, Other = 3 };
enum class BridgeKind : uint8_t { Bridge = 0, OverheadCable = 1, OverheadPipeline = 2, Tunnel = 3, Pylon = 4, Conveyor = 5, Gate = 6, Dam = 7, Causeway = 8 };

constexpr double kRoadCellDeg = 0.05;
constexpr int8_t kLevelUntagged = -128;

inline float RoadUntagged() { return std::numeric_limits<float>::quiet_NaN(); }

// A point of a way: the node id (a junction where another way shares it) and its place.
struct RoadPoint {
    int64_t nodeId = 0;
    double lon = 0.0, lat = 0.0;   // degrees
    // Vertical members a richer source fills; untagged under OSM and TomTom.
    int8_t level = kLevelUntagged;      // HERE: a level per shape point
    float heightM = RoadUntagged();     // an HD product or lidar: the surface itself, on the height datum
};

// One way of one source, as the file states it. Nothing here is derived: the motor curve M(s), the
// height profile and a bridge's deck are compose's, as `bottom` and `top` are for a building.
struct RoadWay {
    int64_t id = 0;               // OSM way id; 0 when the source's id is a string (nativeId)
    std::string nativeId;         // a vendor's own id, namespaced by the source's name
    std::string source;           // the source's name in the scene (provenance per record)
    uint8_t highway = 254;        // vocabulary index (the manifest's "highway"); 254 other
    std::string highwayTag;       // the vocabulary word, resolved at load ("residential", ...)
    uint8_t rank = 255;           // 0 motorway .. 8; 255 untagged
    RoadForm form = RoadForm::Untagged;
    RoadRamp ramp = RoadRamp::None;
    RoadStructure structure = RoadStructure::Untagged;
    int8_t levelFrom = kLevelUntagged, levelTo = kLevelUntagged;
    RoadOneway oneway = RoadOneway::Untagged;
    std::string surface;          // the vocabulary word; "" untagged
    RoadAccess access = RoadAccess::Untagged;
    float lanes = RoadUntagged(), lanesForward = RoadUntagged(), lanesBackward = RoadUntagged();
    float widthM = RoadUntagged(), maxspeedKph = RoadUntagged(), maxheightM = RoadUntagged(), maxweightT = RoadUntagged();
    std::string name, ref;
    std::vector<RoadPoint> points;
    // A ranged attribute (Overture, HERE conditions): value over [s0, s1] of the way's length, 0..1.
    // The scalar members above are the one-entry [0, 1] case; a reader that has ranges fills these.
    struct Ranged {
        float s0 = 0.0f, s1 = 1.0f;
        std::string key, value;
    };
    std::vector<Ranged> ranged;
};

// A junction: where ways meet. OSM's shared nodes become rows of this; Overture's connectors and
// TomTom's junction ids are rows of it directly.
struct RoadNode {
    int64_t id = 0;
    double lon = 0.0, lat = 0.0;
    std::vector<int64_t> ways;    // the ways that pass through or end here
};

// One surveyed span: a bridge (or a cable, a pipeline, a tunnel, a pier) with the clearances a
// survey measured, as the file states them. Lengths in metres; NaN untagged; +inf no restriction.
struct BridgeSpan {
    int64_t id = 0;
    BridgeSource source = BridgeSource::Other;
    std::string sourceName;       // the source's name in the scene
    BridgeKind kind = BridgeKind::Bridge;
    uint8_t category = 0;         // S-57 CATBRG; 0 unknown
    uint8_t primitive = 0;        // 0 point, 1 line, 2 area
    uint8_t verdat = 0;           // S-57 VERDAT of the clearances; 0 not stated
    uint8_t clrUnderRef = 255;    // 'H', 'R', 'N'; 255 blank
    uint8_t serviceOn = 255, serviceUnder = 255, nbiKind = 255, nbiType = 255;   // 255 blank
    uint8_t navigation = 255;
    float vertClr = RoadUntagged(), vertClrClosed = RoadUntagged(), vertClrOpen = RoadUntagged(), vertClrSafe = RoadUntagged();
    float horClr = RoadUntagged(), clrOverDeck = RoadUntagged(), clrUnder = RoadUntagged(), minVertClrRoute = RoadUntagged();
    float lengthM = RoadUntagged(), maxSpanM = RoadUntagged(), deckWidthM = RoadUntagged();
    float heightM = RoadUntagged();   // a pylon's height (ENC HEIGHT)
    uint16_t spansMain = 0, spansApproach = 0, yearBuilt = 0;
    std::string name, crosses, origin, note;
    std::vector<std::vector<double>> parts;   // each: lon0, lat0, lon1, lat1, ... (degrees)
    std::vector<uint8_t> outer;
    // Later: the deck surface from lidar, metres on the height datum along the span.
    std::vector<std::array<double, 3>> deck;   // lon, lat, height
};

struct RoadSourceSpec {
    std::string name, path;   // a .roads.json manifest (GAROAD01)
    double over = 0.0;
};
struct BridgeSourceSpec {
    std::string name, path;   // a .bridges.json manifest (GABRDG01)
    double over = 0.0;
};

// Every way of one harvest whose first point lies in the box (degrees), the manifest's vocabularies
// resolved onto the records. False, and why, when the file is refused.
bool LoadRoadSource(const std::string& manifestPath, double lon0, double lat0, double lon1, double lat1,
                    std::vector<RoadWay>& out, std::string* why);
bool LoadBridgeSource(const std::string& manifestPath, double lon0, double lat0, double lon1, double lat1,
                      std::vector<BridgeSpan>& out, std::string* why);

// The junction table of a set of ways: every node id two or more ways share, or a way's end.
std::vector<RoadNode> JunctionsOf(const std::vector<RoadWay>& ways);

// Which surveyed spans belong to which bridge ways. A span is a DECK: every way tagged bridge that
// runs on it shares its clearances (the two carriageways of US 1 and the cycleway beside them are
// one Gillis Bridge), so a span may pair with several ways and the law is OVERLAP, not nearness:
//   overlap(way, span) = the length in metres of the way's polyline that lies within the reach r of
//   the span's geometry (inside an area or within r of its edge, or within r of a line or a point),
// measured on a local chart about the span (dx = dlon * 111320 cos lat, dy = dlat * 110574), every
// part of the span counted. THE REACH IS THE SURVEY'S OWN POSITIONAL UNCERTAINTY, not one number:
//   withinPointM  a point span (NBI: one coordinate per bridge, anywhere on or near it): 60 m;
//   withinShapeM  a charted line or area (ENC: metre-grade geometry of the deck itself): 5 m.
// One reach for both lets a parallel railway bridge 30 m off share the road's clearances, or a
// bridge's NBI point miss its deck. A pair is kept where overlap > 0 (a point span: the nearest
// bridge way within withinPointM only, overlap = its length within). Returned sorted by span, then
// overlap descending, then rank ascending: every way on the deck is listed, and a consumer that
// needs the road takes the lowest rank among them (a primary before the cycleway beside it).
struct BridgeMatch {
    size_t way = 0, span = 0;
    double overlapM = 0.0;
};
std::vector<BridgeMatch> MatchBridges(const std::vector<RoadWay>& ways, const std::vector<BridgeSpan>& spans,
                                      double withinPointM = 60.0, double withinShapeM = 5.0);

// The [roads] block of --selftest (compose/RoadWaysTest.cpp): the formats round-trip and the match.
bool RunRoadSelfTest();

}  // namespace ga
