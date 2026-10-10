// ================================================================================================
//  RoadWaysTest - the [roads] block of --selftest: GAROAD01 and GABRDG01 written here by hand
//  (manifest + .idx + .bin under out\roadtest, the layouts of RoadWays.h), read back by box.
//
//  Two cells of 0.05 deg at Newburyport.
//    Cell A: way 1 (residential, plain) and way 2 (a primary bridge) meeting at node 102; ways 4
//            (primary, rank 2) and 5 (cycleway, rank 8), both bridges, side by side on one deck;
//            span 11 (an NBI point 4 m off way 2) and span 13 (an ENC area, the deck under 4 and 5,
//            its ring closed by repeating the first vertex).
//    Cell B: way 3 (vocabulary 254 / 255, lanes NaN, maxheight +inf) and span 12 (an ENC line 2 m
//            off way 3, which is NOT a bridge, so it must stay unmatched; heightM and a note).
//  A box over cell A sees only A's records. The overlap law (RoadWays.h, MatchBridges): span 11
//  pairs with way 2 alone, its overlap the length of way 2 within 60 m of the point; span 13
//  pairs with both 4 and 5 at equal overlap (their whole length), the primary first by rank.
// ================================================================================================
#include "compose/RoadWays.h"

#include "core/Common.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>

namespace ga {

namespace {

struct Bytes {
    std::string b;
    template <class T>
    void Put(T v) {
        b.append(reinterpret_cast<const char*>(&v), sizeof(T));
    }
    void Str(const std::string& s) { b += s; }
};

struct TestPoint {
    int64_t node;
    int32_t x, y;
};

void PutWay(Bytes& o, int64_t id, uint8_t highway, uint8_t rank, uint8_t structure, uint8_t surface, int8_t layer,
            float lanes, float maxheight, const std::string& name, const std::string& ref,
            const std::vector<TestPoint>& pts) {
    o.Put<int64_t>(id);
    o.Put<uint8_t>(highway);
    o.Put<uint8_t>(rank);
    o.Put<uint8_t>(0);   // form: carriageway
    o.Put<uint8_t>(0);   // ramp: none
    o.Put<uint8_t>(structure);
    o.Put<int8_t>(layer);
    o.Put<int8_t>(layer);
    o.Put<uint8_t>(1);   // oneway: forward
    o.Put<uint8_t>(surface);
    o.Put<uint8_t>(0);   // access: public
    o.Put<uint8_t>(0);
    o.Put<uint8_t>(0);
    const float nan = RoadUntagged();
    o.Put<float>(lanes);
    o.Put<float>(nan);
    o.Put<float>(nan);
    o.Put<float>(7.5f);   // width
    o.Put<float>(nan);
    o.Put<float>(maxheight);
    o.Put<float>(nan);
    o.Put<uint16_t>(static_cast<uint16_t>(name.size()));
    o.Put<uint16_t>(static_cast<uint16_t>(ref.size()));
    o.Put<uint16_t>(0);
    o.Str(name);
    o.Str(ref);
    o.Put<uint32_t>(static_cast<uint32_t>(pts.size()));
    for (const TestPoint& p : pts) {
        o.Put<int64_t>(p.node);
        o.Put<int32_t>(p.x);
        o.Put<int32_t>(p.y);
    }
}

void PutSpan(Bytes& o, int64_t id, uint8_t source, uint8_t primitive, float vertClr, float horClr, float heightM,
             const std::string& name, const std::string& origin, const std::string& note,
             const std::vector<int32_t>& xy) {
    o.Put<int64_t>(id);
    o.Put<uint8_t>(source);
    o.Put<uint8_t>(0);   // kind: bridge
    o.Put<uint8_t>(2);   // CATBRG 2 opening
    o.Put<uint8_t>(primitive);
    o.Put<uint8_t>(0);   // verdat
    o.Put<uint8_t>('N');
    for (int k = 0; k < 4; ++k) o.Put<uint8_t>(255);   // serviceOn/Under, nbiKind/Type: blank
    o.Put<uint8_t>(1);   // navigation
    o.Put<uint8_t>(0);
    const float nan = RoadUntagged();
    o.Put<float>(vertClr);
    for (int k = 0; k < 3; ++k) o.Put<float>(nan);
    o.Put<float>(horClr);
    for (int k = 0; k < 6; ++k) o.Put<float>(nan);
    o.Put<float>(heightM);
    o.Put<uint16_t>(0);
    o.Put<uint16_t>(0);
    o.Put<uint16_t>(1939);
    o.Put<uint16_t>(static_cast<uint16_t>(name.size()));
    o.Put<uint16_t>(0);
    o.Put<uint16_t>(static_cast<uint16_t>(origin.size()));
    o.Put<uint16_t>(static_cast<uint16_t>(note.size()));
    o.Str(name);
    o.Str(origin);
    o.Str(note);
    o.Put<uint32_t>(1);
    o.Put<uint32_t>(static_cast<uint32_t>(xy.size() / 2));
    o.Put<uint8_t>(1);
    o.Put<uint8_t>(0);
    o.Put<uint8_t>(0);
    o.Put<uint8_t>(0);
    for (int32_t v : xy) o.Put<int32_t>(v);
}

// The harvest of one format: the two cells' records after the magic, the index, the manifest.
void WriteHarvest(const std::string& dir, const std::string& stem, const char* magic, const char* tail,
                  const Bytes& cellA, int nA, const Bytes& cellB, int nB, const std::string& extra) {
    Bytes bin;
    bin.Str(std::string(magic, 8));
    const int64_t offA = static_cast<int64_t>(bin.b.size());
    bin.Str(cellA.b);
    const int64_t offB = static_cast<int64_t>(bin.b.size());
    bin.Str(cellB.b);
    Bytes idx;   // (ix, iy, off, n), by (iy, ix): A (-1418, 856), B (-1417, 856)
    idx.Put<int32_t>(-1418);
    idx.Put<int32_t>(856);
    idx.Put<int64_t>(offA);
    idx.Put<int64_t>(nA);
    idx.Put<int32_t>(-1417);
    idx.Put<int32_t>(856);
    idx.Put<int64_t>(offB);
    idx.Put<int64_t>(nB);
    const std::string base = dir + "/" + stem + tail;   // e.g. .../t.roads
    std::ofstream(base + ".bin", std::ios::binary) << bin.b;
    std::ofstream(base + ".idx", std::ios::binary) << idx.b;
    std::ofstream(base + ".json", std::ios::binary)
        << "{\n \"format\": \"" << magic << "\",\n \"cellDeg\": 0.05,\n \"cellIndex\": \"" << stem << tail
        << ".idx\",\n \"cellCount\": 2" << extra << "\n}\n";
}

}  // namespace

bool RunRoadSelfTest() {
    bool ok = true;
    auto check = [&ok](bool c, const char* what) {
        Log("[roads] %s %s", c ? "ok  " : "FAIL", what);
        ok &= c;
    };
    std::error_code ec;
    const std::string dir = "out/roadtest";
    std::filesystem::create_directories(dir, ec);
    const float nan = RoadUntagged(), inf = std::numeric_limits<float>::infinity();
    const double kPi = 3.14159265358979323846;

    // Cell A (lon -70.90..-70.85): way 1 west to east along 42.81, way 2 north from its end; ways 4
    // and 5 east-west across the deck of span 13, 22 m apart.
    const std::vector<TestPoint> w1 = {{100, -708700000, 428100000}, {101, -708690000, 428100000}, {102, -708680000, 428100000}};
    const std::vector<TestPoint> w2 = {{102, -708680000, 428100000}, {103, -708680000, 428110000}, {104, -708680000, 428120000}};
    const std::vector<TestPoint> w4 = {{300, -708798000, 428202000}, {301, -708792000, 428202000}};
    const std::vector<TestPoint> w5 = {{400, -708798000, 428204000}, {401, -708792000, 428204000}};
    // Cell B (lon -70.85..-70.80): way 3.
    const std::vector<TestPoint> w3 = {{200, -708200000, 428150000}, {201, -708190000, 428150123}};
    Bytes ra, rb;
    PutWay(ra, 1, 0, 6, 0, 0, 0, 2.0f, nan, "Water Street", "", w1);
    PutWay(ra, 2, 1, 2, 1, 0, 1, 4.0f, nan, "Gillis Bridge", "US 1", w2);
    PutWay(ra, 4, 1, 2, 1, 0, 1, 2.0f, nan, "Deck Road", "US 1", w4);
    PutWay(ra, 5, 2, 8, 1, 0, 1, nan, nan, "Deck Path", "", w5);
    PutWay(rb, 3, 254, 255, 0, 255, kLevelUntagged, nan, inf, "", "", w3);
    WriteHarvest(dir, "t", "GAROAD01", ".roads", ra, 4, rb, 1,
                 ",\n \"highway\": [\"residential\", \"primary\", \"cycleway\", \"(other: 254)\"],\n \"surface\": [\"asphalt\"]");
    // Span 11: 4 m east of way 2. Span 13: the area under ways 4 and 5. Span 12: a line 2 m north
    // of way 3.
    Bytes sa, sb;
    PutSpan(sa, 11, 0, 0, 10.4f, 45.7f, nan, "US 1 over Merrimack", "25|100000001|1", "", {-708679500, 428105000});
    PutSpan(sa, 13, 1, 2, 9.1f, 30.0f, nan, "Deck", "US5MA1AM.000", "Road bridge",
            {-708800000, 428200000, -708790000, 428200000, -708790000, 428206000, -708800000, 428206000, -708800000, 428200000});
    PutSpan(sb, 12, 1, 1, inf, nan, 25.0f, "Not over a bridge", "US5MA1AM.000", "Railway bridge",
            {-708199000, 428150180, -708195000, 428150180});
    WriteHarvest(dir, "t", "GABRDG01", ".bridges", sa, 2, sb, 1, "");

    const std::string roads = dir + "/t.roads.json", bridges = dir + "/t.bridges.json";
    std::string why;
    std::vector<RoadWay> all, a;
    check(LoadRoadSource(roads, -70.90, 42.80, -70.80, 42.85, all, &why), "GAROAD01 opens");
    if (!why.empty()) Log("[roads]   refused: %s", why.c_str());
    check(all.size() == 5, "a box over both cells reads all five ways");
    check(LoadRoadSource(roads, -70.90, 42.80, -70.85, 42.85, a, &why) && a.size() == 4 && a[0].id == 1 && a[3].id == 5,
          "a box over cell A reads only A's four ways");
    if (all.size() == 5) {
        const RoadWay &r1 = all[0], &r2 = all[1], &r5 = all[3], &r3 = all[4];
        check(r1.highwayTag == "residential" && r2.highwayTag == "primary" && r5.highwayTag == "cycleway" &&
                  r3.highwayTag == "other",
              "highway vocabulary resolved (254 -> other)");
        check(r1.surface == "asphalt" && r3.surface.empty(), "surface vocabulary resolved (255 -> untagged)");
        check(std::isnan(r3.lanes) && std::isinf(r3.maxheightM) && r3.maxheightM > 0 && std::isnan(r1.maxheightM),
              "NaN stays NaN, +inf stays +inf");
        check(r2.structure == RoadStructure::Bridge && r2.levelFrom == 1 && r2.levelTo == 1 && r2.ref == "US 1" &&
                  r2.name == "Gillis Bridge" && r2.oneway == RoadOneway::Forward && r2.widthM == 7.5f &&
                  r2.rank == 2 && r5.rank == 8 && r3.rank == 255 && r3.levelFrom == kLevelUntagged,
              "the fixed members and the strings read as written");
        bool exact = r3.points.size() == 2;
        for (size_t k = 0; exact && k < 2; ++k) {
            exact = r3.points[k].nodeId == w3[k].node && r3.points[k].lon == w3[k].x * 1e-7 &&
                    r3.points[k].lat == w3[k].y * 1e-7 &&
                    static_cast<int32_t>(std::llround(r3.points[k].lat * 1e7)) == w3[k].y;
        }
        check(exact, "the points round-trip exactly (node ids, 1e-7 degree)");
    }
    const std::vector<RoadNode> j = JunctionsOf(all);
    bool shared = false;
    for (const RoadNode& n : j) {
        if (n.id == 102) shared = n.ways.size() == 2 && n.ways[0] == 1 && n.ways[1] == 2;
    }
    check(j.size() == 9 && shared, "junctions: shared node 102 (ways 1, 2) and the eight other ends");

    std::vector<BridgeSpan> spans, spansA;
    check(LoadBridgeSource(bridges, -70.90, 42.80, -70.80, 42.85, spans, &why) && spans.size() == 3,
          "GABRDG01 reads three spans");
    check(LoadBridgeSource(bridges, -70.90, 42.80, -70.85, 42.85, spansA, &why) && spansA.size() == 2 &&
              spansA[0].id == 11 && spansA[1].id == 13,
          "a box over cell A reads only spans 11 and 13");
    if (spans.size() == 3) {
        const BridgeSpan& s1 = spans[0];
        const BridgeSpan& s3 = spans[1];
        const BridgeSpan& s2 = spans[2];
        check(s1.source == BridgeSource::Nbi && s1.vertClr == 10.4f && s1.horClr == 45.7f && s1.yearBuilt == 1939 &&
                  s1.name == "US 1 over Merrimack" && s1.origin == "25|100000001|1" && s1.clrUnderRef == 'N' &&
                  s1.serviceOn == 255 && s1.nbiType == 255 && std::isnan(s1.heightM) && s1.note.empty() &&
                  s1.parts.size() == 1 && s1.parts[0].size() == 2 && s1.outer[0] == 1,
              "span 11's members and its point read as written");
        check(s2.source == BridgeSource::Enc && std::isinf(s2.vertClr) && std::isnan(s2.horClr) && s2.primitive == 1 &&
                  s2.heightM == 25.0f && s2.note == "Railway bridge" && s2.parts[0].size() == 4 &&
                  s2.parts[0][2] == -708195000 * 1e-7,
              "span 12: +inf, NaN, heightM, note and its line read as written");
        check(s3.primitive == 2 && s3.note == "Road bridge" && s3.parts[0].size() == 10, "span 13: the area and its note");
    }
    const std::vector<BridgeMatch> m = MatchBridges(all, spans, 60.0, 5.0);
    // Span 11 on its chart: 4 m east of way 2, which runs north from 55 m south of it.
    const double mx11 = 111320.0 * std::cos(42.8105 * kPi / 180.0);
    const double dx11 = (-70.868 - -70.86795) * mx11, south11 = (42.8105 - 42.810) * 110574.0;
    const double want11 = south11 + std::sqrt(60.0 * 60.0 - dx11 * dx11);
    const double want13 = 0.0006 * 111320.0 * std::cos(42.82 * kPi / 180.0);
    for (const BridgeMatch& b : m) {
        Log("[roads]   pair way %lld span %lld overlap %.4f m", static_cast<long long>(all[b.way].id),
            static_cast<long long>(spans[b.span].id), b.overlapM);
    }
    check(m.size() == 3, "three pairs at 60 m: span 11 once, span 13 twice, span 12 none");
    if (m.size() == 3) {
        check(m[0].way == 1 && m[0].span == 0 && std::abs(m[0].overlapM - want11) < 1e-6,
              "span 11 (a point) pairs with bridge way 2 alone, overlap = its length within 60 m");
        check(m[1].span == 1 && m[2].span == 1 && m[1].way == 2 && m[2].way == 3,
              "span 13 (an area) pairs with both ways on its deck, the primary before the cycleway");
        check(std::abs(m[1].overlapM - want13) < 1e-6 && m[1].overlapM == m[2].overlapM,
              "equal overlaps (each way's whole length inside the deck): rank decides the order");
    }
    bool none11 = true;
    for (const BridgeMatch& b : MatchBridges(all, spans, 2.0, 5.0)) none11 &= b.span != 0;
    check(none11, "span 11 pairs with nothing at point reach 2 m (it is 4 m off)");
    {
        // Half the deck: way 4 cut to the area's west half still pairs, with half the overlap at 0 m.
        std::vector<RoadWay> half(all.begin() + 2, all.begin() + 3);
        half[0].points[1].lon = -70.8790 + 0.0004;   // runs 0.0002 deg past the east edge
        std::vector<BridgeSpan> deck(spans.begin() + 1, spans.begin() + 2);
        const auto h = MatchBridges(half, deck, 60.0, 0.0);
        const double wantIn = 0.0008 * 111320.0 * std::cos(42.82 * kPi / 180.0);   // -70.8798 .. -70.8790
        check(h.size() == 1 && std::abs(h[0].overlapM - wantIn) < 1e-6, "an area clips a way at its edge (shape reach 0)");
    }
    if (all.size() == 5) {
        // A charted line 4 m north of way 4, along its whole length: inside the shape reach of 5 m,
        // outside 2 m, whatever the point reach.
        std::vector<RoadWay> one(all.begin() + 2, all.begin() + 3);
        BridgeSpan line;
        line.primitive = 1;
        const double dLat = 4.0 / 110574.0;
        line.parts.push_back({-70.8798, 42.8202 + dLat, -70.8792, 42.8202 + dLat});
        line.outer.push_back(1);
        const auto at5 = MatchBridges(one, {line}, 60.0, 5.0);
        const auto at2 = MatchBridges(one, {line}, 60.0, 2.0);
        const double wantLine = 0.0006 * 111320.0 * std::cos((42.8202 + dLat) * kPi / 180.0);
        check(at5.size() == 1 && std::abs(at5[0].overlapM - wantLine) < 1e-6 && at2.empty(),
              "a line span 4 m off a bridge way pairs at shape reach 5 m, not at 2 m");
    }

    // A refusal: a manifest of the wrong format, and a manifest whose .bin is missing.
    std::ofstream(dir + "/bad.roads.json", std::ios::binary)
        << "{\"format\": \"GABLDG01\", \"cellDeg\": 0.05, \"cellIndex\": \"t.roads.idx\"}";
    std::vector<RoadWay> none;
    why.clear();
    check(!LoadRoadSource(dir + "/bad.roads.json", -71, 42, -70, 43, none, &why) && none.empty() &&
              why.find("not a GAROAD01 manifest") != std::string::npos,
          "a manifest of another format is refused with its reason");
    std::ofstream(dir + "/nobin.roads.json", std::ios::binary)
        << "{\"format\": \"GAROAD01\", \"cellDeg\": 0.05, \"cellIndex\": \"t.roads.idx\"}";
    why.clear();
    check(!LoadRoadSource(dir + "/nobin.roads.json", -71, 42, -70, 43, none, &why) &&
              why.find("no GAROAD01 data beside the manifest") != std::string::npos,
          "a manifest with no .bin beside it is refused");
    Log("[roads] selftest %s", ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
