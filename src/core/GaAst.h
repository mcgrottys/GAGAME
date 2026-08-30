// ================================================================================================
//  GaAst.h - M7j: THE GA AST -- the state diagram, printable and self-checking.
//
//  The user's architecture has always been a state diagram: nodes are GA engines, edges carry
//  geometric products. This file makes the EDGES first-class data. Every place one engine's
//  field crosses into another declares its edge here at wiring time, with the LIVE values:
//  which space each side speaks (world metres, atlas texels, raster rows, wrap uv), which way
//  its second axis points (the +v = north question that has bitten this project repeatedly),
//  the origin, the metres-per-unit scale, the units and expected RANGE of the value fiber, any
//  gain folded in on the way, and whether the sampling code applies a v-flip.
//
//  Every run PRINTS the diagram (the workflow as an AST -- domains, units, scales, ranges, in
//  one table that never goes out of date because it is built from the wiring itself), and
//  VALIDATES it:
//    * the FLIP RULE: an edge whose two frames disagree about +v must declare a flip, and one
//      whose frames agree must not -- the class of bug the orientation ledger was written for;
//    * the ORPHAN RULE: a producer whose consumers are all inactive is flagged -- this exact
//      rule would have caught the swell-shadow field feeding only the retired SeaLayer draw
//      path while one-water rendered fetch-blind foam into the sheltered basin.
//  The gatest gate runs the same validation plus the GA product identities, so a frame
//  mismatch is a FAILED BUILD GATE, not a debugging session three sessions later.
// ================================================================================================
#pragma once

#include <string>
#include <vector>

namespace ga::ast {

struct Frame {
    const char* space;   // "world.m" | "raster.row0N" | "atlas.texel" | "uv01" | "patch.wrap"
    bool vNorth;         // does the second axis grow NORTHWARD? (rows stored north-down: false)
    double orgX = 0.0, orgY = 0.0;   // world metres of sample (0,0), when meaningful
    double metersPerUnit = 0.0;      // texel/px pitch in metres, 0 = n/a (wrap/uv)
};

struct Edge {
    const char* from;    // producing node ("swe.solver", "ocean.fft", "churn.kernel", ...)
    const char* to;      // consuming node ("water.bank", "sea.ps", "globe.ps", ...)
    const char* field;   // "eta", "cascade.disp", "churn", "shadow", ...
    Frame src, dst;
    bool flip;           // does the SAMPLING CODE apply a v-flip on this edge?
    const char* units;   // "m NAVD", "m/s", "0..1 mask", "slope", ...
    const char* range;   // expected value range, human-readable ("+-3", "0..1")
    double gain = 1.0;   // scalar folded in on the way (churn foam gain, sweG.x, hsScale law)
    const char* code;    // anchor of the sampling site ("WaterBank.hlsl CsBankFill")
    bool active = true;  // is the consumer in the CURRENT mode's render path?
};

// Registration (idempotent per field+from+to: re-registering updates in place, so per-frame
// wiring sites are safe). Values should be the LIVE ones -- the print is truth, not doc.
void Register(const Edge& e);
void SetActive(const char* to, bool active);   // a whole node leaves/enters the render path

// The per-run print: the state diagram as an indented table, [gaast] prefixed.
void Print();

// The rules. Returns false (and logs each failure) on any violation.
bool Validate();

// gatest support: lookup for ground-truth assertions.
const std::vector<Edge>& Edges();

// The canonical water-chain table (conventions only; live wiring refines orgs in place).
// Called by gatest so --selftest validates the contract even before any scene exists, and
// by main at boot so the printed diagram always carries every known edge.
void RegisterKnownWaterEdges();
void RegisterKnownComposeEdges();   // the compositor + residency pillars (M7l)

}  // namespace ga::ast

namespace ga {
bool RunGaSelfTest();   // the gatest gate (GaTest.cpp)
}
