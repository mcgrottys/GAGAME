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
    // M7n: the CONVENTION -- are samples at cell CENTERS ((i+0.5)*pitch, the norm here and
    // what hardware samplers assume) or at lattice corners (i*pitch)? A mismatch is a
    // half-lattice TRANSLATION the flip rule cannot see: the coincidence card caught the
    // bank kernel writing corners while BankSample reconstructed centers -- every bank
    // field sat half a texel off (77 m at ring 5). Now it is a boot-checked rule.
    bool centers = true;
};

struct Edge {
    const char* from;    // producing node ("swe.solver", "ocean.fft", "churn.kernel", ...)
    const char* to;      // consuming node ("water.bank", "sea.ps", "globe.ps", ...)
    const char* field;   // "eta", "cascade.disp", "churn", "shadow", ...
    Frame src, dst;
    bool flip;           // does the SAMPLING CODE apply a v-flip on this edge?
    const char* units;   // "m NAVD", "m/s", "0..1 mask", "slope", ...
    // M12 step 4c: the two DESCRIPTIONS are owned text. An edge registered from a declaration
    // spells them at run time (a lattice's Tag, a tenant's slice), and Register copies the
    // Edge, so the edge holds its words rather than pointing into a caller's buffer.
    std::string range;   // expected value range, human-readable ("+-3", "0..1")
    double gain = 1.0;   // scalar folded in on the way (churn foam gain, sweG.x, hsScale law)
    std::string code;    // anchor of the sampling site ("WaterBank.hlsl CsBankFill")
    bool active = true;  // is the consumer in the CURRENT mode's render path?
};

// THE FLIP RULE as the one function both its callers use: the validator (Validate) and a
// registration that DERIVES its flag from two frames (SurfaceFrame::RegisterEdges). It is
// GeoRef.h's NeedsFlipInto said for frames: a flip exactly when the two +v's disagree.
inline bool NeedsFlip(const Frame& src, const Frame& dst) { return src.vNorth != dst.vNorth; }

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
void RegisterKnownComposeEdges();   // the compositor + residency pillars (M7l); the paint
                                    // rows come from the declarations (SurfaceFrame::RegisterEdges)
// M9bh: the edges --pixel-water restores (the two rays' consumers: the imagery bed, the
// height quadtree the refracted cast traces, the cascade slope fibers, the wind's far-field
// sigma^2 and the ocean-colour retrievals). Registered ONLY when the flag is on, so the
// printed diagram describes the path this run actually walks.
void RegisterPixelWaterEdges();
// M9bi: the SUN's edges -- the clock producing a place, and the place producing the one
// direction every shading layer reads. Registered when the ephemeris is driving (--sun pins the
// old art direction and takes them out with it).
void RegisterSolarEdges();
// M7m: persist the diagram as markdown (docs/GA_AST.md) so the scriptorium indexes it --
// a future session QUERIES the edge table instead of rereading shaders out of context.
void WriteMarkdown(const char* path);
// M7u: the graph as MACHINE-READABLE JSON (docs/ga_ast.json) -- nodes + edges with
// frames, units, ranges, gains, anchors. This file is the contract a future node-editor
// UI (the Blueprint ambition) loads and saves, and what tools/astdiagram.py renders to
// the SVG catalog in docs/diagrams/.
void WriteJson(const char* path);

}  // namespace ga::ast

namespace ga {
bool RunGaSelfTest();   // the gatest gate (GaTest.cpp)
}
