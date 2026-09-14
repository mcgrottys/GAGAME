// ================================================================================================
//  Effect - M12 step 5e: THE "PAPER VISUAL" SLOT. A component that declares what it READS and
//  what it WRITES as typed field ports, and registers its edges in the GA AST at boot.
//
//  Every feature arrives as an edge of the state diagram (core/GaAst.h: "the AST is how
//  features arrive now"), and the M7o slice plane was the first one registered as a demo node.
//  An Effect says the same thing as a contract: Inputs() and Outputs() are FieldPorts -- a
//  name, the grade signature of the fiber (a bound on the grades it may carry, the compositor's
//  convention), its unit and the space it is expressed in -- and RegisterEdges() registers the
//  edge(s) that connect them, so the AST's flip and orphan rules validate the effect at boot
//  with every other edge. The first is effects/SlicePlane.h (today's --slice); a later one
//  declares its ports the same way and the diagram, the validator and docs/GA_AST.md carry it
//  without a hand table.
//
//  Prior art, named: Unity's ScriptableRenderPass / Unreal's post-process material (a render
//  effect as an object with declared inputs), Frostbite's frame graph (Halen & O'Donnell 2017:
//  passes declare the resources they read and write and the graph validates the wiring), and
//  USD's typed schemas for the port declarations. The AST registration is this engine's own.
// ================================================================================================
#pragma once

#include "scene/Component.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ga::scene {

// One typed port of an effect: what crosses in or out.
struct FieldPort {
    std::string name;       // "user.plane", "slice"
    uint32_t gradeSig = 0;  // the fiber's grade bound, bit k = grade k (a plane in PGA: 0b10)
    std::string unit;       // "signed distance m", "keep s<=0"
    std::string space;      // the frame it is expressed in: "world.m", "globe.ps"
};

class Effect : public Component {
public:
    virtual std::vector<FieldPort> Inputs() const = 0;
    virtual std::vector<FieldPort> Outputs() const = 0;
    // ast::Register the edge(s) between them -- at boot, before ast::Validate, so the flip
    // and orphan rules see the effect exactly as they see the water chain.
    virtual void RegisterEdges() = 0;
};

using EffectRegistry = Registry<std::unique_ptr<Effect>>;

}  // namespace ga::scene
