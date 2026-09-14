// ================================================================================================
//  SlicePlane - M12 step 5e: THE FIRST EFFECT -- M7o's cutaway plane (today's --slice) as a node.
//
//  One blade, one inner product, one discard: Globe.hlsl keeps the pixels with s <= 0 against
//  the plane z = d (gatest: the sandwich negates s; proofs/slice_plane.py). The Assembly used
//  to write the globe's sliceOn/sliceD from the flags and register the edge by hand; the node
//  owns both now -- Apply feeds the globe, RegisterEdges registers exactly the row the hand
//  table registered (same node names, frames, units, range and anchor, so docs/GA_AST.md is
//  unchanged) -- and --slice reaches it through the shim as effects[slice.plane].
// ================================================================================================
#pragma once

#include "scene/Effect.h"
#include "scene/SceneSchema.h"

namespace ga {
class GlobeLayer;
}

namespace ga::scene {

class SlicePlane final : public Effect {
public:
    struct Observers {
        GlobeLayer* globe = nullptr;   // the consumer: Globe.hlsl's discard
    };

    // ---- Component -------------------------------------------------------------------------
    const char* Name() const override { return m_name.c_str(); }
    const Schema& Props() const override { return SlicePlaneSchema(); }
    std::vector<std::string> Configure(const Wiring& w) override;
    bool Init(Gpu& gpu) override;
    void Apply(const PropSet& props) override;
    void Update(const FrameInfo& f) override;
    void Record(const ViewContext& v) override;
    void ReloadShaders() override {}

    // ---- Effect ----------------------------------------------------------------------------
    std::vector<FieldPort> Inputs() const override;
    std::vector<FieldPort> Outputs() const override;
    void RegisterEdges() override;

    // ---- the wiring and the declaration ------------------------------------------------------
    std::vector<std::string> Configure(const Observers& o);
    // The boot's read of the `effects` section: the name, whether it is on, the offset.
    void Declare(const std::string& name, bool on, double d);
    const SlicePlaneProps& Declared() const { return m_props; }
    // THE FAN-OUT, idempotent: the globe's sliceOn / sliceD from the declaration.
    void FanOut();

private:
    std::string m_name = "slice";
    SlicePlaneProps m_props;
    Observers m_o;
};

}  // namespace ga::scene
