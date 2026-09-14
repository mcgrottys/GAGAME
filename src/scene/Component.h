// ================================================================================================
//  Component - M12 step 5a: THE PLUGIN UNIT. Pure-virtual, house style, and no Direct3D type in
//  any signature: this header is the surface a DLL will see, and tools/hal_lint.py holds it.
//
//  The verbs are the lifecycle every Layer already has (scene/Layer.h: Name / Init /
//  ReloadShaders / Render), with the per-frame setters the layers grew by hand -- SetView,
//  SetSun, SetDroste, SetTime, SetFrame, one bespoke fan-out per layer in main -- replaced by
//  two typed calls:
//
//      Configure(const Wiring&)   the house Configure(const T*) idiom widened ONCE: a plain
//                                 struct of nullable observers; what a component needs and does
//                                 not get is REPORTED (Wiring::Missing), never guessed
//      Init(Gpu&)                 the device work, after Configure
//      Apply(const PropSet&)      IDEMPOTENT. Replaces the hand fan-out; called on load and on
//                                 every hot-reload -- one path, so the boot and the reload can
//                                 never apply different sets (the M8 defect the plan names)
//      Update(const FrameInfo&)   the clock -- camera-free, once a frame
//      Record(const ViewContext&) one view's recording, through hal::CommandContext (a pointer
//                                 to a hal type is a hal spelling: the lint's rule)
//      ReloadShaders()            the swap-on-success law, per component
//
//  FrameInfo carries the physics snapshot's `asOf` beside the clock -- the freshness contract
//  ARCHITECTURE.md records (a consumer receives a coherent snapshot with a declared simulation
//  time and a fallback policy) -- and the clock's whole quanta; Entity::Update (step 5e) is
//  their first consumer (scene/Entity.h states the contract and its default).
//
//  LayerComponent (scene/LayerComponent.h) adapts an existing Layer UNCHANGED, so the nine
//  layers are not edited in the seam steps; their bespoke setters are called from the owning
//  component's Update/Apply. ComponentRegistry is the factory (core/Registry.h) a scene file's
//  `nodes[].type` is looked up in; the in-tree registration is a function per module.
//
//  Prior art, named: Unity's MonoBehaviour and Godot's Node lifecycle (ready / process / draw
//  as fixed verbs a scene file instances by type), USD's typed schemas (the property table is
//  part of the type), and the Template Method of Gamma et al. -- the order of the verbs is the
//  engine's and a component fills the steps.
// ================================================================================================
#pragma once

#include "core/Registry.h"
#include "scene/Props.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ga {
class Camera;
class FieldSet;
class Gpu;
class GpuProfiler;
class ShaderCompiler;
namespace hal {
class CommandContext;
}
}  // namespace ga

namespace ga::scene {

// The observers a component may be wired to. Nullable: a component that needs one it did not
// get lists it in Missing() and Init refuses (reported, not guessed).
struct Wiring {
    Gpu* gpu = nullptr;
    ShaderCompiler* shaders = nullptr;
    FieldSet* fields = nullptr;
    const std::wstring* shaderDir = nullptr;
    // The renderer's shared root signature, carried opaquely (hal::RootSignature is the
    // renderer's business; a plugin never spells it).
    void* rootSignature = nullptr;
};

struct FrameInfo {
    double simUnix = 0.0;   // the scene clock, UTC seconds
    double dt = 0.0;        // this frame's advance, seconds (0 when paused or held)
    uint32_t frame = 0;     // the frame index
    double asOf = 0.0;      // the physics snapshot's declared simulation time (the freshness contract)
    // M12 step 5e: the scene clock's WHOLE quanta this frame (sim/SimClock.h, 240 Hz): what an
    // integrating consumer steps, so a hull steps the same quanta from the windowed clock and
    // the frame-indexed headless one (Entity::Update).
    int quanta = 0;
};

// One view's recording. DECLARED here and defined in scene/ViewContext.h, where step 5b gave
// it the renderer's own rows (the SceneConstants this view records, its target and viewport,
// and the legacy FrameContext a Layer draws from): this header stays the plugin surface, so
// only a component that actually DRAWS pays for the definition.
struct ViewContext;

class Component {
public:
    virtual ~Component() = default;

    virtual const char* Name() const = 0;
    virtual const Schema& Props() const = 0;
    // What was missing at Configure, empty when everything needed was wired.
    virtual std::vector<std::string> Configure(const Wiring& w) = 0;
    virtual bool Init(Gpu& gpu) = 0;
    virtual void Apply(const PropSet& props) = 0;
    virtual void Update(const FrameInfo& f) = 0;
    virtual void Record(const ViewContext& v) = 0;
    virtual void ReloadShaders() = 0;

    bool enabled = true;
};

using ComponentRegistry = Registry<std::unique_ptr<Component>>;

}  // namespace ga::scene
