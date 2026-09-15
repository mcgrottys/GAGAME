// ================================================================================================
//  Layer - a PRODUCT: one thing you can stack into the scene (the tide surface, a curve overlay,
//  later the FFT ocean, the estuary solver, an eddy overlay...).
//
//  The contract is deliberately narrow so that adding the fifth product is exactly as cheap as
//  adding the second (proven in vqview, where the vessel layer touched no existing layer):
//    * A layer owns its own PSOs and its own geometry.
//    * It does NOT own the root signature, the render targets, the camera, or the frame ring.
//    * It reads data only through FieldSet indices, never by holding a texture directly, so the
//      tiled-resource residency manager (M4) can move data underneath it without the layer
//      noticing.
//    * ReloadShaders() must leave the layer usable if compilation fails: print the DXC error,
//      keep drawing with the old PSO.
//
//  Ordering: the Renderer draws layers in registration order.
// ================================================================================================
#pragma once

#include "hal/Context.h"
#include "hal/Gpu.h"
#include "hal/Shader.h"

namespace ga {

class FieldSet;
class Camera;
class GpuProfiler;

struct FrameContext {
    Gpu* gpu = nullptr;
    // M12 3b: the frame's recording, through the facade. Native() is the raw list, and every
    // use of it in a layer is a call site the next sub-steps look at.
    hal::CommandContext* cmd = nullptr;
    const Camera* camera = nullptr;
    D3D12_GPU_VIRTUAL_ADDRESS sceneCb = 0;   // b0, already filled for this frame
    float timeSec = 0;
    uint32_t width = 0, height = 0;
    // --gpu-time: timestamp pairs around a layer's internal sub-passes. Null (the default) means
    // no queries are issued anywhere; wrap with GpuScope, which is a no-op on null.
    GpuProfiler* prof = nullptr;
    // M12 step 5b: WHICH VIEW of the frame's set is recording (scene/ViewContext.h). The
    // renderer walks the views in file order and writes the index here; 0 is the session's own
    // and today the only one. A layer that keys per-view state keys it by THIS index --
    // m_perView[ctx.viewIndex] -- instead of growing a second hard-coded copy. The M10 outer
    // Droste level's water bank (Assembly::waterBankB, "set B") is the precedent that priced
    // it, and it is deliberately NOT re-keyed here: set B stays set B until a scene declares a
    // second view, so this step cannot move a pixel through it.
    uint32_t viewIndex = 0;
};

class Layer {
public:
    virtual ~Layer() = default;

    virtual const char* Name() const = 0;
    virtual void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                      hal::RootSignature rootSig) = 0;
    virtual void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) { (void)gpu; (void)sc; }
    // ONCE A FRAME, before any view records, for every DECLARED layer whatever its per-frame
    // `enabled` says (the water match, step 3). `enabled` is a VIEW's decision -- the mode, the
    // altitude bands -- and it may only decide what is DRAWN. State other consumers read (a solver's
    // step, the regions it serves a hull) advances here, for whoever asked, so a camera in orbit
    // cannot freeze the water a boat is floating on. The context is the frame's first view's.
    virtual void Simulate(const FrameContext& ctx) { (void)ctx; }
    virtual void Render(const FrameContext& ctx) = 0;

    bool enabled = true;
    // M12 step 5d: WHAT THE SCENE'S `layers` LIST SAYS. `enabled` is the per-frame gate the mode
    // and the altitude bands write every frame (FrameLoop's applyMode); this is the STANDING
    // declaration -- a layer the scene file does not carry, or carries `"enabled": false`, is
    // registered (the assembly's construction order is the lifetime law and does not move) and
    // never drawn. Set once, at the end of Assemble(), from the list; nothing else writes it.
    bool declared = true;
};

}  // namespace ga
