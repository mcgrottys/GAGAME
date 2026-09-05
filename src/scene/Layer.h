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

#include "core/Gpu.h"
#include "core/Shader.h"

namespace ga {

class FieldSet;
class Camera;
class GpuProfiler;

struct FrameContext {
    Gpu* gpu = nullptr;
    ID3D12GraphicsCommandList* cl = nullptr;
    const Camera* camera = nullptr;
    D3D12_GPU_VIRTUAL_ADDRESS sceneCb = 0;   // b0, already filled for this frame
    float timeSec = 0;
    uint32_t width = 0, height = 0;
    // --gpu-time: timestamp pairs around a layer's internal sub-passes. Null (the default) means
    // no queries are issued anywhere; wrap with GpuScope, which is a no-op on null.
    GpuProfiler* prof = nullptr;
};

class Layer {
public:
    virtual ~Layer() = default;

    virtual const char* Name() const = 0;
    virtual void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                      ID3D12RootSignature* rootSig) = 0;
    virtual void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) { (void)gpu; (void)sc; }
    virtual void Render(const FrameContext& ctx) = 0;

    bool enabled = true;
};

}  // namespace ga
