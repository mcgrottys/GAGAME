// ================================================================================================
//  VesselLayer - M9bq: draws whatever vessels the sim is stepping, from their specs.
//
//  Built on MarkerLayer's shape (the smallest layer in the engine) and it keeps that contract:
//  owns its PSO and nothing else, reads no texture directly, survives a failed shader reload with
//  the old PSO. What it adds is that its GEOMETRY IS DERIVED FROM THE SPEC rather than authored --
//  see shaders/Vessel.hlsl for why that is the point and not a shortcut.
//
//  Per-frame data goes through Gpu::PushConstants, NOT Exchange::Publish. The Exchange is a
//  double-buffered publish that waits for the GPU to drain (compose/Exchange.h:22); a hull's pose
//  changes every single frame and draining the pipe for it would cost more than the boat.
//
//  Registered AFTER the globe so water and terrain occlude it correctly -- the Renderer draws
//  layers in registration order.
// ================================================================================================
#pragma once

#include "core/Pga.h"
#include "scene/Layer.h"

#include <string>
#include <vector>

namespace ga {

class Vessel;

class VesselLayer : public Layer {
public:
    void Configure(const std::wstring& shaderDir) { m_shaderDir = shaderDir; }

    const char* Name() const override { return "vessels"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              hal::RootSignature rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;

    // Per frame, before RenderFrame: rebuild the part list from the vessels the sim is stepping.
    // Copied rather than referenced -- the physics tick and the frame do not share a clock.
    void SetVessels(const Vessel* const* vessels, int count);
    // ...each in its own space: frames[i] is that space's placement in the root's frame (a hull
    // carried through a gate). A null array is the root for every hull, byte for byte.
    void SetVessels(const Vessel* const* vessels, const Motor* frames, int count);

    uint32_t PartCount() const { return static_cast<uint32_t>(m_cpu.size()); }

    // Mirrors `struct VesselPart` in shaders/Vessel.hlsl. Float, because it is display data that
    // has already been made camera-relative in the shader; the DOUBLE pose lives in RigidBody and
    // is never rounded before the sandwich. Public so the spec-walking helper can build one.
    struct PartGpu {
        float re[4];
        float du[4];
        float half[4];   // xyz half extents, w = palette index (the element kind)
    };
    // A box as the CPU keeps it: its WORLD motor in doubles. It becomes a PartGpu only at Render,
    // RELATIVE TO THE EYE -- the eye's reverse translation composed in doubles, the float cast
    // after it. A world motor cast to float is exact near the Merrimack and wrong far from it:
    // 2054 km out (a hull carried through a gate) float's step is 0.25 m, every box and corner
    // rounds on its own, and the RHIB drew twisted.
    struct PartCpu {
        Motor world;
        float half[4];
    };

private:
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);

    std::wstring m_shaderDir;
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_pso;
    std::vector<PartCpu> m_cpu;
    std::vector<PartGpu> m_parts;   // built from m_cpu at Render, relative to that view's eye
};

}  // namespace ga
