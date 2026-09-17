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
    // ...and with `through[i]` non-zero, hull i is drawn THROUGH a gate's window: its frame is
    // already the apparent one (pulled back by the gate's motor) and its pixels survive only
    // where the window's slab test says the eye reaches them through the box. The complementary
    // half matters just as much: a hull on THIS side is discarded where the window shows the
    // other place, or a boat standing behind the portal would be visible through it.
    void SetVessels(const Vessel* const* vessels, const Motor* frames, const uint8_t* through,
                    int count);
    // The gate's box in the true camera frame -- GlobeLayer::SetGate's own rows, half extents and
    // eye-relative centre. `on` false is the shipped pass: no test, no window, byte for byte.
    // ...and `sun` is the one light as seen from the far place, in this frame: a hull that has
    // gone through is lit from where it is.
    void SetGateWindow(const float rows[9], const float half[3], const float centre[3],
                       const float sun[3], bool on) {
        m_winOn = on;
        if (!on) return;
        for (int i = 0; i < 9; ++i) m_winRows[i] = rows[i];
        for (int i = 0; i < 3; ++i) {
            m_winHalf[i] = half[i];
            m_winC[i] = centre[i];
            m_winSun[i] = sun[i];
        }
    }

    uint32_t PartCount() const { return static_cast<uint32_t>(m_cpu.size()); }
    // Whether a world point lies inside any drawn box, grown by `margin` metres -- the water
    // probe's test for a depth sample that landed on a hull rather than on the sea.
    bool Occupies(double x, double y, double z, double margin) const;

    // Mirrors `struct VesselPart` in shaders/Vessel.hlsl. Float, because it is display data that
    // has already been made camera-relative in the shader; the DOUBLE pose lives in RigidBody and
    // is never rounded before the sandwich. Public so the spec-walking helper can build one.
    struct PartGpu {
        float re[4];
        float du[4];
        float half[4];   // xyz half extents, w = palette index (the element kind)
        // M13: x = 1 when this part is seen THROUGH a gate's window (it belongs to the other
        // place), 0 when it stands in the eye's own. Appended at the END on both sides -- the
        // shader's struct is the mirror, and priors 22 is about exactly this.
        float opt[4];
    };
    // A box as the CPU keeps it: its WORLD motor in doubles. It becomes a PartGpu only at Render,
    // RELATIVE TO THE EYE -- the eye's reverse translation composed in doubles, the float cast
    // after it. A world motor cast to float is exact near the Merrimack and wrong far from it:
    // 2054 km out (a hull carried through a gate) float's step is 0.25 m, every box and corner
    // rounds on its own, and the RHIB drew twisted.
    struct PartCpu {
        Motor world;
        float half[4];
        float through = 0.0f;   // M13: drawn through a gate's window
    };

private:
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);

    std::wstring m_shaderDir;
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_pso;
    std::vector<PartCpu> m_cpu;
    std::vector<PartGpu> m_parts;   // built from m_cpu at Render, relative to that view's eye
    bool m_winOn = false;
    float m_winRows[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float m_winHalf[3] = {0.0f, 0.0f, 0.0f};
    float m_winC[3] = {0.0f, 0.0f, 0.0f};
    float m_winSun[3] = {0.0f, 1.0f, 0.0f};
};

}  // namespace ga
