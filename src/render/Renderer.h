// ================================================================================================
//  Renderer - owns the targets, the shared root signature, and the ordered layer list.
//  Carried from vqview-inlet; the target-chain rationale (HDR offscreen -> tonemap -> backbuffer,
//  depth readable as SRV, headless dump path) is unchanged and still load-bearing.
//
//  THE ROOT SIGNATURE IS SHARED BY EVERY LAYER AND IS NOT EXPECTED TO CHANGE
//      b0            scene constants        (root CBV)
//      b1            per-draw constants     (root CBV)
//      t0, space0    FieldDesc table        (root SRV, structured buffer)
//      t0, space1    unbounded Texture2D[]  (descriptor table over the whole SRV heap)
//      b2            surface constants      (root CBV: ComposedSurfaceCb, filled once a frame by
//                                             the frame loop, pushed and bound once by RenderFrame
//                                             for every layer -- M12 step 4g)
//      b3            globe sky constants    (root CBV: GlobeSkyCb, the globe's sky and limb passes)
//      s0..s3        static samplers        (linear clamp, linear wrap, point clamp, anisotropic)
//  A new product adds textures to the heap and rows to the FieldDesc table. It does not touch this.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"   // ComposedSurfaceCb: the surface constants on b2 (M12 step 4g)
#include "hal/Gpu.h"
#include "hal/GpuProfiler.h"
#include "hal/Root.h"
#include "hal/Shader.h"
#include "render/Camera.h"
#include "scene/Layer.h"

#include <cmath>
#include <memory>
#include <vector>

namespace ga {

// Mirrored in shaders/Common.hlsli. If you change one, change the other; the static_assert below
// only catches size drift, not field reordering.
struct SceneConstants {
    float viewProj[16];
    float sunDir[4];        // xyz unit toward the sun, w unused
    float sigmaW[4];        // per-channel water extinction 1/m
    float bscat[4];         // per-channel backscatter 1/m
    float params0[4];       // time s (window-relative), heightScale, patchWidth m, patchHeight m
    float params1[4];       // depthScale, aspect, nearZ, exposure
    float eyeRelWorld[4];   // camera position in scene-local metres
    // Camera basis, pre-scaled by the projection extents, so a fullscreen pass can rebuild the
    // world-space view ray as camFwd + ndc.x * camRight + ndc.y * camUp without inverting a matrix.
    float camRight[4];
    float camUp[4];
    float camFwd[4];
    float viewport[4];      // w, h, 1/w, 1/h
    float misc[4];          // x water level (m above datum -- the TIDE, in M1), yz = the SUN's
                            // disc (cos of its outer/inner angular radius, M9bi), w spare
    // M13: THE SKY, AS A TABLE. x = the sky-view LUT's heap slot (UINT32_MAX = none, and every
    // consumer falls back to the two shipped constants), y = the planet's radius and z = the
    // eye's own (both metres: the table was built for that altitude and is read at it), w spare.
    // Appended at the END on both sides, and dxtest compares the rows by name (priors 22).
    float skyLut[4];
};
static_assert(sizeof(SceneConstants) % 16 == 0, "SceneConstants must be 16-byte aligned");

// M12 step 5b: THE INPUTS THE SCENE CONSTANTS ARE A PURE FUNCTION OF -- one view (a camera and
// the target it draws into) and the scene-wide lighting and water the renderer carries. Named
// so that the fill can be a FUNCTION instead of the first sixty lines of RenderFrame: a gate
// can build one of these without a device (dxtest holds the frozen old fill against the live
// one at the six recipe poses, byte for byte), and a second eye fills its own rows from its own
// camera without reaching into the renderer. Renderer::FillInputs writes every field from the
// renderer's own members; the initialisers below are inert.
struct SceneFill {
    const Camera* cam = nullptr;
    float timeSec = 0.0f;
    uint32_t width = 1, height = 1;
    float heightScale = 0.0f, patchWidthM = 0.0f, patchHeightM = 0.0f, exposure = 0.0f;
    bool sunPlaced = false;
    float sunDirTangent[3] = {0.0f, 0.0f, 0.0f};
    float sunAzimuthDeg = 0.0f, sunElevationDeg = 0.0f, sunAngRadiusDeg = 0.0f;
    float sigmaW[3] = {0.0f, 0.0f, 0.0f};
    float bscat[3] = {0.0f, 0.0f, 0.0f};
    float waterLevel = 0.0f;
    // M13: the AIR's table (multiple scattering -- the same for every ray on the planet) and the
    // eye's place in the air; the sky is marched from there.
    uint32_t skyMsSrv = 0xFFFFFFFFu;
    float planetRadiusM = 6371000.0f, eyeRadiusM = 6371000.0f;
};

namespace scene {
struct ViewSet;
}

struct RendererDesc {
    std::wstring shaderDir;
    float patchWidthM = 1000.0f;
    float patchHeightM = 1000.0f;
    float heightScale = 1.0f;
    float exposure = 1.0f;
};

class Renderer {
public:
    void Init(Gpu& gpu, const RendererDesc& desc);
    void Shutdown();

    // Must be called before the first RenderFrame. Root parameter 2 is a root SRV and an UNBOUND
    // root descriptor is undefined behaviour, not a no-op: in vqview, leaving it unset removed the
    // device outright, even though the tonemap shader never reads the table.
    void SetFieldTable(D3D12_GPU_VIRTUAL_ADDRESS va) { m_fieldTableVa = va; }

    void AddLayer(std::unique_ptr<Layer> layer);
    Layer* FindLayer(const char* name);
    const std::vector<std::unique_ptr<Layer>>& Layers() const { return m_layers; }

    void OnResize(uint32_t width, uint32_t height);
    void ReloadShaders();

    // Records and submits one frame. Presents when not headless.
    //
    // M12 step 5b, THE SEAM: the frame is a LIST of views, recorded in FILE ORDER into one
    // command list. The residency wants are additive within a frame, so N walks in view order
    // are the union of what the N views want, and the predicted-want hash folds call order --
    // a second view always walks AFTER the main one. With one view this records what it
    // recorded before the list existed.
    void RenderFrame(const scene::ViewSet& views);
    // The one-line forwarder for a caller that has a camera and one view.
    void RenderFrame(const Camera& cam, float timeSec, float dt);
    // That one view, built from a camera exactly as RenderFrame built its constants.
    scene::ViewSet OneView(const Camera& cam, float timeSec) const;
    // THE FILL, static so it can read nothing but its inputs -- the gate's other half.
    static void FillSceneConstants(const SceneFill& f, SceneConstants& out);
    // This renderer's own inputs for one view (its target size, optics, sun, water).
    SceneFill FillInputs(const Camera& cam, float timeSec) const;

    // Reads the tonemapped LDR result back and writes a PNG. Headless-safe.
    bool DumpPng(const std::wstring& path);
    // Raw RGBA8 + its row pitch (which may exceed width*4 -- D3D readback alignment).
    bool DumpRaw(std::vector<uint8_t>& out, uint32_t* rowPitch);
    // --dump-hdr: the RGBA16F scene radiance BEFORE the tonemap, raw de-pitched rows plus a
    // .json sidecar (size, exposure, the shoulder knee and gamma Tonemap.hlsl applies), so a
    // sub-LSB claim is judged in radiance through the curve (tools/imgdiff.py --hdr) rather
    // than in the 8-bit image that already rounded it.
    bool DumpHdr(const std::wstring& path);
    // --water-probe: the scene depth (D32, reversed Z: nearZ / viewZ, 0 where nothing was drawn),
    // de-pitched into width*height floats. An instrument: it waits for the GPU.
    bool ReadDepth(std::vector<float>& out);
    uint32_t Width() const { return m_width; }
    uint32_t Height() const { return m_height; }

    ShaderCompiler& Shaders() { return m_shaders; }
    hal::RootSignature RootSignature() const { return m_rootSig.Get(); }

    // --gpu-time: timestamp pairs around every pass. Off (null) by default -- no queries issued.
    void EnableGpuProfiler();
    GpuProfiler* Profiler() { return m_prof.get(); }
    // The caller's frame number for the profiler's rows (a rail's settle frames are negative).
    int64_t gpuFrameLabel = 0;

    // M12 step 4g: THE SURFACE CONSTANT BUFFER (b2). The frame loop fills these rows once a
    // frame through SurfaceFrame::Fill; RenderFrame pushes them once and binds root parameter 4
    // (b2) before any layer records, so the globe, the sea, the terrain and the GIS vectors
    // read one buffer where each carried a copy of these rows inside its own cbuffer. Zero
    // until filled: every composed channel reads "off".
    ComposedSurfaceCb surfaceCb{};
    // Water level in metres above datum -- in M1 this is the tide at the focus station, published
    // scene-wide because anything that sits in or on the water will need it.
    float waterLevel = 0.0f;
    // M9bi: THE SUN IS A PLACE. When sunPlaced is set, the direction below comes from
    // Ephemeris.h's conformal chain -- the real sun for the scene's own timestamp and latitude,
    // already rotated into this frame -- and the azimuth/elevation art direction is ignored.
    // `--sun az,el` clears the flag and pins the old constants back, which is what every
    // recorded baseline before M9bi was lit by.
    bool sunPlaced = false;
    // M13: what the sky layer built this frame, handed to every shader through the scene
    // constants (the dome, the sea's mirror and the haze must read ONE sky).
    uint32_t skyMsSrv = 0xFFFFFFFFu;
    float planetRadiusM = 6371000.0f, eyeRadiusM = 6371000.0f;
    float sunDirTangent[3] = {0.0f, 1.0f, 0.0f};
    // The sun's own angular RADIUS at the current Earth-Sun distance (0.2621..0.2710 deg over a
    // year). A direction cannot have one; only something at a distance can, and the disc in the
    // sky is drawn from it rather than from the two hand-picked cosines that were there before.
    float sunAngRadiusDeg = 0.26656f;
    float sunAzimuthDeg = 112.0f;
    float sunElevationDeg = 26.0f;
    // The unit sun direction this frame's scene constants will carry -- the placed sun, or the
    // pinned art direction. M10: the globe's level table takes it from here, so the two cannot
    // disagree about which sun the camera's own level is lit by.
    void SunDir(float out[3]) const {
        if (sunPlaced) {
            out[0] = sunDirTangent[0];
            out[1] = sunDirTangent[1];
            out[2] = sunDirTangent[2];
            return;
        }
        const float az = sunAzimuthDeg * 0.017453292519943295f;
        const float el = sunElevationDeg * 0.017453292519943295f;
        out[0] = std::cos(el) * std::sin(az);
        out[1] = std::sin(el);
        out[2] = std::cos(el) * std::cos(az);
    }
    // Turbid coastal water (vqview's calibrated Merrimack optics, kept as the default palette).
    float sigmaW[3] = {0.330f, 0.1238f, 0.1463f};
    float bscat[3] = {0.0173f, 0.0233f, 0.0248f};

private:
    // The table above as a hal::RootLayout -- the one graphics layout, built once by
    // CreateRootSignature and handed to every layer's Init.
    static hal::RootLayout SharedGraphicsLayout();
    void CreateRootSignature();
    void CreateTargets(uint32_t width, uint32_t height);
    void CreateTonemapPso();

    Gpu* m_gpu = nullptr;
    RendererDesc m_desc;
    ShaderCompiler m_shaders;

    hal::RootSignatureRef m_rootSig;
    hal::Pso m_tonemapPso;

    GpuTexture m_sceneColor;
    GpuTexture m_sceneDepth;
    GpuTexture m_ldrTarget;   // tonemap destination when headless; also the readback source
    uint32_t m_sceneColorRtv = UINT32_MAX;
    uint32_t m_sceneDepthDsv = UINT32_MAX;
    uint32_t m_sceneDepthSrv = UINT32_MAX;
    uint32_t m_ldrRtv = UINT32_MAX;

    std::vector<std::unique_ptr<Layer>> m_layers;
    D3D12_GPU_VIRTUAL_ADDRESS m_fieldTableVa = 0;
    uint32_t m_width = 0, m_height = 0;
    std::unique_ptr<GpuProfiler> m_prof;
};

}  // namespace ga
