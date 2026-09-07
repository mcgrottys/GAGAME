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
//      b2            shared surface constants (bound by whichever layer owns the surface)
//      s0..s2        static samplers        (linear clamp, linear wrap, point clamp)
//  A new product adds textures to the heap and rows to the FieldDesc table. It does not touch this.
// ================================================================================================
#pragma once

#include "core/Gpu.h"
#include "core/GpuProfiler.h"
#include "core/Shader.h"
#include "render/Camera.h"
#include "scene/Layer.h"

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
};
static_assert(sizeof(SceneConstants) % 16 == 0, "SceneConstants must be 16-byte aligned");

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
    void RenderFrame(const Camera& cam, float timeSec, float dt);

    // Reads the tonemapped LDR result back and writes a PNG. Headless-safe.
    bool DumpPng(const std::wstring& path);
    // Raw RGBA8 + its row pitch (which may exceed width*4 -- D3D readback alignment).
    bool DumpRaw(std::vector<uint8_t>& out, uint32_t* rowPitch);
    // --dump-hdr: the RGBA16F scene radiance BEFORE the tonemap, raw de-pitched rows plus a
    // .json sidecar (size, exposure, the shoulder knee and gamma Tonemap.hlsl applies), so a
    // sub-LSB claim is judged in radiance through the curve (tools/imgdiff.py --hdr) rather
    // than in the 8-bit image that already rounded it.
    bool DumpHdr(const std::wstring& path);
    uint32_t Width() const { return m_width; }
    uint32_t Height() const { return m_height; }

    ShaderCompiler& Shaders() { return m_shaders; }
    ID3D12RootSignature* RootSignature() const { return m_rootSig.Get(); }

    // --gpu-time: timestamp pairs around every pass. Off (null) by default -- no queries issued.
    void EnableGpuProfiler();
    GpuProfiler* Profiler() { return m_prof.get(); }
    // The caller's frame number for the profiler's rows (a rail's settle frames are negative).
    int64_t gpuFrameLabel = 0;

    // Water level in metres above datum -- in M1 this is the tide at the focus station, published
    // scene-wide because anything that sits in or on the water will need it.
    float waterLevel = 0.0f;
    // M9bi: THE SUN IS A PLACE. When sunPlaced is set, the direction below comes from
    // Ephemeris.h's conformal chain -- the real sun for the scene's own timestamp and latitude,
    // already rotated into this frame -- and the azimuth/elevation art direction is ignored.
    // `--sun az,el` clears the flag and pins the old constants back, which is what every
    // recorded baseline before M9bi was lit by.
    bool sunPlaced = false;
    float sunDirTangent[3] = {0.0f, 1.0f, 0.0f};
    // The sun's own angular RADIUS at the current Earth-Sun distance (0.2621..0.2710 deg over a
    // year). A direction cannot have one; only something at a distance can, and the disc in the
    // sky is drawn from it rather than from the two hand-picked cosines that were there before.
    float sunAngRadiusDeg = 0.26656f;
    float sunAzimuthDeg = 112.0f;
    float sunElevationDeg = 26.0f;
    // Turbid coastal water (vqview's calibrated Merrimack optics, kept as the default palette).
    float sigmaW[3] = {0.330f, 0.1238f, 0.1463f};
    float bscat[3] = {0.0173f, 0.0233f, 0.0248f};

private:
    void CreateRootSignature();
    void CreateTargets(uint32_t width, uint32_t height);
    void CreateTonemapPso();

    Gpu* m_gpu = nullptr;
    RendererDesc m_desc;
    ShaderCompiler m_shaders;

    Com<ID3D12RootSignature> m_rootSig;
    Com<ID3D12PipelineState> m_tonemapPso;

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
