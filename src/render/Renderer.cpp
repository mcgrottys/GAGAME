#include "render/Renderer.h"

#include "core/Image.h"
#include "hal/Context.h"
#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Views.h"
#include "scene/FieldSet.h"

#include <cmath>

using namespace DirectX;

namespace ga {

static constexpr DXGI_FORMAT kSceneColorFormat = DXGI_FORMAT_R16G16B16A16_FLOAT;
static constexpr DXGI_FORMAT kSceneDepthFormat = DXGI_FORMAT_D32_FLOAT;
static constexpr DXGI_FORMAT kLdrFormat = DXGI_FORMAT_R8G8B8A8_UNORM;

void Renderer::Init(Gpu& gpu, const RendererDesc& desc) {
    m_gpu = &gpu;
    m_desc = desc;
    m_shaders.Init();
    CreateRootSignature();
    CreateTargets(gpu.Width(), gpu.Height());
    CreateTonemapPso();
}

void Renderer::Shutdown() {
    m_layers.clear();
    m_prof.reset();
}

void Renderer::EnableGpuProfiler() {
    if (m_prof) return;
    m_prof = std::make_unique<GpuProfiler>();
    m_prof->Init(*m_gpu);
}

hal::RootLayout Renderer::SharedGraphicsLayout() {
    // t0, space1: one unbounded range covering the whole shader-visible heap. This is the piece
    // that makes textures addressable by uint index, so stacking products never edits this table.
    // t0, space2 (M6c): the SAME heap again, viewed as Texture3D -- both ranges start at table
    // offset 0, so any heap slot is addressable as whichever dimensionality its descriptor
    // really is (reading a 2D slot through gTex3D would be invalid; nothing does).
    // t0, space3 (M6e): the heap a third time, as TextureCube -- the streamed planet surfaces
    // (Mars's rescued pyramids, Google's Earth) and their residency-map cubes live there.
    // t0, space4 (M9h): the heap a fourth time, as Texture2D<uint> -- residency maps are
    // R8_UINT, and "the finest level resident here" is an INDEX. Read through the float view it
    // would come back as a normalized fraction, so a level-3 map would sample as 0.0118 and the
    // clamp would silently do nothing.
    // t0, space5 (M9j): the heap as Texture2DArray -- paged GA banks, whose slices are pages.
    // t0, space6 (M9ap): the heap as TextureCubeArray -- slices 0..5 of the colour PAGE
    // tenant viewed as a cube, so the globe keeps seamless cube filtering from an array.
    // (An unbounded range adds nothing to the table offset: Root.h's law, stated for this.)
    hal::RootLayout rl;
    rl.Cbv(0)   // b0: scene constants
        .Cbv(1)   // b1: per-draw constants
        .Srv(0)   // t0, space0: the FieldDesc table
        .Table({hal::SrvRange(0, hal::kUnbounded, 1), hal::SrvRange(0, hal::kUnbounded, 2),
                hal::SrvRange(0, hal::kUnbounded, 3), hal::SrvRange(0, hal::kUnbounded, 4),
                hal::SrvRange(0, hal::kUnbounded, 5), hal::SrvRange(0, hal::kUnbounded, 6)})
        // b2: the shared SURFACE constants slot (vqview's mechanism for letting later layers
        // evaluate the water surface). Bound by whichever layer owns the surface; buoyant
        // things read it. Read-only sharing of one buffer; no second upload.
        .Cbv(2);
    rl.Sampler(hal::StaticSampler(0, D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                  D3D12_TEXTURE_ADDRESS_MODE_CLAMP))
        .Sampler(hal::StaticSampler(1, D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                    D3D12_TEXTURE_ADDRESS_MODE_WRAP))
        // s2 is point-clamp, and it exists specifically for fields that MUST NOT be
        // hardware-filtered -- rotor fields, where a componentwise lerp silently stops being a
        // rotation.
        .Sampler(hal::StaticSampler(2, D3D12_FILTER_MIN_MAG_MIP_POINT,
                                    D3D12_TEXTURE_ADDRESS_MODE_CLAMP))
        // M9z: s3 is ANISOTROPIC, and it exists for one specific failure. The streamed surface
        // was sampled as CalculateLevelOfDetail + SampleLevel -- an ISOTROPIC level chosen by
        // the LONGEST derivative, then one trilinear tap at it. That is correct looking straight
        // down and wrong at a grazing angle, where the texel footprint is a long thin sliver:
        // the mip gets picked for the stretched axis and everything blurs along the compressed
        // one. It shows up on the descent as diagonal smearing exactly where the globe curves
        // away, which is the artefact this sampler is here to remove.
        //
        // 8x rather than 16x: the footprint anisotropy at these angles is a few to one, 8
        // covers it, and the taps are paid on every surface pixel.
        .Sampler(hal::StaticSampler(3, D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
                                    8))
        .Flags(D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);
    return rl;
}

void Renderer::CreateRootSignature() {
    m_rootSig = SharedGraphicsLayout().Build(*m_gpu, "renderer.shared");
    m_rootSig->SetName(L"shared root signature");
}

void Renderer::CreateTargets(uint32_t width, uint32_t height) {
    m_width = std::max(width, 1u);
    m_height = std::max(height, 1u);

    D3D12_CLEAR_VALUE ccv{};
    ccv.Format = kSceneColorFormat;
    m_sceneColor = m_gpu->CreateTexture2D(m_width, m_height, kSceneColorFormat,
                                         D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET,
                                         D3D12_RESOURCE_STATE_RENDER_TARGET, L"sceneColor", &ccv);

    D3D12_CLEAR_VALUE dcv{};
    dcv.Format = kSceneDepthFormat;
    dcv.DepthStencil.Depth = 0.0f;   // reversed-Z: far is 0
    // No DENY_SHADER_RESOURCE, so this depth buffer can be read later by a volumetric pass.
    m_sceneDepth = m_gpu->CreateTexture2D(m_width, m_height, kSceneDepthFormat,
                                          D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL,
                                          D3D12_RESOURCE_STATE_DEPTH_WRITE, L"sceneDepth", &dcv);

    D3D12_CLEAR_VALUE lcv{};
    lcv.Format = kLdrFormat;
    m_ldrTarget = m_gpu->CreateTexture2D(m_width, m_height, kLdrFormat,
                                         D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET,
                                         D3D12_RESOURCE_STATE_RENDER_TARGET, L"ldrTarget", &lcv);

    // The target views: allocated once (UINT32_MAX), re-created into the same slots on resize.
    m_sceneColorRtv = hal::Rtv(*m_gpu, m_sceneColor.res.Get(), m_sceneColorRtv);
    m_ldrRtv = hal::Rtv(*m_gpu, m_ldrTarget.res.Get(), m_ldrRtv);
    m_sceneDepthDsv = hal::Dsv(*m_gpu, m_sceneDepth.res.Get(), kSceneDepthFormat, m_sceneDepthDsv);

    // SRVs are allocated fresh on resize. The heap is a bump allocator, so this leaks slots across
    // resizes; with a 4096-slot heap that is thousands of resizes before it matters.
    m_sceneColor.srv = m_gpu->CreateSrv(m_sceneColor.res.Get(), kSceneColorFormat);
    m_sceneDepthSrv = m_gpu->CreateSrv(m_sceneDepth.res.Get(), DXGI_FORMAT_R32_FLOAT);
    Log("[renderer] targets %ux%u  sceneColor srv=%u depth srv=%u", m_width, m_height,
        m_sceneColor.srv, m_sceneDepthSrv);
}

void Renderer::CreateTonemapPso() {
    const std::wstring path = m_desc.shaderDir + L"/Tonemap.hlsl";
    hal::GraphicsPipelineDesc d;
    d.rootSig = m_rootSig.Get();
    d.vs = m_shaders.Compile(path, L"VsMain", L"vs_6_0");
    d.ps = m_shaders.Compile(path, L"PsMain", L"ps_6_0");
    if (!d.vs.Valid() || !d.ps.Valid()) {
        Log("[renderer] tonemap shader failed to compile");
        if (!m_tonemapPso) throw std::runtime_error("cannot build the tonemap PSO");
        return;   // keep the old PSO on a failed reload
    }
    d.depthClip = TRUE;
    d.dsvFormat = DXGI_FORMAT_UNKNOWN;   // no depth target is bound at the tonemap pass
    d.rtvFormat = kLdrFormat;
    // The reload law: swap only on success. At boot a missing tonemap is fatal.
    const bool built = hal::Reload(
        m_tonemapPso, [&] { return hal::BuildGraphics(*m_gpu, d, "renderer.tonemap"); },
        "renderer.tonemap");
    if (!built && !m_tonemapPso) throw std::runtime_error("cannot build the tonemap PSO");
}

void Renderer::AddLayer(std::unique_ptr<Layer> layer) {
    Log("[renderer] + layer '%s'", layer->Name());
    m_layers.push_back(std::move(layer));
}

Layer* Renderer::FindLayer(const char* name) {
    for (auto& l : m_layers) {
        if (strcmp(l->Name(), name) == 0) return l.get();
    }
    return nullptr;
}

void Renderer::OnResize(uint32_t width, uint32_t height) {
    m_gpu->WaitIdle();
    m_gpu->Resize(width, height);
    CreateTargets(width, height);
}

void Renderer::ReloadShaders() {
    m_gpu->WaitIdle();
    Log("[renderer] reloading shaders");
    CreateTonemapPso();
    for (auto& l : m_layers) l->ReloadShaders(*m_gpu, m_shaders);
}

void Renderer::RenderFrame(const Camera& cam, float timeSec, float dt) {
    (void)dt;
    auto* cl = m_gpu->BeginFrame();
    hal::CommandContext cmd(*m_gpu, cl, hal::Owner::Frame);
    // --gpu-time: reads the slot this frame index last used (fenced by BeginFrame above), then
    // opens the whole-frame pair. Null profiler = the default path, no queries at all.
    GpuProfiler* prof = m_prof.get();
    if (prof) prof->BeginFrame(cl, gpuFrameLabel);

    // ---- scene constants
    SceneConstants sc{};
    const XMMATRIX view = cam.ViewRelative();
    const XMMATRIX proj = cam.Projection(static_cast<float>(m_width) / static_cast<float>(m_height));
    // HLSL here uses mul(float4, matrix), i.e. row-vector convention, which is DirectXMath's
    // native layout. No transpose. Common.hlsli declares the cbuffer matrix row_major to match.
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(sc.viewProj), XMMatrixMultiply(view, proj));

    // M9bi: the placed sun wins. Its direction was computed once, on the CPU, from the sun's
    // conformal POINT in the solar frame and carried here by the versor chain -- so every layer
    // in the scene shares not just one vector but one PLACE.
    if (sunPlaced) {
        sc.sunDir[0] = sunDirTangent[0];
        sc.sunDir[1] = sunDirTangent[1];
        sc.sunDir[2] = sunDirTangent[2];
    } else {
        const float az = XMConvertToRadians(sunAzimuthDeg);
        const float el = XMConvertToRadians(sunElevationDeg);
        sc.sunDir[0] = std::cos(el) * std::sin(az);
        sc.sunDir[1] = std::sin(el);
        sc.sunDir[2] = std::cos(el) * std::cos(az);
    }

    for (int c = 0; c < 3; ++c) {
        sc.sigmaW[c] = sigmaW[c];
        sc.bscat[c] = bscat[c];
    }

    sc.params0[0] = timeSec;
    sc.params0[1] = m_desc.heightScale;
    sc.params0[2] = m_desc.patchWidthM;
    sc.params0[3] = m_desc.patchHeightM;
    sc.params1[0] = 1.0f;
    sc.params1[1] = static_cast<float>(m_width) / static_cast<float>(m_height);
    sc.params1[2] = cam.nearZ;
    sc.params1[3] = m_desc.exposure;
    sc.eyeRelWorld[0] = static_cast<float>(cam.px);
    sc.eyeRelWorld[1] = static_cast<float>(cam.py);
    sc.eyeRelWorld[2] = static_cast<float>(cam.pz);

    // M6j: the ONE render basis (gravity-up aware) -- rays rebuilt from these constants now
    // agree with gViewProj by construction.
    XMFLOAT3 fwd, rgt, upv;
    cam.ViewBasis(fwd, rgt, upv);
    const XMVECTOR vf = XMLoadFloat3(&fwd);
    const XMVECTOR vr = XMLoadFloat3(&rgt);
    const XMVECTOR vu = XMLoadFloat3(&upv);
    const float tanH = std::tan(cam.fovY * 0.5f);
    const float aspect = static_cast<float>(m_width) / static_cast<float>(m_height);
    XMStoreFloat3(reinterpret_cast<XMFLOAT3*>(sc.camFwd), vf);
    XMStoreFloat3(reinterpret_cast<XMFLOAT3*>(sc.camRight), XMVectorScale(vr, tanH * aspect));
    XMStoreFloat3(reinterpret_cast<XMFLOAT3*>(sc.camUp), XMVectorScale(vu, tanH));

    sc.viewport[0] = static_cast<float>(m_width);
    sc.viewport[1] = static_cast<float>(m_height);
    sc.viewport[2] = 1.0f / static_cast<float>(m_width);
    sc.viewport[3] = 1.0f / static_cast<float>(m_height);
    sc.misc[0] = waterLevel;
    // The sun's DISC, from its real angular radius: bright inside 0.85 R, gone by 1.15 R. The
    // constants this replaces (cos 0.44 deg .. cos 0.99 deg) drew a sun between 1.7x and 3.7x
    // too wide, which no amount of exposure tuning could have diagnosed.
    {
        const float r = XMConvertToRadians(sunAngRadiusDeg);
        sc.misc[1] = std::cos(r * 1.15f);
        sc.misc[2] = std::cos(r * 0.85f);
    }

    const D3D12_GPU_VIRTUAL_ADDRESS sceneCb = m_gpu->PushConstants(&sc, sizeof(sc));

    // ---- opaque layers into the HDR target
    cmd.Barrier(m_sceneColor, D3D12_RESOURCE_STATE_RENDER_TARGET);
    cmd.Barrier(m_sceneDepth, D3D12_RESOURCE_STATE_DEPTH_WRITE);

    const auto colorRtv = m_gpu->RtvHeap().Cpu(m_sceneColorRtv);
    const auto depthDsv = m_gpu->DsvHeap().Cpu(m_sceneDepthDsv);
    cmd.Targets(colorRtv, &depthDsv);

    const float clearColor[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    cmd.ClearColor(colorRtv, clearColor);
    // Reversed-Z: clear depth to 0 (far), compare GREATER.
    cmd.ClearDepth(depthDsv, 0.0f);

    cmd.Viewport(m_width, m_height);

    cmd.GraphicsRoot(m_rootSig.Get());
    cmd.GraphicsConstantsAt(0, sceneCb);
    if (m_fieldTableVa) cmd.GraphicsSrvAt(2, m_fieldTableVa);
    cmd.GraphicsBindless(3);

    FrameContext ctx;
    ctx.gpu = m_gpu;
    ctx.cmd = &cmd;
    ctx.camera = &cam;
    ctx.sceneCb = sceneCb;
    ctx.timeSec = timeSec;
    ctx.width = m_width;
    ctx.height = m_height;
    ctx.prof = prof;
    for (auto& l : m_layers) {
        if (!l->enabled) continue;
        PixScope scope(cl, l->Name());
        GpuScope gscope(prof, cl, l->Name());
        l->Render(ctx);
    }

    // ---- tonemap HDR -> LDR
    {
        PixScope scope(cl, "tonemap");
        GpuScope gscope(prof, cl, "tonemap");
        cmd.Barrier(m_sceneColor, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        cmd.Barrier(m_ldrTarget, D3D12_RESOURCE_STATE_RENDER_TARGET);
        const auto ldrRtv = m_gpu->RtvHeap().Cpu(m_ldrRtv);
        cmd.Targets(ldrRtv, nullptr);
        cmd.Pipeline(m_tonemapPso.Get());
        cmd.GraphicsConstantsAt(0, sceneCb);
        if (m_fieldTableVa) cmd.GraphicsSrvAt(2, m_fieldTableVa);
        cmd.GraphicsBindless(3);
        struct { uint32_t srv; uint32_t pad[3]; } tm{m_sceneColor.srv, {0, 0, 0}};
        cmd.GraphicsConstants(1, tm);
        cmd.DrawFullscreen();   // fullscreen triangle from SV_VertexID, no vertex buffer
    }

    // ---- to the swapchain, when there is one
    if (!m_gpu->Headless()) {
        GpuScope gscope(prof, cl, "present-copy");
        hal::Resource bb = m_gpu->BackBuffer();
        // Two independent transitions, one call each (was one ResourceBarrier(2, ...); the
        // same two barriers reach the queue in the same order).
        cmd.Barrier(bb, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_DEST);
        cmd.Barrier(m_ldrTarget.res.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET,
                    D3D12_RESOURCE_STATE_COPY_SOURCE);
        m_ldrTarget.state = D3D12_RESOURCE_STATE_COPY_SOURCE;

        cmd.Copy(bb, m_ldrTarget.res.Get());

        cmd.Barrier(bb, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PRESENT);
    }

    if (prof) prof->EndFrame(cl);   // closes the frame pair and resolves, before Close()
    m_gpu->EndFrame(!m_gpu->Headless());
}

// M9s: the frame as RAW RGBA8, for the recorder that pipes straight to an encoder. Same
// readback DumpPng does; what it saves is the PNG compression, which the rail metrics measured
// at ~90 ms per frame -- more than fifty times the 1.7 ms it takes to RENDER the frame. A
// recording was spending 98% of its wall clock turning pictures into files nothing kept.
bool Renderer::DumpRaw(std::vector<uint8_t>& out, uint32_t* rowPitch) {
    m_gpu->WaitIdle();
    uint32_t rp = 0;
    out = m_gpu->ReadbackTexture(m_ldrTarget, &rp);
    if (rowPitch) *rowPitch = rp;
    return !out.empty();
}

bool Renderer::DumpPng(const std::wstring& path) {
    m_gpu->WaitIdle();
    uint32_t rowPitch = 0;
    std::vector<uint8_t> pixels = m_gpu->ReadbackTexture(m_ldrTarget, &rowPitch);
    if (pixels.empty()) return false;
    const bool ok = SavePng(path, pixels.data(), m_width, m_height, rowPitch, pixels.size());
    Log("[renderer] dump %S : %s", path.c_str(), ok ? "ok" : "FAILED");
    return ok;
}

bool Renderer::DumpHdr(const std::wstring& path) {
    m_gpu->WaitIdle();
    uint32_t rowPitch = 0;
    // sceneColor sits in PIXEL_SHADER_RESOURCE after the tonemap read it; ReadbackTexture
    // transitions to COPY_SOURCE and back, so the next frame's RENDER_TARGET transition holds.
    std::vector<uint8_t> px = m_gpu->ReadbackTexture(m_sceneColor, &rowPitch);
    if (px.empty()) return false;
    FILE* f = nullptr;
    if (_wfopen_s(&f, path.c_str(), L"wb") != 0 || !f) {
        Log("[renderer] dump-hdr %S : FAILED (open)", path.c_str());
        return false;
    }
    const size_t rowBytes = size_t(m_width) * 8;   // RGBA16F, de-pitched
    bool ok = true;
    for (uint32_t y = 0; y < m_height && ok; ++y) {
        ok = fwrite(px.data() + size_t(y) * rowPitch, 1, rowBytes, f) == rowBytes;
    }
    fclose(f);
    const std::wstring side = path + L".json";
    if (_wfopen_s(&f, side.c_str(), L"wb") == 0 && f) {
        // The curve the 8-bit image went through (Tonemap.hlsl): v = pow(shoulder(E * L), 1 / gamma)
        // with shoulder(x) = min(x, knee) + (1 - knee) * (1 - exp(-(x - knee)+ / (1 - knee))).
        fprintf(f,
                "{ \"width\": %u, \"height\": %u, \"format\": \"RGBA16F\", \"exposure\": %.9g, "
                "\"knee\": 0.85, \"gamma\": 2.2 }\n",
                m_width, m_height, double(m_desc.exposure));
        fclose(f);
    } else {
        ok = false;
    }
    Log("[renderer] dump-hdr %S : %s (%ux%u RGBA16F, exposure %.4g, sidecar %S)", path.c_str(),
        ok ? "ok" : "FAILED", m_width, m_height, double(m_desc.exposure), side.c_str());
    return ok;
}

}  // namespace ga
