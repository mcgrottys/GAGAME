#include "render/Renderer.h"

#include "core/Image.h"
#include "hal/Context.h"
#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Views.h"
#include "scene/FieldSet.h"
#include "scene/ViewContext.h"

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
        // b2: THE SURFACE constants (Common.hlsli's SurfaceCb, ga::ComposedSurfaceCb) --
        // vqview's mechanism for letting later layers evaluate one surface, finally bound
        // (M12 step 4g): the frame loop fills the rows once through SurfaceFrame::Fill,
        // RenderFrame pushes them once and binds them here before any layer records, and every
        // layer that samples the planet reads that one buffer. One upload a frame, no copies.
        .Cbv(2)
        // b3: the globe's sky constants (Globe.hlsl's GlobeSkyCb: the camera basis its sky and
        // limb passes rebuild their rays from), moved off b2 so the surface could take it.
        // Appended, so the parameter indices 0..4 every bind site names are unchanged: this
        // one is 5.
        .Cbv(3);
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

// ================================================================================================
//  M12 step 5b: THE FILL, as a function of its inputs. This is the span RenderFrame used to open
//  with, moved without a change of expression -- so the rows can be built for a view that is not
//  the one being recorded, and so the gate (hal/DxTest.cpp) can hold the frozen old body against
//  this one at the six recipe poses and memcmp the result.
// ================================================================================================
void Renderer::FillSceneConstants(const SceneFill& f, SceneConstants& sc) {
    sc = SceneConstants{};
    const Camera& cam = *f.cam;
    const XMMATRIX view = cam.ViewRelative();
    const XMMATRIX proj = cam.Projection(static_cast<float>(f.width) / static_cast<float>(f.height));
    // HLSL here uses mul(float4, matrix), i.e. row-vector convention, which is DirectXMath's
    // native layout. No transpose. Common.hlsli declares the cbuffer matrix row_major to match.
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(sc.viewProj), XMMatrixMultiply(view, proj));

    // M9bi: the placed sun wins. Its direction was computed once, on the CPU, from the sun's
    // conformal POINT in the solar frame and carried here by the versor chain -- so every layer
    // in the scene shares not just one vector but one PLACE.
    if (f.sunPlaced) {
        sc.sunDir[0] = f.sunDirTangent[0];
        sc.sunDir[1] = f.sunDirTangent[1];
        sc.sunDir[2] = f.sunDirTangent[2];
    } else {
        const float az = XMConvertToRadians(f.sunAzimuthDeg);
        const float el = XMConvertToRadians(f.sunElevationDeg);
        sc.sunDir[0] = std::cos(el) * std::sin(az);
        sc.sunDir[1] = std::sin(el);
        sc.sunDir[2] = std::cos(el) * std::cos(az);
    }

    for (int c = 0; c < 3; ++c) {
        sc.sigmaW[c] = f.sigmaW[c];
        sc.bscat[c] = f.bscat[c];
    }

    sc.params0[0] = f.timeSec;
    sc.params0[1] = f.heightScale;
    sc.params0[2] = f.patchWidthM;
    sc.params0[3] = f.patchHeightM;
    sc.params1[0] = 1.0f;
    sc.params1[1] = static_cast<float>(f.width) / static_cast<float>(f.height);
    sc.params1[2] = cam.nearZ;
    sc.params1[3] = f.exposure;
    sc.eyeRelWorld[0] = static_cast<float>(cam.px);
    sc.eyeRelWorld[1] = static_cast<float>(cam.py);
    sc.eyeRelWorld[2] = static_cast<float>(cam.pz);

    // M6j: the ONE render basis (gravity-up aware) -- rays rebuilt from these constants now
    // agree with gViewProj by construction. (M12 step 5b: View::Level is the same frame built
    // from a motor instead of two Euler angles, gated against this one and read by nothing yet.)
    XMFLOAT3 fwd, rgt, upv;
    cam.ViewBasis(fwd, rgt, upv);
    const XMVECTOR vf = XMLoadFloat3(&fwd);
    const XMVECTOR vr = XMLoadFloat3(&rgt);
    const XMVECTOR vu = XMLoadFloat3(&upv);
    const float tanH = std::tan(cam.fovY * 0.5f);
    const float aspect = static_cast<float>(f.width) / static_cast<float>(f.height);
    XMStoreFloat3(reinterpret_cast<XMFLOAT3*>(sc.camFwd), vf);
    XMStoreFloat3(reinterpret_cast<XMFLOAT3*>(sc.camRight), XMVectorScale(vr, tanH * aspect));
    XMStoreFloat3(reinterpret_cast<XMFLOAT3*>(sc.camUp), XMVectorScale(vu, tanH));

    sc.viewport[0] = static_cast<float>(f.width);
    sc.viewport[1] = static_cast<float>(f.height);
    sc.viewport[2] = 1.0f / static_cast<float>(f.width);
    sc.viewport[3] = 1.0f / static_cast<float>(f.height);
    sc.misc[0] = f.waterLevel;
    // The sun's DISC, from its real angular radius: bright inside 0.85 R, gone by 1.15 R. The
    // constants this replaces (cos 0.44 deg .. cos 0.99 deg) drew a sun between 1.7x and 3.7x
    // too wide, which no amount of exposure tuning could have diagnosed.
    {
        const float r = XMConvertToRadians(f.sunAngRadiusDeg);
        sc.misc[1] = std::cos(r * 1.15f);
        sc.misc[2] = std::cos(r * 0.85f);
    }
    // M13: the sky's table. An invalid slot is the shipped two-constant gradient, so a pass that
    // runs without the sky layer (a tool, a test) still draws a sky.
    // The slots are small integers and travel as NUMBERS, not as reinterpreted bit patterns:
    // a heap slot of 0xFFFFFFFF read as a float is a NaN, and a NaN in the scene constants is
    // never equal to itself -- which is exactly what the views gate said when it first ran.
    // -1 is "no table", and every consumer falls back to the gradient.
    sc.skyLut[0] = (f.skyMsSrv == 0xFFFFFFFFu) ? -1.0f : static_cast<float>(f.skyMsSrv);
    sc.skyLut[1] = f.planetRadiusM;
    sc.skyLut[2] = f.eyeRadiusM;
    sc.skyLut[3] = 0.0f;   // spare: the sun's transmittance is a closed form, not a table
}

SceneFill Renderer::FillInputs(const Camera& cam, float timeSec) const {
    SceneFill f;
    f.cam = &cam;
    f.timeSec = timeSec;
    f.width = m_width;
    f.height = m_height;
    f.heightScale = m_desc.heightScale;
    f.patchWidthM = m_desc.patchWidthM;
    f.patchHeightM = m_desc.patchHeightM;
    f.exposure = m_desc.exposure;
    f.sunPlaced = sunPlaced;
    f.sunAzimuthDeg = sunAzimuthDeg;
    f.sunElevationDeg = sunElevationDeg;
    f.sunAngRadiusDeg = sunAngRadiusDeg;
    f.waterLevel = waterLevel;
    f.skyMsSrv = skyMsSrv;
    f.planetRadiusM = planetRadiusM;
    f.eyeRadiusM = eyeRadiusM;
    for (int c = 0; c < 3; ++c) {
        f.sunDirTangent[c] = sunDirTangent[c];
        f.sigmaW[c] = sigmaW[c];
        f.bscat[c] = bscat[c];
    }
    return f;
}

scene::ViewSet Renderer::OneView(const Camera& cam, float timeSec) const {
    scene::ViewSet set;
    set.views.resize(1);
    scene::ViewContext& v = set.views[0];
    v.index = 0;
    // No View yet: the session's Camera is still the state and the View is the render's INPUT.
    // 5d hands the scene's View over; nothing about this seam changes when it does.
    v.view = nullptr;
    FillSceneConstants(FillInputs(cam, timeSec), v.constants);
    v.target = "main";
    v.viewport = {0, 0, m_width, m_height};
    v.legacy.camera = &cam;
    v.legacy.timeSec = timeSec;
    return set;
}

void Renderer::RenderFrame(const Camera& cam, float timeSec, float dt) {
    (void)dt;   // unused before the seam, unused after it
    RenderFrame(OneView(cam, timeSec));
}

void Renderer::RenderFrame(const scene::ViewSet& set) {
    // An empty set is a caller's mistake, and it is refused BEFORE the frame ring opens, so
    // Begin/End stay paired and nothing half-records.
    if (set.views.empty()) {
        Log("[renderer] RenderFrame: empty ViewSet -- no frame recorded");
        return;
    }
    // The ring of per-view constant addresses is a fixed array on purpose: a frame must not
    // allocate, and a set larger than this is REPORTED rather than silently truncated. Eight is
    // four-way split screen with a stereo pair each; nothing today asks for two.
    constexpr size_t kMaxViews = 8;
    const size_t n = set.views.size() < kMaxViews ? set.views.size() : kMaxViews;
    if (set.views.size() > kMaxViews) {
        Log("[renderer] %zu views in one frame: only the first %zu are recorded",
            set.views.size(), kMaxViews);
    }

    auto* cl = m_gpu->BeginFrame();
    hal::CommandContext cmd(*m_gpu, cl, hal::Owner::Frame);
    // --gpu-time: reads the slot this frame index last used (fenced by BeginFrame above), then
    // opens the whole-frame pair. Null profiler = the default path, no queries at all.
    GpuProfiler* prof = m_prof.get();
    if (prof) prof->BeginFrame(cl, gpuFrameLabel);
    // Step 5 E, law 8: the residency turn, first in the frame's list, before any layer reads.
    if (atFrameHead) atFrameHead(cmd);

    // ---- the constants: one b0 per view, pushed in VIEW ORDER, then the surface's b2 once.
    // With one view that is the ring's old two pushes in their old order, which is why the
    // addresses the recording names do not move.
    D3D12_GPU_VIRTUAL_ADDRESS sceneCbs[kMaxViews] = {};
    for (size_t i = 0; i < n; ++i) {
        sceneCbs[i] = m_gpu->PushConstants(&set.views[i].constants, sizeof(SceneConstants));
    }
    // M12 step 4g: the surface's rows (b2), pushed once beside the scene's and bound below for
    // every layer of every view; SurfaceFrame::Fill wrote them into surfaceCb this frame
    // (FrameLoop.cpp). One surface, however many eyes are looking at it.
    const D3D12_GPU_VIRTUAL_ADDRESS surfaceVa = cmd.Push(surfaceCb);

    // ---- the simulation: once a frame, every declared layer, before any view draws -- whatever
    // the per-frame `enabled` gate says, because that gate is a view's (Layer::Simulate). Recorded
    // first, so every layer of every view reads the state this frame advanced.
    {
        FrameContext sim = set.views[0].legacy;
        sim.gpu = m_gpu;
        sim.cmd = &cmd;
        sim.sceneCb = sceneCbs[0];
        sim.width = m_width;
        sim.height = m_height;
        sim.prof = prof;
        sim.viewIndex = set.views[0].index;
        for (auto& l : m_layers) {
            if (l->declared) l->Simulate(sim);
        }
    }

    // ---- opaque layers into the HDR target. The target, the barriers and the CLEAR belong to
    // the frame, not to a view: they happen once, before the first view records into them.
    cmd.Barrier(m_sceneColor, D3D12_RESOURCE_STATE_RENDER_TARGET);
    cmd.Barrier(m_sceneDepth, D3D12_RESOURCE_STATE_DEPTH_WRITE);

    const auto colorRtv = m_gpu->RtvHeap().Cpu(m_sceneColorRtv);
    const auto depthDsv = m_gpu->DsvHeap().Cpu(m_sceneDepthDsv);
    cmd.Targets(colorRtv, &depthDsv);

    const float clearColor[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    cmd.ClearColor(colorRtv, clearColor);
    // Reversed-Z: clear depth to 0 (far), compare GREATER.
    cmd.ClearDepth(depthDsv, 0.0f);

    for (size_t i = 0; i < n; ++i) {
        // The renderer's own copy of the view: it owns the recording context and the fields of
        // the legacy FrameContext that are the FRAME's (the device, the profiler, this view's
        // b0 address and its index), and the caller owns the ones that are the VIEW's.
        scene::ViewContext v = set.views[i];
        const uint32_t vw = v.viewport.w ? v.viewport.w : m_width;
        const uint32_t vh = v.viewport.h ? v.viewport.h : m_height;
        if (v.viewport.x || v.viewport.y) {
            // Reported, not guessed: an offset viewport needs one more overload on
            // hal::CommandContext, which the seam step does not add.
            Log("[renderer] view %u asks for a viewport at (%u, %u): an offset needs "
                "hal::CommandContext::Viewport(x, y, w, h), which does not exist yet -- "
                "recording the whole target instead",
                v.index, v.viewport.x, v.viewport.y);
        }
        v.cmd = &cmd;
        v.legacy.gpu = m_gpu;
        v.legacy.cmd = &cmd;
        v.legacy.sceneCb = sceneCbs[i];
        v.legacy.width = vw;
        v.legacy.height = vh;
        v.legacy.prof = prof;
        v.legacy.viewIndex = v.index;

        cmd.Viewport(vw, vh);

        cmd.GraphicsRoot(m_rootSig.Get());
        cmd.GraphicsConstantsAt(0, sceneCbs[i]);
        cmd.GraphicsConstantsAt(4, surfaceVa);   // b2: the surface, once, for every layer (4g)
        if (m_fieldTableVa) cmd.GraphicsSrvAt(2, m_fieldTableVa);
        cmd.GraphicsBindless(3);

        for (auto& l : m_layers) {
            if (!l->declared || !l->enabled) continue;
            PixScope scope(cl, l->Name());
            GpuScope gscope(prof, cl, l->Name());
            if (beforeLayer) beforeLayer(l->Name());   // step 5: where each layer reads, in the frame
            l->Render(v.legacy);
        }
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
        cmd.GraphicsConstantsAt(0, sceneCbs[0]);   // the frame's exposure: view 0's rows
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

bool Renderer::ReadDepth(std::vector<float>& out) {
    m_gpu->WaitIdle();
    uint32_t rowPitch = 0;
    const std::vector<uint8_t> px = m_gpu->ReadbackTexture(m_sceneDepth, &rowPitch);
    if (px.empty() || rowPitch < m_width * 4u) return false;
    out.resize(size_t(m_width) * m_height);
    for (uint32_t y = 0; y < m_height; ++y) {
        memcpy(out.data() + size_t(y) * m_width, px.data() + size_t(y) * rowPitch,
               size_t(m_width) * 4);
    }
    return true;
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
