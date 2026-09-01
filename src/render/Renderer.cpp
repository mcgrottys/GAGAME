#include "render/Renderer.h"

#include "core/Image.h"
#include "core/PixEvents.h"
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
}

void Renderer::CreateRootSignature() {
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
    D3D12_DESCRIPTOR_RANGE1 ranges[6]{};
    ranges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    ranges[0].NumDescriptors = UINT_MAX;   // unbounded; requires resource binding tier 3
    ranges[0].BaseShaderRegister = 0;
    ranges[0].RegisterSpace = 1;
    ranges[0].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ranges[0].OffsetInDescriptorsFromTableStart = 0;
    ranges[1] = ranges[0];
    ranges[1].RegisterSpace = 2;
    ranges[2] = ranges[0];
    ranges[2].RegisterSpace = 3;
    ranges[3] = ranges[0];
    ranges[3].RegisterSpace = 4;
    ranges[4] = ranges[0];
    ranges[4].RegisterSpace = 5;
    ranges[5] = ranges[0];
    ranges[5].RegisterSpace = 6;

    D3D12_ROOT_PARAMETER1 params[5]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[0].Descriptor.ShaderRegister = 0;   // b0
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[1].Descriptor.ShaderRegister = 1;   // b1
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[2].Descriptor.ShaderRegister = 0;   // t0, space0 -> FieldDesc table
    params[2].Descriptor.RegisterSpace = 0;
    params[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    params[3].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[3].DescriptorTable.NumDescriptorRanges = 6;
    params[3].DescriptorTable.pDescriptorRanges = ranges;
    params[3].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    // b2: the shared SURFACE constants slot (vqview's mechanism for letting later layers evaluate
    // the water surface). Unused in M1 but kept: the FFT ocean will bind it in M2 and buoyant
    // things read it in M7. Read-only sharing of one buffer; no second upload.
    params[4].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[4].Descriptor.ShaderRegister = 2;   // b2
    params[4].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_STATIC_SAMPLER_DESC samplers[4]{};
    auto initSampler = [](D3D12_STATIC_SAMPLER_DESC& s, UINT reg, D3D12_FILTER filter,
                          D3D12_TEXTURE_ADDRESS_MODE addr) {
        s.Filter = filter;
        s.AddressU = s.AddressV = s.AddressW = addr;
        s.MaxLOD = D3D12_FLOAT32_MAX;
        s.ShaderRegister = reg;
        s.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
        s.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER;
    };
    initSampler(samplers[0], 0, D3D12_FILTER_MIN_MAG_MIP_LINEAR, D3D12_TEXTURE_ADDRESS_MODE_CLAMP);
    initSampler(samplers[1], 1, D3D12_FILTER_MIN_MAG_MIP_LINEAR, D3D12_TEXTURE_ADDRESS_MODE_WRAP);
    // s2 is point-clamp, and it exists specifically for fields that MUST NOT be hardware-filtered
    // -- rotor fields, where a componentwise lerp silently stops being a rotation.
    initSampler(samplers[2], 2, D3D12_FILTER_MIN_MAG_MIP_POINT, D3D12_TEXTURE_ADDRESS_MODE_CLAMP);
    // M9z: s3 is ANISOTROPIC, and it exists for one specific failure. The streamed surface was
    // sampled as CalculateLevelOfDetail + SampleLevel -- an ISOTROPIC level chosen by the
    // LONGEST derivative, then one trilinear tap at it. That is correct looking straight down
    // and wrong at a grazing angle, where the texel footprint is a long thin sliver: the mip
    // gets picked for the stretched axis and everything blurs along the compressed one. It
    // shows up on the descent as diagonal smearing exactly where the globe curves away, which
    // is the artefact this sampler is here to remove.
    //
    // 8x rather than 16x: the footprint anisotropy at these angles is a few to one, 8 covers it,
    // and the taps are paid on every surface pixel.
    initSampler(samplers[3], 3, D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_CLAMP);
    samplers[3].MaxAnisotropy = 8;

    D3D12_VERSIONED_ROOT_SIGNATURE_DESC vd{};
    vd.Version = D3D_ROOT_SIGNATURE_VERSION_1_1;
    vd.Desc_1_1.NumParameters = _countof(params);
    vd.Desc_1_1.pParameters = params;
    vd.Desc_1_1.NumStaticSamplers = _countof(samplers);
    vd.Desc_1_1.pStaticSamplers = samplers;
    vd.Desc_1_1.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

    Com<ID3DBlob> blob, err;
    HRESULT hr = D3D12SerializeVersionedRootSignature(&vd, &blob, &err);
    if (FAILED(hr)) {
        if (err) Log("[renderer] root signature: %s", static_cast<const char*>(err->GetBufferPointer()));
        GA_CHECK(hr);
    }
    GA_CHECK(m_gpu->Device()->CreateRootSignature(0, blob->GetBufferPointer(),
                                                 blob->GetBufferSize(), IID_PPV_ARGS(&m_rootSig)));
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

    if (m_sceneColorRtv == UINT32_MAX) {
        m_sceneColorRtv = m_gpu->RtvHeap().Alloc();
        m_ldrRtv = m_gpu->RtvHeap().Alloc();
        m_sceneDepthDsv = m_gpu->DsvHeap().Alloc();
    }
    m_gpu->Device()->CreateRenderTargetView(m_sceneColor.res.Get(), nullptr,
                                            m_gpu->RtvHeap().Cpu(m_sceneColorRtv));
    m_gpu->Device()->CreateRenderTargetView(m_ldrTarget.res.Get(), nullptr,
                                            m_gpu->RtvHeap().Cpu(m_ldrRtv));

    D3D12_DEPTH_STENCIL_VIEW_DESC dsv{};
    dsv.Format = kSceneDepthFormat;
    dsv.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
    m_gpu->Device()->CreateDepthStencilView(m_sceneDepth.res.Get(), &dsv,
                                            m_gpu->DsvHeap().Cpu(m_sceneDepthDsv));

    // SRVs are allocated fresh on resize. The heap is a bump allocator, so this leaks slots across
    // resizes; with a 4096-slot heap that is thousands of resizes before it matters.
    m_sceneColor.srv = m_gpu->CreateSrv(m_sceneColor.res.Get(), kSceneColorFormat);
    m_sceneDepthSrv = m_gpu->CreateSrv(m_sceneDepth.res.Get(), DXGI_FORMAT_R32_FLOAT);
    Log("[renderer] targets %ux%u  sceneColor srv=%u depth srv=%u", m_width, m_height,
        m_sceneColor.srv, m_sceneDepthSrv);
}

void Renderer::CreateTonemapPso() {
    const std::wstring path = m_desc.shaderDir + L"/Tonemap.hlsl";
    ShaderBlob vs = m_shaders.Compile(path, L"VsMain", L"vs_6_0");
    ShaderBlob ps = m_shaders.Compile(path, L"PsMain", L"ps_6_0");
    if (!vs.Valid() || !ps.Valid()) {
        Log("[renderer] tonemap shader failed to compile");
        if (!m_tonemapPso) throw std::runtime_error("cannot build the tonemap PSO");
        return;   // keep the old PSO on a failed reload
    }

    D3D12_GRAPHICS_PIPELINE_STATE_DESC d{};
    d.pRootSignature = m_rootSig.Get();
    d.VS = {vs.Data(), vs.Size()};
    d.PS = {ps.Data(), ps.Size()};
    d.BlendState.RenderTarget[0].SrcBlend = D3D12_BLEND_ONE;
    d.BlendState.RenderTarget[0].DestBlend = D3D12_BLEND_ZERO;
    d.BlendState.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
    d.BlendState.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
    d.BlendState.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_ZERO;
    d.BlendState.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
    d.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    d.SampleMask = UINT_MAX;
    d.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    d.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    d.RasterizerState.DepthClipEnable = TRUE;
    d.DepthStencilState.DepthEnable = FALSE;
    d.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    d.NumRenderTargets = 1;
    d.RTVFormats[0] = kLdrFormat;
    d.SampleDesc.Count = 1;

    Com<ID3D12PipelineState> pso;
    HRESULT hr = m_gpu->Device()->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso));
    if (FAILED(hr)) {
        Log("[renderer] tonemap PSO: %s", HrString(hr).c_str());
        if (!m_tonemapPso) GA_CHECK(hr);
        return;
    }
    m_tonemapPso = pso;
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

    // ---- scene constants
    SceneConstants sc{};
    const XMMATRIX view = cam.ViewRelative();
    const XMMATRIX proj = cam.Projection(static_cast<float>(m_width) / static_cast<float>(m_height));
    // HLSL here uses mul(float4, matrix), i.e. row-vector convention, which is DirectXMath's
    // native layout. No transpose. Common.hlsli declares the cbuffer matrix row_major to match.
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(sc.viewProj), XMMatrixMultiply(view, proj));

    const float az = XMConvertToRadians(sunAzimuthDeg);
    const float el = XMConvertToRadians(sunElevationDeg);
    sc.sunDir[0] = std::cos(el) * std::sin(az);
    sc.sunDir[1] = std::sin(el);
    sc.sunDir[2] = std::cos(el) * std::cos(az);

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

    const D3D12_GPU_VIRTUAL_ADDRESS sceneCb = m_gpu->PushConstants(&sc, sizeof(sc));

    // ---- opaque layers into the HDR target
    m_gpu->Transition(cl, m_sceneColor, D3D12_RESOURCE_STATE_RENDER_TARGET);
    m_gpu->Transition(cl, m_sceneDepth, D3D12_RESOURCE_STATE_DEPTH_WRITE);

    const auto colorRtv = m_gpu->RtvHeap().Cpu(m_sceneColorRtv);
    const auto depthDsv = m_gpu->DsvHeap().Cpu(m_sceneDepthDsv);
    cl->OMSetRenderTargets(1, &colorRtv, FALSE, &depthDsv);

    const float clearColor[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    cl->ClearRenderTargetView(colorRtv, clearColor, 0, nullptr);
    // Reversed-Z: clear depth to 0 (far), compare GREATER.
    cl->ClearDepthStencilView(depthDsv, D3D12_CLEAR_FLAG_DEPTH, 0.0f, 0, 0, nullptr);

    D3D12_VIEWPORT vp{0, 0, static_cast<float>(m_width), static_cast<float>(m_height), 0.0f, 1.0f};
    D3D12_RECT scissor{0, 0, static_cast<LONG>(m_width), static_cast<LONG>(m_height)};
    cl->RSSetViewports(1, &vp);
    cl->RSSetScissorRects(1, &scissor);

    cl->SetGraphicsRootSignature(m_rootSig.Get());
    cl->SetGraphicsRootConstantBufferView(0, sceneCb);
    if (m_fieldTableVa) cl->SetGraphicsRootShaderResourceView(2, m_fieldTableVa);
    cl->SetGraphicsRootDescriptorTable(3, m_gpu->SrvHeap().Gpu(0));

    FrameContext ctx;
    ctx.gpu = m_gpu;
    ctx.cl = cl;
    ctx.camera = &cam;
    ctx.sceneCb = sceneCb;
    ctx.timeSec = timeSec;
    ctx.width = m_width;
    ctx.height = m_height;
    for (auto& l : m_layers) {
        if (!l->enabled) continue;
        PixScope scope(cl, l->Name());
        l->Render(ctx);
    }

    // ---- tonemap HDR -> LDR
    {
        PixScope scope(cl, "tonemap");
        m_gpu->Transition(cl, m_sceneColor, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        m_gpu->Transition(cl, m_ldrTarget, D3D12_RESOURCE_STATE_RENDER_TARGET);
        const auto ldrRtv = m_gpu->RtvHeap().Cpu(m_ldrRtv);
        cl->OMSetRenderTargets(1, &ldrRtv, FALSE, nullptr);
        cl->SetPipelineState(m_tonemapPso.Get());
        cl->SetGraphicsRootConstantBufferView(0, sceneCb);
        if (m_fieldTableVa) cl->SetGraphicsRootShaderResourceView(2, m_fieldTableVa);
        cl->SetGraphicsRootDescriptorTable(3, m_gpu->SrvHeap().Gpu(0));
        struct { uint32_t srv; uint32_t pad[3]; } tm{m_sceneColor.srv, {0, 0, 0}};
        cl->SetGraphicsRootConstantBufferView(1, m_gpu->PushConstants(&tm, sizeof(tm)));
        cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        cl->DrawInstanced(3, 1, 0, 0);   // fullscreen triangle from SV_VertexID, no vertex buffer
    }

    // ---- to the swapchain, when there is one
    if (!m_gpu->Headless()) {
        ID3D12Resource* bb = m_gpu->BackBuffer();
        D3D12_RESOURCE_BARRIER toCopy[2]{};
        toCopy[0].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        toCopy[0].Transition.pResource = bb;
        toCopy[0].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        toCopy[0].Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
        toCopy[0].Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_DEST;
        toCopy[1].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        toCopy[1].Transition.pResource = m_ldrTarget.res.Get();
        toCopy[1].Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        toCopy[1].Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
        toCopy[1].Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
        cl->ResourceBarrier(2, toCopy);
        m_ldrTarget.state = D3D12_RESOURCE_STATE_COPY_SOURCE;

        cl->CopyResource(bb, m_ldrTarget.res.Get());

        D3D12_RESOURCE_BARRIER toPresent{};
        toPresent.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        toPresent.Transition.pResource = bb;
        toPresent.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        toPresent.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
        toPresent.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
        cl->ResourceBarrier(1, &toPresent);
    }

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

}  // namespace ga
