// ================================================================================================
//  Pipeline.h - M12 step 3c: THE PIPELINE BUILDERS. The thirteen copied graphics-PSO blocks
//  (a 25-line D3D12_GRAPHICS_PIPELINE_STATE_DESC per layer, varying only in blend, depth, clip
//  and topology) become one description with the shared defaults, and the nine compute PSOs
//  one call.
//
//  THE DEFAULTS ARE THE SKY'S: opaque one/zero blend, write mask all, solid fill, cull none, no
//  depth clip, depth OFF, the HDR target (R16G16B16A16_FLOAT) and the reversed-Z depth format
//  (D32_FLOAT), triangle topology, one sample. A layer that differs says so in the fields it
//  differs in -- the limb's dual-source blend, depth GREATER + write and depth clip for the
//  surfaces, lines for the plots, HS/DS under a patch list for the sea -- and the D3D struct the
//  builder produces is exactly the struct the layer used to spell out. That equality WAS step
//  3c's gate: every block's hand-written struct against the builder's, bitwise, in one probe
//  run before the block was deleted. Every field the hardware reads matched at every site;
//  eight blocks differed only in fields D3D12 does not read (the blend factors under
//  BlendEnable FALSE, DepthFunc under DepthEnable FALSE), where the old blocks disagreed
//  among themselves -- there the builder says what the sky's block said.
//
//  RELOAD IS ONE LAW: build the new pipeline, and only on success swap it in. Every layer wrote
//  that rule by hand (`Com<ID3D12PipelineState> keep = m_pso; if (!Build()) m_pso = keep;`);
//  Reload() is that rule, and the old pipeline stays alive until the last frame that recorded it
//  has retired (the frame ring holds the reference; step 3e's Retire says so once for every
//  such object).
//
//  DX12-first: the fields ARE the D3D enums. No enum of our own stands between a layer and the
//  hardware; the builder only fills in what nobody wants to type nine times.
// ================================================================================================
#pragma once

#include "hal/Gpu.h"
#include "hal/Shader.h"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ga::hal {

struct GraphicsPipelineDesc {
    ID3D12RootSignature* rootSig = nullptr;
    ShaderBlob vs, ps;
    ShaderBlob hs, ds;   // the tessellated sea: both set, under the patch-list topology
    // blend (render target 0)
    bool blend = false;
    D3D12_BLEND srcBlend = D3D12_BLEND_ONE, dstBlend = D3D12_BLEND_ZERO;
    D3D12_BLEND_OP blendOp = D3D12_BLEND_OP_ADD;
    D3D12_BLEND srcBlendAlpha = D3D12_BLEND_ONE, dstBlendAlpha = D3D12_BLEND_ZERO;
    D3D12_BLEND_OP blendOpAlpha = D3D12_BLEND_OP_ADD;
    UINT8 writeMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    // rasterizer
    D3D12_FILL_MODE fill = D3D12_FILL_MODE_SOLID;
    D3D12_CULL_MODE cull = D3D12_CULL_MODE_NONE;
    BOOL frontCcw = FALSE;
    // The sky's: a backdrop has no near or far. The surfaces, the tonemap, the markers and the
    // hulls say TRUE -- eight of the thirteen blocks did, so the default is the minority's.
    BOOL depthClip = FALSE;
    // depth: OFF by default (the backdrops); the surfaces say GREATER + write (reversed-Z)
    bool depthTest = false;
    bool depthWrite = false;
    D3D12_COMPARISON_FUNC depthFunc = D3D12_COMPARISON_FUNC_GREATER;
    DXGI_FORMAT dsvFormat = DXGI_FORMAT_D32_FLOAT;
    // targets
    uint32_t numRenderTargets = 1;
    DXGI_FORMAT rtvFormat = DXGI_FORMAT_R16G16B16A16_FLOAT;
    D3D12_PRIMITIVE_TOPOLOGY_TYPE topology = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    uint32_t sampleCount = 1;

    // The D3D struct, filled from the fields above. A layer with a need the fields do not
    // cover takes this, edits it, and calls BuildGraphicsRaw -- the 10 % path, still one call.
    D3D12_GRAPHICS_PIPELINE_STATE_DESC ToDesc() const {
        D3D12_GRAPHICS_PIPELINE_STATE_DESC d{};
        d.pRootSignature = rootSig;
        d.VS = {vs.Data(), vs.Size()};
        d.PS = {ps.Data(), ps.Size()};
        if (hs.Valid()) d.HS = {hs.Data(), hs.Size()};
        if (ds.Valid()) d.DS = {ds.Data(), ds.Size()};
        auto& rt = d.BlendState.RenderTarget[0];
        rt.BlendEnable = blend ? TRUE : FALSE;
        rt.SrcBlend = srcBlend;
        rt.DestBlend = dstBlend;
        rt.BlendOp = blendOp;
        rt.SrcBlendAlpha = srcBlendAlpha;
        rt.DestBlendAlpha = dstBlendAlpha;
        rt.BlendOpAlpha = blendOpAlpha;
        rt.RenderTargetWriteMask = writeMask;
        d.SampleMask = UINT_MAX;
        d.RasterizerState.FillMode = fill;
        d.RasterizerState.CullMode = cull;
        d.RasterizerState.FrontCounterClockwise = frontCcw;
        d.RasterizerState.DepthClipEnable = depthClip;
        d.DepthStencilState.DepthEnable = depthTest ? TRUE : FALSE;
        d.DepthStencilState.DepthWriteMask =
            depthWrite ? D3D12_DEPTH_WRITE_MASK_ALL : D3D12_DEPTH_WRITE_MASK_ZERO;
        // A comparison exists only where a test does: the sky's block never states one, and
        // D3D12 reads DepthFunc only under DepthEnable.
        if (depthTest) d.DepthStencilState.DepthFunc = depthFunc;
        d.DSVFormat = dsvFormat;
        d.PrimitiveTopologyType = topology;
        d.NumRenderTargets = numRenderTargets;
        d.RTVFormats[0] = rtvFormat;
        d.SampleDesc.Count = sampleCount;
        return d;
    }
};

// Builds, or logs the HRESULT under `tag` and returns null. Never throws: Init decides whether
// a missing pipeline is fatal (it is, at boot) and Reload decides to keep the old one.
inline Com<ID3D12PipelineState> BuildGraphicsRaw(Gpu& gpu,
                                                 const D3D12_GRAPHICS_PIPELINE_STATE_DESC& d,
                                                 const char* tag) {
    Com<ID3D12PipelineState> pso;
    const HRESULT hr = gpu.Device()->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso));
    if (FAILED(hr)) {
        Log("[%s] PSO: %s", tag, HrString(hr).c_str());
        return nullptr;
    }
    return pso;
}
inline Com<ID3D12PipelineState> BuildGraphics(Gpu& gpu, const GraphicsPipelineDesc& desc,
                                              const char* tag) {
    if (!desc.vs.Valid() || !desc.ps.Valid()) return nullptr;   // the compiler already logged
    // A patch list needs both tessellation stages: the sea's block refused to build without
    // them, and so does this -- the same failure at the same point, not a device error.
    if (desc.topology == D3D12_PRIMITIVE_TOPOLOGY_TYPE_PATCH &&
        (!desc.hs.Valid() || !desc.ds.Valid()))
        return nullptr;
    return BuildGraphicsRaw(gpu, desc.ToDesc(), tag);
}
inline Com<ID3D12PipelineState> BuildCompute(Gpu& gpu, ID3D12RootSignature* rootSig,
                                             const ShaderBlob& cs, const char* tag) {
    if (!cs.Valid()) return nullptr;
    D3D12_COMPUTE_PIPELINE_STATE_DESC d{};
    d.pRootSignature = rootSig;
    d.CS = {cs.Data(), cs.Size()};
    Com<ID3D12PipelineState> pso;
    const HRESULT hr = gpu.Device()->CreateComputePipelineState(&d, IID_PPV_ARGS(&pso));
    if (FAILED(hr)) {
        Log("[%s] compute PSO: %s", tag, HrString(hr).c_str());
        return nullptr;
    }
    return pso;
}

// THE RELOAD LAW: build the new one; only on success does it replace the old. `build` returns
// a Com that is null on failure. Returns whether the swap happened.
template <class Build>
bool Reload(Com<ID3D12PipelineState>& pso, Build build, const char* tag) {
    Com<ID3D12PipelineState> fresh = build();
    if (!fresh) {
        // A first build has nothing to keep: the builder or the compiler has said why, and
        // Init decides what a missing pipeline means.
        if (pso) Log("[%s] reload failed; keeping the previous PSO", tag);
        return false;
    }
    pso = std::move(fresh);
    return true;
}

// THE SAME LAW FOR A SET. A layer whose passes must agree (the tessellated sea and its spectrum
// plot, the tide's ribbon and curves, the four FFT kernels) builds every replacement first and
// swaps all of them or none: Add() refuses a null build, Commit() swaps what was added.
class ReloadSet {
public:
    bool Add(Com<ID3D12PipelineState>& slot, Com<ID3D12PipelineState> fresh) {
        if (!fresh) return false;
        m_set.emplace_back(&slot, std::move(fresh));
        return true;
    }
    void Commit() {
        for (auto& [slot, fresh] : m_set) *slot = std::move(fresh);
        m_set.clear();
    }

private:
    std::vector<std::pair<Com<ID3D12PipelineState>*, Com<ID3D12PipelineState>>> m_set;
};

// BOOT POLICY: a pipeline that must exist. The Init sites that GA_CHECKed the create call keep
// their fatality here; `what` names the kernel the way their throw did ("Swe kernel").
inline Com<ID3D12PipelineState> Require(Com<ID3D12PipelineState> pso, const char* what) {
    if (!pso) throw std::runtime_error(std::string(what) + " failed");
    return pso;
}

}  // namespace ga::hal
