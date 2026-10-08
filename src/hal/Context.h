// ================================================================================================
//  Context.h - M12 step 3b: THE COMMAND CONTEXT. One object a layer records into, instead of a
//  raw command list plus the device it belongs to.
//
//  WHAT IT IS. A thin, concrete Direct3D 12 convenience -- DX12-first and DX12-only, by the
//  owner's ruling: this file exists to make a layer's life easier, not to make the engine
//  portable. It wraps the one command list the frame ring hands out with the operations the
//  1,273 raw call sites actually repeat: a barrier on a tracked texture, a typed push of
//  constants into a root CBV, a pipeline bind, a draw, a dispatch, the render-target and
//  viewport setup the renderer used to spell out. Nothing here is virtual and nothing hides
//  the API: Native() is the raw list, named so every use of it is a call site the lint counts
//  and a facade method that does not exist yet.
//
//  WHY A LAYER STOPS SEEING ID3D12GraphicsCommandList. FrameContext used to carry the raw list
//  and every one of the nine layers, the residency manager, the profiler, the atlas, the FFT
//  and the solver took it in their signatures -- 28 header lines, the widest Direct3D leak in
//  the tree. With the context in their place the plugin surface (scene/Component.h, step 5)
//  carries no hardware type at all, and the rule "Direct3D lives in src/hal/" (tools/hal_lint.py)
//  becomes checkable instead of remembered.
//
//  TEMPORAL OWNERSHIP (the review's point, made a contract in step 3e): a context is
//  valid for ONE recording -- the frame ring's list between BeginFrame and EndFrame, or the
//  immediate-upload list between BeginUpload and EndUpload -- and it says which by Who().
//  Nothing recorded through it survives the submission it belongs to; anything a recorded
//  command still reads (an upload allocation, a replaced pipeline, a tile being NULL-mapped)
//  retires on the fence, never on scope exit.
// ================================================================================================
#pragma once

#include "hal/Gpu.h"

#include <cstdint>
#include <initializer_list>
#include <stdexcept>

namespace ga::hal {

enum class Owner : uint8_t { Frame, Upload };

class CommandContext {
public:
    CommandContext(Gpu& gpu, ID3D12GraphicsCommandList* cl, Owner owner)
        : m_gpu(&gpu), m_cl(cl), m_owner(owner) {}

    Gpu& Device() const { return *m_gpu; }
    Owner Who() const { return m_owner; }
    // THE ESCAPE HATCH, named. The exotic sites (the query heap, the two capability probes,
    // DirectStorage) reach the list here; so does every site a later sub-step has not yet
    // given a method to. Count them: tools/hal_lint.py does.
    ID3D12GraphicsCommandList* Native() const { return m_cl; }

    // ---- state ------------------------------------------------------------------------------
    // A tracked texture (GpuTexture carries its state): the transition the device layer already
    // wrote, through the context so the layer never spells a barrier.
    void Barrier(GpuTexture& tex, D3D12_RESOURCE_STATES to) { m_gpu->Transition(m_cl, tex, to); }
    // An untracked resource (the residency arrays, a borrowed page): both states said.
    void Barrier(ID3D12Resource* res, D3D12_RESOURCE_STATES from, D3D12_RESOURCE_STATES to) {
        if (from == to) return;
        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource = res;
        b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        b.Transition.StateBefore = from;
        b.Transition.StateAfter = to;
        m_cl->ResourceBarrier(1, &b);
    }
    void UavBarrier(ID3D12Resource* res = nullptr) {
        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
        b.UAV.pResource = res;
        m_cl->ResourceBarrier(1, &b);
    }
    // Several UAV barriers in ONE ResourceBarrier call (M12 step 3f; the solver's three after
    // its clears): the same n barriers in the same order, one call, as the site spelled them.
    static constexpr uint32_t kMaxBarriers = 8;
    void UavBarriers(std::initializer_list<ID3D12Resource*> resources) {
        if (resources.size() > kMaxBarriers) throw std::runtime_error("UavBarriers: too many");
        D3D12_RESOURCE_BARRIER b[kMaxBarriers]{};
        UINT n = 0;
        for (ID3D12Resource* r : resources) {
            b[n].Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
            b[n].UAV.pResource = r;
            ++n;
        }
        m_cl->ResourceBarrier(n, b);
    }

    // ---- binding ----------------------------------------------------------------------------
    void Pipeline(ID3D12PipelineState* pso) { m_cl->SetPipelineState(pso); }
    void GraphicsRoot(ID3D12RootSignature* rs) { m_cl->SetGraphicsRootSignature(rs); }
    void ComputeRoot(ID3D12RootSignature* rs) { m_cl->SetComputeRootSignature(rs); }
    // Constants: the per-frame linear arena's push and the root CBV bind, as one call. The POD
    // is the layer's own cbuffer mirror; DxTest's parity gate is what keeps it honest.
    template <class T>
    D3D12_GPU_VIRTUAL_ADDRESS Push(const T& pod) {
        return m_gpu->PushConstants(&pod, sizeof(T));
    }
    template <class T>
    void GraphicsConstants(uint32_t slot, const T& pod) {
        m_cl->SetGraphicsRootConstantBufferView(slot, Push(pod));
    }
    template <class T>
    void ComputeConstants(uint32_t slot, const T& pod) {
        m_cl->SetComputeRootConstantBufferView(slot, Push(pod));
    }
    void GraphicsConstantsAt(uint32_t slot, D3D12_GPU_VIRTUAL_ADDRESS va) {
        m_cl->SetGraphicsRootConstantBufferView(slot, va);
    }
    void ComputeConstantsAt(uint32_t slot, D3D12_GPU_VIRTUAL_ADDRESS va) {
        m_cl->SetComputeRootConstantBufferView(slot, va);
    }
    void GraphicsSrvAt(uint32_t slot, D3D12_GPU_VIRTUAL_ADDRESS va) {
        m_cl->SetGraphicsRootShaderResourceView(slot, va);
    }
    void ComputeSrvAt(uint32_t slot, D3D12_GPU_VIRTUAL_ADDRESS va) {
        m_cl->SetComputeRootShaderResourceView(slot, va);
    }
    // The bindless table: the whole shader-visible heap from slot 0, as the shared root
    // signature declares it (Renderer.h); or a table starting at a given heap slot.
    void GraphicsBindless(uint32_t rootParam) {
        m_cl->SetGraphicsRootDescriptorTable(rootParam, m_gpu->SrvHeap().Gpu(0));
    }
    void ComputeBindless(uint32_t rootParam) {
        m_cl->SetComputeRootDescriptorTable(rootParam, m_gpu->SrvHeap().Gpu(0));
    }
    void GraphicsTable(uint32_t rootParam, uint32_t heapSlot) {
        m_cl->SetGraphicsRootDescriptorTable(rootParam, m_gpu->SrvHeap().Gpu(heapSlot));
    }
    void ComputeTable(uint32_t rootParam, uint32_t heapSlot) {
        m_cl->SetComputeRootDescriptorTable(rootParam, m_gpu->SrvHeap().Gpu(heapSlot));
    }
    // The shader-visible heap, bound to this list (M12 step 3d). The frame list gets it at
    // BeginFrame; the upload list has none until a layer says so (the cloud and wind builds,
    // the gulf's gradient), and the solver's spin-up records on whichever list it is handed --
    // re-setting the same heap on the frame list is a no-op the runtime allows. The one heap
    // every table above indexes.
    void BindHeaps() {
        ID3D12DescriptorHeap* heaps[] = {m_gpu->SrvHeap().Heap()};
        m_cl->SetDescriptorHeaps(1, heaps);
    }

    // ---- work -------------------------------------------------------------------------------
    void Topology(D3D12_PRIMITIVE_TOPOLOGY t) { m_cl->IASetPrimitiveTopology(t); }
    void Draw(uint32_t vertices, uint32_t instances = 1, uint32_t firstVertex = 0,
              uint32_t firstInstance = 0) {
        m_cl->DrawInstanced(vertices, instances, firstVertex, firstInstance);
    }
    // A fullscreen triangle from SV_VertexID, no vertex buffer -- the tonemap, the sky, the limb.
    void DrawFullscreen() {
        m_cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        m_cl->DrawInstanced(3, 1, 0, 0);
    }
    void Dispatch(uint32_t x, uint32_t y = 1, uint32_t z = 1) { m_cl->Dispatch(x, y, z); }
    // Mesh shaders (M12 step 3f). DispatchMesh is ID3D12GraphicsCommandList6's; the Gpu keeps
    // the frame list's interface, queried once when the list was created (Gpu::MeshList), and
    // hands it back for this context's list -- null for the upload list or a runtime without
    // it. MeshCapable() is the globe's test before its first dispatch (no interface: the
    // classic path), as its own QueryInterface was.
    bool MeshCapable() const { return m_gpu->MeshList(m_cl) != nullptr; }
    void DispatchMesh(uint32_t x, uint32_t y = 1, uint32_t z = 1) {
        ID3D12GraphicsCommandList6* cl6 = m_gpu->MeshList(m_cl);
        if (!cl6) throw std::runtime_error("DispatchMesh: no List6 for this list (MeshCapable)");
        cl6->DispatchMesh(x, y, z);
    }

    // ---- targets and the viewport -------------------------------------------------------------
    void Targets(D3D12_CPU_DESCRIPTOR_HANDLE rtv, const D3D12_CPU_DESCRIPTOR_HANDLE* dsv) {
        m_cl->OMSetRenderTargets(1, &rtv, FALSE, dsv);
    }
    void ClearColor(D3D12_CPU_DESCRIPTOR_HANDLE rtv, const float rgba[4]) {
        m_cl->ClearRenderTargetView(rtv, rgba, 0, nullptr);
    }
    // Reversed-Z: the far plane is 0.
    void ClearDepth(D3D12_CPU_DESCRIPTOR_HANDLE dsv, float depth = 0.0f) {
        m_cl->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, depth, 0, 0, nullptr);
    }
    void Viewport(uint32_t w, uint32_t h) { Viewport(0, 0, w, h); }
    // A rectangle of the target: the viewport maps clip space onto it and the scissor keeps every
    // pixel inside it, so a view drawn into a corner cannot spill out of its corner.
    void Viewport(uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
        const D3D12_VIEWPORT vp{static_cast<float>(x), static_cast<float>(y), static_cast<float>(w),
                                static_cast<float>(h), 0.0f, 1.0f};
        const D3D12_RECT sc{static_cast<LONG>(x), static_cast<LONG>(y), static_cast<LONG>(x + w),
                            static_cast<LONG>(y + h)};
        m_cl->RSSetViewports(1, &vp);
        m_cl->RSSetScissorRects(1, &sc);
    }
    // The clears of one rectangle: a second view starts from far depth and black in its own
    // rectangle and leaves the rest of the target as the views before it drew it.
    void ClearColor(D3D12_CPU_DESCRIPTOR_HANDLE rtv, const float rgba[4], const D3D12_RECT& r) {
        m_cl->ClearRenderTargetView(rtv, rgba, 1, &r);
    }
    void ClearDepth(D3D12_CPU_DESCRIPTOR_HANDLE dsv, float depth, const D3D12_RECT& r) {
        m_cl->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH, depth, 0, 1, &r);
    }

    // ---- copies -----------------------------------------------------------------------------
    void Copy(ID3D12Resource* dst, ID3D12Resource* src) { m_cl->CopyResource(dst, src); }

private:
    Gpu* m_gpu;
    ID3D12GraphicsCommandList* m_cl;
    Owner m_owner;
};

}  // namespace ga::hal
