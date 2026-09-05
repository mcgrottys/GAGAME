#include "core/Gpu.h"

#include <algorithm>
#include <cstdio>
#include <string>

namespace ga {

// ================================================================================ DescriptorHeap

void DescriptorHeap::Init(ID3D12Device* dev, D3D12_DESCRIPTOR_HEAP_TYPE type, uint32_t capacity,
                          bool shaderVisible, const wchar_t* name) {
    D3D12_DESCRIPTOR_HEAP_DESC d{};
    d.Type = type;
    d.NumDescriptors = capacity;
    d.Flags = shaderVisible ? D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE
                            : D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
    GA_CHECK(dev->CreateDescriptorHeap(&d, IID_PPV_ARGS(&m_heap)));
    if (name) m_heap->SetName(name);
    m_stride = dev->GetDescriptorHandleIncrementSize(type);
    m_cpuStart = m_heap->GetCPUDescriptorHandleForHeapStart();
    if (shaderVisible) m_gpuStart = m_heap->GetGPUDescriptorHandleForHeapStart();
    m_capacity = capacity;
    m_used = 0;
    m_shaderVisible = shaderVisible;
}

uint32_t DescriptorHeap::Alloc(uint32_t count) {
    if (m_used + count > m_capacity) {
        throw std::runtime_error("descriptor heap exhausted");
    }
    uint32_t base = m_used;
    m_used += count;
    return base;
}

D3D12_CPU_DESCRIPTOR_HANDLE DescriptorHeap::Cpu(uint32_t index) const {
    D3D12_CPU_DESCRIPTOR_HANDLE h = m_cpuStart;
    h.ptr += static_cast<SIZE_T>(index) * m_stride;
    return h;
}

D3D12_GPU_DESCRIPTOR_HANDLE DescriptorHeap::Gpu(uint32_t index) const {
    D3D12_GPU_DESCRIPTOR_HANDLE h = m_gpuStart;
    h.ptr += static_cast<UINT64>(index) * m_stride;
    return h;
}

// ================================================================================ Gpu init

void Gpu::Init(HWND hwnd, uint32_t width, uint32_t height, bool wantDebugLayer,
               bool allowTearing) {
    m_hwnd = hwnd;
    m_width = std::max(width, 1u);
    m_height = std::max(height, 1u);
    m_wantTearing = allowTearing && hwnd != nullptr;

    UINT factoryFlags = 0;
    if (wantDebugLayer) {
        // Needs the "Graphics Tools" optional Windows feature. Absent on plenty of machines, so a
        // failure here is informational, not fatal.
        Com<ID3D12Debug> dbg;
        if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&dbg)))) {
            dbg->EnableDebugLayer();
            factoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
            Log("[gpu] D3D12 debug layer enabled");
        } else {
            Log("[gpu] debug layer unavailable (install the Graphics Tools feature); continuing");
        }
    }

    GA_CHECK(CreateDXGIFactory2(factoryFlags, IID_PPV_ARGS(&m_factory)));

    // Highest-performance adapter that can actually make a device. Iterating rather than taking
    // adapter 0 matters on machines with an integrated GPU present (this one: RTX 5060 + 780M).
    Com<IDXGIAdapter1> adapter;
    DXGI_ADAPTER_DESC1 chosen{};
    UINT chosenRank = 0;
    for (UINT i = 0; m_factory->EnumAdapterByGpuPreference(
                         i, DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE,
                         IID_PPV_ARGS(&adapter)) != DXGI_ERROR_NOT_FOUND;
         ++i) {
        DXGI_ADAPTER_DESC1 ad{};
        adapter->GetDesc1(&ad);
        if (ad.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) { adapter = nullptr; continue; }
        if (SUCCEEDED(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_12_0,
                                        IID_PPV_ARGS(&m_device)))) {
            m_dedicatedVram = ad.DedicatedVideoMemory;
            chosen = ad;
            chosenRank = i;
            Log("[gpu] adapter: %S  (%llu MB dedicated)", ad.Description,
                static_cast<uint64_t>(ad.DedicatedVideoMemory / (1024 * 1024)));
            break;
        }
        adapter = nullptr;
    }
    if (!m_device) throw std::runtime_error("no D3D12 feature level 12.0 adapter found");

    // ---- BOOT REPORT: which adapter, whether it is the high-performance pick, and whether it
    // owns a display output. On a hybrid laptop the panel usually hangs off the integrated GPU;
    // a swapchain on the discrete GPU is then a CROSS-ADAPTER present (DWM copies every frame
    // to the adapter that owns the output), which is a different animal from a local flip.
    {
        Log("[gpu] boot: chosen adapter '%S' -- rank %u in DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE "
            "order (%s), vendor 0x%04x device 0x%04x, %llu MB dedicated / %llu MB shared",
            chosen.Description, chosenRank,
            chosenRank == 0 ? "IS the high-performance adapter"
                            : "NOT the top pick: a higher-ranked adapter failed FL 12_0",
            chosen.VendorId, chosen.DeviceId,
            static_cast<unsigned long long>(chosen.DedicatedVideoMemory >> 20),
            static_cast<unsigned long long>(chosen.SharedSystemMemory >> 20));
        auto countOutputs = [](IDXGIAdapter1* a, std::string* names) {
            UINT n = 0;
            Com<IDXGIOutput> out;
            for (UINT k = 0; a->EnumOutputs(k, &out) != DXGI_ERROR_NOT_FOUND; ++k, out = nullptr) {
                DXGI_OUTPUT_DESC od{};
                if (SUCCEEDED(out->GetDesc(&od))) {
                    char buf[160];
                    snprintf(buf, sizeof(buf), "%s%S %ldx%ld", n ? ", " : "", od.DeviceName,
                             static_cast<long>(od.DesktopCoordinates.right -
                                               od.DesktopCoordinates.left),
                             static_cast<long>(od.DesktopCoordinates.bottom -
                                               od.DesktopCoordinates.top));
                    if (names) *names += buf;
                }
                ++n;
            }
            return n;
        };
        std::string mine;
        const UINT myOutputs = countOutputs(adapter.Get(), &mine);
        Log("[gpu] boot: chosen adapter owns %u DXGI output%s%s%s", myOutputs,
            myOutputs == 1 ? "" : "s", myOutputs ? ": " : "", mine.c_str());
        // Who owns the outputs, then.
        std::string ownerName;
        UINT ownerOutputs = 0;
        Com<IDXGIAdapter1> other;
        for (UINT i = 0; m_factory->EnumAdapters1(i, &other) != DXGI_ERROR_NOT_FOUND;
             ++i, other = nullptr) {
            DXGI_ADAPTER_DESC1 od{};
            other->GetDesc1(&od);
            if (od.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) continue;
            std::string names;
            const UINT n = countOutputs(other.Get(), &names);
            const bool same = od.AdapterLuid.LowPart == chosen.AdapterLuid.LowPart &&
                              od.AdapterLuid.HighPart == chosen.AdapterLuid.HighPart;
            Log("[gpu] boot:   adapter %u '%S'%s: %u output%s%s%s", i, od.Description,
                same ? " (chosen)" : "", n, n == 1 ? "" : "s", n ? " -- " : "", names.c_str());
            if (!same && n) {
                char buf[256];
                snprintf(buf, sizeof(buf), "%S", od.Description);
                ownerName = buf;
                ownerOutputs += n;
            }
        }
        if (!m_hwnd) {
            Log("[gpu] boot: headless -- no swapchain, so no present at all");
        } else if (myOutputs) {
            Log("[gpu] boot: present is SAME-ADAPTER -- the swapchain's adapter owns the "
                "output it presents to");
        } else {
            Log("[gpu] boot: present is CROSS-ADAPTER -- the swapchain's adapter ('%S') owns NO "
                "display output; the panel belongs to '%s' (%u output%s). Every frame is "
                "rendered on the %S, copied across to the %s by DWM, and flipped there.",
                chosen.Description, ownerName.empty() ? "(no other adapter found)" : ownerName.c_str(),
                ownerOutputs, ownerOutputs == 1 ? "" : "s", chosen.Description,
                ownerName.empty() ? "?" : ownerName.c_str());
        }
    }

    // Resource binding tier 3 makes the unbounded SRV table legal. The tiled-resources tier is the
    // M0 question: >= TIER_2 is guaranteed on FL 12_0 hardware per the D3D12 docs, and TIER_2 is
    // what gives NULL-mapped tiles their read-as-zero / writes-discarded semantics. The self-test
    // in TileAtlas.cpp verifies the guarantee empirically rather than trusting the enum.
    D3D12_FEATURE_DATA_D3D12_OPTIONS opts{};
    if (SUCCEEDED(m_device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS, &opts, sizeof(opts)))) {
        m_tiledTier = opts.TiledResourcesTier;
        Log("[gpu] resource binding tier %d, tiled resources tier %d",
            static_cast<int>(opts.ResourceBindingTier), static_cast<int>(opts.TiledResourcesTier));
        if (opts.ResourceBindingTier < D3D12_RESOURCE_BINDING_TIER_3) {
            Log("[gpu] WARNING: binding tier < 3; the unbounded SRV table may not work");
        }
        if (opts.TiledResourcesTier < D3D12_TILED_RESOURCES_TIER_2) {
            Log("[gpu] WARNING: tiled tier < 2; null-tile reads are UNDEFINED on this adapter");
        }
    }

    D3D12_FEATURE_DATA_D3D12_OPTIONS7 o7{};
    if (SUCCEEDED(m_device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &o7, sizeof(o7)))) {
        Log("[gpu] mesh shader tier %d, sampler feedback tier %d",
            static_cast<int>(o7.MeshShaderTier), static_cast<int>(o7.SamplerFeedbackTier));
    }
    D3D12_FEATURE_DATA_SHADER_MODEL sm{D3D_SHADER_MODEL_6_7};
    while (sm.HighestShaderModel > D3D_SHADER_MODEL_5_1 &&
           FAILED(m_device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &sm, sizeof(sm)))) {
        sm.HighestShaderModel = static_cast<D3D_SHADER_MODEL>(sm.HighestShaderModel - 1);
    }
    Log("[gpu] highest shader model 0x%02x", static_cast<int>(sm.HighestShaderModel));

    D3D12_COMMAND_QUEUE_DESC qd{};
    qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
    GA_CHECK(m_device->CreateCommandQueue(&qd, IID_PPV_ARGS(&m_queue)));
    m_queue->SetName(L"main queue");

    m_srvHeap.Init(m_device.Get(), D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, kSrvHeapCapacity, true,
                   L"srv heap");
    m_rtvHeap.Init(m_device.Get(), D3D12_DESCRIPTOR_HEAP_TYPE_RTV, 64, false, L"rtv heap");
    m_dsvHeap.Init(m_device.Get(), D3D12_DESCRIPTOR_HEAP_TYPE_DSV, 16, false, L"dsv heap");

    if (m_hwnd) CreateSwapchain();
    CreateFrameResources();

    GA_CHECK(m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_fence)));
    m_fenceEvent = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    if (!m_fenceEvent) throw std::runtime_error("CreateEvent failed");

    GA_CHECK(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
                                             IID_PPV_ARGS(&m_uploadAlloc)));
    GA_CHECK(m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, m_uploadAlloc.Get(),
                                         nullptr, IID_PPV_ARGS(&m_uploadList)));
    m_uploadList->Close();
}

void Gpu::CreateSwapchain() {
    DXGI_SWAP_CHAIN_DESC1 sd{};
    sd.BufferCount = kFrameCount;
    sd.Width = m_width;
    sd.Height = m_height;
    sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
    sd.SampleDesc.Count = 1;

    // --no-vsync: tearing is a swapchain CREATION flag as well as a Present flag, and only
    // legal when the factory says the OS/driver pair supports it.
    m_tearing = false;
    if (m_wantTearing) {
        BOOL allow = FALSE;
        if (SUCCEEDED(m_factory->CheckFeatureSupport(DXGI_FEATURE_PRESENT_ALLOW_TEARING, &allow,
                                                     sizeof(allow))) &&
            allow) {
            sd.Flags |= DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;
            m_tearing = true;
            Log("[gpu] boot: present mode: --no-vsync -> DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING + "
                "Present(0, DXGI_PRESENT_ALLOW_TEARING)");
        } else {
            Log("[gpu] boot: present mode: --no-vsync requested but "
                "DXGI_FEATURE_PRESENT_ALLOW_TEARING is unsupported here; falling back to "
                "Present(1, 0)");
        }
    } else {
        Log("[gpu] boot: present mode: vsync, Present(1, 0), FLIP_DISCARD x%u buffers, no "
            "waitable object, no latency setting",
            kFrameCount);
    }

    Com<IDXGISwapChain1> sc1;
    GA_CHECK(m_factory->CreateSwapChainForHwnd(m_queue.Get(), m_hwnd, &sd, nullptr, nullptr, &sc1));
    GA_CHECK(m_factory->MakeWindowAssociation(m_hwnd, DXGI_MWA_NO_ALT_ENTER));
    GA_CHECK(sc1.As(&m_swapchain));
    m_frameIndex = m_swapchain->GetCurrentBackBufferIndex();

    for (uint32_t i = 0; i < kFrameCount; ++i) {
        GA_CHECK(m_swapchain->GetBuffer(i, IID_PPV_ARGS(&m_backBuffers[i])));
        m_backBufferRtv[i] = m_rtvHeap.Alloc();
        m_device->CreateRenderTargetView(m_backBuffers[i].Get(), nullptr,
                                         m_rtvHeap.Cpu(m_backBufferRtv[i]));
    }
}

void Gpu::CreateFrameResources() {
    for (uint32_t i = 0; i < kFrameCount; ++i) {
        GA_CHECK(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
                                                 IID_PPV_ARGS(&m_alloc[i])));
        m_cbArena[i] = CreateUploadBuffer(4 * 1024 * 1024, L"cb arena");
    }
    GA_CHECK(m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, m_alloc[0].Get(),
                                         nullptr, IID_PPV_ARGS(&m_cmdList)));
    m_cmdList->Close();
}

void Gpu::Shutdown() {
    if (m_device) WaitIdle();
    if (m_fenceEvent) { CloseHandle(m_fenceEvent); m_fenceEvent = nullptr; }
}

// ================================================================================ frame lifecycle

ID3D12GraphicsCommandList* Gpu::BeginFrame() {
    // Wait only for the frame we are about to reuse, not for the GPU to go idle.
    if (m_frameFence[m_frameIndex] != 0 &&
        m_fence->GetCompletedValue() < m_frameFence[m_frameIndex]) {
        GA_CHECK(m_fence->SetEventOnCompletion(m_frameFence[m_frameIndex], m_fenceEvent));
        WaitForSingleObject(m_fenceEvent, INFINITE);
    }
    m_cbOffset[m_frameIndex] = 0;
    GA_CHECK(m_alloc[m_frameIndex]->Reset());
    GA_CHECK(m_cmdList->Reset(m_alloc[m_frameIndex].Get(), nullptr));

    ID3D12DescriptorHeap* heaps[] = {m_srvHeap.Heap()};
    m_cmdList->SetDescriptorHeaps(1, heaps);
    return m_cmdList.Get();
}

void Gpu::EndFrame(bool present) {
    GA_CHECK(m_cmdList->Close());
    ID3D12CommandList* lists[] = {m_cmdList.Get()};
    m_queue->ExecuteCommandLists(1, lists);

    if (present && m_swapchain) {
        GA_CHECK(m_tearing ? m_swapchain->Present(0, DXGI_PRESENT_ALLOW_TEARING)
                           : m_swapchain->Present(1, 0));
        ++m_presents;
        if (!m_presentStats0Valid && m_presents >= 8) {
            DXGI_FRAME_STATISTICS fs{};
            if (SUCCEEDED(m_swapchain->GetFrameStatistics(&fs))) {
                m_presentStats0 = fs;
                m_presentStats0Valid = true;
            }
        }
    }

    m_frameFence[m_frameIndex] = ++m_fenceValue;
    GA_CHECK(m_queue->Signal(m_fence.Get(), m_fenceValue));

    m_frameIndex = m_swapchain ? m_swapchain->GetCurrentBackBufferIndex()
                               : (m_frameIndex + 1) % kFrameCount;
}

void Gpu::WaitIdle() {
    if (!m_queue || !m_fence) return;
    const uint64_t v = ++m_fenceValue;
    GA_CHECK(m_queue->Signal(m_fence.Get(), v));
    if (m_fence->GetCompletedValue() < v) {
        GA_CHECK(m_fence->SetEventOnCompletion(v, m_fenceEvent));
        WaitForSingleObject(m_fenceEvent, INFINITE);
    }
}

void Gpu::ReportPresentStats() {
    if (!m_swapchain) return;
    // The panel behind the window: its current mode's refresh rate is the denominator.
    // MEASURED on this laptop: the swapchain sits on the RTX, which owns no output, so
    // GetContainingOutput fails (cross-adapter present); the window's monitor names the panel
    // instead. The same composed present is why DXGI_FRAME_STATISTICS counts nothing (0
    // presents over 0 refreshes after 400 Present() calls): the flip happens on the 780M
    // under DWM, out of this swapchain's sight. Both cases are said rather than printed as 0.
    auto panelHz = [](const wchar_t* deviceName) {
        DEVMODEW dm{};
        dm.dmSize = sizeof(dm);
        return EnumDisplaySettingsW(deviceName, ENUM_CURRENT_SETTINGS, &dm)
                   ? static_cast<unsigned>(dm.dmDisplayFrequency)
                   : 0u;
    };
    std::string panel = "(unknown output)";
    unsigned hz = 0;
    {
        Com<IDXGIOutput> out;
        DXGI_OUTPUT_DESC od{};
        char buf[96];
        if (SUCCEEDED(m_swapchain->GetContainingOutput(&out)) && SUCCEEDED(out->GetDesc(&od))) {
            snprintf(buf, sizeof(buf), "%S", od.DeviceName);
            panel = buf;
            hz = panelHz(od.DeviceName);
        } else if (m_hwnd) {
            MONITORINFOEXW mi{};
            mi.cbSize = sizeof(mi);
            if (GetMonitorInfoW(MonitorFromWindow(m_hwnd, MONITOR_DEFAULTTONEAREST), &mi)) {
                snprintf(buf, sizeof(buf), "%S (the window's monitor; the swapchain's adapter "
                         "owns no output)", mi.szDevice);
                panel = buf;
                hz = panelHz(mi.szDevice);
            }
        }
    }
    DXGI_FRAME_STATISTICS fs{};
    const HRESULT hr = m_swapchain->GetFrameStatistics(&fs);
    if (FAILED(hr) || !m_presentStats0Valid) {
        Log("[present] %s at %u Hz: %u Present() calls; DXGI_FRAME_STATISTICS unavailable (%s) "
            "-- presents per refresh not measured",
            panel.c_str(), hz, m_presents,
            FAILED(hr) ? (hr == DXGI_ERROR_FRAME_STATISTICS_DISJOINT ? "disjoint" : "failed")
                       : "no first sample");
        return;
    }
    // Deltas since the first sample: PresentCount counts images that reached the monitor,
    // SyncRefreshCount the v-blanks that passed. Their ratio is what a viewer sees; under
    // Present(1,0) it cannot exceed 1, under tearing it can.
    const UINT shown = fs.PresentCount - m_presentStats0.PresentCount;
    const UINT refreshes = fs.SyncRefreshCount - m_presentStats0.SyncRefreshCount;
    if (shown == 0 && refreshes == 0) {
        Log("[present] %s at %u Hz: %u Present() calls (%s), but DXGI_FRAME_STATISTICS counted "
            "0 presents over 0 refreshes -- a composed (cross-adapter) present reports none, so "
            "presents per refresh is not measurable from this swapchain; PresentMon is the tool "
            "(probe P12)",
            panel.c_str(), hz, m_presents,
            m_tearing ? "Present(0, ALLOW_TEARING)" : "Present(1, 0)");
        return;
    }
    const double perRefresh = refreshes ? double(shown) / double(refreshes) : 0.0;
    Log("[present] %s at %u Hz: %u presents shown over %u refreshes = %.3f presents/refresh "
        "(~%.1f fps perceived); %u Present() calls, %s",
        panel.c_str(), hz, shown, refreshes, perRefresh, perRefresh * double(hz), m_presents,
        m_tearing ? "Present(0, ALLOW_TEARING)" : "Present(1, 0)");
}

void Gpu::Resize(uint32_t width, uint32_t height) {
    width = std::max(width, 1u);
    height = std::max(height, 1u);
    if (width == m_width && height == m_height) return;
    WaitIdle();
    m_width = width;
    m_height = height;
    if (!m_swapchain) return;
    for (auto& bb : m_backBuffers) bb.Reset();
    GA_CHECK(m_swapchain->ResizeBuffers(kFrameCount, m_width, m_height,
                                        DXGI_FORMAT_R8G8B8A8_UNORM,
                                        m_tearing ? DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING : 0));
    for (uint32_t i = 0; i < kFrameCount; ++i) {
        GA_CHECK(m_swapchain->GetBuffer(i, IID_PPV_ARGS(&m_backBuffers[i])));
        m_device->CreateRenderTargetView(m_backBuffers[i].Get(), nullptr,
                                         m_rtvHeap.Cpu(m_backBufferRtv[i]));
    }
    m_frameIndex = m_swapchain->GetCurrentBackBufferIndex();
}

ID3D12Resource* Gpu::BackBuffer() const {
    return m_swapchain ? m_backBuffers[m_frameIndex].Get() : nullptr;
}

D3D12_CPU_DESCRIPTOR_HANDLE Gpu::BackBufferRtv() const {
    return m_rtvHeap.Cpu(m_backBufferRtv[m_frameIndex]);
}

D3D12_GPU_VIRTUAL_ADDRESS Gpu::PushConstants(const void* data, size_t bytes) {
    const uint64_t aligned = AlignUp<uint64_t>(bytes, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);
    GpuBuffer& arena = m_cbArena[m_frameIndex];
    uint64_t& off = m_cbOffset[m_frameIndex];
    if (off + aligned > arena.size) throw std::runtime_error("constant arena exhausted this frame");
    memcpy(arena.cpu + off, data, bytes);
    const D3D12_GPU_VIRTUAL_ADDRESS va = arena.gpu + off;
    off += aligned;
    return va;
}

// ================================================================================ resources

static D3D12_HEAP_PROPERTIES HeapProps(D3D12_HEAP_TYPE t) {
    D3D12_HEAP_PROPERTIES p{};
    p.Type = t;
    p.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
    p.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
    return p;
}

static D3D12_RESOURCE_DESC BufferDesc(uint64_t bytes) {
    D3D12_RESOURCE_DESC d{};
    d.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    d.Width = bytes;
    d.Height = 1;
    d.DepthOrArraySize = 1;
    d.MipLevels = 1;
    d.Format = DXGI_FORMAT_UNKNOWN;
    d.SampleDesc.Count = 1;
    d.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    return d;
}

GpuBuffer Gpu::CreateUploadBuffer(uint64_t bytes, const wchar_t* name) {
    GpuBuffer b;
    b.size = bytes;
    const auto hp = HeapProps(D3D12_HEAP_TYPE_UPLOAD);
    const auto rd = BufferDesc(bytes);
    GA_CHECK(m_device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
                                              D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
                                              IID_PPV_ARGS(&b.res)));
    if (name) b.res->SetName(name);
    D3D12_RANGE none{0, 0};
    GA_CHECK(b.res->Map(0, &none, reinterpret_cast<void**>(&b.cpu)));
    b.gpu = b.res->GetGPUVirtualAddress();
    return b;
}

GpuBuffer Gpu::CreateDefaultBuffer(const void* data, uint64_t bytes, const wchar_t* name) {
    GpuBuffer b;
    b.size = bytes;
    const auto hp = HeapProps(D3D12_HEAP_TYPE_DEFAULT);
    const auto rd = BufferDesc(bytes);
    GA_CHECK(m_device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
                                              D3D12_RESOURCE_STATE_COMMON, nullptr,
                                              IID_PPV_ARGS(&b.res)));
    if (name) b.res->SetName(name);
    b.gpu = b.res->GetGPUVirtualAddress();

    if (data) {
        GpuBuffer staging = CreateUploadBuffer(bytes, L"staging");
        memcpy(staging.cpu, data, bytes);
        auto* cl = BeginUpload();
        cl->CopyBufferRegion(b.res.Get(), 0, staging.res.Get(), 0, bytes);
        D3D12_RESOURCE_BARRIER br{};
        br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        br.Transition.pResource = b.res.Get();
        br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        br.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
        br.Transition.StateAfter = D3D12_RESOURCE_STATE_COMMON;
        cl->ResourceBarrier(1, &br);
        m_uploadKeepAlive.push_back(staging.res);
        EndUpload();
    }
    return b;
}

GpuTexture Gpu::CreateTexture2D(uint32_t w, uint32_t h, DXGI_FORMAT fmt,
                                D3D12_RESOURCE_FLAGS flags,
                                D3D12_RESOURCE_STATES initialState, const wchar_t* name,
                                const D3D12_CLEAR_VALUE* clear, uint16_t mips) {
    GpuTexture t;
    t.width = w;
    t.height = h;
    t.format = fmt;
    t.state = initialState;

    D3D12_RESOURCE_DESC d{};
    d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    d.Width = w;
    d.Height = h;
    d.DepthOrArraySize = 1;
    d.MipLevels = mips;
    d.Format = fmt;
    d.SampleDesc.Count = 1;
    d.Flags = flags;

    const auto hp = HeapProps(D3D12_HEAP_TYPE_DEFAULT);
    GA_CHECK(m_device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, initialState, clear,
                                              IID_PPV_ARGS(&t.res)));
    if (name) t.res->SetName(name);
    return t;
}

void Gpu::UploadTexture(GpuTexture& tex, const void* rows, uint32_t srcRowPitchBytes,
                        uint32_t mip) {
    D3D12_RESOURCE_DESC d = tex.res->GetDesc();
    D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp{};
    UINT numRows = 0;
    UINT64 rowBytes = 0, total = 0;
    m_device->GetCopyableFootprints(&d, mip, 1, 0, &fp, &numRows, &rowBytes, &total);

    GpuBuffer staging = CreateUploadBuffer(total - fp.Offset, L"tex staging");
    const UINT64 base = fp.Offset;
    fp.Offset = 0;
    for (UINT y = 0; y < numRows; ++y) {
        memcpy(staging.cpu + static_cast<uint64_t>(y) * fp.Footprint.RowPitch,
               static_cast<const uint8_t*>(rows) + static_cast<uint64_t>(y) * srcRowPitchBytes,
               static_cast<size_t>(rowBytes));
    }
    (void)base;

    auto* cl = BeginUpload();
    D3D12_TEXTURE_COPY_LOCATION dst{}, src{};
    dst.pResource = tex.res.Get();
    dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    dst.SubresourceIndex = mip;
    src.pResource = staging.res.Get();
    src.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    src.PlacedFootprint = fp;
    cl->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);

    // Transition on the LAST mip only (mip uploads arrive 0..N-1; barriers per-subresource).
    D3D12_RESOURCE_BARRIER br{};
    br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    br.Transition.pResource = tex.res.Get();
    br.Transition.Subresource = mip;
    br.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    br.Transition.StateAfter = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
    cl->ResourceBarrier(1, &br);
    if (mip == 0) tex.state = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;

    m_uploadKeepAlive.push_back(staging.res);
    EndUpload();
}

uint32_t Gpu::CreateSrv(ID3D12Resource* res, DXGI_FORMAT fmt) {
    const uint32_t slot = m_srvHeap.Alloc();
    D3D12_SHADER_RESOURCE_VIEW_DESC s{};
    s.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    s.Format = fmt;
    s.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    s.Texture2D.MipLevels = UINT(-1);   // whole chain; identical behaviour for single-mip
    m_device->CreateShaderResourceView(res, &s, m_srvHeap.Cpu(slot));
    return slot;
}

uint32_t Gpu::CreateSrvArray(ID3D12Resource* res, DXGI_FORMAT fmt, uint32_t mips,
                             uint32_t slices) {
    const uint32_t slot = m_srvHeap.Alloc();
    D3D12_SHADER_RESOURCE_VIEW_DESC s{};
    s.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    s.Format = fmt;
    s.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DARRAY;
    s.Texture2DArray.MipLevels = mips;
    s.Texture2DArray.ArraySize = slices;
    m_device->CreateShaderResourceView(res, &s, m_srvHeap.Cpu(slot));
    return slot;
}

uint32_t Gpu::CreateSrv3D(ID3D12Resource* res, DXGI_FORMAT fmt) {
    const uint32_t slot = m_srvHeap.Alloc();
    D3D12_SHADER_RESOURCE_VIEW_DESC s{};
    s.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    s.Format = fmt;
    s.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
    s.Texture3D.MipLevels = 1;
    m_device->CreateShaderResourceView(res, &s, m_srvHeap.Cpu(slot));
    return slot;
}

uint32_t Gpu::CreateStructuredBufferSrv(ID3D12Resource* res, uint32_t numElements,
                                        uint32_t strideBytes) {
    const uint32_t slot = m_srvHeap.Alloc();
    D3D12_SHADER_RESOURCE_VIEW_DESC s{};
    s.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    s.Format = DXGI_FORMAT_UNKNOWN;
    s.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
    s.Buffer.NumElements = numElements;
    s.Buffer.StructureByteStride = strideBytes;
    m_device->CreateShaderResourceView(res, &s, m_srvHeap.Cpu(slot));
    return slot;
}

uint32_t Gpu::CreateTextureUav(ID3D12Resource* res, DXGI_FORMAT fmt, D3D12_UAV_DIMENSION dim,
                               uint32_t mipSlice) {
    const uint32_t slot = m_srvHeap.Alloc();
    D3D12_UNORDERED_ACCESS_VIEW_DESC u{};
    u.Format = fmt;
    u.ViewDimension = dim;
    // M9h: a mip-chained reserved resource is written one LEVEL at a time, so a UAV has to be
    // able to name which. Texture2D defaults to slice 0, which is every pre-chain call site.
    if (dim == D3D12_UAV_DIMENSION_TEXTURE2D) u.Texture2D.MipSlice = mipSlice;
    if (dim == D3D12_UAV_DIMENSION_TEXTURE3D) {
        u.Texture3D.MipSlice = mipSlice;
        u.Texture3D.FirstWSlice = 0;
        u.Texture3D.WSize = UINT(-1);   // all depth slices
    }
    m_device->CreateUnorderedAccessView(res, nullptr, &u, m_srvHeap.Cpu(slot));
    return slot;
}

void Gpu::Transition(ID3D12GraphicsCommandList* cl, GpuTexture& tex, D3D12_RESOURCE_STATES to) {
    if (tex.state == to) return;
    D3D12_RESOURCE_BARRIER br{};
    br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    br.Transition.pResource = tex.res.Get();
    br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    br.Transition.StateBefore = tex.state;
    br.Transition.StateAfter = to;
    cl->ResourceBarrier(1, &br);
    tex.state = to;
}

ID3D12GraphicsCommandList* Gpu::BeginUpload() {
    GA_CHECK(m_uploadAlloc->Reset());
    GA_CHECK(m_uploadList->Reset(m_uploadAlloc.Get(), nullptr));
    return m_uploadList.Get();
}

void Gpu::EndUpload() {
    GA_CHECK(m_uploadList->Close());
    ID3D12CommandList* lists[] = {m_uploadList.Get()};
    m_queue->ExecuteCommandLists(1, lists);
    WaitIdle();
    m_uploadKeepAlive.clear();   // safe: WaitIdle above guarantees the copies retired
}

bool Gpu::ReadbackTexel(ID3D12Resource* res, uint32_t subresource, uint32_t x, uint32_t y,
                        D3D12_RESOURCE_STATES state, uint8_t out[16]) {
    const D3D12_RESOURCE_DESC d = res->GetDesc();
    D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp{};
    fp.Footprint.Format = d.Format;
    fp.Footprint.Width = 1;
    fp.Footprint.Height = 1;
    fp.Footprint.Depth = 1;
    fp.Footprint.RowPitch = 256;   // minimum alignment; one texel fits any format we use

    Com<ID3D12Resource> rb;
    const auto hp = HeapProps(D3D12_HEAP_TYPE_READBACK);
    const auto rd = BufferDesc(256);
    if (FAILED(m_device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
                                                 D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                 IID_PPV_ARGS(&rb)))) {
        return false;
    }
    auto* cl = BeginUpload();
    D3D12_RESOURCE_BARRIER br{};
    br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    br.Transition.pResource = res;
    br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    if (state != D3D12_RESOURCE_STATE_COPY_SOURCE) {
        br.Transition.StateBefore = state;
        br.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
        cl->ResourceBarrier(1, &br);
    }
    D3D12_TEXTURE_COPY_LOCATION dst{}, src{};
    dst.pResource = rb.Get();
    dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    dst.PlacedFootprint = fp;
    src.pResource = res;
    src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    src.SubresourceIndex = subresource;
    D3D12_BOX box{x, y, 0, x + 1, y + 1, 1};
    cl->CopyTextureRegion(&dst, 0, 0, 0, &src, &box);
    if (state != D3D12_RESOURCE_STATE_COPY_SOURCE) {
        br.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE;
        br.Transition.StateAfter = state;
        cl->ResourceBarrier(1, &br);
    }
    EndUpload();
    void* p = nullptr;
    if (FAILED(rb->Map(0, nullptr, &p))) return false;
    std::memcpy(out, p, 16);
    rb->Unmap(0, nullptr);
    return true;
}

std::vector<uint8_t> Gpu::ReadbackTexture(GpuTexture& tex, uint32_t* outRowPitch,
                                          uint32_t mip) {
    D3D12_RESOURCE_DESC d = tex.res->GetDesc();
    D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp{};
    UINT numRows = 0;
    UINT64 rowBytes = 0, total = 0;
    // M9h: a mip-chained bank is verified one LEVEL at a time, so the readback has to be
    // able to name which subresource it means.
    m_device->GetCopyableFootprints(&d, mip, 1, 0, &fp, &numRows, &rowBytes, &total);

    Com<ID3D12Resource> rb;
    const auto hp = HeapProps(D3D12_HEAP_TYPE_READBACK);
    const auto rd = BufferDesc(total);
    GA_CHECK(m_device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
                                              D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                              IID_PPV_ARGS(&rb)));

    const D3D12_RESOURCE_STATES was = tex.state;
    auto* cl = BeginUpload();
    if (was != D3D12_RESOURCE_STATE_COPY_SOURCE) {
        D3D12_RESOURCE_BARRIER br{};
        br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        br.Transition.pResource = tex.res.Get();
        br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        br.Transition.StateBefore = was;
        br.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
        cl->ResourceBarrier(1, &br);
    }
    D3D12_TEXTURE_COPY_LOCATION dst{}, src{};
    dst.pResource = rb.Get();
    dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    dst.PlacedFootprint = fp;
    src.pResource = tex.res.Get();
    src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    src.SubresourceIndex = mip;
    cl->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
    if (was != D3D12_RESOURCE_STATE_COPY_SOURCE) {
        D3D12_RESOURCE_BARRIER br{};
        br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        br.Transition.pResource = tex.res.Get();
        br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        br.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE;
        br.Transition.StateAfter = was;
        cl->ResourceBarrier(1, &br);
    }
    EndUpload();

    std::vector<uint8_t> out(static_cast<size_t>(total));
    void* p = nullptr;
    D3D12_RANGE all{0, static_cast<SIZE_T>(total)};
    GA_CHECK(rb->Map(0, &all, &p));
    memcpy(out.data(), p, static_cast<size_t>(total));
    D3D12_RANGE none{0, 0};
    rb->Unmap(0, &none);

    if (outRowPitch) *outRowPitch = fp.Footprint.RowPitch;
    return out;
}

std::vector<uint8_t> Gpu::ReadbackBuffer(ID3D12Resource* buf, uint64_t bytes,
                                         D3D12_RESOURCE_STATES currentState) {
    Com<ID3D12Resource> rb;
    const auto hp = HeapProps(D3D12_HEAP_TYPE_READBACK);
    const auto rd = BufferDesc(bytes);
    GA_CHECK(m_device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
                                              D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                              IID_PPV_ARGS(&rb)));

    auto* cl = BeginUpload();
    if (currentState != D3D12_RESOURCE_STATE_COPY_SOURCE) {
        D3D12_RESOURCE_BARRIER br{};
        br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        br.Transition.pResource = buf;
        br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        br.Transition.StateBefore = currentState;
        br.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
        cl->ResourceBarrier(1, &br);
    }
    cl->CopyBufferRegion(rb.Get(), 0, buf, 0, bytes);
    if (currentState != D3D12_RESOURCE_STATE_COPY_SOURCE) {
        D3D12_RESOURCE_BARRIER br{};
        br.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        br.Transition.pResource = buf;
        br.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        br.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE;
        br.Transition.StateAfter = currentState;
        cl->ResourceBarrier(1, &br);
    }
    EndUpload();

    std::vector<uint8_t> out(static_cast<size_t>(bytes));
    void* p = nullptr;
    D3D12_RANGE all{0, static_cast<SIZE_T>(bytes)};
    GA_CHECK(rb->Map(0, &all, &p));
    memcpy(out.data(), p, static_cast<size_t>(bytes));
    D3D12_RANGE none{0, 0};
    rb->Unmap(0, &none);
    return out;
}

}  // namespace ga
