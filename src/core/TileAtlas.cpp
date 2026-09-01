#include "core/TileAtlas.h"

#include "core/PixEvents.h"

#include <vector>
#include <string>

namespace ga {

namespace {

constexpr uint32_t kTexSize = 1024;         // 8x8 tiles of 128x128 at R32_FLOAT
constexpr uint32_t kResultSlots = 66;       // 64 2D tiles + 2 slots for the 3D probes

struct TestConstants {
    uint32_t w, h, d;
    uint32_t tileW, tileH;
    uint32_t tilesX, tilesY;
    uint32_t pad;
};

struct Uint4 { uint32_t x, y, z, w; };

D3D12_HEAP_PROPERTIES DefaultHeapProps() {
    D3D12_HEAP_PROPERTIES p{};
    p.Type = D3D12_HEAP_TYPE_DEFAULT;
    return p;
}

void Barrier(ID3D12GraphicsCommandList* cl, ID3D12Resource* res, D3D12_RESOURCE_STATES from,
             D3D12_RESOURCE_STATES to) {
    D3D12_RESOURCE_BARRIER b{};
    b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    b.Transition.pResource = res;
    b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    b.Transition.StateBefore = from;
    b.Transition.StateAfter = to;
    cl->ResourceBarrier(1, &b);
}

void UavBarrier(ID3D12GraphicsCommandList* cl, ID3D12Resource* res) {
    D3D12_RESOURCE_BARRIER b{};
    b.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
    b.UAV.pResource = res;
    cl->ResourceBarrier(1, &b);
}

// The self-test harness: owns the reserved resources, the heap, the compute pipelines, and the
// residency bookkeeping the CPU verifies against.
class TileSelfTest {
public:
    bool Run(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir);

private:
    bool CreatePipelines(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir);
    bool Create2D(Gpu& gpu);
    bool Create3D(Gpu& gpu);   // only when tier >= 3
    void MapCheckerboard(Gpu& gpu);
    void RunWrite2D(Gpu& gpu);
    std::vector<Uint4> RunRead(Gpu& gpu, bool with3D);
    bool Verify2D(const std::vector<Uint4>& r, const char* phase);
    bool Verify3D(const std::vector<Uint4>& r, const char* phase);

    Com<ID3D12RootSignature> m_rootSig;
    Com<ID3D12PipelineState> m_write2D, m_read2D, m_write3D, m_read3D;

    Com<ID3D12Resource> m_tex2D;
    Com<ID3D12Resource> m_tex3D;
    Com<ID3D12Resource> m_results;
    Com<ID3D12Heap> m_heap;

    D3D12_RESOURCE_STATES m_state2D = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    D3D12_RESOURCE_STATES m_state3D = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;

    uint32_t m_tableBase = UINT32_MAX;   // 4 slots: [t0 2D srv][t1 3D srv][u0 2D uav][u1 3D uav]
    uint32_t m_tilesX = 0, m_tilesY = 0, m_numTiles = 0;
    uint32_t m_tileW = 0, m_tileH = 0;
    uint32_t m_heapTiles = 0, m_nextHeapTile = 0;
    std::vector<uint8_t> m_mapped;       // CPU residency truth, per 2D tile

    uint32_t m_tiles3DTotal = 0;
    D3D12_TILE_SHAPE m_shape3D{};
    bool m_have3D = false;
};

bool TileSelfTest::CreatePipelines(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    // Compute root signature: b0 constants, one table {t0..t1, u0..u1}, u2 root UAV (results),
    // s0 point-clamp static sampler.
    D3D12_DESCRIPTOR_RANGE1 ranges[2]{};
    ranges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    ranges[0].NumDescriptors = 2;
    ranges[0].BaseShaderRegister = 0;
    ranges[0].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ranges[0].OffsetInDescriptorsFromTableStart = 0;
    ranges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
    ranges[1].NumDescriptors = 2;
    ranges[1].BaseShaderRegister = 0;
    ranges[1].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    ranges[1].OffsetInDescriptorsFromTableStart = 2;

    D3D12_ROOT_PARAMETER1 params[3]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[0].Constants.ShaderRegister = 0;
    params[0].Constants.Num32BitValues = sizeof(TestConstants) / 4;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[1].DescriptorTable.NumDescriptorRanges = 2;
    params[1].DescriptorTable.pDescriptorRanges = ranges;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
    params[2].Descriptor.ShaderRegister = 2;   // u2

    D3D12_STATIC_SAMPLER_DESC samp{};
    samp.Filter = D3D12_FILTER_MIN_MAG_MIP_POINT;
    samp.AddressU = samp.AddressV = samp.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    samp.MaxLOD = D3D12_FLOAT32_MAX;
    samp.ShaderRegister = 0;
    samp.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER;

    D3D12_VERSIONED_ROOT_SIGNATURE_DESC vd{};
    vd.Version = D3D_ROOT_SIGNATURE_VERSION_1_1;
    vd.Desc_1_1.NumParameters = _countof(params);
    vd.Desc_1_1.pParameters = params;
    vd.Desc_1_1.NumStaticSamplers = 1;
    vd.Desc_1_1.pStaticSamplers = &samp;

    Com<ID3DBlob> blob, err;
    HRESULT hr = D3D12SerializeVersionedRootSignature(&vd, &blob, &err);
    if (FAILED(hr)) {
        if (err) Log("[tiletest] root sig: %s", static_cast<const char*>(err->GetBufferPointer()));
        return false;
    }
    GA_CHECK(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(),
                                              IID_PPV_ARGS(&m_rootSig)));
    m_rootSig->SetName(L"tiletest root signature");

    const std::wstring path = shaderDir + L"/TileTest.hlsl";
    struct Entry { const wchar_t* name; Com<ID3D12PipelineState>* pso; };
    const Entry entries[] = {
        {L"CsWrite2D", &m_write2D}, {L"CsRead2D", &m_read2D},
        {L"CsWrite3D", &m_write3D}, {L"CsRead3D", &m_read3D},
    };
    for (const Entry& e : entries) {
        ShaderBlob cs = sc.Compile(path, e.name, L"cs_6_0");
        if (!cs.Valid()) { Log("[tiletest] %S failed to compile", e.name); return false; }
        D3D12_COMPUTE_PIPELINE_STATE_DESC d{};
        d.pRootSignature = m_rootSig.Get();
        d.CS = {cs.Data(), cs.Size()};
        GA_CHECK(gpu.Device()->CreateComputePipelineState(&d, IID_PPV_ARGS(e.pso->GetAddressOf())));
        (*e.pso)->SetName(e.name);
    }
    return true;
}

bool TileSelfTest::Create2D(Gpu& gpu) {
    D3D12_RESOURCE_DESC rd{};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    rd.Width = kTexSize;
    rd.Height = kTexSize;
    rd.DepthOrArraySize = 1;
    rd.MipLevels = 1;
    rd.Format = DXGI_FORMAT_R32_FLOAT;
    rd.SampleDesc.Count = 1;
    rd.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;   // required for reserved resources
    rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;

    GA_CHECK(gpu.Device()->CreateReservedResource(&rd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                                 nullptr, IID_PPV_ARGS(&m_tex2D)));
    m_tex2D->SetName(L"tiletest.reserved2D (G-bank stand-in)");
    m_state2D = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;

    UINT numTiles = 0;
    D3D12_PACKED_MIP_INFO packed{};
    D3D12_TILE_SHAPE shape{};
    UINT numSub = 1;
    D3D12_SUBRESOURCE_TILING sub{};
    gpu.Device()->GetResourceTiling(m_tex2D.Get(), &numTiles, &packed, &shape, &numSub, 0, &sub);
    m_numTiles = numTiles;
    m_tileW = shape.WidthInTexels;
    m_tileH = shape.HeightInTexels;
    m_tilesX = sub.WidthInTiles;
    m_tilesY = sub.HeightInTiles;
    Log("[tiletest] 2D reserved %ux%u R32F: %u tiles of %ux%u texels (%ux%u grid)", kTexSize,
        kTexSize, m_numTiles, m_tileW, m_tileH, m_tilesX, m_tilesY);
    if (m_numTiles == 0 || m_numTiles > 4096) return false;
    m_mapped.assign(m_numTiles, 0);

    // Heap: checkerboard (half the tiles) + slack for the remap phase. Reserved-resource heaps on
    // heap-tier-1 hardware must be single-purpose; ALLOW_ONLY_NON_RT_DS_TEXTURES is that.
    m_heapTiles = m_numTiles / 2 + 4;
    D3D12_HEAP_DESC hd{};
    hd.SizeInBytes = static_cast<uint64_t>(m_heapTiles) * D3D12_TILED_RESOURCE_TILE_SIZE_IN_BYTES;
    hd.Properties = DefaultHeapProps();
    hd.Flags = D3D12_HEAP_FLAG_ALLOW_ONLY_NON_RT_DS_TEXTURES;
    GA_CHECK(gpu.Device()->CreateHeap(&hd, IID_PPV_ARGS(&m_heap)));
    m_heap->SetName(L"tiletest.tilePool");

    // Results buffer, written by the read passes through a root UAV.
    D3D12_RESOURCE_DESC bd{};
    bd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    bd.Width = kResultSlots * sizeof(Uint4);
    bd.Height = 1;
    bd.DepthOrArraySize = 1;
    bd.MipLevels = 1;
    bd.SampleDesc.Count = 1;
    bd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    bd.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
    const auto hp = DefaultHeapProps();
    GA_CHECK(gpu.Device()->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &bd,
                                                  D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr,
                                                  IID_PPV_ARGS(&m_results)));
    m_results->SetName(L"tiletest.results");

    // Descriptor table: 4 consecutive slots. The 3D views are filled with NULL descriptors first
    // and replaced with real ones if tier 3 creates the volume resource; a bound-but-unread NULL
    // descriptor is legal, an unbound table slot is not.
    m_tableBase = gpu.SrvHeap().Alloc(4);

    D3D12_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv.Format = DXGI_FORMAT_R32_FLOAT;
    sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    sv.Texture2D.MipLevels = 1;
    gpu.Device()->CreateShaderResourceView(m_tex2D.Get(), &sv, gpu.SrvHeap().Cpu(m_tableBase + 0));

    D3D12_SHADER_RESOURCE_VIEW_DESC sv3{};
    sv3.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv3.Format = DXGI_FORMAT_R32_FLOAT;
    sv3.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
    sv3.Texture3D.MipLevels = 1;
    gpu.Device()->CreateShaderResourceView(nullptr, &sv3, gpu.SrvHeap().Cpu(m_tableBase + 1));

    D3D12_UNORDERED_ACCESS_VIEW_DESC uv{};
    uv.Format = DXGI_FORMAT_R32_FLOAT;
    uv.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    gpu.Device()->CreateUnorderedAccessView(m_tex2D.Get(), nullptr, &uv,
                                            gpu.SrvHeap().Cpu(m_tableBase + 2));

    D3D12_UNORDERED_ACCESS_VIEW_DESC uv3{};
    uv3.Format = DXGI_FORMAT_R32_FLOAT;
    uv3.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE3D;
    uv3.Texture3D.WSize = UINT(-1);
    gpu.Device()->CreateUnorderedAccessView(nullptr, nullptr, &uv3,
                                            gpu.SrvHeap().Cpu(m_tableBase + 3));
    return true;
}

bool TileSelfTest::Create3D(Gpu& gpu) {
    D3D12_RESOURCE_DESC rd{};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE3D;
    rd.Width = 128;
    rd.Height = 128;
    rd.DepthOrArraySize = 64;
    rd.MipLevels = 1;
    rd.Format = DXGI_FORMAT_R32_FLOAT;
    rd.SampleDesc.Count = 1;
    rd.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
    rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;

    const HRESULT hr = gpu.Device()->CreateReservedResource(
        &rd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&m_tex3D));
    if (FAILED(hr)) {
        Log("[tiletest] 3D reserved resource creation FAILED despite tier 3: %s",
            HrString(hr).c_str());
        return false;
    }
    m_tex3D->SetName(L"tiletest.reserved3D (volume G-bank stand-in)");
    m_state3D = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;

    UINT numTiles = 0;
    D3D12_PACKED_MIP_INFO packed{};
    UINT numSub = 1;
    D3D12_SUBRESOURCE_TILING sub{};
    gpu.Device()->GetResourceTiling(m_tex3D.Get(), &numTiles, &packed, &m_shape3D, &numSub, 0,
                                    &sub);
    m_tiles3DTotal = numTiles;
    Log("[tiletest] 3D reserved 128x128x64 R32F: %u tiles of %ux%ux%u texels", numTiles,
        m_shape3D.WidthInTexels, m_shape3D.HeightInTexels, m_shape3D.DepthInTexels);

    // Map exactly one tile: the origin corner. Everything else stays NULL.
    D3D12_TILED_RESOURCE_COORDINATE at{};
    D3D12_TILE_REGION_SIZE size{};
    size.NumTiles = 1;
    D3D12_TILE_RANGE_FLAGS flag = D3D12_TILE_RANGE_FLAG_NONE;
    UINT heapStart = m_nextHeapTile++;
    UINT count = 1;
    gpu.Queue()->UpdateTileMappings(m_tex3D.Get(), 1, &at, &size, m_heap.Get(), 1, &flag,
                                    &heapStart, &count, D3D12_TILE_MAPPING_FLAG_NONE);

    // Swap the real 3D views into the reserved table slots.
    D3D12_SHADER_RESOURCE_VIEW_DESC sv3{};
    sv3.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv3.Format = DXGI_FORMAT_R32_FLOAT;
    sv3.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
    sv3.Texture3D.MipLevels = 1;
    gpu.Device()->CreateShaderResourceView(m_tex3D.Get(), &sv3, gpu.SrvHeap().Cpu(m_tableBase + 1));

    D3D12_UNORDERED_ACCESS_VIEW_DESC uv3{};
    uv3.Format = DXGI_FORMAT_R32_FLOAT;
    uv3.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE3D;
    uv3.Texture3D.WSize = UINT(-1);
    gpu.Device()->CreateUnorderedAccessView(m_tex3D.Get(), nullptr, &uv3,
                                            gpu.SrvHeap().Cpu(m_tableBase + 3));
    m_have3D = true;
    return true;
}

void TileSelfTest::MapCheckerboard(Gpu& gpu) {
    // One UpdateTileMappings call, one region covering the whole surface (row-major tile order),
    // per-tile ranges alternating NONE (mapped, next pool tile) and NULL. This is exactly the
    // "batch aggressively" shape M4 will use, at toy scale.
    D3D12_TILED_RESOURCE_COORDINATE start{};
    D3D12_TILE_REGION_SIZE size{};
    size.NumTiles = m_numTiles;

    std::vector<D3D12_TILE_RANGE_FLAGS> flags(m_numTiles);
    std::vector<UINT> starts(m_numTiles), counts(m_numTiles, 1);
    for (uint32_t i = 0; i < m_numTiles; ++i) {
        const uint32_t tx = i % m_tilesX, ty = i / m_tilesX;
        const bool mapped = ((tx + ty) & 1) == 0;
        m_mapped[i] = mapped ? 1 : 0;
        flags[i] = mapped ? D3D12_TILE_RANGE_FLAG_NONE : D3D12_TILE_RANGE_FLAG_NULL;
        starts[i] = mapped ? m_nextHeapTile++ : 0;
    }
    gpu.Queue()->UpdateTileMappings(m_tex2D.Get(), 1, &start, &size, m_heap.Get(), m_numTiles,
                                    flags.data(), starts.data(), counts.data(),
                                    D3D12_TILE_MAPPING_FLAG_NONE);
    Log("[tiletest] mapped a %ux%u checkerboard (%u resident, %u null)", m_tilesX, m_tilesY,
        m_nextHeapTile, m_numTiles - m_nextHeapTile);
}

void TileSelfTest::RunWrite2D(Gpu& gpu) {
    auto* cl = gpu.BeginUpload();
    ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
    cl->SetDescriptorHeaps(1, heaps);
    {
        PixScope scope(cl, "tiletest.write2D (covers NULL tiles too - those writes must vanish)");
        if (m_state2D != D3D12_RESOURCE_STATE_UNORDERED_ACCESS) {
            Barrier(cl, m_tex2D.Get(), m_state2D, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            m_state2D = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        }
        TestConstants c{kTexSize, kTexSize, 1, m_tileW, m_tileH, m_tilesX, m_tilesY, 0};
        cl->SetComputeRootSignature(m_rootSig.Get());
        cl->SetComputeRoot32BitConstants(0, sizeof(c) / 4, &c, 0);
        cl->SetComputeRootDescriptorTable(1, gpu.SrvHeap().Gpu(m_tableBase));
        cl->SetComputeRootUnorderedAccessView(2, m_results->GetGPUVirtualAddress());
        cl->SetPipelineState(m_write2D.Get());
        cl->Dispatch(kTexSize / 8, kTexSize / 8, 1);
        UavBarrier(cl, m_tex2D.Get());
    }
    if (m_have3D) {
        PixScope scope(cl, "tiletest.write3D");
        if (m_state3D != D3D12_RESOURCE_STATE_UNORDERED_ACCESS) {
            Barrier(cl, m_tex3D.Get(), m_state3D, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            m_state3D = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        }
        TestConstants c{128, 128, 64, m_shape3D.WidthInTexels, m_shape3D.HeightInTexels,
                        m_tilesX, m_tilesY, 0};
        cl->SetComputeRootSignature(m_rootSig.Get());
        cl->SetComputeRoot32BitConstants(0, sizeof(c) / 4, &c, 0);
        cl->SetComputeRootDescriptorTable(1, gpu.SrvHeap().Gpu(m_tableBase));
        cl->SetComputeRootUnorderedAccessView(2, m_results->GetGPUVirtualAddress());
        cl->SetPipelineState(m_write3D.Get());
        cl->Dispatch(128 / 4, 128 / 4, 64 / 4);
        UavBarrier(cl, m_tex3D.Get());
    }
    gpu.EndUpload();
}

std::vector<Uint4> TileSelfTest::RunRead(Gpu& gpu, bool with3D) {
    auto* cl = gpu.BeginUpload();
    ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
    cl->SetDescriptorHeaps(1, heaps);
    {
        PixScope scope(cl, "tiletest.read2D (Load + SampleLevel + CheckAccessFullyMapped per tile)");
        // A transition with StateBefore == StateAfter is invalid and fails at Close(); the read
        // pass can legitimately run twice in a row (unmap/remap phases), so guard it.
        if (m_state2D != D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE) {
            Barrier(cl, m_tex2D.Get(), m_state2D, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            m_state2D = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
        }
        TestConstants c{kTexSize, kTexSize, 1, m_tileW, m_tileH, m_tilesX, m_tilesY, 0};
        cl->SetComputeRootSignature(m_rootSig.Get());
        cl->SetComputeRoot32BitConstants(0, sizeof(c) / 4, &c, 0);
        cl->SetComputeRootDescriptorTable(1, gpu.SrvHeap().Gpu(m_tableBase));
        cl->SetComputeRootUnorderedAccessView(2, m_results->GetGPUVirtualAddress());
        cl->SetPipelineState(m_read2D.Get());
        cl->Dispatch(m_tilesX, m_tilesY, 1);
    }
    if (with3D && m_have3D) {
        PixScope scope(cl, "tiletest.read3D");
        if (m_state3D != D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE) {
            Barrier(cl, m_tex3D.Get(), m_state3D, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            m_state3D = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
        }
        TestConstants c{128, 128, 64, m_shape3D.WidthInTexels, m_shape3D.HeightInTexels,
                        m_tilesX, m_tilesY, 0};
        cl->SetComputeRootSignature(m_rootSig.Get());
        cl->SetComputeRoot32BitConstants(0, sizeof(c) / 4, &c, 0);
        cl->SetComputeRootDescriptorTable(1, gpu.SrvHeap().Gpu(m_tableBase));
        cl->SetComputeRootUnorderedAccessView(2, m_results->GetGPUVirtualAddress());
        cl->SetPipelineState(m_read3D.Get());
        cl->Dispatch(1, 1, 1);
    }
    gpu.EndUpload();

    std::vector<uint8_t> raw = gpu.ReadbackBuffer(m_results.Get(), kResultSlots * sizeof(Uint4),
                                                  D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    std::vector<Uint4> out(kResultSlots);
    memcpy(out.data(), raw.data(), raw.size());
    return out;
}

// Result encoding, mirrored in TileTest.hlsl CsRead2D:
//   x = of 5 probed texels, how many LOADED the exact written pattern
//   y = of 5 probed texels, how many SAMPLED the exact written pattern
//   z = of 5 probes, how many reported CheckAccessFullyMapped == true
//   w = loadZeros | (sampleZeros << 8)  -- how many probes read exactly 0.0
bool TileSelfTest::Verify2D(const std::vector<Uint4>& r, const char* phase) {
    uint32_t bad = 0;
    std::string expectMap, gotMap;
    for (uint32_t ty = 0; ty < m_tilesY; ++ty) {
        for (uint32_t tx = 0; tx < m_tilesX; ++tx) {
            const uint32_t i = ty * m_tilesX + tx;
            const Uint4& v = r[i];
            const uint32_t loadZeros = v.w & 0xFF, sampleZeros = (v.w >> 8) & 0xFF;
            bool ok;
            if (m_mapped[i]) {
                ok = (v.x == 5) && (v.y == 5) && (v.z == 5);
            } else {
                ok = (v.x == 0) && (v.y == 0) && (v.z == 0) && (loadZeros == 5) &&
                     (sampleZeros == 5);
            }
            if (!ok && bad < 4) {
                Log("[tiletest] %s: tile (%u,%u) %s: loadPat=%u samplePat=%u mappedStatus=%u "
                    "zeros=%u/%u",
                    phase, tx, ty, m_mapped[i] ? "MAPPED" : "NULL", v.x, v.y, v.z, loadZeros,
                    sampleZeros);
            }
            bad += ok ? 0 : 1;
            expectMap += m_mapped[i] ? '#' : '.';
            gotMap += (v.z == 5) ? '#' : (v.z == 0 ? '.' : '?');
        }
        expectMap += ' ';
        gotMap += ' ';
    }
    Log("[tiletest] %s residency  expected |%s|  shader-reported |%s|", phase, expectMap.c_str(),
        gotMap.c_str());
    if (bad) Log("[tiletest] %s: %u of %u tiles FAILED", phase, bad, m_numTiles);
    else Log("[tiletest] %s: all %u tiles behave per Tier-2 spec", phase, m_numTiles);
    return bad == 0;
}

bool TileSelfTest::Verify3D(const std::vector<Uint4>& r, const char* phase) {
    if (!m_have3D) return true;
    // Slot 64: probe inside the one mapped tile -- expect pattern + mapped status.
    // Slot 65: probe far outside it -- expect exact zero + unmapped status.
    const Uint4& m = r[64];
    const Uint4& n = r[65];
    const bool okM = m.x == 1 && m.y == 1 && m.z == 1;
    const bool okN = n.x == 1 && n.y == 1 && n.z == 0;
    if (!okM) Log("[tiletest] %s 3D mapped-voxel probe FAILED: load=%u sample=%u status=%u",
                  phase, m.x, m.y, m.z);
    if (!okN) Log("[tiletest] %s 3D null-voxel probe FAILED: loadZero=%u sampleZero=%u status=%u",
                  phase, n.x, n.y, n.z);
    if (okM && okN) Log("[tiletest] %s: 3D volume tile semantics hold (tier 3)", phase);
    return okM && okN;
}

bool TileSelfTest::Run(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    const int tier = static_cast<int>(gpu.TiledTier());
    Log("[tiletest] ---- M0 gate: reserved-resource null-tile semantics ----");
    Log("[tiletest] tiled resources tier: %d %s", tier,
        tier >= 3 ? "(3D volume banks available)" :
        tier == 2 ? "(2D banks guaranteed; no 3D tiled)" : "(BELOW SPEC FLOOR)");
    if (tier < 2) {
        Log("[tiletest] FAIL: tier < 2 means null-tile reads are undefined; the atlas design "
            "needs the dense fallback path on this adapter");
        return false;
    }

    if (!CreatePipelines(gpu, sc, shaderDir)) return false;
    if (!Create2D(gpu)) return false;
    MapCheckerboard(gpu);
    if (tier >= 3) Create3D(gpu);

    bool pass = true;

    // Phase 1: write everywhere (null writes must vanish), read everywhere, verify checkerboard.
    RunWrite2D(gpu);
    {
        const std::vector<Uint4> r1 = RunRead(gpu, true);
        pass &= Verify2D(r1, "phase1-checkerboard");
        pass &= Verify3D(r1, "phase1-3D");
    }

    // Phase 2: unmap one previously-mapped tile; its data must disappear from view (reads 0,
    // reported unmapped) with no new write in between.
    {
        const uint32_t tx = 2, ty = 2;   // (2+2)&1==0 -> was mapped
        D3D12_TILED_RESOURCE_COORDINATE at{tx, ty, 0, 0};
        D3D12_TILE_REGION_SIZE size{};
        size.NumTiles = 1;
        D3D12_TILE_RANGE_FLAGS flag = D3D12_TILE_RANGE_FLAG_NULL;
        UINT dummyStart = 0, count = 1;
        gpu.Queue()->UpdateTileMappings(m_tex2D.Get(), 1, &at, &size, nullptr, 1, &flag,
                                        &dummyStart, &count, D3D12_TILE_MAPPING_FLAG_NONE);
        m_mapped[ty * m_tilesX + tx] = 0;
        Log("[tiletest] unmapped tile (2,2); its prior contents must now be unreachable");
        pass &= Verify2D(RunRead(gpu, false), "phase2-after-unmap");
    }

    // Phase 3: remap that tile to a FRESH pool tile (contents undefined until written), write
    // again, verify the write stuck and the full checkerboard is back.
    {
        const uint32_t tx = 2, ty = 2;
        D3D12_TILED_RESOURCE_COORDINATE at{tx, ty, 0, 0};
        D3D12_TILE_REGION_SIZE size{};
        size.NumTiles = 1;
        D3D12_TILE_RANGE_FLAGS flag = D3D12_TILE_RANGE_FLAG_NONE;
        UINT heapStart = m_nextHeapTile++;
        UINT count = 1;
        gpu.Queue()->UpdateTileMappings(m_tex2D.Get(), 1, &at, &size, m_heap.Get(), 1, &flag,
                                        &heapStart, &count, D3D12_TILE_MAPPING_FLAG_NONE);
        m_mapped[ty * m_tilesX + tx] = 1;
        Log("[tiletest] remapped tile (2,2) to fresh pool tile %u; rewriting", heapStart);
        RunWrite2D(gpu);
        pass &= Verify2D(RunRead(gpu, false), "phase3-after-remap");
    }

    Log("[tiletest] ---- %s ----", pass ? "PASS: the tiled multivector atlas contract holds on "
                                          "this GPU"
                                        : "FAIL: see tile reports above");
    return pass;
}

}  // namespace

bool RunTileSelfTest(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    TileSelfTest test;
    return test.Run(gpu, sc, shaderDir);
}

// ================================================================================ TileAtlas2D

void TileAtlas2D::Init(Gpu& gpu, uint32_t widthTexels, uint32_t heightTexels, DXGI_FORMAT fmt,
                       const wchar_t* name, uint32_t heapChunkTiles, uint32_t mipLevels,
                       uint32_t arraySlices) {
    m_heapChunkTiles = heapChunkTiles;

    D3D12_RESOURCE_DESC rd{};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    rd.Width = widthTexels;
    rd.Height = heightTexels;
    rd.DepthOrArraySize = static_cast<UINT16>(arraySlices ? arraySlices : 1);
    rd.MipLevels = static_cast<UINT16>(mipLevels ? mipLevels : 1);
    rd.Format = fmt;
    rd.SampleDesc.Count = 1;
    rd.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
    rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
    GA_CHECK(gpu.Device()->CreateReservedResource(&rd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                                 nullptr, IID_PPV_ARGS(&m_res)));
    m_res->SetName(name);

    m_mipCount = rd.MipLevels;
    UINT numTiles = 0;
    D3D12_PACKED_MIP_INFO packed{};
    D3D12_TILE_SHAPE shape{};
    UINT numSub = m_mipCount;
    std::vector<D3D12_SUBRESOURCE_TILING> subs(m_mipCount);
    gpu.Device()->GetResourceTiling(m_res.Get(), &numTiles, &packed, &shape, &numSub, 0,
                                    subs.data());
    m_tileW = shape.WidthInTexels;
    m_tileH = shape.HeightInTexels;
    // D3D12 packs every mip small enough to share tiles into a single tail. Those are not
    // per-tile mappable, and their WidthInTiles is reported as 0.
    m_standardMips = packed.NumStandardMips ? packed.NumStandardMips : m_mipCount;
    if (m_standardMips > m_mipCount) m_standardMips = m_mipCount;

    m_mip.assign(m_mipCount, MipInfo{});
    uint32_t base = 0;
    for (uint32_t m = 0; m < m_mipCount; ++m) {
        MipInfo& mi = m_mip[m];
        if (m < m_standardMips) {
            mi.tilesX = subs[m].WidthInTiles;
            mi.tilesY = subs[m].HeightInTiles;
            mi.startTile = subs[m].StartTileIndexInOverallResource;
        } else {
            const uint32_t w = (widthTexels >> m) ? (widthTexels >> m) : 1u;
            const uint32_t h = (heightTexels >> m) ? (heightTexels >> m) : 1u;
            mi.tilesX = (w + m_tileW - 1) / m_tileW;
            mi.tilesY = (h + m_tileH - 1) / m_tileH;
            if (!mi.tilesX) mi.tilesX = 1;
            if (!mi.tilesY) mi.tilesY = 1;
            mi.startTile = packed.StartTileIndexInOverallResource;
        }
        mi.base = base;
        if (m < m_standardMips) base += mi.tilesX * mi.tilesY;
    }
    m_tilesX = m_mip[0].tilesX;
    m_tilesY = m_mip[0].tilesY;
    m_slices = rd.DepthOrArraySize;
    m_tilesPerSlice = base;
    m_sliceActive.assign(m_slices, 0);
    m_state.assign(static_cast<size_t>(base) * m_slices, 0);
    m_tilePool.assign(m_state.size(), 0);
    m_packedTiles = packed.NumTilesForPackedMips;

    m_srv = (rd.DepthOrArraySize > 1) ? gpu.CreateSrvArray(m_res.Get(), fmt, m_mipCount,
                                                          rd.DepthOrArraySize)
                                      : gpu.CreateSrv(m_res.Get(), fmt);
    m_uav = gpu.CreateTextureUav(m_res.Get(), fmt, D3D12_UAV_DIMENSION_TEXTURE2D);
    m_fmt = fmt;
    m_mipUav.resize(m_mipCount);
    for (uint32_t m = 0; m < m_mipCount; ++m) {
        m_mipUav[m] = (m == 0) ? m_uav
                               : gpu.CreateTextureUav(m_res.Get(), fmt,
                                                      D3D12_UAV_DIMENSION_TEXTURE2D, m);
    }

    // ---- PIN THE PACKED TAIL. It is a handful of tiles and it is the coarsest description of
    // the whole field. Keeping it resident forever is what makes a miss impossible: a sample
    // that finds nothing finer still lands on real data, so quality degrades to blur and never
    // to garbage -- and never to a pop, because no code path appears or disappears.
    // WHICH LEVEL IS THE FLOOR. Not every shape has a packed tail -- measured: 4096^2 x8
    // x5mip reports 0 packed, because its coarsest mip is still larger than one tile. With no
    // tail there is nothing pinned, so the coarsest STANDARD mip becomes the floor instead.
    // The floor is PINNED PER SLICE by ActivateSlice, never here: pinning 1024 floors up front
    // would commit a great deal of memory for pages that carry nothing.
    m_pinnedFloor = packed.NumPackedMips > 0 ? (m_mipCount - 1) : (m_standardMips - 1);

    if (m_mipCount > 1) {
        // One texel per (slice, mip-0 tile); slices stack downward, so a shader that knows its
        // slice reads row (slice * tilesY + ty). 0xFF = nothing resident, which is what an
        // un-activated slice honestly is.
        const uint32_t mh = m_tilesY * m_slices;
        m_resMapCpu.assign(static_cast<size_t>(m_tilesX) * mh, 0xFFu);
        m_resMap = gpu.CreateTexture2D(m_tilesX, mh, DXGI_FORMAT_R8_UINT,
                                       D3D12_RESOURCE_FLAG_NONE,
                                       D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
                                       L"atlas.residencyMap");
        m_resMapSrv = gpu.CreateSrv(m_resMap.res.Get(), DXGI_FORMAT_R8_UINT);
        gpu.UploadTexture(m_resMap, m_resMapCpu.data(), m_tilesX);
    }

    // Backward compatibility, and the right default: a single-slice bank has exactly one page
    // and every existing caller expects it live from the start. Arrays activate on demand.
    if (m_slices == 1) ActivateSlice(gpu, 0);

    Log("[atlas] %S: %ux%u x%u slices = %ux%u tiles of %ux%u, %u mips (%u standard + %u "
        "packed), floor mip %u (%.2f GB virtual, resident on demand)",
        name, widthTexels, heightTexels, m_slices, m_tilesX, m_tilesY, m_tileW, m_tileH,
        m_mipCount, m_standardMips, m_mipCount - m_standardMips, m_pinnedFloor,
        VirtualBytes() / 1073741824.0);
}

// The finest mip actually resident over a mip-0 tile region. Coarser mips cover proportionally
// more ground, so mip-0 tile (tx0, ty0) lands on (tx0 >> m, ty0 >> m) at mip m.
uint32_t TileAtlas2D::FinestResident(uint32_t slice, uint32_t tx0, uint32_t ty0) const {
    if (!IsSliceActive(slice)) return kNothingResident;   // no floor: the page does not exist
    for (uint32_t m = 0; m < m_standardMips; ++m) {
        if (IsResident(slice, m, tx0 >> m, ty0 >> m)) return m;
    }
    return m_pinnedFloor;   // packed tail, or the coarsest standard mip
}

// Pin one slice's floor. Everything the chain promises -- a sample can never miss, quality
// degrades to blur and never to garbage -- holds PER SLICE, and only once that slice's floor
// exists. Activation is on demand for exactly that reason: a slice nobody has asked for
// should cost nothing, and FinestResident reports kNothingResident for it rather than naming
// a level that is NULL.
void TileAtlas2D::ActivateSlice(Gpu& gpu, uint32_t slice) {
    if (slice >= m_slices || IsSliceActive(slice)) return;
    m_sliceActive[slice] = 1;
    // A bank with no chain makes no floor promise -- there is no coarser level to fall back
    // to, so there is nothing to pin. Pinning "the coarsest standard mip" here would map mip 0
    // in its entirety and quietly make every flat bank fully resident.
    if (m_mipCount <= 1) return;
    if (m_packedTiles > 0) {
        D3D12_HEAP_DESC hd{};
        hd.SizeInBytes = static_cast<uint64_t>(m_packedTiles) * kTileBytes;
        D3D12_HEAP_PROPERTIES hp{};
        hp.Type = D3D12_HEAP_TYPE_DEFAULT;
        hd.Properties = hp;
        hd.Flags = D3D12_HEAP_FLAG_ALLOW_ONLY_NON_RT_DS_TEXTURES;
        Com<ID3D12Heap> heap;
        GA_CHECK(gpu.Device()->CreateHeap(&hd, IID_PPV_ARGS(&heap)));
        heap->SetName(L"atlas.packedTail");
        m_heaps.push_back(heap);
        D3D12_TILED_RESOURCE_COORDINATE c{};
        c.Subresource = m_standardMips + slice * m_mipCount;
        D3D12_TILE_REGION_SIZE rs{};
        rs.NumTiles = m_packedTiles;
        rs.UseBox = FALSE;
        D3D12_TILE_RANGE_FLAGS rf = D3D12_TILE_RANGE_FLAG_NONE;
        UINT st = 0, ct = m_packedTiles;
        gpu.Queue()->UpdateTileMappings(m_res.Get(), 1, &c, &rs, heap.Get(), 1, &rf, &st, &ct,
                                        D3D12_TILE_MAPPING_FLAG_NONE);
    } else if (m_standardMips > 0) {
        const MipInfo& top = m_mip[m_standardMips - 1];
        for (uint32_t ty = 0; ty < top.tilesY; ++ty) {
            for (uint32_t tx = 0; tx < top.tilesX; ++tx) {
                RequestMap(slice, m_standardMips - 1, tx, ty);
            }
        }
        CommitMappings(gpu, nullptr);
    }
}

// The map the SHADER reads: one texel per mip-0 tile, value = finest resident mip there. A
// sampler clamps its LOD to this, so a region streaming in gets sharper without any code path
// changing -- which is the whole no-pop contract. Only uploaded when it actually moved.
void TileAtlas2D::RebuildResidencyMap(Gpu& gpu) {
    if (m_mipCount <= 1 || m_resMapCpu.empty()) return;
    bool moved = false;
    for (uint32_t sl = 0; sl < m_slices; ++sl) {
        for (uint32_t ty = 0; ty < m_tilesY; ++ty) {
            for (uint32_t tx = 0; tx < m_tilesX; ++tx) {
                const uint32_t f = FinestResident(sl, tx, ty);
                const uint8_t v = (f == kNothingResident) ? 0xFFu : static_cast<uint8_t>(f);
                uint8_t& slot =
                    m_resMapCpu[(static_cast<size_t>(sl) * m_tilesY + ty) * m_tilesX + tx];
                if (slot != v) { slot = v; moved = true; }
            }
        }
    }
    if (moved) gpu.UploadTexture(m_resMap, m_resMapCpu.data(), m_tilesX);
}


// ---- M9h: FILLING THE CHAIN -------------------------------------------------------------
// A pinned tail that reads zero is real memory pretending to be data. These two build the
// levels so the floor actually carries the field.

void TileAtlas2D::MapAllCoarse(uint32_t slice) {
    for (uint32_t m = 1; m < m_standardMips; ++m) {
        for (uint32_t ty = 0; ty < m_mip[m].tilesY; ++ty) {
            for (uint32_t tx = 0; tx < m_mip[m].tilesX; ++tx) RequestMap(slice, m, tx, ty);
        }
    }
}

void TileAtlas2D::BuildMips(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
                            ID3D12GraphicsCommandList* cl, uint32_t slice) {
    if (m_mipCount < 2) return;

    // Which reducer: a typed UAV's format must match the resource's family, so the bank picks
    // by what it stores. Anything else is refused loudly rather than reduced with the wrong
    // channel count -- a mis-typed UAV reads garbage, it does not fail.
    int channels = 0;
    switch (m_fmt) {
        case DXGI_FORMAT_R32_FLOAT:
        case DXGI_FORMAT_R16_FLOAT: channels = 1; break;
        case DXGI_FORMAT_R32G32_FLOAT:
        case DXGI_FORMAT_R16G16_FLOAT: channels = 2; break;
        case DXGI_FORMAT_R32G32B32A32_FLOAT:
        case DXGI_FORMAT_R16G16B16A16_FLOAT: channels = 4; break;
        default: break;
    }
    if (!channels) {
        Log("[atlas] BuildMips: format %d has no reducer -- chain left unfilled", int(m_fmt));
        return;
    }

    if (!m_mipPso) {
        D3D12_DESCRIPTOR_RANGE range{};
        range.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
        range.NumDescriptors = 2;
        range.BaseShaderRegister = 0;
        D3D12_ROOT_PARAMETER rp[2]{};
        rp[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        rp[0].Constants.Num32BitValues = 4;
        rp[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
        rp[1].DescriptorTable.NumDescriptorRanges = 1;
        rp[1].DescriptorTable.pDescriptorRanges = &range;
        D3D12_ROOT_SIGNATURE_DESC rsd{};
        rsd.NumParameters = 2;
        rsd.pParameters = rp;
        Com<ID3DBlob> blob, err;
        if (FAILED(D3D12SerializeRootSignature(&rsd, D3D_ROOT_SIGNATURE_VERSION_1, &blob,
                                               &err))) {
            Log("[atlas] BuildMips: root signature failed");
            return;
        }
        if (FAILED(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(),
                                                     blob->GetBufferSize(),
                                                     IID_PPV_ARGS(&m_mipRs)))) {
            return;
        }
        const std::wstring def = L"GA_MIP_CH=" + std::to_wstring(channels);
        ShaderBlob cs =
            sc.Compile(shaderDir + L"/MipReduce.hlsl", L"CsMipReduce", L"cs_6_0", {def});
        if (!cs.Valid()) {
            Log("[atlas] BuildMips: MipReduce.hlsl (%d ch) failed to compile", channels);
            return;
        }
        D3D12_COMPUTE_PIPELINE_STATE_DESC pd{};
        pd.pRootSignature = m_mipRs.Get();
        pd.CS = {cs.Data(), cs.Size()};
        if (FAILED(gpu.Device()->CreateComputePipelineState(&pd, IID_PPV_ARGS(&m_mipPso)))) {
            Log("[atlas] BuildMips: PSO failed");
            return;
        }
        // ONE DESCRIPTOR PAIR PER LEVEL. Reusing a single pair looks right and is not:
        // descriptor writes land on the CPU immediately while the dispatches execute later,
        // so every level would read whichever pair was written last. Caught by the mip-1
        // value check -- the whole coarse level came back carrying one tile's stamp.
        m_mipTable = gpu.SrvHeap().Alloc(2 * (m_mipCount - 1));
    }

    cl->SetComputeRootSignature(m_mipRs.Get());
    cl->SetPipelineState(m_mipPso.Get());

    const uint32_t w0 = m_tilesX * m_tileW, h0 = m_tilesY * m_tileH;
    for (uint32_t m = 0; m + 1 < m_mipCount; ++m) {
        const uint32_t sw = (w0 >> m) ? (w0 >> m) : 1u, sh = (h0 >> m) ? (h0 >> m) : 1u;
        const uint32_t dw = (w0 >> (m + 1)) ? (w0 >> (m + 1)) : 1u;
        const uint32_t dh = (h0 >> (m + 1)) ? (h0 >> (m + 1)) : 1u;

        D3D12_UNORDERED_ACCESS_VIEW_DESC u{};
        u.Format = m_fmt;
        u.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
        const uint32_t slot = m_mipTable + 2 * m;
        // On an array the reduction runs inside ONE slice, so these must be ARRAY views
        // pinned to it -- a plain Texture2D view would silently address slice 0 every time.
        if (m_slices > 1) {
            u.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2DARRAY;
            u.Texture2DArray.FirstArraySlice = slice;
            u.Texture2DArray.ArraySize = 1;
            u.Texture2DArray.MipSlice = m;
        } else {
            u.Texture2D.MipSlice = m;
        }
        gpu.Device()->CreateUnorderedAccessView(m_res.Get(), nullptr, &u,
                                                gpu.SrvHeap().Cpu(slot));
        if (m_slices > 1) {
            u.Texture2DArray.MipSlice = m + 1;
        } else {
            u.Texture2D.MipSlice = m + 1;
        }
        gpu.Device()->CreateUnorderedAccessView(m_res.Get(), nullptr, &u,
                                                gpu.SrvHeap().Cpu(slot + 1));

        const uint32_t consts[4] = {dw, dh, sw, sh};
        cl->SetComputeRoot32BitConstants(0, 4, consts, 0);
        cl->SetComputeRootDescriptorTable(1, gpu.SrvHeap().Gpu(slot));
        cl->Dispatch((dw + 7) / 8, (dh + 7) / 8, 1);

        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
        b.UAV.pResource = m_res.Get();
        cl->ResourceBarrier(1, &b);
    }
}

void TileAtlas2D::RequestMap(uint32_t slice, uint32_t mip, uint32_t tx, uint32_t ty) {
    if (mip >= m_standardMips || slice >= m_slices) return;   // the tail is pinned, not asked
    const MipInfo& mi = m_mip[mip];
    if (tx >= mi.tilesX || ty >= mi.tilesY) return;
    const uint32_t i = slice * m_tilesPerSlice + mi.base + ty * mi.tilesX + tx;
    if (m_state[i] == 0) {
        m_state[i] = 2;   // pending map
        m_pendingMap.push_back(i);
    }
}

void TileAtlas2D::RequestUnmap(uint32_t slice, uint32_t mip, uint32_t tx, uint32_t ty) {
    if (mip >= m_standardMips || slice >= m_slices) return;
    const MipInfo& mi = m_mip[mip];
    if (tx >= mi.tilesX || ty >= mi.tilesY) return;
    const uint32_t i = slice * m_tilesPerSlice + mi.base + ty * mi.tilesX + tx;
    if (m_state[i] == 1) {
        m_state[i] = 3;   // pending unmap
        m_pendingUnmap.push_back(i);
    }
}

// Flat state index -> (mip, tx, ty). Linear over at most a dozen mips; called only while
// building mapping batches, never per texel.
void TileAtlas2D::Decode(uint32_t i, uint32_t& slice, uint32_t& mip, uint32_t& tx,
                         uint32_t& ty) const {
    slice = m_tilesPerSlice ? (i / m_tilesPerSlice) : 0;
    const uint32_t i0 = m_tilesPerSlice ? (i % m_tilesPerSlice) : i;
    for (uint32_t m = m_standardMips; m-- > 0;) {
        if (i0 >= m_mip[m].base) {
            const uint32_t r = i0 - m_mip[m].base;
            mip = m;
            tx = r % m_mip[m].tilesX;
            ty = r / m_mip[m].tilesX;
            return;
        }
    }
    mip = 0;
    tx = i0 % m_tilesX;
    ty = i0 / m_tilesX;
}

void TileAtlas2D::RequestUnmapAll() {
    for (uint32_t i = 0; i < m_state.size(); ++i) {
        if (m_state[i] == 1) {
            m_state[i] = 3;
            m_pendingUnmap.push_back(i);
        }
    }
}

void TileAtlas2D::CommitMappings(Gpu& gpu, std::vector<uint32_t>* outNewlyMapped) {
    // ---- unmaps first (their pool tiles feed the maps below)
    if (!m_pendingUnmap.empty()) {
        std::vector<D3D12_TILED_RESOURCE_COORDINATE> coords;
        std::vector<D3D12_TILE_REGION_SIZE> sizes;
        std::vector<D3D12_TILE_RANGE_FLAGS> flags;
        std::vector<UINT> starts, counts;
        for (uint32_t i : m_pendingUnmap) {
            if (m_state[i] != 3) continue;
            uint32_t sl = 0, mp = 0, cx = 0, cy = 0;
            Decode(i, sl, mp, cx, cy);
            coords.push_back({cx, cy, 0, mp + sl * m_mipCount});
            sizes.push_back({1, FALSE, 0, 0, 0});
            flags.push_back(D3D12_TILE_RANGE_FLAG_NULL);
            starts.push_back(0);
            counts.push_back(1);
            m_freeTiles.push_back(m_tilePool[i]);
            m_state[i] = 0;
        }
        if (!coords.empty()) {
            gpu.Queue()->UpdateTileMappings(m_res.Get(), static_cast<UINT>(coords.size()),
                                            coords.data(), sizes.data(), nullptr,
                                            static_cast<UINT>(flags.size()), flags.data(),
                                            starts.data(), counts.data(),
                                            D3D12_TILE_MAPPING_FLAG_NONE);
        }
        m_pendingUnmap.clear();
    }

    // ---- maps, grouped per heap (one UpdateTileMappings can only reference one heap)
    if (!m_pendingMap.empty()) {
        struct Batch {
            std::vector<D3D12_TILED_RESOURCE_COORDINATE> coords;
            std::vector<D3D12_TILE_REGION_SIZE> sizes;
            std::vector<D3D12_TILE_RANGE_FLAGS> flags;
            std::vector<UINT> starts, counts;
        };
        std::vector<Batch> batches;
        for (uint32_t i : m_pendingMap) {
            if (m_state[i] != 2) continue;
            if (m_freeTiles.empty()) {
                D3D12_HEAP_DESC hd{};
                hd.SizeInBytes = static_cast<uint64_t>(m_heapChunkTiles) * kTileBytes;
                D3D12_HEAP_PROPERTIES hp{};
                hp.Type = D3D12_HEAP_TYPE_DEFAULT;
                hd.Properties = hp;
                hd.Flags = D3D12_HEAP_FLAG_ALLOW_ONLY_NON_RT_DS_TEXTURES;
                Com<ID3D12Heap> heap;
                GA_CHECK(gpu.Device()->CreateHeap(&hd, IID_PPV_ARGS(&heap)));
                heap->SetName(L"atlas.heapChunk");
                const uint32_t hIdx = static_cast<uint32_t>(m_heaps.size());
                m_heaps.push_back(heap);
                for (uint32_t t = m_heapChunkTiles; t > 0; --t) {
                    m_freeTiles.push_back((hIdx << 16) | (t - 1));
                }
            }
            const uint32_t slot = m_freeTiles.back();
            m_freeTiles.pop_back();
            const uint32_t heapIdx = slot >> 16;
            if (heapIdx >= batches.size()) batches.resize(m_heaps.size());
            Batch& b = batches[heapIdx];
            uint32_t sl = 0, mp = 0, cx = 0, cy = 0;
            Decode(i, sl, mp, cx, cy);
            b.coords.push_back({cx, cy, 0, mp + sl * m_mipCount});
            b.sizes.push_back({1, FALSE, 0, 0, 0});
            b.flags.push_back(D3D12_TILE_RANGE_FLAG_NONE);
            b.starts.push_back(slot & 0xFFFF);
            b.counts.push_back(1);
            m_tilePool[i] = slot;
            m_state[i] = 1;
            if (outNewlyMapped) outNewlyMapped->push_back(i);
        }
        for (size_t h = 0; h < batches.size(); ++h) {
            Batch& b = batches[h];
            if (b.coords.empty()) continue;
            gpu.Queue()->UpdateTileMappings(m_res.Get(), static_cast<UINT>(b.coords.size()),
                                            b.coords.data(), b.sizes.data(), m_heaps[h].Get(),
                                            static_cast<UINT>(b.flags.size()), b.flags.data(),
                                            b.starts.data(), b.counts.data(),
                                            D3D12_TILE_MAPPING_FLAG_NONE);
        }
        m_pendingMap.clear();
    }

    RebuildResidencyMap(gpu);
    m_residentList.clear();
    for (uint32_t i = 0; i < m_state.size(); ++i) {
        if (m_state[i] == 1) m_residentList.push_back(i);
    }
}

// ================================================================================ TileAtlas3D

void TileAtlas3D::Init(Gpu& gpu, uint32_t w, uint32_t h, uint32_t d, DXGI_FORMAT fmt,
                       const wchar_t* name, uint32_t heapChunkTiles) {
    m_heapChunkTiles = heapChunkTiles;
    GA_CHECK(gpu.TiledTier() >= D3D12_TILED_RESOURCES_TIER_3
                 ? S_OK
                 : E_FAIL);   // the volume bank is a tier-3 feature; M0 verified this GPU

    D3D12_RESOURCE_DESC rd{};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE3D;
    rd.Width = w;
    rd.Height = h;
    rd.DepthOrArraySize = static_cast<UINT16>(d);
    rd.MipLevels = 1;
    rd.Format = fmt;
    rd.SampleDesc.Count = 1;
    rd.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
    rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
    GA_CHECK(gpu.Device()->CreateReservedResource(&rd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                                 nullptr, IID_PPV_ARGS(&m_res)));
    m_res->SetName(name);

    UINT numTiles = 0;
    D3D12_PACKED_MIP_INFO packed{};
    D3D12_TILE_SHAPE shape{};
    UINT numSub = 1;
    D3D12_SUBRESOURCE_TILING sub{};
    gpu.Device()->GetResourceTiling(m_res.Get(), &numTiles, &packed, &shape, &numSub, 0, &sub);
    m_tileW = shape.WidthInTexels;
    m_tileH = shape.HeightInTexels;
    m_tileD = shape.DepthInTexels;
    m_tilesX = sub.WidthInTiles;
    m_tilesY = sub.HeightInTiles;
    m_tilesZ = sub.DepthInTiles;
    m_state.assign(static_cast<size_t>(m_tilesX) * m_tilesY * m_tilesZ, 0);
    m_tilePool.assign(m_state.size(), 0);

    m_srv = gpu.CreateSrv3D(m_res.Get(), fmt);
    m_uav = gpu.CreateTextureUav(m_res.Get(), fmt, D3D12_UAV_DIMENSION_TEXTURE3D);

    Log("[atlas] %S: %ux%ux%u texels = %ux%ux%u tiles of %ux%ux%u (%.0f MB virtual, "
        "resident on demand)",
        name, w, h, d, m_tilesX, m_tilesY, m_tilesZ, m_tileW, m_tileH, m_tileD,
        VirtualBytes() / 1048576.0);
}

void TileAtlas3D::RequestMap(uint32_t tx, uint32_t ty, uint32_t tz) {
    const size_t i = (static_cast<size_t>(tz) * m_tilesY + ty) * m_tilesX + tx;
    if (m_state[i] == 0) {
        m_state[i] = 2;
        m_pendingMap.push_back(static_cast<uint32_t>(i));
    }
}

void TileAtlas3D::RequestUnmapAll() {
    std::vector<D3D12_TILED_RESOURCE_COORDINATE> coords;
    std::vector<D3D12_TILE_REGION_SIZE> sizes;
    std::vector<D3D12_TILE_RANGE_FLAGS> flags;
    std::vector<UINT> starts, counts;
    for (uint32_t i = 0; i < m_state.size(); ++i) {
        if (m_state[i] != 1) continue;
        const uint32_t tx = i % m_tilesX;
        const uint32_t ty = (i / m_tilesX) % m_tilesY;
        const uint32_t tz = i / (m_tilesX * m_tilesY);
        coords.push_back({tx, ty, tz, 0});
        sizes.push_back({1, FALSE, 0, 0, 0});
        flags.push_back(D3D12_TILE_RANGE_FLAG_NULL);
        starts.push_back(0);
        counts.push_back(1);
        m_freeTiles.push_back(m_tilePool[i]);
        m_state[i] = 0;
    }
    // The queue-side call needs a Gpu; defer actual unmapping to the next CommitMappings by
    // stashing the coords? Simpler and sufficient for the cloud bank: the caller always
    // follows RequestUnmapAll with CommitMappings, and pending state 0 tiles need no call --
    // EXCEPT the GPU mapping still exists. So this path records them as pending-null.
    for (size_t k = 0; k < coords.size(); ++k) {
        m_pendingNull.push_back(coords[k]);
    }
}

void TileAtlas3D::CommitMappings(Gpu& gpu, std::vector<uint32_t>* outNewlyMapped) {
    if (!m_pendingNull.empty()) {
        std::vector<D3D12_TILE_REGION_SIZE> sizes(m_pendingNull.size(), {1, FALSE, 0, 0, 0});
        std::vector<D3D12_TILE_RANGE_FLAGS> flags(m_pendingNull.size(),
                                                  D3D12_TILE_RANGE_FLAG_NULL);
        std::vector<UINT> starts(m_pendingNull.size(), 0), counts(m_pendingNull.size(), 1);
        gpu.Queue()->UpdateTileMappings(m_res.Get(), static_cast<UINT>(m_pendingNull.size()),
                                        m_pendingNull.data(), sizes.data(), nullptr,
                                        static_cast<UINT>(flags.size()), flags.data(),
                                        starts.data(), counts.data(),
                                        D3D12_TILE_MAPPING_FLAG_NONE);
        m_pendingNull.clear();
    }
    if (!m_pendingMap.empty()) {
        struct Batch {
            std::vector<D3D12_TILED_RESOURCE_COORDINATE> coords;
            std::vector<D3D12_TILE_REGION_SIZE> sizes;
            std::vector<D3D12_TILE_RANGE_FLAGS> flags;
            std::vector<UINT> starts, counts;
        };
        std::vector<Batch> batches;
        for (uint32_t i : m_pendingMap) {
            if (m_state[i] != 2) continue;
            if (m_freeTiles.empty()) {
                D3D12_HEAP_DESC hd{};
                hd.SizeInBytes =
                    static_cast<uint64_t>(m_heapChunkTiles) * TileAtlas2D::kTileBytes;
                D3D12_HEAP_PROPERTIES hp{};
                hp.Type = D3D12_HEAP_TYPE_DEFAULT;
                hd.Properties = hp;
                hd.Flags = D3D12_HEAP_FLAG_ALLOW_ONLY_NON_RT_DS_TEXTURES;
                Com<ID3D12Heap> heap;
                GA_CHECK(gpu.Device()->CreateHeap(&hd, IID_PPV_ARGS(&heap)));
                heap->SetName(L"atlas3d.heapChunk");
                const uint32_t hIdx = static_cast<uint32_t>(m_heaps.size());
                m_heaps.push_back(heap);
                for (uint32_t t = m_heapChunkTiles; t > 0; --t) {
                    m_freeTiles.push_back((hIdx << 16) | (t - 1));
                }
            }
            const uint32_t slot = m_freeTiles.back();
            m_freeTiles.pop_back();
            const uint32_t heapIdx = slot >> 16;
            if (heapIdx >= batches.size()) batches.resize(m_heaps.size());
            Batch& b = batches[heapIdx];
            const uint32_t tx = i % m_tilesX;
            const uint32_t ty = (i / m_tilesX) % m_tilesY;
            const uint32_t tz = i / (m_tilesX * m_tilesY);
            b.coords.push_back({tx, ty, tz, 0});
            b.sizes.push_back({1, FALSE, 0, 0, 0});
            b.flags.push_back(D3D12_TILE_RANGE_FLAG_NONE);
            b.starts.push_back(slot & 0xFFFF);
            b.counts.push_back(1);
            m_tilePool[i] = slot;
            m_state[i] = 1;
            if (outNewlyMapped) outNewlyMapped->push_back(i);
        }
        for (size_t h = 0; h < batches.size(); ++h) {
            Batch& b = batches[h];
            if (b.coords.empty()) continue;
            gpu.Queue()->UpdateTileMappings(m_res.Get(), static_cast<UINT>(b.coords.size()),
                                            b.coords.data(), b.sizes.data(), m_heaps[h].Get(),
                                            static_cast<UINT>(b.flags.size()), b.flags.data(),
                                            b.starts.data(), b.counts.data(),
                                            D3D12_TILE_MAPPING_FLAG_NONE);
        }
        m_pendingMap.clear();
    }

    m_residentList.clear();
    for (uint32_t i = 0; i < m_state.size(); ++i) {
        if (m_state[i] == 1) m_residentList.push_back(i);
    }
}

// ================================================================================ M4 self-tests

namespace {

// Brute-force Cl(2) product on a full multivector, mirroring GA.hlsli's GeometricProduct.
struct Mv2c {
    double s = 0, x = 0, y = 0, b = 0;
};
Mv2c Mul(const Mv2c& a, const Mv2c& b) {
    Mv2c r;
    r.s = a.s * b.s + a.x * b.x + a.y * b.y - a.b * b.b;
    r.x = a.s * b.x + a.x * b.s - a.y * b.b + a.b * b.y;
    r.y = a.s * b.y + a.y * b.s + a.x * b.b - a.b * b.x;
    r.b = a.s * b.b + a.b * b.s + a.x * b.y - a.y * b.x;
    return r;
}
uint8_t SigOf(const Mv2c& m) {
    uint8_t s = 0;
    if (m.s != 0) s |= 1;
    if (m.x != 0 || m.y != 0) s |= 2;
    if (m.b != 0) s |= 4;
    return s;
}
Mv2c FromSig(uint8_t sig, double v0, double v1, double v2) {
    Mv2c m;
    if (sig & 1) m.s = v0;
    if (sig & 2) { m.x = v1; m.y = v1 * 0.7 + 0.1; }
    if (sig & 4) m.b = v2;
    return m;
}

bool TestCayleyClosure() {
    // Soundness: the product of any two multivectors with grade sets A, B only ever produces
    // grades inside Cl2ProductSignature(A, B). Tightness: every predicted grade is HIT by some
    // basis-blade pair -- a signature that over-promises would waste kernels, one that
    // under-promises would drop physics.
    bool pass = true;
    for (uint8_t a = 1; a < 8; ++a) {
        for (uint8_t b = 1; b < 8; ++b) {
            const uint8_t pred = Cl2ProductSignature(a, b);
            for (int trial = 0; trial < 8; ++trial) {
                const Mv2c ma = FromSig(a, 1.1 + trial, 0.6 - 0.3 * trial, 2.0 + 0.5 * trial);
                const Mv2c mb = FromSig(b, -0.7 + trial, 1.3 + 0.2 * trial, 0.9 - trial);
                const uint8_t got = SigOf(Mul(ma, mb));
                if (got & ~pred) {
                    Log("[atlastest] closure UNSOUND: sig(%u,%u) predicted %u, product hit %u",
                        a, b, pred, got);
                    pass = false;
                }
            }
            uint8_t hit = 0;
            const uint8_t blades[3] = {1, 2, 4};
            for (uint8_t ba : blades) {
                for (uint8_t bb : blades) {
                    if ((a & ba) && (b & bb)) {
                        // Distinct operand values: identical vectors have a zero wedge, which
                        // would falsely hide the grade-2 output of g1*g1.
                        hit |= SigOf(Mul(FromSig(ba, 1, 1, 1), FromSig(bb, 2, 3, 5)));
                    }
                }
            }
            if (hit != pred) {
                Log("[atlastest] closure NOT TIGHT: sig(%u,%u) predicted %u, blades reach %u",
                    a, b, pred, hit);
                pass = false;
            }
        }
    }
    if (pass) Log("[atlastest] Cl(2) Cayley signature closure: sound and tight for all 49 pairs");
    return pass;
}

}  // namespace

bool RunAtlasSelfTest(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    Log("[atlastest] ---- M4 gate: TileAtlas2D residency + grade signatures ----");
    bool pass = TestCayleyClosure();

    // A small live atlas: map three tiles, clear them, stamp the test pattern over the resident
    // list, verify pattern-inside / zero-outside, then unmap one and verify it vanishes.
    TileAtlas2D atlas;
    atlas.Init(gpu, 1024, 512, DXGI_FORMAT_R16_FLOAT, L"atlastest.bank", 8, 4);

    ShaderBlob csClear = sc.Compile(shaderDir + L"/SeaChurn.hlsl", L"CsChurnClear", L"cs_6_0");
    ShaderBlob csStamp = sc.Compile(shaderDir + L"/SeaChurn.hlsl", L"CsChurnTestPattern",
                                    L"cs_6_0");
    if (!csClear.Valid() || !csStamp.Valid()) {
        Log("[atlastest] SeaChurn kernels failed to compile");
        return false;
    }

    // Minimal root signature matching SeaChurn.hlsl: b0 CBV, t0 root SRV (tile list), table
    // [t1 srv, u0 uav].
    Com<ID3D12RootSignature> rs;
    {
        D3D12_DESCRIPTOR_RANGE1 ranges[2]{};
        ranges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
        ranges[0].NumDescriptors = 1;
        ranges[0].BaseShaderRegister = 1;
        ranges[0].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
        ranges[0].OffsetInDescriptorsFromTableStart = 0;
        ranges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
        ranges[1].NumDescriptors = 1;
        ranges[1].BaseShaderRegister = 0;
        ranges[1].Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
        ranges[1].OffsetInDescriptorsFromTableStart = 1;
        D3D12_ROOT_PARAMETER1 params[3]{};
        params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        params[0].Descriptor.ShaderRegister = 0;
        params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
        params[1].Descriptor.ShaderRegister = 0;
        params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
        params[2].DescriptorTable.NumDescriptorRanges = 2;
        params[2].DescriptorTable.pDescriptorRanges = ranges;
        D3D12_VERSIONED_ROOT_SIGNATURE_DESC vd{};
        vd.Version = D3D_ROOT_SIGNATURE_VERSION_1_1;
        vd.Desc_1_1.NumParameters = _countof(params);
        vd.Desc_1_1.pParameters = params;
        Com<ID3DBlob> blob, err;
        GA_CHECK(D3D12SerializeVersionedRootSignature(&vd, &blob, &err));
        GA_CHECK(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(),
                                                  blob->GetBufferSize(), IID_PPV_ARGS(&rs)));
    }
    auto makePso = [&](ShaderBlob& cs) {
        D3D12_COMPUTE_PIPELINE_STATE_DESC d{};
        d.pRootSignature = rs.Get();
        d.CS = {cs.Data(), cs.Size()};
        Com<ID3D12PipelineState> pso;
        GA_CHECK(gpu.Device()->CreateComputePipelineState(&d, IID_PPV_ARGS(&pso)));
        return pso;
    };
    Com<ID3D12PipelineState> psoClear = makePso(csClear), psoStamp = makePso(csStamp);

    const uint32_t table = gpu.SrvHeap().Alloc(2);
    D3D12_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
    sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    sv.Texture2D.MipLevels = 1;
    gpu.Device()->CreateShaderResourceView(nullptr, &sv, gpu.SrvHeap().Cpu(table + 0));
    D3D12_UNORDERED_ACCESS_VIEW_DESC uv{};
    uv.Format = DXGI_FORMAT_R16_FLOAT;
    uv.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    gpu.Device()->CreateUnorderedAccessView(atlas.Res(), nullptr, &uv,
                                            gpu.SrvHeap().Cpu(table + 1));

    atlas.RequestMap(0, 0);
    atlas.RequestMap(1, 1);
    atlas.RequestMap(2, 0);
    std::vector<uint32_t> fresh;
    atlas.CommitMappings(gpu, &fresh);
    Log("[atlastest] mapped %u tiles (%zu fresh), %u x %u texels each", atlas.ResidentCount(),
        fresh.size(), atlas.TileW(), atlas.TileH());

    struct ChurnCbMini {
        float originX, originZ, texelM, domainM;
        uint32_t tilesX, tileW, tileH, listCount;
        float dt, tau, pad0, pad1;
        float jet[8];
        float misc[4];
    };
    auto runPass = [&](ID3D12PipelineState* pso, const std::vector<uint32_t>& list) {
        if (list.empty()) return;
        auto* cl = gpu.BeginUpload();
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
        cl->SetDescriptorHeaps(1, heaps);
        ChurnCbMini cb{};
        cb.tilesX = atlas.TilesX();
        cb.tileW = atlas.TileW();
        cb.tileH = atlas.TileH();
        cb.listCount = static_cast<uint32_t>(list.size());
        cl->SetComputeRootSignature(rs.Get());
        cl->SetComputeRootConstantBufferView(0, gpu.PushConstants(&cb, sizeof(cb)));
        cl->SetComputeRootShaderResourceView(
            1, gpu.PushConstants(list.data(), list.size() * 4));
        cl->SetComputeRootDescriptorTable(2, gpu.SrvHeap().Gpu(table));
        cl->SetPipelineState(pso);
        cl->Dispatch(atlas.TileW() / 16, atlas.TileH() / 16, cb.listCount);
        gpu.EndUpload();
    };
    runPass(psoClear.Get(), atlas.ResidentList());
    runPass(psoStamp.Get(), atlas.ResidentList());

    auto verify = [&](const char* phase, bool tile11Resident) {
        GpuTexture t;
        t.res = atlas.Res();
        t.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        t.format = DXGI_FORMAT_R16_FLOAT;
        t.width = 1024;
        t.height = 512;
        uint32_t pitch = 0;
        const std::vector<uint8_t> data = gpu.ReadbackTexture(t, &pitch);
        uint32_t bad = 0;
        for (uint32_t ty = 0; ty < atlas.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < atlas.TilesX(); ++tx) {
                const bool resident =
                    atlas.IsResident(tx, ty) && (tile11Resident || !(tx == 1 && ty == 1));
                const uint32_t px = tx * atlas.TileW() + atlas.TileW() / 2;
                const uint32_t py = ty * atlas.TileH() + atlas.TileH() / 2;
                const uint16_t* row = reinterpret_cast<const uint16_t*>(
                    data.data() + static_cast<size_t>(py) * pitch);
                const float v = HalfToFloat(row[px]);
                const float expect = resident
                                         ? static_cast<float>(ty * atlas.TilesX() + tx + 1)
                                         : 0.0f;
                if (v != expect) {
                    Log("[atlastest] %s: tile (%u,%u) centre = %g, expected %g", phase, tx, ty,
                        v, expect);
                    ++bad;
                }
            }
        }
        Log("[atlastest] %s: %s", phase, bad ? "FAILED" : "every tile as predicted by residency");
        return bad == 0;
    };
    pass &= verify("stamped", true);

    atlas.RequestUnmap(1, 1);
    atlas.CommitMappings(gpu, nullptr);
    pass &= verify("after-unmap", false);

    // ---- M9h: THE REDUCTION, on values that are known rather than plausible. The bank is
    // stamped so every texel of resident tile (tx,ty) carries ty*tilesX+tx+1, and tile (1,1)
    // was just unmapped. A mip-1 texel well inside a tile's footprint averages four identical
    // fine texels, so it must equal that tile's stamp exactly -- and inside the unmapped
    // tile's footprint it must be 0, because a null tile reads zero and averaging zeros is
    // still zero. That second case is the one that matters: it proves the coarse level
    // inherits the FIELD's semantics instead of inventing coverage.
    {
        atlas.MapAllCoarse();
        atlas.CommitMappings(gpu, nullptr);
        auto* mcl = gpu.BeginUpload();
        ID3D12DescriptorHeap* mheaps[] = {gpu.SrvHeap().Heap()};
        mcl->SetDescriptorHeaps(1, mheaps);
        atlas.BuildMips(gpu, sc, shaderDir, mcl);
        gpu.EndUpload();

        GpuTexture t1;
        t1.res = atlas.Res();
        t1.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        t1.format = DXGI_FORMAT_R16_FLOAT;
        t1.width = 512;
        t1.height = 256;
        uint32_t pitch1 = 0;
        const std::vector<uint8_t> d1 = gpu.ReadbackTexture(t1, &pitch1, 1);
        uint32_t badM = 0;
        for (uint32_t ty = 0; ty < atlas.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < atlas.TilesX(); ++tx) {
                // centre of this mip-0 tile's footprint, in mip-1 texels
                const uint32_t px = (tx * atlas.TileW() + atlas.TileW() / 2) / 2;
                const uint32_t py = (ty * atlas.TileH() + atlas.TileH() / 2) / 2;
                const uint16_t* row = reinterpret_cast<const uint16_t*>(
                    d1.data() + static_cast<size_t>(py) * pitch1);
                const float v = HalfToFloat(row[px]);
                const bool resident = atlas.IsResident(tx, ty);
                const float expect =
                    resident ? static_cast<float>(ty * atlas.TilesX() + tx + 1) : 0.0f;
                if (v != expect) {
                    Log("[atlastest] mip1: tile (%u,%u) = %g, expected %g", tx, ty, v, expect);
                    ++badM;
                }
            }
        }
        Log("[atlastest] mip1 reduction: %s (null tiles averaged to 0, as the field says)",
            badM ? "FAILED" : "every coarse texel is the mean of its four");
        pass &= (badM == 0);
    }

    Log("[atlastest] resident %u tiles = %.2f MB of %.0f MB virtual", atlas.ResidentCount(),
        atlas.ResidentBytes() / 1048576.0, atlas.VirtualBytes() / 1048576.0);

    // ---- M9h: THE CHAIN. The no-pop contract is a claim about the RESOURCE, so it is proved
    // on the resource rather than argued from the render. Three things must hold, and the
    // third is the one everything else leans on.
    {
        TileAtlas2D chain;
        chain.Init(gpu, 2048, 2048, DXGI_FORMAT_R16_FLOAT, L"atlastest.chain", 64, 6);
        bool okC = true;

        // 1. The chain exists and D3D12 split it where the docs say it does.
        Log("[atlastest] chain: %u mips (%u standard + %u packed)", chain.MipCount(),
            chain.StandardMips(), chain.MipCount() - chain.StandardMips());
        if (chain.MipCount() < 2 || chain.StandardMips() == 0) {
            Log("[atlastest] chain: FAILED -- no usable mip chain");
            okC = false;
        }

        // 2. THE FLOOR. With nothing whatsoever requested, every mip-0 region must still
        // resolve to a real resident mip -- that is the pinned packed tail doing its job, and
        // it is what makes a miss impossible. If this fails, sampling can hit NULL and the
        // "degrade to blur, never garbage" promise is empty.
        uint32_t worst = 0;
        for (uint32_t ty = 0; ty < chain.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < chain.TilesX(); ++tx) {
                const uint32_t f = chain.FinestResident(tx, ty);
                if (f > worst) worst = f;
                if (f >= chain.MipCount()) okC = false;
            }
        }
        Log("[atlastest] chain: cold floor -- every region resolves, coarsest %u of %u %s",
            worst, chain.MipCount() - 1, (worst == chain.MipCount() - 1) ? "(the pinned tail)"
                                                                        : "");

        // 3. A fine tile going resident SHARPENS exactly its own region and nothing else --
        // residency is the only thing that changed, and it is local.
        const uint32_t before = chain.FinestResident(0, 0);
        chain.RequestMap(0, 0, 0);
        chain.CommitMappings(gpu, nullptr);
        const uint32_t after = chain.FinestResident(0, 0);
        const uint32_t neigh = chain.FinestResident(chain.TilesX() - 1, chain.TilesY() - 1);
        if (!(after == 0 && before > 0)) {
            Log("[atlastest] chain: FAILED -- mapping mip 0 tile (0,0) moved finest %u -> %u",
                before, after);
            okC = false;
        }
        if (neigh != worst) {
            Log("[atlastest] chain: FAILED -- a far region changed (%u -> %u) from a local map",
                worst, neigh);
            okC = false;
        }
        Log("[atlastest] chain: mapping one mip-0 tile moved its own region %u -> %u, far "
            "region unchanged at %u",
            before, after, neigh);

        // 4. And it is reversible: unmapping returns that region to the coarser truth without
        // ever passing through "nothing".
        chain.RequestUnmap(0, 0, 0);
        chain.CommitMappings(gpu, nullptr);
        const uint32_t back = chain.FinestResident(0, 0);
        if (back != before) {
            Log("[atlastest] chain: FAILED -- unmap left finest at %u, expected %u", back,
                before);
            okC = false;
        }
        if (chain.ResidencyMapSrv() == UINT32_MAX) {
            Log("[atlastest] chain: FAILED -- no residency map for a chained atlas");
            okC = false;
        }
        Log("[atlastest] chain: %s", okC ? "the floor holds, residency is the only variable"
                                         : "FAILED");
        pass &= okC;
    }

    // ---- M9h: THE ARRAY. Slices are PAGES of the shared (level, x, y) address space, so the
    // property that matters is ISOLATION: a mapping in one slice must not disturb another.
    // Get the subresource index wrong (it is mip + slice * mipCount) and everything still
    // runs, tiles still map, nothing errors -- the pages just quietly alias each other.
    {
        TileAtlas2D arr;
        arr.Init(gpu, 4096, 4096, DXGI_FORMAT_R16_FLOAT, L"atlastest.array", 64, 5, 16);
        bool okA = true;
        Log("[atlastest] array: %u slices, %u mips (%u standard); every slice starts INACTIVE "
            "(slice 0 finest = %s)",
            arr.Slices(), arr.MipCount(), arr.StandardMips(),
            arr.FinestResident(0, 0, 0) == TileAtlas2D::kNothingResident ? "nothing" : "?!");

        // 1. An un-activated slice has NO floor and must say so, rather than naming a level
        // that is NULL. This is the honest-absence case the whole design leans on.
        if (arr.FinestResident(3, 0, 0) != TileAtlas2D::kNothingResident) {
            Log("[atlastest] array: FAILED -- inactive slice 3 claims a resident level");
            okA = false;
        }

        // 2. Activation gives that slice, and only that slice, a floor.
        arr.ActivateSlice(gpu, 3);
        const uint32_t f3 = arr.FinestResident(3, 0, 0);
        const uint32_t f4 = arr.FinestResident(4, 0, 0);
        if (f3 == TileAtlas2D::kNothingResident || f4 != TileAtlas2D::kNothingResident) {
            Log("[atlastest] array: FAILED -- activation leaked (slice3 %u, slice4 %u)", f3, f4);
            okA = false;
        }

        // 3. ISOLATION. Map one mip-0 tile in slice 3; slice 5 (also active) must not move.
        arr.ActivateSlice(gpu, 5);
        const uint32_t before5 = arr.FinestResident(5, 0, 0);
        arr.RequestMap(3u, 0u, 0u, 0u);
        arr.CommitMappings(gpu, nullptr);
        const uint32_t after3 = arr.FinestResident(3, 0, 0);
        const uint32_t after5 = arr.FinestResident(5, 0, 0);
        if (after3 != 0) {
            Log("[atlastest] array: FAILED -- slice 3 mip0 mapped but finest is %u", after3);
            okA = false;
        }
        if (after5 != before5) {
            Log("[atlastest] array: FAILED -- slice 5 moved %u -> %u from a slice-3 mapping",
                before5, after5);
            okA = false;
        }
        Log("[atlastest] array: slice3 floor %u -> %u after its own map; slice5 unmoved at %u",
            f3, after3, after5);
        Log("[atlastest] array: %s", okA ? "pages are isolated; slices do not alias" : "FAILED");
        pass &= okA;
    }

    // ---- M9h: HOW BIG CAN A RESERVED RESOURCE ACTUALLY BE? A reserved resource buys a huge
    // VIRTUAL memory space, but memory and EXTENT are different axes and it was not obvious
    // (to me) which one bounds a planet-scale tree. Measured rather than asserted, because the
    // answer decides whether one bank can span globe-to-centimetre or whether the tree has to
    // be addressed across multiple resources. Creating these costs nothing: no tile is mapped,
    // so nothing is committed.
    {
        Log("[atlastest] reserved-resource extent probe (virtual only, nothing committed):");
        const uint32_t dims[] = {16384u, 32768u, 65536u, 131072u, 1048576u};
        uint32_t largest = 0;
        for (uint32_t d : dims) {
            D3D12_RESOURCE_DESC rd{};
            rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
            rd.Width = d;
            rd.Height = d;
            rd.DepthOrArraySize = 1;
            rd.MipLevels = 1;
            rd.Format = DXGI_FORMAT_R16_FLOAT;
            rd.SampleDesc.Count = 1;
            rd.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
            rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
            Com<ID3D12Resource> probe;
            const HRESULT hr = gpu.Device()->CreateReservedResource(
                &rd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&probe));
            const double mPerTexel = 40.0e6 / double(d);
            Log("[atlastest]   %7u x %-7u  %s   (Earth: %.2f m/texel)", d, d,
                SUCCEEDED(hr) ? "created" : "REFUSED ", mPerTexel);
            if (SUCCEEDED(hr)) largest = d;
        }
        Log("[atlastest]   largest accepted: %u (%.2f m/texel over Earth's circumference)",
            largest, 40.0e6 / double(largest ? largest : 1));

        // ---- Arrays. The docs are explicit that the array axis caps at 2048
        // (D3D11_REQ_TEXTURE2D_ARRAY_AXIS_DIMENSION) and that Texture2DArray tiles PER SLICE
        // -- "each mip level at a given array slice is a subresource". They are equally
        // explicit that "exhaustion of GPU virtual address space ... may easily occur first",
        // and it does: 16384^2 x 2048 x 2B is a TERABYTE of VA, and asking for it REMOVED THE
        // DEVICE on this driver rather than failing cleanly. So the budget that matters is
        // total virtual BYTES, not slices -- and the sweep stops at the first refusal instead
        // of walking further off the cliff.

        // Tiling first, on a shape too small to be risky: does a reserved ARRAY tile per slice?
        {
            D3D12_RESOURCE_DESC rd{};
            rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
            rd.Width = 4096;
            rd.Height = 4096;
            rd.DepthOrArraySize = 8;
            rd.MipLevels = 5;
            rd.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
            rd.SampleDesc.Count = 1;
            rd.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
            rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
            Com<ID3D12Resource> arr;
            const HRESULT ahr = gpu.Device()->CreateReservedResource(
                &rd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&arr));
            if (FAILED(ahr)) {
                Log("[atlastest] array tiling: REFUSED hr=0x%08X", static_cast<unsigned>(ahr));
            } else {
                UINT nt = 0, nsub = 40;
                D3D12_PACKED_MIP_INFO pk{};
                D3D12_TILE_SHAPE sh{};
                std::vector<D3D12_SUBRESOURCE_TILING> st(nsub);
                gpu.Device()->GetResourceTiling(arr.Get(), &nt, &pk, &sh, &nsub, 0, st.data());
                Log("[atlastest] array tiling: 4096^2 x8 slices x5 mips RGBA16F -> %u tiles, "
                    "%u subresources, tile %ux%u",
                    nt, nsub, sh.WidthInTexels, sh.HeightInTexels);
                Log("[atlastest]   per slice: %u standard mips, %u packed, %u tiles for the "
                    "tail -- that tail cost MULTIPLIES by every resident slice",
                    pk.NumStandardMips, pk.NumPackedMips, pk.NumTilesForPackedMips);
                Log("[atlastest]   slice0 mip0 %ux%u tiles, slice1 mip0 %ux%u tiles: per-slice "
                    "tiling %s",
                    st[0].WidthInTiles, st[0].HeightInTiles, st[5].WidthInTiles,
                    st[5].HeightInTiles,
                    (st[5].WidthInTiles == st[0].WidthInTiles) ? "CONFIRMED" : "DIFFERS");
            }
        }

        // Now the VA ceiling, in total virtual bytes, stopping at the first refusal.
        Log("[atlastest] virtual-address ceiling sweep (stops at first refusal):");
        const uint64_t gb = 1024ull * 1024ull * 1024ull;
        const uint64_t want[] = {16 * gb, 64 * gb, 128 * gb, 256 * gb, 512 * gb, 1024 * gb};
        uint64_t okVa = 0;
        for (uint64_t bytes : want) {
            const uint32_t slices = static_cast<uint32_t>(bytes / (16384ull * 16384ull * 2ull));
            if (slices == 0 || slices > 2048) continue;
            D3D12_RESOURCE_DESC rd{};
            rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
            rd.Width = 16384;
            rd.Height = 16384;
            rd.DepthOrArraySize = static_cast<UINT16>(slices);
            rd.MipLevels = 1;
            rd.Format = DXGI_FORMAT_R16_FLOAT;
            rd.SampleDesc.Count = 1;
            rd.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
            rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
            Com<ID3D12Resource> probe;
            const HRESULT hr = gpu.Device()->CreateReservedResource(
                &rd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&probe));
            Log("[atlastest]   %4llu GB virtual (%u slices)  %s", bytes / gb, slices,
                SUCCEEDED(hr) ? "created" : "REFUSED -- stopping");
            if (FAILED(hr)) break;
            okVa = bytes;
        }
        Log("[atlastest]   virtual address space is the budget, not the slice count: %llu GB "
            "accepted in one reserved array",
            okVa / gb);
    }

    Log("[atlastest] ---- %s ----", pass ? "PASS" : "FAIL");
    return pass;
}

}  // namespace ga
