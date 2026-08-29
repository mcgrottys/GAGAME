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
                       const wchar_t* name, uint32_t heapChunkTiles) {
    m_heapChunkTiles = heapChunkTiles;

    D3D12_RESOURCE_DESC rd{};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    rd.Width = widthTexels;
    rd.Height = heightTexels;
    rd.DepthOrArraySize = 1;
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
    m_tilesX = sub.WidthInTiles;
    m_tilesY = sub.HeightInTiles;
    m_state.assign(static_cast<size_t>(m_tilesX) * m_tilesY, 0);
    m_tilePool.assign(m_state.size(), 0);

    m_srv = gpu.CreateSrv(m_res.Get(), fmt);
    m_uav = gpu.CreateTextureUav(m_res.Get(), fmt, D3D12_UAV_DIMENSION_TEXTURE2D);

    Log("[atlas] %S: %ux%u texels = %ux%u tiles of %ux%u (%.0f MB virtual, resident on demand)",
        name, widthTexels, heightTexels, m_tilesX, m_tilesY, m_tileW, m_tileH,
        VirtualBytes() / 1048576.0);
}

void TileAtlas2D::RequestMap(uint32_t tx, uint32_t ty) {
    const uint32_t i = ty * m_tilesX + tx;
    if (m_state[i] == 0) {
        m_state[i] = 2;   // pending map
        m_pendingMap.push_back(i);
    }
}

void TileAtlas2D::RequestUnmap(uint32_t tx, uint32_t ty) {
    const uint32_t i = ty * m_tilesX + tx;
    if (m_state[i] == 1) {
        m_state[i] = 3;   // pending unmap
        m_pendingUnmap.push_back(i);
    }
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
            coords.push_back({i % m_tilesX, i / m_tilesX, 0, 0});
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
            b.coords.push_back({i % m_tilesX, i / m_tilesX, 0, 0});
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
    atlas.Init(gpu, 1024, 512, DXGI_FORMAT_R16_FLOAT, L"atlastest.bank", 8);

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

    Log("[atlastest] resident %u tiles = %.2f MB of %.0f MB virtual", atlas.ResidentCount(),
        atlas.ResidentBytes() / 1048576.0, atlas.VirtualBytes() / 1048576.0);
    Log("[atlastest] ---- %s ----", pass ? "PASS" : "FAIL");
    return pass;
}

}  // namespace ga
