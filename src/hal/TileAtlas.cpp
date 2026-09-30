#include "hal/TileAtlas.h"

#include "compose/SurfaceFrame.h"
#include "core/Lattice.h"
#include "core/Space.h"
#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Root.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
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

// ---- HIERARCHY step 0: the probe array. A page tenant's shape at toy size: 1024^2 RGBA8 is 8x8
// tiles of 128x128 at mip 0, and four mips stop at the one-tile level, so there is no packed tail
// (the tenants' rule, Residency.cpp) and CopyTiles reaches every tile. Four slices: three for the
// shared tile, one for the WRAP probe.
constexpr uint32_t kProbeDim = 1024, kProbeSlices = 4, kProbeMips = 4;
constexpr uint32_t kProbeTileTexels = 128;   // RGBA8: 64 KB = 128 x 128 x 4 bytes
constexpr uint32_t kProbeTileBytes = D3D12_TILED_RESOURCE_TILE_SIZE_IN_BYTES;
// One upload block per pattern, and the heap tile of the same index that holds it.
enum ProbeBlock : uint32_t {
    kBlockA, kBlockB,                        // the shared tile's pattern, the planted tile's
    kBlockL0, kBlockR0, kBlockL1, kBlockR1,  // the WRAP probe's edges: left/right, mip 0/1
    kProbeBlocks
};
constexpr uint32_t kNullTile = UINT32_MAX;   // MapProbeTiles: this place goes to NULL

constexpr UINT ProbeSub(uint32_t slice, uint32_t mip) { return mip + slice * kProbeMips; }

// Pattern A, byte i of one tile's LINEAR layout (CopyTiles' side: 128 texels a row, 4 bytes a
// texel, so texel t = i / 4 sits at x = t % 128, y = t / 128). Red = x + 1 and green = y + 1
// are a different pair at every texel, so a swizzle or an offset error cannot hide; blue
// scrambles t and alpha carries x ^ y. No byte is 0, so a NULL tile's zeros match nothing, and
// none is 0xFF, so pattern B -- the planted tile's -- is A with every bit flipped: nonzero as
// well, and different from A at every one of the 65536 bytes.
uint8_t PatternA(uint32_t i) {
    const uint32_t t = i >> 2, x = t & 127u, y = t >> 7;
    switch (i & 3u) {
        case 0: return static_cast<uint8_t>(1u + x);
        case 1: return static_cast<uint8_t>(1u + y);
        case 2: return static_cast<uint8_t>(1u + (t * 97u) % 251u);
        default: return static_cast<uint8_t>(0x40u | ((x ^ y) & 0x3Fu));
    }
}
uint8_t PatternB(uint32_t i) { return static_cast<uint8_t>(~PatternA(i)); }

struct TileMatch {
    uint32_t same = 0;      // bytes equal to the pattern, of 65536
    uint32_t zero = 0;      // bytes that read exactly 0
    uint32_t moved = 0;     // pattern A only: texels of A found whole at ANOTHER texel
    int32_t first = -1;     // byte offset of the first difference; -1 when there is none
    uint8_t want = 0, got = 0;
    bool Holds() const { return same == kProbeTileBytes; }
};

TileMatch MatchTile(const uint8_t* got, uint8_t (*pattern)(uint32_t)) {
    TileMatch m;
    for (uint32_t i = 0; i < kProbeTileBytes; ++i) {
        const uint8_t w = pattern(i);
        if (got[i] == w) {
            ++m.same;
        } else if (m.first < 0) {
            m.first = static_cast<int32_t>(i);
            m.want = w;
            m.got = got[i];
        }
        if (got[i] == 0) ++m.zero;
    }
    // A mismatch made of A's own texels in another order is the same memory read through another
    // swizzle -- a different finding from different memory. Every texel of A names its own
    // position in red and green, so each texel read is checked against where it says it is from.
    if (pattern == PatternA && m.first >= 0) {
        for (uint32_t t = 0; t < kProbeTileBytes / 4; ++t) {
            const uint8_t* p = got + 4 * t;
            if (p[0] < 1 || p[0] > 128 || p[1] < 1 || p[1] > 128) continue;
            const uint32_t from = (p[1] - 1u) * 128u + (p[0] - 1u);
            if (from == t) continue;
            bool whole = true;
            for (uint32_t c = 0; c < 4; ++c) whole = whole && p[c] == PatternA(4 * from + c);
            m.moved += whole ? 1 : 0;
        }
    }
    return m;
}

void LogMatch(const char* phase, const char* where, const TileMatch& m, const char* pattern) {
    if (m.first < 0) {
        Log("[tiletest] %s %s: 65536 of 65536 bytes match pattern %s", phase, where, pattern);
        return;
    }
    const uint32_t t = static_cast<uint32_t>(m.first) / 4;
    Log("[tiletest] %s %s: %u of 65536 bytes match pattern %s -- first difference at byte %d "
        "(texel %u,%u channel %u): wanted 0x%02X, read 0x%02X; %u bytes read 0, %u of 16384 "
        "texels are A's own from another texel",
        phase, where, m.same, pattern, m.first, t % kProbeTileTexels, t / kProbeTileTexels,
        static_cast<uint32_t>(m.first) % 4, m.want, m.got, m.zero, m.moved);
}

// The self-test harness: owns the reserved resources, the heap, the compute pipelines, and the
// residency bookkeeping the CPU verifies against.
class TileSelfTest {
public:
    bool Run(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir);
    // HIERARCHY step 0, run after the M0 verdict: see the banner above its definition.
    bool RunHierarchyProbes(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir);

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

    // ---- HIERARCHY step 0
    bool CreateProbeArray(Gpu& gpu);
    void ProbeState(ID3D12GraphicsCommandList* cl, D3D12_RESOURCE_STATES to);
    void MapProbeTiles(Gpu& gpu, const D3D12_TILED_RESOURCE_COORDINATE* at,
                       const uint32_t* heapTile, uint32_t n);
    void FillProbeTiles(Gpu& gpu, const D3D12_TILED_RESOURCE_COORDINATE* at,
                        const uint32_t* block, uint32_t n);
    std::vector<uint8_t> ReadProbeTiles(Gpu& gpu, const D3D12_TILED_RESOURCE_COORDINATE* at,
                                        uint32_t n);
    bool ProbeShare(Gpu& gpu);
    bool ProbeWrap(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir);

    Com<ID3D12Resource> m_probe;     // the probe array (kProbeDim above)
    Com<ID3D12Heap> m_probeHeap;     // one heap tile per ProbeBlock
    GpuBuffer m_probeSrc;            // upload: one 64 KB block per ProbeBlock
    D3D12_RESOURCE_STATES m_probeState = D3D12_RESOURCE_STATE_COPY_DEST;
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

// ================================================================ HIERARCHY step 0: the probes
// docs/HIERARCHY.md builds its windows on facts Microsoft documents and this engine had never
// measured. The address bits are asked at boot (Gpu::Init); the other two are probed here, each
// with a planted failure it must be seen to catch:
//   share  one heap tile mapped at three places of ONE reserved array -- mip 0 of two slices and
//          mip 1 of a third -- filled through one place, read back through all three;
//   wrap   a WRAP tap across one slice's edge, in the pixel stage, plain and under Sample's
//          min-LOD clamp.
// The suite fails only when an instrument is broken: a resource refused, a call failing, a
// planted failure not caught, a copy that does not come back out through its own place. A fact
// that does not hold is a FINDING, said loudly and not failed: for sharing the design has its
// fallback (section 4.6: a window carries three mips and mixes the straddling pair by hand).
bool TileSelfTest::RunHierarchyProbes(Gpu& gpu, ShaderCompiler& sc,
                                      const std::wstring& shaderDir) {
    Log("[tiletest] ---- HIERARCHY step 0: one heap tile at three places of one reserved array, "
        "WRAP across a slice's edge (the address bits are the [gpu] boot line) ----");
    if (gpu.TiledTier() < D3D12_TILED_RESOURCES_TIER_2) {
        Log("[tiletest] step0: FAIL -- below tier 2 a NULL place reads undefined bytes, so no "
            "probe here can be judged");
        return false;
    }
    if (!CreateProbeArray(gpu)) return false;
    bool sound = ProbeShare(gpu);
    sound &= ProbeWrap(gpu, sc, shaderDir);
    // UpdateTileMappings and CopyTiles return nothing: a call the device could not take shows
    // up as its removal, and every number above would then be the dead device's.
    const HRESULT removed = gpu.Device()->GetDeviceRemovedReason();
    if (FAILED(removed)) {
        Log("[tiletest] step0: FAIL -- the device was removed during the probes: %s",
            HrString(removed).c_str());
        sound = false;
    }
    Log("[tiletest] ---- step 0 %s ----",
        sound ? "PASS: every instrument caught its planted failure; the facts are the HOLDS / "
                "DOES NOT HOLD lines"
              : "FAIL: an instrument is broken, see above");
    return sound;
}

bool TileSelfTest::CreateProbeArray(Gpu& gpu) {
    // Reserved, NO flags, born a copy destination -- a page tenant's array (Residency.cpp) at toy
    // size: the probes ask about those arrays, and a UAV or render-target flag could change the
    // layout the answer depends on.
    D3D12_RESOURCE_DESC rd{};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    rd.Width = kProbeDim;
    rd.Height = kProbeDim;
    rd.DepthOrArraySize = kProbeSlices;
    rd.MipLevels = kProbeMips;
    rd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    rd.SampleDesc.Count = 1;
    rd.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
    HRESULT hr = gpu.Device()->CreateReservedResource(&rd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                      IID_PPV_ARGS(&m_probe));
    if (FAILED(hr)) {
        Log("[tiletest] step0: FAIL -- the probe array was refused: %s", HrString(hr).c_str());
        return false;
    }
    m_probe->SetName(L"tiletest.step0 array");
    m_probeState = D3D12_RESOURCE_STATE_COPY_DEST;

    UINT numTiles = 0, numSub = kProbeSlices * kProbeMips;
    D3D12_PACKED_MIP_INFO packed{};
    D3D12_TILE_SHAPE shape{};
    D3D12_SUBRESOURCE_TILING sub[kProbeSlices * kProbeMips]{};
    gpu.Device()->GetResourceTiling(m_probe.Get(), &numTiles, &packed, &shape, &numSub, 0, sub);
    Log("[tiletest] step0 array: reserved %ux%u x%u slices x%u mips RGBA8: %u tiles of %ux%u, "
        "%u standard + %u packed mips, mip 0 %ux%u tiles, mip 1 %ux%u",
        kProbeDim, kProbeDim, kProbeSlices, kProbeMips, numTiles, shape.WidthInTexels,
        shape.HeightInTexels, packed.NumStandardMips, packed.NumPackedMips, sub[0].WidthInTiles,
        sub[0].HeightInTiles, sub[1].WidthInTiles, sub[1].HeightInTiles);
    // Every address below assumes exactly this tiling, and CopyTiles cannot reach a packed mip:
    // on any other the probes would measure a shape they do not describe, so they refuse.
    if (packed.NumPackedMips != 0 || shape.WidthInTexels != kProbeTileTexels ||
        shape.HeightInTexels != kProbeTileTexels ||
        sub[0].WidthInTiles != kProbeDim / kProbeTileTexels ||
        sub[1].WidthInTiles != kProbeDim / 2 / kProbeTileTexels) {
        Log("[tiletest] step0: FAIL -- not the tiling the probes are written for; refusing to "
            "run them");
        return false;
    }

    D3D12_HEAP_DESC hd{};
    hd.SizeInBytes = static_cast<uint64_t>(kProbeBlocks) * kProbeTileBytes;
    hd.Properties = DefaultHeapProps();
    hd.Flags = D3D12_HEAP_FLAG_ALLOW_ONLY_NON_RT_DS_TEXTURES;
    hr = gpu.Device()->CreateHeap(&hd, IID_PPV_ARGS(&m_probeHeap));
    if (FAILED(hr)) {
        Log("[tiletest] step0: FAIL -- the probe heap was refused: %s", HrString(hr).c_str());
        return false;
    }
    m_probeHeap->SetName(L"tiletest.step0 heap");

    // The blocks. Probe C's edge tiles are solid: R is the side (left 0, right 255) and B its
    // complement, G names the mip (64 at mip 0, 192 at mip 1).
    m_probeSrc = gpu.CreateUploadBuffer(static_cast<uint64_t>(kProbeBlocks) * kProbeTileBytes,
                                        L"tiletest.step0 patterns");
    const uint8_t solid[4][4] = {
        {0, 64, 255, 255}, {255, 64, 0, 255}, {0, 192, 255, 255}, {255, 192, 0, 255}};
    uint8_t* dst = m_probeSrc.cpu;
    for (uint32_t i = 0; i < kProbeTileBytes; ++i) {
        dst[size_t(kBlockA) * kProbeTileBytes + i] = PatternA(i);
        dst[size_t(kBlockB) * kProbeTileBytes + i] = PatternB(i);
        for (uint32_t k = 0; k < 4; ++k) {
            dst[size_t(kBlockL0 + k) * kProbeTileBytes + i] = solid[k][i & 3u];
        }
    }
    return true;
}

void TileSelfTest::ProbeState(ID3D12GraphicsCommandList* cl, D3D12_RESOURCE_STATES to) {
    if (m_probeState == to) return;   // before == after fails at Close(), as RunRead says
    Barrier(cl, m_probe.Get(), m_probeState, to);
    m_probeState = to;
}

// n single tiles of the probe array, each onto its heap tile (kNullTile: onto NULL), in ONE
// UpdateTileMappings with a region and a range per tile.
void TileSelfTest::MapProbeTiles(Gpu& gpu, const D3D12_TILED_RESOURCE_COORDINATE* at,
                                 const uint32_t* heapTile, uint32_t n) {
    std::vector<D3D12_TILE_REGION_SIZE> sizes(n, D3D12_TILE_REGION_SIZE{1, FALSE, 0, 0, 0});
    std::vector<D3D12_TILE_RANGE_FLAGS> flags(n);
    std::vector<UINT> starts(n), counts(n, 1);
    for (uint32_t k = 0; k < n; ++k) {
        const bool null = heapTile[k] == kNullTile;
        flags[k] = null ? D3D12_TILE_RANGE_FLAG_NULL : D3D12_TILE_RANGE_FLAG_NONE;
        starts[k] = null ? 0 : heapTile[k];
    }
    gpu.Queue()->UpdateTileMappings(m_probe.Get(), n, at, sizes.data(), m_probeHeap.Get(), n,
                                    flags.data(), starts.data(), counts.data(),
                                    D3D12_TILE_MAPPING_FLAG_NONE);
}

// Upload blocks into places, linear to swizzled: one submission, waited on.
void TileSelfTest::FillProbeTiles(Gpu& gpu, const D3D12_TILED_RESOURCE_COORDINATE* at,
                                  const uint32_t* block, uint32_t n) {
    auto* cl = gpu.BeginUpload();
    {
        PixScope scope(cl, "tiletest.step0 fill (CopyTiles, linear -> swizzled)");
        ProbeState(cl, D3D12_RESOURCE_STATE_COPY_DEST);
        const D3D12_TILE_REGION_SIZE one{1, FALSE, 0, 0, 0};
        for (uint32_t k = 0; k < n; ++k) {
            cl->CopyTiles(m_probe.Get(), &at[k], &one, m_probeSrc.res.Get(),
                          static_cast<uint64_t>(block[k]) * kProbeTileBytes,
                          D3D12_TILE_COPY_FLAG_LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE);
        }
    }
    gpu.EndUpload();
}

// Places out, swizzled to linear, 64 KB each in the order given, into a readback buffer made
// for this pass alone. Whether a copy that silently did not happen would show is the planted
// pass's question, not this function's.
std::vector<uint8_t> TileSelfTest::ReadProbeTiles(Gpu& gpu,
                                                  const D3D12_TILED_RESOURCE_COORDINATE* at,
                                                  uint32_t n) {
    const uint64_t bytes = static_cast<uint64_t>(n) * kProbeTileBytes;
    D3D12_HEAP_PROPERTIES hp{};
    hp.Type = D3D12_HEAP_TYPE_READBACK;
    D3D12_RESOURCE_DESC bd{};
    bd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    bd.Width = bytes;
    bd.Height = 1;
    bd.DepthOrArraySize = 1;
    bd.MipLevels = 1;
    bd.SampleDesc.Count = 1;
    bd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    Com<ID3D12Resource> rb;
    GA_CHECK(gpu.Device()->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &bd,
                                                  D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                  IID_PPV_ARGS(&rb)));
    auto* cl = gpu.BeginUpload();
    {
        PixScope scope(cl, "tiletest.step0 read (CopyTiles, swizzled -> linear)");
        ProbeState(cl, D3D12_RESOURCE_STATE_COPY_SOURCE);
        const D3D12_TILE_REGION_SIZE one{1, FALSE, 0, 0, 0};
        for (uint32_t k = 0; k < n; ++k) {
            cl->CopyTiles(m_probe.Get(), &at[k], &one, rb.Get(),
                          static_cast<uint64_t>(k) * kProbeTileBytes,
                          D3D12_TILE_COPY_FLAG_SWIZZLED_TILED_RESOURCE_TO_LINEAR_BUFFER);
        }
    }
    gpu.EndUpload();
    std::vector<uint8_t> out(static_cast<size_t>(bytes));
    void* p = nullptr;
    const D3D12_RANGE all{0, static_cast<SIZE_T>(bytes)};
    GA_CHECK(rb->Map(0, &all, &p));
    memcpy(out.data(), p, out.size());
    const D3D12_RANGE none{0, 0};
    rb->Unmap(0, &none);
    return out;
}

bool TileSelfTest::ProbeShare(Gpu& gpu) {
    // Mip 0 of two slices is one surface shape: two windows of one rank overlapping. Mip 1 of a
    // third is another mip's dimensions: the rung a window shares with the rank above (section
    // 4.6), which Microsoft's duplicate-mapping page does not settle. Each is reported apart.
    const D3D12_TILED_RESOURCE_COORDINATE at[3] = {
        {0, 0, 0, ProbeSub(0, 0)}, {1, 1, 0, ProbeSub(1, 0)}, {0, 0, 0, ProbeSub(2, 1)}};
    const char* const where[3] = {"slice 0 mip 0 tile (0,0)", "slice 1 mip 0 tile (1,1)",
                                  "slice 2 mip 1 tile (0,0)"};
    TileMatch share[3], planted[3], unmap[3];
    auto read = [&](TileMatch (&m)[3]) {
        std::vector<uint8_t> r = ReadProbeTiles(gpu, at, 3);
        for (uint32_t k = 0; k < 3; ++k) {
            m[k] = MatchTile(r.data() + size_t(k) * kProbeTileBytes, PatternA);
        }
        return r;
    };

    // 1. SHARE. Three ranges naming one heap offset, not D3D12_TILE_RANGE_FLAG_REUSE_SINGLE_TILE:
    // a range per tile, each naming its heap tile, is the batch TileAtlas2D::CommitMappings emits
    // and the one a manager of global tiles would, so it is the path the windows will take.
    // Filled through the first place ONLY: a copy into two mappings of one tile is undefined
    // unless the bytes agree.
    const uint32_t shared[3] = {kBlockA, kBlockA, kBlockA};
    MapProbeTiles(gpu, at, shared, 3);
    FillProbeTiles(gpu, at, shared, 1);
    Log("[tiletest] share: heap tile %u mapped at %s, %s and %s (three ranges naming it, one "
        "UpdateTileMappings); pattern A copied in through the first place only",
        kBlockA, where[0], where[1], where[2]);
    read(share);
    for (uint32_t k = 0; k < 3; ++k) LogMatch("share  ", where[k], share[k], "A");
    if (!share[0].Holds()) {
        Log("[tiletest] share: FAIL -- pattern A did not come back out through the place it went "
            "in by; the copy path is broken and nothing past it can be judged");
        return false;
    }

    // 2. PLANTED. The second place onto another heap tile holding pattern B. The comparison must
    // fail THERE, that place must read its own B (one tile through its only mapping), and the
    // other two must not move.
    const uint32_t other = kBlockB;
    MapProbeTiles(gpu, &at[1], &other, 1);
    FillProbeTiles(gpu, &at[1], &other, 1);
    Log("[tiletest] planted: %s moved to heap tile %u, which holds pattern B (A with every bit "
        "flipped)",
        where[1], kBlockB);
    const std::vector<uint8_t> r2 = read(planted);
    for (uint32_t k = 0; k < 3; ++k) LogMatch("planted", where[k], planted[k], "A");
    const TileMatch ownB = MatchTile(r2.data() + kProbeTileBytes, PatternB);
    const bool caught = !planted[1].Holds() && ownB.Holds();
    const bool others = planted[0].Holds() && planted[2].Holds() == share[2].Holds();
    Log("[tiletest] planted: %s reads %u of 65536 bytes of its own pattern B -- the wrong mapping "
        "is %s, and the other two places are %s",
        where[1], ownB.same, caught ? "CAUGHT" : "NOT CAUGHT", others ? "unchanged" : "CHANGED");
    if (!caught || !others) {
        Log("[tiletest] share: FAIL -- the instrument cannot pin a wrong mapping to its place");
        return false;
    }

    // 3. UNMAP. The second place back on the shared tile, the first onto NULL: the other two
    // must keep reading A, and the first must read zero (tier 2).
    const D3D12_TILED_RESOURCE_COORDINATE back[2] = {at[1], at[0]};
    const uint32_t backTo[2] = {kBlockA, kNullTile};
    MapProbeTiles(gpu, back, backTo, 2);
    Log("[tiletest] unmap: %s back on heap tile %u, %s mapped to NULL", where[1], kBlockA,
        where[0]);
    read(unmap);
    Log("[tiletest] unmap   %s: %u of 65536 bytes read 0 (tier 2: a NULL place reads zero)",
        where[0], unmap[0].zero);
    for (uint32_t k = 1; k < 3; ++k) LogMatch("unmap  ", where[k], unmap[k], "A");

    // The verdict: a place shares where it read A as mapped AND after the first place left.
    std::string no;
    for (uint32_t k = 1; k < 3; ++k) {
        if (share[k].Holds() && unmap[k].Holds()) continue;
        const TileMatch& m = share[k].Holds() ? unmap[k] : share[k];
        const uint32_t t = static_cast<uint32_t>(m.first) / 4;
        char b[256];
        snprintf(b, sizeof b,
                 "%s%s (%s): first differing byte %d, texel %u,%u channel %u, wanted 0x%02X, "
                 "read 0x%02X",
                 no.empty() ? "" : "; ", where[k],
                 share[k].Holds() ? "after the unmap" : "as mapped", m.first,
                 t % kProbeTileTexels, t / kProbeTileTexels,
                 static_cast<uint32_t>(m.first) % 4, m.want, m.got);
        no += b;
    }
    if (no.empty()) {
        Log("[tiletest] share: HOLDS on this adapter -- one heap tile reads back as the same 65536 "
            "bytes through mip 0 of two slices and mip 1 of a third, and the two left keep "
            "reading it when the first place is unmapped");
    } else {
        Log("[tiletest] share: DOES NOT HOLD on this adapter -- %s", no.c_str());
    }
    if (unmap[0].zero != kProbeTileBytes) {
        Log("[tiletest] unmap: the NULL place did NOT read zero (%u of 65536 bytes 0), which "
            "tier 2 promises",
            unmap[0].zero);
    }
    return true;
}

bool TileSelfTest::ProbeWrap(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    // Slice 3's two edge tiles at mip 0 and at mip 1, solid, each on its own heap tile: every tap
    // of every row lands in one of them (TileWrap.hlsl's row geometry).
    constexpr uint32_t kSlice = 3, kW = 16, kRows = 6;
    const D3D12_TILED_RESOURCE_COORDINATE edge[4] = {
        {0, 0, 0, ProbeSub(kSlice, 0)},
        {kProbeDim / kProbeTileTexels - 1, 0, 0, ProbeSub(kSlice, 0)},
        {0, 0, 0, ProbeSub(kSlice, 1)},
        {kProbeDim / 2 / kProbeTileTexels - 1, 0, 0, ProbeSub(kSlice, 1)}};
    const uint32_t solid[4] = {kBlockL0, kBlockR0, kBlockL1, kBlockR1};
    MapProbeTiles(gpu, edge, solid, 4);
    FillProbeTiles(gpu, edge, solid, 4);

    // Four root constants, the heap as Texture2DArray[] (space 5, as the shared layout has it),
    // and the two samplers a row chooses between. A float target keeps the filter's own numbers.
    hal::RootLayout rl;
    rl.Constants(0, 4)
        .Table({hal::SrvRange(0, hal::kUnbounded, 5)})
        .Sampler(hal::StaticSampler(0, D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                    D3D12_TEXTURE_ADDRESS_MODE_WRAP))
        .Sampler(hal::StaticSampler(1, D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                    D3D12_TEXTURE_ADDRESS_MODE_CLAMP));
    const Com<ID3D12RootSignature> rs = rl.Build(gpu, "tiletest.wrap");
    hal::GraphicsPipelineDesc pd;
    pd.rootSig = rs.Get();
    pd.vs = sc.Compile(shaderDir + L"/TileWrap.hlsl", L"VsWrap", L"vs_6_0");
    pd.ps = sc.Compile(shaderDir + L"/TileWrap.hlsl", L"PsWrap", L"ps_6_0");
    pd.rtvFormat = DXGI_FORMAT_R32G32B32A32_FLOAT;
    pd.dsvFormat = DXGI_FORMAT_UNKNOWN;
    const Com<ID3D12PipelineState> pso = hal::BuildGraphics(gpu, pd, "tiletest.wrap");
    if (!pso) {
        Log("[tiletest] wrap: FAIL -- the probe's pipeline did not build (shaders/TileWrap.hlsl)");
        return false;
    }

    // Row r draws mode r / 2 through the WRAP sampler (r even) or the CLAMP one, into a target
    // cleared to -1: a pixel no draw reached can never pass for a mix.
    D3D12_CLEAR_VALUE cv{};
    cv.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
    for (float& c : cv.Color) c = -1.0f;
    GpuTexture rt = gpu.CreateTexture2D(kW, kRows, cv.Format,
                                        D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET,
                                        D3D12_RESOURCE_STATE_RENDER_TARGET, L"tiletest.wrapRows",
                                        &cv);
    const D3D12_CPU_DESCRIPTOR_HANDLE rtv = gpu.RtvHeap().Cpu(gpu.RtvHeap().Alloc());
    gpu.Device()->CreateRenderTargetView(rt.res.Get(), nullptr, rtv);
    const uint32_t srv =
        gpu.CreateSrvArray(m_probe.Get(), DXGI_FORMAT_R8G8B8A8_UNORM, kProbeMips, kProbeSlices);
    auto* cl = gpu.BeginUpload();
    {
        PixScope scope(cl, "tiletest.step0 wrap (six rows across one slice's edge)");
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
        cl->SetDescriptorHeaps(1, heaps);
        ProbeState(cl, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        cl->ClearRenderTargetView(rtv, cv.Color, 0, nullptr);
        cl->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
        cl->SetGraphicsRootSignature(rs.Get());
        cl->SetPipelineState(pso.Get());
        cl->SetGraphicsRootDescriptorTable(1, gpu.SrvHeap().Gpu(0));
        cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        for (uint32_t r = 0; r < kRows; ++r) {
            const D3D12_VIEWPORT vp{0.0f, float(r), float(kW), 1.0f, 0.0f, 1.0f};
            const D3D12_RECT sr{0, LONG(r), LONG(kW), LONG(r + 1)};
            const uint32_t c[4] = {srv, kSlice, r / 2, r & 1u};
            cl->RSSetViewports(1, &vp);
            cl->RSSetScissorRects(1, &sr);
            cl->SetGraphicsRoot32BitConstants(0, 4, c, 0);
            cl->DrawInstanced(3, 1, 0, 0);
        }
    }
    gpu.EndUpload();
    uint32_t pitch = 0;
    const std::vector<uint8_t> px = gpu.ReadbackTexture(rt, &pitch);

    // What a bilinear tap at pixel i reads, as the weight of the RIGHT edge: its position at the
    // mip, the two texels it straddles wrapped (or clamped) onto the mip's width, and the side
    // each lands on.
    auto rightWeight = [](uint32_t i, uint32_t mip, bool wrap) {
        const double w = double(kProbeDim >> mip);
        const double p = (1022.5 + 0.25 * i) / 1024.0 * w - 0.5;
        const double t0 = std::floor(p), f = p - t0;
        auto right = [w, wrap](double t) {
            t = wrap ? t - w * std::floor(t / w) : std::clamp(t, 0.0, w - 1.0);
            return t >= 0.5 * w ? 1.0 : 0.0;
        };
        return (1.0 - f) * right(t0) + f * right(t0 + 1.0);
    };
    // D3D's floor is eight bits of sub-texel weight, so a tap lands within 1/256 of its mix; the
    // rows' weights are multiples of 1/8, and WRAP and CLAMP differ by at least 1/8 wherever they
    // differ at all. 1/128 tells one from the other with room on both sides.
    constexpr double kTol = 1.0 / 128.0;
    const char* const kMode[3] = {"SampleLevel(0)     ", "PageSample, clamp 0",
                                  "PageSample, clamp 1"};
    bool sound = true, holds = true;
    for (uint32_t r = 0; r < kRows; ++r) {
        const uint32_t mode = r / 2, mip = mode == 2 ? 1u : 0u;
        const bool clampRow = (r & 1u) != 0;
        const float* v = reinterpret_cast<const float*>(px.data() + size_t(r) * pitch);
        const double g = (mip ? 192.0 : 64.0) / 255.0;
        double errWrap = 0.0, errClamp = 0.0, gSum = 0.0;
        std::string weights;
        for (uint32_t i = 0; i < kW; ++i) {
            const float* t = v + 4 * i;
            for (int wrap = 0; wrap < 2; ++wrap) {
                const double wr = rightWeight(i, mip, wrap != 0);
                const double want[4] = {wr, g, 1.0 - wr, 1.0};
                double& err = wrap ? errWrap : errClamp;
                for (int c = 0; c < 4; ++c) err = std::max(err, std::fabs(double(t[c]) - want[c]));
            }
            gSum += t[1];
            char b[16];
            snprintf(b, sizeof b, " %.3f", t[0]);
            weights += b;
        }
        const char* reads = errWrap <= kTol    ? "WRAP's mix"
                            : errClamp <= kTol ? "CLAMP's edge"
                                               : "NEITHER";
        Log("[tiletest] wrap  %s %s: right edge%s | G %.3f (mip %u is %.3f) -> %s (off WRAP's "
            "mix by %.4f, off CLAMP's edge by %.4f)",
            clampRow ? "CLAMP" : "WRAP ", kMode[mode], weights.c_str(), gSum / kW, mip, g, reads,
            errWrap, errClamp);
        if (clampRow) {
            sound = sound && errClamp <= kTol;   // the planted failure: the unmixed edge
        } else {
            holds = holds && errWrap <= kTol;
        }
    }
    Log("[tiletest] planted: the CLAMP rows %s",
        sound ? "read the unmixed right edge and are told apart from WRAP's mix (CAUGHT)"
              : "did NOT read the unmixed edge");
    if (!sound) {
        Log("[tiletest] wrap: FAIL -- the instrument cannot tell WRAP from CLAMP");
        return false;
    }
    Log("[tiletest] wrap: %s",
        holds ? "HOLDS on this adapter -- in the pixel stage a WRAP tap past u = 1 filters with "
                "the texel at u = 0 of the same slice: SampleLevel and PageSample at mip 0, and "
                "PageSample clamped onto mip 1"
              : "DOES NOT HOLD on this adapter -- see the WRAP rows above");
    return true;
}

// ================================================================ HIERARCHY step 3: the address
// docs/HIERARCHY.md 4.4's address of a face-plane window, on this GPU: PageSample.hlsli's own
// PageTexel in the pixel stage (shaders/TileTexel.hlsl), over spacetest's helm sample at rungs
// 15 and 9 (core/Lattice.h FaceWindowHelmSample: the points the CPU gate judged). Each point's
// eye-relative float32 position is one texel of a float texture; each rung is drawn once with
// the rows FaceWindow::PlanesIn builds and once with the plant (w's cancellation in float32), and
// the float target is read back. THE GATE is the GPU's texel against the doubles -- under 0.01
// texel at both rungs, its uv too, and the plant past that at both. The GPU against the CPU twin
// (FaceWindow::PageTexel) is a RECORD, never the gate: a GPU may fuse a dot's multiply-adds or
// round a division its own way, and the bit-equal count and the worst ulps say how this one does.

// Common.h's UlpDistance, for float32: the floats between two values (0 for equal values and for
// the two zeros, UINT32_MAX when either is a NaN).
uint32_t UlpDistanceF(float a, float b) {
    if (a != a || b != b) return UINT32_MAX;
    int32_t ia = 0, ib = 0;
    memcpy(&ia, &a, sizeof ia);
    memcpy(&ib, &b, sizeof ib);
    const int64_t oa = ia < 0 ? int64_t(INT32_MIN) - ia : ia;
    const int64_t ob = ib < 0 ? int64_t(INT32_MIN) - ib : ib;
    return static_cast<uint32_t>(oa >= ob ? oa - ob : ob - oa);
}

// A max that keeps a NaN: a texel that came back NaN must fail the bound, not vanish in a max.
void Worse(double& worst, double e) {
    if (!(e <= worst)) worst = e;
}

bool ProbeAddress(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    Log("[tiletest] ---- HIERARCHY step 3: the address of a face-plane window, PageTexel in the "
        "pixel stage against the doubles (the gate) and against its CPU twin (a record) ----");
    constexpr uint32_t kW = 64;           // points a row of the texture
    constexpr double kBound = 0.01;       // texels of the rung: spacetest's bound
    constexpr float kUnreached = 1e30f;   // the target's clear, which no window's texel can be
    const double dim = double(Lattice::kFaceDim);
    const int kRung[2] = {15, 9};
    const FaceWindowSample s[2] = {FaceWindowHelmSample(kRung[0]), FaceWindowHelmSample(kRung[1])};
    uint32_t count[2] = {}, row0[2] = {}, rows[2] = {}, H = 0;
    for (int i = 0; i < 2; ++i) {
        count[i] = static_cast<uint32_t>(s[i].p.size() / 3);
        row0[i] = H;
        rows[i] = (count[i] + kW - 1) / kW;
        H += rows[i];
    }
    if (count[0] == 0 || count[1] == 0) {
        Log("[tiletest] address: FAIL -- spacetest's helm sample came back empty");
        return false;
    }

    // The points, block after block, one RGBA32F texel each (w = 1); the slots past a block's
    // last point stay 0 and are never judged.
    std::vector<float> points(size_t(kW) * H * 4, 0.0f);
    for (int i = 0; i < 2; ++i) {
        for (uint32_t k = 0; k < count[i]; ++k) {
            float* t = &points[(size_t(row0[i] + k / kW) * kW + k % kW) * 4];
            t[0] = s[i].p[3 * k];
            t[1] = s[i].p[3 * k + 1];
            t[2] = s[i].p[3 * k + 2];
            t[3] = 1.0f;
        }
    }
    // Born a copy destination: UploadTexture's barrier starts from COPY_DEST (review finding 43).
    GpuTexture pts = gpu.CreateTexture2D(kW, H, DXGI_FORMAT_R32G32B32A32_FLOAT,
                                         D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_COPY_DEST,
                                         L"tiletest.addressPoints");
    gpu.UploadTexture(pts, points.data(), kW * 16);
    const uint32_t srv = gpu.CreateSrv(pts.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);

    // Fourteen root constants (TileTexel.hlsl's TexelCb) and the heap as Texture2D[] (space 1).
    hal::RootLayout rl;
    rl.Constants(0, 14).Table({hal::SrvRange(0, hal::kUnbounded, 1)});
    const Com<ID3D12RootSignature> rs = rl.Build(gpu, "tiletest.address");
    hal::GraphicsPipelineDesc pd;
    pd.rootSig = rs.Get();
    pd.vs = sc.Compile(shaderDir + L"/TileTexel.hlsl", L"VsTexel", L"vs_6_0");
    pd.ps = sc.Compile(shaderDir + L"/TileTexel.hlsl", L"PsTexel", L"ps_6_0");
    pd.rtvFormat = DXGI_FORMAT_R32G32B32A32_FLOAT;
    pd.dsvFormat = DXGI_FORMAT_UNKNOWN;
    const Com<ID3D12PipelineState> pso = hal::BuildGraphics(gpu, pd, "tiletest.address");
    if (!pso) {
        Log("[tiletest] address: FAIL -- the probe's pipeline did not build "
            "(shaders/TileTexel.hlsl)");
        return false;
    }

    // The target: the rows' texels in its first H rows, the plant's in the next H, all cleared to
    // kUnreached so that a pixel no draw reached fails the gate on its own.
    D3D12_CLEAR_VALUE cv{};
    cv.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
    for (float& c : cv.Color) c = kUnreached;
    GpuTexture rt = gpu.CreateTexture2D(kW, 2 * H, cv.Format,
                                        D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET,
                                        D3D12_RESOURCE_STATE_RENDER_TARGET,
                                        L"tiletest.addressTexels", &cv);
    const D3D12_CPU_DESCRIPTOR_HANDLE rtv = gpu.RtvHeap().Cpu(gpu.RtvHeap().Alloc());
    gpu.Device()->CreateRenderTargetView(rt.res.Get(), nullptr, rtv);
    auto* cl = gpu.BeginUpload();
    {
        PixScope scope(cl, "tiletest.step3 address (PageTexel over spacetest's helm sample)");
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
        cl->SetDescriptorHeaps(1, heaps);
        cl->ClearRenderTargetView(rtv, cv.Color, 0, nullptr);
        cl->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
        cl->SetGraphicsRootSignature(rs.Get());
        cl->SetPipelineState(pso.Get());
        cl->SetGraphicsRootDescriptorTable(1, gpu.SrvHeap().Gpu(0));
        cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        for (uint32_t plant = 0; plant < 2; ++plant) {
            for (int i = 0; i < 2; ++i) {
                const FaceWindow::Planes& pl = plant ? s[i].planted : s[i].planes;
                const uint32_t top = row0[i] + plant * H;
                const D3D12_VIEWPORT vp{0.0f, float(top), float(kW), float(rows[i]), 0.0f, 1.0f};
                const D3D12_RECT sr{0, LONG(top), LONG(kW), LONG(top + rows[i])};
                uint32_t c[14];
                memcpy(c, pl.u, sizeof pl.u);
                memcpy(c + 4, pl.v, sizeof pl.v);
                memcpy(c + 8, pl.w, sizeof pl.w);
                c[12] = srv;
                c[13] = plant * H;
                cl->RSSetViewports(1, &vp);
                cl->RSSetScissorRects(1, &sr);
                cl->SetGraphicsRoot32BitConstants(0, 14, c, 0);
                cl->DrawInstanced(3, 1, 0, 0);
            }
        }
    }
    gpu.EndUpload();
    uint32_t pitch = 0;
    const std::vector<uint8_t> px = gpu.ReadbackTexture(rt, &pitch);
    auto texel = [&](int i, uint32_t k, uint32_t plant) {
        const size_t row = row0[i] + k / kW + plant * H;
        return reinterpret_cast<const float*>(px.data() + row * pitch) + 4 * (k % kW);
    };

    bool within = true, caught = true;
    uint32_t unreached = 0;
    double planted[2] = {};
    for (int i = 0; i < 2; ++i) {
        double gate = 0.0, gateUv = 0.0, offTwin = 0.0;
        uint32_t equal = 0, ulps = 0;
        for (uint32_t k = 0; k < count[i]; ++k) {
            const float* g = texel(i, k, 0);
            const float* q = texel(i, k, 1);
            unreached += (g[0] == kUnreached ? 1u : 0u) + (q[0] == kUnreached ? 1u : 0u);
            float twin[2] = {};
            FaceWindow::PageTexel(&s[i].p[3 * k], s[i].planes, twin[0], twin[1]);
            for (int c = 0; c < 2; ++c) {
                const double ref = s[i].ref[2 * k + c];
                Worse(gate, std::fabs(g[c] - ref));
                Worse(gateUv, std::fabs(double(g[2 + c]) * dim - ref));
                Worse(planted[i], std::fabs(q[c] - ref));
                const uint32_t u = UlpDistanceF(g[c], twin[c]);
                equal += u == 0 ? 1u : 0u;
                ulps = (std::max)(ulps, u);
                Worse(offTwin, std::fabs(double(g[c]) - double(twin[c])));
            }
        }
        within = within && gate < kBound && gateUv < kBound;
        caught = caught && planted[i] > kBound;
        Log("[tiletest] address rung %2d: %u points -- the GPU's texel against the doubles worst "
            "%.6f (its uv x 16384 %.6f); against the CPU twin, a record, %u of %u coordinates "
            "bit-equal, worst %u ulps (|d| %.3g texel)",
            kRung[i], count[i], gate, gateUv, equal, 2 * count[i], ulps, offTwin);
    }
    Log("[tiletest] address planted (w's cancellation in float32): the GPU's texel against the "
        "doubles worst %.3g at rung %d and %.3g at rung %d -- %s",
        planted[0], kRung[0], planted[1], kRung[1],
        caught ? "CAUGHT: past 0.01 at both" : "NOT CAUGHT: the instrument is blind");
    if (unreached != 0) {
        Log("[tiletest] address: FAIL -- %u pixels still hold the clear: no draw reached them",
            unreached);
    }
    const HRESULT removed = gpu.Device()->GetDeviceRemovedReason();
    if (FAILED(removed)) {
        Log("[tiletest] address: FAIL -- the device was removed during the draw: %s",
            HrString(removed).c_str());
    }
    const bool pass = within && caught && unreached == 0 && SUCCEEDED(removed);
    Log("[tiletest] ---- step 3 %s ----",
        pass ? "PASS: PageTexel on this GPU is within 0.01 texel of the doubles at rungs 15 and "
               "9, its uv too, and the plant is caught"
             : "FAIL: see above");
    return pass;
}

// ================================================================ HIERARCHY 4.17 commit 3: blocks
// The standing blocks' address at the pixel stage WITH THE KEY'S OWN ROWS: the Merrimack key's
// blocks at rungs 6 and 9 (SurfaceFrame::DeclareBlocks), their rows by SurfaceFrame::BlockRows --
// the function the frame's Fill fills them with -- about the eye's own tangent frame at the helm
// (east, up, north at the eye: the address block's tangent case), over spacetest's helm sample
// carried into that frame. THE GATE is the GPU's texel, taken back to the block's own origin by
// the rows' whole blocks, against the doubles: under 0.01 texel at both rungs, and its uv with
// the whole blocks added in float32 as CsBlockUv adds them. THE PLANT is commit 2's form, the
// planes through the planet's centre and the point a float32 direction: past the bound at both.
bool ProbeBlockAddress(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    Log("[tiletest] ---- HIERARCHY 4.17 commit 3: the standing blocks' address with the key's own "
        "rows (SurfaceFrame::BlockRows), PageTexel in the pixel stage against the doubles ----");
    constexpr uint32_t kW = 64;
    constexpr double kBound = 0.01;
    constexpr float kUnreached = 1e30f;
    const double dim = double(Lattice::kFaceDim);
    SurfaceFrame key;
    // Rank 1 comes first since commit 4 (the directory's law): blocks 0, 1, 2 = rungs 3, 6, 9.
    if (!key.DeclareBlocks("-70.8125,42.816,3;-70.8125,42.816,6;-70.8125,42.816,9") ||
        key.blocks.size() != 3) {
        Log("[tiletest] block address: FAIL -- the Merrimack key declared no blocks");
        return false;
    }
    // The helm eye (spacetest's: 3 m over the mouth on the 6371 km sphere) and its tangent frame.
    const double kDeg = 3.141592653589793 / 180.0, lat = 42.816 * kDeg, lon = -70.8125 * kDeg;
    const double eyeR = 6371000.0 + 3.0;
    const double eye[3] = {std::cos(lat) * std::cos(lon) * eyeR, std::sin(lat) * eyeR,
                           std::cos(lat) * std::sin(lon) * eyeR};
    const double el = std::sqrt(eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2]);
    const double up[3] = {eye[0] / el, eye[1] / el, eye[2] / el};
    const double yl = std::sqrt(up[0] * up[0] + up[2] * up[2]);
    const double east[3] = {-up[2] / yl, 0.0, up[0] / yl};
    const double north[3] = {east[1] * up[2] - east[2] * up[1], east[2] * up[0] - east[0] * up[2],
                             east[0] * up[1] - east[1] * up[0]};
    const Placement own = Placement::Frame(east, up, north, eye);
    struct Job {
        FaceWindow::Planes pl{};
        float off[2] = {0.0f, 0.0f};
        std::vector<float> p;     // x y z per point
        std::vector<double> ref;  // the doubles: texel relative to the block's own origin, x y
        uint32_t n = 0, row0 = 0, rows = 0;
    };
    const int kRung[2] = {9, 6};
    Job job[4];   // the key's rows at rungs 9 and 6, then the plant at rungs 9 and 6
    for (int i = 0; i < 2; ++i) {
        const FaceWindowSample s = FaceWindowHelmSample(kRung[i]);
        const FaceWindow& b = key.blocks[kRung[i] == 6 ? 1 : 2];
        Job& r = job[i];
        Job& q = job[2 + i];
        SurfaceFrame::BlockRows(b, own, eye, r.pl, r.off);
        q.pl = b.PlanesIn(Placement{});
        const size_t n = s.p.size() / 3;
        for (size_t k = 0; k < n; ++k) {
            const double pe[3] = {s.p[3 * k], s.p[3 * k + 1], s.p[3 * k + 2]};
            for (int c = 0; c < 2; ++c) {
                const double ref = s.ref[2 * k + c] +
                                   double(c ? s.win.anchorY - b.anchorY : s.win.anchorX - b.anchorX);
                r.ref.push_back(ref);
                q.ref.push_back(ref);
            }
            for (const double* ax : {east, up, north}) {   // the point in the eye's tangent frame
                r.p.push_back(static_cast<float>(pe[0] * ax[0] + pe[1] * ax[1] + pe[2] * ax[2]));
            }
            const double P[3] = {eye[0] + pe[0], eye[1] + pe[1], eye[2] + pe[2]};
            const double pl = std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]);
            for (int c = 0; c < 3; ++c) q.p.push_back(static_cast<float>(P[c] / pl));
        }
        r.n = q.n = static_cast<uint32_t>(n);
    }
    uint32_t H = 0;
    for (Job& j : job) {
        j.row0 = H;
        j.rows = (j.n + kW - 1) / kW;
        H += j.rows;
    }
    if (job[0].n == 0 || job[1].n == 0) {
        Log("[tiletest] block address: FAIL -- spacetest's helm sample came back empty");
        return false;
    }
    std::vector<float> points(size_t(kW) * H * 4, 0.0f);
    for (const Job& j : job) {
        for (uint32_t k = 0; k < j.n; ++k) {
            float* t = &points[(size_t(j.row0 + k / kW) * kW + k % kW) * 4];
            t[0] = j.p[3 * k];
            t[1] = j.p[3 * k + 1];
            t[2] = j.p[3 * k + 2];
            t[3] = 1.0f;
        }
    }
    GpuTexture pts = gpu.CreateTexture2D(kW, H, DXGI_FORMAT_R32G32B32A32_FLOAT,
                                         D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_COPY_DEST,
                                         L"tiletest.blockPoints");
    gpu.UploadTexture(pts, points.data(), kW * 16);
    const uint32_t srv = gpu.CreateSrv(pts.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
    hal::RootLayout rl;
    rl.Constants(0, 14).Table({hal::SrvRange(0, hal::kUnbounded, 1)});
    const Com<ID3D12RootSignature> rs = rl.Build(gpu, "tiletest.blockAddress");
    hal::GraphicsPipelineDesc pd;
    pd.rootSig = rs.Get();
    pd.vs = sc.Compile(shaderDir + L"/TileTexel.hlsl", L"VsTexel", L"vs_6_0");
    pd.ps = sc.Compile(shaderDir + L"/TileTexel.hlsl", L"PsTexel", L"ps_6_0");
    pd.rtvFormat = DXGI_FORMAT_R32G32B32A32_FLOAT;
    pd.dsvFormat = DXGI_FORMAT_UNKNOWN;
    const Com<ID3D12PipelineState> pso = hal::BuildGraphics(gpu, pd, "tiletest.blockAddress");
    if (!pso) {
        Log("[tiletest] block address: FAIL -- the probe's pipeline did not build");
        return false;
    }
    D3D12_CLEAR_VALUE cv{};
    cv.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
    for (float& c : cv.Color) c = kUnreached;
    GpuTexture rt = gpu.CreateTexture2D(kW, H, cv.Format, D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET,
                                        D3D12_RESOURCE_STATE_RENDER_TARGET,
                                        L"tiletest.blockTexels", &cv);
    const D3D12_CPU_DESCRIPTOR_HANDLE rtv = gpu.RtvHeap().Cpu(gpu.RtvHeap().Alloc());
    gpu.Device()->CreateRenderTargetView(rt.res.Get(), nullptr, rtv);
    auto* cl = gpu.BeginUpload();
    {
        PixScope scope(cl, "tiletest.4.17 block address (the key's rows over spacetest's helm sample)");
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
        cl->SetDescriptorHeaps(1, heaps);
        cl->ClearRenderTargetView(rtv, cv.Color, 0, nullptr);
        cl->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
        cl->SetGraphicsRootSignature(rs.Get());
        cl->SetPipelineState(pso.Get());
        cl->SetGraphicsRootDescriptorTable(1, gpu.SrvHeap().Gpu(0));
        cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        for (const Job& j : job) {
            const D3D12_VIEWPORT vp{0.0f, float(j.row0), float(kW), float(j.rows), 0.0f, 1.0f};
            const D3D12_RECT sr{0, LONG(j.row0), LONG(kW), LONG(j.row0 + j.rows)};
            uint32_t c[14];
            memcpy(c, j.pl.u, sizeof j.pl.u);
            memcpy(c + 4, j.pl.v, sizeof j.pl.v);
            memcpy(c + 8, j.pl.w, sizeof j.pl.w);
            c[12] = srv;
            c[13] = 0;
            cl->RSSetViewports(1, &vp);
            cl->RSSetScissorRects(1, &sr);
            cl->SetGraphicsRoot32BitConstants(0, 14, c, 0);
            cl->DrawInstanced(3, 1, 0, 0);
        }
    }
    gpu.EndUpload();
    uint32_t pitch = 0;
    const std::vector<uint8_t> px = gpu.ReadbackTexture(rt, &pitch);
    bool within = true, caught = true;
    uint32_t unreached = 0;
    double worst[4] = {}, worstUv[2] = {};
    for (int ji = 0; ji < 4; ++ji) {
        const Job& j = job[ji];
        for (uint32_t k = 0; k < j.n; ++k) {
            const float* g = reinterpret_cast<const float*>(px.data() + size_t(j.row0 + k / kW) * pitch) +
                             4 * (k % kW);
            unreached += g[0] == kUnreached ? 1u : 0u;
            for (int c = 0; c < 2; ++c) {
                const double ref = j.ref[2 * k + c];
                Worse(worst[ji], std::fabs(double(g[c]) + double(j.off[c]) * dim - ref));
                if (ji < 2) {
                    const float uv = g[2 + c] + j.off[c];   // CsBlockUv's float32 addition
                    Worse(worstUv[ji], std::fabs(double(uv) * dim - ref));
                }
            }
        }
    }
    for (int i = 0; i < 2; ++i) {
        within = within && worst[i] < kBound && worstUv[i] < kBound;
        caught = caught && worst[2 + i] > kBound;
        Log("[tiletest] block address rung %d, block (%lld,%lld), rows anchored %+.0f,%+.0f blocks "
            "from it: %u points -- the GPU's texel against the doubles worst %.6f (its uv %.6f); the "
            "plant, a float32 direction through the centre, worst %.4f",
            kRung[i], key.blocks[kRung[i] == 6 ? 1 : 2].anchorX / Lattice::kFaceDim,
            key.blocks[kRung[i] == 6 ? 1 : 2].anchorY / Lattice::kFaceDim, job[i].off[0],
            job[i].off[1], job[i].n, worst[i], worstUv[i], worst[2 + i]);
    }
    const HRESULT removed = gpu.Device()->GetDeviceRemovedReason();
    const bool pass = within && caught && unreached == 0 && SUCCEEDED(removed);
    Log("[tiletest] ---- block address %s ----",
        pass ? "PASS: the key's rows put the GPU within 0.01 texel of the doubles at rungs 9 and 6, "
               "its uv too, and the float32 direction is CAUGHT past it at both"
             : (unreached ? "FAIL: pixels no draw reached" : "FAIL: see above"));
    return pass;
}

// ================================================================ HIERARCHY 4.17 commit 4: the walk
// The directory walk's two bodies held equal: Walk.hlsli, run in the pixel stage through
// TileWalk.hlsl and read back, against SurfaceFrame::Walk in doubles -- the Merrimack key's six
// blocks (ranks 1 to 3) and their directory, the rows about the helm's tangent frame as the
// surface fills them (BlockRows). 10,000 ground points of face 5 in and around the blocks (4,000
// within ~20 km of the mouth, 3,000 within ~150 km, 3,000 within ~800 km) and 1,000 on each other
// face. THE GATE: every chain the same length and the same slices, every uv within 0.01 texel.
// THE PLANT: the face-5 cell under the mouth names the rung-6 block where the directory names
// rank 1's, drawn through the same shader: the chains through that cell must differ.
bool ProbeWalk(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    Log("[tiletest] ---- HIERARCHY 4.17 commit 4: the directory walk, Walk.hlsli in the pixel stage "
        "against SurfaceFrame::Walk in doubles ----");
    constexpr uint32_t kW = 64;   // points a row
    constexpr double kBound = 0.01;
    constexpr float kUnreached = 1e30f;
    const double dim = double(Lattice::kFaceDim);
    SurfaceFrame key;
    if (!key.DeclareBlocks("-70.8125,42.816,3;-70.8125,42.816,6;-70.8125,42.816,9;-70.849,42.816,9;"
                           "-70.8125,42.789,9;-70.849,42.789,9") ||
        key.blocks.size() != 6 || key.directory.empty()) {
        Log("[tiletest] walk: FAIL -- the Merrimack key declared no directory");
        return false;
    }
    const double kDeg = 3.141592653589793 / 180.0, lat = 42.816 * kDeg, lon = -70.8125 * kDeg;
    const double R = 6371000.0;
    const double eye[3] = {std::cos(lat) * std::cos(lon) * (R + 3.0), std::sin(lat) * (R + 3.0),
                           std::cos(lat) * std::sin(lon) * (R + 3.0)};
    const double el = std::sqrt(eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2]);
    const double up[3] = {eye[0] / el, eye[1] / el, eye[2] / el};
    const double yl = std::sqrt(up[0] * up[0] + up[2] * up[2]);
    const double east[3] = {-up[2] / yl, 0.0, up[0] / yl};
    const double north[3] = {east[1] * up[2] - east[2] * up[1], east[2] * up[0] - east[0] * up[2],
                             east[0] * up[1] - east[1] * up[0]};
    const Placement own = Placement::Frame(east, up, north, eye);
    std::vector<float> rows(8 * 4 * 4, 0.0f);   // RGBA32F 8 x 4
    for (size_t i = 0; i < key.blocks.size(); ++i) {
        FaceWindow::Planes pl{};
        float off[2] = {0.0f, 0.0f};
        SurfaceFrame::BlockRows(key.blocks[i], own, eye, pl, off);
        memcpy(&rows[(0 * 8 + i) * 4], pl.u, 16);
        memcpy(&rows[(1 * 8 + i) * 4], pl.v, 16);
        memcpy(&rows[(2 * 8 + i) * 4], pl.w, 16);
        rows[(3 * 8 + i / 2) * 4 + (i & 1) * 2] = off[0];
        rows[(3 * 8 + i / 2) * 4 + (i & 1) * 2 + 1] = off[1];
    }
    // The ground points, planet frame, on the sphere.
    std::vector<std::array<double, 3>> P;
    std::mt19937 rng(0x4174u);
    std::uniform_real_distribution<double> U01(0.0, 1.0);
    auto add = [&](uint32_t f, double u, double v) {
        double d[3];
        ComposeCubeDir(f, (std::min)(0.9999, (std::max)(0.0001, u)), (std::min)(0.9999, (std::max)(0.0001, v)), d);
        P.push_back({d[0] * R, d[1] * R, d[2] * R});
    };
    double m[2];
    CubeFaceOfDir(eye, m);
    const double half[3] = {0.0016, 0.012, 0.063};   // ~20, 150 and 800 km of face-5 uv
    const int many[3] = {4000, 3000, 3000};
    for (int b = 0; b < 3; ++b) {
        for (int n = 0; n < many[b]; ++n) {
            add(5, m[0] + (U01(rng) * 2 - 1) * half[b], m[1] + (U01(rng) * 2 - 1) * half[b]);
        }
    }
    for (uint32_t f = 0; f < 5; ++f) {
        for (int n = 0; n < 1000; ++n) add(f, U01(rng), U01(rng));
    }
    const uint32_t N = uint32_t(P.size()), H = (N + kW - 1) / kW;
    std::vector<float> pts(size_t(kW) * H * 4, 0.0f), dirs(size_t(kW) * H * 4, 0.0f);
    for (uint32_t k = 0; k < N; ++k) {
        const double q[3] = {P[k][0] - eye[0], P[k][1] - eye[1], P[k][2] - eye[2]};
        const double pl = std::sqrt(P[k][0] * P[k][0] + P[k][1] * P[k][1] + P[k][2] * P[k][2]);
        float* t = &pts[size_t(k) * 4];
        float* d = &dirs[size_t(k) * 4];
        int c = 0;
        for (const double* ax : {east, up, north}) t[c++] = static_cast<float>(q[0] * ax[0] + q[1] * ax[1] + q[2] * ax[2]);
        for (int j = 0; j < 3; ++j) d[j] = static_cast<float>(P[k][j] / pl);
        t[3] = d[3] = 1.0f;
    }
    // The directory twice: as built, and planted (the face-5 cell under the mouth names slice 7).
    const uint32_t dirRows = uint32_t(key.directory.size() / SurfaceFrame::kCells);
    std::vector<uint16_t> dirTex(size_t(128) * dirRows, SurfaceFrame::kNone);
    for (uint32_t r = 0; r < dirRows; ++r) {
        for (uint32_t c = 0; c < SurfaceFrame::kCells; ++c) {
            dirTex[size_t(r) * 128 + c] = key.directory[size_t(r) * SurfaceFrame::kCells + c];
        }
    }
    std::vector<uint16_t> planted = dirTex;
    const uint32_t pcx = uint32_t(m[0] * 16.0), pcy = uint32_t(m[1] * 16.0);
    planted[size_t(5 * 16 + pcy) * 128 + pcx] = 7;
    auto texture = [&](uint32_t w, uint32_t h, DXGI_FORMAT fmt, const void* data, uint32_t pitch,
                       const wchar_t* name) {
        GpuTexture t = gpu.CreateTexture2D(w, h, fmt, D3D12_RESOURCE_FLAG_NONE,
                                           D3D12_RESOURCE_STATE_COPY_DEST, name);
        gpu.UploadTexture(t, data, pitch);
        return t;
    };
    GpuTexture tPts = texture(kW, H, DXGI_FORMAT_R32G32B32A32_FLOAT, pts.data(), kW * 16, L"tiletest.walkPoints");
    GpuTexture tDirs = texture(kW, H, DXGI_FORMAT_R32G32B32A32_FLOAT, dirs.data(), kW * 16, L"tiletest.walkDirs");
    GpuTexture tRows = texture(8, 4, DXGI_FORMAT_R32G32B32A32_FLOAT, rows.data(), 8 * 16, L"tiletest.walkRows");
    GpuTexture tDir = texture(128, dirRows, DXGI_FORMAT_R16_UINT, dirTex.data(), 256, L"tiletest.walkDirectory");
    GpuTexture tBad = texture(128, dirRows, DXGI_FORMAT_R16_UINT, planted.data(), 256, L"tiletest.walkPlanted");
    const uint32_t sPts = gpu.CreateSrv(tPts.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
    const uint32_t sDirs = gpu.CreateSrv(tDirs.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
    const uint32_t sRows = gpu.CreateSrv(tRows.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
    const uint32_t sDir = gpu.CreateSrv(tDir.res.Get(), DXGI_FORMAT_R16_UINT);
    const uint32_t sBad = gpu.CreateSrv(tBad.res.Get(), DXGI_FORMAT_R16_UINT);
    hal::RootLayout rl;
    rl.Constants(0, 5)
        .Table({hal::SrvRange(0, hal::kUnbounded, 1)})
        .Table({hal::SrvRange(0, hal::kUnbounded, 2)});
    const Com<ID3D12RootSignature> rs = rl.Build(gpu, "tiletest.walk");
    hal::GraphicsPipelineDesc pd;
    pd.rootSig = rs.Get();
    pd.vs = sc.Compile(shaderDir + L"/TileWalk.hlsl", L"VsWalk", L"vs_6_0", {L"GA_BLOCK_RANKS=5"});
    pd.ps = sc.Compile(shaderDir + L"/TileWalk.hlsl", L"PsWalk", L"ps_6_0", {L"GA_BLOCK_RANKS=5"});
    pd.rtvFormat = DXGI_FORMAT_R32G32B32A32_FLOAT;
    pd.dsvFormat = DXGI_FORMAT_UNKNOWN;
    const Com<ID3D12PipelineState> pso = hal::BuildGraphics(gpu, pd, "tiletest.walk");
    if (!pso) {
        Log("[tiletest] walk: FAIL -- the probe's pipeline did not build (shaders/TileWalk.hlsl)");
        return false;
    }
    D3D12_CLEAR_VALUE cv{};
    cv.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
    for (float& c : cv.Color) c = kUnreached;
    GpuTexture rt = gpu.CreateTexture2D(5 * kW, 2 * H, cv.Format, D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET,
                                        D3D12_RESOURCE_STATE_RENDER_TARGET, L"tiletest.walkChains", &cv);
    const D3D12_CPU_DESCRIPTOR_HANDLE rtv = gpu.RtvHeap().Cpu(gpu.RtvHeap().Alloc());
    gpu.Device()->CreateRenderTargetView(rt.res.Get(), nullptr, rtv);
    auto* cl = gpu.BeginUpload();
    {
        PixScope scope(cl, "tiletest.4.17 walk (Walk.hlsli against SurfaceFrame::Walk)");
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
        cl->SetDescriptorHeaps(1, heaps);
        cl->ClearRenderTargetView(rtv, cv.Color, 0, nullptr);
        cl->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
        cl->SetGraphicsRootSignature(rs.Get());
        cl->SetPipelineState(pso.Get());
        cl->SetGraphicsRootDescriptorTable(1, gpu.SrvHeap().Gpu(0));
        cl->SetGraphicsRootDescriptorTable(2, gpu.SrvHeap().Gpu(0));
        cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        for (uint32_t plant = 0; plant < 2; ++plant) {
            const D3D12_VIEWPORT vp{0.0f, float(plant * H), float(5 * kW), float(H), 0.0f, 1.0f};
            const D3D12_RECT sr{0, LONG(plant * H), LONG(5 * kW), LONG((plant + 1) * H)};
            const uint32_t c[5] = {sPts, sDirs, sRows, plant ? sBad : sDir, plant * H};
            cl->RSSetViewports(1, &vp);
            cl->RSSetScissorRects(1, &sr);
            cl->SetGraphicsRoot32BitConstants(0, 5, c, 0);
            cl->DrawInstanced(3, 1, 0, 0);
        }
    }
    gpu.EndUpload();
    uint32_t pitch = 0;
    const std::vector<uint8_t> px = gpu.ReadbackTexture(rt, &pitch);
    uint32_t chains = 0, sameChain = 0, unreached = 0, plantDiffer = 0, plantThrough = 0, steps = 0;
    uint32_t perFace[6] = {};
    double worst = 0.0;
    for (uint32_t k = 0; k < N; ++k) {
        SurfaceFrame::WalkStep ref[SurfaceFrame::kMaxRanks];
        const uint32_t n = key.Walk(P[k].data(), ref);
        double uv[2];
        const uint32_t f = CubeFaceOfDir(P[k].data(), uv);
        ++perFace[f];
        bool same = true, differ = false;
        for (uint32_t plant = 0; plant < 2; ++plant) {
            for (uint32_t s = 0; s < SurfaceFrame::kMaxRanks; ++s) {
                const float* g = reinterpret_cast<const float*>(px.data() + size_t(plant * H + k / kW) * pitch) +
                                 4 * (5 * (k % kW) + s);
                if (g[3] == kUnreached) { ++unreached; continue; }
                const bool gHas = g[0] >= 0.0f, rHas = s < n;
                if (plant == 0) {
                    if (gHas != rHas || (s == 0 && uint32_t(g[3]) != n)) { same = false; continue; }
                    if (!rHas) continue;
                    if (uint32_t(g[0] + 0.5f) != ref[s].slice) { same = false; continue; }
                    Worse(worst, (std::max)(std::fabs(double(g[1]) - ref[s].u), std::fabs(double(g[2]) - ref[s].v)) * dim);
                    ++steps;
                } else if (gHas != rHas || (rHas && uint32_t(g[0] + 0.5f) != ref[s].slice)) {
                    differ = true;
                }
            }
        }
        ++chains;
        sameChain += same ? 1u : 0u;
        const bool through = f == 5 && uint32_t(uv[0] * 16.0) == pcx && uint32_t(uv[1] * 16.0) == pcy;
        plantThrough += through ? 1u : 0u;
        plantDiffer += differ ? 1u : 0u;
    }
    const bool within = sameChain == chains && worst < kBound && unreached == 0;
    const bool caught = plantDiffer > 0;
    Log("[tiletest] walk: %u points (face 0..5: %u %u %u %u %u %u) -- %u of %u chains the same, slice "
        "for slice; %u steps, uv worst %.6f texel against the doubles; %u pixels unreached",
        chains, perFace[0], perFace[1], perFace[2], perFace[3], perFace[4], perFace[5], sameChain,
        chains, steps, worst, unreached);
    Log("[tiletest] walk planted (face 5's cell (%u,%u) names slice 7 where the directory names 6): "
        "%u chains differ, %u points lie in that cell -- %s",
        pcx, pcy, plantDiffer, plantThrough, caught ? "CAUGHT" : "NOT CAUGHT: the readback is blind");
    const HRESULT removed = gpu.Device()->GetDeviceRemovedReason();
    const bool pass = within && caught && SUCCEEDED(removed);
    Log("[tiletest] ---- walk %s ----",
        pass ? "PASS: the two bodies of the walk agree on every chain, the uv within 0.01 texel, "
               "and the planted cell is caught"
             : "FAIL: see above");
    return pass;
}

}  // namespace

bool RunTileSelfTest(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    TileSelfTest test;
    const bool m0 = test.Run(gpu, sc, shaderDir);
    // HIERARCHY step 0 runs AFTER the M0 verdict, never inside it: the gate's lines and its
    // verdict stay what they were, and a probe fails the suite only when its instrument breaks.
    const bool step0 = test.RunHierarchyProbes(gpu, sc, shaderDir);
    // HIERARCHY step 3 after step 0's verdict, the same way. It is a GATE, not a probe: a texel
    // 0.01 off the doubles fails the suite, as a broken instrument does.
    const bool step3 = ProbeAddress(gpu, sc, shaderDir);
    // HIERARCHY 4.17 commit 3: the standing blocks' address with the key's own rows, a gate too.
    const bool blocks = ProbeBlockAddress(gpu, sc, shaderDir);
    // HIERARCHY 4.17 commit 4: the directory walk's two bodies held equal, a gate too.
    const bool walk = ProbeWalk(gpu, sc, shaderDir);
    return m0 && step0 && step3 && blocks && walk;
}

namespace {
// ---- M9h: POOL ACCOUNTING ACROSS BANKS ---------------------------------------------------
// Budgets stay PER BANK on purpose. A shared allocator would not help the algebra: banks align
// through the (level, x, y) ADDRESS SPACE, not through where their bytes physically sit, and
// co-locating two operands' tiles buys nothing -- they are read through the texture unit with
// swizzled layout and page-table translation, so adjacency removes no memory transaction, and
// concentrating them can narrow the channel interleave that the memory controller relies on.
//
// What a shared pool WOULD have caught is the sum: ten banks at 2 GB each is a 20 GB ceiling on
// a 7.9 GB card. So the sum is simply accounted and reported, which keeps banks independent and
// still makes the one real failure mode loud.
//
// (The genuine cross-bank coupling is not memory at all: if C = A * B, C's tiles are only
// useful where BOTH operands are resident, so a refusal in A wastes whatever B spent. That is
// DeriveDemand's job -- residency correlation, not allocation.)
// Atomic because main reads it for the recording's cost line while banks grow. Plain
// read-modify-write on a shared counter is a race even when every writer happens to be the
// frame thread today -- and this file's banks are exactly what a thread manager might move.
std::atomic<uint64_t> g_poolCommittedBytes{0};
std::atomic<bool> g_poolWarned{false};

void PoolCommitted(Gpu& gpu, uint64_t bytes) {
    const uint64_t held = g_poolCommittedBytes.fetch_add(bytes, std::memory_order_relaxed) + bytes;
    const uint64_t vram = gpu.DedicatedVramBytes();
    if (!vram) return;
    if (held * 10 > vram * 7 && !g_poolWarned.exchange(true)) {
        Log("[atlas] POOL PRESSURE: tile pools now hold %.2f GB of %.2f GB dedicated (>70%%). "
            "Budgets are per bank; nothing enforces the SUM, so this is the warning that the "
            "next bank to grow may be the one that fails.",
            held / 1073741824.0, vram / 1073741824.0);
    }
}
}   // namespace

// Committed tile-pool bytes across every atlas. Outside the anonymous namespace above on
// purpose: a recording reports the sparse structure's real cost, so main has to see it.
uint64_t PoolCommittedBytes() { return g_poolCommittedBytes.load(std::memory_order_relaxed); }

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

    // A paged bank is sampled through a Texture2DArray view (page = slice). A flat bank keeps
    // its plain view, because every existing consumer reads it as Texture2D.
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
// Send a pending residency map. Safe ONLY outside command-list recording.
void TileAtlas2D::FlushResidencyMap(Gpu& gpu) {
    if (!m_resMapDirty || m_resMapCpu.empty()) return;
    m_resMapDirty = false;
    gpu.UploadTexture(m_resMap, m_resMapCpu.data(), m_tilesX);
}

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
    // DO NOT UPLOAD HERE. Gpu::UploadTexture opens its own command list, and CommitMappings
    // is reachable from inside a frame's recording (BuildChain calls it) -- re-entering
    // BeginUpload there crashes. Mark it and let FlushResidencyMap send it from a context
    // that owns no open list. Discovered the hard way: the SWE spinup died on the first
    // chained bank, with no error, right where Record started.
    if (moved) m_resMapDirty = true;
    (void)gpu;
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
        std::vector<std::wstring> defs{L"GA_MIP_CH=" + std::to_wstring(channels)};
        if (m_coverageCh >= 0 && m_coverageCh < channels && channels >= 2) {
            // The bank declared which channel is coverage, so the reduction can weight by it.
            defs.push_back(L"GA_MIP_COVCH=" + std::to_wstring(m_coverageCh));
            Log("[atlas] mip reduce is COVERAGE-WEIGHTED on channel %d -- absent texels do "
                "not vote", m_coverageCh);
        }
        ShaderBlob cs =
            sc.Compile(shaderDir + L"/MipReduce.hlsl", L"CsMipReduce", L"cs_6_0", defs);
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
        // ALWAYS array views, even for a one-slice bank: MipReduce.hlsl declares
        // RWTexture2DArray so that one kernel serves paged and unpaged banks alike, and a
        // plain Texture2D view against that declaration is a binding mismatch. Pinned to the
        // slice being reduced, so a reduction can never cross a page boundary.
        u.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2DARRAY;
        u.Texture2DArray.FirstArraySlice = slice;
        u.Texture2DArray.ArraySize = 1;
        u.Texture2DArray.MipSlice = m;
        gpu.Device()->CreateUnorderedAccessView(m_res.Get(), nullptr, &u,
                                                gpu.SrvHeap().Cpu(slot));
        u.Texture2DArray.MipSlice = m + 1;
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
            if (m_freeTiles.empty() && m_poolTiles >= m_poolCapTiles) {
                // At budget. REFUSE rather than grow, and rather than evict -- this bank may
                // be holding state nothing else can reproduce (churn and foam memory exist
                // only here). A silent eviction would delete physics; a refusal is a hole,
                // and holes get counted and reported.
                m_state[i] = 0;
                ++m_refusedMaps;
                continue;
            }
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
                m_poolTiles += m_heapChunkTiles;
                PoolCommitted(gpu, hd.SizeInBytes);
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
            if (m_freeTiles.empty() && m_poolTiles >= m_poolCapTiles) {
                // At budget. REFUSE rather than grow, and rather than evict -- this bank may
                // be holding state nothing else can reproduce (churn and foam memory exist
                // only here). A silent eviction would delete physics; a refusal is a hole,
                // and holes get counted and reported.
                m_state[i] = 0;
                ++m_refusedMaps;
                continue;
            }
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
                m_poolTiles += m_heapChunkTiles;
                PoolCommitted(gpu, hd.SizeInBytes);
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
        // 1 TB is DELIBERATELY ABSENT. Measured once: 2048 slices of 16384^2 does not refuse
        // cleanly, it returns DXGI_ERROR_DEVICE_REMOVED -- and re-discovering that on every
        // --selftest run would take the device down every run and mask whatever ran after it
        // (it did: the floor probe below reported five spurious REFUSEDs). The ceiling is
        // recorded here rather than re-measured: 512 GB accepted, 1 TB removes the device.
        const uint64_t want[] = {16 * gb, 64 * gb, 128 * gb, 256 * gb, 512 * gb};
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

        // ---- WHAT DOES THE FLOOR COST? Every active page pins one, so this multiplies by the
        // number of pages held at once and could easily become the dominant memory cost. The
        // answer turns on whether a shape HAS a packed tail: a tail is typically a single 64 KB
        // tile, but a shape without one pins its coarsest STANDARD mip instead, which is a
        // whole mip level of tiles. Same guarantee, wildly different price.
        // ---- M9h: THE POOL CAP. The sample this design descends from budgets its pool and lives
    // inside it; this atlas simply created another heap whenever it ran dry. Prove the cap
    // holds, and prove that hitting it is LOUD -- a refused mapping is a hole in the field,
    // and a hole nobody counted is the failure mode worth preventing.
    {
        TileAtlas2D cap;
        cap.Init(gpu, 2048, 2048, DXGI_FORMAT_R16_FLOAT, L"atlastest.cap", 8);
        cap.SetPoolCapBytes(16 * 64 * 1024);   // 16 tiles, deliberately far too small
        for (uint32_t ty = 0; ty < cap.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < cap.TilesX(); ++tx) cap.RequestMap(tx, ty);
        }
        cap.CommitMappings(gpu, nullptr);
        const uint32_t asked = cap.TilesX() * cap.TilesY();
        const bool held = cap.PoolTiles() <= cap.PoolCapTiles();
        const bool counted = cap.RefusedMaps() == (asked - cap.ResidentCount());
        Log("[atlastest] pool cap: asked %u tiles, cap %u, resident %u, refused %u -- %s",
            asked, cap.PoolCapTiles(), cap.ResidentCount(), cap.RefusedMaps(),
            (held && counted) ? "cap held and every refusal counted" : "FAILED");
        if (!held || !counted) pass = false;
    }

    Log("[atlastest] floor cost per active page (pinned tail, or coarsest standard mip):");
        struct Shape { uint32_t w, h, mips; DXGI_FORMAT f; const char* n; };
        const Shape shapes[] = {
            {16384, 16384, 15, DXGI_FORMAT_R16_FLOAT, "16384^2 R16F   full chain"},
            {16384, 16384, 15, DXGI_FORMAT_R16G16B16A16_FLOAT, "16384^2 RGBA16F full chain"},
            {16384, 16384, 8, DXGI_FORMAT_R16_FLOAT, "16384^2 R16F   8 mips"},
            {4096, 4096, 5, DXGI_FORMAT_R16G16B16A16_FLOAT, "4096^2 RGBA16F 5 mips"},
            {4096, 4096, 12, DXGI_FORMAT_R16G16B16A16_FLOAT, "4096^2 RGBA16F full chain"},
        };
        for (const Shape& sp : shapes) {
            D3D12_RESOURCE_DESC rd{};
            rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
            rd.Width = sp.w;
            rd.Height = sp.h;
            rd.DepthOrArraySize = 1;
            rd.MipLevels = static_cast<UINT16>(sp.mips);
            rd.Format = sp.f;
            rd.SampleDesc.Count = 1;
            rd.Layout = D3D12_TEXTURE_LAYOUT_64KB_UNDEFINED_SWIZZLE;
            rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
            Com<ID3D12Resource> r;
            if (FAILED(gpu.Device()->CreateReservedResource(
                    &rd, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&r)))) {
                Log("[atlastest]   %-28s REFUSED", sp.n);
                continue;
            }
            UINT nt = 0, nsub = sp.mips;
            D3D12_PACKED_MIP_INFO pk{};
            D3D12_TILE_SHAPE sh{};
            std::vector<D3D12_SUBRESOURCE_TILING> st(nsub);
            gpu.Device()->GetResourceTiling(r.Get(), &nt, &pk, &sh, &nsub, 0, st.data());
            const uint32_t stdMips = pk.NumStandardMips ? pk.NumStandardMips : sp.mips;
            uint32_t floorTiles;
            const char* which;
            if (pk.NumPackedMips > 0) {
                floorTiles = pk.NumTilesForPackedMips;
                which = "tail";
            } else {
                floorTiles = st[stdMips - 1].WidthInTiles * st[stdMips - 1].HeightInTiles;
                which = "coarsest std mip";
            }
            const double kb = floorTiles * 64.0;
            Log("[atlastest]   %-28s floor = %4u tiles (%s) = %7.0f KB;  256 pages = %6.1f MB, "
                "1024 pages = %6.1f MB",
                sp.n, floorTiles, which, kb, kb * 256.0 / 1024.0, kb * 1024.0 / 1024.0);
        }
    }

    Log("[atlastest] ---- %s ----", pass ? "PASS" : "FAIL");
    return pass;
}

}  // namespace ga
