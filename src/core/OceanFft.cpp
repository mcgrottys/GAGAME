#include "core/OceanFft.h"

#include "core/PixEvents.h"

#include <algorithm>
#include <cmath>

namespace ga {

namespace {

constexpr float kPi = 3.14159265358979f;

struct FftConsts {
    uint32_t n, dir, cascade, seed;
    float kLo, kHi, patchL, time;
    float lambda, foamScale, foamBias;
    uint32_t pad0;
    float pad[4];
};

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

Com<ID3D12Resource> MakeTex(Gpu& gpu, uint32_t n, DXGI_FORMAT fmt, const wchar_t* name,
                            D3D12_RESOURCE_STATES state) {
    D3D12_RESOURCE_DESC d{};
    d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    d.Width = n;
    d.Height = n;
    d.DepthOrArraySize = 1;
    d.MipLevels = 1;
    d.Format = fmt;
    d.SampleDesc.Count = 1;
    d.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
    D3D12_HEAP_PROPERTIES hp{};
    hp.Type = D3D12_HEAP_TYPE_DEFAULT;
    Com<ID3D12Resource> res;
    GA_CHECK(gpu.Device()->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, state, nullptr,
                                                  IID_PPV_ARGS(&res)));
    res->SetName(name);
    return res;
}

}  // namespace

double OceanFft::MeasureHs(Gpu& gpu) {
    double m0 = 0.0;
    for (uint32_t c = 0; c < kCascades; ++c) {
        const D3D12_RESOURCE_STATES st = m_cascade[c].outputsArePs
                                             ? D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE
                                             : D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        GpuTexture t;
        t.res = m_cascade[c].disp;
        t.state = st;
        t.format = DXGI_FORMAT_R16G16B16A16_FLOAT;
        t.width = kN;
        t.height = kN;
        uint32_t pitch = 0;
        const std::vector<uint8_t> data = gpu.ReadbackTexture(t, &pitch);
        double sum = 0.0, sum2 = 0.0;
        for (uint32_t y = 0; y < kN; ++y) {
            const uint16_t* row = reinterpret_cast<const uint16_t*>(data.data() +
                                                                    static_cast<size_t>(y) * pitch);
            for (uint32_t x = 0; x < kN; ++x) {
                const double h = HalfToFloat(row[x * 4 + 1]);   // .y = height
                sum += h;
                sum2 += h * h;
            }
        }
        const double n = static_cast<double>(kN) * kN;
        const double mean = sum / n;
        m0 += sum2 / n - mean * mean;

        // Derivative stats: where does the Jacobian actually sit per cascade?
        GpuTexture d;
        d.res = m_cascade[c].deriv;
        d.state = st;
        d.format = DXGI_FORMAT_R16G16B16A16_FLOAT;
        d.width = kN;
        d.height = kN;
        uint32_t dpitch = 0;
        const std::vector<uint8_t> dd = gpu.ReadbackTexture(d, &dpitch);
        double minJ = 1e9, maxJ = -1e9;
        uint32_t foamy = 0;
        for (uint32_t y = 0; y < kN; ++y) {
            const uint16_t* row = reinterpret_cast<const uint16_t*>(dd.data() +
                                                                    static_cast<size_t>(y) * dpitch);
            for (uint32_t x = 0; x < kN; ++x) {
                const double J = HalfToFloat(row[x * 4 + 2]);
                const double fo = HalfToFloat(row[x * 4 + 3]);
                minJ = std::min(minJ, J);
                maxJ = std::max(maxJ, J);
                foamy += (fo > 0.01) ? 1 : 0;
            }
        }
        Log("[verify] cascade %u: J range [%.3f, %.3f], foam on %.1f%% of texels", c, minJ, maxJ,
            100.0 * foamy / (kN * kN));
    }
    return 4.0 * std::sqrt(std::max(m0, 0.0));
}

void OceanFft::Init(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir) {
    m_shaderDir = shaderDir;

    // Cascade bands: cuts at 60 m and 12 m wavelengths so no wave is synthesised twice.
    const float cut01 = 2.0f * kPi / 60.0f;
    const float cut12 = 2.0f * kPi / 12.0f;
    m_bandLo[0] = 2.0f * kPi / m_patchL[0];
    m_bandHi[0] = cut01;
    m_bandLo[1] = cut01;
    m_bandHi[1] = cut12;
    m_bandLo[2] = cut12;
    m_bandHi[2] = 0.9f * kPi * kN / m_patchL[2];   // guard band below Nyquist

    // Root signature: b1 root constants, b0 CBV (partitions), one table of 4 UAVs.
    D3D12_DESCRIPTOR_RANGE1 range{};
    range.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
    range.NumDescriptors = 4;
    range.BaseShaderRegister = 0;
    range.Flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE;
    range.OffsetInDescriptorsFromTableStart = 0;

    D3D12_ROOT_PARAMETER1 params[3]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[0].Constants.ShaderRegister = 1;   // b1
    params[0].Constants.Num32BitValues = sizeof(FftConsts) / 4;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[1].Descriptor.ShaderRegister = 0;  // b0
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[2].DescriptorTable.NumDescriptorRanges = 1;
    params[2].DescriptorTable.pDescriptorRanges = &range;

    D3D12_VERSIONED_ROOT_SIGNATURE_DESC vd{};
    vd.Version = D3D_ROOT_SIGNATURE_VERSION_1_1;
    vd.Desc_1_1.NumParameters = _countof(params);
    vd.Desc_1_1.pParameters = params;

    Com<ID3DBlob> blob, err;
    HRESULT hr = D3D12SerializeVersionedRootSignature(&vd, &blob, &err);
    if (FAILED(hr)) {
        if (err) Log("[ocean] root sig: %s", static_cast<const char*>(err->GetBufferPointer()));
        GA_CHECK(hr);
    }
    GA_CHECK(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(),
                                              IID_PPV_ARGS(&m_rootSig)));
    m_rootSig->SetName(L"ocean compute root signature");

    if (!BuildPipelines(gpu, sc)) throw std::runtime_error("ocean compute shaders failed");

    for (uint32_t c = 0; c < kCascades; ++c) {
        Cascade& k = m_cascade[c];
        wchar_t name[64];
        auto mk = [&](const wchar_t* what, DXGI_FORMAT fmt, D3D12_RESOURCE_STATES st) {
            swprintf(name, 64, L"ocean.c%u.%s", c, what);
            return MakeTex(gpu, kN, fmt, name, st);
        };
        k.h0 = mk(L"h0", DXGI_FORMAT_R32G32B32A32_FLOAT, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        k.pingA = mk(L"pingA", DXGI_FORMAT_R32G32B32A32_FLOAT,
                     D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        k.pongA = mk(L"pongA", DXGI_FORMAT_R32G32B32A32_FLOAT,
                     D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        k.pingB = mk(L"pingB", DXGI_FORMAT_R32G32B32A32_FLOAT,
                     D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        k.pongB = mk(L"pongB", DXGI_FORMAT_R32G32B32A32_FLOAT,
                     D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        k.disp = mk(L"disp", DXGI_FORMAT_R16G16B16A16_FLOAT,
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        k.deriv = mk(L"deriv", DXGI_FORMAT_R16G16B16A16_FLOAT,
                     D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

        // Bindless SRVs for the draw side.
        m_dispSrv[c] = gpu.CreateSrv(k.disp.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);
        m_derivSrv[c] = gpu.CreateSrv(k.deriv.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);

        // Descriptor blocks (4 consecutive UAVs each), one per kernel binding pattern.
        auto block = [&](ID3D12Resource* u0, ID3D12Resource* u1, ID3D12Resource* u2,
                         ID3D12Resource* u3) {
            const uint32_t base = gpu.SrvHeap().Alloc(4);
            ID3D12Resource* res[4] = {u0, u1, u2, u3};
            for (uint32_t s = 0; s < 4; ++s) {
                D3D12_UNORDERED_ACCESS_VIEW_DESC uv{};
                uv.Format = (res[s] == k.disp.Get() || res[s] == k.deriv.Get())
                                ? DXGI_FORMAT_R16G16B16A16_FLOAT
                                : DXGI_FORMAT_R32G32B32A32_FLOAT;
                uv.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
                gpu.Device()->CreateUnorderedAccessView(res[s], nullptr, &uv,
                                                        gpu.SrvHeap().Cpu(base + s));
            }
            return base;
        };
        k.blockInit = block(k.h0.Get(), k.pingA.Get(), k.pingB.Get(), k.pongA.Get());
        k.blockMod = block(k.h0.Get(), k.pingA.Get(), k.pingB.Get(), k.pongA.Get());
        k.blockRows = block(k.pingA.Get(), k.pongA.Get(), k.pingB.Get(), k.pongB.Get());
        k.blockCols = block(k.pongA.Get(), k.pingA.Get(), k.pongB.Get(), k.pingB.Get());
        k.blockAsm = block(k.disp.Get(), k.deriv.Get(), k.pingA.Get(), k.pingB.Get());
    }
    Log("[ocean] %u cascades of %u^2, patches %.0f/%.0f/%.0f m, disp srv %u/%u/%u", kCascades,
        kN, m_patchL[0], m_patchL[1], m_patchL[2], m_dispSrv[0], m_dispSrv[1], m_dispSrv[2]);
    m_ready = true;
}

bool OceanFft::BuildPipelines(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/OceanCompute.hlsl";
    struct Entry { const wchar_t* name; Com<ID3D12PipelineState>* pso; };
    Entry entries[] = {
        {L"CsInitSpectrum", &m_init}, {L"CsModulate", &m_modulate},
        {L"CsFft", &m_fft}, {L"CsAssemble", &m_assemble},
    };
    for (Entry& e : entries) {
        ShaderBlob cs = sc.Compile(path, e.name, L"cs_6_0");
        if (!cs.Valid()) return false;
        D3D12_COMPUTE_PIPELINE_STATE_DESC d{};
        d.pRootSignature = m_rootSig.Get();
        d.CS = {cs.Data(), cs.Size()};
        Com<ID3D12PipelineState> pso;
        if (FAILED(gpu.Device()->CreateComputePipelineState(&d, IID_PPV_ARGS(&pso)))) return false;
        pso->SetName(e.name);
        *e.pso = pso;
    }
    return true;
}

bool OceanFft::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    Com<ID3D12PipelineState> keep[4] = {m_init, m_modulate, m_fft, m_assemble};
    if (!BuildPipelines(gpu, sc)) {
        m_init = keep[0];
        m_modulate = keep[1];
        m_fft = keep[2];
        m_assemble = keep[3];
        return false;
    }
    m_spectrumDirty = true;   // new spectrum kernel may pack differently; rebuild h0
    return true;
}

void OceanFft::SetSeaState(const PartParam* parts, int count, uint32_t seed) {
    m_parts = PartsCbData{};
    m_parts.count[0] = static_cast<uint32_t>(count);
    for (int i = 0; i < count && i < 4; ++i) m_parts.parts[i] = parts[i];
    m_seed = seed;
    m_spectrumDirty = true;
}

void OceanFft::Dispatch(ID3D12GraphicsCommandList* cl, Gpu& gpu, ID3D12PipelineState* pso,
                        uint32_t block, uint32_t cascade, uint32_t dir, float tSec, uint32_t gx,
                        uint32_t gy) {
    FftConsts c{};
    c.n = kN;
    c.dir = dir;
    c.cascade = cascade;
    c.seed = m_seed;
    c.kLo = m_bandLo[cascade];
    c.kHi = m_bandHi[cascade];
    c.patchL = m_patchL[cascade];
    c.time = tSec;
    c.lambda = m_lambda;
    // Whitecaps where the Jacobian approaches folding. With the erf-integrated spectrum the
    // per-bin energies are honest (the old point-sampled overshoot used to fold calm seas), so a
    // moderate threshold gives foam on storm crests and a clean surface at Hs 0.4 m.
    c.foamScale = 4.0f;
    c.foamBias = 0.80f;
    cl->SetComputeRoot32BitConstants(0, sizeof(c) / 4, &c, 0);
    cl->SetComputeRootConstantBufferView(1, gpu.PushConstants(&m_parts, sizeof(m_parts)));
    cl->SetComputeRootDescriptorTable(2, gpu.SrvHeap().Gpu(block));
    cl->SetPipelineState(pso);
    cl->Dispatch(gx, gy, 1);
}

void OceanFft::Record(ID3D12GraphicsCommandList* cl, Gpu& gpu, float tSec) {
    if (!m_ready) return;
    PixScope frame(cl, "ocean.fft (3 cascades: modulate -> rows -> cols -> assemble)");
    cl->SetComputeRootSignature(m_rootSig.Get());

    for (uint32_t ci = 0; ci < kCascades; ++ci) {
        Cascade& k = m_cascade[ci];

        if (k.outputsArePs) {
            Barrier(cl, k.disp.Get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE,
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            Barrier(cl, k.deriv.Get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE,
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            k.outputsArePs = false;
        }

        if (m_spectrumDirty) {
            PixScope scope(cl, "ocean.initSpectrum");
            Dispatch(cl, gpu, m_init.Get(), k.blockInit, ci, 0, tSec, kN / 8, kN / 8);
            UavBarrier(cl, k.h0.Get());
        }
        {
            PixScope scope(cl, "ocean.modulate (the rotor)");
            Dispatch(cl, gpu, m_modulate.Get(), k.blockMod, ci, 0, tSec, kN / 8, kN / 8);
            UavBarrier(cl, k.pingA.Get());
            UavBarrier(cl, k.pingB.Get());
        }
        {
            PixScope scope(cl, "ocean.fft.rows");
            Dispatch(cl, gpu, m_fft.Get(), k.blockRows, ci, 0, tSec, kN, 1);
            UavBarrier(cl, k.pongA.Get());
            UavBarrier(cl, k.pongB.Get());
        }
        {
            PixScope scope(cl, "ocean.fft.cols");
            Dispatch(cl, gpu, m_fft.Get(), k.blockCols, ci, 1, tSec, kN, 1);
            UavBarrier(cl, k.pingA.Get());
            UavBarrier(cl, k.pingB.Get());
        }
        {
            PixScope scope(cl, "ocean.assemble");
            Dispatch(cl, gpu, m_assemble.Get(), k.blockAsm, ci, 0, tSec, kN / 8, kN / 8);
        }

        // ALL_SHADER_RESOURCE (pixel + non-pixel) so the churn COMPUTE pass can read the chop
        // derivatives in the same frame the sea PS samples them.
        Barrier(cl, k.disp.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
        Barrier(cl, k.deriv.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
        k.outputsArePs = true;
    }
    m_spectrumDirty = false;
}

}  // namespace ga
