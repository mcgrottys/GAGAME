#include "core/OceanFft.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Resources.h"
#include "hal/Root.h"
#include "hal/Views.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

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

hal::ResourceRef MakeTex(Gpu& gpu, uint32_t n, DXGI_FORMAT fmt, const wchar_t* name,
                            D3D12_RESOURCE_STATES state, uint32_t mips = 1) {
    return hal::Committed(gpu, name, n, n, fmt, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, state,
                          "the FFT's working set: a dense n^2 UAV texture the kernels ping-pong "
                          "through whole, never partly resident, so not a bank",
                          static_cast<uint16_t>(mips));
}

}  // namespace

// M9bo: fill one cascade texture's chain, coarse level by coarse level, while the resource is
// still in UNORDERED_ACCESS from assembly. A UAV barrier between levels because level m+1 reads
// what level m just wrote.
void OceanFft::BuildMips(hal::CommandContext& cmd, Gpu& gpu, Cascade& k) {
    (void)gpu;   // the context carries the device
    if (!m_mipPso) return;
    cmd.ComputeRoot(m_mipRs.Get());
    cmd.Pipeline(m_mipPso.Get());
    hal::Resource res[2] = {k.disp.Get(), k.deriv.Get()};
    const uint32_t table[2] = {k.mipTableDisp, k.mipTableDeriv};
    for (uint32_t t = 0; t < 2; ++t) {
        for (uint32_t m = 0; m + 1 < kMips; ++m) {
            const uint32_t sw = (kN >> m) ? (kN >> m) : 1u;
            const uint32_t dw = (kN >> (m + 1)) ? (kN >> (m + 1)) : 1u;
            const uint32_t consts[4] = {dw, dw, sw, sw};
            cmd.Native()->SetComputeRoot32BitConstants(0, 4, consts, 0);
            cmd.ComputeTable(1, table[t] + 2 * m);
            cmd.Dispatch((dw + 7) / 8, (dw + 7) / 8, 1);
            cmd.UavBarrier(res[t]);
        }
    }
    // Record binds the ocean root signature ONCE, before the cascade loop, and Dispatch never
    // re-binds it -- so leaving m_mipRs bound here runs the NEXT cascade's FFT kernels against
    // the wrong root signature. That is an access violation inside the driver, reported at
    // OceanFft::Dispatch with nothing in this function on the stack. Put it back.
    cmd.ComputeRoot(m_rootSig.Get());
}

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
    m_rootSig = hal::RootLayout{}
                    .Constants(1, sizeof(FftConsts) / 4)
                    .Cbv(0)
                    .Table({hal::UavRange(0, 4)})
                    .Build(gpu, "ocean");
    m_rootSig->SetName(L"ocean compute root signature");

    if (!BuildPipelines(gpu, sc)) throw std::runtime_error("ocean compute shaders failed");

    // M9bo: the 2x2 box reduce that fills disp/deriv's mip chain. Averaging is the CORRECT
    // reduction here for the reason MipReduce.hlsl's header gives: these are FIELD banks, where
    // a texel absent means the quantity is identically zero, so a box average is an honest
    // average of the field. (The coverage-weighted path exists for texture tenants, where null
    // means ABSENT; using it here would be wrong in the other direction.)
    {
        // (Until step 3d the one version-1.0 root signature in the tree; 1.1 with the volatile
        // flag 1.0 implied is the same signature to the driver.)
        m_mipRs = hal::RootLayout{}
                      .Constants(0, 4)
                      .Table({hal::UavRange(0, 2)})
                      .Build(gpu, "ocean.mip");
        m_mipRs->SetName(L"ocean mip reduce root signature");
        std::vector<std::wstring> defs{L"GA_MIP_CH=4"};
        m_mipPso = hal::Require(
            hal::BuildCompute(gpu, m_mipRs.Get(),
                              sc.Compile(shaderDir + L"/MipReduce.hlsl", L"CsMipReduce",
                                         L"cs_6_0", defs),
                              "ocean.mip"),
            "ocean MipReduce kernel");
        m_mipPso->SetName(L"ocean CsMipReduce");
    }

    for (uint32_t c = 0; c < kCascades; ++c) {
        Cascade& k = m_cascade[c];
        wchar_t name[64];
        auto mk = [&](const wchar_t* what, DXGI_FORMAT fmt, D3D12_RESOURCE_STATES st,
                      uint32_t mips = 1) {
            swprintf(name, 64, L"ocean.c%u.%s", c, what);
            return MakeTex(gpu, kN, fmt, name, st, mips);
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
                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS, kMips);
        k.deriv = mk(L"deriv", DXGI_FORMAT_R16G16B16A16_FLOAT,
                     D3D12_RESOURCE_STATE_UNORDERED_ACCESS, kMips);

        // Bindless SRVs for the draw side.
        m_dispSrv[c] = gpu.CreateSrv(k.disp.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);
        m_derivSrv[c] = gpu.CreateSrv(k.deriv.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);

        // Descriptor blocks (4 consecutive UAVs each), one per kernel binding pattern.
        auto block = [&](hal::Resource u0, hal::Resource u1, hal::Resource u2,
                         hal::Resource u3) {
            hal::Table t = hal::Table::Alloc(gpu, 4, "ocean.block");
            hal::Resource res[4] = {u0, u1, u2, u3};
            for (uint32_t s = 0; s < 4; ++s) {
                const DXGI_FORMAT fmt = (res[s] == k.disp.Get() || res[s] == k.deriv.Get())
                                            ? DXGI_FORMAT_R16G16B16A16_FLOAT
                                            : DXGI_FORMAT_R32G32B32A32_FLOAT;
                t.Uav2D(s, res[s], fmt);
            }
            return t.Base();
        };
        k.blockInit = block(k.h0.Get(), k.pingA.Get(), k.pingB.Get(), k.pongA.Get());
        k.blockMod = block(k.h0.Get(), k.pingA.Get(), k.pingB.Get(), k.pongA.Get());
        k.blockRows = block(k.pingA.Get(), k.pongA.Get(), k.pingB.Get(), k.pongB.Get());
        k.blockCols = block(k.pongA.Get(), k.pingA.Get(), k.pongB.Get(), k.pingB.Get());
        k.blockAsm = block(k.disp.Get(), k.deriv.Get(), k.pingA.Get(), k.pingB.Get());

        // M9bo: the reduce chain's descriptors. Texture2DArray views pinned to slice 0 --
        // MipReduce.hlsl declares RWTexture2DArray so ONE kernel serves paged and unpaged
        // banks alike, and a plain Texture2D view against that declaration is a binding
        // mismatch (TileAtlas2D::BuildMips carries the same note).
        auto mipTable = [&](hal::Resource res) {
            hal::Table t = hal::Table::Alloc(gpu, 2 * (kMips - 1), "ocean.mip");
            for (uint32_t m = 0; m + 1 < kMips; ++m) {
                t.UavArray(2 * m, res, DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1, m);
                t.UavArray(2 * m + 1, res, DXGI_FORMAT_R16G16B16A16_FLOAT, 0, 1, m + 1);
            }
            return t.Base();
        };
        k.mipTableDisp = mipTable(k.disp.Get());
        k.mipTableDeriv = mipTable(k.deriv.Get());
    }
    Log("[ocean] %u cascades of %u^2, patches %.0f/%.0f/%.0f m, disp srv %u/%u/%u", kCascades,
        kN, m_patchL[0], m_patchL[1], m_patchL[2], m_dispSrv[0], m_dispSrv[1], m_dispSrv[2]);
    m_ready = true;
}

bool OceanFft::BuildPipelines(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/OceanCompute.hlsl";
    struct Entry { const wchar_t* name; hal::Pso* pso; };
    Entry entries[] = {
        {L"CsInitSpectrum", &m_init}, {L"CsModulate", &m_modulate},
        {L"CsFft", &m_fft}, {L"CsAssemble", &m_assemble},
    };
    // The four kernels swap together or not at all: a spectrum from one and an assembly from
    // another would not agree.
    hal::ReloadSet set;
    for (Entry& e : entries) {
        auto pso = hal::BuildCompute(gpu, m_rootSig.Get(), sc.Compile(path, e.name, L"cs_6_0"),
                                     "ocean");
        if (!pso) return false;
        pso->SetName(e.name);
        set.Add(*e.pso, std::move(pso));
    }
    set.Commit();
    return true;
}

bool OceanFft::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    if (!BuildPipelines(gpu, sc)) return false;
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

void OceanFft::Dispatch(hal::CommandContext& cmd, Gpu& gpu, hal::PsoPtr pso,
                        uint32_t block, uint32_t cascade, uint32_t dir, float tSec, uint32_t gx,
                        uint32_t gy) {
    (void)gpu;   // the context carries the device
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
    cmd.Native()->SetComputeRoot32BitConstants(0, sizeof(c) / 4, &c, 0);
    cmd.ComputeConstants(1, m_parts);
    cmd.ComputeTable(2, block);
    cmd.Pipeline(pso);
    cmd.Dispatch(gx, gy, 1);
}

void OceanFft::Record(hal::CommandContext& cmd, Gpu& gpu, float tSec) {
    if (!m_ready) return;
    PixScope frame(cmd.Native(), "ocean.fft (3 cascades: modulate -> rows -> cols -> assemble)");
    cmd.ComputeRoot(m_rootSig.Get());

    for (uint32_t ci = 0; ci < kCascades; ++ci) {
        Cascade& k = m_cascade[ci];

        if (k.outputsArePs) {
            cmd.Barrier(k.disp.Get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE,
                        D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            cmd.Barrier(k.deriv.Get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE,
                        D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            k.outputsArePs = false;
        }

        if (m_spectrumDirty) {
            PixScope scope(cmd.Native(), "ocean.initSpectrum");
            Dispatch(cmd, gpu, m_init.Get(), k.blockInit, ci, 0, tSec, kN / 8, kN / 8);
            cmd.UavBarrier(k.h0.Get());
        }
        {
            PixScope scope(cmd.Native(), "ocean.modulate (the rotor)");
            Dispatch(cmd, gpu, m_modulate.Get(), k.blockMod, ci, 0, tSec, kN / 8, kN / 8);
            cmd.UavBarrier(k.pingA.Get());
            cmd.UavBarrier(k.pingB.Get());
        }
        {
            PixScope scope(cmd.Native(), "ocean.fft.rows");
            Dispatch(cmd, gpu, m_fft.Get(), k.blockRows, ci, 0, tSec, kN, 1);
            cmd.UavBarrier(k.pongA.Get());
            cmd.UavBarrier(k.pongB.Get());
        }
        {
            PixScope scope(cmd.Native(), "ocean.fft.cols");
            Dispatch(cmd, gpu, m_fft.Get(), k.blockCols, ci, 1, tSec, kN, 1);
            cmd.UavBarrier(k.pingA.Get());
            cmd.UavBarrier(k.pingB.Get());
        }
        {
            PixScope scope(cmd.Native(), "ocean.assemble");
            Dispatch(cmd, gpu, m_assemble.Get(), k.blockAsm, ci, 0, tSec, kN / 8, kN / 8);
            cmd.UavBarrier(k.disp.Get());
            cmd.UavBarrier(k.deriv.Get());
        }
        {
            // M9bo: the chain, while both are still UAVs. The bank kernel reads these at the
            // RING's texel, which is coarser than the cascade's by up to 6.7x on ring 0 and far
            // more outward; without the chain that read was a mip-0 bilinear tap and everything
            // between the two scales aliased into the geometry.
            PixScope scope(cmd.Native(), "ocean.mips");
            BuildMips(cmd, gpu, k);
        }

        // ALL_SHADER_RESOURCE (pixel + non-pixel) so the churn COMPUTE pass can read the chop
        // derivatives in the same frame the sea PS samples them.
        cmd.Barrier(k.disp.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                    D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
        cmd.Barrier(k.deriv.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                    D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
        k.outputsArePs = true;
    }
    m_spectrumDirty = false;
}

}  // namespace ga
