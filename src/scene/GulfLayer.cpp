#include "scene/GulfLayer.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Resources.h"
#include "hal/Root.h"
#include "hal/Views.h"
#include "scene/FieldSet.h"

#include <cmath>
#include <vector>

namespace ga {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

void GulfLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
                     hal::RootSignature rootSig) {
    (void)fields;
    m_rootSig = rootSig;
    if (!m_currents || !m_currents->Field().Valid()) {
        throw std::runtime_error("GulfLayer needs a current field");
    }
    const CurrentField& f = m_currents->Field();

    if (!BuildDrawPso(gpu, sc)) throw std::runtime_error("gulf draw PSO failed");

    // ---- upload the field as (u, v, mask, 0) RGBA32F
    std::vector<float> texels(static_cast<size_t>(f.nx) * f.ny * 4);
    for (int i = 0; i < f.nx * f.ny; ++i) {
        const bool water = f.u[i] > -900.0f;
        texels[i * 4 + 0] = water ? f.u[i] : 0.0f;
        texels[i * 4 + 1] = water ? f.v[i] : 0.0f;
        texels[i * 4 + 2] = water ? 1.0f : 0.0f;
        texels[i * 4 + 3] = 0.0f;
    }
    m_uvTex = gpu.CreateTexture2D(f.nx, f.ny, DXGI_FORMAT_R32G32B32A32_FLOAT,
                                  D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                                  D3D12_RESOURCE_STATE_COPY_DEST, L"gulf.uvField");
    gpu.UploadTexture(m_uvTex, texels.data(), f.nx * 16);
    m_uvSrv = gpu.CreateSrv(m_uvTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);

    auto makeOut = [&](const wchar_t* name) {
        return hal::Committed(gpu, name, static_cast<uint32_t>(f.nx), static_cast<uint32_t>(f.ny),
                              DXGI_FORMAT_R16G16B16A16_FLOAT,
                              D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                              D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                              "a derived field over the gulf's current grid, written once by the "
                              "gradient kernel and read as a texture: dense and small, not a bank");
    };
    m_mvTex = makeOut(L"gulf.mv2 (div, u, v, curl)");
    m_owTex = makeOut(L"gulf.okuboWeiss");
    m_mvSrv = gpu.CreateSrv(m_mvTex.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);
    m_owSrv = gpu.CreateSrv(m_owTex.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);

    RunVelGrad(gpu, sc);

    // ---- static CB parts
    const double midLat = f.lat0 + 0.5 * f.dlat * f.ny;
    const double lonSpanKm = f.dlon * f.nx * 111.320 * std::cos(midLat * kPi / 180.0);
    const double latSpanKm = f.dlat * f.ny * 110.574;
    m_aspectWoverH = static_cast<float>(lonSpanKm / latSpanKm);
    m_cb.srvs[0] = m_uvSrv;
    m_cb.srvs[1] = m_mvSrv;
    m_cb.srvs[2] = m_owSrv;
    m_cb.params[0] = 0.55f;      // colour ramp full scale, m/s
    m_cb.params[1] = 3.0f;       // |OW| threshold in the STORED 1e-10 1/s^2 units (= 3e-10)
    m_cb.params[2] = 1.7f;       // brightness into the ACES curve
    m_cb.params[3] = 0.5f;       // graticule spacing, degrees
    m_cb.geo[0] = static_cast<float>(f.lon0);
    m_cb.geo[1] = static_cast<float>(f.lat0);
    m_cb.geo[2] = static_cast<float>(f.dlon * f.nx);
    m_cb.geo[3] = static_cast<float>(f.dlat * f.ny);
    m_cb.marker[0] = static_cast<float>((kBuoyLon - f.lon0) / (f.dlon * f.nx));
    m_cb.marker[1] = static_cast<float>((kBuoyLat - f.lat0) / (f.dlat * f.ny));
    m_cb.marker[2] = 0.012f;
    m_cb.marker[3] = 1.0f;

    // ---- numeric gate: field at the buoy vs its ADCP
    float mu = 0, mv = 0;
    char buf[160];
    if (m_currents->Field().Sample(kBuoyLon, kBuoyLat, mu, mv)) {
        const double ms = std::sqrt(mu * mu + mv * mv);
        const double toward = std::fmod(std::atan2(mu, mv) * 180.0 / kPi + 360.0, 360.0);
        if (m_currents->adcpValid) {
            snprintf(buf, sizeof(buf),
                     "%s obs %.2f m/s @%03.0f | GoMOFS %.2f @%03.0f", m_currents->adcpId.c_str(),
                     m_currents->adcpMs, m_currents->adcpTowardDeg, ms, toward);
        } else {
            snprintf(buf, sizeof(buf), "GoMOFS at %s: %.2f m/s @%03.0f", m_currents->adcpId.c_str(), ms, toward);
        }
        validation = buf;
        Log("[gulf] %s", buf);
    }
    m_haveField = true;
}

bool GulfLayer::BuildDrawPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Gulf.hlsl";
    hal::GraphicsPipelineDesc d;   // the sky's defaults, exactly: a depth-off HDR quad
    d.rootSig = m_rootSig;
    d.vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    d.ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    return hal::Reload(m_drawPso, [&] { return hal::BuildGraphics(gpu, d, "gulf"); }, "gulf");
}

void GulfLayer::RunVelGrad(Gpu& gpu, ShaderCompiler& sc) {
    // Tiny dedicated compute root signature: constants + one 3-UAV table.
    if (!m_csRootSig) {
        m_csRootSig = hal::RootLayout{}
                          .Constants(0, 4)
                          .Table({hal::UavRange(0, 3)})
                          .Build(gpu, "gulf.velgrad");
        m_csRootSig->SetName(L"gulf velgrad root signature");

        m_csTable = hal::Table::Alloc(gpu, 3, "gulf.velgrad");
        m_csTable.Uav2D(0, m_uvTex.res.Get(), DXGI_FORMAT_R32G32B32A32_FLOAT);
        m_csTable.Uav2D(1, m_mvTex.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);
        m_csTable.Uav2D(2, m_owTex.Get(), DXGI_FORMAT_R16G16B16A16_FLOAT);
    }

    m_csPso = hal::Require(
        hal::BuildCompute(gpu, m_csRootSig.Get(),
                          sc.Compile(m_shaderDir + L"/VelGrad.hlsl", L"CsVelGrad", L"cs_6_0"),
                          "gulf.velgrad"),
        "VelGrad kernel");
    m_csPso->SetName(L"CsVelGrad");

    const CurrentField& f = m_currents->Field();
    struct { uint32_t nx, ny; float cx, cy; } c{};
    c.nx = static_cast<uint32_t>(f.nx);
    c.ny = static_cast<uint32_t>(f.ny);
    const double midLat = f.lat0 + 0.5 * f.dlat * f.ny;
    c.cx = static_cast<float>(f.dlon * 111320.0 * std::cos(midLat * kPi / 180.0));
    c.cy = static_cast<float>(f.dlat * 110574.0);

    hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
    up.BindHeaps();
    {
        PixScope scope(up.Native(), "gulf.velgrad (grad(U): div->grade0, vorticity->grade2, OW)");
        up.Barrier(m_uvTex.res.Get(), m_uvTex.state, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        up.ComputeRoot(m_csRootSig.Get());
        up.Native()->SetComputeRoot32BitConstants(0, 4, &c, 0);
        up.ComputeTable(1, m_csTable.Base());
        up.Pipeline(m_csPso.Get());
        up.Dispatch((c.nx + 7) / 8, (c.ny + 7) / 8, 1);
        up.Barrier(m_uvTex.res.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                   D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        up.Barrier(m_mvTex.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                   D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        up.Barrier(m_owTex.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                   D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    }
    gpu.EndUpload();
    m_uvTex.state = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
}

void GulfLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildDrawPso(gpu, sc);   // the reload law lives in BuildDrawPso: swap only on success
}

void GulfLayer::Render(const FrameContext& ctx) {
    if (!m_haveField || !m_drawPso) return;
    PixScope scope(ctx.cmd->Native(), "gulf.map (speed ramp + OW eddies by bivector sign)");

    // Letterbox the geographic aspect into the viewport.
    const float vpAspect = static_cast<float>(ctx.width) / static_cast<float>(ctx.height);
    float halfW = 0.94f, halfH = 0.94f;
    const float panelAspect = m_aspectWoverH / vpAspect;   // ndc width per ndc height
    if (panelAspect > 1.0f) halfH = 0.94f / panelAspect;
    else halfW = 0.94f * panelAspect;
    m_cb.panel[0] = 0.0f;
    m_cb.panel[1] = 0.0f;
    m_cb.panel[2] = halfW;
    m_cb.panel[3] = halfH;

    ctx.cmd->Pipeline(m_drawPso.Get());
    ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.cmd->GraphicsConstants(1, m_cb);
    ctx.cmd->Draw(6, 1, 0, 0);
}

}  // namespace ga
