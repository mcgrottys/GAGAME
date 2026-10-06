#include "scene/GlobeLayer.h"
#include "scene/Air.h"
#include "core/ThreadManager.h"

#include "hal/GpuProfiler.h"

#include "compose/DomainSource.h"
#include "sim/BathyModel.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
#include "hal/Resources.h"
#include "hal/Root.h"
#include "hal/Views.h"
#include "hal/Shader.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

using namespace DirectX;

namespace ga {

// ================================================================================================
//  M9q: a global plane, through the process. GA Load (MemGridLoader over the array GlobeModel
//  already parsed) -> GA Compose (RasterSource through DomainCompositor, so coverage and the
//  unit check apply) -> a reserved, paged, mipped GradeBank.
//
//  Every mip is COMPOSED at its own rung rather than reduced from the level below. That costs a
//  few more source reads and buys the thing reduction cannot give: each level asks the source
//  for its own footprint, so absence is resolved per level instead of being averaged in from
//  finer texels. (MipReduce's coverage-weighted path exists for banks that must reduce; a bank
//  built from a source that can answer at any rung should just ask.)
//
//  The verification is the same discipline as the bed: compose L0, compare it against the array
//  the committed texture was uploaded from, and print the worst disagreement. A conversion that
//  cannot say it matches is not a conversion, it is a rewrite.
// ================================================================================================
bool GlobeLayer::BuildPlaneBank(Gpu& gpu, PlaneBank& out, const char* name,
                                const char* structure, const GeoRef& ref,
                                std::vector<MemGridLoader::Plane> planes, DXGI_FORMAT fmt,
                                float nodataFill) {
    const std::vector<MemGridLoader::Plane> keep = planes;
    auto ld = std::make_unique<MemGridLoader>(name, structure, ref, std::move(planes));
    if (!ld->Valid()) {
        Log("[globe] %s: loader rejected the planes -- not converted", name);
        return false;
    }
    const uint32_t nx = ref.width, ny = ref.height, ch = uint32_t(keep.size());
    auto ras = std::make_shared<RasterSource>(std::move(ld), 0);
    if (!ras->Valid()) {
        Log("[globe] %s: no samples -- not converted", name);
        return false;
    }
    DomainCompositor dc;
    LevelLadder lad;
    lad.level0MetersPerTexel = ref.MetersPerTexelX();
    dc.SetLadder(lad);
    if (!dc.Add(ras)) return false;   // the unit stage refused it; the log says why

    GradeBankDesc d;
    d.name = name;
    d.width = nx;
    d.height = ny;
    d.fmt = fmt;
    d.gradeSig = kG0;
    d.metersPerTexel = lad.level0MetersPerTexel;
    d.units = ref.valueUnit;
    d.range = "";
    d.mipLevels = 5;
    d.arraySlices = 1;
    out.bank = std::make_unique<GradeBank>();
    out.bank->Init(gpu, d, policy::None());
    out.bank->ActivateSlice(gpu, 0);
    out.bank->MapAllLevels(gpu, 0);

    double worst = 0.0;
    uint32_t levels = 0, coveredL0 = 0, holesL0 = 0;
    for (uint32_t lvl = 0; lvl < d.mipLevels; ++lvl) {
        const uint32_t w = (nx >> lvl) ? (nx >> lvl) : 1u;
        const uint32_t h = (ny >> lvl) ? (ny >> lvl) : 1u;
        DomainCompositor::PageGeo g;
        g.dLon = ref.scaleX * double(nx) / double(w);
        g.dLat = ref.scaleY * double(ny) / double(h);
        g.lon0 = ref.originX + (ref.centers ? 0.5 : 0.0) * g.dLon;
        g.lat0 = ref.originY + (ref.centers ? 0.5 : 0.0) * g.dLat;
        std::vector<float> page, cov;
        const uint32_t covered =
            dc.ComposePage(PageAddr{lvl, 0, 0}, g, w, h, ch, page, cov, 0.0, 0.0, nodataFill);
        if (!covered) continue;
        if (lvl == 0) {
            coveredL0 = covered;
            holesL0 = uint32_t(size_t(w) * h) - covered;
            for (uint32_t y = 0; y < h; ++y) {
                for (uint32_t x = 0; x < w; ++x) {
                    const size_t i = size_t(y) * w + x;
                    if (cov[i] <= 0.0f) continue;
                    // A texel is comparable only where EVERY channel retrieved, because that is
                    // the loader's rule: the ocean model consumes chl, Kd and SPM together, so
                    // one missing plane substitutes the pure-water limit into ALL of them. A
                    // per-channel skip is not enough -- it leaves the retrieved channels of a
                    // partly-missing texel being compared against a substitution that was
                    // correct, which is what reported 6.0 here. Same statement as the loader's,
                    // written once more because the check must not have its own opinion.
                    bool comparable = true;
                    for (uint32_t c = 0; c < ch && comparable; ++c) {
                        if (!keep[c].data) continue;   // derived channel: no array of its own
                        if ((*keep[c].data)[i] <= keep[c].nodataBelow) comparable = false;
                    }
                    if (!comparable) continue;
                    for (uint32_t c = 0; c < ch; ++c) {
                        if (!keep[c].data) continue;
                        const double src = double((*keep[c].data)[i]);
                        worst = (std::max)(worst, std::abs(double(page[i * ch + c]) - src));
                    }
                }
            }
        }
        out.bank->UploadLevel(gpu, lvl, page.data(), w * ch * sizeof(float), w, h, 0);
        ++levels;
    }
    if (!levels) {
        Log("[globe] %s: composed nothing -- not converted", name);
        out.bank.reset();
        return false;
    }
    out.srv = gpu.CreateSrv(out.bank->Res(), fmt);
    Log("[globe] %s -> sparse bank: %ux%u x%u ch, %u levels, %u/%u covered at L0 (%u nodata), "
        "worst |GA - array| = %.6g (%s)",
        name, nx, ny, ch, levels, coveredL0, nx * ny, holesL0, worst,
        (worst < 1e-4) ? "equivalent" : "DIVERGENT");
    return true;
}


namespace {

constexpr double kPi = 3.14159265358979;

// M9: the pure-water limit, in the fibers' own encodings. docs/ALGEBRA.md "optics" pins the
// model exact here (chl -> 0, SPM -> 0, Kd490 -> Kdw(490)), so an unretrieved texel renders as
// the clearest water there is rather than as a hole. The chl/SPM values are the retrievals'
// own valid_min floors -- the smallest quantity either product is willing to assert.
constexpr float kOcPureChlLog10 = -3.0f;    // 0.001 mg/m^3
constexpr float kOcPureKd490 = 0.0224f;     // m^-1, pure seawater at the 490 nm anchor
constexpr float kOcPureSpmLog10 = -2.0f;    // 0.01 mg/L
constexpr float kOcDeepGain = 2.0331f;      // g, the declared albedo gain (proofs/water_optics.py)

void CubeDirD(int face, double u, double v, double out[3]) {
    const double cx = u * 2.0 - 1.0, cy = v * 2.0 - 1.0;
    double p[3];
    switch (face) {
        case 0: p[0] = 1;   p[1] = cy;  p[2] = -cx; break;
        case 1: p[0] = -1;  p[1] = cy;  p[2] = cx;  break;
        case 2: p[0] = cx;  p[1] = 1;   p[2] = -cy; break;
        case 3: p[0] = cx;  p[1] = -1;  p[2] = cy;  break;
        case 4: p[0] = cx;  p[1] = cy;  p[2] = 1;   break;
        default: p[0] = -cx; p[1] = cy; p[2] = -1;  break;
    }
    const double len = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    out[0] = p[0] / len;
    out[1] = p[1] / len;
    out[2] = p[2] / len;
}

}  // namespace

// M10: THE ADDRESS AND THE PLACE. A leaf (face, level, ix, iy) of the walk's own quadtree --
// the node the Droste link hangs from -- and its inverse, in CubeDirD's face convention (NOT the
// hardware cube's: LeafWants flips v for the tenants, the walk never does).
void GlobeLayer::LeafDir(int face, int level, uint32_t ix, uint32_t iy, double out[3]) {
    const double n = double(1u << level);
    CubeDirD(face, (double(ix) + 0.5) / n, (double(iy) + 0.5) / n, out);
}

void GlobeLayer::LeafOf(const double d[3], int level, int& face, uint32_t& ix, uint32_t& iy) {
    const double ax = std::abs(d[0]), ay = std::abs(d[1]), az = std::abs(d[2]);
    double cx = 0.0, cy = 0.0;
    if (ax >= ay && ax >= az) {
        face = d[0] > 0.0 ? 0 : 1;
        cy = d[1] / ax;
        cx = (d[0] > 0.0 ? -d[2] : d[2]) / ax;
    } else if (ay >= az) {
        face = d[1] > 0.0 ? 2 : 3;
        cx = d[0] / ay;
        cy = (d[1] > 0.0 ? -d[2] : d[2]) / ay;
    } else {
        face = d[2] > 0.0 ? 4 : 5;
        cx = (d[2] > 0.0 ? d[0] : -d[0]) / az;
        cy = d[1] / az;
    }
    const double n = double(1u << level);
    const double u = (cx + 1.0) * 0.5, v = (cy + 1.0) * 0.5;
    ix = static_cast<uint32_t>(std::clamp(std::floor(u * n), 0.0, n - 1.0));
    iy = static_cast<uint32_t>(std::clamp(std::floor(v * n), 0.0, n - 1.0));
}

void GlobeLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, hal::RootSignature rootSig) {
    m_rootSig = rootSig;
    if (!m_globe || !m_globe->Ready()) throw std::runtime_error("GlobeLayer needs globe data");
    if (!BuildPso(gpu, sc)) throw std::runtime_error("globe PSO failed");

    // M6j: the unified mesh-shader surface (orbit to helm, one pipeline). Falls back to the
    // classic VS path if the device or compile says no.
    if (msSurface && BuildMeshPso(gpu, sc)) {
        for (uint32_t i = 0; i < Gpu::kFrameCount; ++i) {
            m_recBuf[i] = gpu.CreateUploadBuffer(
                static_cast<uint64_t>(kMaxMeshlets) * sizeof(MeshletRec),
                L"globe.meshlets (per-frame records)");
        }
        m_msPath = true;
        Log("[globe] mesh-shader surface ACTIVE (unified orbit-to-helm)");
    } else if (msSurface) {
        Log("[globe] mesh-shader surface unavailable; classic VS path");
    }

    // M6i: no committed relief texture any more -- the composed height cube (ETOPO + NE 15s +
    // CUDEM, or MOLA) streams the same data through the residency manager, painted once and
    // cached. The CPU-side equirect grid stays in GlobeModel for picking and camera clamps.
    const int nx = m_globe->Nx(), ny = m_globe->Ny();
    if (m_globe->WavesNx() > 0 && !m_globe->Hs().empty()) {
        const int wn = m_globe->WavesNx(), wm = m_globe->WavesNy();
        // The GFS grids are NODE-centred on 0..360 -- stated here rather than assumed, because
        // ocean colour below is cell-centred on -180..180 and the two differ by half a texel.
        // gfswave writes -1 for land, which is a real answer and rides through as the nodata
        // fill so Globe.hlsl's `hsS >= 0` test keeps working unchanged.
        GeoRef wref = GeoRef::Declared(4326, CrsKind::Geographic, m_globe->WavesLon1(),
                                       m_globe->WavesLat1(), m_globe->WavesDLon(),
                                       -m_globe->WavesDLat(), uint32_t(wn), uint32_t(wm));
        wref.centers = false;
        wref.hasNoData = true;
        wref.noData = -1.0f;
        wref.valueUnit = "m";
        BuildPlaneBank(gpu, m_hsB, "globe.hs (gfswave)", "equirect float32 global", wref,
                       {{&m_globe->Hs(), -0.5f, -1.0f}}, DXGI_FORMAT_R32_FLOAT, -1.0f);
        if (!m_globe->Wind().empty()) {
            GeoRef uref = wref;
            uref.valueUnit = "m/s";
            BuildPlaneBank(gpu, m_windB, "globe.wind (gfswave)", "equirect float32 global", uref,
                           {{&m_globe->Wind(), -0.5f, -1.0f}}, DXGI_FORMAT_R32_FLOAT, -1.0f);
        }
    }
    // M9 (docs/ALGEBRA.md "optics"): the water-quality plane. Three retrievals ride up as ONE
    // RGBA texture -- (log10 chl, Kd490, log10 SPM, retrieved?) -- because the optics model
    // consumes them together and one fetch beats three. The log encoding is NOT about fp16
    // range (measured: it does not bite); it is about the FILTER: ocean colour spans four
    // decades inside one bilinear footprint at a coastal front, and linear-space blending
    // there returns an arithmetic mean that paints a bloom the data does not contain.
    //
    // Where nothing was retrieved (polar night, the gap fill's own edges) the texel carries
    // the PURE-WATER limit with alpha 0, so the hardware lerp degrades smoothly toward clear
    // water at the data's edge instead of toward a sentinel. The proof pins that limit exact,
    // so an absent field is a zero-regression fallback, not a hole. (Climatology is the named
    // upgrade for that texel; pure water is what v1 declares.)
    if (m_globe->OcNx() > 0 && !m_globe->OcChlLog10().empty()) {
        const int on = m_globe->OcNx(), om = m_globe->OcNy();
        // M9q: the hand-rolled interleave retired. Packing four planes is composition, and
        // doing it at the call site hard-coded the channel order where nothing declared it --
        // MemGridLoader now states each plane and its own sentinel. The pure-water fallback is
        // preserved exactly: an unretrieved texel carries the pure-water limit with alpha 0, so
        // the hardware lerp still degrades toward clear water at the data's edge.
        //
        // Cell-centred on -180..180, unlike the GFS grids above. Half a texel, stated.
        const float nul = m_globe->OcNull();
        GeoRef oref = GeoRef::Declared(4326, CrsKind::Geographic, m_globe->OcLon1(),
                                       m_globe->OcLat1(), m_globe->OcDLon(),
                                       -m_globe->OcDLat(), uint32_t(on), uint32_t(om));
        oref.centers = true;
        oref.hasNoData = true;
        oref.noData = nul;
        // Four channels that mean four different things (log10 chl, m^-1, log10 SPM, a flag),
        // so the product is declared dimensionless: there is exactly one source, nothing to
        // blend it against, and claiming a single physical unit for the pack would be a lie.
        oref.valueUnit = "1";
        BuildPlaneBank(gpu, m_oceanB, "globe.ocean (chl/Kd490/SPM/valid)",
                       "equirect float32 global, 4 packed retrievals", oref,
                       {{&m_globe->OcChlLog10(), nul, kOcPureChlLog10},
                        {&m_globe->OcKd490(), nul, kOcPureKd490},
                        {&m_globe->OcSpmLog10(), nul, kOcPureSpmLog10},
                        MemGridLoader::Plane{nullptr, 0.0f, 0.0f, true}},
                       DXGI_FORMAT_R32G32B32A32_FLOAT, 0.0f);
        Log("[globe] water optics %dx%d (%s)", on, om, m_globe->OcEpoch().c_str());
    }
    if (!m_globe->Ice().empty()) {
        const int wn = m_globe->WavesNx(), wm = m_globe->WavesNy();
        GeoRef iref = GeoRef::Declared(4326, CrsKind::Geographic, m_globe->WavesLon1(),
                                       m_globe->WavesLat1(), m_globe->WavesDLon(),
                                       -m_globe->WavesDLat(), uint32_t(wn), uint32_t(wm));
        iref.centers = false;
        iref.valueUnit = "fraction";
        BuildPlaneBank(gpu, m_iceB, "globe.ice (gfs icec)", "equirect float32 global", iref,
                       {{&m_globe->Ice(), -1e30f, 0.0f}}, DXGI_FORMAT_R32_FLOAT, 0.0f);
    }
    if (m_globe->CloudsNx() > 0) InitClouds(gpu, sc);
    InitNeAndWind(gpu, sc);

    Log("[globe] layer ready (relief %dx%d via composed height cube, waves %s, clouds %s)",
        nx, ny, m_hsB.Valid() ? m_globe->WavesCycle().c_str() : "absent",
        m_cloudReady ? m_globe->CloudsCycle().c_str() : "absent");
}

// M6c: the sky volume. GFS isobaric cloud fraction rides up as a small dense 3D source
// texture; the CPU decides residency (a tile with no cloud in its footprint stays NULL = the
// hardware's clear air); one list-driven build pass writes density into the resident tiles.
void GlobeLayer::InitClouds(Gpu& gpu, ShaderCompiler& sc) {
    const int snx = m_globe->CloudsNx(), sny = m_globe->CloudsNy(), snz = m_globe->CloudsNz();
    const auto& src = m_globe->Clouds();
    const auto& alts = m_globe->CloudAltsM();
    if (alts.size() < static_cast<size_t>(snz)) return;

    // ---- dense source volume (720x361x10 R32F, ~10 MB)
    {
        m_cloudSrc.res = hal::Committed3D(
            gpu, L"globe.cloudSrc (GFS isobaric TCDC)", snx, sny, snz, DXGI_FORMAT_R32_FLOAT,
            D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_COPY_DEST,
            "the clouds' GFS source volume (720x361x10): uploaded once, sampled once by the "
            "build kernel into the sparse bank the frame reads -- dense because it is the "
            "input, not the field");
        m_cloudSrc.state = D3D12_RESOURCE_STATE_COPY_DEST;

        // The footprint query stays raw (the one such site): a copy layout, not a creation,
        // over the resource's own desc as Gpu::UploadTexture does.
        const auto rd = m_cloudSrc.res->GetDesc();
        D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp{};
        UINT numRows = 0;
        UINT64 rowBytes = 0, total = 0;
        gpu.Device()->GetCopyableFootprints(&rd, 0, 1, 0, &fp, &numRows, &rowBytes, &total);
        GpuBuffer staging = gpu.CreateUploadBuffer(total, L"cloudSrc staging");
        for (UINT r = 0; r < numRows; ++r) {
            memcpy(staging.cpu + fp.Offset + static_cast<uint64_t>(r) * fp.Footprint.RowPitch,
                   reinterpret_cast<const uint8_t*>(src.data()) +
                       static_cast<uint64_t>(r) * rowBytes,
                   static_cast<size_t>(rowBytes));
        }
        hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
        D3D12_TEXTURE_COPY_LOCATION dst{}, sl{};
        dst.pResource = m_cloudSrc.res.Get();
        dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        sl.pResource = staging.res.Get();
        sl.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
        sl.PlacedFootprint = fp;
        up.Native()->CopyTextureRegion(&dst, 0, 0, 0, &sl, nullptr);
        up.Barrier(m_cloudSrc.res.Get(), D3D12_RESOURCE_STATE_COPY_DEST,
                   D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        gpu.EndUpload();
        m_cloudSrc.state = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
    }

    // ---- the sparse bank + residency from the CPU field
    m_cloud.Init(gpu, kVolNx, kVolNy, kVolNz, DXGI_FORMAT_R16_FLOAT,
                 L"globe.cloud (sparse VOLUME bank: NULL tile = clear air)");
    const float dAlt = kShellTopM / kVolNz;
    for (uint32_t tz = 0; tz < m_cloud.TilesZ(); ++tz) {
        const float altLo = tz * m_cloud.TileD() * dAlt - 1800.0f;   // Gaussian slab reach
        const float altHi = (tz + 1) * m_cloud.TileD() * dAlt + 1800.0f;
        for (uint32_t ty = 0; ty < m_cloud.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < m_cloud.TilesX(); ++tx) {
                const int sx0 = tx * m_cloud.TileW() * snx / kVolNx;
                const int sx1 = ((tx + 1) * m_cloud.TileW() * snx + kVolNx - 1) / kVolNx;
                const int sy0 = ty * m_cloud.TileH() * sny / kVolNy;
                const int sy1 = ((ty + 1) * m_cloud.TileH() * sny + kVolNy - 1) / kVolNy;
                bool cloudy = false;
                for (int L = 0; L < snz && !cloudy; ++L) {
                    if (alts[L] < altLo || alts[L] > altHi) continue;
                    const size_t base = static_cast<size_t>(L) * snx * sny;
                    for (int y = sy0; y < sy1 && !cloudy; ++y) {
                        for (int x = sx0; x < sx1; ++x) {
                            if (src[base + static_cast<size_t>(y) * snx + x] > 12.0f) {
                                cloudy = true;
                                break;
                            }
                        }
                    }
                }
                if (cloudy) m_cloud.RequestMap(tx, ty, tz);
            }
        }
    }
    std::vector<uint32_t> fresh;
    m_cloud.CommitMappings(gpu, &fresh);
    Log("[globe] cloud residency: %u/%u tiles (%.0f/%.0f MB) -- the rest IS clear air",
        m_cloud.ResidentCount(), m_cloud.TilesX() * m_cloud.TilesY() * m_cloud.TilesZ(),
        m_cloud.ResidentBytes() / 1048576.0, m_cloud.VirtualBytes() / 1048576.0);

    // ---- build kernel (b0 CBV, t0 list root SRV, table [t1 src, u0 vol], s0 clamp)
    m_cloudRs = hal::RootLayout{}
                    .Cbv(0)
                    .Srv(0)
                    .Table({hal::SrvRange(1, 1), hal::UavRange(0, 1)})
                    .Sampler(hal::StaticSampler(0, D3D12_FILTER_MIN_MAG_MIP_LINEAR,
                                                D3D12_TEXTURE_ADDRESS_MODE_CLAMP))
                    .Build(gpu, "globe.cloud");
    m_cloudBuild = hal::Require(
        hal::BuildCompute(gpu, m_cloudRs.Get(),
                          sc.Compile(m_shaderDir + L"/CloudVol.hlsl", L"CsCloudBuild", L"cs_6_0"),
                          "globe.cloud"),
        "CloudVol kernel");

    m_cloudTable = hal::Table::Alloc(gpu, 2, "globe.cloud");
    m_cloudTable.Srv3D(0, m_cloudSrc.res.Get(), DXGI_FORMAT_R32_FLOAT);
    m_cloudTable.Uav3D(1, m_cloud.Res(), DXGI_FORMAT_R16_FLOAT);

    // ---- one build pass (clouds are static per forecast cycle)
    {
        CloudCbData cb{};
        cb.volNx = kVolNx;
        cb.volNy = kVolNy;
        cb.volNz = kVolNz;
        cb.listCount = m_cloud.ResidentCount();
        cb.tilesX = m_cloud.TilesX();
        cb.tilesY = m_cloud.TilesY();
        cb.tileW = m_cloud.TileW();
        cb.tileH = m_cloud.TileH();
        cb.tileD = m_cloud.TileD();
        cb.srcNx = static_cast<uint32_t>(snx);
        cb.srcNy = static_cast<uint32_t>(sny);
        cb.srcNz = static_cast<uint32_t>(snz);
        for (int i = 0; i < 4; ++i) cb.altA[i] = alts[i];
        for (int i = 0; i < 4; ++i) cb.altB[i] = alts[4 + i];
        cb.altC[0] = alts[8];
        cb.altC[1] = alts[9];
        cb.altC[2] = kShellTopM;
        cb.altC[3] = 1.0f;    // density gamma (linear: GFS fraction is already conservative)

        hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
        up.BindHeaps();   // the upload list has no heap bound until a layer says so
        up.ComputeRoot(m_cloudRs.Get());
        up.ComputeConstants(0, cb);
        const auto& list = m_cloud.ResidentList();
        up.ComputeSrvAt(1, gpu.PushConstants(list.data(), list.size() * 4));
        up.ComputeTable(2, m_cloudTable.Base());
        up.Pipeline(m_cloudBuild.Get());
        up.Dispatch(m_cloud.TileW() / 8, m_cloud.TileH() / 8, cb.listCount);
        up.Barrier(m_cloud.Res(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                   D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                       D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        gpu.EndUpload();
        gpu.ResetConstantArenaAfterIdle();
    }
    m_cloudReady = true;
}

// M6d: the sparse Mv2 wind bank. (The NE 15s relief window that used to load here is a LAYER
// in the composed earth.height stack now -- painted at compose time, not blended per pixel.)
void GlobeLayer::ApplyWindDemand(Gpu& gpu, const std::vector<uint8_t>& derived, uint32_t dTx,
                                 uint32_t dTy) {
    const uint32_t tX = m_windBank.TilesX(), tY = m_windBank.TilesY();
    if (m_windPhys.empty() || dTx != tX || dTy != tY || derived.size() != size_t(tX) * tY) {
        Log("[globe] wind demand: signature grid %ux%u != bank tiles %ux%u -- closure not "
            "applied (a demand on the wrong lattice is worse than none)",
            dTx, dTy, tX, tY);
        return;
    }
    uint32_t alg = 0, phys = 0, both = 0;
    for (uint32_t ty = 0; ty < tY; ++ty) {
        for (uint32_t tx = 0; tx < tX; ++tx) {
            const size_t i = size_t(ty) * tX + tx;
            const bool a = derived[i] != 0, p = m_windPhys[i] != 0;
            alg += a ? 1u : 0u;
            phys += p ? 1u : 0u;
            const bool want = a && p;
            both += want ? 1u : 0u;
            if (want && !m_windBank.IsResident(tx, ty)) m_windBank.RequestMap(tx, ty);
            if (!want && m_windBank.IsResident(tx, ty)) m_windBank.RequestUnmap(tx, ty);
        }
    }
    std::vector<uint32_t> fresh;
    m_windBank.CommitMappings(gpu, &fresh);
    Log("[globe] wind Mv2 residency DRIVEN: algebra %u, physics %u, resident %u of %u tiles",
        alg, phys, both, tX * tY);
    if (alg == tX * tY) {
        Log("[globe]   note: the closure demands EVERY tile here. At 6 tiles across a planet "
            "one tile spans 60 deg, and wind is non-zero somewhere in all of them -- the "
            "bound is correct and uninformative. Sparsity needs a finer signature lattice, "
            "not a better closure.");
    }
}

void GlobeLayer::InitNeAndWind(Gpu& gpu, ShaderCompiler& sc) {
    if (m_globe->WindNx() <= 0 || m_globe->WindU().empty()) return;

    const int wn = m_globe->WindNx(), wm = m_globe->WindNy();
    {
        // M9r: the last committed plane. The hand interleave of u and v is gone -- packing two
        // planes is a LOAD concern, and MemGridLoader states them. GFS 10 m wind is node-centred
        // like the wave grids.
        //
        // No state transition here any more, and that is exactly why this one was left until
        // last: a bank's resource lives in UNORDERED_ACCESS, so the kernel reads it as a UAV.
        // Keeping the old SRV binding would have meant transitioning against the bank's own
        // tracking on every upload.
        GeoRef vref = GeoRef::Declared(4326, CrsKind::Geographic, m_globe->WindLon1(),
                                       m_globe->WindLat1(), m_globe->WindDLon(),
                                       -m_globe->WindDLat(), uint32_t(wn), uint32_t(wm));
        vref.centers = false;
        vref.valueUnit = "m/s";
        BuildPlaneBank(gpu, m_windSrcB, "globe.windSrc (GFS 10 m u,v)",
                       "equirect float32 global, u/v", vref,
                       {{&m_globe->WindU(), -1e30f, 0.0f}, {&m_globe->WindV(), -1e30f, 0.0f}},
                       DXGI_FORMAT_R32G32_FLOAT, 0.0f);
    }

    m_windBank.Init(gpu, wn, wm, DXGI_FORMAT_R16G16B16A16_FLOAT,
                    L"globe.windMv2 (sparse: resident where storms live)");
    m_windPhys.assign(size_t(m_windBank.TilesX()) * m_windBank.TilesY(), 0);

    // Residency from a CPU curl estimate: the sparsity pattern IS the weather.
    const auto& U = m_globe->WindU();
    const auto& V = m_globe->WindV();
    const double dLat = std::abs(m_globe->WindDLat());
    const double dyM = GlobeModel::kR * dLat * 3.14159265358979 / 180.0;
    uint32_t resident = 0;
    for (uint32_t ty = 0; ty < m_windBank.TilesY(); ++ty) {
        for (uint32_t tx = 0; tx < m_windBank.TilesX(); ++tx) {
            bool active = false;
            const uint32_t y1 = (std::min)((ty + 1) * m_windBank.TileH(),
                                           static_cast<uint32_t>(wm));
            const uint32_t x1 = (std::min)((tx + 1) * m_windBank.TileW(),
                                           static_cast<uint32_t>(wn));
            for (uint32_t y = ty * m_windBank.TileH() + 1; y + 1 < y1 && !active; y += 2) {
                const double latDeg = m_globe->WindLat1() - (y + 0.5) * dLat;
                const double dxM =
                    dyM * (std::max)(std::cos(latDeg * 3.14159265358979 / 180.0), 0.05);
                for (uint32_t x = tx * m_windBank.TileW(); x < x1; x += 2) {
                    const size_t i = static_cast<size_t>(y) * wn + x;
                    const uint32_t xm = (x + wn - 1) % wn, xp = (x + 1) % wn;
                    const double curl =
                        (V[static_cast<size_t>(y) * wn + xp] -
                         V[static_cast<size_t>(y) * wn + xm]) / (2.0 * dxM) -
                        (U[i - wn] - U[i + wn]) / (2.0 * dyM);
                    const double spd = std::sqrt(U[i] * U[i] + V[i] * V[i]);
                    if (std::abs(curl) > 6.0e-5 || spd > 17.0) {
                        active = true;
                        break;
                    }
                }
            }
            if (active) {
                m_windBank.RequestMap(tx, ty);
                ++resident;
            }
            // M9h: keep the physics verdict. The Cayley closure arrives later (it needs the
            // published signatures) and is AND-ed with this -- algebra bounds, physics tightens.
            m_windPhys[size_t(ty) * m_windBank.TilesX() + tx] = active ? 1u : 0u;
        }
    }
    std::vector<uint32_t> fresh;
    m_windBank.CommitMappings(gpu, &fresh);
    Log("[globe] wind Mv2 residency: %u/%u tiles -- calm air stays NULL", resident,
        m_windBank.TilesX() * m_windBank.TilesY());

    // Build root signature + kernel (b0 CBV, t0 list, table [u1 src, u0 bank]; M9r: the source
    // is a bank now, so both are UAVs).
    m_windRs = hal::RootLayout{}
                   .Cbv(0)
                   .Srv(0)
                   .Table({hal::UavRange(1, 1), hal::UavRange(0, 1)})
                   .Build(gpu, "globe.wind");
    m_windBuild = hal::Require(
        hal::BuildCompute(gpu, m_windRs.Get(),
                          sc.Compile(m_shaderDir + L"/GlobeWind.hlsl", L"CsWindGrad", L"cs_6_0"),
                          "globe.wind"),
        "GlobeWind kernel");

    m_windTable = hal::Table::Alloc(gpu, 2, "globe.wind");
    // A TEXTURE2D UAV over the bank's slice 0 -- the same drop-in trick as the SRV, in the
    // state the bank already holds.
    m_windTable.Uav2D(0, m_windSrcB.bank->Res(), DXGI_FORMAT_R32G32_FLOAT);
    m_windTable.Uav2D(1, m_windBank.Res(), DXGI_FORMAT_R16G16B16A16_FLOAT);

    // One-shot build (static per forecast cycle).
    {
        WindCbData cb{};
        cb.nx = static_cast<uint32_t>(wn);
        cb.ny = static_cast<uint32_t>(wm);
        cb.listCount = m_windBank.ResidentCount();
        cb.tilesX = m_windBank.TilesX();
        cb.tileW = m_windBank.TileW();
        cb.tileH = m_windBank.TileH();
        cb.lat1 = static_cast<float>(m_globe->WindLat1());
        cb.dLatDeg = static_cast<float>(dLat);
        cb.radius = static_cast<float>(GlobeModel::kR);
        cb.scale = 1.0e4f;

        hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
        up.BindHeaps();
        up.ComputeRoot(m_windRs.Get());
        up.ComputeConstants(0, cb);
        const auto& list = m_windBank.ResidentList();
        up.ComputeSrvAt(1, gpu.PushConstants(list.data(), list.size() * 4));
        up.ComputeTable(2, m_windTable.Base());
        up.Pipeline(m_windBuild.Get());
        up.Dispatch(m_windBank.TileW() / 16, m_windBank.TileH() / 16, cb.listCount);
        up.Barrier(m_windBank.Res(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                   D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                       D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        gpu.EndUpload();
        gpu.ResetConstantArenaAfterIdle();
    }
    m_windReady = true;
}

bool GlobeLayer::BuildPso(Gpu& gpu, ShaderCompiler& sc) {
    const std::wstring path = m_shaderDir + L"/Globe.hlsl";
    ShaderBlob vs = sc.Compile(path, L"VsMain", L"vs_6_0");
    ShaderBlob ps = sc.Compile(path, L"PsMain", L"ps_6_0");
    ShaderBlob vsk = sc.Compile(path, L"VsSky", L"vs_6_0");
    ShaderBlob psk = sc.Compile(path, L"PsSky", L"ps_6_0");
    ShaderBlob psl = sc.Compile(path, L"PsLimb", L"ps_6_0");
    if (!vs.Valid() || !ps.Valid() || !vsk.Valid() || !psk.Valid() || !psl.Valid()) return false;

    // Each pipeline swaps in as it builds and a failure stops the walk: at boot Init makes that
    // fatal, on a reload the pipelines already swapped stand and the rest keep their old ones.

    // M10: THE LIMBS (PsLimb). Light added, the scene behind carried through its own air:
    // dual-source, dst = scatter + dst * T, per channel (Rayleigh dims blue first, so a scalar
    // alpha would grey the sea behind a limb). Depth-tested at the shell's entry -- the shader
    // writes it -- and never written: a limb is air, nothing stands on it.
    {
        hal::GraphicsPipelineDesc limb;
        limb.rootSig = m_rootSig;
        limb.vs = vsk;
        limb.ps = psl;
        limb.blend = true;
        limb.srcBlend = D3D12_BLEND_ONE;
        limb.dstBlend = D3D12_BLEND_SRC1_COLOR;
        limb.srcBlendAlpha = D3D12_BLEND_ZERO;
        limb.dstBlendAlpha = D3D12_BLEND_ONE;
        // depthClip stays FALSE: the depth is the shader's, not VsSky's
        limb.depthTest = true;
        limb.depthFunc = D3D12_COMPARISON_FUNC_GREATER_EQUAL;   // reversed-Z
        if (!hal::Reload(m_limbPso, [&] { return hal::BuildGraphics(gpu, limb, "globe.limb"); },
                         "globe.limb")) {
            return false;
        }
    }

    // The atmosphere backdrop: fullscreen, no depth involvement; the surface overdraws it.
    {
        hal::GraphicsPipelineDesc sky;
        sky.rootSig = m_rootSig;
        sky.vs = vsk;
        sky.ps = psk;
        // M10: the backdrop blends over the sky dome by a factor (skyPassWeight): src W + dst
        // (1 - W). At the shipped W = 1 that is src exactly (dst x 0 = 0), the old overwrite;
        // under appealing Droste lighting the two backdrops cross-fade by the gravity weights,
        // so the frame's re-root -- a gauge change -- cannot pop the sky.
        sky.blend = true;
        sky.srcBlend = D3D12_BLEND_BLEND_FACTOR;
        sky.dstBlend = D3D12_BLEND_INV_BLEND_FACTOR;
        // M6g: the backdrop touches ONLY untouched pixels (VsSky emits z=0 = reversed-Z
        // infinity; cleared depth is 0, drawn geometry is > 0, so GREATER_EQUAL passes only
        // where the frame is still empty). No write: it stays a backdrop.
        sky.depthTest = true;
        sky.depthFunc = D3D12_COMPARISON_FUNC_GREATER_EQUAL;
        if (!hal::Reload(m_skyPso, [&] { return hal::BuildGraphics(gpu, sky, "globe.sky"); },
                         "globe.sky")) {
            return false;
        }
    }

    hal::GraphicsPipelineDesc surf;
    surf.rootSig = m_rootSig;
    surf.vs = vs;
    surf.ps = ps;
    // cull stays NONE: cube faces mix winding
    surf.depthClip = TRUE;
    surf.depthTest = true;
    surf.depthWrite = true;   // reversed-Z GREATER, the default comparison
    if (!hal::Reload(m_pso, [&] { return hal::BuildGraphics(gpu, surf, "globe"); }, "globe")) {
        return false;
    }
    // M9b: wireframe twin of the CDLOD fallback, so --wireframe means the same thing on a
    // machine without mesh shaders -- the same struct with one field changed. Optional.
    auto d = surf.ToDesc();
    d.RasterizerState.FillMode = D3D12_FILL_MODE_WIREFRAME;
    hal::Reload(m_psoWire, [&] { return hal::BuildGraphicsRaw(gpu, d, "globe.wire"); },
                "globe.wire");
    ShaderBlob psM = sc.Compile(path, L"PsMeshlet", L"ps_6_0");
    if (psM.Valid()) {
        d.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
        d.PS = {psM.Data(), psM.Size()};
        hal::Reload(m_psoMeshlet, [&] { return hal::BuildGraphicsRaw(gpu, d, "globe.meshlet"); },
                    "globe.meshlet");
    }
    // The residency lens, compiled only when it is asked for: a run without it compiles and
    // draws exactly what it did before the lens existed.
    if (ResidencyLensOn()) {
        ShaderBlob psL =
            sc.Compile(m_shaderDir + L"/ResidencyLens.hlsl", L"PsResidencyLens", L"ps_6_0");
        if (psL.Valid()) {
            d.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
            d.PS = {psL.Data(), psL.Size()};
            hal::Reload(m_psoLens, [&] { return hal::BuildGraphicsRaw(gpu, d, "globe.lens"); },
                        "globe.lens");
        }
    }
    return true;
}

void GlobeLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);
    if (m_msPath) BuildMeshPso(gpu, sc);
}

bool GlobeLayer::BuildMeshPso(Gpu& gpu, ShaderCompiler& sc) {
    D3D12_FEATURE_DATA_D3D12_OPTIONS7 o7{};
    if (FAILED(gpu.Device()->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &o7,
                                                 sizeof(o7))) ||
        o7.MeshShaderTier == D3D12_MESH_SHADER_TIER_NOT_SUPPORTED) {
        return false;
    }
    // M12 step 3f: the mesh pipeline as one description (hal/Pipeline.h MeshPipelineDesc: the
    // subobject stream is its ToStream, ID3D12Device2 is BuildMesh's to fetch). The stream it
    // lays out was compared byte for byte with the block this replaces, at all four variants,
    // before the block went.
    const std::wstring path = m_shaderDir + L"/GlobeMesh.hlsl";
    hal::MeshPipelineDesc d;
    d.rootSig = m_rootSig;
    d.ms = sc.Compile(path, L"MsMain", L"ms_6_5");
    d.ps = sc.Compile(m_shaderDir + L"/Globe.hlsl", L"PsMain", L"ps_6_5");
    if (!d.ms.Valid() || !d.ps.Valid()) return false;
    // cull stays NONE: cube faces mix winding
    d.depthClip = TRUE;
    d.depthTest = true;
    d.depthWrite = true;   // reversed-Z GREATER, the default comparison
    hal::Pso pso = hal::BuildMesh(gpu, d, "globe.mesh");
    if (!pso) {
        Log("[globe] mesh PSO creation failed");
        return false;
    }
    m_msPso = pso;
    // M9b: the same pipeline in WIREFRAME. Identical shaders and identical displacement --
    // only the raster fill differs -- so what the lines show is exactly the geometry the
    // solid pass rasterizes, meshlet seams and all. A failure here is not fatal: the solid
    // path stands and the toggle simply has nothing to switch to.
    d.fill = D3D12_FILL_MODE_WIREFRAME;
    hal::Pso psoWire = hal::BuildMesh(gpu, d, "globe.mesh.wire");
    if (psoWire) {
        m_msPsoWire = psoWire;
    } else {
        Log("[globe] mesh WIREFRAME PSO creation failed (solid path unaffected)");
    }
    // M9bk: ...and the same wireframe with NO water shading on the lines (--wireflat), so the
    // mesh can be read as geometry instead of through the look it is carrying.
    ShaderBlob psW = sc.Compile(m_shaderDir + L"/Globe.hlsl", L"PsWireFlat", L"ps_6_5");
    if (psW.Valid()) {
        const ShaderBlob psKeep = d.ps;
        d.ps = psW;
        hal::Pso psoWF = hal::BuildMesh(gpu, d, "globe.mesh.wireflat");
        if (psoWF) {
            m_msPsoWireFlat = psoWF;
        } else {
            Log("[globe] mesh FLAT-WIRE PSO creation failed (solid path unaffected)");
        }
        d.ps = psKeep;
    }
    // ...and solid again, with the meshlet-identity pixel shader.
    ShaderBlob psM = sc.Compile(m_shaderDir + L"/Globe.hlsl", L"PsMeshlet", L"ps_6_5");
    if (psM.Valid()) {
        d.fill = D3D12_FILL_MODE_SOLID;
        d.ps = psM;
        hal::Pso psoM = hal::BuildMesh(gpu, d, "globe.mesh.meshlet");
        if (psoM) {
            m_msPsoMeshlet = psoM;
        } else {
            Log("[globe] mesh MESHLET-TINT PSO creation failed (solid path unaffected)");
        }
    }
    // ...and the residency lens (shaders/ResidencyLens.hlsl), only when it is asked for.
    if (ResidencyLensOn()) {
        ShaderBlob psL =
            sc.Compile(m_shaderDir + L"/ResidencyLens.hlsl", L"PsResidencyLens", L"ps_6_5");
        if (psL.Valid()) {
            d.fill = D3D12_FILL_MODE_SOLID;
            d.ps = psL;
            hal::Pso psoL = hal::BuildMesh(gpu, d, "globe.mesh.lens");
            if (psoL) {
                m_msPsoLens = psoL;
            } else {
                Log("[globe] mesh RESIDENCY-LENS PSO creation failed (solid path unaffected)");
            }
        }
    }
    return true;
}

namespace {

// Step 5 (docs/PERF_EXPERIMENT.md): THE NODE WALK AS A PURE FUNCTION OF WalkParams.
//
// SelectNode read a dozen members, and PredictWants ran it a second time by overriding
// m_camPos / m_frustum / m_planeCount in place under a flag. Both walks now run THIS code
// over a WalkParams they cannot reach past: the real walk (SetView -- five planes, a leaf
// that Wants and emits the draw records) and the prefetch walk (the worker thread -- the
// predicted eye, no planes, a leaf that records rects for the main thread to replay). One
// geometry, so the two cannot drift; the same expressions in the same order under
// /fp:precise, so the request stream is the old one to the bit (MEASURED, step 5: the
// predicted stream's FNV-1a identical over the storm rail against the in-place walk; the
// real walk's nodes / leaves / Want() touches per frame unchanged).
using WalkParams = GlobeLayer::WalkParams;

// One node: its span and distance, the horizon and frustum culls, the split test; a leaf
// falls through to leaf(face, u0, v0, size, arc, dist, worlds) -- `worlds` the worlds (bit m for
// wp.worlds[m]) that take it as their leaf, 1 for a walk of one; `active` the worlds still
// walking this subtree.
// M9bk: THE DISTANCE AT WHICH A NODE OF THIS ARC STOPS BEING SPLIT.
//
// The morph band's contract (see the leaf lambda) is "fade this LOD out across the band where
// its PARENT would still be split", and it was written as arc*kLodFactor*{1.35, 1.95} because
// under the distance rule alone the parent stops splitting at exactly 2*arc*kLodFactor -- so
// those constants are 0.675 and 0.975 of the handover, not magic numbers.
//
// The wave-grain rule keeps a node splitting FURTHER OUT than the distance rule does, so the
// handover moved and the band did not follow. MEASURED (storm rail, walk probe): fully-morphed
// leaves went 10% -> 41% when the rule landed. A fully-morphed leaf snaps its odd vertices onto
// its even ones, so it renders at HALF the density the walk just paid for -- the amplification
// was buying geometry the morph then threw away, and the collapsed rows are what read as
// ramps in the wireframe.
//
// This returns the handover for either rule, so the band tracks whichever one is binding.
// THE FIELD'S OWN GRAIN, EXACTLY: THE RING LADDER. The bank's ring m has texel grain * 2^m and
// reaches 256 of its texels from the eye, so the ring that holds range d is the first whose reach
// passes d, and the field there is stored at THAT ring's texel -- a step, grain * 2^m, not the
// d / 256 the rule first took for it. d / 256 is the finest a ring could be at d, true only on a
// ring's outer edge; everywhere else inside ring 1 (305..609 m at the shipped 1.19 m) it asked the
// mesh for cells under the data's texel, and the cells' own power-of-two ladder rounded that down
// to grain: four vertices for every texel of data across the whole annulus. MEASURED (helm,
// --mesh-stats, 2026-10-05): 23,700 of 41,500 records were 1.19 m cells between 150 m and 1 km.
double RingTexel(double grain, double d) {
    double t = grain;
    while (d > 256.0 * t && t < 1.0e7) t *= 2.0;
    return t;
}

double SplitRange(const WalkParams& wp, double arc) {
    double d = arc * GlobeLayer::kLodFactor;
    if (wp.waveGrainM > 0.0f) {
        const double cell = arc / 32.0;
        const double grain =
            static_cast<double>(wp.waveGrainM) / GlobeLayer::kWaveOversample;
        if (cell > grain) {
            // The wave rule splits while cell > max(the ring's texel at the node's near point, the
            // pixel floor). A cell on the ladder is past the ring's texel while its near point,
            // dist - arc / 2, lies within 128 of its own cells (the rings below its own); the
            // pixel floor is dist < cell / (pixAng * floor); the range gate caps both.
            const double ringRange = 128.0 * cell * GlobeLayer::kWaveOversample + 0.5 * arc;
            const double pxRange =
                cell / (static_cast<double>(wp.pixAng) * GlobeLayer::kWavePxFloor);
            const double reach = GlobeLayer::kWaveRings * 256.0 * wp.waveGrainM;
            d = (std::max)(d, (std::min)((std::min)(ringRange, pxRange), reach));
        }
    }
    return d;
}

// Whether a node of this level and arc, `dist` from an eye, is split (WalkNode's rule, said once so
// every world of a shared walk asks it from its own eye).
bool SplitAt(const WalkParams& wp, int level, double arc, double dist) {
    // M6j/M8h: the mesh path walks two rungs past CUDEM scale (level 18 = 1.19 m vertex
    // spacing -- level 16's 4.77 m exactly saturated the old 4.8 m bank ring and could
    // not articulate what a finer ring stores; the user's call: more wave vertices).
    // The fallback VS path keeps its classic depth. The distance split (dist < 3*arc) reaches level 18 only within ~115 m of
    // the eye, so the record budget grows by a few hundred, not thousands. (wp.maxDepth is
    // 18 on the mesh path and kMaxDepth on the fallback -- CaptureWalk.)
    bool split = level < wp.maxDepth && dist < arc * GlobeLayer::kLodFactor;

    // M9bk: THE MESH CARRIES THE FIELD AT THE FIELD'S OWN GRAIN.
    //
    // MEASURED (storm rail frame 1199, --wireframe A/B): the distance rule above holds a node
    // CELL at arc/32 with arc ~ dist/3, i.e. cell ~ dist/96 -- a constant ~10 px. The wave
    // bank stores the surface far finer than that: ring m has texel base*2^m and reach
    // 256 texels, so the ring covering a point at range d has texel ~ max(base, d/256). The
    // mesh was therefore coarser than its own data by ~2.7x EVERYWHERE the bank reaches, and
    // the shortfall is what read as terraces down a storm wave's face -- triangles, not the
    // field (the cubic kernel in Globe.hlsl smoothed the field and left the terraces, which
    // is how we know).
    //
    // So: keep splitting while the cell is coarser than the grain the wave field is actually
    // stored at, floored so a triangle never falls below kWavePxFloor pixels (past that the
    // fold has shed the band into sigma^2 and geometry is buying nothing), and bounded by
    // waveMaxDepth.
    //
    // This is ONE rule, and it is self-gating: beyond the outer ring's reach waveTexel runs
    // past what the distance rule already gives, and from orbit every node is far enough that
    // the pixel floor binds first. A globe view emits exactly the meshlets it emitted before
    // -- the helm is the only place the rule has anything to say, which is the whole point.
    if (!split && wp.waveGrainM > 0.0f && level < wp.waveMaxDepth) {
        // THE RANGE GATE, and why it is not optional. A ring holds 512 texels, so ring m
        // reaches 256*grain*2^m from the eye and the OUTER ring is 9.8 km out. Refining to
        // the ring texel over that whole disc is what blew the record budget -- and, worse,
        // a screen-space floor alone is not a helm rule at all: from orbit every node is far,
        // so a floor finer than the walk's own ~10 px refines the WHOLE PLANET. The user's
        // constraint is explicit -- globe views need none of this.
        //
        // So the rule lives where the bank's fine rings actually are: the first kWaveRings
        // rings around the eye. That is the helm's own neighbourhood by construction (at
        // altitude nothing is within 614 m of the camera, so the rule never fires and the
        // globe walk is untouched, leaf for leaf).
        const double reach = GlobeLayer::kWaveRings * 256.0 * wp.waveGrainM;
        if (dist < reach) {
            const double cell = arc / 32.0;
            // The texel of the ring that holds the node's NEAR point (RingTexel, above): a node
            // reaching into a finer ring is cut at that ring's grain.
            const double waveTexel =
                RingTexel(static_cast<double>(wp.waveGrainM), (std::max)(dist - 0.5 * arc, 0.0)) /
                GlobeLayer::kWaveOversample;
            const double pxFloor = dist * wp.pixAng * GlobeLayer::kWavePxFloor;
            split = cell > (std::max)(waveTexel, pxFloor);
        }
    }
    return split;
}

// A world's horizon as its own walk asks it (WalkNode): its eye in the planet frame, the horizon's
// angle from there, and the margin past it -- 0.02 rad above 10 km; below, 0.10 where `farCull`
// (nothing seen from under 10 km lies 0.1 rad beyond it) and no test otherwise.
void HorizonOf(const double planet[3], double R, bool farCull, WalkParams::World& w) {
    for (int i = 0; i < 3; ++i) w.planet[i] = planet[i];
    w.planetR = std::sqrt(planet[0] * planet[0] + planet[1] * planet[1] + planet[2] * planet[2]);
    w.horizon = std::acos(std::clamp(R / w.planetR, 0.0, 1.0));
    w.margin = w.planetR > R + 10000.0 ? 0.02 : (farCull ? 0.10 : -1.0);
    const double limit = w.horizon + w.margin;
    w.cosLimit = (w.margin >= 0.0 && limit < kPi) ? std::cos(limit) : -1.0;
}

template <class Leaf>
void WalkNode(const WalkParams& wp, uint64_t& nodes, int face, int level, double u0,
              double v0, double size, Leaf& leaf, uint32_t active = 1u) {
    ++nodes;
    const double R = wp.R;
    double dir[3];
    CubeDirD(face, u0 + size * 0.5, v0 + size * 0.5, dir);
    const double arc = (kPi / 2.0) * R / (1 << level);   // ground span of this node, m

    // M6g: node position in the TANGENT frame (doubles; the same frame the camera lives in).
    const double px = dir[0] * R, py = dir[1] * R, pz = dir[2] * R;
    const double tx = wp.frameE[0] * px + wp.frameE[1] * py + wp.frameE[2] * pz;
    const double ty = wp.frameU[0] * px + wp.frameU[1] * py + wp.frameU[2] * pz - R;
    const double tz = wp.frameN[0] * px + wp.frameN[1] * py + wp.frameN[2] * pz;
    const double rel[3] = {tx - wp.camPos[0], ty - wp.camPos[1], tz - wp.camPos[2]};
    const double dist = std::sqrt(rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]);

    // Horizon cull in the planet frame (angles, not dots: both can exceed 90 degrees).
    const double nodeAng = arc * 0.80 / R;   // generous half-diagonal
    uint32_t live = active;
    if (wp.worldCount == 0) {
        const double r =
            std::sqrt(wp.camPlanet[0] * wp.camPlanet[0] + wp.camPlanet[1] * wp.camPlanet[1] +
                      wp.camPlanet[2] * wp.camPlanet[2]);
        if (r > R + 10000.0) {
            const double cosA = (dir[0] * wp.camPlanet[0] + dir[1] * wp.camPlanet[1] +
                                 dir[2] * wp.camPlanet[2]) / r;
            const double ang = std::acos(std::clamp(cosA, -1.0, 1.0));
            const double horizon = std::acos(std::clamp(R / r, 0.0, 1.0));
            if (ang > horizon + nodeAng + 0.02) return;
        } else if (wp.probeCullFar) {
            // Step 23 probe: the same test below 10 km with a 0.1 rad (640 km) margin -- nothing
            // visible from under 10 km lies beyond it, so a pixel this cull changes is a ray that
            // left the shell.
            const double cosA = (dir[0] * wp.camPlanet[0] + dir[1] * wp.camPlanet[1] +
                                 dir[2] * wp.camPlanet[2]) / r;
            const double ang = std::acos(std::clamp(cosA, -1.0, 1.0));
            const double horizon = std::acos(std::clamp(R / r, 0.0, 1.0));
            if (ang > horizon + nodeAng + 0.10) return;
        }
    } else {
        // M13: EACH WORLD'S OWN HORIZON, from its own eye by its own walk's rule (HorizonOf).
        // MEASURED why it cannot be the walk's: asked once from the camera, the corridor's
        // home worlds kept tiles 700-4000 km out that their own walks never drew, and a ray
        // under the near sea (discarded there: that water is the window's) met them -- a
        // 348-pixel strip of far land at the waterline of the second window.
        for (int m = 0; m < wp.worldCount; ++m) {
            const GlobeLayer::WalkParams::World& w = wp.worlds[m];
            if (!(live & (1u << m)) || w.margin < 0.0) continue;
            const double cosA =
                (dir[0] * w.planet[0] + dir[1] * w.planet[1] + dir[2] * w.planet[2]) / w.planetR;
            if (cosA >= w.cosLimit) continue;   // inside the limit, whatever the node's size
            const double ang = std::acos(std::clamp(cosA, -1.0, 1.0));
            if (ang > w.horizon + nodeAng + w.margin) live &= ~(1u << m);
        }
        if (!live) return;
    }

    // Frustum cull: bounding sphere in camera-relative space. Radius covers the node's ground
    // extent, its relief, and the display exaggeration. (planeCount 0 -- the prefetch walk --
    // culls nothing: the predicted view is approximate by nature, M6e.)
    double radius = arc * 0.75 + 9000.0 * (std::max)(1.0f, wp.reliefExagg);
    if (wp.relief && arc < 50000.0) {
        // M10 (WalkParams::relief): the node's own height, and what one ETOPO cell can hide --
        // 150 m of structure, slope over the node's half-span, a tenth of the relief itself.
        const double h = wp.relief->ElevAt(std::asin(std::clamp(dir[1], -1.0, 1.0)) * 57.29577951308232,
                                           std::atan2(dir[2], dir[0]) * 57.29577951308232);
        const double margin = 150.0 + 0.6 * arc + 0.1 * std::abs(h);
        radius = arc * 0.75 + (std::abs(h) + margin) * (std::max)(1.0f, wp.reliefExagg);
    }
    uint32_t seen = 1u;
    if (wp.worldCount > 0) {
        // The windows' own bound: the node's local relief (as `relief` gives the extra levels),
        // when this walk's bound is the planet's worst case.
        double tight = radius;
        if (!wp.relief && wp.worldRelief && arc < 50000.0) {
            const double h = wp.worldRelief->ElevAt(
                std::asin(std::clamp(dir[1], -1.0, 1.0)) * 57.29577951308232,
                std::atan2(dir[2], dir[0]) * 57.29577951308232);
            const double margin = 150.0 + 0.6 * arc + 0.1 * std::abs(h);
            tight = arc * 0.75 + (std::abs(h) + margin) * (std::max)(1.0f, wp.reliefExagg);
        }
        // M13: THE WORLDS' OWN CULLS. A node is walked if any world sees it -- inside its planes,
        // and not wholly inside the cone of the next window it shows (that is a deeper world's).
        seen = 0u;
        for (int m = 0; m < wp.worldCount; ++m) {
            if (!(live & (1u << m))) continue;   // stopped above, or past its horizon
            const GlobeLayer::WalkParams::World& w = wp.worlds[m];
            const double q[3] = {rel[0] - w.off[0], rel[1] - w.off[1], rel[2] - w.off[2]};
            // World 0 of the camera's walk keeps the walk's own bound; a window's world takes
            // the local one. A plane that is not a plane (the reversed-Z camera's near row is
            // NaN) culls nothing, exactly as the single walk's `d < -radius` never did.
            const double r = (m == 0) ? radius : tight;
            bool in = true;
            for (int p = 0; p < w.planeCount && in; ++p) {
                in = !(w.planes[p][0] * q[0] + w.planes[p][1] * q[1] + w.planes[p][2] * q[2] +
                           w.planes[p][3] < -r);
            }
            if (in && w.holeCount > 0) {
                // Wholly behind the next window: every point of the node's bound inside its cone.
                bool hidden = true;
                for (int p = 0; p < w.holeCount && hidden; ++p) {
                    hidden = w.hole[p][0] * q[0] + w.hole[p][1] * q[1] + w.hole[p][2] * q[2] +
                                 w.hole[p][3] >= tight;
                }
                in = !hidden;
            }
            if (in) seen |= 1u << m;
        }
        if (!seen) return;
    } else {
        for (int p = 0; p < wp.planeCount; ++p) {
            const double d = wp.frustum[p][0] * rel[0] + wp.frustum[p][1] * rel[1] +
                             wp.frustum[p][2] * rel[2] + wp.frustum[p][3];
            if (d < -radius) return;
        }
    }
    // M10: THE PLANET YOU STAND ON hides the world it floats in (wp.occ; zero radius = none).
    // Exact and conservative: every ray inside the occluder's silhouette cone meets the sphere
    // no later than the tangent distance sqrt(d^2 - r^2), so a node wholly inside the cone and
    // wholly beyond that distance is behind the sphere or inside it -- unseen either way. The
    // node's bound is the frustum's own (relief headroom included), so nothing that could peek
    // over the limb is ever dropped.
    if (wp.occ[3] > 0.0) {
        const double oc[3] = {wp.occ[0] - wp.camPos[0], wp.occ[1] - wp.camPos[1],
                              wp.occ[2] - wp.camPos[2]};
        const double dO = std::sqrt(oc[0] * oc[0] + oc[1] * oc[1] + oc[2] * oc[2]);
        const double rO = wp.occ[3];
        if (dO > rO && dist > radius && dist - radius >= std::sqrt(dO * dO - rO * rO)) {
            const double alphaO = std::asin(rO / dO);
            const double alphaN = std::asin(radius / dist);
            const double cosB = (oc[0] * rel[0] + oc[1] * rel[1] + oc[2] * rel[2]) / (dO * dist);
            if (std::acos(std::clamp(cosB, -1.0, 1.0)) + alphaN <= alphaO) return;
        }
    }

    // WHERE EACH WORLD STOPS (M13). A walk of one splits as it always did. A walk that draws
    // several worlds asks the same rule of each world from its own eye: a world that needs this
    // node no finer takes it as a leaf here -- its own cut, exactly what its own walk would have
    // drawn -- and the walk goes on down only for the worlds that need more. One traversal, one
    // set of tile requests, and every world at its own grain.
    if (wp.worldCount == 0) {
        if (SplitAt(wp, level, arc, dist)) {
            const double h = size * 0.5;
            WalkNode(wp, nodes, face, level + 1, u0, v0, h, leaf, active);
            WalkNode(wp, nodes, face, level + 1, u0 + h, v0, h, leaf, active);
            WalkNode(wp, nodes, face, level + 1, u0, v0 + h, h, leaf, active);
            WalkNode(wp, nodes, face, level + 1, u0 + h, v0 + h, h, leaf, active);
            return;
        }
        leaf(face, u0, v0, size, arc, dist, seen);
        return;
    }
    uint32_t down = 0u, here = 0u;
    double dHere = 1.0e300;   // the nearest world that stops here: the tile requests' distance
    for (int m = 0; m < wp.worldCount; ++m) {
        if (!(seen & (1u << m))) continue;
        const double* o = wp.worlds[m].off;
        const double q[3] = {rel[0] - o[0], rel[1] - o[1], rel[2] - o[2]};
        const double dm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
        if (SplitAt(wp, level, arc, dm)) {
            down |= 1u << m;
        } else {
            here |= 1u << m;
            dHere = (std::min)(dHere, dm);
        }
    }
    if (here) leaf(face, u0, v0, size, arc, dHere, here);
    if (down) {
        const double h = size * 0.5;
        WalkNode(wp, nodes, face, level + 1, u0, v0, h, leaf, down);
        WalkNode(wp, nodes, face, level + 1, u0 + h, v0, h, leaf, down);
        WalkNode(wp, nodes, face, level + 1, u0, v0 + h, h, leaf, down);
        WalkNode(wp, nodes, face, level + 1, u0 + h, v0 + h, h, leaf, down);
    }
}

// A leaf's wants, in emission order: emit(tenant, face, mip, u0, v0, u1, v1) once per
// Want() the leaf asks for, in the order it asks. The real walk sends each to the manager
// at once; the prefetch walk records them and the main thread replays them.
template <class Emit>
void LeafWants(const WalkParams& wp, int face, double u0, double v0, double size, double arc,
               double dist, uint32_t seen, Emit& emit, GlobeLayer::LeafStats* st = nullptr) {
    // M6e: this leaf's on-screen span decides which streamed-texture mip it WANTS; the
    // residency manager turns wants into loads/mappings on its own budgets. The CDLOD walk IS
    // the sampling feedback -- deterministic, no readback pass (the classic had to render one).
    // M6i: the same rects feed every tenant riding this planet -- Mars's native pyramids and
    // the composed color/height cubes alike (Want clamps to each tenant's own mip count).
    // M9ab: THE MIP FOLLOWS THE NODE'S NEAREST POINT, NOT ITS CENTRE.
    //
    // This is where the descent-boundary seam came from. `dist` is to the node CENTRE, and a
    // node that stopped descending is LARGE -- so its near edge sits far closer than its centre
    // and needs a much finer mip than one centre-based number admits. Across the boundary where
    // the walk stops splitting, the fine side kept asking for detail and the coarse side asked
    // for several levels less, and the residency clamp turned that request gap into a hard line
    // with sharp imagery on one side and mush on the other.
    //
    // Nothing about the LOD TREE changes here: no extra level, no new tier, the same nodes. The
    // node simply asks for the resolution its closest pixel actually needs, and the mip chain
    // plus sparse residency supply the gradient from there -- which is the point of having a
    // mip chain in a reserved resource at all. A discrete level decides WHICH NODE; it should
    // never have been deciding how sharp the node is allowed to be.
    // arc is the node's full ground span, so its nearest point is about half a span closer
    // than its centre. (0.75 is the CULLING radius -- deliberately generous, and using it here
    // over-asked: it bought the same seam fix at double the p99, because every node in the
    // frame requested a level finer than its geometry justifies.)
    const double distNear = (std::max)(dist - arc * 0.5, 1.0);
    // Step 5: the same distance is the want's WEIGHT, the order's fourth key (HIERARCHY 4.19):
    // the nearer to what the reader looks at, the sooner.
    const float nearW = static_cast<float>(distNear);
    if (!wp.wants) return;
    // The node's span in pixels at its nearest point: one number, the same for the cube mip
    // and both window mips below (the walk used to recompute it in each block; the same
    // operands give the same double).
    const double px = arc / (distNear * (std::max)(wp.pixAng, 1e-6f));
    const double texAtMip0 = size * 16384.0;
    const int mip = (std::max)(
        0, static_cast<int>(std::ceil(std::log2((std::max)(texAtMip0 / (std::max)(px, 16.0),
                                                           1.0)))));
    // Node rects live in OUR CubeDir face-uv; texture tiles live in the HARDWARE cube
    // convention. Deriving the two per face collapses to one universal rule: v -> 1 - v.
    const float tu0 = static_cast<float>(u0), tu1 = static_cast<float>(u0 + size);
    const float tv0 = static_cast<float>(1.0 - (v0 + size));
    const float tv1 = static_cast<float>(1.0 - v0);
    const uint32_t f = static_cast<uint32_t>(face);
    const uint32_t m = static_cast<uint32_t>(mip);
    if (wp.surfT >= 0) emit(wp.surfT, f, m, tu0, tv0, tu1, tv1, nearW);
    if (wp.normT >= 0) emit(wp.normT, f, m, tu0, tv0, tu1, tv1, nearW);
    if (wp.colorT >= 0) emit(wp.colorT, f, m, tu0, tv0, tu1, tv1, nearW);
    if (wp.hgtT >= 0) emit(wp.hgtT, f, m, tu0, tv0, tu1, tv1, nearW);
    if (wp.maskT >= 0) emit(wp.maskT, f, m, tu0, tv0, tu1, tv1, nearW);
    if (st) st->cube += (wp.surfT >= 0) + (wp.normT >= 0) + (wp.colorT >= 0) + (wp.hgtT >= 0) + (wp.maskT >= 0);
    // PHASE A2: EVERY WORLD THAT SEES THE LEAF ASKS IT OF ITS OWN WINDOWS (D5): the walk's worlds
    // whose bit is in `seen`, each its slot's slices; a walk of no shared worlds asks for its own.
    const uint32_t worlds = wp.worldCount > 0 ? uint32_t(wp.worldCount) : 1u;
    // F5 (2026-10-04, the corridor's walk): THE ADDRESS IS PROJECTIVE. A corner's texel on a
    // window's face is (s, t) = (d . a, d . b) / (d . n), the face's own; its rung only scales it,
    // X = (s / 2 + 1 / 2) N_rung. So the nine corners are projected ONCE per face the windows
    // stand on, lazily, and every (world, rank) below reads them by a scale and its origin -- the
    // same operations in the same order as FaceWindow::TexelOf, so the texels are the bits they
    // were. Before this each of the corridor's eight worlds re-derived the nine directions and
    // the nine projections at each of its five ranks: 360 per leaf, 72 ms of walk at seven deep.
    double cDir[9][3];
    bool cDirDone = false;
    double cST[6][9][2];
    bool cFaceDone[6] = {}, cBehind[6] = {};
    auto cornersOn = [&](uint32_t f) -> const double (*)[2] {
        if (!cDirDone) {
            if (st) ++st->corners;
            for (int cy = 0; cy < 3; ++cy) {
                for (int cx = 0; cx < 3; ++cx) {
                    CubeDirD(face, u0 + size * cx * 0.5, v0 + size * cy * 0.5, cDir[cy * 3 + cx]);
                }
            }
            cDirDone = true;
        }
        if (!cFaceDone[f]) {
            double bn[3], ba[3], bb[3];
            CubeFaceAxes(f, bn, ba, bb);
            bool behind = false;
            for (int c = 0; c < 9 && !behind; ++c) {
                const double* d = cDir[c];
                const double pn = d[0] * bn[0] + d[1] * bn[1] + d[2] * bn[2];
                if (pn <= 1e-6) {
                    behind = true;
                    break;
                }
                cST[f][c][0] = (d[0] * ba[0] + d[1] * ba[1] + d[2] * ba[2]) / pn;
                cST[f][c][1] = (d[0] * bb[0] + d[1] * bb[1] + d[2] * bb[2]) / pn;
            }
            cBehind[f] = behind;
            cFaceDone[f] = true;
        }
        return cBehind[f] ? nullptr : cST[f];
    };
    // F9: A WINDOW ASKS ONCE FOR THE GROUND ANY OF ITS WORLDS SEES. Worlds at one place read one
    // slice at the ranks where their boxes are one address (SurfaceFrame::Share); the first world
    // that sees the leaf asks it of that slice, the rest find it asked. A bit per slice (40 window
    // slices, from 6) across this leaf's worlds.
    uint64_t asked = 0;
    for (uint32_t wi = 0; wi < worlds; ++wi) {
    if (wp.worldCount > 0 && !(seen & (1u << wi))) continue;
    const uint32_t ws = wp.worldCount > 0 ? wp.worlds[wi].slot : wp.walkSlot;
    if (ws >= WalkParams::kSlots) continue;
    // PHASE A1: THE EYE'S WINDOWS' DEMAND. The node's nine points on the window's face plane, in
    // doubles, as global texels of its rung (FaceWindow::TexelOf about the face's corner); the
    // part inside the box [origin, origin + 16384); the mip the same on-screen texel math against
    // the window's chain, and none past its floor (mip 3: past it the rank above is asked, by the
    // same law); the rectangle placed in the slice MODULO 16384 (Tenant.h's banner), so a box that
    // straddles a multiple of 16384 asks two or four rectangles. A node reaching past that plane's
    // horizon has no projection there and asks nothing of the window.
    for (uint32_t b = 0; b < wp.wnK[ws]; ++b) {
        const uint32_t slice = wp.wnSlice[ws][b];
        const uint64_t sliceBit = slice >= 6u && slice < 70u ? (1ull << (slice - 6u)) : 0ull;
        if (st) ++st->worlds;
        if (asked & sliceBit) {   // this window was asked for the leaf by an earlier world
            if (st) ++st->winAsked;
            continue;
        }
        asked |= sliceBit;
        const uint32_t bf = wp.wnFace[ws][b] < 6 ? wp.wnFace[ws][b] : 5;   // CubeFaceAxes' default
        const double (*cst)[2] = cornersOn(bf);
        if (!cst) {   // the leaf reaches past the face's horizon: no projection there
            if (st) ++st->winBehind;
            continue;
        }
        const double N = std::ldexp(double(Lattice::kFaceDim), wp.wnRung[ws][b]);   // FaceTexels
        double bmin[2] = {1e300, 1e300}, bmax[2] = {-1e300, -1e300};
        for (int c = 0; c < 9; ++c) {
            const double tx = (cst[c][0] * 0.5 + 0.5) * N - 0.0;   // TexelOf, anchored at the corner
            const double ty = (cst[c][1] * 0.5 + 0.5) * N - 0.0;
            bmin[0] = (std::min)(bmin[0], tx - double(wp.wnAx[ws][b]));
            bmax[0] = (std::max)(bmax[0], tx - double(wp.wnAx[ws][b]));
            bmin[1] = (std::min)(bmin[1], ty - double(wp.wnAy[ws][b]));
            bmax[1] = (std::max)(bmax[1], ty - double(wp.wnAy[ws][b]));
        }
        const double dim = double(Lattice::kFaceDim);
        if (bmax[0] <= 0.0 || bmax[1] <= 0.0 || bmin[0] >= dim || bmin[1] >= dim) {
            if (st) ++st->winOut;
            continue;
        }
        const double bspan = (std::max)(bmax[0] - bmin[0], bmax[1] - bmin[1]);   // texels, mip 0
        const int bmip = (std::max)(
            0, static_cast<int>(std::ceil(std::log2((std::max)(bspan / (std::max)(px, 16.0), 1.0)))));
        if (bmip > 3) {
            if (st) ++st->winFloor;
            continue;
        }
        const uint32_t bm = static_cast<uint32_t>(bmip);
        // Per axis the box's part, then that part in the slice: [a, a + len) modulo 16384.
        double lo[2][2], hi[2][2];
        int pieces[2];
        const long long org[2] = {wp.wnAx[ws][b], wp.wnAy[ws][b]};
        for (int ax = 0; ax < 2; ++ax) {
            const double c0 = (std::max)(bmin[ax], 0.0), c1 = (std::min)(bmax[ax], dim);
            const double a = double((org[ax] % Lattice::kFaceDim)) + c0;
            const double s0 = a >= dim ? a - dim : a, s1 = s0 + (c1 - c0);
            pieces[ax] = s1 > dim ? 2 : 1;
            lo[ax][0] = s0 / dim;
            hi[ax][0] = (std::min)(s1, dim) / dim;
            lo[ax][1] = 0.0;
            hi[ax][1] = (s1 - dim) / dim;
        }
        for (int py = 0; py < pieces[1]; ++py) {
            for (int px2 = 0; px2 < pieces[0]; ++px2) {
                const float r0 = static_cast<float>(lo[0][px2]), r1 = static_cast<float>(hi[0][px2]);
                const float q0 = static_cast<float>(lo[1][py]), q1 = static_cast<float>(hi[1][py]);
                // The window's tiles carry the leaf's own distance as their weight, as the cube's do.
                if (wp.colorT >= 0) emit(wp.colorT, slice, bm, r0, q0, r1, q1, nearW);
                if (wp.maskT >= 0) emit(wp.maskT, slice, bm, r0, q0, r1, q1, nearW);
                if (st) st->win += (wp.colorT >= 0) + (wp.maskT >= 0);
                // PHASE B2: the height on the same windows, at the same mip but never finer than its
                // finest read (kCsHeightLodFloor: rung 9), and none past the floor.
                if (wp.hgtT >= 0 && wp.hgtWindows) {
                    const int hm = (std::max)(int(bm), wp.wnRung[ws][b] - 9);
                    if (hm <= 3) emit(wp.hgtT, slice, uint32_t(hm), r0, q0, r1, q1, nearW);
                }
            }
        }
    }
    }   // the worlds
}

}  // namespace

// M6j: one leaf -> 16 mesh-shader records (4x4 sub-meshlets of 8x8 cells). Fine meshlets
// (arc <= 650 m) get a DOUBLE-precision camera-relative anchor + the position Jacobian, so
// vertices reconstruct from small numbers only -- the float wall the estuary layers dodged by
// staying flat falls here, and the one surface reaches walking height.
void GlobeLayer::EmitMeshlets(int face, double u0, double v0, double size, double arc,
                              float morphStart, float morphEnd, const double camPos[3],
                              uint32_t slot) {
    // M8h: >= keeps the record count strictly under the cap (the old > guard let the buffer
    // land exactly on it). A dropped leaf is a HOLE in the surface, so it is counted and
    // reported, never silent. (M10: the cap is 2^17 over a 2-D dispatch -- see kMaxMeshlets.)
    if (m_meshlets.size() + 16 >= kMaxMeshlets) {
        ++m_meshletDrops;
        return;
    }
    const double R = m_radius;
    const double step = size / 32.0;
    auto tangent = [&](const double d[3], double out[3]) {
        const double px = d[0] * R, py = d[1] * R, pz = d[2] * R;
        out[0] = m_surface->east[0] * px + m_surface->east[1] * py + m_surface->east[2] * pz;
        out[1] = m_surface->up[0] * px + m_surface->up[1] * py + m_surface->up[2] * pz - R;
        out[2] = m_surface->north[0] * px + m_surface->north[1] * py + m_surface->north[2] * pz;
    };
    for (int my = 0; my < 4; ++my) {
        for (int mx = 0; mx < 4; ++mx) {
            MeshletRec rec{};
            rec.uv0[0] = static_cast<float>(u0);
            rec.uv0[1] = static_cast<float>(v0);
            rec.uvStepCell[0] = static_cast<float>(step);
            rec.uvStepCell[1] = static_cast<float>(step);
            rec.face = static_cast<uint32_t>(face);
            rec.cell0 = static_cast<uint32_t>(mx * 8) | (static_cast<uint32_t>(my * 8) << 8);
            rec.morphStart = morphStart;
            rec.morphEnd = morphEnd;
            rec.arc = static_cast<float>(arc);
            rec.level = slot;   // M10
            const double cu = u0 + (mx * 8 + 4) * step;
            const double cv = v0 + (my * 8 + 4) * step;
            double dc[3], tc[3];
            CubeDirD(face, cu, cv, dc);
            tangent(dc, tc);
            // M10: relative to THIS level's eye, in its own frame -- the doubles cancel here,
            // so an inner globe a few metres across keeps the same millimetre-stable anchors the
            // helm always had (the gauge moves the precision problem nowhere).
            rec.anchorRel[0] = static_cast<float>(tc[0] - camPos[0]);
            rec.anchorRel[1] = static_cast<float>(tc[1] - camPos[1]);
            rec.anchorRel[2] = static_cast<float>(tc[2] - camPos[2]);
            // Jacobian by central difference in doubles; per face-uv UNIT.
            const double e = (std::max)(step * 0.25, 1e-7);
            double dA[3], dB[3], tA[3], tB[3];
            CubeDirD(face, cu + e, cv, dA);
            CubeDirD(face, cu - e, cv, dB);
            tangent(dA, tA);
            tangent(dB, tB);
            for (int i = 0; i < 3; ++i) {
                rec.dPdu[i] = static_cast<float>((tA[i] - tB[i]) / (2.0 * e));
            }
            CubeDirD(face, cu, cv + e, dA);
            CubeDirD(face, cu, cv - e, dB);
            tangent(dA, tA);
            tangent(dB, tB);
            for (int i = 0; i < 3; ++i) {
                rec.dPdv[i] = static_cast<float>((tA[i] - tB[i]) / (2.0 * e));
            }
            rec.upT[0] = static_cast<float>(m_surface->east[0] * dc[0] +
                                            m_surface->east[1] * dc[1] +
                                            m_surface->east[2] * dc[2]);
            rec.upT[1] = static_cast<float>(m_surface->up[0] * dc[0] +
                                            m_surface->up[1] * dc[1] +
                                            m_surface->up[2] * dc[2]);
            rec.upT[2] = static_cast<float>(m_surface->north[0] * dc[0] +
                                            m_surface->north[1] * dc[1] +
                                            m_surface->north[2] * dc[2]);
            m_meshlets.push_back(rec);
        }
    }
}

// Step 23 (docs/PERF_EXPERIMENT.md): THE SEAM TABLE -- which leaf edges meet a coarser
// leaf, so GlobeMesh.hlsl can close those seams (its level bands; docs/ALGEBRA.md priors
// 28).
//
// MEASURED (--lens shell at key7km, settled): the seven crack pixels through which the
// far side of the planet showed all lie on level seams -- a leaf against a neighbour one
// level coarser. The walk is the only thing that knows the leaf set, so it names those
// neighbours here: per boundary record whose outer edge meets a coarser leaf, that leaf's
// record across the edge and which half of the coarse edge this record meets. A same-level
// neighbour needs no word (the fine path's hairline bands are unconditional on the W and N
// sides, the classic path's same-level seams are coincident by construction), a finer
// neighbour owns the seam from its side, and a neighbour on another cube face is not in
// the table -- face seams stay open (none in the five gate poses; a later item). Only a
// leaf's two OUTER edges (the ones on its parent's boundary) can face a coarser leaf: across
// an inner edge the neighbour is a sibling, and the coarser candidate there is the parent,
// which split. Cost: one sort of ~650 keys and two binary searches per leaf.
namespace {
uint32_t SeamWord(uint32_t rec, uint32_t rel, uint32_t off) {
    // M10: 17 bits of record (the budget is 2^17), then the relation and the half.
    return (rec & 0x1FFFFu) | (rel << 17) | (off << 19);
}
}  // namespace

void GlobeLayer::SeamTable() {
    constexpr uint32_t kCoarser = 1u;
    std::sort(m_leafKeys.begin(), m_leafKeys.end(),
              [](const LeafKey& a, const LeafKey& b) { return a.key < b.key; });
    // M10: the Droste level slot rides the top three bits, so a seam only ever finds a
    // neighbour in its OWN level -- two levels share every address, never a seam.
    auto find = [&](uint64_t slot, int face, int level, int64_t ix, int64_t iy) -> int64_t {
        if (level < 0 || ix < 0 || iy < 0 || ix >= (int64_t(1) << level) ||
            iy >= (int64_t(1) << level)) {
            return -1;
        }
        const uint64_t key = (slot << 61) | (static_cast<uint64_t>(face) << 58) |
                             (static_cast<uint64_t>(level) << 52) |
                             (static_cast<uint64_t>(ix) << 26) | static_cast<uint64_t>(iy);
        const auto it = std::lower_bound(
            m_leafKeys.begin(), m_leafKeys.end(), key,
            [](const LeafKey& a, uint64_t k) { return a.key < k; });
        return (it != m_leafKeys.end() && it->key == key) ? int64_t(it->base) : -1;
    };
    for (const LeafKey& lk : m_leafKeys) {
        const uint64_t slot = lk.key >> 61;
        const int face = static_cast<int>((lk.key >> 58) & 7u);
        const int level = static_cast<int>((lk.key >> 52) & 63u);
        const int64_t ix = static_cast<int64_t>((lk.key >> 26) & ((uint64_t(1) << 26) - 1));
        const int64_t iy = static_cast<int64_t>(lk.key & ((uint64_t(1) << 26) - 1));
        MeshletRec* r = m_meshlets.data() + lk.base;   // r[my * 4 + mx]
        // The W or E seam, whichever is the outer one (ix even: W): this leaf's column mx
        // 0 / 3 against the coarse leaf's column 3 / 0. A coarser neighbour's edge is two
        // leaves long; this leaf is its lower half when iy is even. Our meshlet row my then
        // meets its row 2*half + my/2 at cell offset 4*(my&1). One leaf per place, so a
        // hit at level - 1 is the neighbour and a miss means a same-level or finer one.
        {
            const int side = static_cast<int>(ix & 1);   // 0: W is outer, 1: E is outer
            const int64_t nx = side == 0 ? ix - 1 : ix + 1;
            const int mx = side == 0 ? 0 : 3;
            const int nmx = side == 0 ? 3 : 0;
            const int64_t coarser = find(slot, face, level - 1, nx >> 1, iy >> 1);
            if (coarser >= 0) {
                const int half = static_cast<int>(iy & 1);
                for (int my = 0; my < 4; ++my) {
                    const int cmy = 2 * half + my / 2;
                    r[my * 4 + mx].seamX = SeamWord(
                        static_cast<uint32_t>(coarser) + cmy * 4 + nmx, kCoarser, my & 1);
                }
            }
        }
        // The N or S seam (iy even: N), the same way: row my 0 / 3 against row 3 / 0.
        {
            const int side = static_cast<int>(iy & 1);
            const int64_t ny = side == 0 ? iy - 1 : iy + 1;
            const int my = side == 0 ? 0 : 3;
            const int nmy = side == 0 ? 3 : 0;
            const int64_t coarser = find(slot, face, level - 1, ix >> 1, ny >> 1);
            if (coarser >= 0) {
                const int half = static_cast<int>(ix & 1);
                for (int mx = 0; mx < 4; ++mx) {
                    const int cmx = 2 * half + mx / 2;
                    r[my * 4 + mx].seamY = SeamWord(
                        static_cast<uint32_t>(coarser) + nmy * 4 + cmx, kCoarser, mx & 1);
                }
            }
        }
    }
}

// M12 step 4a: the surface, handed over once at assembly (compose/SurfaceFrame.h). The walk
// and the mip-floor wants read the copies below (CaptureWalk captures them into every
// WalkParams); the tangent frame's rows and the fill are read from the surface itself, because
// the session writes the rows after this call. PHASE B3: the tenants alone; their windows are
// the eye's (SurfaceFrame::bound), read per walk.
void GlobeLayer::SetSurface(const SurfaceFrame* s) {
    m_surface = s;
    m_radius = s->planetR;
    m_maskT = s->maskT;   // M9ay: the survey mask pages (same slices as the colour)
    m_colorT = s->colorT;
    m_hgtT = s->hgtT;
}

// Step 5: everything the node walk reads, captured. SetView fills one for the real walk
// (then adds the five planes); StartPredictWalk fills one for the prefetch walk and moves
// only the eye -- the planet-frame position and the pixel angle stay the REAL camera's,
// exactly as PredictWants left them (it overrode m_camPos and the planes, nothing else).
GlobeLayer::WalkParams GlobeLayer::CaptureWalk(const Camera& cam, float viewportH) const {
    WalkParams wp;
    wp.R = m_radius;
    for (int i = 0; i < 3; ++i) {
        wp.frameE[i] = m_surface->east[i];
        wp.frameU[i] = m_surface->up[i];
        wp.frameN[i] = m_surface->north[i];
    }
    wp.camPos[0] = cam.px;
    wp.camPos[1] = cam.py;
    wp.camPos[2] = cam.pz;
    // M6g: the planet-frame position (doubles) for the horizon test.
    const double ry = m_radius + cam.py;
    for (int i = 0; i < 3; ++i) {
        wp.camPlanet[i] = m_surface->up[i] * ry + m_surface->east[i] * cam.px +
                          m_surface->north[i] * cam.pz;
    }
    // Step 24: the eye's own cube face (face order +x -x +y -y +z -z), named once per frame
    // so the real walk and the prefetch walk emit the same subtree first.
    {
        const double ax = std::abs(wp.camPlanet[0]), ay = std::abs(wp.camPlanet[1]),
                     az = std::abs(wp.camPlanet[2]);
        const int axis = (ax >= ay && ax >= az) ? 0 : (ay >= az ? 1 : 2);
        wp.camFace = axis * 2 + (wp.camPlanet[axis] < 0.0 ? 1 : 0);
    }
    wp.planeCount = 0;
    wp.reliefExagg = reliefExagg;
    wp.maxDepth = m_msPath ? 18 : kMaxDepth;   // M6j/M8h, see WalkNode
    // M9bk: the wave-grain rule rides the mesh path in one-water mode only -- that is where
    // the bank IS the water's geometry. Off elsewhere, so nothing but the helm changes.
    wp.waveGrainM = (m_msPath && m_bankSrv[0] != 0xFFFFFFFFu) ? m_bankBase : 0.0f;
    wp.waveMaxDepth = kWaveMaxDepth;
    wp.pixAng = cam.fovY / (std::max)(viewportH, 1.0f);
    wp.wants = m_res && (m_surfT >= 0 || m_colorT >= 0 || m_hgtT >= 0);
    wp.surfT = m_surfT;
    wp.normT = m_normT;
    wp.colorT = m_colorT;
    wp.hgtT = m_hgtT;
    wp.hgtWindows = m_surface && m_surface->hgtWindows;
    wp.maskT = m_maskT;
    static_assert(WalkParams::kBlocks == SurfaceFrame::kMaxRanks, "a node asks of every rank");
    static_assert(WalkParams::kSlots == SurfaceFrame::kWindowSlots && WalkParams::kSlots == kMaxLevels,
                  "a window set a slot of the level table");
    // PHASE A2: every slot's windows as the tenants are bound to them (SurfaceFrame::bound).
    for (uint32_t s = 0; s < WalkParams::kSlots; ++s) {
        // PHASE A3: the set slot s claimed this frame (SurfaceFrame::Assign).
        const uint32_t set = s < m_surface->slotsLive ? m_surface->slotSet[s] : SurfaceFrame::kNoSet;
        wp.wnSet[s] = set == SurfaceFrame::kNoSet ? 0u : set;
        if (set == SurfaceFrame::kNoSet) {
            wp.wnK[s] = 0;
            continue;
        }
        const SurfaceFrame::EyeWindows& ew = m_surface->bound[set];
        wp.wnK[s] = (std::min)(ew.K, uint32_t(WalkParams::kBlocks));
        for (uint32_t i = 0; i < wp.wnK[s]; ++i) {
            const hal::BlockBinding& b = ew.box[i];
            wp.wnFace[s][i] = b.face;
            wp.wnRung[s][i] = b.rung;
            wp.wnAx[s][i] = static_cast<long long>(b.OrgX());
            wp.wnAy[s][i] = static_cast<long long>(b.OrgY());
            wp.wnSlice[s][i] = m_surface->slice[set][i];
        }
    }
    wp.probeCullFar = probeCullFar;
    return wp;
}

void GlobeLayer::DumpMeshlets(const std::wstring& path) const {
    FILE* f = nullptr;
    if (_wfopen_s(&f, path.c_str(), L"wb") != 0 || !f) {
        Log("[globe] dump-meshlets %S : FAILED (open)", path.c_str());
        return;
    }
    const size_t n = m_meshlets.size();
    const bool ok = fwrite(m_meshlets.data(), sizeof(MeshletRec), n, f) == n;
    fclose(f);
    Log("[globe] dump-meshlets %S : %s (%zu records x %zu B)", path.c_str(),
        ok ? "ok" : "FAILED", n, sizeof(MeshletRec));
}

// M6e screw-prefetch, step 5: the same walk under the PREDICTED pose, posted to the worker.
void GlobeLayer::StartPredictWalk(const Camera& cam, const Camera& pred, float viewportH) {
    WalkParams wp = CaptureWalk(cam, viewportH);
    if (!wp.wants) return;   // PredictWants's own gate: no manager, no cube tenant, no walk
    wp.camPos[0] = pred.px;
    wp.camPos[1] = pred.py;
    wp.camPos[2] = pred.pz;
    // planeCount stays 0: no frustum cull, the predicted view is approximate by nature.
    {
        std::lock_guard<std::mutex> lk(m_predict.mx);
        if (m_predict.outstanding) {
            // A second post before the replay would issue one frame's walk under another
            // frame's stamp; main never does it, so a caller that does is a bug, said loudly.
            throw std::runtime_error("globe: prefetch walk posted over an unreplayed one");
        }
        m_predict.params = wp;
        m_predict.busy = true;
        m_predict.outstanding = true;
    }
    // SUBMITTED OUTSIDE THE LOCK. Under --jobs-inline the walk runs on this very thread, and it
    // takes m_predict.mx at both ends -- a self-deadlock if the post still held it.
    Threads().Submit(Lane::Compute, "globe.predict", [this] { PredictWalk(); });
}

// One posted walk, as a pool job (it was a dedicated thread parked on a condition variable).
// It reads nothing of the layer but the job -- its own copy of the params, the vector it swaps
// in and out under the lock -- because the main thread is inside SetView the whole time.
void GlobeLayer::PredictWalk() {
    WalkParams wp;
    std::vector<WantRect> out;
    {
        std::lock_guard<std::mutex> lk(m_predict.mx);
        wp = m_predict.params;
        out.swap(m_predict.rects);   // the buffer the last replay emptied: its capacity
    }
    {
        const auto t0 = std::chrono::steady_clock::now();
        uint64_t nodes = 0, leaves = 0;
        auto leaf = [&](int face, double u0, double v0, double size, double arc, double dist,
                        uint32_t seen) {
            ++leaves;
            auto emit = [&](int tenant, uint32_t f, uint32_t mip, float u0r, float v0r,
                            float u1r, float v1r, float nearW) {
                out.push_back(WantRect{tenant, f, mip, u0r, v0r, u1r, v1r, nearW});
            };
            LeafWants(wp, face, u0, v0, size, arc, dist, seen, emit, nullptr);
        };
        // Step 24: the same face order as the real walk -- one geometry, one order.
        for (int i = 0; i < 6; ++i) {
            WalkNode(wp, nodes, (wp.camFace + i) % 6, 0, 0.0, 0.0, 1.0, leaf);
        }
        const uint64_t ns = uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::steady_clock::now() - t0).count());
        {
            std::lock_guard<std::mutex> lk(m_predict.mx);
            m_predict.rects.swap(out);
            m_predict.nodes = nodes;
            m_predict.leaves = leaves;
            m_predict.walkNs = ns;
            m_predict.busy = false;
        }
        m_predict.cv.notify_all();
    }
}

// The replay: the rects through Want(predicted = true) in the order the walk emitted them,
// on the main thread, where PredictWants used to run -- after SetView's real wants on the
// same tenants (a predicted touch after a real one changes nothing, Residency.cpp) and
// before ProcessQueues moves the stamp frame. Wait-never-skip.
void GlobeLayer::ReplayPredictWants() {
    std::unique_lock<std::mutex> lk(m_predict.mx);
    if (!m_predict.outstanding) return;
    const auto w0 = std::chrono::steady_clock::now();
    m_predict.cv.wait(lk, [&] { return !m_predict.busy; });
    const auto w1 = std::chrono::steady_clock::now();
    // The worker is idle until the next post, so the rects are ours under the lock.
    for (const WantRect& r : m_predict.rects) {
        m_res->Want(m_sampler, r.tenant, r.face, r.mip, r.u0, r.v0, r.u1, r.v1, true, r.nearM);
    }
    const auto w2 = std::chrono::steady_clock::now();
    ++predictWalks;
    predictNodes += m_predict.nodes;
    predictLeaves += m_predict.leaves;
    predictRects += m_predict.rects.size();
    predictWalkNs += m_predict.walkNs;
    predictWaitNs +=
        uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(w1 - w0).count());
    predictReplayNs +=
        uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(w2 - w1).count());
    m_predict.rects.clear();
    m_predict.outstanding = false;
}

GlobeLayer::~GlobeLayer() {
    // There is no thread to join any more; what has to be true is that no pool job is still
    // inside this object. A walk posted and never replayed is exactly that case.
    std::unique_lock<std::mutex> lk(m_predict.mx);
    m_predict.cv.wait(lk, [&] { return !m_predict.busy; });
}

// One walk of the root's six faces under `wp` -- the camera's own level (slot 0, the walk this
// always was) or an M10 Droste level (its eye, planes and exaggeration already in that level's
// own frame). The real walk's leaf: the wants straight to the manager (this frame's stamp), then
// the draw records. The prefetch walk's leaf (PredictWorker) records rects instead; both run
// WalkNode / LeafWants above.
void GlobeLayer::WalkLevel(const WalkParams& wp, uint32_t slot, int sampler) {
    auto leaf = [&](int face, double u0, double v0, double size, double arc, double dist,
                    uint32_t seen) {
        ++walkLeaves;
        const auto wt0 = std::chrono::steady_clock::now();
        auto emit = [&](int tenant, uint32_t f, uint32_t mip, float u0r, float v0r, float u1r,
                        float v1r, float nearW) {
            m_res->Want(sampler, tenant, f, mip, u0r, v0r, u1r, v1r, false, nearW);
        };
        LeafWants(wp, face, u0, v0, size, arc, dist, seen, emit, &leafStats);
        const auto wt1 = std::chrono::steady_clock::now();
        walkWantNs += uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(wt1 - wt0).count());
        struct MeshClock {   // F19: the leaf's meshlets, clocked to the leaf's end
            const GlobeLayer* g;
            std::chrono::steady_clock::time_point t0;
            ~MeshClock() {
                g->walkMeshNs += uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::steady_clock::now() - t0).count());
            }
        } meshClock{this, wt1};

        // Per-LEVEL morph ramp (identical on both sides of every seam = crack-free): fade this
        // LOD out across the band where its parent would still be split.
        // M9bk: against the PARENT's handover under whichever rule is binding. 0.675/0.975 of
        // it reproduce the old arc*kLodFactor*{1.35, 1.95} exactly when the wave rule is inert,
        // so a globe frame's bands are unchanged to the bit.
        const double dHand = SplitRange(wp, arc * 2.0);
        const float morphStart = static_cast<float>(dHand * 0.675);
        const float morphEnd = static_cast<float>(dHand * 0.975);
        // M9bk PROBE: how many leaves sit PAST their own morph band (k == 1, odd vertices
        // snapped onto even = the level renders at half the resolution the walk paid for)?
        if (dist > morphEnd) ++walkMorphFull;
        else if (dist > morphStart) ++walkMorphPart;
        if (m_msPath) {
            // EVERY WORLD THAT STOPS HERE draws this tile (M13), anchored at its own eye and
            // morphed from it -- the records its own walk would have made. The tile was asked of
            // the residency once, above, for the nearest of them.
            const int worlds = wp.worldCount > 0 ? wp.worldCount : 1;
            size_t first = SIZE_MAX;
            for (int m = 0; m < worlds; ++m) {
                if (!(seen & (1u << m))) continue;
                const uint32_t ws = wp.worldCount > 0 ? wp.worlds[m].slot : slot;
                double eye[3] = {wp.camPos[0], wp.camPos[1], wp.camPos[2]};
                if (wp.worldCount > 0) {
                    for (int k = 0; k < 3; ++k) eye[k] += wp.worlds[m].off[k];
                }
                const size_t base = m_meshlets.size();
                EmitMeshlets(face, u0, v0, size, arc, morphStart, morphEnd, eye, ws);
                if (m_meshlets.size() != base + 16) break;
                if (first == SIZE_MAX) first = base;
                levelRecords[ws] += 16u;
                // Step 23: the seam table's key. size is 2^-level and u0, v0 are multiples of
                // it, so the level and the grid position are exact integers. M10: the Droste
                // slot rides the top bits, so seams never cross levels.
                const int level = static_cast<int>(std::lround(-std::log2(size)));
                const uint64_t ix = static_cast<uint64_t>(std::llround(u0 / size));
                const uint64_t iy = static_cast<uint64_t>(std::llround(v0 / size));
                // M13 step 0: the two water tiles this leaf would read on the cube quadtree --
                // its grain (level - 2, one texel per cell) and the morph's target (level - 3).
                // Addresses only; nothing is mapped or filled -- and a tile is one tile however
                // many worlds draw it, so it is counted for the first.
                if (waterTileCount && base == first) {
                    for (int d = 2; d <= 3; ++d) {
                        const int T = level - d;
                        if (T < 0) continue;
                        const uint64_t tx = ix >> d, ty = iy >> d;
                        const uint64_t key = (static_cast<uint64_t>(ws) << 61) |
                                             (static_cast<uint64_t>(face) << 58) |
                                             (static_cast<uint64_t>(T) << 52) | (tx << 26) | ty;
                        if (waterTiles.insert(key).second) ++waterTilesByLevel[T & 31];
                    }
                }
                m_leafKeys.push_back(LeafKey{(static_cast<uint64_t>(ws) << 61) |
                                                 (static_cast<uint64_t>(face) << 58) |
                                                 (static_cast<uint64_t>(level) << 52) |
                                                 (ix << 26) | iy,
                                             static_cast<uint32_t>(base)});
            }
            return;
        }
        if (slot != 0u) return;   // the VS fallback draws the camera's own level only
        NodeData nd{};
        nd.uv0[0] = static_cast<float>(u0);
        nd.uv0[1] = static_cast<float>(v0);
        nd.uvStep[0] = static_cast<float>(size / 32.0);
        nd.uvStep[1] = static_cast<float>(size / 32.0);
        nd.face = static_cast<uint32_t>(face);
        nd.morphStart = morphStart;
        nd.morphEnd = morphEnd;
        m_nodes.push_back(nd);
    };
    // Step 24 (docs/PERF_EXPERIMENT.md): THE EYE'S OWN CUBE FACE FIRST. The near surface
    // records before the far subtrees, so early-Z rejects their fragments instead of shading
    // them and letting GREATER overwrite. The node set, the want set and the records are the
    // old ones -- only the order they are emitted in changes -- and that is exact only
    // because step 23 closed the shell (docs/ALGEBRA.md priors 27/28: with the cracks open
    // the same rotation moved a pixel). MEASURED: all four settled poses bit-identical
    // across binaries, and [gpu] globe.mesh 3.082 -> 2.718 ms whole-rail, 4.981 -> 4.472
    // over the helm phase, p95 5.297 -> 4.801 (fenced; the overlap bench reads the same
    // -0.53 ms helm and takes the shipped loop 4.73 -> 4.30 ms).
    const uint32_t all = wp.worldCount > 0 ? ((1u << wp.worldCount) - 1u) : 1u;
    for (int i = 0; i < 6; ++i) {
        WalkNode(wp, walkNodes, (wp.camFace + i) % 6, 0, 0.0, 0.0, 1.0, leaf, all);
    }
}

// M12 step 4d instrument: THE FRUSTUM TRANSPORT THROUGH THE CYCLE. Each camera-relative plane
// (n . v = d) PULLED through the level's gauge placement -- step 4d-2, THE READ the walk culls
// by -- beside the hand transport it replaced, n -> Q^T n, d -> d / sigma from the portal's Q
// and sigma, computed for the record (core/Space.h PullPlane:
// n' = R^T n, d' = (d - n . t) / s with t = 0, the linear part of Level(rel): the eye-to-eye
// translation the gauge identity S^k(C_k) = C cancels exactly). Compared bit for bit per plane
// per level (core/Common.h UlpTally; the totals in LogDrosteProbe); the dump of both prints when
// the transport changes (its FNV-1a -- a held still transports one frustum every frame). A level
// under half a pixel is never transported, so a walk from orbit measures nothing. The near plane
// is degenerate on both paths (Camera::Projection's clip.z is the constant nearZ, so plane 4 is
// (0, 0, 0, nearZ) over a zero length: NaN, and it never culls): its d reads inf by hand and NaN
// pulled, the one comparison that is neither EQUAL nor a rounding.
void GlobeLayer::ProbeTransport(const TransportProbeRow* rows, int n) {
    if (n == 0) return;
    ++m_probeWalks;
    uint64_t fp = Fnv1aBytes(&n, sizeof n);
    for (int i = 0; i < n; ++i) {
        const TransportProbeRow& r = rows[i];
        fp = Fnv1aBytes(&r.slot, sizeof r.slot, fp);
        fp = Fnv1aBytes(&r.rel, sizeof r.rel, fp);
        fp = Fnv1aBytes(&r.plane, sizeof r.plane, fp);
        fp = Fnv1aBytes(r.hand, sizeof r.hand, fp);
        fp = Fnv1aBytes(r.pulled, sizeof r.pulled, fp);
    }
    // Distinct walks are counted and compared every time; their planes are PRINTED for the first
    // kProbePrintCap (FrameLoop::ProbeDrosteTable's rule and reason: a moving camera walks a new
    // set every frame). The totals line is over every walk.
    static constexpr uint64_t kProbePrintCap = 16;
    const bool dump = fp != m_probeFp;
    m_probeFp = fp;
    if (dump) ++m_probeDumps;
    const bool print = dump && m_probeDumps <= kProbePrintCap;
    if (print) {
        Log("[droste] transport walk %llu: %d planes over the extra levels -- hand (Q^T n, d / "
            "sigma) | pulled (PullPlane through the level's gauge)",
            static_cast<unsigned long long>(m_probeWalks), n);
    }
    for (int i = 0; i < n; ++i) {
        const TransportProbeRow& r = rows[i];
        const std::string wN = UlpWord(r.hand, r.pulled, 3, m_probeN);
        const std::string wD = UlpWord(&r.hand[3], &r.pulled[3], 1, m_probeD);
        if (!print) continue;
        Log("[droste] transport slot %u (rel %+d) plane %d hand: n %.17g %.17g %.17g d %.17g | "
            "pulled: n %.17g %.17g %.17g d %.17g | n %s d %s",
            r.slot, r.rel, r.plane, r.hand[0], r.hand[1], r.hand[2], r.hand[3], r.pulled[0],
            r.pulled[1], r.pulled[2], r.pulled[3], wN.c_str(), wD.c_str());
    }
}

void GlobeLayer::LogDrosteProbe() const {
    Log("[droste] probe totals: frustum transport %llu walks (%llu distinct): n %s | d %s",
        static_cast<unsigned long long>(m_probeWalks),
        static_cast<unsigned long long>(m_probeDumps), m_probeN.Verdict().c_str(),
        m_probeD.Verdict().c_str());
}

void GlobeLayer::SetView(const Camera& cam, float aspect, float viewportH, double simTime) {
    if (!m_globe || !m_globe->Ready()) return;
    m_viewportH = viewportH;
    // M6g: cam is the FLAT (tangent-frame) camera. Keep it for the meshlet anchors and the
    // constants; the walk reads its own capture (the planet-frame eye for the horizon test,
    // the pixel's angular size for the mip selector -- step 5, CaptureWalk).
    m_camPos[0] = cam.px;
    m_camPos[1] = cam.py;
    m_camPos[2] = cam.pz;
    m_wp = CaptureWalk(cam, viewportH);

    // Frustum planes from the camera-relative view-projection (reversed-Z: use the 4 side
    // planes + near; there is no far plane).
    const XMMATRIX vp = cam.ViewRelative() * cam.Projection(aspect);
    XMFLOAT4X4 m;
    XMStoreFloat4x4(&m, vp);
    auto plane = [&](int idx, double a, double b, double c, double d) {
        const double len = std::sqrt(a * a + b * b + c * c);
        m_wp.frustum[idx][0] = a / len;
        m_wp.frustum[idx][1] = b / len;
        m_wp.frustum[idx][2] = c / len;
        m_wp.frustum[idx][3] = d / len;
    };
    plane(0, m._14 + m._11, m._24 + m._21, m._34 + m._31, m._44 + m._41);   // left
    plane(1, m._14 - m._11, m._24 - m._21, m._34 - m._31, m._44 - m._41);   // right
    plane(2, m._14 + m._12, m._24 + m._22, m._34 + m._32, m._44 + m._42);   // bottom
    plane(3, m._14 - m._12, m._24 - m._22, m._34 - m._32, m._44 - m._42);   // top
    plane(4, m._13, m._23, m._33, m._43);                                    // near (z >= 0)
    m_wp.planeCount = 5;

    m_nodes.clear();
    m_meshlets.clear();
    m_leafKeys.clear();
    m_meshletDrops = 0;
    for (uint32_t& n : levelRecords) n = 0;

    // ---- M13: A PLACE'S TILES ARE CHOSEN ONCE. A gate's world is the camera's own frame at sigma
    // 1, so it is the same planet at the same address as any other world whose eye stands at its
    // place -- the eye's own world, or a depth of a corridor of windows that came back. Those
    // share ONE walk (WalkParams::worlds): its eye is the first of them, a node is walked if any
    // of them sees it, asked of the residency once, and drawn once per world that sees it. A
    // world is said to the walk by its slot, its eye, and its cull -- its own horizon, and its
    // planes pulled through its gauge.
    TransportProbeRow probeRows[kMaxLevels * 6];   // M12 step 4d instrument (ProbeTransport)
    int probeN = 0;
    const bool windowsOn = m_gateFirst > 0 && m_gateCount > 0 && m_msPath;
    const size_t nLv = m_msPath ? m_levels.size() : 0u;
    std::vector<int> drawnBy(nLv, -1);   // the slot of the walk that draws each extra level
    auto within = [](const double a[3], const double b[3]) {
        const double d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
        return d[0] * d[0] + d[1] * d[1] + d[2] * d[2] < kShareReachM * kShareReachM;
    };
    // ...and only a world whose own walk would split and bound a node as this walk does: the wave
    // grain and the relief exaggeration are the walk's, not the world's.
    auto sameRules = [&](const DrosteLevel& M, const WalkParams& walk) {
        const float grain = M.bankSet < 0 ? 0.0f : m_wp.waveGrainM;
        return grain == walk.waveGrainM && M.reliefExagg == walk.reliefExagg;
    };
    // A level's eye in the planet frame (the walk's camPlanet for an extra level).
    auto planetOf = [&](const double cam[3], double out[3]) {
        const double ry = m_radius + cam[1];
        for (int i = 0; i < 3; ++i) {
            out[i] = m_surface->up[i] * ry + m_surface->east[i] * cam[0] +
                     m_surface->north[i] * cam[2];
        }
    };
    auto pullWorld = [&](const DrosteLevel& L, uint32_t slot, const double eye[3],
                         WalkParams::World& w) {
        w = WalkParams::World{};
        w.slot = slot;
        for (int i = 0; i < 3; ++i) w.off[i] = L.cam[i] - eye[i];
        // Its horizon, from its own eye: past it at any altitude, as every extra level culls.
        double planet[3];
        planetOf(L.cam, planet);
        HorizonOf(planet, m_wp.R, true, w);
        // A world seen through windows is walked only along the rays through them (its own
        // cull); every other level along the camera's.
        const bool ownCull = L.planeCount > 0;
        w.planeCount = ownCull ? (std::min)(L.planeCount, 6) : m_wp.planeCount;
        for (int p = 0; p < w.planeCount; ++p) {
            const double* n = ownCull ? L.planes[p] : m_wp.frustum[p];
            // M12 step 4d-2: THE PLANE, PULLED through the level's gauge placement (core/Space.h
            // PullPlane: n' = R^T n, d' = (d - n . t) / s with t = 0 -- the linear part of
            // Level(rel), the eye-to-eye translation the gauge identity cancels exactly). The
            // hand transport it replaces -- n -> Q^T n, d -> d / sigma from the portal's Q and
            // sigma -- is computed beside it for the record only (ProbeTransport).
            L.gauge.PullPlane(n, n[3], w.planes[p], w.planes[p][3]);
            if (probeN < kMaxLevels * 6) {
                TransportProbeRow& pr = probeRows[probeN++];
                pr.slot = slot;
                pr.rel = L.rel;
                pr.plane = p;
                for (int j = 0; j < 3; ++j) {
                    pr.hand[j] = L.Q[0][j] * n[0] + L.Q[1][j] * n[1] + L.Q[2][j] * n[2];
                }
                pr.hand[3] = n[3] / L.sigma;
                for (int j = 0; j < 4; ++j) pr.pulled[j] = w.planes[p][j];
            }
        }
        // ...and what it need not walk: the next window's cone, pulled the same way.
        w.holeCount = (std::min)(L.holeCount, 5);
        for (int p = 0; p < w.holeCount; ++p) {
            L.gauge.PullPlane(L.hole[p], L.hole[p][3], w.hole[p], w.hole[p][3]);
        }
    };

    // The camera's own level: slot 0, the identity gauge -- exactly the walk this always was, and
    // when the view has windows, the walk of every world that stands where the camera does. The
    // eye's own world need not walk what the first window hides.
    m_wp.worldCount = 0;
    if (windowsOn && !m_drosteOn) {
        WalkParams::World& w0 = m_wp.worlds[0];
        w0 = WalkParams::World{};
        HorizonOf(m_wp.camPlanet, m_wp.R, m_wp.probeCullFar, w0);
        w0.planeCount = m_wp.planeCount;
        for (int p = 0; p < m_wp.planeCount; ++p) {
            for (int j = 0; j < 4; ++j) w0.planes[p][j] = m_wp.frustum[p][j];
        }
        w0.holeCount = m_viewHoleCount;
        for (int p = 0; p < m_viewHoleCount; ++p) {
            for (int j = 0; j < 4; ++j) w0.hole[p][j] = m_viewHole[p][j];
        }
        m_wp.worldCount = 1;
        for (size_t li = 0; li < nLv && m_wp.worldCount < WalkParams::kMaxWorlds; ++li) {
            const DrosteLevel& L = m_levels[li];
            if (!L.share || !within(L.cam, m_camPos) || !sameRules(L, m_wp)) continue;
            WalkParams::World& w = m_wp.worlds[m_wp.worldCount++];
            pullWorld(L, static_cast<uint32_t>(li + 1), m_camPos, w);
            drawnBy[li] = 0;
        }
        m_wp.worldRelief = m_globe;
    }
    WalkLevel(m_wp, 0u, m_sampler);
    // ---- M10: THE CYCLE, TAKEN. Each further level is the ROOT walked again under the eye
    // S^-k(C) (Droste.h). Everything the walk reads is scale-free or already in the level's own
    // units: the split rule compares distance to arc, the horizon test is angular, and the
    // frustum's planes carry across as n -> Q^T n, d -> d / s^k (a similarity keeps planes
    // planes). Its wants go to the SAME tenants at the SAME addresses -- the sparse structure
    // answers every level from one resident set, and a small globe asks only for coarse mips
    // the root already holds. Its records carry their slot, so the mesh stage knows which
    // gauge to rasterize them through.
    for (size_t li = 0; li < nLv; ++li) {
        if (drawnBy[li] >= 0) continue;   // another world's walk draws it
        const DrosteLevel& L = m_levels[li];
        const uint32_t slot = static_cast<uint32_t>(li + 1);
        // WHERE THE RECURSION STOPS: a globe under half a pixel has nothing to walk. The
        // screen, not a depth limit, ends the tower (the eye's distance to the level's planet
        // centre, in that level's own units, against the pixel's angle).
        const double ry = m_radius + L.cam[1];
        const double dC = std::sqrt(L.cam[0] * L.cam[0] + ry * ry + L.cam[2] * L.cam[2]);
        const double angR = std::asin((std::min)(m_radius / (std::max)(dC, 1.0), 1.0));
        if (angR < double(m_wp.pixAng) * 0.5) continue;
        WalkParams wp = m_wp;
        wp.worldCount = 0;
        wp.walkSlot = slot;
        for (int i = 0; i < 3; ++i) wp.camPos[i] = L.cam[i];
        planetOf(L.cam, wp.camPlanet);
        {
            const double ax = std::abs(wp.camPlanet[0]), ay = std::abs(wp.camPlanet[1]),
                         az = std::abs(wp.camPlanet[2]);
            const int axis = (ax >= ay && ax >= az) ? 0 : (ay >= az ? 1 : 2);
            wp.camFace = axis * 2 + (wp.camPlanet[axis] < 0.0 ? 1 : 0);
        }
        wp.reliefExagg = L.reliefExagg;
        // The wave-grain rule refines toward the bank's fine rings; a level with no ring set
        // is seen from beyond them, so the rule has nothing to reach for there.
        if (L.bankSet < 0) wp.waveGrainM = 0.0f;
        // A level ABOVE the camera's is the world its planet floats in -- and that planet (the
        // portal sphere, in this level's own frame) hides most of it.
        if (L.rel < 0) {
            for (int i = 0; i < 4; ++i) wp.occ[i] = m_portal[i];
        }
        // Every extra level culls with its LOCAL relief, and past its own horizon at any
        // altitude (the step-23 far test: nothing seen from under 10 km lies 0.1 rad beyond it).
        wp.relief = m_globe;
        wp.probeCullFar = true;
        if (L.share) {
            // A gate's world, and every later one standing at its place: one walk from here.
            pullWorld(L, slot, L.cam, wp.worlds[0]);
            wp.worldCount = 1;
            for (size_t lj = li + 1; lj < nLv && wp.worldCount < WalkParams::kMaxWorlds; ++lj) {
                const DrosteLevel& M = m_levels[lj];
                if (drawnBy[lj] >= 0 || !M.share || !within(M.cam, L.cam) ||
                    !sameRules(M, wp)) {
                    continue;
                }
                WalkParams::World& w = wp.worlds[wp.worldCount++];
                pullWorld(M, static_cast<uint32_t>(lj + 1), L.cam, w);
                drawnBy[lj] = static_cast<int>(slot);
            }
        } else {
            WalkParams::World w;
            pullWorld(L, slot, L.cam, w);
            wp.planeCount = w.planeCount;
            for (int p = 0; p < w.planeCount; ++p) {
                for (int j = 0; j < 4; ++j) wp.frustum[p][j] = w.planes[p][j];
            }
        }
        // M13: an extra level answers for its own tiles on the shared cache when it names a
        // sampler (a gate's window does); one that does not is part of this view's reading.
        WalkLevel(wp, slot, L.sampler >= 0 ? L.sampler : m_sampler);
    }
    ProbeTransport(probeRows, probeN);
    if (m_msPath) SeamTable();
    // M8h: a dropped leaf is a hole. Report on the transition (once per episode), with
    // the count -- the fix is a coarser view or a bigger kMaxMeshlets, not silence.
    if (m_meshletDrops > 0 && !m_dropsReported) {
        Log("[globe] meshlet budget hit: %u leaves dropped (%zu/%u records) -- holes "
            "until the view coarsens",
            m_meshletDrops, m_meshlets.size(), kMaxMeshlets);
        if (!m_levels.empty()) {
            Log("[globe]   records by Droste slot: own %u | %u %u %u %u %u (rel %d %d %d %d %d)",
                levelRecords[0], levelRecords[1], levelRecords[2], levelRecords[3],
                levelRecords[4], levelRecords[5],
                m_levels.size() > 0 ? m_levels[0].rel : 0, m_levels.size() > 1 ? m_levels[1].rel : 0,
                m_levels.size() > 2 ? m_levels[2].rel : 0, m_levels.size() > 3 ? m_levels[3].rel : 0,
                m_levels.size() > 4 ? m_levels[4].rel : 0);
        }
        m_dropsReported = true;
    } else if (m_meshletDrops == 0) {
        m_dropsReported = false;
    }
    if (meshStats && ++m_meshStatWalks == 8) {
        // Log-spaced distance buckets from the eye. Every record carries its own
        // camera-relative anchor and its node's ground span, so cell = arc/32 is the
        // vertex spacing this meshlet actually emits -- no inference from pixels.
        const double edge[9] = {0.0, 10.0, 25.0, 60.0, 150.0, 400.0, 1000.0, 3000.0, 1e30};
        size_t n[8] = {};
        double cmin[8], cmax[8], csum[8] = {};
        for (int b = 0; b < 8; ++b) { cmin[b] = 1e30; cmax[b] = 0.0; }
        for (const MeshletRec& mr : m_meshlets) {
            const double d = std::sqrt(static_cast<double>(mr.anchorRel[0]) * mr.anchorRel[0] +
                                       static_cast<double>(mr.anchorRel[1]) * mr.anchorRel[1] +
                                       static_cast<double>(mr.anchorRel[2]) * mr.anchorRel[2]);
            const double cell = mr.arc / 32.0;
            for (int b = 0; b < 8; ++b) {
                if (d >= edge[b] && d < edge[b + 1]) {
                    ++n[b];
                    csum[b] += cell;
                    cmin[b] = (std::min)(cmin[b], cell);
                    cmax[b] = (std::max)(cmax[b], cell);
                    break;
                }
            }
        }
        Log("[meshstats] %zu records, %u dropped (cap %u). cell = node arc / 32:",
            m_meshlets.size(), m_meshletDrops, kMaxMeshlets);
        for (int b = 0; b < 8; ++b) {
            if (!n[b]) continue;
            Log("[meshstats]   d %6.0f-%-7.0f m  n %6zu  cell mean %7.2f  min %7.2f  "
                "max %7.2f m",
                edge[b], (edge[b + 1] > 1e29) ? 99999.0 : edge[b + 1], n[b],
                csum[b] / n[b], cmin[b], cmax[b]);
        }
    }

    // THE EYE'S HEIGHT IS MEASURED FROM THE PLANET'S CENTRE, which in the flat frame sits at
    // (0, -R, 0) -- not at its origin. Taking |camPos| read true only while the eye stood near
    // the tangent point and went badly wrong anywhere else: a camera carried 1900 km to the
    // Bahamas reported "alt -4214 km" in the title bar. gCamAbs below is this same sum, and the
    // shaders have always had it right; only these two stats lines were reading the wrong length.
    const double ry = m_camPos[1] + m_radius;
    const double r = std::sqrt(m_camPos[0] * m_camPos[0] + ry * ry + m_camPos[2] * m_camPos[2]);
    m_cb.glo[0] = static_cast<float>(m_radius);
    m_cb.glo[1] = reliefExagg;
    // z = the draw's unit count: CDLOD nodes on the fallback, meshlet records on the mesh path
    // (M10: GlobeMesh.hlsl bounds its folded 2-D group id by it). Exact in a float to 2^24.
    m_cb.glo[2] = static_cast<float>(m_msPath ? m_meshlets.size() : m_nodes.size());
    m_cb.glo[3] = static_cast<float>(std::fmod(simTime, 3600.0));
    // M6g: gCamAbs = sphere-CENTRED tangent camera (flat + (0,R,0); the sum in doubles).
    m_cb.camAbs[0] = static_cast<float>(m_camPos[0]);
    m_cb.camAbs[1] = static_cast<float>(m_camPos[1] + m_radius);
    m_cb.camAbs[2] = static_cast<float>(m_camPos[2]);
    m_cb.estGeo[0] = static_cast<float>(m_estGeo[0]);
    m_cb.estGeo[1] = static_cast<float>(m_estGeo[1]);
    m_cb.estGeo[2] =
        m_estGeo[2] > 0.0 ? static_cast<float>(1.0 / m_estGeo[2]) : 0.0f;
    m_cb.estGeo[3] =
        (m_estGeo[3] > 0.0 && !m_streamMars) ? static_cast<float>(1.0 / m_estGeo[3]) : 0.0f;
    // M7: the wave vertex bank rows (one-water mode).
    m_cb.bankU[0] = m_bankSrv[0];
    m_cb.bankU[1] = m_bankSrv[1];
    m_cb.bankU[2] = (m_bankSrv[0] != 0xFFFFFFFFu) ? 1u : 0u;
    m_cb.bankU[3] = 512u;
    m_cb.bankA[0] = m_bankBase;
    m_cb.bankA[1] = 6.0f;
    m_cb.bankA[2] = static_cast<float>(debugLens);
    m_cb.lensU[0] = m_lensSrv;   // M9h: grad(flow) bank for lens 7
    m_cb.lensU[1] = m_lensResMapSrv;
    for (int i = 0; i < 4; ++i) m_cb.lensA[i] = m_lensGeo[i];
    for (int i = 0; i < 4; ++i) m_cb.lensB[i] = m_lensChain[i];
    for (int i = 0; i < 4; ++i) m_cb.lensR[i] = m_lensRegion[i];
    m_cb.bankA[3] = sliceOn ? 1.0f : 0.0f;
    m_cb.bankC[3] = sliceD;
    memcpy(m_cb.bankOrg01, &m_bankOrg[0], 16);
    memcpy(m_cb.bankOrg23, &m_bankOrg[4], 16);
    memcpy(m_cb.bankOrg45, &m_bankOrg[8], 16);
    m_cb.bankU2[0] = m_bankSrv[2];
    for (int i = 0; i < 3; ++i) {
        m_cb.bankU2[i + 1] = m_bankDeriv[i];
        m_cb.bankB[i] = m_bankPatch[i];
        m_cb.bankC[i] = m_bankK[i];
        m_cb.bankD[i] = m_bankRms[i];   // M8: peak-shaping envelope reference
        m_cb.bankFold[i] = m_bankFold[i];   // M9c: the fold's own wavenumber
    }
    m_cb.bankB[3] = m_bankExag;
    m_cb.bankD[3] = foamOpacity;   // M8: peak foam opacity (data/wave_scene.json)
    m_cb.bankE[0] = ringBlendTexels;
    m_cb.bankE[1] = windGateVal;   // Monahan gate for the PS detail-tier whitecaps
    m_cb.bankE[2] = causticStrength;
    m_cb.bankE[3] = editFloorNavd;   // M8g: absolute edit-land floor (datum envelope)

    // M9: the water's quality. The optics switch on only when the plane is actually resident
    // -- an absent field must fall back to the M7c constants, not to an unbound descriptor.
    const bool opticsOn = waterOptics && m_oceanB.Valid();
    m_cb.optU[0] = (opticsOn && m_oceanB.Valid()) ? m_oceanB.srv : UINT32_MAX;
    m_cb.optU[1] = m_iceB.Valid() ? m_iceB.srv : UINT32_MAX;
    m_cb.optU[2] = opticsOn ? 1u : 0u;
    // M9bh: the water's SHADING STAGE. 0 = the vertex-shaded default (PsMain writes the
    // interpolated colour through), 1 = --pixel-water (PsMain runs WaterPixelColor's two rays).
    m_cb.optU[3] = pixelWater ? 1u : 0u;
    m_cb.optA[0] = static_cast<float>(m_globe->OcLat1());
    m_cb.optA[1] = static_cast<float>(m_globe->OcLon1());
    m_cb.optA[2] = static_cast<float>(1.0 / m_globe->OcDLat());
    m_cb.optA[3] = static_cast<float>(1.0 / m_globe->OcDLon());
    m_cb.optB[0] = static_cast<float>(m_globe->OcNx());
    m_cb.optB[1] = static_cast<float>(m_globe->OcNy());
    m_cb.optB[2] = kOcDeepGain;
    m_cb.optB[3] = 0.0f;
    m_cb.texIdx[0] = UINT32_MAX;   // was the equirect relief; the composed height cube owns it
    m_cb.texIdx[1] = m_hsB.Valid() ? m_hsB.srv : UINT32_MAX;
    m_cb.texIdx[2] = m_windB.Valid() ? m_windB.srv : UINT32_MAX;
    m_cb.wavesA[0] = static_cast<float>(m_globe->WavesLat1());
    m_cb.wavesA[1] = static_cast<float>(m_globe->WavesLon1());
    m_cb.wavesA[2] = static_cast<float>(1.0 / m_globe->WavesDLat());
    m_cb.wavesA[3] = static_cast<float>(1.0 / m_globe->WavesDLon());
    m_cb.wavesB[0] = static_cast<float>(m_globe->WavesNx());
    m_cb.wavesB[1] = static_cast<float>(m_globe->WavesNy());
    m_cb.wavesB[2] = cam.fovY / (std::max)(m_viewportH, 1.0f);   // pixel angular size
    m_cb.wavesB[3] = waterNavd;   // M6j: the live waterline for the close-up material model
    m_cb.texIdx[3] = m_cloudReady ? m_cloud.Srv() : UINT32_MAX;
    m_cb.cloudA[0] = 2.4e-3f;      // extinction /m at density 1 (stratiform-effective)
    m_cb.cloudA[1] = kShellTopM;
    m_cb.cloudA[2] = 1.0f;         // sun boost
    m_cb.cloudA[3] = 0.75f;        // ground-shadow strength
    m_cb.texIdx2[0] = UINT32_MAX;  // was the NE 15s window; a composed height LAYER now
    m_cb.texIdx2[1] = m_windReady ? m_windBank.Srv() : UINT32_MAX;
    m_cb.texIdx2[2] = windOverlay ? 1u : 0u;
    m_cb.texIdx2[3] = albedoLens ? 1u : 0u;
    m_cb.windGeo[0] = static_cast<float>(m_globe->WindLat1());
    m_cb.windGeo[1] = static_cast<float>(m_globe->WindLon1());
    m_cb.windGeo[2] = static_cast<float>(1.0 / std::abs(m_globe->WindDLat()));
    m_cb.windGeo[3] = static_cast<float>(1.0 / m_globe->WindDLon());
    m_cb.windB[0] = static_cast<float>(m_globe->WindNx());
    m_cb.windB[1] = static_cast<float>(m_globe->WindNy());

    // ---- M6e: Mars's native streams. On Mars, the live-Earth fields all stand down.
    const bool surfOn = m_res && m_surfT >= 0;
    const bool normOn = m_res && m_normT >= 0;
    m_cb.streamU[0] = surfOn ? m_res->TextureSrv(m_surfT) : UINT32_MAX;
    m_cb.streamU[1] = normOn ? m_res->TextureSrv(m_normT) : UINT32_MAX;
    m_cb.streamU[2] = surfOn ? m_res->ResidencySrv(m_surfT) : UINT32_MAX;
    m_cb.streamU[3] = normOn ? m_res->ResidencySrv(m_normT) : UINT32_MAX;
    m_cb.streamF[0] = surfOn ? 1.0f : 0.0f;
    m_cb.streamF[1] = normOn ? 1.0f : 0.0f;
    m_cb.streamF[2] = m_streamMars ? 1.0f : 0.0f;
    m_cb.streamF[3] = 0.0f;
    // ---- M6i: the composed channels + the one-world frame, through the ONE fill function
    // the terrain also uses -- the two layers cannot disagree about this math (M12 step 4a:
    // SurfaceFrame::Fill, the surface's own; the fingerprint below is the gate).
    // M12 step 4g: the composed-surface rows are the renderer's one buffer (b2), filled by
    // the frame loop from this same SurfaceFrame; the globe's own fill and its step 0
    // fingerprint (equal to the frame loop's at every pose it was ever read) are gone.
    if (m_streamMars) {
        m_cb.texIdx[1] = m_cb.texIdx[2] = m_cb.texIdx[3] = UINT32_MAX;   // waves/wind/clouds
        m_cb.texIdx2[1] = UINT32_MAX;                                    // wind bank
        m_cb.texIdx2[2] = 0;
        m_cb.optU[0] = m_cb.optU[1] = UINT32_MAX;   // Mars has no ocean colour and no sea ice
        m_cb.optU[2] = 0;
    }

    // ---- M10: THE LEVEL TABLE. Slot 0 is the camera's own level -- the identity gauge, the
    // scene's sun, set A -- so with no Droste link the table says exactly what the old
    // constants said. Slots 1..n are the extra levels SetDroste handed over.
    memset(m_cb.droste, 0, sizeof(m_cb.droste));
    auto fillLevel = [&](uint32_t slot, const double c[3], double sigma, const double Q[3][3],
                         float exag, const float sun[3], int bank, const float skyUp[3],
                         float skyDay) {
        float* r = m_cb.droste + slot * 24u;
        r[0] = static_cast<float>(c[0]);
        r[1] = static_cast<float>(c[1] + m_radius);   // sphere-centred, summed in doubles
        r[2] = static_cast<float>(c[2]);
        r[3] = static_cast<float>(sigma);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) r[4 + row * 4 + col] = static_cast<float>(Q[row][col]);
        }
        r[7] = exag;
        r[11] = static_cast<float>(c[1]);
        r[15] = static_cast<float>(bank);
        r[16] = sun[0];
        r[17] = sun[1];
        r[18] = sun[2];
        r[20] = skyUp[0];
        r[21] = skyUp[1];
        r[22] = skyUp[2];
        r[23] = skyDay;
    };
    {
        const double I3[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
        fillLevel(0u, m_camPos, 1.0, I3, reliefExagg, m_camSun, 0, m_camSkyUp, m_camSkyDay);
        for (size_t li = 0; li < m_levels.size(); ++li) {
            const DrosteLevel& L = m_levels[li];
            fillLevel(static_cast<uint32_t>(li + 1), L.cam, L.sigma, L.Q, L.reliefExagg, L.sun,
                      L.bankSet, L.skyUp, L.skyDay);
        }
        EyeInstruments();   // Phase A0: the levels' blocks logged, the two-worlds probe's rows
    }
    // THE VIEW'S WINDOWS: only where their levels are walked (the mesh path); on the fallback a
    // window with no world behind it would be a hole, so there are none.
    const bool gateOn = m_gateFirst > 0 && m_gateCount > 0 && m_msPath;
    m_cb.gateA[0] = gateOn ? static_cast<float>(m_gateFirst) : -1.0f;
    m_cb.gateA[1] = gateOn ? static_cast<float>(m_gateCount) : 0.0f;
    m_cb.gateA[2] = m_cb.gateA[3] = 0.0f;
    for (int k = 0; k < kMaxWindowChain; ++k) {
        if (gateOn && k < m_gateCount) {
            m_gateBoxes[k].Pack(m_cb.gateBox + k * 16);
        } else {
            for (int i = 0; i < 16; ++i) m_cb.gateBox[k * 16 + i] = 0.0f;
        }
    }
    // M13 step 2: the cascade sea's plane at the eye, for the pixel stage's sub-ring bands --
    // SAID IN THE TANGENT FRAME, in doubles (REVIEW finding 7). The chart's law is
    // u = (P - org) . e + off with every term in the planet frame (sim/WaveChart.h), and the pixel
    // stage holds its points in the tangent frame. These rows used to be copied across unchanged,
    // so the ripples were read with axes turned about 19 degrees and one of them squashed to 0.68
    // -- in a plane unrelated to the one the bank filled its texels by. So the chart's axes are
    // turned into the tangent frame with the surface's own rows (east, up, north: the
    // planet-to-tangent rows CsToTangent applies), and the constants are taken about the tangent
    // point A = R up, with R the radius the pixel's point is measured with (gGlo.x):
    //     Ce = (A - org) . e + off[0],   Cn = (A - org) . n + off[1]
    // The pixel then needs only its point less A (Globe.hlsl ChartUOf). Each constant is wrapped
    // to every cascade's patch -- the lengths the shader divides by (gBankB) -- before the cast:
    // the patches are periodic, and a constant of 1e5 m has a float grain of 8 mm.
    {
        const WaveChart::Frame& f = m_chartFrame;
        const double* rows[3] = {m_surface->east, m_surface->up, m_surface->north};
        auto dot3 = [](const double* a, const double* b) {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        };
        double fromOrg[3];   // A - org, planet frame
        for (int i = 0; i < 3; ++i) fromOrg[i] = m_radius * m_surface->up[i] - f.org[i];
        const double ce = dot3(fromOrg, f.e) + f.off[0];
        const double cn = dot3(fromOrg, f.n) + f.off[1];
        // Into [0, L), as WaterBankLayer wraps the kernel's own rows.
        auto wrapped = [&](double c, int k) {
            const double L = (m_bankPatch[k] > 1.0f) ? double(m_bankPatch[k]) : 1.0;
            return static_cast<float>(c - std::floor(c / L) * L);
        };
        for (int i = 0; i < 3; ++i) {
            m_cb.chartOrg[i] = wrapped(ce, i);
            m_cb.chartE[i] = static_cast<float>(dot3(rows[i], f.e));
            m_cb.chartN[i] = static_cast<float>(dot3(rows[i], f.n));
        }
        m_cb.chartOrg[3] = m_chartOn ? 1.0f : 0.0f;
        m_cb.chartE[3] = wrapped(cn, 0);
        m_cb.chartN[3] = wrapped(cn, 1);
        m_cb.chartCn[0] = wrapped(cn, 2);
        m_cb.chartCn[1] = m_cb.chartCn[2] = m_cb.chartCn[3] = 0.0f;
    }
    m_cb.drosteA[0] = static_cast<float>(1 + m_levels.size());
    m_cb.drosteA[1] = static_cast<float>(m_camLevelAbs);
    m_cb.drosteA[2] = static_cast<float>(m_lighting);
    m_cb.drosteA[3] = (m_drosteOn && m_portal[3] > 0.0) ? 1.0f : 0.0f;
    for (int i = 0; i < 3; ++i) m_cb.drostePortal[i] = static_cast<float>(m_portal[i]);
    m_cb.drostePortal[3] = m_drosteOn ? static_cast<float>(m_portal[3]) : 0.0f;
    m_cb.bankBU[0] = m_bankB[0];
    m_cb.bankBU[1] = m_bankB[1];
    m_cb.bankBU[2] = m_bankB[2];
    m_cb.bankBU[3] = (m_bankBOn && m_bankB[0] != 0xFFFFFFFFu) ? 1u : 0u;
    memcpy(m_cb.bankBOrg01, &m_bankBOrg[0], 16);
    memcpy(m_cb.bankBOrg23, &m_bankBOrg[4], 16);
    memcpy(m_cb.bankBOrg45, &m_bankBOrg[8], 16);

    // The sky pass rebuilds pixel rays from this basis (b2). M6j: the ONE render basis --
    // the shell can no longer roll apart from the surface it wraps.
    XMFLOAT3 cf, cr, cu;
    cam.ViewBasis(cf, cr, cu);
    m_skyCb.fwd[0] = cf.x; m_skyCb.fwd[1] = cf.y; m_skyCb.fwd[2] = cf.z;
    m_skyCb.fwd[3] = std::tan(cam.fovY * 0.5f);
    m_skyCb.right[0] = cr.x; m_skyCb.right[1] = cr.y; m_skyCb.right[2] = cr.z;
    m_skyCb.right[3] = aspect;
    m_skyCb.up[0] = cu.x; m_skyCb.up[1] = cu.y; m_skyCb.up[2] = cu.z;
    m_skyCb.up[3] = 0.0f;
    // M10: THE LIMBS. Every level whose air the eye is OUTSIDE of wears its limb (PsLimb, after
    // the surface) -- one rule for every slot, the camera's own included, because the re-root is
    // a gauge change and a rule keyed on "which slot is the camera's" would move a limb on it
    // (priors 36). The backdrop keeps the camera level's air only while the eye is inside it,
    // and then only at its share of the space backdrop (skyOwnAir); without Droste lvl = (0,1,0)
    // is the shipped backdrop exactly.
    m_limbCount = 0;
    m_skyCb.lvl[0] = 0.0f;
    m_skyCb.lvl[1] = 1.0f;
    m_skyCb.lvl[2] = 0.0f;
    m_skyCb.lvl[3] = 0.0f;
    for (int i = 0; i < 3; ++i) m_skyCb.spaceSun[i] = m_spaceSun[i];
    m_skyCb.spaceSun[3] = 0.0f;
    if (m_drosteOn && m_msPath && !albedoLens) {
        // The top of the planet's air (scene/Air.h), the same the sky's integral ends at.
        const double top = m_radius + AirOf(m_streamMars ? "mars" : "earth").row[4][1];
        struct Cand { uint32_t slot; double dist; };
        Cand cand[kMaxLevels];
        int nc = 0;
        // True if the eye is outside the level's air. Its limb joins the list when the shell
        // spans a pixel; the list is sorted by the TRUE distance to the shell.
        auto consider = [&](uint32_t slot, const double c[3], double sigma) {
            const double ry = m_radius + c[1];
            const double dC = std::sqrt(c[0] * c[0] + ry * ry + c[2] * c[2]);
            if (dC <= top) return false;
            if (std::asin(top / dC) >= double(m_wp.pixAng) && nc < kMaxLevels) {
                cand[nc++] = {slot, sigma * (dC - top)};
            }
            return true;
        };
        const bool outside0 = consider(0u, m_camPos, 1.0);
        for (size_t li = 0; li < m_levels.size(); ++li) {
            consider(static_cast<uint32_t>(li + 1), m_levels[li].cam, m_levels[li].sigma);
        }
        std::sort(cand, cand + nc, [](const Cand& a, const Cand& b) { return a.dist > b.dist; });
        for (int i = 0; i < nc; ++i) m_limbSlots[m_limbCount++] = cand[i].slot;
        m_skyCb.lvl[1] = outside0 ? 0.0f : std::clamp(skyOwnAir, 0.0f, 1.0f);
        m_skyCb.lvl[2] = 1.0f;
        m_skyCb.lvl[3] = m_wp.pixAng;   // the limb is filtered over this footprint
    }
    // (M6h: the M6b destination beacon is fully retired -- row and shader block deleted.)

    char s[96];
    if (m_msPath) {
        snprintf(s, sizeof(s), "globe %zu meshlets (MS)  alt %.0f km", m_meshlets.size(),
                 (r - m_radius) / 1000.0);
    } else {
        snprintf(s, sizeof(s), "globe %zu nodes  alt %.0f km", m_nodes.size(),
                 (r - m_radius) / 1000.0);
    }
    stats = s;
    if (m_drosteOn) {
        char d[96];
        snprintf(d, sizeof(d), "  droste L%d x%zu", m_camLevelAbs, 1 + m_levels.size());
        stats += d;
    }
    if (m_res) stats += "  " + m_res->stats;
}

void GlobeLayer::Render(const FrameContext& ctx) {
    if (!m_pso || (m_nodes.empty() && m_meshlets.empty())) return;
    PixScope scope(ctx.cmd->Native(),
                   "globe (atmosphere shell + quad-sphere CDLOD + sparse cloud volume)");

    // M6e: the residency manager's per-frame turn -- loads started, budgeted tiles mapped and
    // filled, residency maps refreshed -- BEFORE the surface samples any of it.
    // (The residency turn stands at the head of the frame's command list, law 8: FrameLoop's
    // hook on the renderer, before any layer records a read.)

    const D3D12_GPU_VIRTUAL_ADDRESS cbVa = ctx.gpu->PushConstants(&m_cb, sizeof(m_cb));

    // 1) The atmosphere backdrop: limb scatter + sun for every ray that misses the planet.
    if (m_skyPso && skyPassEnabled) {
        GpuScope gscope(ctx.prof, ctx.cmd->Native(), "globe.sky");
        PixMarker(ctx.cmd->Native(), "globe.sky (single-scatter shell: the limb past the disc)");
        ctx.cmd->Pipeline(m_skyPso.Get());
        const float w = std::clamp(skyPassWeight, 0.0f, 1.0f);
        const float bf[4] = {w, w, w, w};
        ctx.cmd->Native()->OMSetBlendFactor(bf);
        ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        ctx.cmd->GraphicsConstantsAt(1, cbVa);
        ctx.cmd->GraphicsConstants(5, m_skyCb);   // b3 (M12 step 4g: b2 is the surface's)
        ctx.cmd->Draw(3, 1, 0, 0);
    }

    // 2) The surface (which marches the sparse cloud bank on its way down).
    if (m_msPath && m_msPso && !m_meshlets.empty()) {
        // M6j: the unified surface -- meshlet records ride a frame-indexed upload buffer,
        // one DispatchMesh amplifies them from the composed channels.
        if (!ctx.cmd->MeshCapable()) {   // no List6 on this runtime: the classic path (3f)
            m_msPath = false;
            return;
        }
        GpuScope gscope(ctx.prof, ctx.cmd->Native(), "globe.mesh");
        GpuBuffer& rec = m_recBuf[ctx.gpu->FrameIndex()];
        const size_t bytes = m_meshlets.size() * sizeof(MeshletRec);
        const auto copy0 = std::chrono::steady_clock::now();   // meshletCopyMs bracket
        memcpy(rec.cpu, m_meshlets.data(), bytes);
        meshletCopyMs = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - copy0)
                            .count();
        hal::PsoPtr msSel = m_msPso.Get();
        if (surfaceDebug == 1 && m_msPsoWire) msSel = m_msPsoWire.Get();
        else if (surfaceDebug == 2 && m_msPsoMeshlet) msSel = m_msPsoMeshlet.Get();
        else if (surfaceDebug == 3 && m_msPsoWireFlat) msSel = m_msPsoWireFlat.Get();
        else if (ResidencyLensOn() && m_msPsoLens) msSel = m_msPsoLens.Get();
        ctx.cmd->Pipeline(msSel);
        ctx.cmd->GraphicsConstantsAt(1, cbVa);
        ctx.cmd->GraphicsSrvAt(2, rec.res->GetGPUVirtualAddress());
        // M10: 2-D, because one dimension caps at 65535 groups (GlobeMesh.hlsl folds y*65535+x).
        const UINT n = static_cast<UINT>(m_meshlets.size());
        ctx.cmd->DispatchMesh((std::min)(n, 65535u), (n + 65534u) / 65535u, 1);

        // 3) M10: the limbs of every planet whose air the eye is outside of, over the surface
        // just drawn (SetView chose the slots, farthest first).
        if (m_limbPso && m_limbCount > 0) {
            GpuScope lscope(ctx.prof, ctx.cmd->Native(), "globe.limbs");
            PixMarker(ctx.cmd->Native(),
                      "globe.limbs (each level's shell over what lies behind it)");
            ctx.cmd->Pipeline(m_limbPso.Get());
            ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            ctx.cmd->GraphicsConstantsAt(1, cbVa);
            for (int li = 0; li < m_limbCount; ++li) {
                SkyCbData lc = m_skyCb;
                lc.lvl[0] = static_cast<float>(m_limbSlots[li]);
                ctx.cmd->GraphicsConstants(5, lc);   // b3
                ctx.cmd->Draw(3, 1, 0, 0);
            }
        }
        return;
    }
    hal::PsoPtr sel = m_pso.Get();
    if (surfaceDebug == 1 && m_psoWire) sel = m_psoWire.Get();
    else if (surfaceDebug == 2 && m_psoMeshlet) sel = m_psoMeshlet.Get();
    else if (ResidencyLensOn() && m_psoLens) sel = m_psoLens.Get();
    ctx.cmd->Pipeline(sel);
    ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.cmd->GraphicsConstantsAt(1, cbVa);
    // Root param 2 normally carries the FieldSet; the renderer re-binds it every frame and the
    // globe draws last, so the CDLOD node list borrows the slot for this draw.
    ctx.cmd->GraphicsSrvAt(
        2, ctx.gpu->PushConstants(m_nodes.data(), m_nodes.size() * sizeof(NodeData)));
    ctx.cmd->Draw(32 * 32 * 6, static_cast<UINT>(m_nodes.size()), 0, 0);
}


// ---- Phase A0 (plan_eye_windows.md): THE INSTRUMENTS, no behaviour changed. -------------------
// (1) Per slot of the frame's level table (slot 0 the camera, then m_levels), the cells its OWN eye
// names -- rank k's cell is the eye's own address at rung 3k floored at 16384 texels (HIERARCHY
// 4.4: X = u N, a ratio of two planes; cell = floor(X / 16384)), ranks 1..K with
// K = 1 + ceil(-L / 3), L = log2 of the pixel's footprint at the eye's altitude over rank 1's
// texel on the ground there (4.17's measure) -- beside what every slot is BOUND to today: the
// surface's windows. Logged on the frames where
// any of it changes.
// (2) The two-worlds probe's rows (--ground-probe): G, and per slot G less that slot's eye in the
// tangent axes, in doubles, so the lens reads G as each world addresses it.
void GlobeLayer::EyeInstruments() {
    memset(m_cb.probeG, 0, sizeof(m_cb.probeG));
    memset(m_cb.probeP, 0, sizeof(m_cb.probeP));
    for (int i = 0; i < 4; ++i) m_cb.probeX[i] = probeX[i];
    if (!m_surface) return;
    const SurfaceFrame& sf = *m_surface;
    const double R = m_radius;
    auto planetOf = [&](const double c[3], double out[3]) {
        const double ry = R + c[1];
        for (int i = 0; i < 3; ++i) out[i] = sf.up[i] * ry + sf.east[i] * c[0] + sf.north[i] * c[2];
    };
    const size_t slots = 1u + m_levels.size();
    auto camOf = [&](size_t s) -> const double* { return s == 0 ? m_camPos : m_levels[s - 1].cam; };
    ++m_eyeFrame;
    std::string text, key;   // key: the cells and the binding only, so a moving eye logs when they change
    char buf[256];
    for (size_t s = 0; s < slots; ++s) {
        double E[3];
        planetOf(camOf(s), E);
        const double rE = std::sqrt(E[0] * E[0] + E[1] * E[1] + E[2] * E[2]);
        const double d[3] = {E[0] / rE, E[1] / rE, E[2] / rE};
        // THE MEASURE, the law's own function (SurfaceFrame::RanksAt).
        uint32_t face = 0;
        double Lm = 0.0;
        const int K = SurfaceFrame::RanksAt(E, R, double(m_wp.pixAng), &face, &Lm);
        const double alt = rE - R;
        snprintf(buf, sizeof(buf), "\n[eye-blocks]   slot %zu rel %d: face %u, alt %.0f m, K %d (L %.2f) --",
                 s, s == 0 ? 0 : m_levels[s - 1].rel, face, alt, K, Lm);
        text += buf;
        key += "|";
        for (int k = 1; k <= K; ++k) {
            const FaceWindow w{face, 3 * k, 0, 0};
            double X = 0.0, Y = 0.0;
            w.TexelOf(d, X, Y);
            const long long cx = static_cast<long long>(std::floor(X / Lattice::kFaceDim));
            const long long cy = static_cast<long long>(std::floor(Y / Lattice::kFaceDim));
            snprintf(buf, sizeof(buf), " r%d (%lld,%lld) org (%lld,%lld)", k, cx, cy,
                     cx * Lattice::kFaceDim, cy * Lattice::kFaceDim);
            text += buf;
            key += buf;
        }
    }
    // What each slot is bound to: its own windows (PHASE A2).
    std::string bound;
    for (size_t s = 0; s < slots && s < SurfaceFrame::kWindowSlots; ++s) {
        const uint32_t set = sf.slotSet[s];
        snprintf(buf, sizeof(buf), "\n[eye-blocks]   slot %zu bound (set %d):", s,
                 set == SurfaceFrame::kNoSet ? -1 : int(set));
        bound += buf;
        if (set == SurfaceFrame::kNoSet) continue;
        const SurfaceFrame::EyeWindows& ew = sf.bound[set];
        for (uint32_t i = 0; i < ew.K; ++i) {
            const hal::BlockBinding& b = ew.box[i];
            snprintf(buf, sizeof(buf), " slice %u = r%d face %u box org (%llu,%llu);",
                     SurfaceFrame::WindowSlice(set, i + 1), b.rung / 3, b.face,
                     static_cast<unsigned long long>(b.OrgX()), static_cast<unsigned long long>(b.OrgY()));
            bound += buf;
        }
        if (!ew.K) bound += " none (K 0: the cube)";
    }
    text += bound;
    key += bound;
    if (key != m_eyeLast) {
        m_eyeLast = key;
        Log("[eye-blocks] frame %llu, %zu slot(s):%s", static_cast<unsigned long long>(m_eyeFrame),
            slots, text.c_str());
    }
    // (2) the probe's rows.
    if (!groundProbeOn) return;
    double Gt[3];   // G on the sphere, in the tangent axes, less (0, R, 0): geo's own coordinates
    {
        const double* G = groundProbeDir;
        Gt[0] = R * (sf.east[0] * G[0] + sf.east[1] * G[1] + sf.east[2] * G[2]);
        Gt[1] = R * (sf.up[0] * G[0] + sf.up[1] * G[1] + sf.up[2] * G[2]) - R;
        Gt[2] = R * (sf.north[0] * G[0] + sf.north[1] * G[1] + sf.north[2] * G[2]);
    }
    const size_t filled = (std::min)(slots, size_t(kMaxLevels));
    for (int i = 0; i < 3; ++i) m_cb.probeG[i] = static_cast<float>(groundProbeDir[i]);
    m_cb.probeG[3] = static_cast<float>(filled);
    for (size_t s = 0; s < filled; ++s) {
        const double* c = camOf(s);
        for (int i = 0; i < 3; ++i) m_cb.probeP[4 * s + i] = static_cast<float>(Gt[i] - c[i]);
        m_cb.probeP[4 * s + 3] = 1.0f;
    }
    static std::string sLastProbe;
    std::string t;
    for (size_t s = 0; s < filled; ++s) {
        const double* c = camOf(s);
        snprintf(buf, sizeof(buf), " slot %zu p %.3f %.3f %.3f;", s, Gt[0] - c[0], Gt[1] - c[1],
                 Gt[2] - c[2]);
        t += buf;
    }
    if (m_eyeFrame % 30 == 1 || t.size() != sLastProbe.size()) {
        Log("[ground-probe] frame %llu dir %.12f %.12f %.12f, %zu slot(s):%s",
            static_cast<unsigned long long>(m_eyeFrame), groundProbeDir[0], groundProbeDir[1],
            groundProbeDir[2], filled, t.c_str());
    }
    sLastProbe = t;
}

}  // namespace ga
