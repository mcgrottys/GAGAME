#include "scene/GlobeLayer.h"
#include "core/ThreadManager.h"

#include "hal/GpuProfiler.h"

#include "compose/DomainSource.h"
#include "sim/BathyModel.h"

#include "hal/PixEvents.h"
#include "hal/Pipeline.h"
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

void GlobeLayer::Init(Gpu& gpu, ShaderCompiler& sc, FieldSet&, ID3D12RootSignature* rootSig) {
    m_rootSig = rootSig;
    if (!m_globe || !m_globe->Ready()) throw std::runtime_error("GlobeLayer needs globe data");
    if (!BuildPso(gpu, sc)) throw std::runtime_error("globe PSO failed");

    // M6j: the unified mesh-shader surface (orbit to helm, one pipeline). Falls back to the
    // classic VS path -- and the terrain layer -- if the device or compile says no.
    if (msSurface && BuildMeshPso(gpu, sc)) {
        for (uint32_t i = 0; i < Gpu::kFrameCount; ++i) {
            m_recBuf[i] = gpu.CreateUploadBuffer(
                static_cast<uint64_t>(kMaxMeshlets) * sizeof(MeshletRec),
                L"globe.meshlets (per-frame records)");
        }
        m_msPath = true;
        Log("[globe] mesh-shader surface ACTIVE (unified orbit-to-helm; terrain layer "
            "retires as a renderer)");
    } else if (msSurface) {
        Log("[globe] mesh-shader surface unavailable; classic VS path + terrain layer");
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
        D3D12_RESOURCE_DESC rd{};
        rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE3D;
        rd.Width = snx;
        rd.Height = sny;
        rd.DepthOrArraySize = static_cast<UINT16>(snz);
        rd.MipLevels = 1;
        rd.Format = DXGI_FORMAT_R32_FLOAT;
        rd.SampleDesc.Count = 1;
        D3D12_HEAP_PROPERTIES hp{};
        hp.Type = D3D12_HEAP_TYPE_DEFAULT;
        GA_CHECK(gpu.Device()->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
                                                      D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                      IID_PPV_ARGS(&m_cloudSrc.res)));
        m_cloudSrc.res->SetName(L"globe.cloudSrc (GFS isobaric TCDC)");
        m_cloudSrc.state = D3D12_RESOURCE_STATE_COPY_DEST;

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
    D3D12_STATIC_SAMPLER_DESC samp{};
    samp.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
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
    GA_CHECK(D3D12SerializeVersionedRootSignature(&vd, &blob, &err));
    GA_CHECK(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(),
                                              IID_PPV_ARGS(&m_cloudRs)));
    m_cloudBuild = hal::Require(
        hal::BuildCompute(gpu, m_cloudRs.Get(),
                          sc.Compile(m_shaderDir + L"/CloudVol.hlsl", L"CsCloudBuild", L"cs_6_0"),
                          "globe.cloud"),
        "CloudVol kernel");

    m_cloudTable = gpu.SrvHeap().Alloc(2);
    D3D12_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv.Format = DXGI_FORMAT_R32_FLOAT;
    sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
    sv.Texture3D.MipLevels = 1;
    gpu.Device()->CreateShaderResourceView(m_cloudSrc.res.Get(), &sv,
                                           gpu.SrvHeap().Cpu(m_cloudTable + 0));
    D3D12_UNORDERED_ACCESS_VIEW_DESC uv{};
    uv.Format = DXGI_FORMAT_R16_FLOAT;
    uv.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE3D;
    uv.Texture3D.WSize = UINT(-1);
    gpu.Device()->CreateUnorderedAccessView(m_cloud.Res(), nullptr, &uv,
                                            gpu.SrvHeap().Cpu(m_cloudTable + 1));

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
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};   // raw upload list: bind it
        up.Native()->SetDescriptorHeaps(1, heaps);
        up.ComputeRoot(m_cloudRs.Get());
        up.ComputeConstants(0, cb);
        const auto& list = m_cloud.ResidentList();
        up.ComputeSrvAt(1, gpu.PushConstants(list.data(), list.size() * 4));
        up.ComputeTable(2, m_cloudTable);
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

    // Build root signature + kernel (b0 CBV, t0 list, table [t1 src, u0 bank]).
    D3D12_DESCRIPTOR_RANGE1 ranges[2]{};
    ranges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;   // M9r: the source is a bank now
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
    GA_CHECK(gpu.Device()->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(),
                                              IID_PPV_ARGS(&m_windRs)));
    m_windBuild = hal::Require(
        hal::BuildCompute(gpu, m_windRs.Get(),
                          sc.Compile(m_shaderDir + L"/GlobeWind.hlsl", L"CsWindGrad", L"cs_6_0"),
                          "globe.wind"),
        "GlobeWind kernel");

    m_windTable = gpu.SrvHeap().Alloc(2);
    // A TEXTURE2D UAV over the bank's slice 0 -- the same drop-in trick as the SRV, in the
    // state the bank already holds.
    D3D12_UNORDERED_ACCESS_VIEW_DESC srcUav{};
    srcUav.Format = DXGI_FORMAT_R32G32_FLOAT;
    srcUav.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    gpu.Device()->CreateUnorderedAccessView(m_windSrcB.bank->Res(), nullptr, &srcUav,
                                            gpu.SrvHeap().Cpu(m_windTable + 0));
    D3D12_UNORDERED_ACCESS_VIEW_DESC uav{};
    uav.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
    uav.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    gpu.Device()->CreateUnorderedAccessView(m_windBank.Res(), nullptr, &uav,
                                            gpu.SrvHeap().Cpu(m_windTable + 1));

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
        ID3D12DescriptorHeap* heaps[] = {gpu.SrvHeap().Heap()};
        up.Native()->SetDescriptorHeaps(1, heaps);
        up.ComputeRoot(m_windRs.Get());
        up.ComputeConstants(0, cb);
        const auto& list = m_windBank.ResidentList();
        up.ComputeSrvAt(1, gpu.PushConstants(list.data(), list.size() * 4));
        up.ComputeTable(2, m_windTable);
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
    return true;
}

void GlobeLayer::ReloadShaders(Gpu& gpu, ShaderCompiler& sc) {
    BuildPso(gpu, sc);
    if (m_msPath) BuildMeshPso(gpu, sc);
}

// The mesh PSO rides the subobject STREAM (no d3dx12 in this repo): every subobject is
// { type, payload }, each aligned to a pointer boundary -- exactly what the runtime parses.
namespace {
template <typename T, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE Tag>
struct alignas(void*) StreamSub {
    D3D12_PIPELINE_STATE_SUBOBJECT_TYPE type = Tag;
    T val{};
};
}  // namespace

bool GlobeLayer::BuildMeshPso(Gpu& gpu, ShaderCompiler& sc) {
    Com<ID3D12Device2> dev2;
    if (FAILED(gpu.Device()->QueryInterface(IID_PPV_ARGS(&dev2)))) return false;
    D3D12_FEATURE_DATA_D3D12_OPTIONS7 o7{};
    if (FAILED(gpu.Device()->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &o7,
                                                 sizeof(o7))) ||
        o7.MeshShaderTier == D3D12_MESH_SHADER_TIER_NOT_SUPPORTED) {
        return false;
    }
    const std::wstring path = m_shaderDir + L"/GlobeMesh.hlsl";
    ShaderBlob ms = sc.Compile(path, L"MsMain", L"ms_6_5");
    ShaderBlob ps = sc.Compile(m_shaderDir + L"/Globe.hlsl", L"PsMain", L"ps_6_5");
    if (!ms.Valid() || !ps.Valid()) return false;

    struct {
        StreamSub<ID3D12RootSignature*, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_ROOT_SIGNATURE> rs;
        StreamSub<D3D12_SHADER_BYTECODE, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_MS> ms;
        StreamSub<D3D12_SHADER_BYTECODE, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_PS> ps;
        StreamSub<D3D12_BLEND_DESC, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_BLEND> blend;
        StreamSub<UINT, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_SAMPLE_MASK> mask;
        StreamSub<D3D12_RASTERIZER_DESC, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_RASTERIZER> rast;
        StreamSub<D3D12_DEPTH_STENCIL_DESC, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DEPTH_STENCIL>
            ds;
        StreamSub<DXGI_FORMAT, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DEPTH_STENCIL_FORMAT> dsv;
        StreamSub<D3D12_RT_FORMAT_ARRAY, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_RENDER_TARGET_FORMATS>
            rtv;
        StreamSub<DXGI_SAMPLE_DESC, D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_SAMPLE_DESC> sample;
    } stream;
    stream.rs.val = m_rootSig;
    stream.ms.val = {ms.Data(), ms.Size()};
    stream.ps.val = {ps.Data(), ps.Size()};
    stream.blend.val.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    stream.mask.val = UINT_MAX;
    stream.rast.val.FillMode = D3D12_FILL_MODE_SOLID;
    stream.rast.val.CullMode = D3D12_CULL_MODE_NONE;   // cube faces mix winding
    stream.rast.val.DepthClipEnable = TRUE;
    stream.ds.val.DepthEnable = TRUE;
    stream.ds.val.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
    stream.ds.val.DepthFunc = D3D12_COMPARISON_FUNC_GREATER;   // reversed-Z
    stream.dsv.val = DXGI_FORMAT_D32_FLOAT;
    stream.rtv.val.NumRenderTargets = 1;
    stream.rtv.val.RTFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    stream.sample.val = {1, 0};

    D3D12_PIPELINE_STATE_STREAM_DESC sd{sizeof(stream), &stream};
    Com<ID3D12PipelineState> pso;
    if (FAILED(dev2->CreatePipelineState(&sd, IID_PPV_ARGS(&pso)))) {
        Log("[globe] mesh PSO creation failed");
        return false;
    }
    m_msPso = pso;
    // M9b: the same pipeline in WIREFRAME. Identical shaders and identical displacement --
    // only the raster fill differs -- so what the lines show is exactly the geometry the
    // solid pass rasterizes, meshlet seams and all. A failure here is not fatal: the solid
    // path stands and the toggle simply has nothing to switch to.
    stream.rast.val.FillMode = D3D12_FILL_MODE_WIREFRAME;
    Com<ID3D12PipelineState> psoWire;
    if (SUCCEEDED(dev2->CreatePipelineState(&sd, IID_PPV_ARGS(&psoWire)))) {
        m_msPsoWire = psoWire;
    } else {
        Log("[globe] mesh WIREFRAME PSO creation failed (solid path unaffected)");
    }
    // M9bk: ...and the same wireframe with NO water shading on the lines (--wireflat), so the
    // mesh can be read as geometry instead of through the look it is carrying.
    ShaderBlob psW = sc.Compile(m_shaderDir + L"/Globe.hlsl", L"PsWireFlat", L"ps_6_5");
    if (psW.Valid()) {
        const auto psKeep = stream.ps.val;
        stream.ps.val = {psW.Data(), psW.Size()};
        Com<ID3D12PipelineState> psoWF;
        if (SUCCEEDED(dev2->CreatePipelineState(&sd, IID_PPV_ARGS(&psoWF)))) {
            m_msPsoWireFlat = psoWF;
        } else {
            Log("[globe] mesh FLAT-WIRE PSO creation failed (solid path unaffected)");
        }
        stream.ps.val = psKeep;
    }
    // ...and solid again, with the meshlet-identity pixel shader.
    ShaderBlob psM = sc.Compile(m_shaderDir + L"/Globe.hlsl", L"PsMeshlet", L"ps_6_5");
    if (psM.Valid()) {
        stream.rast.val.FillMode = D3D12_FILL_MODE_SOLID;
        stream.ps.val = {psM.Data(), psM.Size()};
        Com<ID3D12PipelineState> psoM;
        if (SUCCEEDED(dev2->CreatePipelineState(&sd, IID_PPV_ARGS(&psoM)))) {
            m_msPsoMeshlet = psoM;
        } else {
            Log("[globe] mesh MESHLET-TINT PSO creation failed (solid path unaffected)");
        }
    }
    return true;
}

// sin(1.4844) -- the old latitude clamp, moved onto the sine so the Mercator bound needs no
// asin. Monotonic, so clamping either side of it is the same statement.
static const double kSinLatClamp = std::sin(1.4844);

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
// falls through to leaf(face, u0, v0, size, arc, dist).
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
double SplitRange(const WalkParams& wp, double arc) {
    double d = arc * GlobeLayer::kLodFactor;
    if (wp.waveGrainM > 0.0f) {
        const double cell = arc / 32.0;
        const double grain =
            static_cast<double>(wp.waveGrainM) / GlobeLayer::kWaveOversample;
        if (cell > grain) {
            // The wave rule splits while cell > max(grain, dist*c); with cell past the grain
            // floor that is dist < cell/c, and the range gate caps it at the rings' reach.
            const double c = (std::max)(1.0 / 256.0,
                                        static_cast<double>(wp.pixAng) *
                                            GlobeLayer::kWavePxFloor);
            const double reach = GlobeLayer::kWaveRings * 256.0 * wp.waveGrainM;
            d = (std::max)(d, (std::min)(cell / c, reach));
        }
    }
    return d;
}

template <class Leaf>
void WalkNode(const WalkParams& wp, uint64_t& nodes, int face, int level, double u0,
              double v0, double size, Leaf& leaf) {
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
    const double dist =
        std::sqrt(rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]);

    // Horizon cull in the planet frame (angles, not dots: both can exceed 90 degrees).
    const double r =
        std::sqrt(wp.camPlanet[0] * wp.camPlanet[0] + wp.camPlanet[1] * wp.camPlanet[1] +
                  wp.camPlanet[2] * wp.camPlanet[2]);
    if (r > R + 10000.0) {
        const double cosA = (dir[0] * wp.camPlanet[0] + dir[1] * wp.camPlanet[1] +
                             dir[2] * wp.camPlanet[2]) / r;
        const double ang = std::acos(std::clamp(cosA, -1.0, 1.0));
        const double horizon = std::acos(std::clamp(R / r, 0.0, 1.0));
        const double nodeAng = arc * 0.80 / R;   // generous half-diagonal
        if (ang > horizon + nodeAng + 0.02) return;
    } else if (wp.probeCullFar) {
        // Step 23 probe: the same test below 10 km with a 0.1 rad (640 km) margin -- nothing
        // visible from under 10 km lies beyond it, so a pixel this cull changes is a ray that
        // left the shell.
        const double cosA = (dir[0] * wp.camPlanet[0] + dir[1] * wp.camPlanet[1] +
                             dir[2] * wp.camPlanet[2]) / r;
        const double ang = std::acos(std::clamp(cosA, -1.0, 1.0));
        const double horizon = std::acos(std::clamp(R / r, 0.0, 1.0));
        const double nodeAng = arc * 0.80 / R;
        if (ang > horizon + nodeAng + 0.10) return;
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
    for (int p = 0; p < wp.planeCount; ++p) {
        const double d = wp.frustum[p][0] * rel[0] + wp.frustum[p][1] * rel[1] +
                         wp.frustum[p][2] * rel[2] + wp.frustum[p][3];
        if (d < -radius) return;
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

    // M6j/M8h: the mesh path walks two rungs past CUDEM scale (level 18 = 1.19 m vertex
    // spacing -- level 16's 4.77 m exactly saturated the old 4.8 m bank ring and could
    // not articulate what a finer ring stores; the user's call: more wave vertices).
    // The fallback VS path keeps its classic depth (the terrain layer covers the near
    // field). The distance split (dist < 3*arc) reaches level 18 only within ~115 m of
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
            const double waveTexel =
                (std::max)(static_cast<double>(wp.waveGrainM), dist / 256.0) /
                GlobeLayer::kWaveOversample;
            const double pxFloor = dist * wp.pixAng * GlobeLayer::kWavePxFloor;
            split = cell > (std::max)(waveTexel, pxFloor);
        }
    }

    if (split) {
        const double h = size * 0.5;
        WalkNode(wp, nodes, face, level + 1, u0, v0, h, leaf);
        WalkNode(wp, nodes, face, level + 1, u0 + h, v0, h, leaf);
        WalkNode(wp, nodes, face, level + 1, u0, v0 + h, h, leaf);
        WalkNode(wp, nodes, face, level + 1, u0 + h, v0 + h, h, leaf);
        return;
    }
    leaf(face, u0, v0, size, arc, dist);
}

// A leaf's wants, in emission order: emit(tenant, face, mip, u0, v0, u1, v1) once per
// Want() the leaf asks for, in the order it asks. The real walk sends each to the manager
// at once; the prefetch walk records them and the main thread replays them.
template <class Emit>
void LeafWants(const WalkParams& wp, int face, double u0, double v0, double size, double arc,
               double dist, Emit& emit) {
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
    if (wp.surfT >= 0) emit(wp.surfT, f, m, tu0, tv0, tu1, tv1);
    if (wp.normT >= 0) emit(wp.normT, f, m, tu0, tv0, tu1, tv1);
    if (wp.colorT >= 0) emit(wp.colorT, f, m, tu0, tv0, tu1, tv1);
    if (wp.hgtT >= 0) emit(wp.hgtT, f, m, tu0, tv0, tu1, tv1);
    if (wp.maskT >= 0) emit(wp.maskT, f, m, tu0, tv0, tu1, tv1);
    // M6f: the window's demand -- the node's corners in Mercator z14-pixel space,
    // intersected with the window; its mip matches the same on-screen texel math against
    // the window's OWN pyramid (mip 0 = z14). Color and height windows share the frame,
    // so one rect feeds both.
    if (wp.winT >= 0 || wp.hgtWinT >= 0) {
        double mmin[2] = {1e18, 1e18}, mmax[2] = {-1e18, -1e18};
        double s1min = 1e18, s1max = -1e18;
        for (int cy = 0; cy < 3; ++cy) {
            for (int cx = 0; cx < 3; ++cx) {
                // M9y: THE SAME MERCATOR, WITHOUT asin AND tan.
                //
                // This ran nine times per leaf -- 653 leaves a frame, so ~29000
                // transcendental calls -- to bound a node in z14 pixel space. Four of the
                // five per corner were avoidable, because CubeDirD already hands back the
                // unit direction and d[1] IS sin(latitude):
                //
                //   tan(pi/4 + phi/2) == (1 + sin phi) / cos phi == (1 + d1) / sqrt(1 - d1^2)
                //
                // so asin (to get phi) and tan (to undo it) cancel algebraically. What is
                // left is one log and one sqrt. The latitude clamp moves onto the SINE,
                // which is exactly equivalent because asin is monotonic on [-1, 1].
                double d[3];
                CubeDirD(face, u0 + size * cx * 0.5, v0 + size * cy * 0.5, d);
                const double lon = std::atan2(d[2], d[0]);
                const double n14 = 16384.0 * 256.0;
                const double mx = (lon / 3.14159265358979 * 0.5 + 0.5) * n14;
                // my is STRICTLY DECREASING in d[1], so its extremes over the corners are
                // f() of the extremes of the sine -- track the sine here and evaluate the
                // log exactly twice, after the loop, instead of nine times inside it.
                // Monotonicity, not approximation: the same two numbers come out.
                s1min = (std::min)(s1min, d[1]);
                s1max = (std::max)(s1max, d[1]);
                mmin[0] = (std::min)(mmin[0], mx);
                mmax[0] = (std::max)(mmax[0], mx);
            }
        }
        // The two log evaluations the loop above no longer does nine times each. Decreasing
        // in the sine, so the largest sine gives the smallest y.
        {
            const double n14 = 16384.0 * 256.0;
            const double lo = std::clamp(s1min, -kSinLatClamp, kSinLatClamp);
            const double hi = std::clamp(s1max, -kSinLatClamp, kSinLatClamp);
            const double k = n14 / (2.0 * 3.14159265358979);
            mmin[1] = 0.5 * n14 - std::log((1.0 + hi) / std::sqrt(1.0 - hi * hi)) * k;
            mmax[1] = 0.5 * n14 - std::log((1.0 + lo) / std::sqrt(1.0 - lo * lo)) * k;
        }
        const double du0 = (mmin[0] - wp.detOrg[0]) / wp.detSize;
        const double dv0 = (mmin[1] - wp.detOrg[1]) / wp.detSize;
        const double du1 = (mmax[0] - wp.detOrg[0]) / wp.detSize;
        const double dv1 = (mmax[1] - wp.detOrg[1]) / wp.detSize;
        if (du1 > 0.0 && dv1 > 0.0 && du0 < 1.0 && dv0 < 1.0) {
            const double span = (std::max)(du1 - du0, dv1 - dv0);
            const double texAtMip0W = span * 16384.0;
            const int dmip = (std::max)(
                0, static_cast<int>(std::ceil(
                       std::log2((std::max)(texAtMip0W / (std::max)(px, 16.0), 1.0)))));
            const float wu0 = static_cast<float>((std::max)(du0, 0.0));
            const float wv0 = static_cast<float>((std::max)(dv0, 0.0));
            const float wu1 = static_cast<float>((std::min)(du1, 1.0));
            const float wv1 = static_cast<float>((std::min)(dv1, 1.0));
            const uint32_t dm = static_cast<uint32_t>(dmip);
            if (wp.winT >= 0) emit(wp.winT, wp.winFace, dm, wu0, wv0, wu1, wv1);
            if (wp.hgtWinT >= 0) emit(wp.hgtWinT, wp.hgtWinFace, dm, wu0, wv0, wu1, wv1);
            if (wp.maskT >= 0) emit(wp.maskT, 6u, dm, wu0, wv0, wu1, wv1);
        }
        // M7f: the z17 DETAIL window rides the same node box, 8x finer frame.
        if (wp.detWinT >= 0) {
            const double eu0 = (mmin[0] * 8.0 - wp.det17Org[0]) / 16384.0;
            const double ev0 = (mmin[1] * 8.0 - wp.det17Org[1]) / 16384.0;
            const double eu1 = (mmax[0] * 8.0 - wp.det17Org[0]) / 16384.0;
            const double ev1 = (mmax[1] * 8.0 - wp.det17Org[1]) / 16384.0;
            if (eu1 > 0.0 && ev1 > 0.0 && eu0 < 1.0 && ev0 < 1.0) {
                const double spanD = (std::max)(eu1 - eu0, ev1 - ev0);
                const int emip = (std::max)(
                    0, static_cast<int>(std::ceil(std::log2(
                           (std::max)(spanD * 16384.0 / (std::max)(px, 16.0), 1.0)))));
                const uint32_t em = static_cast<uint32_t>(emip);
                const float du0f = static_cast<float>((std::max)(eu0, 0.0));
                const float dv0f = static_cast<float>((std::max)(ev0, 0.0));
                const float du1f = static_cast<float>((std::min)(eu1, 1.0));
                const float dv1f = static_cast<float>((std::min)(ev1, 1.0));
                emit(wp.detWinT, wp.detFace, em, du0f, dv0f, du1f, dv1f);
                if (wp.maskT >= 0) emit(wp.maskT, 7u, em, du0f, dv0f, du1f, dv1f);
            }
        }
    }
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
        out[0] = m_frameE[0] * px + m_frameE[1] * py + m_frameE[2] * pz;
        out[1] = m_frameU[0] * px + m_frameU[1] * py + m_frameU[2] * pz - R;
        out[2] = m_frameN[0] * px + m_frameN[1] * py + m_frameN[2] * pz;
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
            rec.upT[0] = static_cast<float>(m_frameE[0] * dc[0] + m_frameE[1] * dc[1] +
                                            m_frameE[2] * dc[2]);
            rec.upT[1] = static_cast<float>(m_frameU[0] * dc[0] + m_frameU[1] * dc[1] +
                                            m_frameU[2] * dc[2]);
            rec.upT[2] = static_cast<float>(m_frameN[0] * dc[0] + m_frameN[1] * dc[1] +
                                            m_frameN[2] * dc[2]);
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

// Step 5: everything the node walk reads, captured. SetView fills one for the real walk
// (then adds the five planes); StartPredictWalk fills one for the prefetch walk and moves
// only the eye -- the planet-frame position and the pixel angle stay the REAL camera's,
// exactly as PredictWants left them (it overrode m_camPos and the planes, nothing else).
GlobeLayer::WalkParams GlobeLayer::CaptureWalk(const Camera& cam, float viewportH) const {
    WalkParams wp;
    wp.R = m_radius;
    for (int i = 0; i < 3; ++i) {
        wp.frameE[i] = m_frameE[i];
        wp.frameU[i] = m_frameU[i];
        wp.frameN[i] = m_frameN[i];
    }
    wp.camPos[0] = cam.px;
    wp.camPos[1] = cam.py;
    wp.camPos[2] = cam.pz;
    // M6g: the planet-frame position (doubles) for the horizon test.
    const double ry = m_radius + cam.py;
    for (int i = 0; i < 3; ++i) {
        wp.camPlanet[i] = m_frameU[i] * ry + m_frameE[i] * cam.px + m_frameN[i] * cam.pz;
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
    wp.waveGrainM = (m_msPath && m_oneWater) ? m_bankBase : 0.0f;
    wp.waveMaxDepth = kWaveMaxDepth;
    wp.pixAng = cam.fovY / (std::max)(viewportH, 1.0f);
    wp.wants = m_res && (m_surfT >= 0 || m_colorT >= 0 || m_hgtT >= 0);
    wp.surfT = m_surfT;
    wp.normT = m_normT;
    wp.colorT = m_colorT;
    wp.hgtT = m_hgtT;
    wp.maskT = m_maskT;
    wp.winT = m_winT;
    wp.hgtWinT = m_hgtWinT;
    wp.detWinT = m_detWinT;
    wp.winFace = m_winFace;
    wp.hgtWinFace = m_hgtWinFace;
    wp.detFace = m_detFace;
    wp.detOrg[0] = m_detOrg[0];
    wp.detOrg[1] = m_detOrg[1];
    wp.detSize = m_detSize;
    wp.det17Org[0] = m_det17Org[0];
    wp.det17Org[1] = m_det17Org[1];
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
        auto leaf = [&](int face, double u0, double v0, double size, double arc, double dist) {
            ++leaves;
            auto emit = [&](int tenant, uint32_t f, uint32_t mip, float u0r, float v0r,
                            float u1r, float v1r) {
                out.push_back(WantRect{tenant, f, mip, u0r, v0r, u1r, v1r});
            };
            LeafWants(wp, face, u0, v0, size, arc, dist, emit);
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
        m_res->Want(r.tenant, r.face, r.mip, r.u0, r.v0, r.u1, r.v1, true);
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
void GlobeLayer::WalkLevel(const WalkParams& wp, uint32_t slot) {
    auto leaf = [&](int face, double u0, double v0, double size, double arc, double dist) {
        ++walkLeaves;
        const auto wt0 = std::chrono::steady_clock::now();
        auto emit = [&](int tenant, uint32_t f, uint32_t mip, float u0r, float v0r, float u1r,
                        float v1r) { m_res->Want(tenant, f, mip, u0r, v0r, u1r, v1r); };
        LeafWants(wp, face, u0, v0, size, arc, dist, emit);
        walkWantNs += uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::steady_clock::now() - wt0).count());

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
            const size_t base = m_meshlets.size();
            EmitMeshlets(face, u0, v0, size, arc, morphStart, morphEnd, wp.camPos, slot);
            // Step 23: the seam table's key. size is 2^-level and u0, v0 are multiples of
            // it, so the level and the grid position are exact integers. M10: the Droste slot
            // rides the top bits, so seams never cross levels.
            if (m_meshlets.size() == base + 16) {
                const int level = static_cast<int>(std::lround(-std::log2(size)));
                const uint64_t ix = static_cast<uint64_t>(std::llround(u0 / size));
                const uint64_t iy = static_cast<uint64_t>(std::llround(v0 / size));
                m_leafKeys.push_back(LeafKey{(static_cast<uint64_t>(slot) << 61) |
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
    for (int i = 0; i < 6; ++i) {
        WalkNode(wp, walkNodes, (wp.camFace + i) % 6, 0, 0.0, 0.0, 1.0, leaf);
    }
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
    // The camera's own level: slot 0, the identity gauge -- exactly the walk this always was.
    WalkLevel(m_wp, 0u);
    levelRecords[0] = static_cast<uint32_t>(m_meshlets.size());
    // ---- M10: THE CYCLE, TAKEN. Each further level is the ROOT walked again under the eye
    // S^-k(C) (Droste.h). Everything the walk reads is scale-free or already in the level's own
    // units: the split rule compares distance to arc, the horizon test is angular, and the
    // frustum's planes carry across as n -> Q^T n, d -> d / s^k (a similarity keeps planes
    // planes). Its wants go to the SAME tenants at the SAME addresses -- the sparse structure
    // answers every level from one resident set, and a small globe asks only for coarse mips
    // the root already holds. Its records carry their slot, so the mesh stage knows which
    // gauge to rasterize them through.
    for (size_t li = 0; li < m_levels.size() && m_msPath; ++li) {
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
        for (int i = 0; i < 3; ++i) {
            wp.camPos[i] = L.cam[i];
            wp.camPlanet[i] = m_frameU[i] * ry + m_frameE[i] * L.cam[0] + m_frameN[i] * L.cam[2];
        }
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
        for (int p = 0; p < m_wp.planeCount; ++p) {
            const double* n = m_wp.frustum[p];
            for (int j = 0; j < 3; ++j) {
                wp.frustum[p][j] = L.Q[0][j] * n[0] + L.Q[1][j] * n[1] + L.Q[2][j] * n[2];
            }
            wp.frustum[p][3] = n[3] / L.sigma;
        }
        const size_t before = m_meshlets.size();
        WalkLevel(wp, slot);
        levelRecords[slot] = static_cast<uint32_t>(m_meshlets.size() - before);
    }
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

    const double r = std::sqrt(m_camPos[0] * m_camPos[0] + m_camPos[1] * m_camPos[1] +
                               m_camPos[2] * m_camPos[2]);
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
    m_cb.bankU[2] = (m_oneWater && m_bankSrv[0] != 0xFFFFFFFFu) ? 1u : 0u;
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
    // the terrain also uses -- the two layers cannot disagree about this math.
    // M7g: THE MIP FLOOR. The top of every window pyramid (mips 4..7, ~85 tiles, a few
    // MB) is wanted EVERY frame: high-altitude views sample one consistent capture instead
    // of a residency-shaped patchwork of vintages, and a fast ascent can never outrun the
    // loader into grey -- the coarse rung is always there to fall back on.
    for (int fm = 4; fm <= 7; ++fm) {
        if (m_winT >= 0) m_res->Want(m_winT, m_winFace, fm, 0.0f, 0.0f, 1.0f, 1.0f, false);
        if (m_hgtWinT >= 0) m_res->Want(m_hgtWinT, m_hgtWinFace, fm, 0.0f, 0.0f, 1.0f, 1.0f, false);
        if (m_detWinT >= 0) m_res->Want(m_detWinT, m_detFace, fm, 0.0f, 0.0f, 1.0f, 1.0f, false);
        if (m_maskT >= 0) {
            m_res->Want(m_maskT, 6u, fm, 0.0f, 0.0f, 1.0f, 1.0f, false);
            m_res->Want(m_maskT, 7u, fm, 0.0f, 0.0f, 1.0f, 1.0f, false);
        }
    }
    FillComposedCb(m_cb.cs, m_res, m_colorT, m_winT, m_hgtT, m_hgtWinT, m_detOrg[0],
                   m_detOrg[1], m_detSize, 14, m_radius, m_frameE, m_frameU, m_frameN,
                   stencilOverlay, m_maskT, m_detWinT, m_det17Org, 17,
                   m_winFace ? m_winFace : UINT32_MAX, m_detFace ? m_detFace : UINT32_MAX,
                   m_hgtWinFace ? m_hgtWinFace : UINT32_MAX);
    {   // M12 step 0 instrument: does this fill agree with main.cpp's ([surface] main fill)?
        static uint64_t sLastFill = 0;
        const uint64_t h = Fnv1aBytes(&m_cb.cs, sizeof(m_cb.cs));
        if (h != sLastFill) {
            sLastFill = h;
            Log("[surface] globe fill FNV-1a %016llx", static_cast<unsigned long long>(h));
        }
    }
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
        const double top = m_radius + 60000.0;   // Globe.hlsl kAtmTop
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
    if (m_res) {
        GpuScope gscope(ctx.prof, ctx.cmd->Native(), "globe.residency");
        m_res->ProcessQueues(*ctx.gpu, ctx.cmd->Native());
    }

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
        ctx.cmd->GraphicsConstants(4, m_skyCb);
        ctx.cmd->Draw(3, 1, 0, 0);
    }

    // 2) The surface (which marches the sparse cloud bank on its way down).
    if (m_msPath && m_msPso && !m_meshlets.empty()) {
        // M6j: the unified surface -- meshlet records ride a frame-indexed upload buffer,
        // one DispatchMesh amplifies them from the composed channels.
        if (!m_cl6 && FAILED(ctx.cmd->Native()->QueryInterface(IID_PPV_ARGS(&m_cl6)))) {
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
        ID3D12PipelineState* msSel = m_msPso.Get();
        if (surfaceDebug == 1 && m_msPsoWire) msSel = m_msPsoWire.Get();
        else if (surfaceDebug == 2 && m_msPsoMeshlet) msSel = m_msPsoMeshlet.Get();
        else if (surfaceDebug == 3 && m_msPsoWireFlat) msSel = m_msPsoWireFlat.Get();
        ctx.cmd->Pipeline(msSel);
        ctx.cmd->GraphicsConstantsAt(1, cbVa);
        ctx.cmd->GraphicsSrvAt(2, rec.res->GetGPUVirtualAddress());
        // M10: 2-D, because one dimension caps at 65535 groups (GlobeMesh.hlsl folds y*65535+x).
        const UINT n = static_cast<UINT>(m_meshlets.size());
        m_cl6->DispatchMesh((std::min)(n, 65535u), (n + 65534u) / 65535u, 1);

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
                ctx.cmd->GraphicsConstants(4, lc);
                ctx.cmd->Draw(3, 1, 0, 0);
            }
        }
        return;
    }
    ID3D12PipelineState* sel = m_pso.Get();
    if (surfaceDebug == 1 && m_psoWire) sel = m_psoWire.Get();
    else if (surfaceDebug == 2 && m_psoMeshlet) sel = m_psoMeshlet.Get();
    ctx.cmd->Pipeline(sel);
    ctx.cmd->Topology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx.cmd->GraphicsConstantsAt(1, cbVa);
    // Root param 2 normally carries the FieldSet; the renderer re-binds it every frame and the
    // globe draws last, so the CDLOD node list borrows the slot for this draw.
    ctx.cmd->GraphicsSrvAt(
        2, ctx.gpu->PushConstants(m_nodes.data(), m_nodes.size() * sizeof(NodeData)));
    ctx.cmd->Draw(32 * 32 * 6, static_cast<UINT>(m_nodes.size()), 0, 0);
}

}  // namespace ga
