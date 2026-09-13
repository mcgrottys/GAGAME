// ================================================================================================
//  GradeBank.h - M12 step 3e: THE GRADE BANK, under src/hal/ where the tiles are. Moved
//  VERBATIM from core/GradeField.h (M9h); the expression layer it serves -- the grade bits, the
//  descriptor, the residency policies, Field<Sig> and the product / sum / grad expressions
//  whose types the algebra decides -- stays in core/FieldExpr.h with no tile in it. This file
//  is the GPU half: the reserved-resource atlas, the mip chain, the dense fill by CopyTiles,
//  the per-tile grade signatures a policy reads. The two members of Field<Sig> that
//  dereference the bank (Srv, AgreesWithBank) are defined at the bottom, where the bank is
//  complete; the expression layer names the bank only through a pointer. core/GradeField.h
//  forwards to both halves, so no includer changed.
// ================================================================================================
#pragma once

#include "core/FieldExpr.h"
#include "hal/Context.h"
#include "hal/Gpu.h"
#include "hal/TileAtlas.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace ga {

// ================================================================================================
//  GradeBank -- one grade bank: a reserved texture, its residency, its policy, its descriptor.
//
//  Replaces the five hand-rolled RequestMap/CommitMappings/clear-fresh loops (SweSolver,
//  SeaLayer churn, WaterBankLayer x3, GlobeLayer windBank/cloud) with one that is correct by
//  construction about the atlas's single hazard: NULL-TILE WRITES ARE SILENTLY DISCARDED, so
//  every fill dispatches from ResidentList(), never over the full domain.
// ================================================================================================
class GradeBank {
public:
    // Newly mapped tiles carry UNDEFINED contents; the fill runs over them before use.
    using FillFn = std::function<void(Gpu&, hal::CommandContext&,
                                      const std::vector<uint32_t>& tiles)>;

    void Init(Gpu& gpu, GradeBankDesc desc, TilePolicy policy) {
        m_desc = std::move(desc);
        m_policy = policy ? std::move(policy) : policy::All();
        m_atlas.Init(gpu, m_desc.width, m_desc.height, m_desc.fmt,
                     std::wstring(m_desc.name.begin(), m_desc.name.end()).c_str(), 64,
                     m_desc.mipLevels, m_desc.arraySlices);
        m_sig.assign(size_t(m_atlas.TilesX()) * m_atlas.TilesY(), m_desc.gradeSig);
    }

    // M9h: fill the chain after mip 0 changes. Coarse levels ARE the floor -- unfilled they
    // are real memory reading zero, which looks exactly like "the field is zero here".
    // Two halves, and they must run in different places. Mapping touches the queue and the
    // residency map, so it belongs OUTSIDE command-list recording; the reduction is pure
    // command-list work and belongs inside. Collapsing them crashed the SWE spinup.
    void ActivateSlice(Gpu& gpu, uint32_t slice) { m_atlas.ActivateSlice(gpu, slice); }

    // Map EVERY level of a page. ActivateSlice only pins the floor, and a write into an
    // unmapped tile is DISCARDED by the hardware -- silently, which is hazard 1 arriving by
    // the upload path instead of the dispatch path. A composed page that is uploaded before
    // its levels are mapped simply is not there, and nothing says so.
    void MapAllLevels(Gpu& gpu, uint32_t slice) {
        m_atlas.ActivateSlice(gpu, slice);
        for (uint32_t m = 0; m < m_atlas.StandardMips(); ++m) {
            for (uint32_t ty = 0; ty < m_atlas.TilesY(m); ++ty) {
                for (uint32_t tx = 0; tx < m_atlas.TilesX(m); ++tx) {
                    m_atlas.RequestMap(slice, m, tx, ty);
                }
            }
        }
        m_atlas.CommitMappings(gpu, nullptr);
        m_atlas.FlushResidencyMap(gpu);
    }
    uint32_t Slices() const { return m_atlas.Slices(); }

    void EnsureCoarseMapped(Gpu& gpu) {
        if (m_desc.mipLevels < 2 || m_coarseMapped) return;
        m_atlas.MapAllCoarse();
        m_atlas.CommitMappings(gpu, nullptr);
        m_atlas.FlushResidencyMap(gpu);
        m_coarseMapped = true;
    }
    // M9n: declare which channel holds coverage so the mip chain weights by it instead of
    // letting absent texels drag the coarse levels toward zero. Set before the first BuildChain.
    void SetCoverageChannel(int ch) { m_atlas.SetCoverageChannel(ch); }

    void BuildChain(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
                    hal::CommandContext& cmd) {
        if (m_desc.mipLevels < 2 || !m_coarseMapped) return;
        m_atlas.BuildMips(gpu, sc, shaderDir, cmd.Native());
    }
    uint32_t ResidencyMapSrv() const { return m_atlas.ResidencyMapSrv(); }

    // M9i: PUT A COMPOSED PAGE IN. This is the step that closes the loop -- a FieldCompositor
    // produces a page of floats in CPU memory, and this is how it reaches the reserved array.
    // Deliberately generic: the bank does not know or care that the page came from GoMOFS, a
    // GeoTIFF, a buoy set, or all three composed. It takes a level and some texels.
    //
    // Must run OUTSIDE command-list recording -- Gpu::UploadTexture opens its own list.
    // `rows` is float32, channels interleaved. If the bank stores RGBA16F -- most do -- the
    // floats are converted here rather than by the caller: UploadTexture copies raw bytes, so
    // a caller that forgot would write garbage that reports as a successful upload and only
    // shows up as a page that renders as nothing.
    void UploadLevel(Gpu& gpu, uint32_t mip, const void* rows, uint32_t rowPitchBytes,
                     uint32_t w, uint32_t h, uint32_t slice = 0) {
        if (mip >= m_atlas.MipCount() || slice >= m_atlas.Slices()) return;
        std::vector<uint16_t> half;
        if (m_desc.fmt == DXGI_FORMAT_R16G16B16A16_FLOAT ||
            m_desc.fmt == DXGI_FORMAT_R16G16_FLOAT || m_desc.fmt == DXGI_FORMAT_R16_FLOAT) {
            const uint32_t ch = (m_desc.fmt == DXGI_FORMAT_R16G16B16A16_FLOAT) ? 4u
                                : (m_desc.fmt == DXGI_FORMAT_R16G16_FLOAT)     ? 2u
                                                                               : 1u;
            const float* src = static_cast<const float*>(rows);
            half.resize(size_t(w) * h * ch);
            for (size_t i = 0; i < half.size(); ++i) half[i] = F32ToHalf(src[i]);
            rows = half.data();
            rowPitchBytes = w * ch * 2u;
        }
        GpuTexture t;
        t.res = m_atlas.Res();
        t.state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        t.format = m_desc.fmt;
        t.width = w;
        t.height = h;
        // Com is ComPtr: assigning the raw pointer AddRef'd it, so the destructor balances
        // that. Detaching here would LEAK the reference, not protect the atlas.
        // Subresource is mip + slice * mipCount -- the same indexing the mapping path uses.
        gpu.UploadTexture(t, rows, rowPitchBytes, mip + slice * m_atlas.MipCount());
    }
    uint32_t MipCount() const { return m_atlas.MipCount(); }

    // One frame step: re-evaluate the policy, map/unmap the delta, clear what arrived. Returns
    // the tiles that became resident this frame (undefined contents until filled).
    const std::vector<uint32_t>& Update(Gpu& gpu) {
        m_fresh.clear();
        for (uint32_t ty = 0; ty < m_atlas.TilesY(); ++ty) {
            for (uint32_t tx = 0; tx < m_atlas.TilesX(); ++tx) {
                const bool want = m_policy(tx, ty);
                const bool have = m_atlas.IsResident(tx, ty);
                if (want && !have) m_atlas.RequestMap(tx, ty);
                else if (!want && have) m_atlas.RequestUnmap(tx, ty);
            }
        }
        m_atlas.CommitMappings(gpu, &m_fresh);
        m_atlas.FlushResidencyMap(gpu);
        return m_fresh;
    }

    void SetPolicy(TilePolicy p) { m_policy = p ? std::move(p) : policy::All(); }

    // ---- manual drive -----------------------------------------------------------------------
    // Some fields cannot express residency as a pure predicate evaluated fresh each frame. The
    // churn/foam bank is the proving case: it is HYSTERETIC (a tile stays warm for several
    // decay constants after breaking stops, because unmapping the instant it stops would delete
    // the memory the field exists to carry) and it resets wholesale on a time scrub. Those
    // callers drive the atlas directly and keep their own state; they still get the descriptor,
    // the grade signatures, the stats registration and the resident-list discipline.
    //
    // policy::Sticky() exists for the common shape, but a field is never forced through it --
    // a policy that lies is worse than a loop that is honest.
    uint32_t TilesX() const { return m_atlas.TilesX(); }
    uint32_t TilesY() const { return m_atlas.TilesY(); }
    bool IsResident(uint32_t tx, uint32_t ty) const { return m_atlas.IsResident(tx, ty); }
    void RequestMap(uint32_t tx, uint32_t ty) { m_atlas.RequestMap(tx, ty); }
    void RequestUnmap(uint32_t tx, uint32_t ty) { m_atlas.RequestUnmap(tx, ty); }
    void RequestUnmapAll() { m_atlas.RequestUnmapAll(); }
    void CommitMappings(Gpu& gpu, std::vector<uint32_t>* fresh) {
        m_atlas.CommitMappings(gpu, fresh);
        m_atlas.FlushResidencyMap(gpu);
    }
    uint32_t ResidentCount() const { return m_atlas.ResidentCount(); }
    // ---- CPU -> resident tiles ---------------------------------------------------------------
    // Fill this bank's resident tiles from a DENSE CPU image (row-major, srcRowTexels wide).
    // One CopyTiles per tile out of a 64 KB staging region, which is ALSO exactly one
    // DirectStorage request -- so the same path serves a disk-backed loader unchanged when the
    // source stops being a CPU array. Edge tiles that overhang the source are zero-filled;
    // zero is the correct value there because it is what a NULL tile would have read anyway.
    //
    // The resource lives in UNORDERED_ACCESS (TileAtlas2D creates it that way), so this makes
    // the round trip to COPY_DEST and back. Runs on the upload list, which EndUpload() waits
    // on -- correct for a per-solve fill, not for a per-frame one.
    void UploadDenseTiles(Gpu& gpu, const uint8_t* src, uint32_t srcRowTexels,
                          uint32_t texelBytes, const std::vector<uint32_t>& tiles) {
        if (tiles.empty() || src == nullptr) return;
        const uint32_t tw = m_atlas.TileW(), th = m_atlas.TileH();
        const uint64_t tileBytes = TileAtlas2D::kTileBytes;
        // A tile's linear footprint must be exactly one 64 KB tile, or the format's tile shape
        // disagrees with what this copy assumes and silence would be the worst outcome.
        if (uint64_t(tw) * th * texelBytes != tileBytes) {
            Log("[gradebank] %s: tile %ux%u x %uB != %llu -- dense upload skipped",
                m_desc.name.c_str(), tw, th, texelBytes,
                static_cast<unsigned long long>(tileBytes));
            return;
        }
        constexpr uint32_t kBatch = 32;   // 2 MB of staging
        if (!m_stage.res) {
            m_stage = gpu.CreateUploadBuffer(tileBytes * kBatch, L"gradebank.tileStage");
        }
        const uint32_t rowBytes = tw * texelBytes;

        for (size_t base = 0; base < tiles.size(); base += kBatch) {
            const size_t n = (std::min)(size_t(kBatch), tiles.size() - base);
            for (size_t k = 0; k < n; ++k) {
                const uint32_t packed = tiles[base + k];
                const uint32_t tx = packed % m_atlas.TilesX();
                const uint32_t ty = packed / m_atlas.TilesX();
                uint8_t* dst = m_stage.cpu + k * tileBytes;
                for (uint32_t r = 0; r < th; ++r) {
                    const uint32_t sy = ty * th + r;
                    uint8_t* drow = dst + size_t(r) * rowBytes;
                    if (sy >= m_desc.height) {   // overhang below the source
                        memset(drow, 0, rowBytes);
                        continue;
                    }
                    const uint32_t sx = tx * tw;
                    const uint32_t have = (sx < srcRowTexels)
                                              ? (std::min)(tw, srcRowTexels - sx)
                                              : 0u;
                    if (have) {
                        memcpy(drow,
                               src + (size_t(sy) * srcRowTexels + sx) * texelBytes,
                               size_t(have) * texelBytes);
                    }
                    if (have < tw) memset(drow + size_t(have) * texelBytes, 0,
                                          size_t(tw - have) * texelBytes);
                }
            }
            hal::CommandContext up(gpu, gpu.BeginUpload(), hal::Owner::Upload);
            up.Barrier(m_atlas.Res(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                       D3D12_RESOURCE_STATE_COPY_DEST);
            for (size_t k = 0; k < n; ++k) {
                const uint32_t packed = tiles[base + k];
                const D3D12_TILED_RESOURCE_COORDINATE coord{
                    packed % m_atlas.TilesX(), packed / m_atlas.TilesX(), 0, 0};
                const D3D12_TILE_REGION_SIZE size{1, FALSE, 0, 0, 0};
                up.Native()->CopyTiles(
                    m_atlas.Res(), &coord, &size, m_stage.res.Get(), k * tileBytes,
                    D3D12_TILE_COPY_FLAG_LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE);
            }
            up.Barrier(m_atlas.Res(), D3D12_RESOURCE_STATE_COPY_DEST,
                       D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
            gpu.EndUpload();
        }
    }

    uint32_t TileW() const { return m_atlas.TileW(); }
    uint32_t TileH() const { return m_atlas.TileH(); }
    ID3D12Resource* Res() const { return m_atlas.Res(); }

    // Per-tile grade signatures, for DeriveDemand / policy::Derived downstream.
    const std::vector<uint8_t>& Signatures() const { return m_sig; }
    void SetSignature(uint32_t tx, uint32_t ty, uint8_t s) {
        m_sig[size_t(ty) * m_atlas.TilesX() + tx] = s;
    }

    const GradeBankDesc& Desc() const { return m_desc; }
    TileAtlas2D& Atlas() { return m_atlas; }
    const TileAtlas2D& Atlas() const { return m_atlas; }
    uint32_t Srv() const { return m_atlas.Srv(); }
    uint32_t Uav() const { return m_atlas.Uav(); }
    const std::vector<uint32_t>& ResidentList() const { return m_atlas.ResidentList(); }
    uint64_t ResidentBytes() const { return m_atlas.ResidentBytes(); }
    uint64_t VirtualBytes() const { return m_atlas.VirtualBytes(); }

private:
    GradeBankDesc m_desc;
    TileAtlas2D m_atlas;
    bool m_coarseMapped = false;
    TilePolicy m_policy;
    std::vector<uint8_t> m_sig;
    std::vector<uint32_t> m_fresh;
    GpuBuffer m_stage;   // 64 KB-per-tile staging for CPU-sourced fills
};

// ---- Field<Sig>'s two reads of the bank, where the bank is complete ---------------------------
template <uint8_t Sig>
uint32_t Field<Sig>::Srv() const {
    return m_bank ? m_bank->Srv() : UINT32_MAX;
}
template <uint8_t Sig>
bool Field<Sig>::AgreesWithBank() const {
    return m_bank && (m_bank->Desc().gradeSig == Sig);
}

}   // namespace ga
