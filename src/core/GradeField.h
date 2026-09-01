// ================================================================================================
//  GradeField - M9h: THE SPARSE MULTIVECTOR FIELD AS A TYPE.
//
//  The user's sketch, verbatim:
//
//      SparseGATypeA dataA = new SparseGATypeA(plugin.LoadFileTypeX("filepath1"));
//      SparseGATypeB dataB = new SparseGATypeB(plugin.LoadFileTypeY("filepath2"));
//      var dataC = dataA * dataB;
//
//  ...so that a new dataset costs a LOADER and a grade declaration, not a bespoke pipeline.
//  The sparse structure is the product; the D3D12 renderer is how we show it is worth having.
//
//  Three things make that sketch work here, and two of them already existed:
//
//  1. THE TYPE IS THE ALGEBRA. Cl2ProductSignature is the selftest-pinned Cayley closure over
//     grade-signature bits. Made constexpr (TileAtlas.h), it computes the RESULT TYPE of a
//     product at compile time: Field<A> * Field<B> is a Field<Cl2ProductSignature(A, B)>. No
//     declared result type, no runtime grade check, and a grade that cannot arise is a
//     compile error rather than a silent zero. C# would need generic gymnastics for this;
//     C++ gets it from constexpr.
//
//  2. THE SPARSITY IS THE ALGEBRA TOO. ResidencyManager::DeriveDemand already applies that same
//     closure per tile: a derived field needs tiles exactly where its inputs' product signature
//     is non-zero -- decided WITHOUT reading data. That is the load-bearing claim of the whole
//     atlas (GAMEPLAN 4.1), and the tile self-test proves the platform honours it: reads from
//     NULL tiles return zero, so a consumer adds sparse contributions unconditionally.
//
//  3. THE FRAME IS DECLARED. A field that crosses into another engine registers a GA AST edge
//     (GaAst.h) carrying frame, units, range and flip -- validated at boot, printed every run.
//     That is what makes "add a loader, see it on the renderer" safe rather than hopeful: the
//     orientation class of bug this project keeps hitting is caught by the validator, not by a
//     render three sessions later.
//
//  WHAT THIS FILE IS NOT (yet): it does not evaluate. Building `a * b` records an EXPRESSION;
//  materializing it into a bank is the caller's step. That is deliberate -- eager evaluation
//  would allocate a full atlas per intermediate, and the whole point is that most tiles do not
//  exist. Fusing an expression into one dispatch over the resident list is the next milestone.
//
//  Composition over inheritance, on purpose: what varies between banks is DATA (dims, format,
//  grade, policy), not behaviour, and virtual dispatch inside a per-tile residency loop buys
//  nothing. The one seam that is genuinely polymorphic -- how a tile gets filled -- is a
//  std::function, not a base class.
// ================================================================================================
#pragma once

#include "core/Common.h"
#include "core/Gpu.h"
#include "core/TileAtlas.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace ga {

// ---- grade bits of Cl(2), matching Cl2ProductSignature's convention -------------------------
inline constexpr uint8_t kG0 = 0b001;   // scalar    -- SSH, depth, foam, temperature
inline constexpr uint8_t kG1 = 0b010;   // vector    -- current, wind, wave momentum
inline constexpr uint8_t kG2 = 0b100;   // bivector  -- vorticity, orbital plane, EM F

// A Cl(2) multivector is exactly one RGBA16F texel (s, v.x, v.y, b) -- the packing accident
// GAMEPLAN 4.1 calls out, and the reason a grade bank is a plain 2D reserved texture.
constexpr uint32_t GradeChannels(uint8_t sig) {
    return ((sig & kG0) ? 1u : 0u) + ((sig & kG1) ? 2u : 0u) + ((sig & kG2) ? 1u : 0u);
}

// ================================================================================================
//  The descriptor. Everything about a bank that is DATA. Immutable by convention: a bank's
//  identity is its descriptor, so a cache key or an AST edge can be derived from it.
// ================================================================================================
struct GradeBankDesc {
    std::string name;                 // "wave.solved", "terrain.bed.dev", "globe.waves"
    uint32_t width = 0, height = 0;   // virtual domain, texels
    DXGI_FORMAT fmt = DXGI_FORMAT_R16G16B16A16_FLOAT;
    uint8_t gradeSig = kG0;           // which grades this bank carries
    // M9h: > 1 builds the chain. A chained bank gets a pinned floor and a residency map, so a
    // sample can never miss and detail changes without any code path changing. Costs one tile
    // per page for the floor -- provided the chain is not truncated (docs/SPARSE_GA.md 18).
    uint32_t mipLevels = 1;
    // M9j: > 1 makes this a PAGED bank -- one reserved array whose slices are pages of the
    // shared address space. Slice 0 is the bank's own domain; further slices are whatever the
    // compositor puts there (a region, a survey, a second body).
    uint32_t arraySlices = 1;

    // The frame contract -- the same quantities the GA AST edge will publish, kept here so the
    // edge can be registered FROM the descriptor instead of hand-written per call site.
    double orgX = 0.0, orgZ = 0.0;    // world metres at texel (0,0)
    double metersPerTexel = 1.0;
    bool vNorth = true;               // does the second axis grow northward?
    const char* units = "";
    const char* range = "";
};

// ================================================================================================
//  Residency policy: a PURE predicate over tiles, and the combinators that compose them.
//
//  This is the pipeline seam. A policy never reads field data -- it answers "could this tile be
//  non-zero", which is exactly the question the Cayley closure answers for derived fields. The
//  combinators are the F#-shaped part: small total functions, composed, no inheritance.
// ================================================================================================
using TilePolicy = std::function<bool(uint32_t tx, uint32_t ty)>;

namespace policy {

inline TilePolicy All() {
    return [](uint32_t, uint32_t) { return true; };
}
inline TilePolicy None() {
    return [](uint32_t, uint32_t) { return false; };
}
inline TilePolicy Not(TilePolicy p) {
    return [p = std::move(p)](uint32_t x, uint32_t y) { return !p(x, y); };
}
inline TilePolicy And(TilePolicy a, TilePolicy b) {
    return [a = std::move(a), b = std::move(b)](uint32_t x, uint32_t y) {
        return a(x, y) && b(x, y);
    };
}
inline TilePolicy Or(TilePolicy a, TilePolicy b) {
    return [a = std::move(a), b = std::move(b)](uint32_t x, uint32_t y) {
        return a(x, y) || b(x, y);
    };
}
// Tile-space rectangle, half-open [x0, x1) x [y0, y1).
inline TilePolicy Rect(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1) {
    return [x0, y0, x1, y1](uint32_t x, uint32_t y) {
        return x >= x0 && x < x1 && y >= y0 && y < y1;
    };
}
// A precomputed per-tile mask (wet/dry, cloud fraction, jet envelope -- the physics policies
// the existing field tenants already own, lifted into the same currency).
inline TilePolicy Mask(std::vector<uint8_t> m, uint32_t tilesX) {
    return [m = std::move(m), tilesX](uint32_t x, uint32_t y) {
        const size_t i = size_t(y) * tilesX + x;
        return i < m.size() && m[i] != 0;
    };
}
// THE ALGEBRAIC ONE: a derived field's tiles, from its inputs' per-tile grade signatures.
// out is non-zero exactly where Cl2ProductSignature(a, b) is -- no data read. This is
// DeriveDemand's rule expressed as a policy so it composes with the rest.
inline TilePolicy Derived(std::vector<uint8_t> sigA, std::vector<uint8_t> sigB,
                          uint32_t tilesX) {
    return [a = std::move(sigA), b = std::move(sigB), tilesX](uint32_t x, uint32_t y) {
        const size_t i = size_t(y) * tilesX + x;
        if (i >= a.size() || i >= b.size()) return false;
        return Cl2ProductSignature(a[i], b[i]) != 0;
    };
}

}   // namespace policy

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
    using FillFn = std::function<void(Gpu&, ID3D12GraphicsCommandList*,
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
    void BuildChain(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
                    ID3D12GraphicsCommandList* cl) {
        if (m_desc.mipLevels < 2 || !m_coarseMapped) return;
        m_atlas.BuildMips(gpu, sc, shaderDir, cl);
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
            ID3D12GraphicsCommandList* cl = gpu.BeginUpload();
            D3D12_RESOURCE_BARRIER b{};
            b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            b.Transition.pResource = m_atlas.Res();
            b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
            b.Transition.StateBefore = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
            b.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_DEST;
            cl->ResourceBarrier(1, &b);
            for (size_t k = 0; k < n; ++k) {
                const uint32_t packed = tiles[base + k];
                const D3D12_TILED_RESOURCE_COORDINATE coord{
                    packed % m_atlas.TilesX(), packed / m_atlas.TilesX(), 0, 0};
                const D3D12_TILE_REGION_SIZE size{1, FALSE, 0, 0, 0};
                cl->CopyTiles(m_atlas.Res(), &coord, &size, m_stage.res.Get(), k * tileBytes,
                              D3D12_TILE_COPY_FLAG_LINEAR_BUFFER_TO_SWIZZLED_TILED_RESOURCE);
            }
            std::swap(b.Transition.StateBefore, b.Transition.StateAfter);
            cl->ResourceBarrier(1, &b);
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

// ================================================================================================
//  Field<Sig> -- the typed handle. Sig is a COMPILE-TIME grade signature, so the algebra below
//  resolves result types statically.
//
//  A Field does not own its bank (banks outlive expressions); it is a lightweight handle plus
//  the type-level grade. Copying one is free.
// ================================================================================================
template <uint8_t Sig>
class Field {
public:
    static constexpr uint8_t kSig = Sig;

    Field() = default;
    explicit Field(GradeBank* bank) : m_bank(bank) {}

    GradeBank* Bank() const { return m_bank; }
    bool Valid() const { return m_bank != nullptr; }
    uint32_t Srv() const { return m_bank ? m_bank->Srv() : UINT32_MAX; }

    // A field carrying a grade the bank does not is a contradiction; catch it at wiring time.
    bool AgreesWithBank() const {
        return m_bank && (m_bank->Desc().gradeSig == Sig);
    }

private:
    GradeBank* m_bank = nullptr;
};

// ================================================================================================
//  The expression layer. `a * b` does NOT dispatch -- it records what the product IS, with its
//  grade already decided by the algebra. Materialization (allocate the output bank with
//  policy::Derived, dispatch one kernel over its resident list) is a separate, explicit step,
//  so intermediates never allocate tiles that the closure says are zero.
// ================================================================================================
template <uint8_t A, uint8_t B>
struct GeometricProductExpr {
    static constexpr uint8_t kSig = Cl2ProductSignature(A, B);
    Field<A> lhs;
    Field<B> rhs;

    // The tiles the result can be non-zero on -- algebra, not data.
    TilePolicy Demand() const {
        return policy::Derived(lhs.Bank()->Signatures(), rhs.Bank()->Signatures(),
                               lhs.Bank()->Atlas().TilesX());
    }
};

template <uint8_t A, uint8_t B>
constexpr GeometricProductExpr<A, B> operator*(const Field<A>& a, const Field<B>& b) {
    return GeometricProductExpr<A, B>{a, b};
}

// Addition keeps every grade either side carries -- the union, not the Cayley product.
template <uint8_t A, uint8_t B>
struct SumExpr {
    static constexpr uint8_t kSig = static_cast<uint8_t>(A | B);
    Field<A> lhs;
    Field<B> rhs;
};

template <uint8_t A, uint8_t B>
constexpr SumExpr<A, B> operator+(const Field<A>& a, const Field<B>& b) {
    return SumExpr<A, B>{a, b};
}

// grad of a vector field splits by grade: divergence (scalar) + vorticity (bivector), which is
// GAMEPLAN application 2 -- one RGBA texel, div in grade 0 and curl in grade 2, exactly what
// FieldSet.h anticipated. The TYPE says so.
template <uint8_t A>
struct GradExpr {
    static constexpr uint8_t kSig = static_cast<uint8_t>((A & kG1) ? (kG0 | kG2) : A);
    Field<A> src;
};

template <uint8_t A>
constexpr GradExpr<A> grad(const Field<A>& f) {
    return GradExpr<A>{f};
}

// ---- compile-time proofs of the type algebra (free; they cost nothing at runtime) -----------
static_assert(GeometricProductExpr<kG1, kG1>::kSig == (kG0 | kG2),
              "v * v = dot + wedge: scalar plus bivector");
static_assert(GeometricProductExpr<kG0, kG1>::kSig == kG1, "scalar * vector stays a vector");
static_assert(GeometricProductExpr<kG2, kG2>::kSig == kG0, "bivector squares to a scalar");
static_assert(GradExpr<kG1>::kSig == (kG0 | kG2), "grad v = div (g0) + curl (g2)");

}   // namespace ga
