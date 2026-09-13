// ================================================================================================
//  FieldExpr - M9h: THE SPARSE MULTIVECTOR FIELD AS A TYPE. (Was GradeField.h; M12 step 3e
//  split the GPU bank out to hal/GradeBank.h -- see the end of this banner.)
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
//
//  M12 STEP 3e, THE SPLIT. This header is the EXPRESSION LAYER: the grade bits, the descriptor
//  (a bank's frame contract, and now SameGround over it), the residency policies, Field<Sig>
//  and the product / sum / grad expressions whose types the algebra decides -- nothing here
//  touches a tile. The GPU bank (GradeBank: the reserved atlas, the mip chain, the CopyTiles
//  fill) moved verbatim to hal/GradeBank.h, where the tiles are; it includes this file, and
//  Field<Sig>'s two bank reads (Srv, AgreesWithBank) are defined there, where the bank is
//  complete -- the handle names the bank only through a pointer. core/GradeField.h forwards to
//  both halves so no includer changed. Cl2ProductSignature stays in hal/TileAtlas.h beside the
//  self-test that pins it, and is included from there as before.
// ================================================================================================
#pragma once

#include "core/Common.h"
#include "hal/TileAtlas.h"

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

    // M12 step 3e: two banks name the same ground iff their frame contract is the same POD --
    // extent, origin, texel size and axis sense (Lattice::SameGround, for a bank's own dense
    // frame). What Demand() refuses to combine across.
    bool SameGround(const GradeBankDesc& o) const {
        return width == o.width && height == o.height && orgX == o.orgX && orgZ == o.orgZ &&
               metersPerTexel == o.metersPerTexel && vNorth == o.vNorth;
    }
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
// out MAY be non-zero only where Cl2ProductSignature(a, b) is -- no data read. A signature is
// a BOUND on the output's grades, not a promise of a non-zero: cancellation can still give
// zero inside it (an outside review, 2026-09-13, on a comment that promised more). This is
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

class GradeBank;   // hal/GradeBank.h: the GPU half; the handle below holds it by pointer

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
    // The two reads of the bank: defined in hal/GradeBank.h, where the bank is complete.
    uint32_t Srv() const;
    // A field carrying a grade the bank does not is a contradiction; catch it at wiring time.
    bool AgreesWithBank() const;

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

    // The tiles the result can be non-zero on -- algebra, not data. M12 step 3e: the two
    // operands' grids are combined index by index, which is a statement about the ground only
    // when both banks tile the same one (GradeBankDesc::SameGround: extent, origin, texel
    // size, axis sense). A pair that does not would need a resample node, so it is refused --
    // an empty demand, said once. Every shipped operand pair shares its ground.
    TilePolicy Demand() const {
        if (!lhs.Bank()->Desc().SameGround(rhs.Bank()->Desc())) {
            static bool said = false;
            if (!said) {
                said = true;
                Log("[field] %s * %s: the operands are not on the same ground -- demand refused",
                    lhs.Bank()->Desc().name.c_str(), rhs.Bank()->Desc().name.c_str());
            }
            return policy::None();
        }
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
    // M12 (after an outside review, 2026-09-13): the vector derivative is the geometric product
    // of a VECTOR with the field, so its grades are Cl2ProductSignature(kG1, A) -- a scalar's
    // gradient is a vector, a vector's is scalar + bivector, a bivector's is a vector. The old
    // form `(A & kG1) ? (kG0 | kG2) : A` was right for the vector case its one assertion tested
    // and wrong for the other six signatures (a scalar stayed a scalar).
    static constexpr uint8_t kSig = Cl2ProductSignature(kG1, A);
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
static_assert(GradExpr<kG0>::kSig == kG1, "grad s is a vector");
static_assert(GradExpr<kG2>::kSig == kG1, "grad B is a vector (Cl(2): e_i d_i B)");
static_assert(GradExpr<kG0 | kG2>::kSig == kG1, "grad (s + B) is a vector");
static_assert(GradExpr<kG0 | kG1>::kSig == (kG0 | kG1 | kG2), "grad (s + v) fills every grade");

}   // namespace ga
