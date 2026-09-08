#include "sim/OceanCpu.h"

#include "core/Common.h"

#include <algorithm>
#include <cmath>

namespace ga {

namespace {

// The shader's constants. They are FLOAT literals in HLSL (PI = 3.14159265f, G = 9.81f); carried
// here in double on purpose -- house law, the GPU's floats are the approximation and not the
// reference. The resulting phase disagreement with the shader is ~2e-8 relative, i.e. micrometres
// on a 200 m wave, and is swamped by the fp32 w*fTime the shader computes at forecast-hour times.
constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.81;

// ------------------------------------------------------------------- deterministic per-texel draw
// shaders/OceanCompute.hlsl:56. EVERY operation is uint32 wraparound -- that is not incidental, it
// IS the hash. Written with explicit uint32_t so the C++ and the HLSL and proofs/ocean_cpu.py all
// overflow at the same place; gate 4a compares all three against one golden table.
struct Uint2 { uint32_t x, y; };

Uint2 Pcg2d(Uint2 v) {
    v.x = v.x * 1664525u + 1013904223u;
    v.y = v.y * 1664525u + 1013904223u;
    v.x += v.y * 1664525u;
    v.y += v.x * 1664525u;
    v.x ^= v.x >> 16u;
    v.y ^= v.y >> 16u;
    v.x += v.y * 1664525u;
    v.y += v.x * 1664525u;
    v.x ^= v.x >> 16u;
    v.y ^= v.y >> 16u;
    return v;
}

// shaders/OceanCompute.hlsl:65. Box-Muller on 24 bits of each hash lane. The 1e-6 floor on u1 is
// the shader's guard against log(0); it is reproduced exactly because it is part of the draw.
void GaussianPair(uint32_t ix, uint32_t iy, uint32_t seed, uint32_t cascade, double* gx,
                  double* gy) {
    const Uint2 h = Pcg2d({ix * 3u + (seed + cascade * 197u), iy * 5u + seed * 13u});
    const double u1 = std::max((h.x & 0xFFFFFFu) / 16777216.0, 1e-6);
    const double u2 = (h.y & 0xFFFFFFu) / 16777216.0;
    const double r = std::sqrt(-2.0 * std::log(u1));
    *gx = r * std::cos(2.0 * kPi * u2);
    *gy = r * std::sin(2.0 * kPi * u2);
}

// ------------------------------------------------------------------------------- the spectrum
// shaders/OceanCompute.hlsl:74 == SeaState.cpp:27. Unit shape only; the level is specScale, which
// SeaState::MakePartition normalises numerically so m0 hits the partition's Hs.
double ShapeJonswap(double f, double fp, double gamma) {
    if (f <= 1e-4) return 0.0;
    const double r = fp / f;
    const double sigma = (f <= fp) ? 0.07 : 0.09;
    const double d = (f - fp) / (sigma * fp);
    return std::pow(f, -5.0) * std::exp(-1.25 * r * r * r * r) *
           std::pow(gamma, std::exp(-0.5 * d * d));
}

// shaders/OceanCompute.hlsl:83, Abramowitz & Stegun 7.1.26, |error| <= 1.5e-7 absolute.
//
// This is DELIBERATELY not std::erf. std::erf is more accurate, and being more accurate here would
// mean drawing a different bin amplitude than the GPU drew -- a different ocean, which is the one
// thing this class exists not to be. The difference is small but it is not zero: measured on a
// Tp 14 s swell (proofs/ocean_cpu.py's machinery), swapping in the exact erf moves cascade 0's
// TOTAL variance by 2.6e-8 relative, individual bins above 1e-6 of the peak by up to 5.9e-4, and
// far-tail bins by up to 1.5e-2. So the approximation is the contract, not a shortcut.
//
// THE CANCELLATION, and why the reported residual has a floor. In a narrow swell's far tail both
// bin-edge erfs saturate to within 1e-15 of 1, and their DIFFERENCE -- the bin's whole energy --
// is then a catastrophic cancellation: about 400 of a cascade-0 swell's 496 live bins have an erf
// difference below 1e-12, so their variance carries only a handful of significant digits even in
// double. Those bins are always dropped by the truncation (they are ~1e-11 of the total), so the
// FIELD is unaffected -- C++ and numpy agree on it to 5e-9 relative -- but they sit in the
// DENOMINATOR of ResidualVarianceFraction, which is therefore only good to about 5e-5 absolute
// across libm implementations. On the GPU it is far worse: fp32's ulp near 1.0 is 1.2e-7, so in
// the shader those tail bins are pure quantisation noise. Nothing downstream cares at that level,
// but the number below the fifth decimal of a residual is not real, and should not be quoted.
double Erf(double x) {
    const double s = (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0);   // HLSL sign(): sign(0) == 0
    x = std::abs(x);
    const double t = 1.0 / (1.0 + 0.3275911 * x);
    const double y = 1.0 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t
                             - 0.284496736) * t + 0.254829592) * t * std::exp(-x * x);
    return s * y;
}

// The band this cascade owns and the partition set to evaluate over it.
struct Band {
    const PartParam* parts;
    int count;
    double patchL, kLo, kHi;
};

// shaders/OceanCompute.hlsl:100. Directional variance for one k bin, ported line for line.
//
// ** THE NARROW-SWELL TRAP, and why the port must be exact. ** A Tp 14 s swell's Gaussian peak
// (sigma_f = 4.0 mHz, the floor SeaState::MakePartition clamps to) is NARROWER than one bin's
// frequency footprint on a 756 m patch (dfdk * dk = 14.5 mHz at that peak), and its cos^2s lobe
// (sigma_theta = sqrt(2/60) = 10.5 deg) is narrower than a bin's angular half-cell there
// (dk/2k = 11.6 deg). Point-sampling either one makes a bin's energy depend on where the grid
// happens to fall relative to the peak, so m0 swings with Tp instead of staying put. Both
// directions are therefore INTEGRATED over the bin: closed-form erf for the Gaussian swell and for
// the angular lobe, point * width for the broad JONSWAP where sampling is fine. The angular erf
// difference is the cell's exact energy FRACTION, which is why the partition's dirNorm never
// appears here -- SeaState::MakePartition computes it and this path is dead to it, in the shader
// too.
//
// proofs/ocean_cpu.py gate 1 measures the difference over a Tp 9-18 s sweep of a Hs 1.5 m swell:
// the erf form holds m0 to within +3.9%, radial point-sampling swings -42.6% to +40.6%, and
// angular point-sampling -52.4% to +2.3%. That 83-point swing IS the trap the shader's comment
// says cost the first storm render a third of its Hs.
double BinVariance(double kx, double kz, const Band& b) {
    const double kLen = std::sqrt(kx * kx + kz * kz);
    if (kLen < 1e-5 || kLen < b.kLo || kLen >= b.kHi) return 0.0;
    const double w = std::sqrt(kG * kLen);
    const double f = w / (2.0 * kPi);
    const double dfdk = kG / (4.0 * kPi * w);
    const double dk = 2.0 * kPi / b.patchL;

    double acc = 0.0;
    for (int i = 0; i < b.count; ++i) {
        const PartParam& p = b.parts[i];
        double eF;   // integral of S(f) over this bin's radial frequency extent
        if (p.gamma > 0.0f) {
            eF = p.specScale * ShapeJonswap(f, p.fp, p.gamma) * dfdk * dk;
        } else {
            const double hw = 0.5 * dfdk * dk;
            const double d0 = (f - hw - p.fp) / p.sigF;
            const double d1 = (f + hw - p.fp) / p.sigF;
            // sqrt(2*pi)/2 = 1.2533141; erf argument scale 1/sqrt(2)
            eF = p.specScale * p.sigF * 1.2533141 *
                 (Erf(d1 * 0.70710678) - Erf(d0 * 0.70710678));
        }
        // The cos^2s lobe is ~exp(-s*dTh^2/4), a Gaussian of sigma = sqrt(2/s); integrate it over
        // this bin's angular cell (half-width dk/(2k), the cell's arc measured in radians).
        const double cosD = (kx * p.dirToX + kz * p.dirToZ) / kLen;
        const double crossD = (kx * p.dirToZ - kz * p.dirToX) / kLen;
        const double dTh = std::atan2(crossD, cosD);
        const double sigTh = std::sqrt(2.0 / p.spreadS);
        const double hTh = 0.5 * dk / kLen;
        const double frac = 0.5 * (Erf((dTh + hTh) / (sigTh * 1.41421356)) -
                                   Erf((dTh - hTh) / (sigTh * 1.41421356)));
        acc += eF * frac;
    }
    return acc;
}

// shaders/OceanCompute.hlsl:140. The DFT's index-to-wavenumber map: the top half of the index
// range is the negative half of k. Index kN/2 is its own mirror -- see the guard-band note in
// SetSeaState.
double SignedIndex(int i) {
    return (i < OceanCpu::kN / 2) ? static_cast<double>(i)
                                  : static_cast<double>(i) - static_cast<double>(OceanCpu::kN);
}

}  // namespace

void OceanCpu::SetSeaState(const PartParam* parts, int count, uint32_t seed, const float patchL[3],
                           const float kLo[3], const float kHi[3], double lambda) {
    m_lambda = lambda;
    m_ready = false;

    // OceanFft's constant buffer holds four partitions; more than that and the shader reads past
    // gPart. Clamp identically so the two agree about which partitions exist.
    PartParam use[4];
    const int n = std::max(0, std::min(count, 4));
    for (int i = 0; i < n; ++i) use[i] = parts[i];

    // |A|^2 + |B|^2: the pair's TIME-AVERAGED contribution to the height variance, up to the
    // factor 2 applied below. Time-averaged because hk(k,t) = A e^{iwt} + conj(B) e^{-iwt} carries
    // a cross term 2*Re[A*B*e^{2iwt}] that beats at 2w -- ranking on an instantaneous amplitude
    // would reshuffle the retained set every frame, which is exactly what a stateless synthesis
    // must never do.
    const auto energy = [](const Bin& b) {
        return b.ar * b.ar + b.ai * b.ai + b.br * b.br + b.bi * b.bi;
    };

    for (int c = 0; c < kCascades; ++c) {
        const Band band{use, n, patchL[c], kLo[c], kHi[c]};
        std::vector<Bin> all;
        all.reserve(4096);

        // THE PRECONDITION FOR PAIRING, checked exactly and once. Summing conjugate PAIRS needs
        // WaveK(mirror) == -WaveK(k), which fails only where a signed index is kN/2 (its own
        // negation). The smallest |k| anywhere on those two lines is |sx| = kN/2 with sy = 0, i.e.
        // the grid's Nyquist wavenumber pi*kN/L exactly -- so BinVariance, which zeroes everything
        // at or above kHi, empties both lines precisely when kHi <= pi*kN/L. That is the 0.9 guard
        // band OceanFft::Init sets, and this is a scalar comparison rather than a sampled sweep
        // because a sweep over the retained half-plane could not see the whole Nyquist row.
        const double kNyquist = kPi * kN / patchL[c];
        if (kHi[c] > kNyquist) {
            Log("[oceancpu] cascade %d: kHi %.4f has crossed the Nyquist wavenumber %.4f, so the "
                "kN/2 row and column carry energy where hk(-k) != conj(hk(k)) -- the conjugate "
                "pair sum is no longer exact and the reconstructed field will not be real",
                c, kHi[c], kNyquist);
        }

        // ONE walk of the full 256^2 grid, keeping one texel of each conjugate pair. The
        // representative is the half-plane sy > 0 (plus the sy == 0, sx > 0 ray) -- the same
        // choice for every seed and every sea state, so the retained list is reproducible.
        for (int iy = 0; iy < kN; ++iy) {
            const double sy = SignedIndex(iy);
            for (int ix = 0; ix < kN; ++ix) {
                const double sx = SignedIndex(ix);
                if (!(sy > 0.0 || (sy == 0.0 && sx > 0.0))) continue;

                const double kx = 2.0 * kPi / band.patchL * sx;
                const double kz = 2.0 * kPi / band.patchL * sy;
                // CsInitSpectrum: ap from V(+k) with THIS texel's Gaussian, am from V(-k) -- the
                // continuous negation, not the mirror texel's k -- with the MIRROR texel's
                // Gaussian. Both halves matter: am/ap differ wherever the spectrum is directional,
                // and gm is what makes h0(-k) here equal h0(+k) stored at the mirror texel.
                const double vp = BinVariance(kx, kz, band);
                const double vm = BinVariance(-kx, -kz, band);
                if (vp <= 0.0 && vm <= 0.0) continue;

                const uint32_t mx = static_cast<uint32_t>((kN - ix) % kN);
                const uint32_t my = static_cast<uint32_t>((kN - iy) % kN);
                double gpx = 0, gpy = 0, gmx = 0, gmy = 0;
                GaussianPair(static_cast<uint32_t>(ix), static_cast<uint32_t>(iy), seed,
                             static_cast<uint32_t>(c), &gpx, &gpy);
                GaussianPair(mx, my, seed, static_cast<uint32_t>(c), &gmx, &gmy);
                const double ap = 0.5 * std::sqrt(std::max(vp, 0.0));
                const double am = 0.5 * std::sqrt(std::max(vm, 0.0));

                Bin b;
                b.kx = kx;
                b.kz = kz;
                const double kLen = std::sqrt(kx * kx + kz * kz);
                b.w = std::sqrt(kG * kLen);
                b.invK = 1.0 / kLen;
                b.ar = ap * gpx;
                b.ai = ap * gpy;
                b.br = am * gmx;
                b.bi = am * gmy;
                all.push_back(b);
            }
        }

        // Largest first. stable_sort so two bins of identical energy keep grid order and the
        // retained set is bit-identical across compilers -- headless dumps are diffed.
        std::stable_sort(all.begin(), all.end(),
                         [&](const Bin& a, const Bin& b) { return energy(a) > energy(b); });

        double total = 0.0;
        for (const Bin& b : all) total += energy(b);

        // Adaptive N: take pairs until the tail left behind is under kResidualTarget of the
        // cascade's own variance, capped at kMaxBins. One law, no branch on sea state -- a calm
        // sea simply satisfies it sooner. What it actually cost is reported, never assumed.
        double kept = 0.0;
        size_t take = 0;
        const double want = (1.0 - kResidualTarget) * total;
        while (take < all.size() && take < static_cast<size_t>(kMaxBins) && kept < want) {
            kept += energy(all[take]);
            ++take;
        }
        all.resize(take);
        all.shrink_to_fit();

        // A pair contributes 2*(|A|^2 + |B|^2) to the height variance: its two texels carry
        // |hk(k)|^2 and |hk(-k)|^2 = |hk(k)|^2, and <|hk|^2>_t = |A|^2 + |B|^2.
        m_varAll[c] = 2.0 * total;
        m_varKept[c] = 2.0 * kept;
        m_bin[c] = std::move(all);
    }

    m_ready = true;
    Log("[oceancpu] %d partitions, seed %08x: retained %d/%d/%d pairs, residual %.3f/%.3f/%.3f%%, "
        "Hs %.3f m", n, seed, RetainedBins(0), RetainedBins(1), RetainedBins(2),
        100.0 * ResidualVarianceFraction(0), 100.0 * ResidualVarianceFraction(1),
        100.0 * ResidualVarianceFraction(2),
        4.0 * std::sqrt(std::max(Variance(0) + Variance(1) + Variance(2), 0.0)));
}

// The truncated direct sum. Per retained pair this is one rotor (the same e^{+iwt} CsModulate
// applies) times one spatial phasor (the e^{+ik.x} CsFft synthesises), and then the eight linear
// spectral fields are all real multiples of that product's real and imaginary parts:
//
//     P = hk * e^{i k.x}
//     h  = 2 Re[P]                        Dx = 2 (kx/k) Im[P]      Dz = 2 (kz/k) Im[P]
//     hx = -2 kx Im[P]                    hz = -2 kz Im[P]
//     Jxx = 2 (kx^2/k) Re[P]              Jzz = 2 (kz^2/k) Re[P]   Jxz = 2 (kx kz/k) Re[P]
//
// (Only the first row is needed here; the rest is written down because it is the same P, so a
// normal or a Jacobian costs no extra transcendental if this ever grows one. Derivation: the
// factor 2 is the conjugate pair; Dx = 2 Re[(-i kx/k) hk e^{ikx}] = 2 (kx/k) Im[P] because
// Re[-i z] = Im[z], and that -i is the chop sign read off CsModulate's float2(hk.y, -hk.x).)
void OceanCpu::Displacement(double wx, double wz, double tSec, double out[3]) const {
    double dx = 0.0, h = 0.0, dz = 0.0;
    if (m_ready) {
        for (int c = 0; c < kCascades; ++c) {
            for (const Bin& b : m_bin[c]) {
                const double wt = b.w * tSec;
                const double cw = std::cos(wt), sw = std::sin(wt);
                // hk = h0(+k) e^{+iwt} + conj(h0(-k)) e^{-iwt}   (CsModulate, verbatim)
                const double hkr = (b.ar * cw - b.ai * sw) + (b.br * cw - b.bi * sw);
                const double hki = (b.ar * sw + b.ai * cw) - (b.br * sw + b.bi * cw);

                // P = hk * e^{+i k.x}, the spatial phasor CsFft's synthesis transform supplies.
                const double th = b.kx * wx + b.kz * wz;
                const double ct = std::cos(th), st = std::sin(th);
                const double pRe = hkr * ct - hki * st;
                const double pIm = hkr * st + hki * ct;

                h += 2.0 * pRe;
                const double s = 2.0 * pIm * b.invK;
                dx += s * b.kx;
                dz += s * b.kz;
            }
        }
    }
    // CsAssemble: float4(fLambda * dx, h, fLambda * dz, 0). The height channel is NOT scaled.
    out[0] = m_lambda * dx;
    out[1] = h;
    out[2] = m_lambda * dz;
}

double OceanCpu::ResidualVarianceFraction(int cascade) const {
    if (cascade < 0 || cascade >= kCascades || m_varAll[cascade] <= 0.0) return 0.0;
    return 1.0 - m_varKept[cascade] / m_varAll[cascade];
}

int OceanCpu::RetainedBins(int cascade) const {
    if (cascade < 0 || cascade >= kCascades) return 0;
    return static_cast<int>(m_bin[cascade].size());
}

double OceanCpu::Variance(int cascade) const {
    if (cascade < 0 || cascade >= kCascades) return 0.0;
    return m_varKept[cascade];
}

}  // namespace ga
