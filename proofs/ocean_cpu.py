# ==================================================================================================
#  proofs/ocean_cpu.py - the cascade spectrum, re-derived in numpy, as the gate on src/sim/OceanCpu.
#
#  WHY THIS FILE EXISTS.  OceanCpu claims something strong: that a truncated direct sum on the CPU
#  reproduces THE SAME REALIZATION the GPU's three FFT cascades draw -- the same individual crests,
#  not a statistically similar sea.  A merely similar sea would put a hull on water the render
#  never drew, and nothing would ever look wrong enough to find.  The claim has four independent
#  failure modes and one gate each:
#
#    1  SPECTRUM     does the summed per-bin variance equal the analytic m0 of the partition set
#                    restricted to that cascade's k band?  (the erf bin-integration, the cure for
#                    the narrow-swell trap, is what makes this true)
#    2  ENERGY       does 4 sqrt(total variance) land on the partition set's target Hs -- in the
#                    ensemble, and how far does ONE realization scatter from it?
#    3  TRUNCATION   what variance fraction does keeping the largest-N bins actually drop?
#                    (a NUMBER, measured, per cascade -- never asserted to be small)
#    4  REALIZATION  do pcg2d / GaussianPair agree bit-for-bit with the C++; does the truncated
#                    direct sum agree with a real 2-D inverse FFT of the same packed spectrum
#                    (the GPU's own algorithm); and does the C++ agree with this file?
#
#  Gate 4b is the one that pins every SIGN and SCALE at once: the numpy pipeline below is
#  CsInitSpectrum -> CsModulate -> the unnormalised e^{+i} inverse transform -> CsAssemble,
#  transcribed from shaders/OceanCompute.hlsl, and the direct sum has to match it texel for texel.
#  A flipped choppy sign, a flipped FFT sign, a dropped factor of two, a wrong Hermitian partner --
#  each one breaks that comparison, and each one is run here as a NEGATIVE CONTROL that has to
#  fail.  This repo has been bitten by a gate that computed f(a) - f(a) and passed for any f; a
#  control that advances both sides of a comparison is exactly that bug, so every control below
#  perturbs ONE side only.
#
#  Deterministic (the "randomness" is an integer hash of the texel index).  numpy only.
#  Prints PASS/FAIL per assertion, exits nonzero on any failure.
#
#  Usage:  py proofs/ocean_cpu.py [path/to/cxx_dump.txt]
#          The optional argument is the dump written by a standalone harness around
#          src/sim/OceanCpu.cpp (see GATE 5); without it gates 1-4 still run in full.
# ==================================================================================================
import math
import sys

import numpy as np

G = 9.81
PI = math.pi
N = 256                      # OceanFft::kN
LAMBDA = 1.1                 # OceanFft::m_lambda -- the choppy scale CsAssemble multiplies in
RESIDUAL_TARGET = 0.01       # OceanCpu::kResidualTarget
MAX_BINS = 4096              # OceanCpu::kMaxBins

np.seterr(over="ignore")     # uint32 wraparound in pcg2d is the intent, not an error

FAILS = []


def check(name, ok, detail=""):
    print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, (" -- " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def check_fails(name, ok, detail=""):
    """A NEGATIVE control: the deliberately wrong variant must NOT pass."""
    print("[%s] %s%s" % ("PASS" if not ok else "FAIL", "(neg) " + name,
                         (" -- " + detail) if detail else ""))
    if ok:
        FAILS.append("(neg) " + name)


# ==================================================================================================
#  The cascade geometry, exactly as OceanFft::Init derives it (src/core/OceanFft.cpp:150-158).
#  Restated here rather than imported: a proof that imports the thing it grades proves nothing.
#  (OceanCpu takes them as SetSeaState parameters for the opposite reason -- so the ENGINE cannot
#  hold two copies of a band cut that disagree.)
# ==================================================================================================
PATCH_L = np.array([756.0, 186.0, 47.0])
CUT01 = 2.0 * PI / 60.0
CUT12 = 2.0 * PI / 12.0
BAND_LO = np.array([2.0 * PI / PATCH_L[0], CUT01, CUT12])
BAND_HI = np.array([CUT01, CUT12, 0.9 * PI * N / PATCH_L[2]])


# ==================================================================================================
#  The port.  Every function mirrors one in shaders/OceanCompute.hlsl and one in sim/OceanCpu.cpp.
# ==================================================================================================
U32 = np.uint32


def pcg2d(vx, vy):
    """shaders/OceanCompute.hlsl:56.  uint32 wraparound throughout -- that IS the hash."""
    vx = np.asarray(vx, dtype=U32)
    vy = np.asarray(vy, dtype=U32)
    m = U32(1664525)
    vx = vx * m + U32(1013904223)
    vy = vy * m + U32(1013904223)
    vx = vx + vy * m
    vy = vy + vx * m
    vx = vx ^ (vx >> U32(16))
    vy = vy ^ (vy >> U32(16))
    vx = vx + vy * m
    vy = vy + vx * m
    vx = vx ^ (vx >> U32(16))
    vy = vy ^ (vy >> U32(16))
    return vx, vy


def gaussian_pair(ix, iy, seed, cascade):
    """shaders/OceanCompute.hlsl:65.  Box-Muller on 24 bits of each hash lane."""
    ix = np.asarray(ix, dtype=U32)
    iy = np.asarray(iy, dtype=U32)
    sx = U32((seed + cascade * 197) & 0xFFFFFFFF)
    sy = U32((seed * 13) & 0xFFFFFFFF)
    hx, hy = pcg2d(ix * U32(3) + sx, iy * U32(5) + sy)
    u1 = np.maximum((hx & U32(0xFFFFFF)).astype(np.float64) / 16777216.0, 1e-6)
    u2 = (hy & U32(0xFFFFFF)).astype(np.float64) / 16777216.0
    r = np.sqrt(-2.0 * np.log(u1))
    return r * np.cos(2.0 * PI * u2), r * np.sin(2.0 * PI * u2)


def erf_as(x):
    """Abramowitz & Stegun 7.1.26, the shader's Erf, |error| <= 1.5e-7 absolute.  Kept rather than
    math.erf on purpose: being MORE accurate here would draw a different bin amplitude than the
    GPU drew, i.e. a different ocean."""
    x = np.asarray(x, dtype=np.float64)
    s = np.sign(x)
    x = np.abs(x)
    t = 1.0 / (1.0 + 0.3275911 * x)
    y = 1.0 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t
                - 0.284496736) * t + 0.254829592) * t * np.exp(-x * x)
    return s * y


def shape_jonswap(f, fp, gamma):
    """shaders/OceanCompute.hlsl:74 == SeaState.cpp:27.  Unit shape; the level is specScale."""
    f = np.asarray(f, dtype=np.float64)
    sigma = np.where(f <= fp, 0.07, 0.09)
    d = (f - fp) / (sigma * fp)
    r = fp / np.maximum(f, 1e-30)
    out = np.power(np.maximum(f, 1e-30), -5.0) * np.exp(-1.25 * r ** 4) \
        * np.power(gamma, np.exp(-0.5 * d * d))
    return np.where(f <= 1e-4, 0.0, out)


def wave_k(patch_l):
    """shaders/OceanCompute.hlsl:140.  kx, kz over the full 256^2 grid, indexed [iy, ix]."""
    idx = np.arange(N)
    s = np.where(idx < N // 2, idx, idx - N).astype(np.float64)
    sx, sy = np.meshgrid(s, s, indexing="xy")     # sx varies along axis 1 (= id.x)
    return 2.0 * PI / patch_l * sx, 2.0 * PI / patch_l * sy


def bin_variance(kx, kz, parts, patch_l, k_lo, k_hi, *, radial_point=False, angular_point=False):
    """shaders/OceanCompute.hlsl:100.  Directional variance of one k bin.

    radial_point / angular_point undo one half each of the erf bin integration.  They exist only
    to build the negative controls: point-sampling is precisely the bug the shader's comment says
    cost the first storm render a third of its Hs."""
    kx = np.asarray(kx, dtype=np.float64)
    kz = np.asarray(kz, dtype=np.float64)
    # sqrt(kx^2 + kz^2), NOT np.hypot: HLSL's length() is sqrt(dot(v,v)) and OceanCpu.cpp mirrors
    # it. hypot is the numerically careful algorithm and differs in the last ulp, which is enough
    # to reorder two near-equal bins at the truncation cutoff and make gate 5 disagree.
    klen = np.sqrt(kx * kx + kz * kz)
    live = (klen >= 1e-5) & (klen >= k_lo) & (klen < k_hi)
    safe = np.where(live, klen, 1.0)

    w = np.sqrt(G * safe)
    f = w / (2.0 * PI)
    dfdk = G / (4.0 * PI * w)
    dk = 2.0 * PI / patch_l

    acc = np.zeros_like(safe)
    for p in parts:
        if p["gamma"] > 0.0:
            ef = p["specScale"] * shape_jonswap(f, p["fp"], p["gamma"]) * dfdk * dk
        elif radial_point:
            d = (f - p["fp"]) / p["sigF"]
            ef = p["specScale"] * np.exp(-0.5 * d * d) * dfdk * dk
        else:
            hw = 0.5 * dfdk * dk
            d0 = (f - hw - p["fp"]) / p["sigF"]
            d1 = (f + hw - p["fp"]) / p["sigF"]
            ef = p["specScale"] * p["sigF"] * 1.2533141 \
                * (erf_as(d1 * 0.70710678) - erf_as(d0 * 0.70710678))

        cos_d = (kx * p["dirToX"] + kz * p["dirToZ"]) / safe
        cross_d = (kx * p["dirToZ"] - kz * p["dirToX"]) / safe
        dth = np.arctan2(cross_d, cos_d)
        sig_th = math.sqrt(2.0 / p["spreadS"])
        hth = 0.5 * dk / safe
        if angular_point:
            frac = np.exp(-0.5 * (dth / sig_th) ** 2) / (sig_th * math.sqrt(2.0 * PI)) * (2.0 * hth)
        else:
            frac = 0.5 * (erf_as((dth + hth) / (sig_th * 1.41421356))
                          - erf_as((dth - hth) / (sig_th * 1.41421356)))
        acc = acc + ef * frac
    return np.where(live, acc, 0.0)


# --------------------------------------------------------------------------------------------------
#  SeaState::MakePartition (src/sim/SeaState.cpp:143), so this proof can build a realistic sea from
#  (Hs, Tp, direction) the way the engine does.
# --------------------------------------------------------------------------------------------------
def make_partition(hs, tp, from_deg, windsea):
    p = {
        "fp": 1.0 / tp,
        "gamma": 3.3 if windsea else 0.0,
        "sigF": min(max(0.10 / (tp * tp), 0.004), 0.02),
        "spreadS": 8.0 if windsea else 60.0,
    }
    toward = (from_deg + 180.0) * PI / 180.0
    p["dirToX"] = math.sin(toward)
    p["dirToZ"] = math.cos(toward)
    # The same 0.005..1.2 Hz / 0.0005 Hz rectangle rule the engine uses, so specScale matches.
    f = np.arange(0.005, 1.2, 0.0005)
    shape = shape_jonswap(f, p["fp"], p["gamma"]) if windsea else \
        np.exp(-0.5 * ((f - p["fp"]) / p["sigF"]) ** 2)
    p["specScale"] = ((hs / 4.0) ** 2) / (float(np.sum(shape)) * 0.0005)
    return p


def analytic_m0(parts, f0, f1, nf=400001):
    """Reference m0 over a frequency window: the SAME shapes, integrated finely in f, with the
    directional distribution integrating to 1 over the circle (which the erf-fraction construction
    guarantees).  Independent of the k grid entirely -- that is the point."""
    f = np.linspace(f0, f1, nf)
    tot = 0.0
    for p in parts:
        s = p["specScale"] * (shape_jonswap(f, p["fp"], p["gamma"]) if p["gamma"] > 0
                              else np.exp(-0.5 * ((f - p["fp"]) / p["sigF"]) ** 2))
        tot += float(np.trapezoid(s, f))
    return tot


def band_freqs(c):
    """The frequency window a cascade's k band maps to under deep-water dispersion."""
    return (math.sqrt(G * BAND_LO[c]) / (2.0 * PI), math.sqrt(G * BAND_HI[c]) / (2.0 * PI))


def grid_variance(parts, c, patch_l=None, **kw):
    """sum of BinVariance over the whole grid.  patch_l overrides the cascade's own L, which is how
    the tolerance below is SHOWN to be discretisation rather than asserted to be."""
    pl = PATCH_L[c] if patch_l is None else patch_l
    kx, kz = wave_k(pl)
    return float(np.sum(bin_variance(kx, kz, parts, pl, BAND_LO[c], BAND_HI[c], **kw)))


# ==================================================================================================
#  The realization: h0, the rotor, the packed spectra, the direct sum, and the reference FFT.
# ==================================================================================================
def build_h0(parts, seed, c, **kw):
    """CsInitSpectrum (shaders/OceanCompute.hlsl:147).  A = h0(k), B = h0(-k) as stored at each
    texel -- B uses the MIRROR texel's Gaussian and the CONTINUOUS -k for its variance, which is
    exactly what the shader's am/gm pair does."""
    kx, kz = wave_k(PATCH_L[c])
    ix = np.arange(N)
    IX, IY = np.meshgrid(ix, ix, indexing="xy")
    MX, MY = np.meshgrid((N - ix) % N, (N - ix) % N, indexing="xy")
    gpx, gpy = gaussian_pair(IX, IY, seed, c)
    gmx, gmy = gaussian_pair(MX, MY, seed, c)
    ap = 0.5 * np.sqrt(bin_variance(kx, kz, parts, PATCH_L[c], BAND_LO[c], BAND_HI[c], **kw))
    am = 0.5 * np.sqrt(bin_variance(-kx, -kz, parts, PATCH_L[c], BAND_LO[c], BAND_HI[c], **kw))
    return (ap * gpx) + 1j * (ap * gpy), (am * gmx) + 1j * (am * gmy), kx, kz


def reference_fields(A, B, kx, kz, t, *, chop_sign=-1.0, fft_sign=+1.0):
    """CsModulate + CsFft + CsAssemble, straight through, on the full 256^2 grid.

    The transform is the shader's: radix-2 with tw = e^{+i}, NO normalisation, which is exactly
    N^2 * numpy.fft.ifft2.  The packing is the shader's: C = f_hat + i g_hat lands f in .re and g
    in .im, which is why (h, Dx) come out of one complex plane.  chop_sign/fft_sign are the
    negative-control knobs; -1 / +1 are the shader's own values."""
    klen = np.sqrt(kx * kx + kz * kz)
    live = klen > 1e-5
    safe = np.where(live, klen, 1.0)
    w = np.sqrt(G * safe)
    rot = np.exp(1j * w * t)
    hk = A * rot + np.conj(B) * np.conj(rot)

    dxh = np.where(live, chop_sign * 1j * (kx / safe) * hk, 0.0)   # shader: -i (kx/k) h
    dzh = np.where(live, chop_sign * 1j * (kz / safe) * hk, 0.0)
    jxx = np.where(live, (kx * kx / safe) * hk, 0.0)
    jzz = np.where(live, (kz * kz / safe) * hk, 0.0)
    jxz = np.where(live, (kx * kz / safe) * hk, 0.0)

    def synth(z):
        return np.fft.ifft2(z) * (N * N) if fft_sign > 0 else np.fft.fft2(z)

    a = synth(hk + 1j * dxh)          # .re = h, .im = Dx
    b = synth(dzh + 1j * jxx)         # .re = Dz, .im = Jxx
    d = synth(jzz + 1j * jxz)         # .re = Jzz, .im = Jxz
    return {"h": np.real(a), "dx": np.imag(a), "dz": np.real(b), "jxx": np.imag(b),
            "jzz": np.real(d), "jxz": np.imag(d),
            "imag_leak": float(np.max(np.abs(np.imag(synth(hk)))))}


def build_pairs(A, B, kx, kz):
    """The retained-bin list OceanCpu::SetSeaState builds: one entry per (+k,-k) CONJUGATE PAIR,
    ranked by its time-averaged variance contribution |A|^2 + |B|^2, largest first.

    Why pairs: hk(-k) = conj(hk(k)) for every bin with nonzero variance -- the kHi guard band puts
    the whole Nyquist row/column, where that identity would break, outside every band -- so a pair
    contributes 2*Re[hk e^{ik.x}] and the reconstructed field stays exactly real."""
    idx = np.arange(N)
    s = np.where(idx < N // 2, idx, idx - N)
    SX, SY = np.meshgrid(s, s, indexing="xy")
    keep = ((SY > 0) | ((SY == 0) & (SX > 0))) & ((np.abs(A) ** 2 + np.abs(B) ** 2) > 0.0)
    wgt = (np.abs(A) ** 2 + np.abs(B) ** 2)[keep]
    order = np.argsort(-wgt)
    return kx[keep][order], kz[keep][order], A[keep][order], B[keep][order], wgt[order]


def truncate(wgt):
    """OceanCpu::SetSeaState's adaptive N: take pairs until the tail is under kResidualTarget of
    the cascade's variance, capped at kMaxBins.  Returns (n, residual)."""
    tot = float(np.sum(wgt))
    if tot <= 0.0:
        return 0, 0.0
    cum = np.cumsum(wgt)
    n = int(np.searchsorted(cum, (1.0 - RESIDUAL_TARGET) * tot) + 1)
    n = min(n, MAX_BINS, len(wgt))
    return n, 1.0 - float(cum[n - 1]) / tot


def direct_sum(pk_x, pk_z, pA, pB, x, z, t, *, chop_sign=-1.0):
    """OceanCpu::Displacement, one point.  Returns (Dx, h, Dz) BEFORE the lambda scale."""
    klen = np.sqrt(pk_x * pk_x + pk_z * pk_z)
    w = np.sqrt(G * klen)
    hk = pA * np.exp(1j * w * t) + np.conj(pB) * np.exp(-1j * w * t)
    p = hk * np.exp(1j * (pk_x * x + pk_z * z))
    h = 2.0 * float(np.sum(np.real(p)))
    # Dx = 2 Re[(-i kx/k) hk e^{ikx}] = 2 (kx/k) Im[p]   (chop_sign = -1 is the shader's)
    dx = 2.0 * float(np.sum((-chop_sign) * (pk_x / klen) * np.imag(p)))
    dz = 2.0 * float(np.sum((-chop_sign) * (pk_z / klen) * np.imag(p)))
    return dx, h, dz


# ==================================================================================================
#  The sea states the gates run on.
# ==================================================================================================
WIND = make_partition(2.0, 8.0, 60.0, True)            # Hs 2 m, Tp 8 s wind sea from the ENE
SWELL = make_partition(1.5, 14.0, 110.0, False)        # the narrow-swell trap, Hs 1.5 m, Tp 14 s
BOTH = [WIND, SWELL]
SEED = 0x9E3779B1

print("=" * 98)
print("proofs/ocean_cpu.py -- the cascade spectrum on the CPU (gate on src/sim/OceanCpu.*)")
print("=" * 98)
for nm, p in (("windsea", WIND), ("swell", SWELL)):
    print("  %-8s fp %.4f Hz  gamma %.1f  sigF %.4f  spreadS %2.0f  specScale %.6g  "
          "dirTo (%+.3f,%+.3f)" % (nm, p["fp"], p["gamma"], p["sigF"], p["spreadS"],
                                   p["specScale"], p["dirToX"], p["dirToZ"]))
for c in range(3):
    f0, f1 = band_freqs(c)
    print("  cascade %d: L %6.1f m  dk %.5f  k [%.5f, %.5f) rad/m -> f [%.4f, %.4f) Hz  "
          "lambda [%.1f, %.0f] m"
          % (c, PATCH_L[c], 2 * PI / PATCH_L[c], BAND_LO[c], BAND_HI[c], f0, f1,
             2 * PI / BAND_HI[c], 2 * PI / BAND_LO[c]))
print()


# ==================================================================================================
#  GATE 1 -- SPECTRUM.  sum of BinVariance over the whole 256^2 grid == analytic m0 over the band.
# ==================================================================================================
print("-" * 98)
print("GATE 1  spectrum: sum_bins BinVariance(k)  vs  analytic m0 restricted to the cascade band")
print("-" * 98)

TOL1 = 0.08     # justified, not fitted -- see the refinement check at the end of this gate

for label, parts in (("windsea", [WIND]), ("swell", [SWELL]), ("both", BOTH)):
    for c in range(3):
        f0, f1 = band_freqs(c)
        num, ana = grid_variance(parts, c), analytic_m0(parts, f0, f1)
        if ana < 1e-9 and num < 1e-9:
            print("      %-8s cascade %d: both below 1e-9 m^2 (this band carries no energy)"
                  % (label, c))
            continue
        rel = (num - ana) / ana
        check("gate1 %s cascade %d" % (label, c), abs(rel) < TOL1,
              "grid %.6e m^2 vs analytic %.6e m^2, rel %+.3f%%" % (num, ana, 100 * rel))

# WHY THE TOLERANCE IS 8%: the error is grid discretisation -- square bins tiling an annulus, a
# point-sampled x dk JONSWAP radial, and rings only 3-4 bins in radius at each band's inner cut.
# Every one of those shrinks when the grid gets finer in k, so REFINE L by 4x over the SAME band
# and watch it go.  If the error were a spectrum bug instead, refinement would not touch it.
print("      refinement (same band, patch L x4 => dk / 4, so the same physics on a finer grid):")
refine_ok = True
for c in (1, 2):
    f0, f1 = band_freqs(c)
    ana = analytic_m0(BOTH, f0, f1)
    e1 = abs(grid_variance(BOTH, c) - ana) / ana
    e4 = abs(grid_variance(BOTH, c, patch_l=4.0 * PATCH_L[c]) - ana) / ana
    print("        cascade %d: |rel| %.3f%% at L, %.3f%% at 4L" % (c, 100 * e1, 100 * e4))
    refine_ok = refine_ok and e4 < 0.5 * e1
check("gate1 the residual is discretisation (refining the k grid shrinks it)", refine_ok,
      "4x finer dk more than halves the error on both cascades that show one")

# NEGATIVE CONTROLS.  Undo one half of the bin integration at a time and sweep Tp: the trap is not
# that point-sampling is biased, it is that the answer depends on WHERE the grid falls relative to
# a peak narrower than a bin -- so m0 swings with Tp.  The erf form must not.
print("      negative controls: sweep a narrow swell's Tp and watch m0 move")
tps = [9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0]
sweep = {}
for mode, kw in (("erf (the shader)", {}), ("radial point-sampled", {"radial_point": True}),
                 ("angular point-sampled", {"angular_point": True})):
    errs = []
    for tp in tps:
        sw = [make_partition(1.5, tp, 110.0, False)]
        f0, f1 = band_freqs(0)
        ana = analytic_m0(sw, f0, f1)
        errs.append((grid_variance(sw, 0, **kw) - ana) / ana)
    sweep[mode] = errs
    print("        %-22s rel err over Tp 9..18 s: min %+6.1f%%  max %+6.1f%%  spread %5.1f%%"
          % (mode, 100 * min(errs), 100 * max(errs), 100 * (max(errs) - min(errs))))
check("gate1 erf form is stable across Tp", max(abs(e) for e in sweep["erf (the shader)"]) < TOL1,
      "worst |rel| %.2f%% over the sweep" % (100 * max(abs(e) for e in sweep["erf (the shader)"])))
for mode in ("radial point-sampled", "angular point-sampled"):
    check_fails("gate1 %s survives the Tp sweep" % mode,
                max(abs(e) for e in sweep[mode]) < TOL1,
                "worst |rel| %.1f%% (erf form: %.2f%%)"
                % (100 * max(abs(e) for e in sweep[mode]),
                   100 * max(abs(e) for e in sweep["erf (the shader)"])))
print()


# ==================================================================================================
#  GATE 2 -- ENERGY.  4 sqrt(total variance) vs the partition target, in the ensemble and in ONE
#  draw.  The CPU twin of OceanFft::MeasureHs, which reads the rendered texture back and computes
#  the same number from the height channel.
# ==================================================================================================
print("-" * 98)
print("GATE 2  energy: Hs = 4 sqrt(sum over all three cascades) vs the partition target")
print("-" * 98)

for label, parts, target in (("windsea Hs 2.0 Tp 8", [WIND], 2.0),
                             ("swell   Hs 1.5 Tp 14", [SWELL], 1.5),
                             ("both", BOTH, math.hypot(2.0, 1.5))):
    per = [grid_variance(parts, c) for c in range(3)]
    hs = 4.0 * math.sqrt(sum(per))
    f_lo, f_hi = band_freqs(0)[0], band_freqs(2)[1]
    frac_out = 1.0 - analytic_m0(parts, f_lo, f_hi) / analytic_m0(parts, 0.005, 1.2)
    print("      %-22s per-cascade m0 = %.5f / %.5f / %.5f m^2" % (label, per[0], per[1], per[2]))
    check("gate2 %s" % label, abs(hs / target - 1.0) < 0.04,
          "Hs %.3f m vs target %.3f m (%.3f%% of m0 falls outside the union of the three bands)"
          % (hs, target, 100 * frac_out))

# 2b -- THE REALIZED sea, not the expected one.  One draw of a NARROW swell is a small number of
# modes, so its m0 is a chi-square with few degrees of freedom and scatters hard.  Averaging over
# seeds must converge to the analytic value; if it did not, the Gaussian draw would be biased.
print("      one draw vs the ensemble (a narrow swell is only a handful of modes):")
SEEDS = [0x9E3779B1 + 0x9E3779B9 * i for i in range(64)]
for c in (0, 1):
    exp_v = grid_variance(BOTH, c)
    real = []
    for sd in SEEDS:
        A, B, kx, kz = build_h0(BOTH, sd, c)
        _, _, _, _, w = build_pairs(A, B, kx, kz)
        real.append(2.0 * float(np.sum(w)))
    real = np.array(real)
    sem = float(np.std(real, ddof=1)) / math.sqrt(len(real))
    print("        cascade %d: expected %.5f m^2; %d draws mean %.5f (sem %.5f), "
          "single-draw sd %.1f%%, seed %08x gave %.5f (%+.1f%%)"
          % (c, exp_v, len(SEEDS), real.mean(), sem, 100 * real.std(ddof=1) / real.mean(),
             SEED, real[0], 100 * (real[0] / exp_v - 1)))
    check("gate2b cascade %d ensemble mean == sum of BinVariance" % c,
          abs(real.mean() - exp_v) < 3.0 * sem + 1e-12,
          "|mean - expected| = %.3e vs 3*sem = %.3e" % (abs(real.mean() - exp_v), 3.0 * sem))
print()


# ==================================================================================================
#  GATE 3 -- TRUNCATION.  The measured residual variance fraction, per cascade, per cap.
# ==================================================================================================
print("-" * 98)
print("GATE 3  truncation: variance fraction DROPPED by keeping the largest-N conjugate pairs")
print("-" * 98)

CAPS = (128, 256, 512, 1024, 2048, 4096)
STORM = [make_partition(6.0, 11.0, 45.0, False), make_partition(2.5, 4.5, 45.0, True)]
for label, parts in (("both (Hs 2.5)", BOTH), ("storm (Hs 6.5)", STORM)):
    for c in range(3):
        A, B, kx, kz = build_h0(parts, SEED, c)
        _, _, _, _, wgt = build_pairs(A, B, kx, kz)
        tot = float(np.sum(wgt))
        cum = np.cumsum(wgt)
        n, resid = truncate(wgt)
        print("      %-14s cascade %d: %6d pairs carry energy, realized m0 %.6f m^2; "
              "adaptive N = %4d, residual %.3f%%" % (label, c, len(wgt), 2 * tot, n, 100 * resid))
        print("                     residual at N = " + "  ".join(
            "%d:%.3f%%" % (cap, 100 * (1.0 - float(cum[min(cap, len(cum)) - 1]) / tot))
            for cap in CAPS))
        check("gate3 %s cascade %d hits the 1%% target under the cap" % (label, c),
              resid <= RESIDUAL_TARGET + 1e-12 or n >= MAX_BINS,
              "N = %d (cap %d), residual %.4f%%" % (n, MAX_BINS, 100 * resid))
print()


# ==================================================================================================
#  GATE 4a -- REALIZATION, the hash.  pcg2d / GaussianPair against a golden table that BOTH the
#  numpy and the C++ implementations reproduce.  uint32 must wrap identically in both.
# ==================================================================================================
print("-" * 98)
print("GATE 4a realization: pcg2d / GaussianPair vs the golden table")
print("-" * 98)

HASH_TEXELS = [(0, 0), (1, 0), (0, 1), (1, 1), (7, 13), (255, 255), (128, 128), (255, 0),
               (0, 255), (37, 199), (200, 41), (64, 192)]
HASH_CASES = [(0x9E3779B1, 0), (0x9E3779B1, 1), (0x9E3779B1, 2), (0xFFFFFFFF, 2), (1, 0)]
# Produced by BOTH shaders/OceanCompute.hlsl's arithmetic in numpy and src/sim/OceanCpu.cpp's in
# MSVC. seed 0xFFFFFFFF and cascade 2 are in here on purpose: seed*13 and seed+cascade*197 both
# wrap there, which is where a 64-bit intermediate would show up.
GOLDEN = """
9e3779b1 0   0   0 cfaf4777 39bfec5c -1.638979645e-03 -8.703978412e-01
9e3779b1 0   1   0 98712f4c 979e9ece -9.334813158e-01 -8.723099121e-01
9e3779b1 0   0   1 2e8d42d5 1814a389 +9.535355364e-01 +5.290609577e-01
9e3779b1 0   1   1 77d60c82 a9080319 +5.867555357e-01 +1.168940951e-01
9e3779b1 0   7  13 95353817 aedc2e14 +1.130461564e+00 -1.365128857e+00
9e3779b1 0 255 255 55daccbe b2e7dc97 +4.648904735e-01 -3.129179761e-01
9e3779b1 0 128 128 6102a06b 94510384 -1.227344920e+00 +2.766328609e+00
9e3779b1 0 255   0 056c4423 8ce093a8 +9.407162510e-01 -9.144520556e-01
9e3779b1 0   0 255 123f7bee 66497a20 -3.849432280e-01 +1.624990655e+00
9e3779b1 0  37 199 de629a26 3a187de6 +1.139216131e+00 +7.812525319e-01
9e3779b1 0 200  41 6d88401e 83dfc0f6 +7.893411203e-01 -7.989404847e-01
9e3779b1 0  64 192 bfc7a563 2761ba72 -5.193259989e-01 +4.770360363e-01
9e3779b1 1   0   0 b44a5089 94951af0 -1.366477317e+00 -7.787682715e-01
9e3779b1 1   1   0 ed6d2c51 2cad661d -5.755496749e-01 -1.171848057e+00
9e3779b1 1   0   1 59d50285 288b0a2b -5.842418492e-01 -1.622948211e-01
9e3779b1 1   1   1 f5f5b5d4 69674ba8 -2.353804465e-01 +1.632323829e-01
9e3779b1 1   7  13 b101271a f5bb0a87 -3.991025016e-01 -3.262941590e+00
9e3779b1 1 255 255 6fb7174c b8539c1e -3.790341854e-01 +7.257557054e-01
9e3779b1 1 128 128 72e84f6e e99f77bf -3.156584557e-01 -3.075173353e-01
9e3779b1 1 255   0 0446e80f 2ee37341 +1.224829390e+00 -1.033162250e+00
9e3779b1 1   0 255 e2905a60 f856b6c4 -5.663163737e-01 +9.083594539e-01
9e3779b1 1  37 199 9e6995da 061bf490 +1.029730841e+00 +8.431902979e-01
9e3779b1 1 200  41 9a56f315 a863e4c5 -1.133571736e+00 +9.352615279e-01
9e3779b1 1  64 192 e43300a7 f02ce883 +8.112366556e-01 +1.602653670e+00
9e3779b1 2   0   0 dc22efe0 f944c79e -2.335975594e-01 +1.982093217e+00
9e3779b1 2   1   0 3ca1bd12 7afa947f +9.498613302e-01 -1.271061921e-01
9e3779b1 2   0   1 e82a724d 619f8d62 -1.355146608e+00 -1.325686325e+00
9e3779b1 2   1   1 4d611721 57b23b62 -4.616488821e-01 -1.313754649e+00
9e3779b1 2   7  13 93ab00fd cc6400db -6.944591247e-01 +5.698325621e-01
9e3779b1 2 255 255 144e76c2 e2957c02 -1.328983646e+00 -7.738734547e-01
9e3779b1 2 128 128 719b1325 ba6484f6 -7.820256002e-01 +6.252806589e-01
9e3779b1 2 255   0 515100dc fec8fb09 +3.316781025e-01 -1.480325414e+00
9e3779b1 2   0 255 bf2cd851 1deff4a5 +1.723669477e+00 -7.161667763e-01
9e3779b1 2  37 199 dd3d7f1d 79dcee5d +1.100981557e+00 -1.280715736e+00
9e3779b1 2 200  41 ccb1c57c decd5d59 +2.751384497e-01 -8.084902902e-01
9e3779b1 2  64 192 6a0a8324 b956f9fc -1.350658797e+00 +2.135647391e+00
ffffffff 2   0   0 8e23fd99 3466d983 -1.615307823e+00 +1.146568384e+00
ffffffff 2   1   0 e9f78c48 38bdbd27 -1.437313993e-02 -2.587267084e-01
ffffffff 2   0   1 4d0aab4c ba625961 -1.882406887e+00 +1.676958690e+00
ffffffff 2   1   1 da3483ae f24e9d31 -6.248307394e-01 +1.666663613e+00
ffffffff 2   7  13 0e2e085e aec8ed56 +4.026439470e-01 -1.808184005e+00
ffffffff 2 255 255 238c6552 10112953 +1.000292960e+00 +4.481495067e-01
ffffffff 2 128 128 4ee4bbbf 0b3e8bc9 +1.693265061e-02 +4.742932162e-01
ffffffff 2 255   0 d99d9c40 7c285036 +5.408884032e-01 +8.231299464e-01
ffffffff 2   0 255 25519c8e aa966e29 -1.288679752e+00 -7.910233846e-01
ffffffff 2  37 199 5bca8a74 40cbd733 +1.961217547e-01 -6.557500512e-01
ffffffff 2 200  41 409087df dabb950e -1.157134122e-01 -1.063005536e+00
ffffffff 2  64 192 aabc02a2 ec3e985b +2.708676444e-02 +7.852565598e-01
00000001 0   0   0 c55fad98 160c60e8 +1.338734337e+00 +4.197217986e-01
00000001 0   1   0 6fbfc901 68670b68 -6.218443636e-01 +4.369356726e-01
00000001 0   0   1 61818a1d f7a6d655 -6.758969522e-01 -9.516020249e-01
00000001 0   1   1 53d16c82 53917333 -5.765053385e-01 -2.632033881e-01
00000001 0   7  13 217e71d3 2917ac26 +9.928494878e-01 +6.519170495e-01
00000001 0 255 255 2fc95fdd 558ea8ad -6.484764216e-01 -2.439279416e-01
00000001 0 128 128 ae98ceab 14b5aec9 -2.545025681e-01 -9.834775409e-01
00000001 0 255   0 f2eb474a 3181124c -4.107293020e-01 -1.080380202e-02
00000001 0   0 255 b12454b6 6de64abe +1.595622368e+00 -1.165764468e+00
00000001 0  37 199 8493f233 a07196b8 -9.823877535e-01 +3.627303131e-01
00000001 0 200  41 6045758f 39dd6773 +1.067083822e+00 -1.212521160e+00
00000001 0  64 192 e2107b56 fc2f4f25 +9.328765253e-01 +2.148391282e+00
""".strip().splitlines()


def hash_table(seed_bump=0):
    lines = []
    for seed, cas in HASH_CASES:
        seed = (seed + seed_bump) & 0xFFFFFFFF
        for (ix, iy) in HASH_TEXELS:
            sx = U32((seed + cas * 197) & 0xFFFFFFFF)
            sy = U32((seed * 13) & 0xFFFFFFFF)
            hx, hy = pcg2d(np.uint32(ix) * U32(3) + sx, np.uint32(iy) * U32(5) + sy)
            gx, gy = gaussian_pair(np.uint32(ix), np.uint32(iy), seed, cas)
            lines.append("%08x %d %3d %3d %08x %08x %+.9e %+.9e"
                         % (seed, cas, ix, iy, int(hx), int(hy), float(gx), float(gy)))
    return lines


mine = hash_table()
check("gate4a numpy vs golden", [ln.strip() for ln in GOLDEN] == mine,
      "%d rows, exact string match on the uint32 hex AND the 9-digit Gaussians" % len(mine))
# Not vacuous: one bit of seed must move every single row of the table.
bumped = hash_table(seed_bump=1)
moved = sum(1 for a, b in zip(mine, bumped) if a.split()[4:] != b.split()[4:])
check_fails("gate4a table is insensitive to the seed", moved < len(mine),
            "seed+1 changed %d of %d rows" % (moved, len(mine)))
print()


# ==================================================================================================
#  GATE 4b -- REALIZATION, the field.  THE SIGN AND SCALE GATE.  The truncated direct sum against
#  an actual 2-D inverse FFT of the same packed spectrum -- the GPU's own algorithm, in numpy.
# ==================================================================================================
print("-" * 98)
print("GATE 4b realization: truncated direct sum vs the unnormalised e^{+i} 2-D inverse FFT")
print("-" * 98)

T_TEST = 137.25
cache = {}
for c in range(3):
    A, B, kx, kz = build_h0(BOTH, SEED, c)
    ref = reference_fields(A, B, kx, kz, T_TEST)
    pkx, pkz, pA, pB, wgt = build_pairs(A, B, kx, kz)
    n, resid = truncate(wgt)
    cache[c] = (A, B, kx, kz, ref, pkx, pkz, pA, pB, n, resid)

    rms_h = float(np.sqrt(np.mean(ref["h"] ** 2)))
    # Sample where the FFT's own grid point lives: texel (nx, ny) is the field at (nx*L/N, ny*L/N).
    picks = np.random.default_rng(7).integers(0, N, size=(48, 2))
    err = 0.0
    for (nx_, ny_) in picks:
        x, z = nx_ * PATCH_L[c] / N, ny_ * PATCH_L[c] / N
        dxs, hs_, dzs = direct_sum(pkx[:n], pkz[:n], pA[:n], pB[:n], x, z, T_TEST)
        err = max(err, abs(hs_ - ref["h"][ny_, nx_]), abs(dxs - ref["dx"][ny_, nx_]),
                  abs(dzs - ref["dz"][ny_, nx_]))
    # Dropping a variance fraction r leaves an rms error of sqrt(r)*rms; allow 3x for the max of
    # 48 samples and a 1e-9 floor for the cascades that drop nothing at all.
    budget = 3.0 * math.sqrt(max(resid, 0.0)) * rms_h + 1e-9
    check("gate4b cascade %d: direct sum == IFFT" % c, err <= budget,
          "max err %.3e m over 48 texels (field rms %.3e m); N = %d dropped %.3f%% of the "
          "variance, budget %.3e m" % (err, rms_h, n, 100 * resid, budget))
    check("gate4b cascade %d: the field is exactly real" % c,
          ref["imag_leak"] < 1e-9 * max(rms_h, 1e-12),
          "max |Im| of the IFFT of h_hat = %.3e m -- Hermitian symmetry is exact because the "
          "0.9*pi*N/L guard band excludes the whole Nyquist row/column" % ref["imag_leak"])

# NEGATIVE CONTROLS.  Perturb the REFERENCE only (never both sides -- that is the vacuous gate).
A, B, kx, kz, ref, pkx, pkz, pA, pB, n, resid = cache[0]
rms_h = float(np.sqrt(np.mean(ref["h"] ** 2)))
rms_d = float(np.sqrt(np.mean(ref["dx"] ** 2)))
PROBES = ((11, 3), (100, 200), (255, 1), (60, 60), (7, 240))


def worst_vs(reference):
    """max |h| and |Dx| error of the UNCHANGED direct sum against a (possibly wrong) reference."""
    eh, ed = 0.0, 0.0
    for (nx_, ny_) in PROBES:
        x, z = nx_ * PATCH_L[0] / N, ny_ * PATCH_L[0] / N
        dxs, hs_, _ = direct_sum(pkx[:n], pkz[:n], pA[:n], pB[:n], x, z, T_TEST)
        eh = max(eh, abs(hs_ - reference["h"][ny_, nx_]))
        ed = max(ed, abs(dxs - reference["dx"][ny_, nx_]))
    return eh, ed


# The bar every control has to clear: the truncation floor. Cascade 0 keeps 164 of its 248 pairs,
# so the direct sum is ALREADY sqrt(0.0099)*rms off the full-grid reference, and a control only
# proves something if it lands well outside that. Require 3x.
BUDGET = 3.0 * math.sqrt(resid) * rms_h + 1e-9
eh0, ed0 = worst_vs(ref)
check("gate4b baseline sits at the truncation floor", eh0 <= BUDGET and ed0 <= BUDGET,
      "max |dh| %.3e m, max |dDx| %.3e m; budget %.3e m from dropping %.3f%% of a %.3e m rms field"
      % (eh0, ed0, BUDGET, 100 * resid, rms_h))
# Each control is judged by THE SAME test the gate applies (error <= BUDGET), so the bar is not a
# second free parameter: if a control still passes that test, the gate cannot tell it from correct.
for name, kwargs, field in (
        ("the choppy sign (-i -> +i)", {"chop_sign": +1.0}, "dx"),
        ("the FFT synthesis sign (e^{+i} -> e^{-i})", {"fft_sign": -1.0}, "h")):
    bad = reference_fields(A, B, kx, kz, T_TEST, **kwargs)
    e = worst_vs(bad)[1 if field == "dx" else 0]
    check_fails("gate4b survives flipping %s" % name, e <= BUDGET,
                "max |d%s| becomes %.3e m -- %.1fx the truncation floor" % (field, e, e / BUDGET))
bad = reference_fields(A, B, kx, kz, T_TEST + 3.0)      # the reference moves, the sum does not
e = worst_vs(bad)[0]
check_fails("gate4b survives a 3 s offset in the reference's rotor", e <= BUDGET,
            "max |dh| becomes %.3e m -- %.1fx the truncation floor" % (e, e / BUDGET))
bad = dict(ref)
bad["h"] = ref["h"] * 0.5                               # a dropped factor of two on the pair sum
e = worst_vs(bad)[0]
check_fails("gate4b survives halving the reference height", e <= BUDGET,
            "max |dh| becomes %.3e m -- %.1fx the truncation floor" % (e, e / BUDGET))
print()


# ==================================================================================================
#  GATE 5 -- the C++ itself.  Optional: pass the harness dump's path.  Compares the retained
#  counts, the residuals, the delivered variances, and eight Displacement values against this
#  file's own truncated direct sum, using the partition floats the C++ actually held.
# ==================================================================================================
print("-" * 98)
print("GATE 5  src/sim/OceanCpu.cpp vs this file")
print("-" * 98)

if len(sys.argv) > 1:
    rows = [ln.strip() for ln in open(sys.argv[1]) if ln.startswith("#")]
    hashes = [ln.strip() for ln in open(sys.argv[1]) if not ln.startswith(("#", "["))
              and ln.strip()]
    same = sum(1 for a, b in zip(hashes, mine) if a.split()[:6] == b.split()[:6])
    check("gate5 C++ pcg2d bit-for-bit with numpy", len(hashes) == len(mine) == same,
          "%d/%d rows identical in the uint32 lanes" % (same, len(mine)))
    if len(hashes) == len(mine):
        dg = max(abs(float(a.split()[i]) - float(b.split()[i]))
                 for a, b in zip(hashes, mine) for i in (6, 7))
        check("gate5 C++ GaussianPair matches numpy", dg <= 1e-9, "max |delta| %.3e" % dg)

    state, cxx = None, {}
    for ln in rows:
        t = ln.split()
        if t[0] == "#STATE":
            state = t[1]
            cxx[state] = {"parts": [], "casc": {}, "disp": []}
        elif t[0] == "#PART":
            cxx[state]["parts"].append({"fp": float(t[3]), "specScale": float(t[5]),
                                        "sigF": float(t[7]), "gamma": float(t[9]),
                                        "dirToX": float(t[11]), "dirToZ": float(t[13]),
                                        "spreadS": float(t[15])})
        elif t[0] == "#CASCADE":
            cxx[state]["casc"][int(t[1])] = (int(t[3]), float(t[5]), float(t[7]))
        elif t[0] == "#DISP":
            cxx[state]["disp"].append([float(v) for v in t[2:]])

    # Three separate tolerances, each set by what limits it -- not one loose number covering all.
    #   count     EXACT.  The retained set is a discrete decision; it may not differ at all.
    #   variance  1e-7 relative.  This is the energy the evaluator DELIVERS, summed over retained
    #             bins whose amplitudes are well conditioned; only libm ulps separate the two.
    #   residual  1e-3 absolute.  Its denominator includes the swell's far-tail bins, where the
    #             erf DIFFERENCE is a catastrophic cancellation of two numbers within 1e-15 of 1
    #             (about 400 of a cascade-0 swell's 496 live bins).  Those bins are always dropped,
    #             so they never touch the field, but they make the residual FRACTION only good to
    #             ~5e-5 across libm implementations.  See the note in OceanCpu.cpp's Erf().
    #   field     1e-7 m.  A tenth of a micron on a 2.5 m sea.
    for state, d in cxx.items():
        worst_d, worst_v, worst_r = 0.0, 0.0, 0.0
        pairs = {}
        for c in range(3):
            A, B, kx, kz = build_h0(d["parts"], SEED, c)
            pkx, pkz, pA, pB, wgt = build_pairs(A, B, kx, kz)
            n, resid = truncate(wgt)
            pairs[c] = (pkx[:n], pkz[:n], pA[:n], pB[:n])
            cn, cr, cv = d["casc"][c]
            kept = 2.0 * float(np.sum(wgt[:n]))
            check("gate5 %s cascade %d retained count" % (state, c), cn == n,
                  "C++ %d, numpy %d pairs" % (cn, n))
            worst_v = max(worst_v, abs(cv - kept) / max(kept, 1e-12))
            worst_r = max(worst_r, abs(cr - resid))
        check("gate5 %s delivered variance" % state, worst_v < 1e-7,
              "worst relative mismatch %.3e" % worst_v)
        check("gate5 %s residual fraction" % state, worst_r < 1e-3,
              "worst absolute mismatch %.3e (cancellation-limited, see above)" % worst_r)
        for row in d["disp"]:
            x, z, t = row[0], row[1], row[2]
            dx = h = dz = 0.0
            for c in range(3):
                a, b_, cc, dd = pairs[c]
                if len(a) == 0:
                    continue
                sdx, sh, sdz = direct_sum(a, b_, cc, dd, x, z, t)
                dx += sdx
                h += sh
                dz += sdz
            worst_d = max(worst_d, abs(LAMBDA * dx - row[3]), abs(h - row[4]),
                          abs(LAMBDA * dz - row[5]))
        check("gate5 %s Displacement over %d points" % (state, len(d["disp"])), worst_d < 1e-7,
              "max |delta| %.3e m across (Dx, h, Dz)" % worst_d)
    for ln in rows:
        if ln.startswith("#TIME"):
            print("      " + ln[1:])
else:
    print("      SKIPPED: pass the C++ harness dump as argv[1] to run this gate.")
    print("      (build a standalone TU around src/sim/OceanCpu.cpp + SeaState.cpp that prints")
    print("       #HASH / #STATE / #PART / #CASCADE / #DISP lines; gates 1-4 do not need it.)")
print()


# ==================================================================================================
#  REPORT (not a gate): the SHAPE the shader's conventions actually produce.  Two questions a hull
#  cares about, answered by measurement rather than by reading the sign off the source.
# ==================================================================================================
print("-" * 98)
print("REPORT  measured conventions (single swell, one direction, so the answer is unambiguous)")
print("-" * 98)

ONE = [make_partition(2.0, 12.0, 270.0, False)]        # FROM the west => dirTo = due EAST
c_deep = G / (2.0 * PI * ONE[0]["fp"])                 # deep-water phase speed at the peak
print("      Tp 12 s swell, from 270 deg  =>  dirTo = (%+.3f, %+.3f) (east, north), c = %.2f m/s"
      % (ONE[0]["dirToX"], ONE[0]["dirToZ"], c_deep))
Ao, Bo, kxo, kzo = build_h0(ONE, SEED, 0)
f0 = reference_fields(Ao, Bo, kxo, kzo, 0.0)
f1 = reference_fields(Ao, Bo, kxo, kzo, 1.0)

# Which way did the pattern move in 1 s?  h0 == roll(h1, s) means the field advanced by -s texels.
corr = np.array([float(np.mean(f0["h"] * np.roll(f1["h"], s, axis=1))) for s in range(-30, 31)])
i = int(np.argmax(corr))
sub = 0.0 if i in (0, len(corr) - 1) else \
    0.5 * (corr[i - 1] - corr[i + 1]) / (corr[i - 1] - 2 * corr[i] + corr[i + 1])
shift = (i - 30) + sub
speed = -shift * PATCH_L[0] / N
print("      cross-correlation peak at %+.2f texels => the crest pattern moved %+.2f m/s in +x, "
      "i.e. %s" % (shift, speed, "EAST" if speed > 0 else "WEST"))
print("      => a bin at wavevector k travels toward -k^: with e^{+i(k.x + wt)} the phase fronts")
print("         run AGAINST k, so the sea runs OPPOSITE to the partition's dirTo.  (Tessendorf's")
print("         equations have this too; it is invisible under his |k^.w^|^2 spectrum, which is")
print("         symmetric in k.  The cos^2s lobe here is not, so it shows.)")

# Does the choppy displacement compress crests (real Gerstner) or stretch them?  The Jacobian
# CsAssemble writes is the whole-field answer: J < 1 means the surface folded IN.
J = (1.0 + LAMBDA * f0["jxx"]) * (1.0 + LAMBDA * f0["jzz"]) - (LAMBDA ** 2) * f0["jxz"] ** 2
hi = f0["h"] >= np.quantile(f0["h"], 0.90)
lo = f0["h"] <= np.quantile(f0["h"], 0.10)
print("      Jacobian J = det(I + lambda grad D): mean %.3f in the top decile of h (crests), "
      "%.3f in the bottom decile (troughs)" % (float(J[hi].mean()), float(J[lo].mean())))
print("      => lambda*D points AWAY from crests: the choppy term BROADENS crests and sharpens")
print("         troughs, and the foam channel (saturate((0.80 - J) * 4)) therefore fires in the")
print("         TROUGHS.  A physical Gerstner wave is the other way round (ALGEBRA.md `caustics`:")
print("         \"crests compress by 1 - s*a*k\").  OceanCpu mirrors the shader exactly, so the")
print("         hull rides what the render draws; fix them together or not at all.")
print()

print("=" * 98)
if FAILS:
    print("FAILED: " + ", ".join(FAILS))
    sys.exit(1)
print("all gates passed")
