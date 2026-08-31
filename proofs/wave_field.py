# ==================================================================================================
#  proofs/wave_field.py - GAGAME's solved wave field, twin-tested against the vqview-inlet bake.
#
#  ALGEBRA FIRST: this is the authoritative small-scale rendering of the wave-bvp derivation
#  (scratchpad math/wave-bvp.derivation.md); src/sim/WaveField.cpp must later mirror these
#  functions 1:1.  It is an INDEPENDENT transcription of the math (no vqview code imported;
#  proofs/vqview_ref.py only decodes their exported textures), run on vqview's own inputs and
#  diffed texel-for-texel against their solution.
#
#  Physics per component i (absolute frequency sigma_i conserved):
#    dispersion   (sigma + k*Uopp)^2 = g k tanh(k h),  Uopp = max(-U.dhat, 0)
#                 solved by 96-point log bracket + 48 bisections, first +to- sign change
#                 (the physical branch, cg_r > Uopp); BLOCKED where no sign change exists.
#    shoaling     Ks = sqrt(cg0 / max(cg + along, 0.15))          (energy-flux conservation)
#    refraction   Snell vs the fixed bar normal 285 deg, PER-COMPONENT direction dir_i:
#                 sin th = clip(sin th0_i * c/c0), Kr = sqrt(cos th0_i / cos th)
#    limiter      ONE uniform factor so rms = sqrt(sum a^2) <= 0.60*max(h,0.05)/(2 sqrt 2)
#    phase gauge  phi = cumsum_x(k d_e Dx) + cumsum_y(rowmean(k d_n) Dy), row 0 north,
#                 stored/compared as the unit spinor (cos phi, sin phi)  (house cl2 law)
#
#  Reference decode is truncating 8-bit (byte = floor(clip(x/max)*255)); the unbiased decode
#  used here is (byte + 0.5)/255 * max, so a faithful port lands at exactly <= 0.5 LSB.
#
#  Deterministic (fixed seed unused - no randomness), stdlib + numpy + PIL only.
#  Prints PASS/FAIL per assertion, exits nonzero on any failure, saves proofs/wave_field.png.
# ==================================================================================================
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vqview_ref  # noqa: E402  (decoder only - the math below is transcribed, not imported)

G = 9.81
CELL = 1.5
TIDE = 0.30
BAR_NORMAL_DEG = 285.0
GAMMA_HS = 0.60
N_COMP = 16
HS, TP, MWD, SPREAD = 2.60, 6.0, 75.0, 26.0
GOLDEN = 0.6180339887498949

FAILS = []


def check(name, ok, detail=""):
    print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, (" -- " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


# --------------------------------------------------------------------------------------------------
#  Portable core (the C++ port mirrors these three functions 1:1)
# --------------------------------------------------------------------------------------------------
def dispersion_k(T, h, Uopp, nsamp=96):
    """Solve (sigma + k*Uopp)^2 = g*k*tanh(k*h) for the physical (first) root.

    Bracket on a 96-point log grid [1e-4, 10^0.7] rad/m (first + -> - sign change selects the
    branch with intrinsic group speed > Uopp; Newton is structurally unsafe near blocking), then
    48 bisections (bracket width -> ~4e-16 * k).  Cells with no sign change are BLOCKED (no
    propagating solution: sigma > sigma_max(Uopp, h)); their k is held at 0.25*10^0.7 rad/m
    (~1.252968, lambda ~ 5 m: short/steep arrested chop, k*CELL = 0.598*pi < Nyquist).
    Sequential scan keeps memory at O(grid) instead of the reference's 96 x grid broadcast.
    """
    sig = 2.0 * np.pi / T
    h = np.maximum(h, 0.15)
    kk = np.logspace(-4.0, 0.7, nsamp)
    lo = np.full(h.shape, kk[0])
    hi = np.full(h.shape, kk[1])
    found = np.zeros(h.shape, dtype=bool)
    f_prev = (sig + kk[0] * Uopp) ** 2 - G * kk[0] * np.tanh(kk[0] * h)
    for i in range(1, nsamp):
        f_cur = (sig + kk[i] * Uopp) ** 2 - G * kk[i] * np.tanh(kk[i] * h)
        new = (~found) & (f_prev > 0) & (f_cur <= 0)
        lo = np.where(new, kk[i - 1], lo)
        hi = np.where(new, kk[i], hi)
        found |= new
        f_prev = f_cur
    for _ in range(48):
        mid = 0.5 * (lo + hi)
        fm = (sig + mid * Uopp) ** 2 - G * mid * np.tanh(mid * h)
        hi = np.where(fm <= 0, mid, hi)
        lo = np.where(fm > 0, mid, lo)
    k = 0.5 * (lo + hi)
    blocked = ~found
    k = np.where(blocked, kk[-1] * 0.25, k)
    return k, blocked


def component(h, cell, T, a0, dir_deg, u, v, bar_normal_deg=BAR_NORMAL_DEG):
    """One spectral component on the (u, v) current field.

    dir_deg is THIS component's from-direction (mwd + spread*(2 g_i - 1)); both the Doppler
    resolve and the Snell incidence angle use it - NOT the mean mwd (fidelity-verified).
    Returns dict with a_raw (pre-limiter), k, phi (t=0 spatial phase), sigma, dhat, blocked.
    """
    sig = 2.0 * np.pi / T
    prop = np.radians((dir_deg + 180.0) % 360.0)          # waves travel toward here
    d = np.array([np.sin(prop), np.cos(prop)])            # (east, north), unit
    along = u * d[0] + v * d[1]
    Uopp = np.maximum(-along, 0.0)                        # only the opposing part shortens

    c0 = G * T / (2.0 * np.pi)
    cg0 = 0.5 * c0
    k, blocked = dispersion_k(T, h, Uopp)

    c = (2.0 * np.pi / k) / T                             # ground-frame-solved intrinsic c
    kh = np.clip(k * np.maximum(h, 0.15), 1e-4, 30.0)
    n = 0.5 * (1.0 + 2.0 * kh / np.sinh(2.0 * kh))
    cg = n * c
    cg_eff = np.maximum(cg + along, 0.15)
    Ks = np.sqrt(cg0 / cg_eff)                            # energy-flux shoaling

    th0 = np.radians(((dir_deg + 180.0) - bar_normal_deg + 180.0) % 360.0 - 180.0)
    sin_t = np.clip(np.sin(th0) * c / c0, -0.999, 0.999)
    Kr = np.sqrt(max(np.cos(th0), 1e-3) / np.maximum(np.cos(np.arcsin(sin_t)), 1e-3))

    a_raw = a0 * Ks * Kr

    # phase gauge: exact d(phi)/dx per cell; d(phi)/dy carries the ROW MEAN of k*d_n
    kx = k * d[0] * cell
    ky = k * d[1] * cell
    phi = np.cumsum(kx, axis=1)
    phi += np.cumsum(ky.mean(axis=1, keepdims=True), axis=0)

    # dispersion residual of the returned k (proof-only diagnostic, unblocked cells)
    hf = np.maximum(h, 0.15)
    resid = (sig + k * Uopp) ** 2 - G * k * np.tanh(k * hf)
    return dict(a_raw=a_raw, k=k, phi=phi, sigma=sig, dhat=d, blocked=blocked,
                resid=resid, Uopp=Uopp)


def build_spectrum(h, cell, Hs, Tp, mwd, u, v, n_comp=N_COMP, spread_deg=SPREAD):
    """16 components: JONSWAP gamma=1 sqrt(S df) weights, golden-ratio directions,
    TOTAL-Hs limiter (one uniform factor on the rms envelope, GAMMA_HS = 0.60)."""
    fp = 1.0 / Tp
    fr = np.geomspace(0.62 * fp, 2.30 * fp, n_comp)
    S = fr ** -5.0 * np.exp(-1.25 * (fp / fr) ** 4)
    wts = np.sqrt(np.maximum(S * np.gradient(fr), 0.0))
    wts = wts / np.sqrt(np.sum(wts ** 2))                 # sum w^2 = 1 exactly
    g = (np.arange(n_comp) * GOLDEN) % 1.0
    dirs = mwd + spread_deg * (2.0 * g - 1.0)
    a_tot = Hs / 4.0 * np.sqrt(2.0)                       # Hs = 4 sigma_eta = 2 sqrt2 rms

    comps = [component(h, cell, Tp * (fp / fr[i]), a_tot * wts[i], dirs[i], u, v)
             for i in range(n_comp)]

    rms_raw = np.sqrt(np.sum([c["a_raw"] ** 2 for c in comps], axis=0))
    rms_limit = GAMMA_HS * np.maximum(h, 0.05) / (2.0 * np.sqrt(2.0))
    excess_total = rms_raw / np.maximum(rms_limit, 1e-6)
    limiter = np.minimum(1.0, 1.0 / np.maximum(excess_total, 1e-6))
    for c in comps:
        c["a"] = c["a_raw"] * limiter
    return comps, dict(rms_raw=rms_raw, rms_limit=rms_limit, excess=excess_total,
                       limiter=limiter, wts=wts, dirs=dirs, fr=fr, i_peak=int(np.argmax(wts)))


# --------------------------------------------------------------------------------------------------
#  Figure (PIL - no matplotlib)
# --------------------------------------------------------------------------------------------------
_STOPS = np.array([[13, 8, 135], [126, 3, 168], [203, 71, 119],
                   [248, 149, 64], [240, 249, 33]], dtype=np.float64)


def _cmap(x01):
    """Simple 5-stop plasma-like colormap; x01 in [0,1] -> uint8 RGB."""
    t = np.clip(x01, 0.0, 1.0) * (len(_STOPS) - 1)
    i0 = np.clip(t.astype(np.int64), 0, len(_STOPS) - 2)
    f = (t - i0)[..., None]
    rgb = _STOPS[i0] * (1.0 - f) + _STOPS[i0 + 1] * f
    return rgb.astype(np.uint8)


def render_figure(path, eta, k_peak, wet, T_peak):
    ny, nx = eta.shape
    dry = np.array([28, 24, 20], dtype=np.uint8)

    amp = max(2.5 * float(np.std(eta[wet])), 1e-6)
    g8 = np.clip((eta / amp) * 0.5 + 0.5, 0.0, 1.0)
    p1 = (g8[..., None] * 255.0).astype(np.uint8).repeat(3, axis=2)
    p1[~wet] = dry

    lo, hi = np.percentile(k_peak[wet], [1.0, 99.0])
    p2 = _cmap((k_peak - lo) / max(hi - lo, 1e-9))
    p2[~wet] = dry

    gap = np.full((6, nx, 3), 255, dtype=np.uint8)
    img = Image.fromarray(np.concatenate([p1, gap, p2], axis=0))
    dr = ImageDraw.Draw(img)
    dr.text((8, 6), "eta(t=0) [m], chop_big Hs 2.60 Tp 6.0 mwd 75, tide +0.30 (row 0 = north)",
            fill=(255, 255, 90))
    dr.text((8, ny + 12), "k of peak component (T=%.2f s) [rad/m %.3f..%.3f] - "
            "shortening over the bar" % (T_peak, lo, hi), fill=(255, 255, 255))
    img.save(path)
    print("figure -> %s  (%d x %d)" % (path, img.width, img.height))


# --------------------------------------------------------------------------------------------------
#  Twin test
# --------------------------------------------------------------------------------------------------
def main():
    print("== wave_field: GAGAME solved wave field vs vqview reference bake ==")

    z = vqview_ref.load_bathy()
    u, v, ctide = vqview_ref.load_current(sign=1.0)
    h = np.maximum(TIDE - z, 0.0)
    # wet mask uses the limiter depth floor 0.05 m (matches the reference session's 81.3%;
    # h > 0 gives 81.42% - sliver cells shallower than the rms-limit floor are not "wet sea")
    wet = h > 0.05
    p99 = float(np.percentile(np.hypot(u, v), 99))
    print("bathy %s  wet %.1f%%  current p99 %.3f m/s  npz tide %+.2f" % (
        z.shape, 100.0 * wet.mean(), p99, ctide))
    check("current_vintage", abs(p99 - 0.594) < 5e-4 and abs(ctide - TIDE) < 1e-6,
          "p99 %.4f (want 0.594), tide %+.2f (want +0.30)" % (p99, ctide))

    print("solving 16 components (96-pt bracket + 48 bisections, 690x1720)...")
    comps, agg = build_spectrum(h, CELL, HS, TP, MWD, u, v)

    # -- dispersion residual + physical branch health (unblocked cells) ---------------------------
    worst_rel = 0.0
    n_blocked = 0
    for c in comps:
        ub = ~c["blocked"]
        n_blocked += int(c["blocked"].sum())
        worst_rel = max(worst_rel, float(np.abs(c["resid"][ub]).max()) / c["sigma"] ** 2)
    check("dispersion_residual", worst_rel <= 1e-9,
          "max |f(k)|/sigma^2 = %.2e (tol 1e-9)" % worst_rel)
    frac_blocked = n_blocked / (N_COMP * h.size)
    check("blocked_fraction", frac_blocked <= 5e-5,
          "%d blocked comp-cells (%.4f%%; chop_big never kinematically blocks)" % (
              n_blocked, 100.0 * frac_blocked))

    # -- reference decode --------------------------------------------------------------------------
    sol = vqview_ref.load_solution("chop_big")
    kmax = [float(x) for x in sol["meta"]["kMax"]]
    amax = [float(x) for x in sol["meta"]["aMax"]]
    env_max = float(sol["meta"]["envMax"])
    exc_max = float(sol["meta"]["excMax"])
    sum_max = float(sol["meta"]["sumMax"])

    # -- uniforms pin: sigma_i and dhat_i ---------------------------------------------------------
    sig_ours = np.array([c["sigma"] for c in comps])
    dir_ours = np.array([c["dhat"] for c in comps])
    e_sig = float(np.abs(sig_ours - sol["sigma"]).max())
    e_dir = float(np.abs(dir_ours - sol["dir"]).max())
    check("uniforms_sigma_dhat", e_sig <= 1e-9 and e_dir <= 1e-9,
          "max |dsigma| %.2e, max |ddhat| %.2e (tol 1e-9)" % (e_sig, e_dir))

    # -- per-component k, a, phase ----------------------------------------------------------------
    print("\n comp  T[s]   dir[deg]  |dk|max/LSB  |da|max/LSB   relRMS(k)  relRMS(a)   "
          "phase mean/max [rad]")
    ok_k = ok_a = True
    dot_worst = 1.0
    err_all = []
    for i, c in enumerate(comps):
        s = i // 4
        lsb_k, lsb_a = kmax[s] / 255.0, amax[s] / 255.0
        k_ref = sol["k"][i] + 0.5 * lsb_k          # unbiased truncation-centered decode
        a_ref = sol["a"][i] + 0.5 * lsb_a
        dk = np.abs(np.clip(c["k"], 0.0, kmax[s]) - k_ref)
        da = np.abs(np.clip(c["a"], 0.0, amax[s]) - a_ref)
        dk_max, da_max = float(dk.max()), float(da.max())
        ok_k &= dk_max <= 0.75 * lsb_k
        ok_a &= da_max <= 0.75 * lsb_a
        rel_k = float(np.sqrt(np.mean(dk[wet] ** 2)) / np.sqrt(np.mean(k_ref[wet] ** 2)))
        rel_a = float(np.sqrt(np.mean(da[wet] ** 2)) / np.sqrt(np.mean(a_ref[wet] ** 2)))

        cos_r = sol["cos"][i] + 1.0 / 255.0        # (byte+0.5)/255*2-1 == stored + 1/255
        sin_r = sol["sin"][i] + 1.0 / 255.0
        nrm = np.hypot(cos_r, sin_r)
        dot = np.clip((cos_r * np.cos(c["phi"]) + sin_r * np.sin(c["phi"])) / nrm, -1.0, 1.0)
        dot_worst = min(dot_worst, float(dot.min()))
        err = np.arccos(dot)
        err_all.append(err.astype(np.float32).ravel())
        print("  %2d  %5.2f   %7.2f     %5.3f        %5.3f       %.2e   %.2e    %.4f / %.4f" % (
            i, 2 * np.pi / c["sigma"], agg["dirs"][i], dk_max / lsb_k, da_max / lsb_a,
            rel_k, rel_a, float(err.mean()), float(err.max())))

    check("k_vs_textures", ok_k, "every component max |dk| <= 0.75 LSB of its set")
    check("a_vs_textures", ok_a, "every component max |da| <= 0.75 LSB of its set")

    err_all = np.concatenate(err_all)
    e_mean, e_p99, e_max = (float(err_all.mean()), float(np.percentile(err_all, 99)),
                            float(err_all.max()))
    print("\nphase aggregate: mean %.4f  p99 %.4f  max %.4f rad   worst spinor dot %.6f" % (
        e_mean, e_p99, e_max, dot_worst))
    check("phase_spinor", e_mean <= 0.003 and e_p99 <= 0.006 and e_max <= 0.008
          and dot_worst >= 0.99997,
          "tol: mean<=0.003 p99<=0.006 max<=0.008 rad, dot>=0.99997")
    del err_all

    # -- env fields (rms / excess / coherent sum of the LIMITED set) ------------------------------
    rms = agg["rms_raw"] * agg["limiter"]
    wsum = np.sum([c["a"] for c in comps], axis=0)
    d_r = float(np.abs(np.clip(rms, 0, env_max) - (sol["env_rms"] + 0.5 * env_max / 255)).max())
    d_g = float(np.abs(np.clip(agg["excess"], 0, exc_max)
                       - (sol["env_exc"] + 0.5 * exc_max / 255)).max())
    d_b = float(np.abs(np.clip(wsum, 0, sum_max) - (sol["env_sum"] + 0.5 * sum_max / 255)).max())
    check("env_rgb", d_r <= 0.75 * env_max / 255 and d_g <= 0.75 * exc_max / 255
          and d_b <= 0.75 * sum_max / 255,
          "|d| rms %.5f exc %.5f sum %.5f (0.75 LSB = %.5f/%.5f/%.5f)" % (
              d_r, d_g, d_b, 0.75 * env_max / 255, 0.75 * exc_max / 255, 0.75 * sum_max / 255))

    # -- phase-gauge increment identities (all 16 components) -------------------------------------
    # x: d(phi)/dx is exact per cell.  y: the row-to-row increment of phi MINUS the x-cumsum's
    # own row change equals the row-mean of k*d_n*CELL (the y-term of the gauge; the raw
    # phi(j,i)-phi(j-1,i) additionally carries the x-integral's row dependence by construction).
    ex = ey = 0.0
    for c in comps:
        d_e, d_n = c["dhat"]
        ex = max(ex, float(np.abs((c["phi"][:, 1:] - c["phi"][:, :-1])
                                  - c["k"][:, 1:] * d_e * CELL).max()))
        X = np.cumsum(c["k"] * d_e * CELL, axis=1)
        rk = (c["k"] * d_n * CELL).mean(axis=1)
        ey = max(ey, float(np.abs((c["phi"][1:, :] - c["phi"][:-1, :])
                                  - (X[1:, :] - X[:-1, :]) - rk[1:, None]).max()))
    check("gauge_increments", ex <= 1e-11 and ey <= 1e-11,
          "x %.2e, y %.2e (tol 1e-11)" % (ex, ey))

    # -- headline physics -------------------------------------------------------------------------
    hs_field = 2.0 * np.sqrt(2.0) * rms
    wet_pct = 100.0 * wet.mean()
    hs50 = float(np.percentile(hs_field[wet], 50))
    hs99 = float(np.percentile(hs_field[wet], 99))
    exc_pct = 100.0 * float((agg["excess"][wet] > 1.0).mean())
    print("headline: wet %.1f%%  Hs p50 %.3f p99 %.3f m  excess>1 %.1f%%  blocked %.2f%%" % (
        wet_pct, hs50, hs99, exc_pct, 100.0 * frac_blocked))
    check("headline_physics",
          abs(wet_pct - 81.3) <= 0.1 and abs(hs50 - 2.38) <= 0.01
          and abs(hs99 - 2.76) <= 0.01 and abs(exc_pct - 42.6) <= 0.2,
          "want wet 81.3+-0.1%%, Hs p50 2.38 p99 2.76 (+-0.01), excess>1 42.6+-0.2%%")

    # -- figure -----------------------------------------------------------------------------------
    ip = agg["i_peak"]
    eta = np.zeros_like(h)
    for c in comps:
        eta += c["a"] * np.cos(c["phi"])
    render_figure(os.path.join(HERE, "wave_field.png"), eta, comps[ip]["k"], wet,
                  2 * np.pi / comps[ip]["sigma"])

    print("\n[match] %s -- GAGAME wave field vs vqview textures_wide chop_big, "
          "texel-for-texel at the 8-bit floor" % ("AGREES" if not FAILS else
                                                  "DIVERGES (inspect: %s)" % ", ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
