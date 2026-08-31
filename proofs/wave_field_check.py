# ==================================================================================================
#  proofs/wave_field_check.py - the C++ port held to the python proof, texel for texel.
#
#  src/sim/WaveField.cpp caches every solve to cache/wave/<fnv>.bin, and (version 2) appends
#  the solve's OWN INPUT PLANES (bed, u, v as float32) after the packed atlas -- the solve is
#  a pure function of (cfg, level, spectrum scalars, bed, u, v) by construction, so this
#  script can re-ask the identical question of proofs/wave_field.py (the authoritative
#  prototype, itself twin-tested against the vqview-inlet bake) WITHOUT running the engine.
#
#  What is compared, per uploaded component (aMax > 0 in the GpuTable):
#    k, a     : dequantized ((byte+0.5)/255 * max, the truncating quantizer's unbiased
#               center) vs the python re-solve clipped to [0, max]; PASS at <= 0.75 LSB
#               (0.5 LSB is the quantization floor; the 0.25 margin absorbs boundary flips
#               from sub-LSB float noise -- the same tolerance the vqview twin test used).
#    phase    : spinor dot between the dequantized (cos,sin) and the re-solved gauge;
#               PASS at worst dot >= 0.9999 (~0.014 rad).
#    env      : rms / excess / coherent-sum planes, <= 0.75 LSB each.
#  Engine decisions layered over the proof are MIRRORED here, not smuggled into it:
#    * dry cells (h <= 0): k held at 0.25*10^0.7, a_raw = 0, BEFORE the gauge cumsum and
#      the limiter (the proof floors h at 0.15 and solves land; the engine skips it).
#    * the limiter/env recomputed from the patched a_raw with the proof's own formulas.
#    * upload gate lambda_deep >= minSamplesPerLambda*cell cross-checked against the aMax
#      pattern; gated slices must be all-zero bytes.
#
#  Cache layout (little-endian, no padding -- mirrored from WaveField.cpp wavecore):
#      head   <IIQ8I12d> = magic 'WAVF', version, key, tableBytes, atlasW, atlasH, nx, ny,
#             nComp, hasInputs, pad, hs, tp, mwdDeg, cellM, spreadDeg, barNormalDeg,
#             gammaHs, minSamplesPerLambda, orgX, orgZ, level, currentMs        (144 B)
#      table  <4f4I80f4f> = the GpuTable verbatim                               (368 B)
#      atlas  atlasW*atlasH*4 RGBA8 (slice s at ((s&1)*nx, (s//2)*ny), row 0 = SOUTH)
#      inputs [hasInputs] bed, u, v float32[ny*nx] planes, row 0 = SOUTH
#
#  Usage:  py proofs/wave_field_check.py [path\to\file.bin]
#    - no argument: newest cache/wave/*.bin; if none exist, DRY MODE (decode self-test
#      on synthetic bytes only) and exit 0 -- the engine wiring dumps the first real file.
#    - the decode self-test always runs first; any FAIL exits nonzero.
# ==================================================================================================
import glob
import os
import struct
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

G = 9.81
HEAD_FMT = "<IIQ8I12d"
HEAD_SIZE = struct.calcsize(HEAD_FMT)          # 144
TABLE_FMT = "<4f4I80f4f"
TABLE_SIZE = struct.calcsize(TABLE_FMT)        # 368
MAGIC = 0x46564157                             # 'WAVF' little-endian
VERSION = 2
HOLD_K = (10.0 ** 0.7) * 0.25                  # the blocked/dry hold, kk[-1]*0.25

FAILS = []


def check(name, ok, detail=""):
    print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, (" -- " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def ceil3(v):
    """The dequant-scale rule: 1.05*max applied by the caller; here round UP to 3 decimals,
    floor 1e-3 (mirrors WaveField.cpp Ceil3)."""
    return max(np.ceil(v * 1000.0) / 1000.0, 1e-3)


def quant8(x, maxv):
    """The truncating quantizer, byte = floor(clip(x/max)*255) (reference exporter law)."""
    return np.floor(np.clip(np.asarray(x, np.float64) / maxv, 0.0, 1.0) * 255.0).astype(np.uint8)


def deq(b, maxv):
    """Unbiased decode center of the truncating quantizer."""
    return (b.astype(np.float64) + 0.5) / 255.0 * maxv


# --------------------------------------------------------------------------------------------------
#  Binary parsing
# --------------------------------------------------------------------------------------------------
def load_bin(path):
    with open(path, "rb") as f:
        buf = f.read()
    if len(buf) < HEAD_SIZE + TABLE_SIZE:
        raise ValueError("file too short (%d B)" % len(buf))
    hv = struct.unpack_from(HEAD_FMT, buf, 0)
    d = dict(zip(["magic", "version", "key", "tableBytes", "atlasW", "atlasH", "nx", "ny",
                  "nComp", "hasInputs", "pad0", "hs", "tp", "mwdDeg", "cellM", "spreadDeg",
                  "barNormalDeg", "gammaHs", "minSamplesPerLambda", "orgX", "orgZ", "level",
                  "currentMs"], hv))
    if d["magic"] != MAGIC:
        raise ValueError("bad magic 0x%08x (want 'WAVF')" % d["magic"])
    if d["version"] != VERSION:
        raise ValueError("cache version %d (this checker speaks %d)" % (d["version"], VERSION))
    if d["tableBytes"] != TABLE_SIZE:
        raise ValueError("tableBytes %d != %d" % (d["tableBytes"], TABLE_SIZE))
    tv = struct.unpack_from(TABLE_FMT, buf, HEAD_SIZE)
    nc = 16                                       # GpuTable rows are fixed kMaxComp = 16
    t = dict(orgX=tv[0], orgZ=tv[1], invCell=tv[2], feather=tv[3],
             nx=tv[4], ny=tv[5], nUsed=tv[6], envSlice=tv[7],
             sigma=np.array(tv[8:8 + nc]), dirX=np.array(tv[24:24 + nc]),
             dirZ=np.array(tv[40:40 + nc]), aMax=np.array(tv[56:56 + nc]),
             kMax=np.array(tv[72:72 + nc]),
             envMax=tv[88], sumMax=tv[89], excMax=tv[90], level=tv[91])
    off = HEAD_SIZE + TABLE_SIZE
    nbytes = d["atlasW"] * d["atlasH"] * 4
    atlas = np.frombuffer(buf, np.uint8, nbytes, off).reshape(d["atlasH"], d["atlasW"], 4)
    off += nbytes
    planes = {}
    if d["hasInputs"]:
        cells = d["nx"] * d["ny"]
        for name in ("bed", "u", "v"):
            planes[name] = np.frombuffer(buf, np.float32, cells, off).reshape(
                d["ny"], d["nx"]).astype(np.float64)
            off += cells * 4
    if off != len(buf):
        raise ValueError("trailing bytes: parsed %d of %d" % (off, len(buf)))
    return d, t, atlas, planes


def slice_of(atlas, nx, ny, s):
    sx, sy = (s & 1) * nx, (s >> 1) * ny
    return atlas[sy:sy + ny, sx:sx + nx, :]


# --------------------------------------------------------------------------------------------------
#  Decode self-test (always runs): synthetic planes -> quantize -> pack -> parse -> dequant.
#  Validates the layout constants and the decode path with no engine and no cache file.
# --------------------------------------------------------------------------------------------------
def dry_selftest():
    print("== decode self-test (synthetic bytes) ==")
    nx, ny, nc = 7, 5, 3
    rows = (nc + 2) // 2
    aw, ah = 2 * nx, rows * ny
    rng = np.random.default_rng(7)
    a = rng.uniform(0.0, 1.7, (nc, ny, nx))
    k = rng.uniform(0.01, 1.3, (nc, ny, nx))
    phi = rng.uniform(0.0, 40.0, (nc, ny, nx))
    amax = [ceil3(1.05 * a[i].max()) for i in range(nc)]
    kmax = [ceil3(1.05 * k[i].max()) for i in range(nc)]
    rms = np.sqrt(np.sum(a ** 2, axis=0))
    envmax = ceil3(1.05 * rms.max())
    wsum = np.sum(a, axis=0)
    summax = ceil3(1.05 * wsum.max())
    exc = rng.uniform(0.0, 3.0, (ny, nx))

    atlas = np.zeros((ah, aw, 4), np.uint8)
    for i in range(nc):
        sl = slice_of(atlas, nx, ny, i)
        sl[..., 0] = quant8(a[i], amax[i])
        sl[..., 1] = quant8(k[i], kmax[i])
        sl[..., 2] = quant8(np.cos(phi[i]) * 0.5 + 0.5, 1.0)
        sl[..., 3] = quant8(np.sin(phi[i]) * 0.5 + 0.5, 1.0)
    env = slice_of(atlas, nx, ny, nc)
    env[..., 0] = quant8(rms, envmax)
    env[..., 1] = quant8(exc, 2.5)
    env[..., 2] = quant8(wsum, summax)

    sig = np.zeros(16); sig[:nc] = [1.0, 1.5, 2.0]
    am16 = np.zeros(16); am16[:nc] = amax
    km16 = np.zeros(16); km16[:nc] = kmax
    table = struct.pack(TABLE_FMT, -320.0, -200.0, 0.5, 120.0, nx, ny, nc, nc,
                        *sig, *np.linspace(0, 1, 16), *np.linspace(1, 0, 16), *am16, *km16,
                        envmax, summax, 2.5, 0.5)
    head = struct.pack(HEAD_FMT, MAGIC, VERSION, 0x1234ABCD5678EF01, TABLE_SIZE, aw, ah,
                       nx, ny, nc, 1, 0, 2.6, 6.0, 75.0, 2.0, 26.0, 285.0, 0.60, 8.0,
                       -320.0, -200.0, 0.5, -0.9)
    bed = rng.uniform(-15, 2, (ny, nx)).astype(np.float32)
    uv = np.zeros((ny, nx), np.float32)
    blob = head + table + atlas.tobytes() + bed.tobytes() + uv.tobytes() + uv.tobytes()

    tmp = os.path.join(tempfile.gettempdir(), "wave_field_check_selftest.bin")
    with open(tmp, "wb") as f:
        f.write(blob)
    d, t, atlas2, planes = load_bin(tmp)
    os.remove(tmp)

    ok_head = (d["nx"] == nx and d["ny"] == ny and d["nComp"] == nc and d["hs"] == 2.6
               and d["tp"] == 6.0 and d["barNormalDeg"] == 285.0 and d["level"] == 0.5)
    check("selftest_header_roundtrip", ok_head)
    check("selftest_bed_roundtrip", float(np.abs(planes["bed"] - bed).max()) == 0.0)
    worst = 0.0
    for i in range(nc):
        sl = slice_of(atlas2, nx, ny, i)
        da = np.abs(deq(sl[..., 0], t["aMax"][i]) - np.clip(a[i], 0, t["aMax"][i]))
        dk = np.abs(deq(sl[..., 1], t["kMax"][i]) - np.clip(k[i], 0, t["kMax"][i]))
        worst = max(worst, float(da.max()) / (t["aMax"][i] / 255.0),
                    float(dk.max()) / (t["kMax"][i] / 255.0))
    check("selftest_dequant_half_lsb", worst <= 0.5 + 1e-9, "worst %.4f LSB" % worst)
    dots = 1.0
    for i in range(nc):
        sl = slice_of(atlas2, nx, ny, i)
        cr, sr = deq(sl[..., 2], 1.0) * 2 - 1, deq(sl[..., 3], 1.0) * 2 - 1
        n = np.hypot(cr, sr)
        dots = min(dots, float(((cr * np.cos(phi[i]) + sr * np.sin(phi[i])) / n).min()))
    check("selftest_spinor_decode", dots >= 0.99995, "worst dot %.6f" % dots)


# --------------------------------------------------------------------------------------------------
#  The twin check: re-solve the cache file's own inputs with the python proof.
# --------------------------------------------------------------------------------------------------
def full_check(path):
    print("\n== wave_field_check: %s ==" % path)
    d, t, atlas, planes = load_bin(path)
    nx, ny, nc = d["nx"], d["ny"], d["nComp"]
    print("window %dx%d @ %.1f m  comps %d  lvl %+.2f  cur %+.2f  Hs %.2f Tp %.1f mwd %.1f" % (
        nx, ny, d["cellM"], nc, d["level"], d["currentMs"], d["hs"], d["tp"], d["mwdDeg"]))
    check("table_geometry", t["nx"] == nx and t["ny"] == ny and t["nUsed"] == nc
          and t["envSlice"] == nc and abs(t["level"] - d["level"]) < 1e-5)

    if d["hs"] <= 0.0:
        check("zero_field_aMax", float(np.abs(t["aMax"]).max()) == 0.0)
        check("zero_field_atlas", int(atlas.max()) == 0)
        print("flat-sea file: nothing further to twin-test.")
        return

    if not d["hasInputs"]:
        check("input_planes_present", False,
              "hasInputs=0 -- cannot re-solve; write the v2 cache from a real solve")
        return

    import wave_field as wf   # the authoritative prototype (deferred: imports PIL)
    # component()'s bar normal is a def-time default and GAMMA_HS a module constant: the
    # config must PIN them, or this checker would be comparing two different problems.
    check("cfg_pins", d["barNormalDeg"] == wf.BAR_NORMAL_DEG and d["gammaHs"] == wf.GAMMA_HS,
          "bar %.1f (want %.1f), gammaHs %.2f (want %.2f)" % (
              d["barNormalDeg"], wf.BAR_NORMAL_DEG, d["gammaHs"], wf.GAMMA_HS))
    if FAILS:
        return

    h = np.maximum(d["level"] - planes["bed"], 0.0)
    dry = h <= 0.0
    print("re-solving %dx%dx%d in python (dry %.1f%%)..." % (
        nx, ny, nc, 100.0 * float(dry.mean())))
    comps, agg = wf.build_spectrum(h, d["cellM"], d["hs"], d["tp"], d["mwdDeg"],
                                   planes["u"], planes["v"], n_comp=nc,
                                   spread_deg=d["spreadDeg"])

    # -- engine dry rule, applied BEFORE gauge and limiter (WaveField.cpp file header) -----
    for c in comps:
        c["k"] = np.where(dry, HOLD_K, c["k"])
        c["a_raw"] = np.where(dry, 0.0, c["a_raw"])
        kx = c["k"] * c["dhat"][0] * d["cellM"]
        ky = c["k"] * c["dhat"][1] * d["cellM"]
        phi = np.cumsum(kx, axis=1)               # SW anchor: x west->east left-inclusive,
        phi += np.cumsum(ky.mean(axis=1, keepdims=True), axis=0)   # y south->north row-mean
        c["phi"] = phi
    rms_raw = np.sqrt(np.sum([c["a_raw"] ** 2 for c in comps], axis=0))
    rms_limit = wf.GAMMA_HS * np.maximum(h, 0.05) / (2.0 * np.sqrt(2.0))
    excess = rms_raw / np.maximum(rms_limit, 1e-6)
    limiter = np.minimum(1.0, 1.0 / np.maximum(excess, 1e-6))
    for c in comps:
        c["a"] = c["a_raw"] * limiter

    # -- uniforms + the upload gate pattern ------------------------------------------------
    sig_py = np.array([c["sigma"] for c in comps])
    e_sig = float(np.abs(sig_py - t["sigma"][:nc]).max())
    e_dir = max(float(np.abs(np.array([c["dhat"][0] for c in comps]) - t["dirX"][:nc]).max()),
                float(np.abs(np.array([c["dhat"][1] for c in comps]) - t["dirZ"][:nc]).max()))
    check("uniforms_sigma_dhat", e_sig <= 1e-4 and e_dir <= 1e-6,
          "max |dsigma| %.2e |ddhat| %.2e (float32 table)" % (e_sig, e_dir))
    gate = 2.0 * np.pi * G / sig_py ** 2 >= d["minSamplesPerLambda"] * d["cellM"]
    check("upload_gate_pattern", bool(np.all((t["aMax"][:nc] > 0) == gate)),
          "uploaded %s" % np.flatnonzero(t["aMax"][:nc] > 0).tolist())

    # -- per-component planes ---------------------------------------------------------------
    print("\n comp  T[s]   dir[deg]  |dk|max/LSB  |da|max/LSB   phase mean/max [rad]   dot")
    ok_k = ok_a = True
    dot_worst = 1.0
    e_mean_all, e_max_all = [], 0.0
    for i, c in enumerate(comps):
        if not gate[i]:
            gz = slice_of(atlas, nx, ny, i)
            check("gated_slice_%02d_zero" % i, int(gz.max()) == 0)
            continue
        sl = slice_of(atlas, nx, ny, i)
        lsb_a, lsb_k = t["aMax"][i] / 255.0, t["kMax"][i] / 255.0
        da = np.abs(deq(sl[..., 0], t["aMax"][i]) - np.clip(c["a"], 0.0, t["aMax"][i]))
        dk = np.abs(deq(sl[..., 1], t["kMax"][i]) - np.clip(c["k"], 0.0, t["kMax"][i]))
        da_max, dk_max = float(da.max()), float(dk.max())
        ok_a &= da_max <= 0.75 * lsb_a
        ok_k &= dk_max <= 0.75 * lsb_k
        cr = deq(sl[..., 2], 1.0) * 2.0 - 1.0
        sr = deq(sl[..., 3], 1.0) * 2.0 - 1.0
        nrm = np.hypot(cr, sr)
        dot = np.clip((cr * np.cos(c["phi"]) + sr * np.sin(c["phi"])) / nrm, -1.0, 1.0)
        dmin = float(dot.min())
        dot_worst = min(dot_worst, dmin)
        err = np.arccos(dot)
        e_mean_all.append(float(err.mean()))
        e_max_all = max(e_max_all, float(err.max()))
        print("  %2d  %5.2f   %7.2f     %5.3f        %5.3f       %.4f / %.4f      %.6f" % (
            i, 2 * np.pi / c["sigma"], agg["dirs"][i], dk_max / lsb_k, da_max / lsb_a,
            float(err.mean()), float(err.max()), dmin))
    check("k_vs_atlas", ok_k, "every uploaded component max |dk| <= 0.75 LSB")
    check("a_vs_atlas", ok_a, "every uploaded component max |da| <= 0.75 LSB")
    check("phase_spinor", dot_worst >= 0.9999,
          "worst dot %.6f (>= 0.9999), err mean %.4f max %.4f rad" % (
              dot_worst, float(np.mean(e_mean_all)) if e_mean_all else 0.0, e_max_all))

    # -- env slice ---------------------------------------------------------------------------
    env = slice_of(atlas, nx, ny, nc)
    rms = rms_raw * limiter
    wsum = np.sum([c["a"] for c in comps], axis=0)
    d_r = float(np.abs(deq(env[..., 0], t["envMax"]) - np.clip(rms, 0, t["envMax"])).max())
    d_g = float(np.abs(deq(env[..., 1], t["excMax"]) - np.clip(excess, 0, t["excMax"])).max())
    d_b = float(np.abs(deq(env[..., 2], t["sumMax"]) - np.clip(wsum, 0, t["sumMax"])).max())
    check("env_rgb", d_r <= 0.75 * t["envMax"] / 255 and d_g <= 0.75 * t["excMax"] / 255
          and d_b <= 0.75 * t["sumMax"] / 255,
          "|d| rms %.5f exc %.5f sum %.5f (0.75 LSB = %.5f/%.5f/%.5f)" % (
              d_r, d_g, d_b, 0.75 * t["envMax"] / 255, 0.75 * t["excMax"] / 255,
              0.75 * t["sumMax"] / 255))

    hs_field = 2.0 * np.sqrt(2.0) * rms
    wet = ~dry
    if wet.any():
        print("\nheadline: wet %.1f%%  Hs p50 %.3f p99 %.3f m  excess>1 %.1f%%" % (
            100.0 * wet.mean(), np.percentile(hs_field[wet], 50),
            np.percentile(hs_field[wet], 99), 100.0 * float((excess[wet] > 1.0).mean())))


def main():
    dry_selftest()
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        cands = glob.glob(os.path.join(HERE, "..", "cache", "wave", "*.bin"))
        if not cands:
            print("\n[DRY MODE] no cache/wave/*.bin yet and no path given -- decode self-test"
                  " only. The engine wiring (or the offline harness) writes the first file.")
            return 1 if FAILS else 0
        path = max(cands, key=os.path.getmtime)
    full_check(path)
    print("\n[match] %s -- WaveField.cpp vs proofs/wave_field.py on the file's own inputs" %
          ("AGREES" if not FAILS else "DIVERGES (inspect: %s)" % ", ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
