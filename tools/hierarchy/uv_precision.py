"""Measure, do not argue: how well does float32 find a texel of a cube-face window?

Three spellings of the same window coordinate, against a double reference:
  (a) DIRECT     float32 unit direction -> face uv -> texel      (what a shader does from `dir`)
  (b) MERCATOR   float32 lat/lon -> absolute Mercator px -> uv   (today's PageUv spelling)
  (c) RELATIVE   eye-relative float32 position, the large terms cancelled in double on the CPU:
                 W = k * (A + p.a - s0 * p.n) / (En + p.n)       (a ratio of two plane evaluations)
  (d) MERC ABOUT the Mercator window about the anchor (plan_address.md; PageSample.hlsli's
                 PageMercAbout + PageUvAbout, op for op in the shader's order): the eye's px less
                 the window's origin from doubles, plus the chart's exact difference formed from the
                 eye-relative float32 point -- the same zoom as (b), so its column sits beside (b)'s.

Every float32 operation is rounded after it happens (struct round trip), in the order a shader
would run them. No numpy (absent on this machine).
"""
import math
import random
import struct

R = 6371000.0
TAU0 = 40075016.686 / (4.0 * 16384.0)      # the cube's mip-0 texel, m (Lattice::GroundRes(0))


def f32(x):
    return struct.unpack("f", struct.pack("f", x))[0]


def dir_of(lat, lon):
    cl = math.cos(lat)
    return (cl * math.cos(lon), math.sin(lat), cl * math.sin(lon))


def face5_st(P):
    # Lattice.h CubeFaceOfDir, face 5 (-z): s = -x/|z| = x/z, t = -y/|z| = y/z
    return (P[0] / P[2], P[1] / P[2])


def texel_ref(P, rung):
    n = 16384.0 * (2.0 ** rung)
    s, t = face5_st(P)
    return ((s * 0.5 + 0.5) * n, (t * 0.5 + 0.5) * n)


def texel_direct_f32(P, rung):
    l = math.sqrt(P[0] ** 2 + P[1] ** 2 + P[2] ** 2)
    d = [f32(c / l) for c in P]                      # the interpolated direction, float32
    az = f32(abs(d[2]))
    s = f32(f32(-d[0]) / az)                         # HpCubeFace, face 5: s = -d.x / |d.z|
    t = f32(f32(-d[1]) / az)
    n = f32(16384.0 * (2.0 ** rung))
    u = f32(f32(f32(s * 0.5) + 0.5) * n)
    v = f32(f32(f32(t * 0.5) + 0.5) * n)
    return (u, v)


def merc_px_ref(lat, lon, z):
    w = 256.0 * (2.0 ** z)
    return ((math.degrees(lon) + 180.0) / 360.0 * w,
            (0.5 - math.log(math.tan(math.pi / 4 + lat / 2)) / (2 * math.pi)) * w)


def merc_px_f32(lat, lon, z):
    w = f32(256.0 * (2.0 ** z))
    lonDeg = f32(math.degrees(lon))
    latf = f32(lat)
    mx = f32(f32(f32(lonDeg + 180.0) / 360.0) * w)
    tn = f32(math.tan(f32(f32(0.785398163) + f32(latf * 0.5))))
    my = f32(f32(0.5 - f32(f32(math.log(tn)) / f32(2.0 * 3.14159265358979))) * w)
    return (mx, my)


def _ser(u, c):
    acc = f32(c[-1])
    for k in reversed(c[:-1]):
        acc = f32(f32(acc * u) + f32(k))
    return acc


_ATAN = [1.0, -1 / 3, 1 / 5, -1 / 7, 1 / 9, -1 / 11, 1 / 13]
_ATANH = [1.0, 1 / 3, 1 / 5, 1 / 7, 1 / 9, 1 / 11, 1 / 13]


def merc_about_f32(p_enu, eyeA, k, epx):
    """PageMercAbout then PageUvAbout, float32 op by op. p_enu: the point in the eye's east,
    north, up (the shader's three dots, taken as exact here); eyeA = (sin phi, cos phi, rho_E,
    R_E); k = world px / 2 pi; epx = the eye's px less the window's origin. Returns window px."""
    pe, pn, pu = (f32(c) for c in p_enu)
    sphi, cphi, rho, re = (f32(c) for c in eyeA)
    a = f32(f32(rho + f32(pu * cphi)) - f32(pn * sphi))
    if abs(pe) < f32(0.25 * a):
        t = f32(pe / a)
        dlon = f32(t * _ser(f32(t * t), _ATAN))
    else:
        dlon = f32(math.atan2(pe, a))
    h2 = f32(f32(pe * pe) + f32(pn * pn))
    ru = f32(re + pu)
    w = f32(math.sqrt(f32(f32(ru * ru) + h2)))
    d = f32(f32(h2 + f32(pu * f32(f32(2.0 * re) + pu))) / f32(w + re))
    s = f32(f32(f32(pn * cphi) + f32(sphi * f32(pu - d))) / w)
    x = f32(s / f32(f32(cphi * cphi) - f32(sphi * s)))
    if abs(x) < 0.25:
        dpsi = f32(x * _ser(f32(x * x), _ATANH))
    else:
        dpsi = f32(0.5 * f32(math.log(f32(f32(1.0 + x) / f32(1.0 - x)))))
    kf = f32(k)
    return (f32(f32(epx[0]) + f32(dlon * kf)), f32(f32(epx[1]) + f32(f32(-dpsi) * kf)))


def texel_relative_f32(P, E, rung, org):
    """Window texel from the eye-relative position. `org` = the window's origin, in face texels."""
    n = 16384.0 * (2.0 ** rung)
    out = []
    for axis in (0, 1):
        s0 = 2.0 * org[axis] / n - 1.0                 # the origin edge's s (double, CPU)
        A = f32(E[axis] - s0 * E[2])                   # the big cancellation, in double, THEN cast
        k = f32(0.5 * n)
        s0f = f32(s0)
        p_a = f32(P[axis] - E[axis])                   # eye-relative, as the vertex path carries it
        p_n = f32(P[2] - E[2])
        En = f32(E[2])
        num = f32(f32(A + p_a) - f32(s0f * p_n))
        den = f32(En + p_n)
        out.append(f32(f32(num / den) * k))
    return tuple(out)


def run(name, lat0, lon0, eye_alt, reach_m, rung, samples=4000, seed=7):
    rnd = random.Random(seed)
    lat0r, lon0r = math.radians(lat0), math.radians(lon0)
    d0 = dir_of(lat0r, lon0r)
    E = tuple(c * (R + eye_alt) for c in d0)
    assert abs(d0[2]) >= abs(d0[0]) and abs(d0[2]) >= abs(d0[1]) and d0[2] < 0, "not on face 5"
    eu, ev = texel_ref(E, rung)
    # the window: 16384 texels, origin snapped to the half-page lattice, eye in its central half
    org = (math.floor(eu / 8192.0 - 0.5) * 8192.0, math.floor(ev / 8192.0 - 0.5) * 8192.0)
    worst = {"direct": 0.0, "merc": 0.0, "rel": 0.0, "about": 0.0}
    inside = 0
    mz = 8 + rung                                       # the Mercator zoom of the same nominal grain
    # (d)'s window: 16384 px with the eye's px 8192 in, its origin a whole px; the eye's frame.
    rE = math.sqrt(sum(c * c for c in E))
    sphi, cphi = E[1] / rE, math.hypot(E[0], E[2]) / rE
    ex_d, ey_d = merc_px_ref(math.asin(sphi), math.atan2(E[2], E[0]), mz)
    morg = (math.floor(ex_d) - 8192.0, math.floor(ey_d) - 8192.0)
    epx = (ex_d - morg[0], ey_d - morg[1])
    eyeA = (sphi, cphi, rE * cphi, rE)
    kmz = 256.0 * (2.0 ** mz) / (2.0 * math.pi)
    lonE = math.atan2(E[2], E[0])
    eE = (-math.sin(lonE), 0.0, math.cos(lonE))
    eN = (-sphi * math.cos(lonE), cphi, -sphi * math.sin(lonE))
    eU = tuple(c / rE for c in E)
    for _ in range(samples):
        # a ground point within reach of the eye
        dn = (rnd.random() * 2 - 1) * reach_m
        de = (rnd.random() * 2 - 1) * reach_m
        lat = lat0r + dn / R
        lon = lon0r + de / (R * math.cos(lat0r))
        d = dir_of(lat, lon)
        P = tuple(c * R for c in d)
        ru, rv = texel_ref(P, rung)
        wu, wv = ru - org[0], rv - org[1]
        if not (0.0 <= wu < 16384.0 and 0.0 <= wv < 16384.0):
            continue
        inside += 1
        du, dv = texel_direct_f32(P, rung)
        worst["direct"] = max(worst["direct"], abs(du - ru), abs(dv - rv))
        mu, mv = merc_px_ref(lat, lon, mz)
        fu, fv = merc_px_f32(lat, lon, mz)
        worst["merc"] = max(worst["merc"], abs(fu - mu), abs(fv - mv))
        qu, qv = texel_relative_f32(P, E, rung, org)
        worst["rel"] = max(worst["rel"], abs(qu - wu), abs(qv - wv))
        pr = tuple(P[i] - E[i] for i in range(3))
        penu = tuple(sum(pr[i] * ax[i] for i in range(3)) for ax in (eE, eN, eU))
        au, av = merc_about_f32(penu, eyeA, kmz, epx)
        worst["about"] = max(worst["about"], abs(au - (mu - morg[0])), abs(av - (mv - morg[1])))
    tau = TAU0 / (2.0 ** rung)
    print(f"{name:<26} rung {rung:>2} (z{mz:<2}, {tau:9.4f} m nominal)  reach {reach_m:>8.0f} m  "
          f"n={inside:<5} worst texel error:  direct {worst['direct']:10.4f}   "
          f"mercator {worst['merc']:10.4f}   relative {worst['rel']:10.6f}   "
          f"merc about {worst['about']:9.6f}")


if __name__ == "__main__":
    print(f"cube mip-0 texel {TAU0:.3f} m; Merrimack mouth 42.816 N 70.8125 W is on face 5")
    d = dir_of(math.radians(42.816), math.radians(-70.8125))
    s, t = face5_st(d)
    print(f"  face-5 uv = ({s*0.5+0.5:.5f}, {t*0.5+0.5:.5f});"
          f" texels from the v=0 edge at mip 0: {(t*0.5+0.5)*16384:.1f}")
    # latitude where face 5 meets face 2 (+y) on this meridian: tan(lat) = |sin(lon)|
    late = math.degrees(math.atan(abs(math.sin(math.radians(-70.8125)))))
    print(f"  the face 5 / face 2 seam on this meridian is at {late:.3f} N,"
          f" {(late-42.816)*math.pi/180*R/1000:.1f} km north of the mouth")
    print()
    for rung, reach in ((6, 60000.0), (9, 8000.0), (12, 1000.0), (15, 120.0), (16, 60.0)):
        run("Merrimack helm (eye 3 m)", 42.816, -70.8125, 3.0, reach, rung)
    print()
    for rung, reach in ((6, 60000.0), (9, 8000.0), (12, 1000.0), (15, 120.0)):
        run("Merrimack 10 km up", 42.816, -70.8125, 10000.0, reach, rung)
