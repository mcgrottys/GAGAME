"""Measure, do not argue: how well does float32 find a texel of a cube-face window?

Three spellings of the same window coordinate, against a double reference:
  (a) DIRECT     float32 unit direction -> face uv -> texel      (what a shader does from `dir`)
  (b) MERCATOR   float32 lat/lon -> absolute Mercator px -> uv   (today's PageUv spelling)
  (c) RELATIVE   eye-relative float32 position, the large terms cancelled in double on the CPU:
                 W = k * (A + p.a - s0 * p.n) / (En + p.n)       (a ratio of two plane evaluations)

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
    worst = {"direct": 0.0, "merc": 0.0, "rel": 0.0}
    inside = 0
    mz = 8 + rung                                       # the Mercator zoom of the same nominal grain
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
    tau = TAU0 / (2.0 ** rung)
    print(f"{name:<26} rung {rung:>2} (z{mz:<2}, {tau:9.4f} m nominal)  reach {reach_m:>8.0f} m  "
          f"n={inside:<5} worst texel error:  direct {worst['direct']:10.4f}   "
          f"mercator {worst['merc']:10.4f}   relative {worst['rel']:10.6f}")


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
