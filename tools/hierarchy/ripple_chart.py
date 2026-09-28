"""The pixel water's ripple coordinate: where the cascade sea's chart is read. (REVIEW finding 7.)

sim/WaveChart.h says where a place sits in a chart's plane:   u = (P - org) . e + off,
with P the place on the sphere, org / e / n the chart cell's centre and axes, ALL IN THE PLANET
FRAME. The bank's kernel fills its texels by that law. The pixel stage must read the same plane.

    TODAY      ChartUOf(upT * R)       upT is the direction in the TANGENT frame; the rows are
                                       the planet frame's, copied unchanged (GlobeLayer.cpp)
    PROPOSED   u = q . eT + c          q  = the sphere point relative to the tangent point, in
                                            tangent axes: (x, -(x^2 + z^2) / (R (1 + upT.y)), z)
                                       eT = the chart's axis turned into the tangent frame (CPU)
                                       c  = (R up_anchor - org) . e + off, in double, wrapped to
                                            the cascade's period before it is cast

Errors are taken modulo the cascade's period, since the patch is periodic. float32 is emulated
operation by operation. No numpy.
"""
import math
import random
import struct

R = 6371000.0
PERIODS = (756.0, 186.0, 47.0)


def f32(x):
    return struct.unpack("f", struct.pack("f", x))[0]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def dot32(a, b):
    acc = f32(a[0] * b[0])
    acc = f32(acc + f32(a[1] * b[1]))
    return f32(acc + f32(a[2] * b[2]))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def dir_of(lat_deg, lon_deg):
    la, lo = math.radians(lat_deg), math.radians(lon_deg)
    return (math.cos(la) * math.cos(lo), math.sin(la), math.cos(la) * math.sin(lo))


def frame_at(d):
    # WaveChart::FrameAt: east = d(dir)/dlon normalized, north = east x up (the engine's order)
    yl = math.hypot(d[0], d[2])
    east = (-d[2] / yl, 0.0, d[0] / yl)
    return east, cross(east, d)


def cube_dir(face, u, v):
    s, t = u * 2 - 1, v * 2 - 1
    p = {0: (1, -t, -s), 1: (-1, -t, s), 2: (s, 1, t), 3: (s, -1, -t), 4: (s, -t, 1)}.get(
        face, (-s, -t, -1))
    l = math.sqrt(dot(p, p))
    return tuple(c / l for c in p)


def face_of(d):
    a = [abs(c) for c in d]
    if a[0] >= a[1] and a[0] >= a[2]:
        f, s, t = (0, -d[2] / a[0], -d[1] / a[0]) if d[0] > 0 else (1, d[2] / a[0], -d[1] / a[0])
    elif a[1] >= a[2]:
        f, s, t = (2, d[0] / a[1], d[2] / a[1]) if d[1] > 0 else (3, d[0] / a[1], -d[2] / a[1])
    else:
        f, s, t = (4, d[0] / a[2], -d[1] / a[2]) if d[2] > 0 else (5, -d[0] / a[2], -d[1] / a[2])
    return f, s * 0.5 + 0.5, t * 0.5 + 0.5


def wrap(d, period):
    return (d + period / 2) % period - period / 2


def run(name, lat, lon, reach, samples=4000, seed=5):
    rnd = random.Random(seed)
    up_a = dir_of(lat, lon)                         # the anchor: the tangent frame's rows
    east_a, north_a = frame_at(up_a)
    # the chart cell the anchor stands in (WaveChart level 5)
    n = 32
    face, cu, cv = face_of(up_a)
    ix, iy = min(int(cu * n), n - 1), min(int(cv * n), n - 1)
    dc = cube_dir(face, (ix + 0.5) / n, (iy + 0.5) / n)
    e, nn = frame_at(dc)
    org = tuple(c * R for c in dc)
    off = (31415.926535, 27182.818284)              # any offset: the hash's is as arbitrary
    centre_km = math.acos(max(-1.0, min(1.0, dot(dc, up_a)))) * R / 1000.0

    def to_tangent(p):
        return (dot(east_a, p), dot(up_a, p), dot(north_a, p))

    e_t, n_t = to_tangent(e), to_tangent(nn)
    anchor = tuple(c * R for c in up_a)
    c_e = dot(tuple(a - o for a, o in zip(anchor, org)), e) + off[0]
    c_n = dot(tuple(a - o for a, o in zip(anchor, org)), nn) + off[1]

    worst = {"today": [0.0] * 3, "new": [0.0] * 3, "grain": 0.0}
    for _ in range(samples):
        x = (rnd.random() * 2 - 1) * reach
        z = (rnd.random() * 2 - 1) * reach
        dy = math.sqrt(1.0 - (x * x + z * z) / (R * R))
        d = tuple(east_a[i] * (x / R) + up_a[i] * dy + north_a[i] * (z / R) for i in range(3))
        # THE PLACE THE SHADER IS GIVEN is the one its float32 direction names, not the one that
        # was meant: the direction's own grain is the input's error, and no arithmetic after it
        # can take it back. It is reported by itself (`grain`) and the two forms are judged
        # against the place the direction names.
        up32 = tuple(f32(c) for c in d)
        l32 = math.sqrt(dot(up32, up32))
        p_true = tuple(c * R for c in d)
        p = tuple(c * R / l32 for c in up32)
        worst["grain"] = max(worst["grain"],
                             math.sqrt(sum((a - b) ** 2 for a, b in zip(p, p_true))))
        ref = (dot(tuple(a - o for a, o in zip(p, org)), e) + off[0],
               dot(tuple(a - o for a, o in zip(p, org)), nn) + off[1])
        # ---- today: the planet-frame direction in float32, turned to the tangent frame, times R,
        # against planet-frame rows
        upt = (dot32(tuple(f32(c) for c in east_a), up32),
               dot32(tuple(f32(c) for c in up_a), up32),
               dot32(tuple(f32(c) for c in north_a), up32))
        pt = tuple(f32(c * f32(R)) for c in upt)
        r = tuple(f32(a - f32(o)) for a, o in zip(pt, org))
        t_u = f32(dot32(r, tuple(f32(c) for c in e)) + f32(off[0]))
        t_v = f32(dot32(r, tuple(f32(c) for c in nn)) + f32(off[1]))
        # ---- proposed
        wx, wz = pt[0], pt[2]
        qy = f32(-f32(f32(wx * wx) + f32(wz * wz)) / f32(f32(R) * f32(1.0 + upt[1])))
        q = (wx, qy, wz)
        e32 = tuple(f32(c) for c in e_t)
        n32 = tuple(f32(c) for c in n_t)
        for k, period in enumerate(PERIODS):
            worst["today"][k] = max(worst["today"][k], abs(wrap(t_u - ref[0], period)),
                                    abs(wrap(t_v - ref[1], period)))
            ce = f32(c_e % period)
            cn = f32(c_n % period)
            p_u = f32(dot32(q, e32) + ce)
            p_v = f32(dot32(q, n32) + cn)
            worst["new"][k] = max(worst["new"][k], abs(wrap(p_u - ref[0], period)),
                                  abs(wrap(p_v - ref[1], period)))
    ang = math.degrees(math.acos(max(-1.0, min(1.0, dot(e, east_a)))))
    print(f"{name}: the chart cell's centre is {centre_km:.0f} km from the anchor; its east is "
          f"{ang:.2f} deg from the anchor's")
    print(f"    in the tangent frame the chart's east is ({e_t[0]:+.4f}, {e_t[1]:+.4f}, {e_t[2]:+.4f});"
          f" today's rows say ({e[0]:+.4f}, {e[1]:+.4f}, {e[2]:+.4f})")
    print(f"    the float32 direction's own grain, which both forms inherit: {worst['grain']:.3f} m")
    for k, period in enumerate(PERIODS):
        print(f"    cascade {k} ({period:5.0f} m patch), within {reach:.0f} m: worst error  today "
              f"{worst['today'][k]:9.3f} m   proposed {worst['new'][k]:9.6f} m")


if __name__ == "__main__":
    run("Merrimack anchor", 42.81833, -70.81, 200.0)
    run("Merrimack anchor", 42.81833, -70.81, 5000.0)
