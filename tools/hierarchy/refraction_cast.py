"""The pixel water's refracted ray, cast onto the bed: today's arithmetic against the same cast
written in small numbers, both in emulated float32, against doubles. (REVIEW finding 6.)

Globe.hlsl WaterPixelColor marches the refracted ray two secant steps onto the bed:

    TODAY      Pw  = sLvlCamAbs + rel                 (the eye sphere-centred: |Pw| ~ 6.4e6 m)
               gap = (length(Pw + t s) - R) - bed(normalize(Pw + t s))
    PROPOSED   mu  = t . up,   tp = t - mu up         (the ray, split along and across the radial)
               alt = a0 + s mu + s^2 |tp|^2 / (2 (R + a0))
               gap = alt - bed(normalize(up + s / (R + alt) tp))

Both take  s += gap / muD  twice, from  s = depth / muD,  clamped to [0.3, 140].

The bed here is a plane in the tangent frame, bed = -depth0 + slope * x, so the exact landing is
known. Every float32 operation is rounded where the shader would round it. No numpy.
"""
import math
import random
import struct

R = 6371000.0


def f32(x):
    return struct.unpack("f", struct.pack("f", x))[0]


def v32(v):
    return tuple(f32(c) for c in v)


def add32(a, b):
    return tuple(f32(x + y) for x, y in zip(a, b))


def mul32(a, s):
    return tuple(f32(x * s) for x in a)


def dot32(a, b):
    acc = f32(a[0] * b[0])
    acc = f32(acc + f32(a[1] * b[1]))
    return f32(acc + f32(a[2] * b[2]))


def len32(a):
    return f32(math.sqrt(dot32(a, a)))


def norm32(a):
    l = len32(a)
    return tuple(f32(x / l) for x in a)


def bed_of_dir(d, depth0, slope, exact):
    # the tangent-plane x of the direction's sphere point, then the plane's height there
    if exact:
        return -depth0 + slope * (R * d[0] / d[1])
    x = f32(f32(f32(R) * d[0]) / d[1])
    return f32(f32(-depth0) + f32(f32(slope) * x))


def cast_today(eye, p0, t, depth, depth0, slope):
    cam = v32((eye[0], eye[1] + R, eye[2]))                    # sLvlCamAbs, as GlobeLayer fills it
    rel = v32(tuple(p - e for p, e in zip(p0, eye)))           # exact difference, then one cast
    up = norm32(add32(cam, rel))
    pw = add32(cam, rel)
    mud = max(f32(-dot32(t, up)), f32(0.10))
    s = f32(f32(depth) / mud)
    for _ in range(2):
        pb = add32(pw, mul32(t, s))
        gap = f32(f32(len32(pb) - f32(R)) - bed_of_dir(norm32(pb), depth0, slope, False))
        s = min(max(f32(s + f32(gap / mud)), f32(0.3)), f32(140.0))
    return s, add32(pw, mul32(t, s))


def cast_small(up, a0, t, depth, depth0, slope):
    mu = dot32(t, up)
    tp = add32(t, mul32(up, f32(-mu)))
    tp2 = dot32(tp, tp)
    mud = max(f32(-mu), f32(0.10))
    s = f32(f32(depth) / mud)
    a0f, rf = f32(a0), f32(R)
    d = up
    for _ in range(2):
        alt = f32(f32(a0f + f32(s * mu)) + f32(f32(f32(s * s) * tp2) / f32(2.0 * f32(rf + a0f))))
        d = norm32(add32(up, mul32(tp, f32(s / f32(rf + alt)))))
        gap = f32(alt - bed_of_dir(d, depth0, slope, False))
        s = min(max(f32(s + f32(gap / mud)), f32(0.3)), f32(140.0))
    alt = f32(f32(a0f + f32(s * mu)) + f32(f32(f32(s * s) * tp2) / f32(2.0 * f32(rf + a0f))))
    return s, norm32(add32(up, mul32(tp, f32(s / f32(rf + alt))))), alt


def cast_ref(p0c, t, depth, depth0, slope):
    # the same two steps in doubles: the arithmetic is what is under test, not the iteration
    l0 = math.sqrt(sum(c * c for c in p0c))
    up = tuple(c / l0 for c in p0c)
    mud = max(-sum(a * b for a, b in zip(t, up)), 0.10)
    s = depth / mud
    for _ in range(2):
        pb = tuple(p + c * s for p, c in zip(p0c, t))
        lb = math.sqrt(sum(c * c for c in pb))
        gap = (lb - R) - bed_of_dir(tuple(c / lb for c in pb), depth0, slope, True)
        s = min(max(s + gap / mud, 0.3), 140.0)
    return s, tuple(p + c * s for p, c in zip(p0c, t))


def run(name, reach, samples=6000, seed=11):
    rnd = random.Random(seed)
    w_today = w_small = 0.0
    g_today = g_small = 0.0
    n = 0
    for _ in range(samples):
        depth0 = 0.3 + rnd.random() * 6.0                       # metres of water over the bed
        slope = (rnd.random() - 0.5) * 0.10                     # a bar's flank
        a0 = (rnd.random() - 0.5) * 1.5                         # level + wave height here
        eye = (0.0, a0 + 2.0 + rnd.random() * 3.0, 0.0)         # a helm's eye, tangent frame
        ang = rnd.random() * 2 * math.pi
        dist = 3.0 + rnd.random() * reach
        x, z = dist * math.cos(ang), dist * math.sin(ang)
        # the surface point on the sphere of radius R + a0 above that tangent-plane place
        dl = math.sqrt(x * x + R * R + z * z)
        upx = (x / dl, R / dl, z / dl)
        p0 = tuple(c * (R + a0) for c in upx)                   # sphere-centred
        p0_t = (p0[0], p0[1] - R, p0[2])                        # tangent frame (origin at anchor)
        # the view ray and its refraction through a level facet
        v = tuple(p - e for p, e in zip(p0_t, eye))
        lv = math.sqrt(sum(c * c for c in v))
        din = tuple(c / lv for c in v)
        ci = -sum(a * b for a, b in zip(din, upx))
        if ci <= 0.02:
            continue
        eta = 1.0 / 1.34
        st2 = eta * eta * max(1.0 - ci * ci, 0.0)
        k = eta * ci - math.sqrt(max(1.0 - st2, 0.0))
        t = tuple(eta * d + k * u for d, u in zip(din, upx))
        lt = math.sqrt(sum(c * c for c in t))
        t = tuple(c / lt for c in t)
        depth = max(a0 - (-depth0 + slope * x), 0.0)
        if depth < 0.05:
            continue
        s_ref, l_ref = cast_ref(p0, t, depth, depth0, slope)
        t32 = v32(t)
        s_a, l_a = cast_today(eye, p0_t, t32, depth, depth0, slope)
        s_b, d_b, alt_b = cast_small(v32(upx), a0, t32, depth, depth0, slope)
        # where each lands on the tangent plane, against the reference
        def ground(pc):
            return (R * pc[0] / pc[1], R * pc[2] / pc[1])
        gr = ground(l_ref)
        ga = ground(l_a)
        gb = (R * d_b[0] / d_b[1], R * d_b[2] / d_b[1])
        w_today = max(w_today, abs(s_a - s_ref))
        w_small = max(w_small, abs(s_b - s_ref))
        g_today = max(g_today, math.hypot(ga[0] - gr[0], ga[1] - gr[1]))
        g_small = max(g_small, math.hypot(gb[0] - gr[0], gb[1] - gr[1]))
        n += 1
    print(f"{name:<22} n={n:<5} worst error in the ray's length:  today {w_today:8.4f} m   "
          f"small {w_small:9.6f} m   |   in where it lands:  today {g_today:8.4f} m   "
          f"small {g_small:9.6f} m")


if __name__ == "__main__":
    print("the refracted cast, float32 against doubles; bed a plane, 0.3-6.3 m of water")
    for name, reach in (("within 30 m", 30.0), ("within 300 m", 300.0), ("within 3 km", 3000.0)):
        run(name, reach)
