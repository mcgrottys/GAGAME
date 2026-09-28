"""How many 16384^2 windows does one viewer need, as a function of the stride between ranks?

A rung r (texel tau_r = 611.496 m / 2^r) is WANTED on the ground within the distance at which one
texel subtends one pixel: d_r = tau_r / pixelAngle. In that rung's own texels the wanted radius is
1 / pixelAngle, whatever the rung. A window is 16384 texels of its finest rung and carries `stride`
rungs (its mips 0 .. stride-1); its coarsest carried rung wants a radius 2^(stride-1) / pixelAngle
of the window's own texels. Windows needed per rank = the square of (wanted diameter / window
extent), rounded up, at least one.
"""
import math

TAU0 = 40075016.686 / (4.0 * 16384.0)
PAGE = 16384.0


def pixel_angle(h_px, fov_deg):
    return math.radians(fov_deg) / h_px


print("rung  texel(m)     wanted within (1600x900, fovY 60)")
pa = pixel_angle(900, 60.0)
for r in (0, 3, 6, 9, 12, 15, 16):
    tau = TAU0 / 2 ** r
    print(f"{r:>4}  {tau:10.4f}   {tau / pa / 1000.0:10.3f} km")

print()
print("windows per rank per viewer (wanted diameter of the coarsest carried rung / 16384)^2")
print(f"{'display':<14}{'px angle mrad':>14}" + "".join(f"{'stride ' + str(s):>12}" for s in (2, 3, 4, 7)))
for name, h, fov in (("1600x900", 900, 60.0), ("1920x1080", 1080, 60.0), ("2560x1440", 1440, 60.0),
                     ("3840x2160", 2160, 60.0)):
    pa = pixel_angle(h, fov)
    row = f"{name:<14}{pa * 1000.0:>14.3f}"
    for s in (2, 3, 4, 7):
        diam = 2.0 * (2 ** (s - 1)) / pa            # in the window's own mip-0 texels
        n = max(1, math.ceil(diam / PAGE)) ** 2
        row += f"{n:>12}"
    print(row)

print()
print("ranks needed to reach a target texel from the cube's 611.5 m")
for target in (1.0, 0.15, 0.0373, 0.01):
    rung = math.ceil(math.log2(TAU0 / target))
    print(f"  {target*100:8.2f} cm -> rung {rung:>2} ({TAU0 / 2 ** rung * 100:.2f} cm): "
          + ", ".join(f"stride {s}: {math.ceil(rung / s)} ranks" for s in (2, 3, 4, 7)))

print()
print("virtual address per 16384^2 slice (mip 0 + chain), and slices that fit in a budget")
for fmt, bpt in (("RGBA8 / sRGB colour", 4), ("R16F height", 2), ("RGBA16F", 8), ("BC7 colour", 1)):
    gib = PAGE * PAGE * bpt * (4.0 / 3.0) / 2 ** 30
    print(f"  {fmt:<22} {gib:6.2f} GiB/slice   128 slices = {128 * gib:7.1f} GiB   "
          f"256 slices = {256 * gib:7.1f} GiB")

print()
print("gnomonic texel shape on the ground, rung 0 (R ds = R dt = 777.7 m at a face centre)")
R = 6371000.0
ds = 2.0 / 16384.0
for name, s, t in (("face centre", 0.0, 0.0), ("edge middle", 0.0, 1.0), ("corner", 1.0, 1.0),
                   ("Merrimack mouth", -0.3480, -0.9810)):
    q = (1 + s * s + t * t)
    gs = R * ds * math.sqrt(1 + t * t) / q          # ground length of a step in s
    gt = R * ds * math.sqrt(1 + s * s) / q          # ground length of a step in t
    cosang = -s * t / math.sqrt((1 + s * s) * (1 + t * t))
    print(f"  {name:<16} {gs:7.1f} m x {gt:7.1f} m   axes meet at {math.degrees(math.acos(cosang)):6.1f} deg"
          f"   area vs centre {gs * gt * math.sqrt(1 - cosang ** 2) / (R * ds) ** 2:5.3f}")
