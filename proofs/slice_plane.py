# proofs/slice_plane.py -- THE SLICE PLANE, algebra first (M7o).
#
# The cut is a plane pi = (n, d): unit normal n, offset d. A point P is KEPT when the
# signed distance s(P) = P.n - d <= 0 and discarded otherwise -- the predicate is the
# sign of the inner product <P, pi>. The identity the gatest gate pins: reflection
# through pi is the sandwich  P' = P - 2 s(P) n  (the expanded -pi P pi for a normalized
# plane), and s(P') = -s(P): the sandwich NEGATES the signed distance, so the predicate
# splits space into the two halves the sandwich exchanges. That antisymmetry is the whole
# correctness argument for the renderer's discard.
#
# The small render below shows the same statement in 2D before the engine ever runs it:
# a circle "planet" with a synthetic bathymetry profile, sliced at z = d; the kept half
# keeps its fill, the cut edge draws the profile the 3D cutaway must reproduce.
import math

from PIL import Image, ImageDraw

W, H = 900, 520
R = 200
CX, CY = 300, 260
D_CUT = 40.0  # slice offset in "world" units (z of the cut plane), 1 px = 1 unit

im = Image.new("RGB", (W, H), (14, 16, 22))
dr = ImageDraw.Draw(im)

def bathy(x):
    # synthetic shelf-channel-bar profile, metres below the rim
    return (28.0 * math.sin(x * 0.018) + 14.0 * math.sin(x * 0.061 + 1.3) + 36.0)

# proof check, numerically, before drawing anything (the same assertions gatest pins):
for p in [(-3.0, 7.0), (120.0, -45.0), (D_CUT, 0.0), (500.0, D_CUT + 1e-3)]:
    z = p[1]
    s = z - D_CUT
    z_reflected = z - 2.0 * s
    s_reflected = z_reflected - D_CUT
    assert abs(s_reflected + s) < 1e-9, "sandwich must negate the signed distance"

# the planet disc: keep the half with s <= 0 (z <= D_CUT in screen-down coords)
for yy in range(H):
    for xx in range(W):
        dx, dy = xx - CX, yy - CY
        rr = math.hypot(dx, dy)
        if rr > R:
            continue
        z_world = -dy  # screen up = +z
        s = z_world - D_CUT
        if s > 0.0:
            continue  # discarded half -- the renderer's discard, drawn as absence
        depth = bathy(xx) if rr > R - 60 else 0.0
        shade = 60 + int(140 * (1.0 - rr / R))
        im.putpixel((xx, yy), (int(shade * 0.55), int(shade * 0.75), shade))

# the cut edge: the bathymetry profile the cutaway exposes
y_cut = CY - int(D_CUT)
for xx in range(W):
    dxc = xx - CX
    if abs(dxc) > R:
        continue
    prof = int(bathy(xx) * 0.8)
    for yy in range(y_cut, min(y_cut + prof, H - 1)):
        im.putpixel((xx, yy), (208, 176, 96))
    if 0 <= y_cut < H:
        im.putpixel((xx, y_cut), (255, 240, 200))

dr.text((20, 16), "THE SLICE PLANE  pi=(n,d): keep s(P)=P.n-d<=0", fill=(230, 230, 240))
dr.text((20, 34), "sandwich -pi P pi negates s  ->  the predicate splits the halves it "
                  "exchanges", fill=(160, 165, 185))
dr.text((20, 52), "cut edge carries the bathymetry profile the 3D cutaway must show",
        fill=(208, 176, 96))
im.save("proofs/slice_plane.png")
print("proofs/slice_plane.png written; sandwich antisymmetry asserted")
