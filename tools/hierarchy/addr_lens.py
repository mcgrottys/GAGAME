"""Read --lens addr's HDR dump (plan_address.md): the per-pixel distance, in z17 texels, between
today's window address (PageUv of the float32 direction) and the new one (PageUvAbout of the
pixel's point), and the TRUTH SQUARES -- two 256x256 blocks at the bottom left whose pixels address
known points q = (16 i, 0, 16 j) m about the eye -- held against the same points' z17 px taken here
in doubles from the eye the engine logged ([addr] line).

    py -3 tools/hierarchy/addr_lens.py <dump.rgba16f> <engine log> [label]
"""
import json, math, re, sys
import numpy as np

path, logp = sys.argv[1], sys.argv[2]
label = sys.argv[3] if len(sys.argv) > 3 else path
meta = json.load(open(path + ".json"))
W, H = meta["width"], meta["height"]
a = np.fromfile(path, dtype=np.float16).reshape(H, W, 4).astype(np.float64)
line = [l for l in open(logp, encoding="utf-8", errors="replace") if l.startswith("[addr] eye")][-1]
n = [float(x) for x in re.findall(r"-?\d+(?:\.\d+)?(?:e-?\d+)?", line.split("[addr]")[1])]
eye, R, east, up, north = n[0:3], n[3], n[4:7], n[7:10], n[10:13]
detx, dety, dz = n[16], n[17], int(n[18])
N = 256.0 * 2 ** dz

# the squares
ys = np.arange(H - 264, H - 8)
xs = np.arange(8, 520)
blk = a[H - 264:H - 8, 8:520]
j = (H - 9 - ys)[:, None] - 128 + 0 * xs[None, :]
i = ((xs - 8) % 256)[None, :] - 128 + 0 * ys[:, None]
qx, qz = 16.0 * i, 16.0 * j
Wp = [eye[k] + east[k] * qx + north[k] * qz for k in range(3)]
lat = np.arctan2(Wp[1], np.hypot(Wp[0], Wp[2]))
lon = np.arctan2(Wp[2], Wp[0])
rx = (lon / (2 * math.pi) + 0.5) * N - detx
ry = (0.5 - np.arctanh(np.sin(lat)) / (2 * math.pi)) * N - dety
gx = blk[..., 1] + blk[..., 0]
gy = blk[..., 3] + blk[..., 2]
ex = gx - np.mod(np.floor(rx), 2048) - (rx - np.floor(rx))
ey = gy - np.mod(np.floor(ry), 2048) - (ry - np.floor(ry))
# a wrap of the mod-2048 integer part would read as +-2048: fold it
ex = (ex + 1024) % 2048 - 1024
ey = (ey + 1024) % 2048 - 1024
out = {}
for name, sl in (("new (PageUvAbout of q)", slice(0, 256)), ("old (PageUv of dir(q))", slice(256, 512))):
    e = np.maximum(np.abs(ex[:, sl]), np.abs(ey[:, sl]))
    out[name] = e
    print(f"{label}: truth square {name}: z17 texels vs doubles, |err| (worst axis) median {np.median(e):.4f}"
          f"  95th {np.percentile(e, 95):.4f}  worst {e.max():.4f}  mean dx {ex[:, sl].mean():+.4f} dy {ey[:, sl].mean():+.4f}"
          f"  (reach +-2 km, 65536 points)")

# the per-pixel picture, outside the squares
mask = np.ones((H, W), bool)
mask[H - 264:H - 8, 8:520] = False
for tag, nm in ((0.25, "own level (geo)"), (0.5, "other level (dir's point)")):
    m = mask & (a[..., 2] == tag)
    if m.sum() == 0:
        print(f"{label}: pixels {nm}: none")
        continue
    r = a[..., 0][m]
    print(f"{label}: pixels {nm}: n={m.sum()}  |old - new| z17 texels median {np.median(r):.3f}  95th {np.percentile(r, 95):.3f}"
          f"  worst {r.max():.3f}  mean (dx, dy) ({a[..., 1][m].mean():+.3f}, {a[..., 3][m].mean():+.3f})")
