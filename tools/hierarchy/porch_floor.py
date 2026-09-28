"""Two small proofs by exhaustion, so the design doc states measured facts.

1. THE FLOOR. Today's residency clamp is gather + max over the 2x2 nearest map cells: sound, and
   piecewise constant (the sharpness of the picture steps at half-cell lines). The proposed
   clamp is DILATE (each cell takes the max of its 3x3 neighbourhood, on the CPU, when the map is
   written) then plain BILINEAR. Claim: it is never finer than today's clamp (so it is at least
   as sound) and it is continuous. Checked on random maps at dense sample points.

2. THE TORUS. A window is 16384 texels of its finest rung, addressed modulo its size at every
   mip. Claim: a global texel X0 at the window's rung lands, at mip m, in slot (X0 >> m) mod
   (16384 >> m), and that equals ((X0 mod 16384) >> m) -- i.e. the hardware's own mip chain of a
   WRAP-addressed slice agrees with the global lattice at every mip, for any window origin.
"""
import random

N = 24          # map cells a side (a slice has 128; the law does not care)


def gather_max(m, x, y):
    # the 2x2 cells whose centres surround (x, y); cell centres at i + 0.5
    i0 = int(min(max(x - 0.5, 0), N - 1))
    j0 = int(min(max(y - 0.5, 0), N - 1))
    i1, j1 = min(i0 + 1, N - 1), min(j0 + 1, N - 1)
    return max(m[j0][i0], m[j0][i1], m[j1][i0], m[j1][i1])


def dilate(m):
    d = [[0] * N for _ in range(N)]
    for j in range(N):
        for i in range(N):
            d[j][i] = max(m[jj][ii]
                          for jj in range(max(j - 1, 0), min(j + 2, N))
                          for ii in range(max(i - 1, 0), min(i + 2, N)))
    return d


def bilinear(d, x, y):
    fx, fy = min(max(x - 0.5, 0.0), N - 1.0), min(max(y - 0.5, 0.0), N - 1.0)
    i0, j0 = int(fx), int(fy)
    i1, j1 = min(i0 + 1, N - 1), min(j0 + 1, N - 1)
    tx, ty = fx - i0, fy - j0
    return ((1 - tx) * (1 - ty) * d[j0][i0] + tx * (1 - ty) * d[j0][i1] +
            (1 - tx) * ty * d[j1][i0] + tx * ty * d[j1][i1])


def floor_law(trials=200, seed=3):
    rnd = random.Random(seed)
    unsound = 0
    worst_step_old = 0.0
    worst_step_new = 0.0
    extra = 0.0
    samples = 0
    for _ in range(trials):
        # a residency map: finest resident mip per cell, 0..7, in patches like a real one
        m = [[7] * N for _ in range(N)]
        for _ in range(rnd.randint(1, 6)):
            cx, cy, r, v = rnd.randrange(N), rnd.randrange(N), rnd.randint(1, 8), rnd.randint(0, 6)
            for j in range(max(cy - r, 0), min(cy + r, N)):
                for i in range(max(cx - r, 0), min(cx + r, N)):
                    m[j][i] = min(m[j][i], v)
        d = dilate(m)
        h = 1.0 / 16.0
        prev_old = prev_new = None
        y = rnd.random() * N
        x = 0.0
        while x < N:
            old = gather_max(m, x, y)
            new = bilinear(d, x, y)
            if new < old - 1e-9:
                unsound += 1
            extra += new - old
            samples += 1
            if prev_old is not None:
                worst_step_old = max(worst_step_old, abs(old - prev_old))
                worst_step_new = max(worst_step_new, abs(new - prev_new))
            prev_old, prev_new = old, new
            x += h
    print("THE FLOOR (dilate + bilinear against gather + max)")
    print(f"  samples {samples}; finer than today's clamp (unsound): {unsound}")
    print(f"  largest step between samples 1/16 cell apart: today {worst_step_old:.3f} mips,"
          f" proposed {worst_step_new:.3f} mips")
    print(f"  mean extra coarseness paid: {extra / samples:.3f} mips")


def torus(trials=200000, seed=5):
    rnd = random.Random(seed)
    bad = 0
    for _ in range(trials):
        x0 = rnd.randrange(0, 2 ** 40)          # a global texel, anywhere on a face plane
        m = rnd.randrange(0, 8)
        a = (x0 >> m) % (16384 >> m)
        b = (x0 % 16384) >> m
        if a != b:
            bad += 1
    print("THE TORUS (global texel -> slot, every mip)")
    print(f"  {trials} random (texel, mip) pairs; disagreements: {bad}")


if __name__ == "__main__":
    floor_law()
    torus()
