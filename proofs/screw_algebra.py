# ==================================================================================================
#  proofs/screw_algebra.py - M9bq: the PGA screw convention, and the proof its gate is not vacuous.
#
#  ALGEBRA FIRST. src/core/Pga.h's Bivector carries BOTH a twist and a wrench in one six-double
#  type, on the claim that the motor sandwich B' = M B ~M transports both -- provided the
#  Euclidean part holds the FREE VECTOR (omega, or f) and the ideal part holds its MOMENT
#  (v_O, or tau_O). That claim is derived in the header from the translation case
#
#      a' = a,   b' = b + t x a
#
#  which both physical readings satisfy. This file is the independent check: a separate
#  dual-quaternion implementation, and -- the part that matters -- the SWAPPED convention run
#  through the same test, which must fail. RunPgaSelfTest's wrench-transport block would pass
#  for any self-consistent convention if the test did not discriminate; here it is shown to.
#
#  (The lesson is ripple_prefilter.py's: that file shipped a gate computing f(a) - f(a), which
#  passes for every f. Build the instrument before trusting the reading.)
# ==================================================================================================

import numpy as np

rng = np.random.default_rng(20260908)


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def qconj(a):
    return np.array([a[0], -a[1], -a[2], -a[3]])


def motor_rot_trans(axis, angle, t):
    """Rotation about the line through the origin with `axis`, then translation by t."""
    axis = axis / np.linalg.norm(axis)
    re = np.concatenate([[np.cos(angle/2)], np.sin(angle/2)*axis])
    # dual part of a pure translation is 0.5*t; compose T(t) * R
    tq = np.array([0.0, *(0.5*np.asarray(t))])
    du = qmul(tq, re)
    return re, du


def dq_mul(A, B):
    (ar, ad), (br, bd) = A, B
    return (qmul(ar, br), qmul(ar, bd) + qmul(ad, br))


def dq_rev(A):
    return (qconj(A[0]), qconj(A[1]))


def sandwich(M, a, b):
    """B' = M B ~M with B embedded as a dual quaternion with empty scalar slots."""
    B = (np.array([0.0, *a]), np.array([0.0, *b]))
    R = dq_mul(dq_mul(M, B), dq_rev(M))
    assert abs(R[0][0]) < 1e-9, "scalar slot polluted: %g" % R[0][0]
    assert abs(R[1][0]) < 1e-9, "pseudoscalar slot polluted: %g" % R[1][0]
    return R[0][1:], R[1][1:]


def xform_point(M, p):
    re, du = M
    v = qmul(qmul(re, np.array([0.0, *p])), qconj(re))[1:]
    return v + 2.0 * qmul(du, qconj(re))[1:]


def xform_dir(M, d):
    re = M[0]
    return qmul(qmul(re, np.array([0.0, *d])), qconj(re))[1:]


def run(swapped):
    """Transport the wrench of a force at a point; compare against transporting force+point."""
    worst = 0.0
    for _ in range(500):
        f = rng.normal(size=3) * 10
        p = rng.normal(size=3) * 5
        M = motor_rot_trans(rng.normal(size=3), rng.uniform(-3, 3), rng.normal(size=3) * 6)

        tau = np.cross(p, f)
        a, b = (tau, f) if swapped else (f, tau)          # <-- the convention under test
        ma, mb = sandwich(M, a, b)

        f2 = xform_dir(M, f)
        p2 = xform_point(M, p)
        tau2 = np.cross(p2, f2)
        ea, eb = (tau2, f2) if swapped else (f2, tau2)

        worst = max(worst, np.max(np.abs(ma - ea)), np.max(np.abs(mb - eb)))
    return worst


def run_power():
    """omega.tau + v.f must be invariant under simultaneous transport."""
    worst = 0.0
    for _ in range(500):
        om, vo = rng.normal(size=3), rng.normal(size=3) * 3
        f, p = rng.normal(size=3) * 8, rng.normal(size=3) * 4
        tau = np.cross(p, f)
        M = motor_rot_trans(rng.normal(size=3), rng.uniform(-3, 3), rng.normal(size=3) * 6)
        before = om @ tau + vo @ f
        ta, tb = sandwich(M, om, vo)
        wa, wb = sandwich(M, f, tau)
        worst = max(worst, abs((ta @ wb + tb @ wa) - before))
    return worst


ok = run(swapped=False)
bad = run(swapped=True)
pw = run_power()

print("wrench transport, a=f  b=tau  (the shipped convention) : max err %.3e" % ok)
print("wrench transport, a=tau b=f   (swapped -- must FAIL)   : max err %.3e" % bad)
print("power invariance omega.tau + v.f                       : max err %.3e" % pw)
print()
assert ok < 1e-9, "the shipped convention does NOT transport"
assert bad > 1.0, "GATE IS VACUOUS: the swapped convention passes too"
assert pw < 1e-9, "power is not invariant"
print("ALL ASSERTS PASSED -- and the gate discriminates (swapped form is off by %.1f)" % bad)
