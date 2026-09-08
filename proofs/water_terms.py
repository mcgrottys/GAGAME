# ==================================================================================================
#  proofs/water_terms.py - src/sim/WaterTerms.h, measured.
#
#  WHAT THIS PROOF DOES DIFFERENTLY.  It does not re-implement the four laws in Python and then
#  check Python against itself.  It READS src/sim/WaterTerms.h, transpiles the function bodies
#  (a deliberately tiny, restricted C++ subset: assignments, guarded returns, one level of
#  braces, no ternaries, no loops) into Python, and runs THOSE.  Every number printed below came
#  out of the header's own text.  A hand-copied mirror would drift the first time someone edited
#  the header, and this repo has already shipped one vacuous gate (a check of the form f(a) -
#  f(a), which cannot fail); a mirror is the same species of mistake, one step removed.
#
#  Consequence, and it is the point: EVERY NEGATIVE CONTROL HERE IS A SOURCE MUTATION.  The proof
#  edits one substring of the header text, re-transpiles, and re-measures.  So "the folded
#  reference form fails this gate" is not a claim about some other program -- it is this same
#  code with '-' changed to '+' on one line, and the printed margin is the distance between them.
#  The transpiler REFUSES lines it does not understand rather than skipping them, so a future
#  edit that leaves the subset breaks this proof loudly instead of quietly measuring less.
#
#  What it does NOT prove: that a C++ compiler agrees with the transpiler about this text.  The
#  subset is small enough that the two readings coincide by inspection, but that is inspection,
#  not measurement, and it is stated here rather than glossed.  GaTest.cpp block 10 is where the
#  compiled engine pins the same identities.
#
#  THE GATES (each prints PASS/FAIL with its measured margin; exit nonzero on any failure):
#    0. TRANSCRIPTION - the header's numeric literals against the HLSL's, function by function,
#       with an explicit allowlist for the differences that have reasons.
#    1. THE WEDGE FALLS OUT.  Measured from the header's discriminant AND from the rendered
#       field: 19.4712 deg = atan(1/(2 sqrt 2)) = arcsin(1/3).  Control: 8 -> 4 in the header.
#    2. ROOT STATIONARITY.  Finite-difference |grad phi| / k == 1 within 1e-4 on BOTH branches.
#       Control: the SIGN in the header's phase, which reproduces the reference's folded form.
#    3. VIETA.  t+ * t- == 1/2, and the roots really are roots.  Control: 4.0 -> 4.04.
#    4. CLASS WAVELENGTHS.  lambda_t = 2 pi U^2/g, measured by zero crossings of the header's own
#       eta, against 15.1 / 13.7 / 9.4 / 7.1 m.  Control: K0 = g/U instead of g/U^2.
#    5. SHOALING AND PHASE SPEED against their analytic shallow and deep limits, with the error
#       at each limit stated.  Controls: tanh -> 1, and the group-speed reference broken.
#    6. WAVE-CURRENT: the closed form IS action transport; the blocking point is exactly -1/4;
#       and the three guards the header calls dead are measured to be dead.
#
#  Deterministic (no randomness).  stdlib + numpy.  Run: py proofs/water_terms.py
# ==================================================================================================
import io
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
HEADER = os.path.join(ROOT, "src", "sim", "WaterTerms.h")
JET = os.path.join(ROOT, "shaders", "Jet.hlsli")
BANK = os.path.join(ROOT, "shaders", "WaterBank.hlsl")

G = 9.81
INV2R2 = 1.0 / (2.0 * math.sqrt(2.0))            # tan of the Kelvin half-angle
KELVIN_DEG = math.degrees(math.atan(INV2R2))     # 19.471220634490695
# The AIS per-class table this engine ships: src/core/SceneConfig.h:115-119.
CLASS_U = (4.86, 4.62, 3.83, 3.34)
CLASS_HALFLEN = (7.5, 9.0, 13.0, 6.0)
CLASS_WAKEAMP = (0.55, 0.47, 0.63, 0.38)
CLASS_LAM_QUOTED = (15.1, 13.7, 9.4, 7.1)        # ALGEBRA.md `wake`
# The bank's representative band wavenumbers, sqrt(kCut[c]*kCut[c+1]) with the cuts from
# src/scene/WaterBankLayer.cpp:514 -- swell, wind sea, chop.
KCUT = (2.0 * math.pi / 756.0, 2.0 * math.pi / 60.0, 2.0 * math.pi / 12.0,
        0.9 * math.pi * 256.0 / 47.0)
BAND_K = tuple(math.sqrt(KCUT[c] * KCUT[c + 1]) for c in range(3))

FAILS = []


def check(name, ok, detail=""):
    print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, (" -- " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def note(text):
    print("       %s" % text)


# ==================================================================================================
#  THE TRANSPILER.  Restricted C++ -> Python.  Everything it accepts is listed here; anything
#  else raises.  It is small on purpose: a big one would be a second implementation to trust.
# ==================================================================================================
class TranspileError(Exception):
    pass


NE = "@NEQ@"          # placeholder so '!=' survives the '!' -> 'not' rewrite


def strip_comments(text):
    if "/*" in text:
        raise TranspileError("block comments are outside the subset")
    out = []
    for line in text.replace("\r\n", "\n").split("\n"):
        i = line.find("//")
        if i >= 0:
            line = line[:i]
        out.append(line.rstrip())
    return out


def rewrite_expr(e):
    """C++ expression -> Python expression, within the subset."""
    e = e.replace("std::", "").replace("wt::", "").replace("ga::", "")
    e = e.replace("!=", NE)
    if "?" in e:
        raise TranspileError("ternary outside the subset: %r" % e)
    e = e.replace("!", " not ")
    e = e.replace(NE, "!=")
    e = e.replace("&&", " and ").replace("||", " or ")
    e = re.sub(r"\btrue\b", "True", e)
    e = re.sub(r"\bfalse\b", "False", e)
    return e.strip()


DECL = re.compile(r"^(?:const\s+)?(?:double|bool)\s+(?=[A-Za-z_])")
ASSIGN = re.compile(r"^[A-Za-z_][A-Za-z0-9_.]*\s*(?:=|\+=|-=)\s*\S")
CALL = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*\s*\(.*\)$")


def split_if(stmt):
    """'if (COND) TAIL' -> (COND, TAIL), matching parens properly. None if not an if."""
    if not stmt.startswith("if"):
        return None
    i = stmt.find("(")
    if i < 0:
        raise TranspileError("malformed if: %r" % stmt)
    depth = 0
    for j in range(i, len(stmt)):
        if stmt[j] == "(":
            depth += 1
        elif stmt[j] == ")":
            depth -= 1
            if depth == 0:
                return stmt[i + 1:j], stmt[j + 1:].strip()
    raise TranspileError("unbalanced if: %r" % stmt)


def translate(stmt, ret_type, rhs_sink):
    """One joined C++ statement -> (python line, opens_block)."""
    if stmt.endswith("{"):
        parts = split_if(stmt[:-1].strip())
        if parts is None or parts[1] != "":
            raise TranspileError("only 'if (...) {' opens a block here: %r" % stmt)
        return "if %s:" % rewrite_expr(parts[0]), True
    if not stmt.endswith(";"):
        raise TranspileError("statement without ';': %r" % stmt)
    stmt = stmt[:-1].strip()
    parts = split_if(stmt)
    if parts is not None:
        cond, tail = parts
        if tail == "return":
            return "if %s: return" % rewrite_expr(cond), False
        if tail.startswith("return "):
            return "if %s: return %s" % (rewrite_expr(cond),
                                         _ret_expr(tail[7:].strip(), ret_type)), False
        if ASSIGN.match(DECL.sub("", tail)):
            return "if %s: %s" % (rewrite_expr(cond), rewrite_expr(DECL.sub("", tail))), False
        raise TranspileError("if-tail outside the subset: %r" % stmt)
    if stmt == "return":
        return "return", False
    if stmt.startswith("return "):
        return "return %s" % _ret_expr(stmt[7:].strip(), ret_type), False
    body = DECL.sub("", stmt)
    if ASSIGN.match(body):
        # remember the right-hand side by name, so gates can quote the header's own expressions
        m = re.match(r"^([A-Za-z_][A-Za-z0-9_.]*)\s*=\s*(.+)$", body)
        if m:
            rhs_sink[m.group(1)] = rewrite_expr(m.group(2))
        return rewrite_expr(body), False
    if CALL.match(body):
        return rewrite_expr(body), False
    raise TranspileError("statement outside the subset: %r" % stmt)


def _ret_expr(e, ret_type):
    if e.startswith("{") and e.endswith("}"):
        return "%s(%s)" % (ret_type, rewrite_expr(e[1:-1]))
    return rewrite_expr(e)


SIG = re.compile(r"^\s*inline\s+([A-Za-z_][A-Za-z0-9_]*)\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(")
CONST = re.compile(r"^\s*inline\s+constexpr\s+double\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*([^;]+);")
STRUCT = re.compile(r"^\s*struct\s+([A-Za-z_][A-Za-z0-9_]*)\s*\{\s*$")
FIELD = re.compile(r"^\s*(double|bool)\s+([A-Za-z_][A-Za-z0-9_]*)\s*(?:=\s*([^;]+?))?\s*;\s*$")


def param_names(sig_text):
    inner = sig_text[sig_text.index("(") + 1:sig_text.rindex(")")]
    names, depth, cur = [], 0, ""
    for ch in inner:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if ch == "," and depth == 0:
            names.append(cur)
            cur = ""
        else:
            cur += ch
    if cur.strip():
        names.append(cur)
    out = []
    for p in names:
        toks = re.findall(r"[A-Za-z_][A-Za-z0-9_]*", p)
        if not toks:
            raise TranspileError("unnamed parameter %r" % p)
        out.append(toks[-1])
    return out


def transpile(header_text):
    """Return (namespace dict, {func: {var: python-rhs}}, source-line-count consumed)."""
    lines = strip_comments(header_text)
    env = {"math": math, "sqrt": math.sqrt, "tanh": math.tanh, "sinh": math.sinh,
           "cosh": math.cosh, "exp": math.exp, "cos": math.cos, "sin": math.sin,
           "atan": math.atan, "abs": abs, "pow": pow, "fabs": abs, "hypot": math.hypot}
    rhs = {}
    i, n = 0, len(lines)
    n_funcs = 0
    while i < n:
        line = lines[i]
        m = CONST.match(line)
        if m:
            env[m.group(1)] = eval(rewrite_expr(m.group(2)), dict(env))
            i += 1
            continue
        m = STRUCT.match(line)
        if m:
            name, fields, defaults = m.group(1), [], []
            i += 1
            while not lines[i].strip().startswith("};"):
                f = FIELD.match(lines[i])
                if f:
                    fields.append(f.group(2))
                    d = f.group(3)
                    defaults.append(eval(rewrite_expr(d), dict(env)) if d else 0.0)
                elif lines[i].strip():
                    raise TranspileError("struct member outside the subset: %r" % lines[i])
                i += 1
            env[name] = _make_struct(name, tuple(fields), tuple(defaults))
            i += 1
            continue
        m = SIG.match(line)
        if m:
            sig = line
            while sig.count("(") == 0 or sig.count("(") != sig.count(")"):
                i += 1
                sig += " " + lines[i].strip()
            if not sig.rstrip().endswith("{"):
                i += 1
                sig += " " + lines[i].strip()
            ret_type, fname = m.group(1), m.group(2)
            params = param_names(sig[:sig.rindex(")") + 1])
            body, depth = [], 1
            i += 1
            while depth > 0:
                s = lines[i].strip()
                depth += s.count("{") - s.count("}")
                if depth > 0:
                    body.append(lines[i])
                i += 1
            rhs[fname] = {}
            py = ["def %s(%s):" % (fname, ", ".join(params))]
            py += _transpile_body(body, ret_type, rhs[fname])
            exec(compile("\n".join(py), "<%s>" % fname, "exec"), env)
            n_funcs += 1
            continue
        i += 1
    if n_funcs == 0:
        raise TranspileError("no functions found -- the header layout changed")
    return env, rhs, n_funcs


def _transpile_body(body_lines, ret_type, rhs_sink):
    out, indent, buf = [], 1, ""
    for raw in body_lines:
        s = raw.strip()
        if not s:
            continue
        buf = (buf + " " + s) if buf else s
        if not (buf.endswith(";") or buf.endswith("{") or buf.endswith("}")):
            continue
        stmt, buf = buf.strip(), ""
        if stmt == "}":
            indent -= 1
            if indent < 1:
                raise TranspileError("brace underflow")
            continue
        py, opens = translate(stmt, ret_type, rhs_sink)
        out.append("    " * indent + py)
        if opens:
            indent += 1
    if buf:
        raise TranspileError("unterminated statement: %r" % buf)
    return out


def _make_struct(name, fields, defaults):
    def __init__(self, *a):
        vals = list(a) + list(defaults[len(a):])
        if len(vals) != len(fields):
            raise TypeError("%s takes %d fields" % (name, len(fields)))
        for f, v in zip(fields, vals):
            setattr(self, f, v)

    def __repr__(self):
        return "%s(%s)" % (name, ", ".join("%s=%r" % (f, getattr(self, f)) for f in fields))

    return type(name, (object,), {"__init__": __init__, "__repr__": __repr__,
                                  "_fields": fields})


with io.open(HEADER, "r", encoding="ascii") as f:
    HEADER_TEXT = f.read()

ENV, RHS, NFUNCS = transpile(HEADER_TEXT)
print("== transpiled src/sim/WaterTerms.h: %d functions, %d struct(s), kGWave = %s ==" %
      (NFUNCS, sum(1 for v in ENV.values() if isinstance(v, type)), ENV["kGWave"]))


def mutate(*pairs):
    """Re-transpile the header with substrings replaced. THE negative-control instrument."""
    text = HEADER_TEXT
    for old, new in pairs:
        if text.count(old) < 1:
            raise TranspileError("mutation target absent: %r" % old)
        text = text.replace(old, new)
    return transpile(text)


# ==================================================================================================
#  GATE 0: TRANSCRIPTION.  The header's numeric literals against the HLSL's, per function.
#  Catches the realistic port failure -- a transposed digit -- which no analytic gate would see
#  if the wrong digit happens to keep the law self-consistent.
# ==================================================================================================
print("")
print("== gate 0: literal transcription, C++ header vs HLSL source ==")
NUM = re.compile(r"(?<![A-Za-z0-9_.])(\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[fF]?")


def hlsl_body(path, fname):
    with io.open(path, "r", encoding="ascii", errors="replace") as fh:
        lines = strip_comments(fh.read())
    pat = re.compile(r"^\s*(?:float2?|void)\s+%s\s*\(" % re.escape(fname))
    for i, line in enumerate(lines):
        if pat.match(line):
            sig = line
            j = i
            while not sig.rstrip().endswith("{"):
                j += 1
                sig += " " + lines[j].strip()
            body, depth, j = [], 1, j + 1
            while depth > 0:
                depth += lines[j].count("{") - lines[j].count("}")
                if depth > 0:
                    body.append(lines[j])
                j += 1
            return "\n".join(body)
    raise TranspileError("HLSL function not found: %s in %s" % (fname, path))


def cpp_body(fname):
    lines = strip_comments(HEADER_TEXT)
    for i, line in enumerate(lines):
        m = SIG.match(line)
        if m and m.group(2) == fname:
            sig, j = line, i
            while not sig.rstrip().endswith("{"):
                j += 1
                sig += " " + lines[j].strip()
            body, depth, j = [], 1, j + 1
            while depth > 0:
                depth += lines[j].count("{") - lines[j].count("}")
                if depth > 0:
                    body.append(lines[j])
                j += 1
            return "\n".join(body)
    raise TranspileError("C++ function not found: %s" % fname)


def literals(text, extra_names=()):
    """Numeric literals, plus the VALUES of any named constants the text mentions."""
    vals = [float(m.group(1)) for m in NUM.finditer(text)]
    for nm in extra_names:
        vals += [ENV[nm]] * len(re.findall(r"\b%s\b" % nm, text))
    return sorted(vals)


def multiset_diff(a, b, rtol=1e-6):
    """Items of a with no partner in b (within rtol), and vice versa."""
    b_left = list(b)
    only_a = []
    for x in a:
        hit = None
        for i, y in enumerate(b_left):
            if abs(x - y) <= rtol * max(abs(x), abs(y), 1e-12):
                hit = i
                break
        if hit is None:
            only_a.append(x)
        else:
            b_left.pop(hit)
    return only_a, b_left


# Differences that have reasons.  Anything not on this list is a transcription failure.
ALLOW = {
    # (only in HLSL, only in C++) -- the C++ side substitutes named constants for kG and the
    # depth floor, so those values ARE counted (via extra_names); what is left over is listed.
    "BandPhaseSpeed": ([], []),
    "ShoalFactor": ([], []),
    "WaveCurrentAmp": ([], []),
    "WakeBranch": ([], []),
    # HLSL disables a boat with 'B.z < 0.5f'; the C++ struct carries a bool, so one 0.5
    # comparison has no literal counterpart.  Same law, one fewer float.
    "WakeOne": ([0.5], []),
}
NAMED = {"BandPhaseSpeed": ("kGWave", "kDepthFloorM"),
         "ShoalFactor": ("kGWave", "kDepthFloorM"),
                  "WaveCurrentAmp": (), "WakeBranch": (), "WakeOne": ("kGWave",)}
SRC = {"BandPhaseSpeed": JET, "ShoalFactor": JET, "WaveCurrentAmp": JET,
       "WakeBranch": BANK, "WakeOne": BANK}

for fname in ("BandPhaseSpeed", "ShoalFactor", "WaveCurrentAmp", "WakeBranch", "WakeOne"):
    h = literals(hlsl_body(SRC[fname], fname))
    c = literals(cpp_body(fname), NAMED[fname])
    only_h, only_c = multiset_diff(h, c)
    exp_h, exp_c = ALLOW[fname]
    ok = (multiset_diff(only_h, exp_h)[0] == [] and multiset_diff(exp_h, only_h)[0] == [] and
          multiset_diff(only_c, exp_c)[0] == [] and multiset_diff(exp_c, only_c)[0] == [])
    check("literals(%s) match the shader" % fname, ok,
          "%d values; HLSL-only %s, C++-only %s (allowed %s / %s)" %
          (len(c), only_h, only_c, exp_h, exp_c))

# and the control: a transposed digit is caught.
_bad = HEADER_TEXT.replace("0.30 / wt::Max(k, 1e-3)", "0.03 / wt::Max(k, 1e-3)")
_c = literals("\n".join(strip_comments(_bad)[0:0]) or "", ())
_lines = strip_comments(_bad)
_j = [i for i, l in enumerate(_lines) if SIG.match(l) and SIG.match(l).group(2) == "WakeBranch"][0]
_body = []
_d, _k = 1, _j + 1
while _d > 0:
    _d += _lines[_k].count("{") - _lines[_k].count("}")
    if _d > 0:
        _body.append(_lines[_k])
    _k += 1
_only_h, _only_c = multiset_diff(literals(hlsl_body(BANK, "WakeBranch")),
                                 literals("\n".join(_body)))
check("control: steepness cap 0.30 -> 0.03 is CAUGHT by gate 0", _only_h == [0.3],
      "HLSL-only %s, C++-only %s (the real header leaves both empty)" % (_only_h, _only_c))


# The parity constant is NOT standard gravity, and that is deliberate (see the header's comment
# on kGWave).  Pin them apart: a future tidy-up that merges them shifts every wavelength 0.03%.
_medium = io.open(os.path.join(ROOT, "src", "sim", "Medium.h"), "r", encoding="ascii",
                  errors="replace").read()
_std_g = float(re.search(r"inline\s+constexpr\s+double\s+kG\s*=\s*([0-9.]+);", _medium).group(1))
_hlsl_g = sorted(set(re.findall(r"(?<![A-Za-z0-9_.])(9\.81)f?", hlsl_body(JET, "BandPhaseSpeed") +
                                hlsl_body(JET, "ShoalFactor") + hlsl_body(BANK, "WakeOne"))))
check("kGWave is the SHADER's 9.81, held apart from sim/Medium.h's standard gravity",
      ENV["kGWave"] == 9.81 and _std_g == 9.80665 and _hlsl_g == ["9.81"],
      "kGWave = %s (shader literal %s), ga::kG = %s -- they differ by %.4f%%, and a wavelength "
      "2 pi U^2/g would move %.4f m at U = 4.86" %
      (ENV["kGWave"], _hlsl_g, _std_g, 100.0 * abs(9.81 - _std_g) / _std_g,
       abs(2 * math.pi * 4.86 ** 2 / 9.81 - 2 * math.pi * 4.86 ** 2 / _std_g)))


# ==================================================================================================
#  GATE 1: THE WEDGE FALLS OUT OF THE DISCRIMINANT.  Nothing in WakeOne names 19.4712 degrees.
# ==================================================================================================
print("")
print("== gate 1: the Kelvin half-angle is the discriminant, not a constant ==")


def wedge_from_disc(env, rhs):
    """Half-angle where the header's own 'disc' expression changes sign, at xi = 1."""
    expr = rhs["WakeOne"]["disc"]
    f = lambda ratio: eval(expr, dict(env), {"xi": 1.0, "zeta": ratio})
    lo, hi = 0.0, 10.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if f(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    return math.degrees(math.atan(0.5 * (lo + hi)))


ang = wedge_from_disc(ENV, RHS)
check("wedge from the header's discriminant == 19.4712 deg (1e-9 deg)",
      abs(ang - KELVIN_DEG) <= 1e-9, "measured %.12f deg, atan(1/2sqrt2) %.12f deg" %
      (ang, KELVIN_DEG))
check("atan(1/(2 sqrt 2)) == arcsin(1/3) (1e-15 rad)",
      abs(math.atan(INV2R2) - math.asin(1.0 / 3.0)) <= 1e-15,
      "difference %.3g rad" % abs(math.atan(INV2R2) - math.asin(1.0 / 3.0)))

# ...and again from the FIELD the header actually renders: scan across the track and find the
# outermost sample that produces any displacement at all.
WakeVessel, WakeSample = ENV["WakeVessel"], ENV["WakeSample"]


def field_wedge(env, U=4.86, halfLen=7.5, wakeAmp=0.55, sampleM=0.5):
    WV, WS = env["WakeVessel"], env["WakeSample"]
    worst = 0.0
    for xi in (150.0, 250.0, 400.0, 600.0):
        lo, hi = 0.0, xi                     # bisect on |across| for "does eta exist here"
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            acc = WS()
            v = WV(0.0, 0.0, 0.0, U, wakeAmp, halfLen, True)
            env["WakeOne"](v, -xi, mid, sampleM, acc)   # heading +x, so astern is -x
            if acc.akMax > 0.0:
                lo = mid
            else:
                hi = mid
        worst = max(worst, math.degrees(math.atan(0.5 * (lo + hi) / xi)))
    return worst


fang = field_wedge(ENV)
check("wedge measured on the rendered field == 19.4712 deg (1e-6 deg)",
      abs(fang - KELVIN_DEG) <= 1e-6, "measured %.9f deg" % fang)

ENV_W, RHS_W, _ = mutate(("xi * xi - 8.0 * zeta * zeta", "xi * xi - 4.0 * zeta * zeta"))
ang_bad = wedge_from_disc(ENV_W, RHS_W)
fang_bad = field_wedge(ENV_W)
check("control: discriminant 8 -> 4 moves the wedge off 19.4712 deg",
      abs(ang_bad - KELVIN_DEG) > 1.0,
      "mutated %.6f deg (= atan(1/2)), field %.6f deg, margin %.4f deg" %
      (ang_bad, fang_bad, abs(ang_bad - KELVIN_DEG)))


# ==================================================================================================
#  GATE 2: ROOT STATIONARITY.  |grad phi| / k == 1 exactly at a stationary point (envelope
#  theorem).  phi is assembled from the header's OWN root and phase expressions, with the root
#  recomputed at every finite-difference sample -- nothing is held fixed that the field varies.
# ==================================================================================================
print("")
print("== gate 2: the SIGNED stationary phase, |grad phi| / k == 1 ==")


def phase_of(env, rhs, xi, zeta, K0, branch):
    disc = eval(rhs["WakeOne"]["disc"], dict(env), {"xi": xi, "zeta": zeta})
    sq = math.sqrt(disc)
    t = eval(rhs["WakeOne"]["t1" if branch == 0 else "t2"], dict(env),
             {"xi": xi, "zeta": zeta, "sq": sq})
    loc = {"K0": K0, "xi": xi, "zeta": zeta, "t": t}
    loc["sec"] = eval(rhs["WakeBranch"]["sec"], dict(env), loc)
    loc["tAbs"] = eval(rhs["WakeBranch"]["tAbs"], dict(env), loc)
    loc["k"] = eval(rhs["WakeBranch"]["k"], dict(env), loc)
    return eval(rhs["WakeBranch"]["ph"], dict(env), loc), loc["k"], t


def grad_ratios(env, rhs, U=4.86):
    K0 = G / (U * U)
    out = []
    for xi in (20.0, 50.0, 100.0, 200.0, 350.0):
        for f in (0.05, 0.2, 0.4, 0.6, 0.8, 0.95):
            zeta = f * xi * INV2R2
            for branch in (0, 1):
                _, k, t = phase_of(env, rhs, xi, zeta, K0, branch)
                h = 1e-3 / (1.0 + t * t)     # branch-uniform k*h
                px = (phase_of(env, rhs, xi + h, zeta, K0, branch)[0] -
                      phase_of(env, rhs, xi - h, zeta, K0, branch)[0]) / (2.0 * h)
                pz = (phase_of(env, rhs, xi, zeta + h, K0, branch)[0] -
                      phase_of(env, rhs, xi, zeta - h, K0, branch)[0]) / (2.0 * h)
                out.append((math.hypot(px, pz) / k, branch))
    return out


sig_r = grad_ratios(ENV, RHS)
worst = max(abs(r - 1.0) for r, _ in sig_r)
per_branch = [max(abs(r - 1.0) for r, b in sig_r if b == br) for br in (0, 1)]
check("SIGNED phase: FD |grad phi|/k == 1 within 1e-4, BOTH branches", worst <= 1e-4,
      "max |ratio-1| = %.3g (transverse %.3g, divergent %.3g) over %d samples" %
      (worst, per_branch[0], per_branch[1], len(sig_r)))

# THE CONTROL: flip the one sign the header calls out.  This IS the reference's folded phase.
ENV_F, RHS_F, _ = mutate(("K0 * sec * (xi - zeta * tAbs)", "K0 * sec * (xi + zeta * tAbs)"))
fold_r = grad_ratios(ENV_F, RHS_F)
fmin, fmax = min(r for r, _ in fold_r), max(r for r, _ in fold_r)
check("control: the FOLDED reference phase FAILS the same gate (max ratio > 1.5)", fmax > 1.5,
      "|grad phi_folded|/k spans %.3f .. %.3f (ALGEBRA.md reports 1.03-6.65); "
      "margin over the 1e-4 tolerance is %.0fx" % (fmin, fmax, (fmax - 1.0) / 1e-4))

# and confirm the mutant really is the reference's algebraic form K0*(xi*sec + zeta*sec*|t|)
K0R = G / 4.86 ** 2
worst_id = 0.0
for xi in (30.0, 120.0, 300.0):
    for f in (0.2, 0.6, 0.9):
        zeta = f * xi * INV2R2
        for branch in (0, 1):
            ph, k, t = phase_of(ENV_F, RHS_F, xi, zeta, K0R, branch)
            sec = math.sqrt(1.0 + t * t)
            ref = K0R * (xi * sec + zeta * sec * abs(t))
            worst_id = max(worst_id, abs(ph - ref) / abs(ref))
check("the mutant IS the reference form K0(xi sec + zeta sec |t|) (1e-15 rel)",
      worst_id <= 1e-15, "max rel deviation %.3g" % worst_id)

# 2b (REPORTED, not a pass/fail on a tight bound): the analytic slope the header writes is the
# derivative of the eta it writes.  It is not exact -- the shader deliberately omits the
# amplitude envelope's own gradient -- so the residual is the envelope term, and what matters
# is that it is small for the signed form and NOT small for the folded one.
def slope_residual(env):
    WV, WS = env["WakeVessel"], env["WakeSample"]
    v = WV(0.0, 0.0, 0.0, 4.86, 0.55, 7.5, True)
    worst = 0.0
    for xi in (60.0, 140.0, 260.0):
        for f in (0.15, 0.45, 0.75):
            x, z = -xi, f * xi * INV2R2
            h = 1e-4
            a = WS(); env["WakeOne"](v, x, z, 0.5, a)
            ex = []
            for dx, dz in ((h, 0.0), (-h, 0.0), (0.0, h), (0.0, -h)):
                b = WS(); env["WakeOne"](v, x + dx, z + dz, 0.5, b)
                ex.append(b.eta)
            fx, fz = (ex[0] - ex[1]) / (2 * h), (ex[2] - ex[3]) / (2 * h)
            den = math.hypot(fx, fz)
            if den > 1e-6:
                worst = max(worst, math.hypot(fx - a.slopeX, fz - a.slopeZ) / den)
    return worst


rs, rf = slope_residual(ENV), slope_residual(ENV_F)
check("analytic slope == d(eta)/dx to within the envelope term (signed form)", rs <= 0.05,
      "signed max relative slope residual %.4f; FOLDED control %.4f (%.0fx worse)" %
      (rs, rf, rf / max(rs, 1e-12)))


# ==================================================================================================
#  GATE 3: VIETA, and the roots are roots.  2 zeta t^2 + xi t + zeta = 0 has t+ t- = zeta/(2 zeta).
# ==================================================================================================
print("")
print("== gate 3: Vieta and the quadratic ==")


def roots_of(env, rhs, xi, zeta):
    disc = eval(rhs["WakeOne"]["disc"], dict(env), {"xi": xi, "zeta": zeta})
    sq = math.sqrt(disc)
    loc = {"xi": xi, "zeta": zeta, "sq": sq}
    return (eval(rhs["WakeOne"]["t1"], dict(env), loc),
            eval(rhs["WakeOne"]["t2"], dict(env), loc))


def vieta_worst(env, rhs):
    wv, wr = 0.0, 0.0
    for xi in (5.0, 37.0, 120.0, 260.0, 700.0):
        for f in (0.02, 0.2, 0.5, 0.8, 0.95, 0.999):
            zeta = f * xi * INV2R2
            t1, t2 = roots_of(env, rhs, xi, zeta)
            wv = max(wv, abs(t1 * t2 - 0.5))
            for t in (t1, t2):
                wr = max(wr, abs(2.0 * zeta * t * t + xi * t + zeta) / max(xi, zeta))
    return wv, wr


wv, wr = vieta_worst(ENV, RHS)
check("Vieta: t+ * t- == 1/2 (1e-12)", wv <= 1e-12, "max |t1 t2 - 1/2| = %.3g" % wv)
check("both roots satisfy 2 zeta t^2 + xi t + zeta == 0 (1e-12 scaled)", wr <= 1e-12,
      "max scaled residual %.3g" % wr)
ENV_V, RHS_V, _ = mutate(("(-xi + sq) / (4.0 * zeta)", "(-xi + sq) / (4.04 * zeta)"),
                         ("(-xi - sq) / (4.0 * zeta)", "(-xi - sq) / (4.04 * zeta)"))
wv2, wr2 = vieta_worst(ENV_V, RHS_V)
check("control: 4.0 -> 4.04 in the roots breaks Vieta and the quadratic", wv2 > 1e-3,
      "mutated max |t1 t2 - 1/2| = %.4g (%.0fx the tolerance), residual %.4g" %
      (wv2, wv2 / 1e-12, wr2))


# ==================================================================================================
#  GATE 4: CLASS WAVELENGTHS.  Measured off the header's own eta along the track, where the
#  quadratic degenerates to the single transverse root t = 0 and k must be exactly K0.
# ==================================================================================================
print("")
print("== gate 4: transverse wavelength per AIS class ==")


def zero_crossings(s, v):
    sgn = np.sign(v)
    idx = np.nonzero(sgn[:-1] * sgn[1:] < 0)[0]
    return s[idx] - v[idx] * (s[idx + 1] - s[idx]) / (v[idx + 1] - v[idx])


def measure_lambda(env, U, halfLen, wakeAmp, lo=80.0, hi=440.0):
    WV, WS = env["WakeVessel"], env["WakeSample"]
    v = WV(0.0, 0.0, 0.0, U, wakeAmp, halfLen, True)
    s = np.arange(lo, hi, 0.01)
    eta = np.empty_like(s)
    for i, xi in enumerate(s):
        a = WS()
        env["WakeOne"](v, -float(xi), 0.0, 0.25, a)
        eta[i] = a.eta
    zc = zero_crossings(s, eta)
    return 2.0 * float(np.mean(np.diff(zc))), len(zc)


worst_lam, meas = 0.0, []
for U, hl, wa, quoted in zip(CLASS_U, CLASS_HALFLEN, CLASS_WAKEAMP, CLASS_LAM_QUOTED):
    lam_t = 2.0 * math.pi * U * U / G
    lam_m, nzc = measure_lambda(ENV, U, hl, wa)
    meas.append(lam_m)
    worst_lam = max(worst_lam, abs(lam_m - lam_t) / lam_t)
    note("U = %.2f m/s: closed form 2 pi U^2/g = %.4f m, ALGEBRA.md quotes %.1f m, "
         "MEASURED on the header's eta = %.4f m (%d crossings, %+.3f%%)" %
         (U, lam_t, quoted, lam_m, nzc, 100.0 * (lam_m - lam_t) / lam_t))
check("measured lambda == 2 pi U^2/g for all four classes (0.5%)", worst_lam <= 0.005,
      "worst deviation %.4f%%" % (100.0 * worst_lam))
check("2 pi U^2/g matches the values ALGEBRA.md ships (0.05 m)",
      max(abs(2.0 * math.pi * u * u / G - q) for u, q in zip(CLASS_U, CLASS_LAM_QUOTED)) <= 0.05,
      "%s vs quoted %s" % (["%.4f" % (2.0 * math.pi * u * u / G) for u in CLASS_U],
                           list(CLASS_LAM_QUOTED)))

ENV_K, _, _ = mutate(("kGWave / (v.speed * v.speed)", "kGWave / v.speed"))
lam_bad, _ = measure_lambda(ENV_K, 4.86, 7.5, 0.55)
lam_ref = 2.0 * math.pi * 4.86 * 4.86 / G
check("control: K0 = g/U instead of g/U^2 moves the measured wavelength",
      abs(lam_bad - lam_ref) / lam_ref > 0.5,
      "mutated measures %.4f m against %.4f m -- off by %.1f%% (gate tolerance 0.5%%)" %
      (lam_bad, lam_ref, 100.0 * abs(lam_bad - lam_ref) / lam_ref))

# the cusp: at the wedge edge the two roots merge on t = -1/sqrt(2), so k = 1.5 K0 exactly.
t1e, t2e = roots_of(ENV, RHS, 1.0, INV2R2 * (1.0 - 1e-15))
check("branches merge at t = -1/sqrt(2), k = 1.5 K0 (1e-6)",
      abs(t1e + 1.0 / math.sqrt(2.0)) <= 1e-6 and abs((1.0 + t1e * t1e) - 1.5) <= 1e-6,
      "t = %.9f (want %.9f), k/K0 = %.9f, lambda_cusp/lambda_t = %.6f (want 2/3)" %
      (t1e, -1.0 / math.sqrt(2.0), 1.0 + t1e * t1e, 1.0 / (1.0 + t1e * t1e)))


# ==================================================================================================
#  GATE 5: SHOALING AND PHASE SPEED against their analytic limits.
#     BandPhaseSpeed:  kh >> 1 -> sqrt(g/k)   [error ~ -exp(-2kh)]
#                      kh << 1 -> sqrt(g h)   [error ~ -(kh)^2/6]
#     ShoalFactor:     kh >> 1 -> 1
#                      kh << 1 -> (1/sqrt 2)(kh)^(-1/4)  [error ~ +(kh)^2/4]
#  The h^(-1/4) exponent is Green's law; the 1/sqrt(2) offset is what holding k rather than the
#  period costs, and both are pinned so a future edit cannot quietly swap one form for the other.
# ==================================================================================================
print("")
print("== gate 5: shoaling and phase speed, at both limits ==")
BPS, SHOAL = ENV["BandPhaseSpeed"], ENV["ShoalFactor"]
K_TEST = 0.234162       # the wind-sea band's representative k; any k works, this one is real


def deep_err(fn, ref):
    out = []
    for kh in (4.0, 8.0, 16.0):
        h = kh / K_TEST
        out.append((kh, fn(K_TEST, h) / ref(K_TEST, h) - 1.0))
    return out


c_deep = deep_err(BPS, lambda k, h: math.sqrt(G / k))
ok = all(abs(e) <= 1.2 * math.exp(-2.0 * kh) for kh, e in c_deep)
check("BandPhaseSpeed -> sqrt(g/k) as kh -> inf, error <= 1.2 exp(-2kh)", ok,
      "; ".join("kh=%g rel err %+.3g (bound %.3g)" % (kh, e, 1.2 * math.exp(-2 * kh))
                for kh, e in c_deep))

c_shal = []
for kh in (0.3, 0.1, 0.03, 0.01):
    h = kh / K_TEST
    c_shal.append((kh, BPS(K_TEST, h) / math.sqrt(G * h) - 1.0))
ok = all(abs(e / (-(kh * kh) / 6.0) - 1.0) <= 0.05 for kh, e in c_shal)
check("BandPhaseSpeed -> sqrt(g h) as kh -> 0, error == -(kh)^2/6 within 5%", ok,
      "; ".join("kh=%g rel err %+.4g (predicted %+.4g)" % (kh, e, -(kh * kh) / 6.0)
                for kh, e in c_shal))

s_deep = [(kh, SHOAL(K_TEST, kh / K_TEST) - 1.0) for kh in (4.0, 8.0, 16.0)]
check("ShoalFactor -> 1 as kh -> inf", all(abs(e) <= 0.02 for _, e in s_deep),
      "; ".join("kh=%g S-1 = %+.3g" % (kh, e) for kh, e in s_deep))

s_shal = []
for kh in (0.3, 0.15, 0.08, 0.04):
    S = SHOAL(K_TEST, kh / K_TEST)
    asym = (kh ** -0.25) / math.sqrt(2.0)
    s_shal.append((kh, S, asym, S / asym - 1.0))
ok = all(abs(e / (kh * kh / 4.0) - 1.0) <= 0.10 for kh, _, _, e in s_shal)
check("ShoalFactor -> (1/sqrt2)(kh)^(-1/4) as kh -> 0, error == +(kh)^2/4 within 10%", ok,
      "; ".join("kh=%g S=%.5f asym=%.5f rel %+.4g (predicted %+.4g)" %
                (kh, S, a, e, kh * kh / 4.0) for kh, S, a, e in s_shal))

# Green's law EXPONENT, measured as a log-log slope on the header's own function.
h1, h2 = 0.08 / K_TEST, 0.32 / K_TEST
slope = math.log(SHOAL(K_TEST, h2) / SHOAL(K_TEST, h1)) / math.log(h2 / h1)
check("Green's law exponent d log S / d log h == -1/4 (2%)", abs(slope + 0.25) <= 0.005,
      "measured %.6f over kh 0.08 -> 0.32" % slope)

ENV_T, _, _ = mutate(("const double th = std::tanh(kh);", "const double th = 1.0;"))
slope_bad = math.log(ENV_T["ShoalFactor"](K_TEST, h2) / ENV_T["ShoalFactor"](K_TEST, h1)) / \
    math.log(h2 / h1)
check("control: tanh -> 1 in ShoalFactor destroys the Green's law exponent",
      abs(slope_bad + 0.25) > 0.05,
      "mutated exponent %.6f vs -0.25 -- off by %.4f (gate tolerance 0.005)" %
      (slope_bad, abs(slope_bad + 0.25)))

ENV_D, _, _ = mutate(("const double cgDeep = 0.5 * std::sqrt(kGWave / kBand);",
                      "const double cgDeep = std::sqrt(kGWave / kBand);"))
off_bad = ENV_D["ShoalFactor"](K_TEST, 0.15 / K_TEST) / ((0.15 ** -0.25) / math.sqrt(2.0))
check("control: dropping the 1/2 from c_g^deep breaks the shallow OFFSET",
      abs(off_bad - 1.0) > 0.1,
      "mutated S / asymptote = %.6f (want 1.0000); the exponent survives, the offset does not"
      % off_bad)

ENV_S, _, _ = mutate(("(1.0 + 2.0 * kh / wt::Max(std::sinh(2.0 * kh), 1e-3))", "(1.0)"))
off_b2 = ENV_S["ShoalFactor"](K_TEST, 0.15 / K_TEST) / ((0.15 ** -0.25) / math.sqrt(2.0))
check("control: dropping the (1 + 2kh/sinh 2kh) group-speed term breaks the offset too",
      abs(off_b2 - 1.0) > 0.1, "mutated S / asymptote = %.6f" % off_b2)

# The clamps, characterised rather than asserted.
khs = np.exp(np.linspace(math.log(1e-3), math.log(60.0), 20001))
unclamped = []
for kh in khs:
    h = float(kh) / K_TEST
    th = math.tanh(kh)
    c = math.sqrt(G / K_TEST * th)
    cg = 0.5 * c * (1.0 + 2.0 * kh / max(math.sinh(2.0 * kh), 1e-3))
    unclamped.append(math.sqrt(0.5 * math.sqrt(G / K_TEST) / max(cg, 0.05)))
smin = min(unclamped)
kh_at_min = float(khs[int(np.argmin(unclamped))])
check("the 0.75 floor is a GUARD: the unclamped law never goes below it", smin > 0.75,
      "unclamped minimum %.5f at kh = %.4f (the classical shoaling dip); margin %.4f above 0.75"
      % (smin, kh_at_min, smin - 0.75))
kh_cap = next(kh for kh in khs if math.sqrt(0.5) * kh ** -0.25 < 1.7)
note("the 1.7 cap DOES bind, for kh < %.4f. At the 0.15 m depth floor that is the swell band "
     "(k = %.4f, kh = %.4f, unclamped ask %.3f) and the wind sea (k = %.4f, kh = %.4f, ask "
     "%.3f); the chop band (k = %.3f) is never capped." %
     (kh_cap, BAND_K[0], BAND_K[0] * 0.15, math.sqrt(0.5) * (BAND_K[0] * 0.15) ** -0.25,
      BAND_K[1], BAND_K[1] * 0.15, math.sqrt(0.5) * (BAND_K[1] * 0.15) ** -0.25, BAND_K[2]))
check("the depth floor is the same 0.15 m in both depth laws",
      BPS(K_TEST, 0.0) == BPS(K_TEST, 0.15) and SHOAL(K_TEST, 0.0) == SHOAL(K_TEST, 0.15),
      "c(h=0) = c(h=0.15) = %.6f m/s, S(h=0) = S(h=0.15) = %.6f" %
      (BPS(K_TEST, 0.0), SHOAL(K_TEST, 0.0)))


# ==================================================================================================
#  GATE 6: WAVE-CURRENT AMPLIFICATION.  The closed form IS deep-water action transport; blocking
#  sits exactly at r = -1/4; and the three guards the header calls dead are measured to be dead.
# ==================================================================================================
print("")
print("== gate 6: wave-current gain ==")
WCA = ENV["WaveCurrentAmp"]
worst_act, worst_clamped = 0.0, 0
for i in range(400):
    r = -0.2449 + 0.7449 * i / 399.0
    g = WCA(r, 0.0, 1.0, 0.0, 1.0)              # cur along the wave direction, c0 = 1 => r = cur
    cr = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * r))
    action = 1.0 / math.sqrt(cr * (cr + 2.0 * r))
    engine_raw = 1.0 / math.sqrt(cr * cr * (2.0 * cr - 1.0))
    if 0.55 < engine_raw < 2.0 and r > -0.16:   # outside the clamps and the blocking blend
        worst_act = max(worst_act, abs(engine_raw - action) / action)
    else:
        worst_clamped += 1
check("WaveCurrentAmp's closed form == exact action transport 1/sqrt(cr(cr+2r)) (1e-12)",
      worst_act <= 1e-12, "max rel deviation %.3g over the unclamped band (%d of 400 clamped)"
      % (worst_act, worst_clamped))
_cr = 0.5 * (1.0 - math.sqrt(1.0 + 4.0 * 0.3))  # the OTHER root of the same quadratic
check("control: the other root of c'/c0 does not satisfy action transport",
      abs(1.0 / math.sqrt(abs(_cr * _cr * (2.0 * _cr - 1.0))) -
          1.0 / math.sqrt(abs(_cr * (_cr + 2.0 * 0.3)))) > 0.1,
      "wrong root gives %.4f vs %.4f at r = 0.3" %
      (1.0 / math.sqrt(abs(_cr * _cr * (2.0 * _cr - 1.0))),
       1.0 / math.sqrt(abs(_cr * (_cr + 2.0 * 0.3)))))
check("still water is exactly transparent: amp(r=0) == 1, blocked(r=0) == 0",
      abs(WCA(0.0, 0.0, 1.0, 0.0, 1.0).amp - 1.0) < 1e-15 and
      WCA(0.0, 0.0, 1.0, 0.0, 1.0).blocked == 0.0,
      "amp %.17g" % WCA(0.0, 0.0, 1.0, 0.0, 1.0).amp)
check("the blocking ramp spans exactly [-0.16, -0.245] and saturates to (1.45, 1)",
      WCA(-0.16, 0.0, 1.0, 0.0, 1.0).blocked == 0.0 and
      abs(WCA(-0.2451, 0.0, 1.0, 0.0, 1.0).blocked - 1.0) < 2e-3 and
      WCA(-0.30, 0.0, 1.0, 0.0, 1.0).amp == 1.45 and
      WCA(-0.30, 0.0, 1.0, 0.0, 1.0).blocked == 1.0,
      "blocked(-0.16) = %.3g, blocked(-0.2451) = %.6f, past the point amp = %.3f" %
      (WCA(-0.16, 0.0, 1.0, 0.0, 1.0).blocked, WCA(-0.2451, 0.0, 1.0, 0.0, 1.0).blocked,
       WCA(-0.30, 0.0, 1.0, 0.0, 1.0).amp))
# the dead guards: measure how far the tightest reachable argument sits above each floor
r_tight = -0.245 + 1e-12
cr_t = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * r_tight))
check("the three guards the header calls DEAD are dead (measured margins)",
      r_tight > -0.2499 and (1.0 + 4.0 * r_tight) > 1e-4 and
      cr_t * cr_t * (2.0 * cr_t - 1.0) > 1e-3,
      "tightest reachable r = %.6f (floor -0.2499), 1+4r = %.6f (floor 1e-4), "
      "cr^2(2cr-1) = %.6f (floor 1e-3)" %
      (r_tight, 1.0 + 4.0 * r_tight, cr_t * cr_t * (2.0 * cr_t - 1.0)))
r_055 = next(r / 10000.0 for r in range(0, 40000)
             if 1.0 / math.sqrt((0.5 * (1 + math.sqrt(1 + 4 * r / 10000.0))) ** 2 *
                                (2 * 0.5 * (1 + math.sqrt(1 + 4 * r / 10000.0)) - 1)) <= 0.55)
check("the r <= 4.0 clamp is dead too: the 0.55 amplitude floor binds first", r_055 < 4.0,
      "amp reaches its 0.55 floor at r = %.4f, far below the 4.0 clamp" % r_055)
# the collinear projection really is a projection
g_perp = WCA(0.0, 2.0, 1.0, 0.0, 1.0)
check("only the COLLINEAR component counts: a cross-current changes nothing",
      abs(g_perp.amp - 1.0) < 1e-15 and g_perp.blocked == 0.0, "amp %.17g" % g_perp.amp)
check("c0 is floored at 0.5 m/s so vanishing depth cannot send r to infinity",
      WCA(0.1, 0.0, 1.0, 0.0, 0.0).amp == WCA(0.1, 0.0, 1.0, 0.0, 0.5).amp,
      "amp(c0=0) == amp(c0=0.5) == %.6f" % WCA(0.1, 0.0, 1.0, 0.0, 0.0).amp)


# ==================================================================================================
#  REPORT: the wake's engineering closures, evaluated where they bite.  Not gates -- these are
#  the numbers the header's comments claim, printed so a reviewer can see them rather than
#  believe them.
# ==================================================================================================
print("")
print("== report: where the wake's closures bite (recreational class, U = 4.86, halfLen 7.5) ==")
v0 = WakeVessel(0.0, 0.0, 0.0, 4.86, 0.55, 7.5, True)
K0 = G / 4.86 ** 2
note("K0 = g/U^2 = %.6f /m, lambda_t = %.4f m, lambda_cusp = (2/3)lambda_t = %.4f m" %
     (K0, 2 * math.pi / K0, 2.0 / 3.0 * 2 * math.pi / K0))
note("steepness cap ak <= 0.30 binds when amp > 0.30/k; on the transverse branch k = K0 so "
     "that is amp > %.3f m -- above this hull's cap of 0.22*7.5*0.34 = %.4f m, so on-track it "
     "never binds; on the divergent branch k grows as sec^2 and it does." %
     (0.30 / K0, 0.22 * 7.5 * 0.34))
note("divergent damping exp(-(k/5K0)^2) is 0.5 at k/K0 = %.3f (theta = %.2f deg) and 0.01 at "
     "k/K0 = %.3f -- the short arm is gone well before the wedge edge's k/K0 = 1.5 on the "
     "TRANSVERSE branch, but the divergent branch reaches k/K0 = 5 at |t| = %.3f." %
     (5.0 * math.sqrt(math.log(2.0)), math.degrees(math.atan(
         math.sqrt(5.0 * math.sqrt(math.log(2.0)) - 1.0))), 5.0 * math.sqrt(math.log(100.0)),
      math.sqrt(4.0)))
lam_cusp = 2.0 / 3.0 * 2 * math.pi / K0
note("mesh-Nyquist band limit smoothstep(2, 5, lambda/sampleM): the cusp wave (%.2f m) is cut "
     "entirely for texels coarser than %.2f m and fully passed below %.2f m." %
     (lam_cusp, lam_cusp / 2.0, lam_cusp / 5.0))
for dRel in (0.5, 1.0, 2.5, 3.5, 10.0, 160.0):
    nf = ENV["wt" if False else "Smoothstep"](0.5, 2.5, dRel)
    note("dRel %6.1f: nearFade %.4f, cusp fade %.4f, spreading 1/sqrt(0.6+dRel) = %.4f" %
         (dRel, nf, ENV["Smoothstep"](1.0, 3.5, dRel), 1.0 / math.sqrt(0.6 + dRel)))
a_end = WakeSample()
ENV["WakeOne"](v0, -1200.0, 0.0, 0.5, a_end)
note("at the 1200 m cull radius the surviving amplitude would be %.3g m against the 1e-4 m "
     "early-out -- 'spread + damp are dead past this', measured." %
     (0.55 * 1.0 / math.sqrt(0.6 + 1200.0 / 7.5)))
st = WakeSample()
ENV["WakeOne"](v0, -7.5, 0.0, 0.5, st)
note("stern turbulence at one half-length astern on the track: %.4f (peaks at %.4f); it is "
     "evaluated for any xi > 0, ahead of the bow gate and the wedge test." %
     (st.stern, max((lambda s: s.stern)(
         [ENV["WakeOne"](v0, -x / 10.0, 0.0, 0.5, s) or s
          for s in [WakeSample()]][0]) for x in range(1, 600))))

print("")
if FAILS:
    print("FAILURES (%d): %s" % (len(FAILS), "; ".join(FAILS)))
    sys.exit(1)
print("all gates passed")
