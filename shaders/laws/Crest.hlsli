// ================================================================================================
//  laws/Crest.hlsli -- THE CREST LAW and its parts: the lip's fall and flight, the landing, the
//  spiller's roller, the swash, which crests plunge. One law for every description of the sea (the
//  solved field, the cascades) on both processors (ALGEBRA.md priors 53-57; docs/WATER_V2.md
//  12.12-12.15). Host contract: Laws.hlsli; needs Dispersion.hlsli.
// ================================================================================================
#ifndef GA_LAWS_CREST_HLSLI
#define GA_LAWS_CREST_HLSLI

// THE LIP'S FALL (2026-10-20; docs/WATER_VOLUME.md 15). Water thrown e metres past the cusp has left
// the face at the crest's speed c and falls for t = e / c, a t^2 / 2. Two corrections from the
// literature: c WITH AMPLITUDE -- linear dispersion runs ~20 % slow at breaking, the cnoidal form
// c^2 = g h (1 + H / h) carried onto the full dispersion as c^2 = (g / k) tanh(kh) (1 + H / h); and a,
// THE JET TIP'S MEASURED FALL: 0.6 - 0.8 g, not g (Shakeri, Tavakolinejad, Maxeiner and Duncan, APS
// DFD 2007). So fall = (a / g) k coth(kh) e^2 / (2 (1 + H / h)), and g cancels: no planet's gravity
// in it. Nothing at the cusp (e = 0).
static const real kJetFallRatio = 0.7;   // a / g, measured (0.6 - 0.8)
inline real LipFall(real ePast, real k, real coth, real H, real h) {
    return kJetFallRatio * 0.5 * k * coth * ePast * ePast / (1.0 + max(H, 0.0) / max(h, 0.3));
}
// d(LipFall)/d(ePast), H held
inline real LipFallDe(real ePast, real k, real coth, real H, real h) {
    return kJetFallRatio * k * coth * ePast / (1.0 + max(H, 0.0) / max(h, 0.3));
}

// THE THROW'S BOUND IS THE LANDING (2026-10-20, Mark: "I see the lip but the waves don't seem to
// barrel correctly"). The jet flies until it lands in the trough: LipFall(e*) = Hc, the crest's
// height over the trough, gives e* = sqrt(2 Hc (1 + Hc / h) / ((a / g) k coth(kh))), and the crest's
// reach is the cusp plus that flight, 1 / k + e*. No new constant: a set wave throws farther and
// lands, as it does.
inline real LipReach(real k, real coth, real Hc, real h) {
    const real H = max(Hc, 0.0);
    const real eLand = sqrt(2.0 * H * (1.0 + H / max(h, 0.3)) / max(kJetFallRatio * k * coth, 1e-6));
    return 1.0 / max(k, 1e-4) + eLand;
}

// THE SPILLER'S ROLLER (2026-10-23, Mark: "deduce the right spiller roll method"). Water that cannot
// stand on a slope runs down it: a height dH carried dH / tan(theta) along a slope tan(theta). A
// spilling crest is that law on the wave's OWN front face: the part of the crest over the depth's
// limit, dH = Hc (1 - 1 / eCr) (eCr the crest's own excess), runs down the face as the roller. The
// face is this crest's, tan(theta) = k Hc / 2 (Duncan 1981 measured the roller riding a face at
// ~12 degrees; k Hc / 2 is 0.15 - 0.3 for a breaking crest). Its reach past the cusp is
// 2 (1 - 1 / eCr) / k: nothing at the limit, the cusp's own length at twice it.
inline real SpillReach(real Hc, real eCr, real k) {
    const real face = max(max(k, 1e-4) * 0.5 * max(Hc, 0.0), 1e-4);
    return max(Hc, 0.0) * (1.0 - 1.0 / max(eCr, 1.0)) / face;
}

// THE SWASH (2026-10-23, Mark: "the raw orbit near the waterline for the run-up and backwash"). At
// the waterline the bore the breaking sea has become runs up the slope and back with the momentum
// of the wave that was: the raw amplitude's orbit. The share of the raw orbit is the swash zone's,
// exp(-h / R), R the run-up height: R = 0.043 sqrt(Hs L0), Stockdon, Holman, Howd and Sallenger
// (Coastal Engineering 53, 2006), the dissipative beaches' R2% (xi < 0.3: no slope in it). 1 at the
// waterline, ~e^-1 where the water is a run-up deep, nothing offshore. L0 the deep-water wavelength
// of the band or component.
inline real SwashShare(real hsRaw, real L0, real depth) {
    const real R = max(0.043 * sqrt(max(hsRaw, 0.0) * max(L0, 0.0)), 1e-3);
    return exp(-max(depth, 0.0) / R);
}
// The lateral's factor in the swash: the delivered orbit raised toward the raw one, raw / delivered
// = rawW (1 / the breaking factor for a band; max(excess, 1) for a solved component), by the swash's
// share on THIS band's or component's own run-up. 1 offshore and wherever nothing broke.
inline real SwashFactor(real rawW, real hsRaw, real L0, real depth) {
    if (!(rawW > 1.0)) return 1.0;
    return 1.0 + SwashShare(hsRaw, L0, depth) * (rawW - 1.0);
}

// WHICH CRESTS PLUNGE (2026-10-22, Mark at the Merrimack bar: "we clip water heights"). The throw
// past the fold onset is a PLUNGING breaker's, and a beach makes plungers only where it is steep
// against the wave: the surf-similarity (Iribarren) number xi = tan(beta) / sqrt(H / L0),
// L0 = 2 pi / (k tanh kh) (Battjes 1974; Iribarren & Nogales 1949): SPILLING below ~0.4, PLUNGING
// to ~2.0, SURGING above. The share of the throw a crest takes is the plunging band's, the product
// of two logistics in ln xi about those bounds, width s = ln(1.5) / (2 ln 9) = 0.092 (10 % to 90 %
// across a factor of 1.5 in xi): a CLOSURE, the classes are bands in the data. tan(beta) is the
// bed's steepest slope (waves refract to face the contours before they break).
inline real PlungeShare(real tanBeta, real Hc, real k, real h) {
    const real kk = max(k, 1e-4);
    const real L0 = 6.2831853 / (kk * tanh(kk * max(h, 0.3)));
    const real xi = max(tanBeta, 1e-6) / sqrt(max(Hc, 1e-3) / L0);
    const real lx = log(xi);
    const real sw = 0.0922;
    return 1.0 / ((1.0 + exp(-(lx - log(0.4)) / sw)) * (1.0 + exp((lx - log(2.0)) / sw)));
}

// The share of the crest's variance over the depth's limit: 0 below it, the plunge's gate. eCr is
// the crest's OWN excess -- the sea's raw ratio, unclamped, times the crest over the significant
// amplitude (a ratio is never clamped before it multiplies: priors 54).
inline real CrestExcessShare(real eta, real aEnv, real excRaw) {
    const real eCr = excRaw * max(eta / aEnv, 0.0);
    return 1.0 - 1.0 / max(eCr * eCr, 1.0);
}

// THE STATE THE CREST LAW ACTS ON: one description's surface at a label -- its elevation, its
// lateral offset (dx, dz) in the units the caller holds it, its slope and the offset's Jacobian.
// A kernel that does not draw the Jacobian passes zeros and ignores them.
struct CrestState {
    real eta;
    real dx;
    real dz;
    real sx;
    real sz;
    real jxx;
    real jxz;
    real jzz;
};

// THE CREST LAW, ONE FOR EVERY DESCRIPTION OF THE SEA (2026-10-23; WATER_V2 12.13 and 12.15). On one
// description's surface at a label (x east, y up, z north; its orbit already on the amplitude the
// depth lets stand, raised toward the raw one in the swash):
//   - at the crest alone (w = 1/2 + eta / 2 A_env), by the crest's own excess, EVERY breaking crest
//     goes forward to the cusp (k A_h = 1: the crest's water at the phase speed, the kinematic
//     criterion, a spiller's as much as a plunger's);
//   - past the cusp the jet flies toward the landing (LipReach) only where the bed's slope says it
//     plunges (PlungeShare gates the flight alone); the spilling share runs down its own face
//     (SpillReach);
//   - water past the cusp descends: the jet by the parabola of its fall (LipFall), the roller by its
//     face's slope; the flight is ONE translator, along the sheet then the fall (Longuet-Higgins'
//     free-fall frame as a boost: GA.hlsli MotorPoint with the real part 1 and the dual part tau/2,
//     Pga.h Motor::Translation -- the sandwich on a point is the shift p + tau, which is what this
//     text, having no float4, writes);
//   - bounded by the trough.
// dBx, dBz: the sea's travel; aEnv: its significant amplitude; excRaw: rms_raw / rms_limit,
// unclamped; kBar, cothBar: its wavenumber in this depth and orbit factor; aH: its DRAWN lateral
// amplitude; post: the factor the caller multiplies the lateral by afterwards (the solved field's
// chop in WaveField::ProbeAt; 1 where the lateral is already drawn); tanBeta: the bed's slope;
// depth. The slope and the Jacobian ride through by the product rule (grad w = grad eta / 2 A_env
// inside the band). Returns the new state and, in `q1`, the crest's forward share (the spray's).
struct CrestOut {
    CrestState s;
    real q1;
};
inline CrestOut CrestLaw(CrestState s, real dBx, real dBz, real aEnv, real excRaw, real kBar, real cothBar,
                         real aH, real post, real tanBeta, real depth) {
    const real wRaw = 0.5 + 0.5 * s.eta / aEnv;
    const real wCr = clamp(wRaw, 0.0, 1.0);
    const bool inBand = wRaw > 0.0 && wRaw < 1.0;
    const real Hc = aEnv + max(s.eta, 0.0);   // this crest's height over the trough
    const real aLoop = LipReach(kBar, cothBar, Hc, depth);
    const real share = CrestExcessShare(s.eta, aEnv, excRaw);
    const real plunge = (share > 0.0) ? PlungeShare(tanBeta, Hc, kBar, depth) : 0.0;
    // to the cusp for every breaker; past it, the plunging share's jet to the landing and the
    // spilling share's roller down its own face (at plunge 1 the law of before)
    const real cusp = 1.0 / max(kBar, 1e-4);
    const real eCr = excRaw * max(s.eta / aEnv, 0.0);
    const real spill = SpillReach(Hc, eCr, kBar);
    const real past = spill + plunge * ((aLoop - cusp) - spill);
    const real rH = 1.0 / max(aH, 1e-3), cuspR = cusp * rH;
    const real q1 = share * (max(cuspR - 1.0, 0.0) + max((cusp + past) * rH - max(cuspR, 1.0), 0.0));
    const real face = max(kBar, 1e-4) * 0.5 * Hc;   // the roller's face, its drop per metre
    const real g = 1.0 + q1 * wCr;                   // the orbit grows; the throw is the flight's
    const real wx = inBand ? 0.5 * s.sx / aEnv : 0.0;   // grad(w) (s.sx is grad(eta) so far)
    const real wz = inBand ? 0.5 * s.sz / aEnv : 0.0;
    // the flight: the throw per unit of w along the sheet (the launch direction is the travel
    // projected into the sheet's tangent plane), horizontal in the state's units, vertical as drawn
    const real m = dBx * s.sx + dBz * s.sz;
    const real r = 1.0 / sqrt(1.0 + m * m);
    const real aHu = aH / max(post, 1e-6);
    const real shX = q1 * aHu * dBx * r, shZ = q1 * aHu * dBz * r;
    const real shY = q1 * aH * m * r;
    const real tx = q1 * s.dx + shX, tz = q1 * s.dz + shZ;   // d(D_new)/d(w)
    CrestOut o;
    o.q1 = q1;
    o.s = s;
    o.s.jxx = s.jxx * g + tx * wx;
    o.s.jzz = s.jzz * g + tz * wz;
    o.s.jxz = s.jxz * g + 0.5 * (tx * wz + tz * wx);
    // water past the cusp descends: the jet by the parabola of its fall, the roller by its face.
    // THE ROLLER RUNS DOWN THE FACE IT SITS ON (2026-10-25, Mark: "the tops seem a bit flat"): the
    // roller is water that cannot stand on the front face sliding down it, so a label is lowered by the
    // roller in the share of the face it sits on -- the sheet's descent along the travel, -m, over the
    // roller's own face, k Hc / 2, saturated: nothing on the level top (m = 0) and on the back face
    // (m > 0), the whole drop where the sheet is as steep as the roller's face or steeper. Lowered at
    // crest weight alone, the top dropped as one shelf with a lip at its end (the plateau); now the
    // top stays rounded and leans forward, and the face below it carries the roller. The jet is in
    // free flight and falls by its parabola wherever it is.
    const real wFace = saturate(-m / face);
    const real ePast = max(aH * g - cusp, 0.0);
    const real fall = ePast * face * wFace + plunge * (LipFall(ePast, kBar, cothBar, Hc, depth) - ePast * face * wFace);
    const real fallDe = face * wFace + plunge * (LipFallDe(ePast, kBar, cothBar, Hc, depth) - face * wFace);   // d(fall)/d(ePast)
    const real px = s.dx * g + shX * wCr, py = s.eta + shY * wCr - fall, pz = s.dz * g + shZ * wCr;
    const real floorY = min(s.eta, -aEnv);
    if (py >= floorY) {
        // the slope through w: the rise along the sheet up, the fall down (d(fall)/d(w))
        const real dd = shY - ((ePast > 0.0) ? fallDe : 0.0) * aH * q1;
        o.s.sx = s.sx + dd * wx;
        o.s.sz = s.sz + dd * wz;
        o.s.eta = py;
    } else {
        o.s.sx = 0.0;
        o.s.sz = 0.0;
        o.s.eta = floorY;
    }
    o.s.dx = px;
    o.s.dz = pz;
    return o;
}

#endif
