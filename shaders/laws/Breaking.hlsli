// ================================================================================================
//  laws/Breaking.hlsli -- what the water lets a sea stand to, and what it takes from one that
//  wants more: Battjes and Janssen's Miche-bounded height, the clipped-Rayleigh factor, a sea's
//  excess over its limit, the dry weight at the water's edge, the trough's bound on the column.
//  Host contract: Laws.hlsli. Needs Dispersion.hlsli for nothing; stands alone.
// ================================================================================================
#ifndef GA_LAWS_BREAKING_HLSLI
#define GA_LAWS_BREAKING_HLSLI

// Hs <= kGammaHs * h at the shore: the observed saturation (WaveField.h gammaHs).
static const real kGammaHs = 0.60;

// THE HEIGHT A WAVE CAN STAND TO, Battjes and Janssen's own (1978, their eq. for Hm):
//     Hmax(k, h) = (0.88 / k) tanh(gammaHs k h / 0.88),
// deep water (kh large): 0.88 / k, the Miche steepness limit (H k = 0.88 is ak = 0.44); the shore
// (kh small): gammaHs h, the depth limit. One tanh between them, no min of the two. `k` is the
// sea's energy-weighted wavenumber at the point (sum a^2 k / sum a^2). MEASURED why the depth limit
// alone is not enough (the Haulover bar, 3.4-6.4 m): with Hs held to 0.6 h the hull's slope ran to
// 42 degrees, because the comps shoaled there are 10-12 m long and a 2 m sea on them is ak ~ 0.6,
// past Miche; the steepness limit binds first and brings Hmax to 1.4 m where 0.6 h said 2.0.
inline real BreakHmax(real k, real depth) {
    const real kk = max(k, 1e-6);
    return (0.88 / kk) * tanh(kGammaHs * kk * max(depth, 0.05) / 0.88);
}

// BREAKING AS A DISSIPATION LAW: the clipped Rayleigh sea. The sea's wave heights at a point are
// Rayleigh about their rms; in water that lets no wave stand taller than Hmax every wave that would
// has broken DOWN TO IT. The energy that survives is the clipped distribution's (Battjes and Janssen
// 1978: the broken waves pile at Hmax):
//     E_clipped / E_raw = 1 - exp(-(Hmax / Hrms_raw)^2),   excess = Hrms_raw / Hmax,
//     L = sqrt(1 - exp(-1 / excess^2)),
// one factor on every amplitude at the point (the spectral shape and the directions survive). Its
// limits are the whole physical content: excess -> 0 (deep water), L -> 1 exponentially; excess ->
// infinity (the shore), L excess -> 1 from below: the rms tends to the limit and never reaches it.
// No min, no max, no clamp on a height anywhere.
inline real BreakFactor(real excess) {
    if (!(excess > 1e-6)) return 1.0;
    return sqrt(1.0 - exp(-1.0 / (excess * excess)));
}

// A SEA AGAINST ITS LIMIT: the raw rms envelope of a sea (sqrt(sum a^2), physical metres) at a point
// of depth h whose energy-weighted wavenumber is kEff. excess = Hs_raw / Hmax with Hs = 2 sqrt2 rms,
// the one number the breaking indicator (foam) and the dissipation read; factor the amplitude
// factor on every component. The solver's limiter (WaveField), the bank's band block and the hull's
// twin all say this.
struct SeaBreak {
    real excess;   // rms_raw / rms_limit, unclamped: how far over (or under) the sea stands
    real factor;   // BreakFactor(excess)
};
inline SeaBreak SeaBreakAt(real rmsRaw, real kEff, real depth) {
    SeaBreak b;
    b.excess = rmsRaw / max(BreakHmax(kEff, depth) / 2.8284271, 1e-6);
    b.factor = BreakFactor(b.excess);
    return b;
}

// THE DRY WEIGHT: the water's edge takes every wave off water too thin to carry one, over the
// band 5 cm .. 65 cm of depth. A rule in fixed metres (docs/WATER_LAWS.md names it); written once.
inline real DryWeight(real depth) {
    return smoothstep(0.05, 0.65, depth);
}

// WATER CANNOT STAND BELOW ITS OWN BED (2026-10-23, the bar's dimples): the drawn excursion below
// the level saturates onto the water column, dy -> -D tanh(-dy / D), D the depth -- unchanged
// where the trough is small against the water, never reaching the bed, the flat trough of a shallow
// sea. Not a cap on a height: a crest (dy > 0) is untouched.
inline real TroughBound(real dy, real depth) {
    if (!(dy < 0.0)) return dy;
    const real colD = max(depth, 1e-3);
    return -colD * tanh(-dy / colD);
}

// THE BORE'S WHITEWATER (2026-10-26, the asteroid's surge over the marsh; Mark: "a rolling wave on the
// coast if the impact is big enough"). A hydraulic jump from a column h1 to a column h2 loses the head
// (h2 - h1)^3 / (4 h1 h2) (Belanger's jump); against the deeper column's own head h2 that loss is the
// share of the water's energy turned to turbulence at the front -- the whitewater. Nought at a tide's
// creep (dh^3 over metres of water), whole at a surge's wetting front (h1 the wet depth, h2 the bore).
// One law, the jump's dissipation fraction, saturated; read in the bank kernel from the solver's
// columns a cell behind and a cell ahead along the flow.
inline real BoreFoam(real hAhead, real hBehind) {
    const real a = max(hAhead, 1e-3), b = max(hBehind, 1e-3);
    const real dh = max(b - a, 0.0);
    return saturate(dh * dh * dh / (4.0 * a * b * b));
}

#endif
