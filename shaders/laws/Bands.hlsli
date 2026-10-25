// ================================================================================================
//  laws/Bands.hlsli -- the three FFT cascades' amplitude law at a point (the bank kernel's band
//  block and the hull's twin, one text): each band's gain from the sea-state scale, the swell
//  shadow, the wave-current gain and Green's shoaling; the sea those gains make against the depth's
//  limit; the one breaking factor on every band. Then the cascades' yield to the solved field and
//  the solved window's weight. Host contract: Laws.hlsli; needs Dispersion.hlsli and Breaking.hlsli.
// ================================================================================================
#ifndef GA_LAWS_BANDS_HLSLI
#define GA_LAWS_BANDS_HLSLI

struct BandSea {
    real gain[3];      // each band's RAW amplitude gain (before breaking: the detail plane's, the swash's)
    real blocked[3];   // each band's blocking indicator (foam), never an amplitude
    real rmsRaw;       // the raw envelope the gains make before breaking, physical m
    real kEff;         // the sea's energy-weighted wavenumber at the point
    real excess;       // its Hs over the depth's limit (SeaBreakAt)
    real factor;       // the clipped-Rayleigh factor: the caller draws gain x factor
};

// THE BAND LAW. hsScale: the local sea-state scale (WaveScale, 1 under a declared storm); expo: the
// swell shadow (1 exposed); curX/curZ: the solver's current (the kernel has no other, so neither
// does its twin); peakX/peakZ: the sea's peak travel, the wave-current gain's projection (none:
// gain 1); k0..k2 the bands' representative wavenumbers (OceanFft::BandCut's geometric means);
// rms0..rms2 the unit sea's rms per band, PHYSICAL (the display exaggeration is applied once, to
// the summed displacement, never inside a law).
inline BandSea BandLaw(real hsScale, real expo, real depth, real curX, real curZ, real peakX, real peakZ,
                       bool peakValid, real k0, real k1, real k2, real rms0, real rms1, real rms2) {
    BandSea s;
    const real kB[3] = {k0, k1, k2};
    const real rm[3] = {rms0, rms1, rms2};
    for (int c = 0; c < 3; ++c) {
        real amp = hsScale * expo;
        real blocked = 0.0;
        if (depth > 0.05) {
            const real cB = BandPhaseSpeed(kB[c], max(depth, 0.3));
            WaveCurrentGain g;
            g.amp = 1.0;
            g.blocked = 0.0;
            if (peakValid) g = WaveCurrentAmp(curX, curZ, peakX, peakZ, cB);
            amp *= g.amp * ShoalFactor(kB[c], depth);
            blocked = g.blocked;
        }
        s.gain[c] = amp;
        s.blocked[c] = blocked;
    }
    // the sea the gains make: its raw envelope, its energy wavenumber, its excess, the one factor
    real e[3];
    real eSum = 0.0, eSumK = 0.0;
    for (int c = 0; c < 3; ++c) {
        e[c] = s.gain[c] * s.gain[c] * rm[c] * rm[c];
        eSum += e[c];
        eSumK += e[c] * kB[c];
    }
    s.rmsRaw = max(sqrt(eSum), 1e-4);
    s.kEff = eSumK / max(eSum, 1e-12);
    const SeaBreak b = SeaBreakAt(s.rmsRaw, s.kEff, depth);
    s.excess = b.excess;
    s.factor = b.factor;
    return s;
}

// THE CASCADES YIELD BY THE ENERGY THE SOLVED FIELD CARRIES (2026-10-25, Mark: "the waves seem a bit
// flat going into the inlet"). Inside the solved window the structure bands (0 and 1) stand down by
// the share of their energy the solved field supplies, min(1, E_solved / E_bands): where the solved
// sea is whole it owns the bands; where it is weak (a lee, a channel) the bands fill to their own
// level. wCas in: the window's weight times the share of the solved planes that answered; rmsSolved:
// the solved sea's delivered rms here; rb0, rb1: the bands' own delivered rms (gain x unit rms).
inline real YieldShare(real wCas, real rmsSolved, real rb0, real rb1) {
    const real eBands = rb0 * rb0 + rb1 * rb1;
    if (!(eBands > 1e-12)) return wCas;
    return wCas * min(1.0, rmsSolved * rmsSolved / eBands);
}

// THE SOLVED WINDOW'S WEIGHT: the distance to the nearest window edge, in metres, feathered over
// featherM so the handover to the cascades is invisible. cx, cy in cells; nx, ny the window; cellM.
inline real WindowEdgeM(real cx, real cy, real nx, real ny, real cellM) {
    return min(min(cx, nx - cx), min(cy, ny - cy)) * cellM;
}
inline real WindowWeight(real edgeM, real featherM) {
    return smoothstep(0.0, max(featherM, 1.0), edgeM);
}

#endif
