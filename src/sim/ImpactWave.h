// ================================================================================================
//  ImpactWave - AN ASTEROID INTO THE SEA, AS A PRODUCER (2026-10-25; docs/WATER_V2.md section 13).
//
//  The scene names the impactor (sea.impact: where, how big, how fast, when); this object makes the
//  cavity and the TABLE the one law sums over (laws/Impact.hlsli: a wavenumber, its frequency over the
//  basin's depth, its amplitude from the cavity's spectrum), and evaluates that law on the CPU -- for
//  the hull (WaterManager's Level producer, SUM with the tide), the solver's open faces, and anyone
//  who asks EtaAt. The same table goes to the GPU every frame (GlobeLayer, WaterBankLayer), where the
//  globe's vertices and the bank's texels sum the same terms. Stateless: the wave at any place and
//  instant is the sum; nothing steps.
//
//  Its clock is the EVENT's (WaterProducer.h Clock): t = 0 at the impact, the scene's `sea.impact.at`
//  or, unnamed, the water clock's first instant.
// ================================================================================================
#pragma once

#include "scene/SceneSchema.h"
#include "sim/WaterTerms.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace ga {

class ImpactWave {
public:
    static constexpr int kMaxTerms = 4096;   // a table's most terms (the GPU sums every one, a vertex and a texel)

    struct Cavity {
        double diameterM = 0.0, radiusM = 0.0, lipRadiusM = 0.0;
        double depthM = 0.0, depthUncappedM = 0.0;
        double impactorKgJ = 0.0, waveJ = 0.0;
    };

    // zeroUnix < 0: the event's zero is the first instant Tick sees.
    void Configure(const scene::ImpactProps& p, double planetR, double g, double zeroUnix) {
        m_p = p;
        m_R = planetR;
        m_g = g;
        m_zero = zeroUnix;
        m_on = p.enabled && p.diameterM > 0.0 && p.basinDepthM > 0.0;
        m_terms.clear();
        if (!m_on) return;
        const double rhoW = 1000.0;
        m_c.diameterM = law::ImpactCavityDiameter(p.diameterM, p.speedMs, p.densityKgM3, rhoW, p.angleDeg);
        m_c.radiusM = 0.5 * m_c.diameterM;
        m_c.lipRadiusM = law::kImpactLipRatio * m_c.radiusM;
        const double mass = p.densityKgM3 * (4.0 / 3.0) * 3.141592653589793 * std::pow(0.5 * p.diameterM, 3.0);
        m_c.impactorKgJ = 0.5 * mass * p.speedMs * p.speedMs;
        m_c.waveJ = p.energyShare * m_c.impactorKgJ;
        m_c.depthUncappedM = law::ImpactCavityDepth(m_c.waveJ, m_c.radiusM, rhoW);
        m_c.depthM = (std::min)(m_c.depthUncappedM, p.basinDepthM);
        // the table: k from 0 to kImpactKMaxOverRd / R_D in n equal steps, each term at its centre
        // THE TERMS FROM THE REACH: the sum over k in equal steps dk has an image at r = 2 pi / dk; it must
        // lie past the reach the scene asks, so dk <= 2 pi / reach and n = kMax / dk = 12 reach / (pi R_D).
        // A 1 km impactor's R_D of 14 km needs ~1600 terms for 6000 km; 512 held only to 3900 km, and the
        // image's wave stood at the Merrimack before the front (2026-10-25).
        const int byReach = static_cast<int>(std::ceil(law::kImpactKMaxOverRd * p.reachM / (3.141592653589793 * m_c.lipRadiusM)));
        m_n = std::clamp(p.terms > 0.0 ? static_cast<int>(p.terms) : byReach, 16, kMaxTerms);
        const double dk = law::kImpactKMaxOverRd / m_c.lipRadiusM / m_n;
        m_terms.resize(static_cast<size_t>(m_n) * 4);
        for (int i = 0; i < m_n; ++i) {
            const double k = (i + 0.5) * dk;
            const double w = law::ImpactOmega(k, p.basinDepthM);
            const double A = law::ImpactSpectrum(k, m_c.radiusM, m_c.depthM) * k * dk;
            m_terms[size_t(i) * 4 + 0] = static_cast<float>(k);
            m_terms[size_t(i) * 4 + 1] = static_cast<float>(w);
            m_terms[size_t(i) * 4 + 2] = static_cast<float>(A);
            m_terms[size_t(i) * 4 + 3] = 0.0f;
        }
        const double la = p.lat * kD2R, lo = p.lon * kD2R;
        m_dir[0] = std::cos(la) * std::cos(lo);   // the globe's frame: lat = asin(y), lon = atan2(z, x)
        m_dir[1] = std::sin(la);
        m_dir[2] = std::cos(la) * std::sin(lo);
    }
    bool On() const { return m_on; }
    void Tick(double simUnix) {
        if (m_on && m_zero < 0.0) m_zero = simUnix;
    }
    bool ZeroSet() const { return m_zero >= 0.0; }
    double ZeroUnix() const { return m_zero; }
    double T(double unixT) const { return m_zero >= 0.0 ? unixT - m_zero : -1.0; }
    const Cavity& C() const { return m_c; }
    const scene::ImpactProps& P() const { return m_p; }
    int Terms() const { return m_n; }
    const std::vector<float>& Table() const { return m_terms; }   // (k, omega, A, 0) x Terms()
    double PlanetR() const { return m_R; }
    double G() const { return m_g; }
    const double* Dir() const { return m_dir; }

    // The great-circle distance (m) from the impact.
    double DistanceM(double latDeg, double lonDeg) const {
        const double la = latDeg * kD2R, lo = lonDeg * kD2R;
        const double d[3] = {std::cos(la) * std::cos(lo), std::sin(la), std::cos(la) * std::sin(lo)};
        const double c = std::clamp(d[0] * m_dir[0] + d[1] * m_dir[1] + d[2] * m_dir[2], -1.0, 1.0);
        return m_R * std::acos(c);
    }
    // The deep basin's level departure (m) at distance r and t seconds after the impact.
    double EtaDeep(double rM, double tS) const {
        if (!m_on || tS < 0.0) return 0.0;
        double e = 0.0;
        for (int i = 0; i < m_n; ++i) {
            const float* T = &m_terms[size_t(i) * 4];
            e += law::ImpactTerm(T[2], T[0], T[1], rM, tS);
        }
        return e;
    }
    // The level departure (m) at a place and instant, shoaled to the depth there (the water above the
    // bed before this departure; the floor is the law's).
    // `bounded`: the shore's saturation bound (the drawn far field's, the hull's); false = Green's law
    // alone, the level a SOLVER's open face is forced by (the solver's columns are the nonlinear reader;
    // a bound there kept the asteroid's surge to six metres at the Merrimack's faces, 2026-10-25).
    double EtaAt(double latDeg, double lonDeg, double unixT, double depthM, bool bounded = true) const {
        const double t = T(unixT);
        if (!m_on || t < 0.0) return 0.0;
        const double eta = EtaDeep(DistanceM(latDeg, lonDeg), t);
        if (bounded) return law::ImpactShoal(eta, m_p.basinDepthM, depthM);
        return eta * std::pow(m_p.basinDepthM / (std::max)(depthM, law::kDepthFloorM), 0.25);
    }
    // The deep-water arrival: r over the long-wave speed sqrt(g h0), the front's time (s).
    double ArrivalS(double latDeg, double lonDeg) const {
        return DistanceM(latDeg, lonDeg) / std::sqrt(m_g * m_p.basinDepthM);
    }

private:
    static constexpr double kD2R = 3.141592653589793 / 180.0;
    scene::ImpactProps m_p;
    Cavity m_c;
    std::vector<float> m_terms;
    int m_n = 0;
    double m_R = 6371000.0, m_g = 9.81, m_zero = -1.0;
    double m_dir[3] = {1.0, 0.0, 0.0};
    bool m_on = false;
};

}  // namespace ga
