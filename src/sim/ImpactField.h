// ================================================================================================
//  ImpactField - THE IMPACTS, AS A RECORD OF EVENTS (2026-10-25, Mark: "a player or some director
//  can cause impacts at will"; docs/WATER_V2.md section 13).
//
//  An impact is an EVENT: an impactor (scene::ImpactProps), where it lands, and the instant it does
//  (the event's own clock zero). The field is the list of them, appended to at any instant -- by the
//  scene at boot (`impacts`), by a key at the point under the cursor (FrameLoop, the scene's
//  `sea.impactor`), by anything that holds the field -- and read everywhere the water is read: the
//  CPU sums every event's law (ImpactWave) for the hull and the solver's faces; the GPU readers sum
//  the same tables from ONE buffer this object packs (Pack), rebuilt when the record changes
//  (Version). The record is replayable: a second player handed the same events sees the same sea.
// ================================================================================================
#pragma once

#include "core/Common.h"
#include "sim/ImpactGpu.h"
#include "sim/ImpactWave.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace ga {

class ImpactField {
public:
    struct Event {
        scene::ImpactProps props;
        double zeroUnix = -1.0;   // < 0 until the clock's first instant names it (Tick)
        ImpactWave wave;
    };
    static constexpr int kHeaderWords = 8;

    void Init(double planetR, double g) {
        m_R = planetR;
        m_g = g;
    }
    // zeroUnix < 0: the event begins at the clock's next instant (Tick).
    int Add(const scene::ImpactProps& p, double zeroUnix) {
        Event e;
        e.props = p;
        e.props.enabled = true;
        e.zeroUnix = zeroUnix;
        e.wave.Configure(e.props, m_R, m_g, zeroUnix);
        const ImpactWave::Cavity& c = e.wave.C();
        Log("[impact] event %zu: %.0f m at %.0f m/s into %.3f N %.3f E at %s: cavity %.1f km across, %.0f m deep "
            "(the energy says %.0f; the ocean %.0f), %.2e J in the wave, %d terms (the sum holds to %.0f km)",
            m_events.size(), p.diameterM, p.speedMs, p.lat, p.lon,
            zeroUnix < 0.0 ? "the clock's next instant" : p.at.empty() ? "the named unix instant" : p.at.c_str(),
            c.diameterM / 1000.0, c.depthM, c.depthUncappedM, p.basinDepthM, c.waveJ, e.wave.Terms(),
            2.0 * 3.141592653589793 * c.lipRadiusM * e.wave.Terms() / law::kImpactKMaxOverRd / 1000.0);
        m_events.push_back(std::move(e));
        ++m_version;
        return static_cast<int>(m_events.size()) - 1;
    }
    // The clock's instant: an event without a zero takes it (and the record changes).
    void Tick(double simUnix) {
        for (Event& e : m_events) {
            if (e.zeroUnix < 0.0) {
                e.zeroUnix = simUnix;
                e.wave.Tick(simUnix);
                ++m_version;
                Log("[impact] event %zu begins at %.1f (the clock's instant)", &e - m_events.data(), simUnix);
            }
        }
    }
    bool Any() const { return !m_events.empty(); }
    size_t Count() const { return m_events.size(); }
    const std::vector<Event>& Events() const { return m_events; }
    uint64_t Version() const { return m_version; }
    double PlanetR() const { return m_R; }
    double G() const { return m_g; }

    // The level departure (m) at a place and instant: every event's wave, each shoaled to the depth
    // there (the law's bound is per wave), summed.
    double EtaAt(double latDeg, double lonDeg, double unixT, double depthM, bool bounded = true) const {
        double e = 0.0;
        for (const Event& ev : m_events) e += ev.wave.EtaAt(latDeg, lonDeg, unixT, depthM, bounded);
        return e;
    }
    bool Begun(double unixT) const {
        for (const Event& ev : m_events) {
            if (ev.zeroUnix >= 0.0 && unixT >= ev.zeroUnix) return true;
        }
        return false;
    }
    // The first event's arrival at a place (s after its zero), for a scene's log.
    double ArrivalS(double latDeg, double lonDeg) const {
        return m_events.empty() ? 0.0 : m_events.front().wave.ArrivalS(latDeg, lonDeg);
    }

    // THE GPU'S COPY: words. Each event: a header of kHeaderWords (dir x, y, z; zero less the base;
    // the basin depth; the display exaggeration; its terms' first word; its term count), the headers
    // first, then every event's terms (k, omega, A, 0). Events without a zero yet are packed with
    // no terms (the GPU sums nothing of them). `base` is the earliest zero (float precision for t).
    void Pack(std::vector<float>& words, double& base) const {
        base = 0.0;
        bool any = false;
        for (const Event& ev : m_events) {
            if (ev.zeroUnix < 0.0) continue;
            base = any ? (std::min)(base, ev.zeroUnix) : ev.zeroUnix;
            any = true;
        }
        words.clear();
        words.resize(m_events.size() * kHeaderWords, 0.0f);
        for (size_t i = 0; i < m_events.size(); ++i) {
            const Event& ev = m_events[i];
            float* h = &words[i * kHeaderWords];
            const double* d = ev.wave.Dir();
            h[0] = static_cast<float>(d[0]);
            h[1] = static_cast<float>(d[1]);
            h[2] = static_cast<float>(d[2]);
            h[3] = static_cast<float>(ev.zeroUnix >= 0.0 ? ev.zeroUnix - base : 0.0);
            h[4] = static_cast<float>(ev.props.basinDepthM);
            h[5] = static_cast<float>(ev.props.exag);
            h[6] = static_cast<float>(words.size());   // its terms begin here (patched below)
            h[7] = 0.0f;
        }
        for (size_t i = 0; i < m_events.size(); ++i) {
            const Event& ev = m_events[i];
            words[i * kHeaderWords + 6] = static_cast<float>(words.size());
            if (ev.zeroUnix < 0.0) continue;
            const std::vector<float>& t = ev.wave.Table();
            words[i * kHeaderWords + 7] = static_cast<float>(ev.wave.Terms());
            words.insert(words.end(), t.begin(), t.begin() + size_t(ev.wave.Terms()) * 4);
        }
    }
    // The per-frame constants for the readers, given the buffer the packed words live in.
    ImpactGpu Gpu(uint32_t slot, double base, double simUnix) const {
        ImpactGpu g;
        g.slot = slot;
        g.events = static_cast<uint32_t>(m_events.size());
        g.tRel = static_cast<float>(simUnix - base);
        g.planetR = static_cast<float>(m_R);
        g.g = static_cast<float>(m_g);
        return g;
    }

private:
    std::vector<Event> m_events;
    uint64_t m_version = 1;
    double m_R = 6371000.0, m_g = 9.81;
};

}  // namespace ga
