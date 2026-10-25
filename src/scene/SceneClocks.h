#pragma once
// ================================================================================================
//  THE SCENE'S CLOCKS (2026-10-23, Mark: "the times for data should either be scene constants or
//  bound to another variable like xaml data binding works"; docs/WATER_V2.md 2.5, the collage rule:
//  each producer binds a clock).
//
//  One clock runs: the SCENE's (time.start, paced by time.timeScale, paused, nudged by the keys). Every
//  other clock is a statement about it, made in the scene file:
//    - a CONSTANT:  {"start": "2026-10-23T18:59:47Z", "rate": 1}  -- that instant at the scene's start,
//      carried at `rate` times the scene's pace (0 holds it);
//    - a BINDING:   {"bind": "sun.earth.clock", "offset": "-3h", "rate": 1}  -- the named clock's instant
//      at the scene's start plus the offset, carried at `rate` times THAT clock's pace (XAML's Binding
//      with its converter: the path, then an affine map);
//    - neither:     the scene's own clock (bind "time"), as every clock was before.
//  `start` and `bind` together: the binding's pace from the given instant. "now" as a start is the
//  session's first instant, read once. The clocks a scene can name: time, water.clock, sun.clock,
//  sun.earth.clock. A cycle of bindings is refused at load -- said in the log, and the clock that closes
//  it reads the scene's clock instead.
//
//  water.clock is the DATA's clock: the tide, the solver and its boundary, the currents, the sea state's
//  hour, the wind, the cascades -- every producer that reads the frame's simUnix reads it (FrameLoop
//  resolves it once a frame). sun.clock is the season, sun.earth.clock the time of day.
// ================================================================================================
#include "scene/SceneSchema.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>

namespace ga::scene {

class SceneClocks {
public:
    enum Id : int { kTime = 0, kWater, kSun, kSunEarth, kCount };

    static const char* Name(int id) {
        static const char* kNames[kCount] = {"time", "water.clock", "sun.clock", "sun.earth.clock"};
        return (id >= 0 && id < kCount) ? kNames[id] : "?";
    }
    static int IdOf(const std::string& path) {
        for (int i = 0; i < kCount; ++i) {
            if (path == Name(i)) return i;
        }
        // the short spellings a scene would write
        if (path == "water") return kWater;
        if (path == "sun") return kSun;
        if (path == "sun.earth") return kSunEarth;
        return -1;
    }

    // An offset: seconds, or a signed run of number+unit ("-3h", "1h30m", "2d", "45s", "90m").
    static bool ParseOffset(const std::string& s, double& out) {
        out = 0.0;
        if (s.empty()) return true;
        const char* p = s.c_str();
        double sign = 1.0;
        if (*p == '+' || *p == '-') { sign = (*p == '-') ? -1.0 : 1.0; ++p; }
        double total = 0.0;
        bool any = false;
        while (*p) {
            char* end = nullptr;
            const double v = std::strtod(p, &end);
            if (end == p) return false;
            p = end;
            double unit = 1.0;
            if (*p == 'd') { unit = 86400.0; ++p; }
            else if (*p == 'h') { unit = 3600.0; ++p; }
            else if (*p == 'm') { unit = 60.0; ++p; }
            else if (*p == 's') { unit = 1.0; ++p; }
            else if (*p != '\0') return false;
            total += v * unit;
            any = true;
        }
        if (!any) return false;
        out = sign * total;
        return true;
    }

    // Resolve the bindings once, at the scene's load. sceneStart: the scene clock's instant at the start;
    // parse: the scene's reader of an instant ("now" is handed in as sessionNow, read once). Returns the
    // refusals and parse errors as a log text ("" when the scene's clocks are all sound).
    template <class Parse>
    std::string Configure(const ClockProps& water, const ClockProps& sun, const ClockProps& sunEarth,
                          double sceneStart, double sessionNow, Parse parse) {
        std::string log;
        m_s0 = sceneStart;
        const ClockProps* props[kCount] = {nullptr, &water, &sun, &sunEarth};
        for (int i = 1; i < kCount; ++i) {
            const ClockProps& p = *props[i];
            Node& n = m_n[i];
            n = Node{};
            n.rate = p.rate;
            if (!p.start.empty()) {
                n.hasStart = true;
                n.start = (p.start == "now") ? sessionNow : parse(p.start);
            }
            if (!p.bind.empty()) {
                const int b = IdOf(p.bind);
                if (b < 0) {
                    log += std::string("[clock] ") + Name(i) + ": bind '" + p.bind +
                           "' names no clock (time, water.clock, sun.clock, sun.earth.clock); it reads the "
                           "scene's clock\n";
                } else {
                    n.bind = b;
                }
            }
            if (!ParseOffset(p.offset, n.offset)) {
                log += std::string("[clock] ") + Name(i) + ": offset '" + p.offset +
                       "' is not seconds or a run of number+unit (d, h, m, s); it is 0\n";
                n.offset = 0.0;
            }
        }
        // A CYCLE IS REFUSED: follow each binding; a clock that comes back to itself closes the cycle,
        // and it reads the scene's clock instead (so the rest of the chain still resolves).
        for (int i = 1; i < kCount; ++i) {
            int c = i;
            std::string path = Name(i);
            for (int step = 0; step < kCount; ++step) {
                c = m_n[c].bind;
                if (c == kTime) break;
                path += std::string(" -> ") + Name(c);
                if (c == i) {
                    log += "[clock] a cycle of bindings is refused: " + path + "; " + Name(i) +
                           " reads the scene's clock\n";
                    m_n[i].bind = kTime;
                    break;
                }
            }
        }
        return log;
    }

    // The clock's instant when the scene's clock reads sceneUnix: its anchor (the start given, else the
    // bound clock at the scene's start) plus the offset, carried at rate times the bound clock's pace.
    double At(int id, double sceneUnix) const {
        if (id <= kTime || id >= kCount) return sceneUnix;
        const Node& n = m_n[id];
        const double b0 = At(n.bind, m_s0);
        const double anchor = (n.hasStart ? n.start : b0) + n.offset;
        return anchor + n.rate * (At(n.bind, sceneUnix) - b0);
    }

    // What each clock is, for the log: "water.clock = time (rate 1)", "sun.clock = 2026-10-23T16:00:00Z
    // held", ...
    std::string Describe(int id) const {
        if (id <= kTime || id >= kCount) return "time";
        const Node& n = m_n[id];
        char buf[192];
        if (n.hasStart) {
            std::snprintf(buf, sizeof(buf), "%s = %s%s at %s's pace x %g", Name(id), Stamp(n.start).c_str(),
                          n.offset != 0.0 ? (" " + Signed(n.offset)).c_str() : "", Name(n.bind), n.rate);
        } else {
            std::snprintf(buf, sizeof(buf), "%s = %s%s x %g", Name(id), Name(n.bind),
                          n.offset != 0.0 ? (" " + Signed(n.offset)).c_str() : "", n.rate);
        }
        return buf;
    }

    // "2026-10-23 18:59:47"
    static std::string Stamp(double unix) {
        const time_t tt = static_cast<time_t>(std::llround(unix));
        tm g{};
        gmtime_s(&g, &tt);
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", g.tm_year + 1900, g.tm_mon + 1,
                      g.tm_mday, g.tm_hour, g.tm_min, g.tm_sec);
        return buf;
    }

private:
    static std::string Signed(double s) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%+gs", s);
        return buf;
    }
    struct Node {
        int bind = kTime;
        bool hasStart = false;
        double start = 0.0, offset = 0.0, rate = 1.0;
    };
    Node m_n[kCount];
    double m_s0 = 0.0;
};

}  // namespace ga::scene
