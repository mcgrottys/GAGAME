#include "sim/SeaState.h"

#include "core/Common.h"
#include "core/Json.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace ga {

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string ReadFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// Unit-amplitude spectral shapes. The absolute level comes from the numeric normalisation in
// BuildParams, so only the SHAPE matters here -- and it must match OceanCompute.hlsl exactly.
double ShapeJonswap(double f, double fp, double gamma) {
    if (f <= 1e-4) return 0.0;
    const double r = fp / f;
    const double sigma = (f <= fp) ? 0.07 : 0.09;
    const double d = (f - fp) / (sigma * fp);
    const double peak = std::pow(gamma, std::exp(-0.5 * d * d));
    return std::pow(f, -5.0) * std::exp(-1.25 * r * r * r * r) * peak;
}

double ShapeSwell(double f, double fp, double sigF) {
    const double d = (f - fp) / sigF;
    return std::exp(-0.5 * d * d);
}

double ShapeOf(const PartParam& p, double f) {
    return (p.gamma > 0.0f) ? ShapeJonswap(f, p.fp, p.gamma) : ShapeSwell(f, p.fp, p.sigF);
}

}  // namespace

bool SeaState::Load(const std::string& jsonPath) {
    const std::string text = ReadFile(jsonPath);
    if (text.empty()) {
        Log("[sea] cannot read %s", jsonPath.c_str());
        return false;
    }
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    if (!err.empty()) {
        Log("[sea] %s: JSON parse failed: %s", jsonPath.c_str(), err.c_str());
        return false;
    }

    m_cycleUnix = root.Num("cycle_unix", 0);
    m_cycleLabel = root.Str("cycle_label", "gfswave ?");

    if (const JsonValue* hours = root.Get("hours");
        hours && hours->type == JsonValue::Type::Array) {
        for (const JsonValue& jh : hours->arr) {
            SeaHour h;
            h.fh = jh.Num("fh", static_cast<double>(m_hours.size()));
            h.windMs = jh.Num("wind_ms", 0);
            h.windFromDeg = jh.Num("wind_from_deg", 0);
            if (const JsonValue* c = jh.Get("combined")) {
                h.combinedHs = c->Num("hs", 0);
                h.combinedTp = c->Num("tp", 0);
                h.combinedFromDeg = c->Num("from_deg", 0);
            }
            if (const JsonValue* parts = jh.Get("partitions");
                parts && parts->type == JsonValue::Type::Array) {
                for (const JsonValue& jp : parts->arr) {
                    SeaPartition p;
                    p.hs = jp.Num("hs", 0);
                    p.tp = jp.Num("tp", 0);
                    p.fromDeg = jp.Num("from_deg", 0);
                    p.windsea = jp.Str("kind") == "windsea";
                    if (p.hs > 0.01 && p.tp > 0.5) h.parts.push_back(p);
                }
            }
            m_hours.push_back(std::move(h));
        }
    }

    if (const JsonValue* buoys = root.Get("buoys");
        buoys && buoys->type == JsonValue::Type::Object) {
        for (const auto& kv : buoys->obj) {
            BuoyObs b;
            b.id = kv.first;
            b.obsUnix = kv.second.Num("obs_unix", 0);
            b.hs = kv.second.Num("hs", 0);
            b.dpd = kv.second.Num("dpd", 0);
            b.apd = kv.second.Num("apd", 0);
            b.mwd = kv.second.Num("mwd", 0);
            b.wspd = kv.second.Num("wspd", 0);
            if (const JsonValue* spec = kv.second.Get("spectrum")) {
                const JsonValue* fq = spec->Get("freq_hz");
                const JsonValue* dn = spec->Get("dens_m2_hz");
                if (fq && dn && fq->arr.size() == dn->arr.size()) {
                    for (size_t i = 0; i < fq->arr.size(); ++i) {
                        b.specFreqHz.push_back(static_cast<float>(fq->arr[i].number));
                        b.specDens.push_back(static_cast<float>(dn->arr[i].number));
                    }
                }
            }
            b.valid = b.hs > 0;
            Log("[sea] buoy %s: Hs %.2f m, DPD %.1f s, MWD %.0f, spectrum %zu bins",
                b.id.c_str(), b.hs, b.dpd, b.mwd, b.specFreqHz.size());
            m_buoys.push_back(std::move(b));
        }
    }

    Log("[sea] %s: %zu forecast hours, first Hs %.2f m with %zu partitions", m_cycleLabel.c_str(),
        m_hours.size(), m_hours.empty() ? 0.0 : m_hours[0].combinedHs,
        m_hours.empty() ? size_t(0) : m_hours[0].parts.size());
    return Ready();
}

const BuoyObs* SeaState::Buoy(const char* id) const {
    for (const BuoyObs& b : m_buoys) {
        if (b.id == id) return &b;
    }
    return nullptr;
}

int SeaState::HourIndex(double simUnix) const {
    if (m_hours.empty()) return 0;
    const double fh = (simUnix - m_cycleUnix) / 3600.0;
    int best = 0;
    double bestD = 1e30;
    for (size_t i = 0; i < m_hours.size(); ++i) {
        const double d = std::abs(m_hours[i].fh - fh);
        if (d < bestD) { bestD = d; best = static_cast<int>(i); }
    }
    return best;
}

PartParam SeaState::MakePartition(double hs, double tp, double fromDeg, bool windsea) {
    PartParam p;
    p.fp = static_cast<float>(1.0 / tp);
    p.gamma = windsea ? 3.3f : 0.0f;
    p.sigF = static_cast<float>(std::clamp(0.10 / (tp * tp), 0.004, 0.02));
    p.spreadS = windsea ? 8.0f : 60.0f;

    // Meteorological "from" -> propagation-toward unit vector (x east, z north).
    const double toward = (fromDeg + 180.0) * kPi / 180.0;
    p.dirToX = static_cast<float>(std::sin(toward));
    p.dirToZ = static_cast<float>(std::cos(toward));

    // Numeric normalisations: the frequency shape integrates to m0 = (Hs/4)^2, and the cos^2s
    // spreading integrates to 1 over theta. Doing this on the CPU keeps the shader a pure
    // evaluator and keeps the plotted S(f), the title-bar Hs, and the synthesised field all on
    // the same energy budget.
    double shapeInt = 0.0;
    const double f0 = 0.005, f1 = 1.2, df = 0.0005;
    PartParam probe = p;
    probe.specScale = 1.0f;
    for (double f = f0; f < f1; f += df) shapeInt += ShapeOf(probe, f) * df;
    const double m0 = (hs / 4.0) * (hs / 4.0);
    p.specScale = (shapeInt > 1e-12) ? static_cast<float>(m0 / shapeInt) : 0.0f;

    double dirInt = 0.0;
    const int nTh = 720;
    for (int i = 0; i < nTh; ++i) {
        const double th = (i + 0.5) / nTh * 2.0 * kPi - kPi;
        dirInt += std::pow(std::abs(std::cos(th * 0.5)), 2.0 * p.spreadS) * (2.0 * kPi / nTh);
    }
    p.dirNorm = (dirInt > 1e-12) ? static_cast<float>(1.0 / dirInt) : 1.0f;
    return p;
}

// M9a: Pierson-Moskowitz -- the fully-developed sea a wind raises when fetch and duration
// stop mattering. PM is written at the 19.5 m anemometer height (the Weather Reporter's mast
// the 1964 fits came from); GFS ships U10, and the open-water log profile puts
// U19.5 = 1.075 U10:
//
//     Hs = 0.0246 U19.5^2          fp = 0.877 g / (2 pi U19.5)
//
// At 2.2 m/s that is Hs 0.14 m at Tp 1.7 s -- a 4.6 m wavelength, which is cascade 2's band
// and nothing else. Textbook, not tuned: the only closure is the DECISION to apply it
// (closures.windSeaFill) and the cap the caller puts on it.
bool SeaState::WindSeaPm(double wind10Ms, double* hsOut, double* tpOut) {
    if (!(wind10Ms > 0.9)) return false;   // below this PM's Hs is under the partition floor
    const double u = 1.075 * wind10Ms;     // U10 -> U19.5
    const double fp = 0.877 * 9.81 / (2.0 * kPi * u);
    if (fp <= 1e-4) return false;
    if (hsOut) *hsOut = 0.0246 * u * u;
    if (tpOut) *tpOut = 1.0 / fp;
    return true;
}

int SeaState::BuildParams(int hourIdx, PartParam out[4]) const {
    const SeaHour& h = m_hours[hourIdx];
    int n = 0;
    auto addPartition = [&](double hs, double tp, double fromDeg, bool windsea) {
        if (n >= 4 || hs < 0.02 || tp < 0.5) return;
        out[n++] = MakePartition(hs, tp, fromDeg, windsea);
    };

    if (!h.parts.empty()) {
        for (const SeaPartition& p : h.parts) addPartition(p.hs, p.tp, p.fromDeg, p.windsea);
    } else if (h.combinedHs > 0.02) {
        // No partition data in this dataset: one JONSWAP from the combined summary.
        addPartition(h.combinedHs, h.combinedTp, h.combinedFromDeg, true);
    }
    return n;
}

double SeaState::SpectrumAt(const PartParam* parts, int n, double fHz) {
    double s = 0.0;
    for (int i = 0; i < n; ++i) s += parts[i].specScale * ShapeOf(parts[i], fHz);
    return s;
}

double SeaState::SignificantHeight(const PartParam* parts, int n) {
    double m0 = 0.0;
    for (double f = 0.005; f < 1.2; f += 0.0005) m0 += SpectrumAt(parts, n, f) * 0.0005;
    return 4.0 * std::sqrt(std::max(m0, 0.0));
}

}  // namespace ga
