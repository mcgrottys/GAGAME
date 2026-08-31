#include "compose/WaterAtlas.h"

#include "core/Common.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace ga {

namespace {
constexpr double kPi = 3.14159265358979;
constexpr double kD2R = kPi / 180.0;

// NOAA constituent speeds (deg/hour) -- the frequency IS the identity; station fit coeffs
// are matched to a constituent by omega, never by index.
}  // namespace

const char* const WaterAtlas::kConName[WaterAtlas::kCon] = {"M2", "S2", "N2", "K1", "O1"};
const double WaterAtlas::kConSpeedDegH[WaterAtlas::kCon] = {
    28.9841042, 30.0000000, 28.4397295, 15.0410686, 13.9430356};

// ---------------------------------------------------------------- the three source rungs

namespace {

double OmegaRadS(int c) {
    return WaterAtlas::kConSpeedDegH[c] * kD2R / 3600.0;
}

// L0: the equilibrium tide -- analytic, global, LOW quality by declaration. Latitude
// structure and westward phase are right; amphidromes are absent. gamma = 1 + k2 - h2.
struct EquilibriumSource : FieldSource {
    SourceInfo info;
    int con;
    double deltaRad = 0;   // epoch calibration (set by the atlas from the station consensus)
    static constexpr double kGamma = 0.69;
    // Equilibrium amplitudes, metres (Doodson coefficients scaled).
    static constexpr double kH[WaterAtlas::kCon] = {0.2441, 0.1136, 0.0467, 0.1416, 0.1006};

    EquilibriumSource(int c) : con(c) {
        info.name = std::string("tide.equilibrium.") + WaterAtlas::kConName[c];
        info.structure = "analytic equilibrium constituent";
        info.crs = "EPSG:4326 (closed form)";
        info.cmPerPixel = 5.0e7;   // ~500 km of honest structure
    }
    const SourceInfo& Info() const override { return info; }
    float Sample(double latRad, double lonRad, double, float out[2]) override {
        const bool semi = con < 3;
        const double amp = kGamma * kH[con] *
                           (semi ? std::cos(latRad) * std::cos(latRad)
                                 : std::abs(std::sin(2.0 * latRad)));
        // Greenwich lag of the equilibrium wave: the bulge tracks the (anti)solar/lunar point,
        // so phase advances WESTWARD: G = -m * lon_east.
        const double G = -(semi ? 2.0 : 1.0) * lonRad;
        const double ph = deltaRad - G;
        out[0] = static_cast<float>(amp * std::cos(ph));
        out[1] = static_cast<float>(amp * std::sin(ph));
        return 1.0f;
    }
};

// L1: EOT20 -- the medium global base. Flat (re, im) float32 grid converted by
// harvest_water.py --eot20; nodata = exact 0 (land / outside 66S..66N). Bilinear with
// nodata-aware weights; Greenwich-lag phasors rotated to the fit epoch by the calibration.
struct Eot20Source : FieldSource {
    SourceInfo info;
    int con;
    double deltaRad = 0;
    int rows = 0, cols = 0;
    double latN = 90, latS = -90, lon0 = 0, lon1 = 360;
    std::vector<float> grid;   // rows*cols*2

    bool Load(const std::string& dir, int c) {
        con = c;
        char jp[256];
        snprintf(jp, sizeof(jp), "%s/eot20_%s.json", dir.c_str(), WaterAtlas::kConName[c]);
        std::ifstream jf(jp);
        if (!jf) return false;
        std::string txt((std::istreambuf_iterator<char>(jf)), std::istreambuf_iterator<char>());
        auto num = [&](const char* key, double dflt) {
            const size_t p = txt.find(std::string("\"") + key + "\"");
            if (p == std::string::npos) return dflt;
            return std::atof(txt.c_str() + txt.find(':', p) + 1);
        };
        rows = static_cast<int>(num("rows", 0));
        cols = static_cast<int>(num("cols", 0));
        latN = num("lat_north", 90);
        latS = num("lat_south", -90);
        lon0 = num("lon_first", 0);
        lon1 = num("lon_last", 360);
        char gp[256];
        snprintf(gp, sizeof(gp), "%s/eot20_%s.rg32", dir.c_str(), WaterAtlas::kConName[c]);
        std::ifstream gf(gp, std::ios::binary);
        if (!gf || rows <= 0 || cols <= 0) return false;
        grid.resize(static_cast<size_t>(rows) * cols * 2);
        gf.read(reinterpret_cast<char*>(grid.data()), grid.size() * 4);
        if (!gf) return false;
        info.name = std::string("tide.eot20.") + WaterAtlas::kConName[c];
        info.structure = "equirect 1/8deg (re,im) float32";
        info.crs = "EPSG:4326 | Greenwich-lag phasors, epoch-laddered";
        info.cmPerPixel = 1.25e6;   // ~12.5 km
        return true;
    }
    const SourceInfo& Info() const override { return info; }
    float Sample(double latRad, double lonRad, double, float out[2]) override {
        double lonDeg = lonRad / kD2R;
        if (lon1 > 180.0 && lonDeg < 0.0) lonDeg += 360.0;
        const double fy = (latN - latRad / kD2R) / (latN - latS) * (rows - 1);
        const double fx = (lonDeg - lon0) / (lon1 - lon0) * (cols - 1);
        const int y0 = static_cast<int>(std::floor(fy)), x0 = static_cast<int>(std::floor(fx));
        if (y0 < 0 || x0 < 0 || y0 + 1 >= rows || x0 + 1 >= cols) return 0.0f;
        const double ty = fy - y0, tx = fx - x0;
        double re = 0, im = 0, wsum = 0;
        for (int dy = 0; dy < 2; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                const size_t i = (static_cast<size_t>(y0 + dy) * cols + (x0 + dx)) * 2;
                const double r = grid[i], m = grid[i + 1];
                if (r == 0.0 && m == 0.0) continue;               // nodata (land)
                const double a2 = r * r + m * m;
                if (a2 > 100.0) continue;                          // rogue cell (S2 Fundy spike)
                const double w = (dy ? ty : 1.0 - ty) * (dx ? tx : 1.0 - tx);
                re += w * r;
                im += w * m;
                wsum += w;
            }
        }
        if (wsum < 0.05) return 0.0f;
        re /= wsum;
        im /= wsum;
        // File convention: h = re cos(theta) + im sin(theta) = A cos(theta - G) -- the
        // Greenwich phasor is (re - i im). Ours: P = e^{i delta} * (re - i im).
        const double cd = std::cos(deltaRad), sd = std::sin(deltaRad);
        out[0] = static_cast<float>(re * cd + im * sd);
        out[1] = static_cast<float>(-im * cd + re * sd);
        // The feather: full weight where all four neighbours carry data, fading at coasts.
        return static_cast<float>(std::min(1.0, wsum));
    }
};

// L2: the New England station field -- the HQ enhancement. Phasor IDW (p=2) over the fitted
// stations; coverage feathers with distance to the NEAREST station (full inside 20 km, gone
// by 90 km), so the field claims exactly the water the survey actually measured.
struct StationFieldSource : FieldSource {
    SourceInfo info;
    struct Sta {
        double latRad, lonRad;
        double re, im;   // phasor at the fit epoch
    };
    std::vector<Sta> stas;

    void Describe(int c, double lon0, double lat0, double lon1, double lat1) {
        info.name = std::string("tide.stations.ne.") + WaterAtlas::kConName[c];
        info.structure = "point harmonics, phasor IDW (fit epoch)";
        info.crs = "EPSG:4326 | CO-OPS station datums";
        info.cmPerPixel = 2.0e5;   // ~2 km: the inter-station spacing carries this
        info.lon0 = lon0;
        info.lat0 = lat0;
        info.lon1 = lon1;
        info.lat1 = lat1;
    }
    const SourceInfo& Info() const override { return info; }
    float Sample(double latRad, double lonRad, double, float out[2]) override {
        const double kx = 111320.0 * std::cos(latRad), ky = 110574.0;
        double re = 0, im = 0, wsum = 0, dminM = 1e18;
        for (const Sta& s : stas) {
            const double dx = (lonRad - s.lonRad) / kD2R * kx;
            const double dy = (latRad - s.latRad) / kD2R * ky;
            const double d2 = dx * dx + dy * dy + 1.0;
            dminM = (std::min)(dminM, std::sqrt(d2));
            const double w = 1.0 / d2;
            re += w * s.re;
            im += w * s.im;
            wsum += w;
        }
        if (wsum <= 0.0) return 0.0f;
        out[0] = static_cast<float>(re / wsum);
        out[1] = static_cast<float>(im / wsum);
        const double t = (dminM - 20000.0) / 70000.0;   // 20 km full -> 90 km gone
        return static_cast<float>(1.0 - std::clamp(t, 0.0, 1.0));
    }
};

}  // namespace

struct WaterAtlas::Impl {
    std::vector<std::unique_ptr<FieldSource>> sources;
    Compositor* comp = nullptr;
};

bool WaterAtlas::Init(Compositor& comp, const TideModel& tides, const std::string& waterDir) {
    m_tides = &tides;
    m_impl = std::make_shared<Impl>();
    m_impl->comp = &comp;
    if (tides.Count() == 0) {
        Log("[water] no fitted stations (run harvest_tides.py); atlas disabled");
        return false;
    }

    // Station phasors per constituent, matched by FREQUENCY (identity = omega, never index).
    struct P {
        double re, im, lat, lon;
    };
    std::vector<P> perCon[kCon];
    double lon0 = 1e9, lat0 = 1e9, lon1 = -1e9, lat1 = -1e9;
    for (size_t i = 0; i < tides.Count(); ++i) {
        const TideStation& s = tides.S(i);
        lon0 = (std::min)(lon0, s.lon);
        lon1 = (std::max)(lon1, s.lon);
        lat0 = (std::min)(lat0, s.lat);
        lat1 = (std::max)(lat1, s.lat);
        for (int c = 0; c < kCon; ++c) {
            const double w = OmegaRadS(c);
            for (const TideCoeff& tc : s.coeffs) {
                if (std::abs(tc.omegaRadS - w) < 1.0e-8) {
                    perCon[c].push_back({tc.ampM * std::cos(tc.phaseRad),
                                         tc.ampM * std::sin(tc.phaseRad), s.lat, s.lon});
                    break;
                }
            }
        }
    }
    m_nStations = static_cast<int>(tides.Count());

    for (int c = 0; c < kCon; ++c) {
        auto eq = std::make_unique<EquilibriumSource>(c);
        auto eot = std::make_unique<Eot20Source>();
        const bool hasEot = eot->Load(waterDir, c);
        m_hasEot20 = m_hasEot20 || hasEot;

        // THE EPOCH LADDER: rotate the global sources so their consensus at the stations
        // matches the fit-epoch phases: delta = arg( sum_s P_s * conj(Q_s) ).
        double zr = 0, zi = 0;
        if (hasEot) {
            for (const P& p : perCon[c]) {
                float q[2];
                eot->deltaRad = 0.0;
                if (eot->Sample(p.lat * kD2R, p.lon * kD2R, 0, q) < 0.2f) continue;
                zr += p.re * q[0] + p.im * q[1];    // P * conj(Q)
                zi += p.im * q[0] - p.re * q[1];
            }
        }
        const double delta = (zr != 0 || zi != 0) ? std::atan2(zi, zr) : 0.0;
        m_deltaDeg[c] = delta / kD2R;
        eot->deltaRad = delta;

        // The equilibrium base calibrates against the STATION consensus directly (its own
        // Greenwich structure is too crude to ladder through EOT20).
        double er = 0, ei = 0;
        for (const P& p : perCon[c]) {
            float q[2];
            eq->deltaRad = 0.0;
            eq->Sample(p.lat * kD2R, p.lon * kD2R, 0, q);
            er += p.re * q[0] + p.im * q[1];
            ei += p.im * q[0] - p.re * q[1];
        }
        eq->deltaRad = (er != 0 || ei != 0) ? std::atan2(ei, er) : 0.0;

        auto sta = std::make_unique<StationFieldSource>();
        sta->Describe(c, lon0 - 1.0, lat0 - 0.7, lon1 + 1.0, lat1 + 0.7);
        for (const P& p : perCon[c]) {
            sta->stas.push_back({p.lat * kD2R, p.lon * kD2R, p.re, p.im});
        }

        std::vector<FieldSource*> stack;
        stack.push_back(eq.get());
        if (hasEot) stack.push_back(eot.get());
        stack.push_back(sta.get());
        m_channel[c] =
            comp.AddFieldChannel(std::string("water.tide.") + kConName[c], std::move(stack));

        m_impl->sources.push_back(std::move(eq));
        if (hasEot) m_impl->sources.push_back(std::move(eot));
        m_impl->sources.push_back(std::move(sta));
    }

    Log("[water] atlas: %d stations (%d..%d constituent fits), eot20 %s, epoch ladder "
        "M2 %+.1f S2 %+.1f N2 %+.1f K1 %+.1f O1 %+.1f deg",
        m_nStations, static_cast<int>(perCon[kCon - 1].size()),
        static_cast<int>(perCon[0].size()), m_hasEot20 ? "loaded" : "ABSENT (--eot20)",
        m_deltaDeg[0], m_deltaDeg[1], m_deltaDeg[2], m_deltaDeg[3], m_deltaDeg[4]);
    m_ready = true;
    return true;
}

void WaterAtlas::Phasor(int c, double latDeg, double lonDeg, double groundResM,
                        float out[2]) const {
    m_impl->comp->SampleFieldStack(m_channel[c], latDeg * kD2R, lonDeg * kD2R, groundResM,
                                   out);
}

double WaterAtlas::Level(double latDeg, double lonDeg, double unixT, double groundResM) const {
    const double tau = unixT - m_tides->EpochUnix();
    double h = 0.0;
    for (int c = 0; c < kCon; ++c) {
        float p[2];
        Phasor(c, latDeg, lonDeg, groundResM, p);
        const double wt = OmegaRadS(c) * tau;
        h += p[0] * std::cos(wt) - p[1] * std::sin(wt);   // Re[P e^{iwt}]
    }
    return h;
}

void WaterAtlas::EnvelopeNavd(double latDeg, double lonDeg, double aroundUnix, float* loM,
                              float* hiM, double groundResM) const {
    // M8g: THE ORIGIN PLANES. The tidal datum envelope at a point: min/max of the same
    // stateless constituent sum Level() evaluates, scanned over one synodic month
    // (29.53 d -- the spring/neap beat closes) at 10-minute steps, referenced to NAVD88
    // through MslNavd. This is MLLW/MHHW's spatial generalization: between gauges the
    // envelope interpolates through the SAME station graph the live level rides, so a
    // place with no data still knows its tidal band. The phasors are time-independent --
    // hoisted once, the scan is a pure rotor sum (microseconds, honest extremes; the
    // sum-of-amplitudes bound overestimates because five incommensurate rotors never
    // quite align inside a month).
    float p[kCon][2];
    for (int c = 0; c < kCon; ++c) Phasor(c, latDeg, lonDeg, groundResM, p[c]);
    // aroundUnix <= 0 = the fit epoch itself (deterministic boot-time default).
    const double tau0 = (aroundUnix > 0.0) ? aroundUnix - m_tides->EpochUnix() : 0.0;
    double lo = 1e18, hi = -1e18;
    constexpr double kStepS = 600.0;
    constexpr int kN = static_cast<int>(29.53 * 86400.0 / kStepS);
    for (int k = 0; k < kN; ++k) {
        const double tau = tau0 + k * kStepS;
        double h = 0.0;
        for (int c = 0; c < kCon; ++c) {
            const double wt = OmegaRadS(c) * tau;
            h += p[c][0] * std::cos(wt) - p[c][1] * std::sin(wt);
        }
        lo = (std::min)(lo, h);
        hi = (std::max)(hi, h);
    }
    const double msl = MslNavd(latDeg, lonDeg);
    *loM = static_cast<float>(msl + lo);
    *hiM = static_cast<float>(msl + hi);
}

// ---------------------------------------------------------------- the selftest gate

bool RunWaterSelfTest() {
    TideModel tides;
    if (!tides.Load("data/tides/stations.json") || tides.Count() == 0) {
        Log("[watertest] no fitted stations (data/tides absent) -- SKIPPED");
        return true;
    }
    Compositor comp;
    WaterAtlas wa;
    if (!wa.Init(comp, tides, "data/water")) {
        Log("[watertest] FAIL: atlas init");
        return false;
    }
    bool ok = true;
    auto fail = [&](const char* what) {
        Log("[watertest] FAIL: %s", what);
        ok = false;
    };

    // 1. GEOPOSITION: the Mercator window math round-trips every station position.
    for (size_t i = 0; i < tides.Count(); ++i) {
        const TideStation& s = tides.S(i);
        const double worldPx = 262144.0;   // z10
        const double Y = (1.0 - std::log(std::tan(kPi / 4.0 + s.lat * kD2R / 2.0)) / kPi) /
                         2.0 * worldPx;
        const double latBack = std::atan(std::sinh(kPi * (1.0 - 2.0 * Y / worldPx))) / kD2R;
        if (std::abs(latBack - s.lat) > 1.0e-9) fail("mercator round-trip");
    }

    // 2. DATUM LADDER: every NAVD-linked station's MSL lands within +-0.6 m of NAVD zero
    // (New England MSL ~ +0.0..0.3 NAVD; a metre-scale excursion means a smuggled datum).
    int linked = 0;
    for (size_t i = 0; i < tides.Count(); ++i) {
        const TideStation& s = tides.S(i);
        if (s.mllwMinusNavdM < -900.0) continue;
        ++linked;
        const double mslNavd = s.meanMllwM + s.mllwMinusNavdM;
        if (std::abs(mslNavd) > 0.6) {
            Log("[watertest]   %s MSL = %+.2f m NAVD", s.name.c_str(), mslNavd);
            fail("datum ladder excursion");
        }
    }

    // 3. FIT REPRODUCTION: at each station the composed stack must return the station's own
    // M2 phasor (the IDW pins it), and the 5-constituent Level must match the same partial
    // sum evaluated straight from the fit coefficients.
    const double t0 = tides.EpochUnix() + 1234567.0;
    double worstAmp = 0, worstLvl = 0;
    for (size_t i = 0; i < tides.Count(); ++i) {
        const TideStation& s = tides.S(i);
        double m2amp = 0, m2ph = 0;
        double direct = 0;
        for (int c = 0; c < WaterAtlas::kCon; ++c) {
            const double w = WaterAtlas::kConSpeedDegH[c] * kD2R / 3600.0;
            for (const TideCoeff& tc : s.coeffs) {
                if (std::abs(tc.omegaRadS - w) < 1.0e-8) {
                    direct += tc.ampM * std::cos(w * 1234567.0 + tc.phaseRad);
                    if (c == 0) {
                        m2amp = tc.ampM;
                        m2ph = tc.phaseRad;
                    }
                    break;
                }
            }
        }
        float p[2];
        wa.Phasor(0, s.lat, s.lon, 500.0, p);
        const double fAmp = std::hypot(p[0], p[1]);
        worstAmp = (std::max)(worstAmp, std::abs(fAmp - m2amp));
        const double lvl = wa.Level(s.lat, s.lon, t0);
        worstLvl = (std::max)(worstLvl, std::abs(lvl - direct));
        (void)m2ph;
    }
    Log("[watertest] fit reproduction: worst M2 amp err %.1f mm, worst level err %.1f mm "
        "over %zu stations",
        worstAmp * 1000.0, worstLvl * 1000.0, tides.Count());
    if (worstAmp > 0.02 || worstLvl > 0.03) fail("station fit reproduction");

    // 4. THE SEAM: level along a transect from Gloucester Harbor out 200 km east must be
    // CONTINUOUS across the station-field feather into the global base (a phase cliff at
    // the handover is the epoch-ladder bug this gate exists to catch).
    double prev = 0, worstStep = 0;
    for (int k = 0; k <= 100; ++k) {
        const double lon = -70.66 + k * (2.4 / 100.0);
        const double lvl = wa.Level(42.60, lon, t0);
        if (k) worstStep = (std::max)(worstStep, std::abs(lvl - prev));
        prev = lvl;
    }
    Log("[watertest] offshore seam: worst 2.4 km step %.1f mm%s", worstStep * 1000.0,
        wa.HasEot20() ? "" : " (equilibrium base only -- run harvest_water --eot20)");
    if (worstStep > 0.10) fail("seam discontinuity (epoch ladder?)");

    // 5. THE SOAK CONTRACT for fields: a mid-Atlantic tile keeps only the global sources;
    // a Boston tile includes the station field.
    {
        Compositor::TileBox farBox{};
        farBox.latMin = 30.0 * kD2R;
        farBox.latMax = 30.1 * kD2R;
        farBox.lonMin = -40.0 * kD2R;
        farBox.lonMax = -39.9 * kD2R;
        farBox.texLat = (farBox.latMax - farBox.latMin) / 128.0;
        farBox.texLon = (farBox.lonMax - farBox.lonMin) / 128.0;
        std::vector<size_t> incFar, incNear;
        comp.FieldSubset(comp.ChannelAt(wa.ChannelId(0)), farBox, incFar);
        Compositor::TileBox near0 = farBox;
        near0.latMin = 42.3 * kD2R;
        near0.latMax = 42.4 * kD2R;
        near0.lonMin = -70.8 * kD2R;
        near0.lonMax = -70.7 * kD2R;
        comp.FieldSubset(comp.ChannelAt(wa.ChannelId(0)), near0, incNear);
        if (incNear.size() <= incFar.size()) fail("field soak subset");
    }

    // 6. PAINT-AND-SAMPLE IDENTITY: one composed RG16F tile over the harbor reproduces the
    // CPU stack at texel centres (half-float tolerance) -- the realization cannot drift
    // from the query path.
    {
        const int zBase = 10;
        const double worldPx = 262144.0;
        const double lat = 42.5, lon = -70.7;
        const double X = (lon / 360.0 + 0.5) * worldPx;
        const double Y = (1.0 - std::log(std::tan(kPi / 4.0 + lat * kD2R / 2.0)) / kPi) / 2.0 *
                         worldPx;
        const long long orgX = static_cast<long long>(X) & ~127ll;
        const long long orgY = static_cast<long long>(Y) & ~127ll;
        auto fn = comp.WindowField(wa.ChannelId(0), orgX, orgY, 128, zBase);
        TileRequest r{};
        r.face = 0;
        r.mip = 0;
        r.x = 0;
        r.y = 0;
        std::vector<uint8_t> tile;
        if (!fn(r, tile) || tile.size() != 65536) {
            fail("field tile paint");
        } else {
            const uint16_t* px = reinterpret_cast<const uint16_t*>(tile.data());
            double worst = 0;
            for (int k = 0; k < 16; ++k) {
                const int tx = (k * 37) % 128, ty = (k * 53) % 128;
                const double plat =
                    std::atan(std::sinh(kPi * (1.0 - 2.0 * (orgY + ty + 0.5) / worldPx)));
                const double plon = ((orgX + tx + 0.5) / worldPx - 0.5) * 2.0 * kPi;
                float v[2];
                comp.SampleFieldStack(wa.ChannelId(0), plat, plon, 152.0, v);
                worst = (std::max)(
                    worst,
                    static_cast<double>(std::abs(HalfToFloat(px[(ty * 128 + tx) * 2]) - v[0])));
            }
            if (worst > 0.005) fail("tile vs stack identity");
        }
    }

    // 7. THE ORIGIN PLANES (M8g): the datum envelope must CONTAIN every sampled live
    // level (it is the min/max of the same rotor sum -- containment is definitional,
    // so a violation means the two code paths diverged), and at the Merrimack entrance
    // its width must be the known great-diurnal-scale range (~2.4..3.6 m).
    {
        const double elat = 42.8190, elon = -70.8031;
        float elo = 0.0f, ehi = 0.0f;
        wa.EnvelopeNavd(elat, elon, 0.0, &elo, &ehi);
        const double msl = wa.MslNavd(elat, elon);
        double worstOut = 0.0;
        for (int k = 0; k < 60; ++k) {
            const double t = tides.EpochUnix() + k * 41231.0;   // ~29 d, incommensurate
            const double lvl = msl + wa.Level(elat, elon, t);
            worstOut = (std::max)(worstOut, (std::max)(elo - lvl, lvl - ehi));
        }
        Log("[watertest] envelope: lo %+.2f hi %+.2f m NAVD (width %.2f), worst "
            "containment excursion %.0f mm",
            elo, ehi, ehi - elo, worstOut * 1000.0);
        if (worstOut > 0.02) fail("envelope containment");
        if (ehi - elo < 2.4f || ehi - elo > 3.6f) fail("envelope width (entrance range)");
    }

    Log("[watertest] ---- %s: %d stations (%d NAVD-linked), eot20 %s, phasor fields "
        "M2/S2/N2/K1/O1 ----",
        ok ? "PASS" : "FAIL", wa.StationsUsed(), linked, wa.HasEot20() ? "on" : "off");
    return ok;
}

double WaveAtlasNavdDelta(const TideModel* tides) {
    // The regional NAVD-MSL offset, calibrated at the nearest station carrying a
    // published link (the main.cpp ResolveDatum recipe: delta = -mllwMinusNavd -
    // meanMllw; smallest |delta| wins -- Boston gives +0.092 m here).
    double best = 0.0;
    bool found = false;
    for (size_t i = 0; i < tides->Count(); ++i) {
        const TideStation& s = tides->S(i);
        if (s.mllwMinusNavdM < -900.0) continue;
        const double delta = -s.mllwMinusNavdM - s.meanMllwM;
        if (!found || std::abs(delta) < std::abs(best)) best = delta;
        found = true;
    }
    return best;
}

double WaterAtlas::MslNavd(double latDeg, double lonDeg) const {
    // Station-IDW of MSL in NAVD88. M8 datum fix: stations WITHOUT a published NAVD
    // link contribute through the regional MSL transfer (msl_navd = -delta) instead of
    // being skipped -- skipping them let the one linked RIVER station (Riverside,
    // msl +0.43 from river slope) dominate the entrance by distance and bias the
    // solver's level +0.46 m. With the transfer, the entrance's own station answers
    // for the entrance. Beyond the survey the geoid stands in for MSL (0).
    const double delta = WaveAtlasNavdDelta(m_tides);
    double acc = 0, wsum = 0, dmin = 1e18;
    const double kx = 111320.0 * std::cos(latDeg * kD2R), ky = 110574.0;
    for (size_t i = 0; i < m_tides->Count(); ++i) {
        const TideStation& s = m_tides->S(i);
        const bool linked = s.mllwMinusNavdM > -900.0;
        const double mslNavd =
            linked ? (s.meanMllwM + s.mllwMinusNavdM) : -delta;
        const double dx = (lonDeg - s.lon) * kx, dy = (latDeg - s.lat) * ky;
        const double d2 = dx * dx + dy * dy + 1.0;
        dmin = (std::min)(dmin, std::sqrt(d2));
        const double w = 1.0 / d2;
        acc += w * mslNavd;
        wsum += w;
    }
    if (wsum <= 0.0) return 0.0;
    const double t = std::clamp((dmin - 20000.0) / 70000.0, 0.0, 1.0);
    return (acc / wsum) * (1.0 - t);
}

}  // namespace ga
