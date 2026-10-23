// ================================================================================================
//  WaterProducer - WATER V2, STAGE 1 (docs/WATER_V2.md sections 2-3): THE SIX PRIMITIVES, DECLARED.
//
//  The water is a sum of declared functions of time over one bed; a solver exists only to carry the
//  part no function can. Every input the water reads is a PRODUCER, and a producer is the product of
//  six declarations:
//
//    Quantity  what it says          level | current | bed | sea | wind | memory, each with ONE law
//                                    for where two producers of it meet (CombineOf)
//    Support   where it speaks       point | grid | tree | window | analytic, each with ONE reach
//                                    law: absence is a weight of 0, never a value of 0
//    grain     the ground (m) it was made at: read coarser it is folded, finer it is magnified
//    TimeForm  how it depends on t   static | harmonic | packet | series | stochastic | transient
//    Clock     whose t it keeps      the scene's, or an event's own zero
//    identity  what its products key on: content, never a place or a name (0: not keyed yet)
//
//  WaterManager holds the registry. Query composes the producers that answer at a place
//  (Producer::at) by their quantity's law; the others are DECLARED, their reader named, so the
//  manager knows every datum's reach and time before stage 2 hands the GPU one term table.
// ================================================================================================
#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <string>

namespace ga::water {

enum class Quantity : uint8_t { Level, Current, Bed, Sea, Wind, Memory };
constexpr int kQuantities = 6;

// Where two producers of one quantity meet (WATER_V2 2.1). SUM of departures: a tide plus a surge
// plus a tsunami (a solver window REPLACES what it resolves: WaterManager::SolverRefine). OVER, the
// finer over the coarser, the later declared on top. ENERGY then the fold: the sea's spectra
// (SeaLayer::PartsAt sums partitions by energy; Query reads only the sea's summary, over). OWN:
// history is a solver's state.
enum class Combine : uint8_t { Sum, Over, Energy, Own };
constexpr Combine CombineOf(Quantity q) {
    return (q == Quantity::Level || q == Quantity::Current) ? Combine::Sum
           : q == Quantity::Sea                             ? Combine::Energy
           : q == Quantity::Memory                          ? Combine::Own
                                                            : Combine::Over;
}

enum class Support : uint8_t { Point, Grid, Tree, Window, Analytic };
enum class TimeForm : uint8_t { Static, Harmonic, Packet, Series, Stochastic, Transient };

// A producer's clock: the scene's t, or an EVENT's own zero (a tsunami's clock starts at impact).
struct Clock {
    bool event = false;
    double zeroUnix = 0.0;
    double Local(double unixT) const { return event ? unixT - zeroUnix : unixT; }
};

// ---- THE REACH OF A POINT (a tide station, a buoy) ---------------------------------------------
// Its weight is 1 / d^2 times PointTaper: whole to kPointFullM, nought at kPointReachM, smoothstep
// between -- one station or buoy speaks for a coast's water, not a planet's. The SHARE the points
// may claim of a place at all is PointCoverage of the nearest one: whole within 20 km, none past
// 90 km. (Was written three times: the atlas's stations, its MSL surface, the buoys.)
constexpr double kPointFullM = 90000.0, kPointReachM = 270000.0;
inline double PointTaper(double dM) {
    const double t = std::clamp((dM - kPointFullM) / (kPointReachM - kPointFullM), 0.0, 1.0);
    return 1.0 - t * t * (3.0 - 2.0 * t);
}
inline double PointCoverage(double dMinM) {
    return 1.0 - std::clamp((dMinM - 20000.0) / 70000.0, 0.0, 1.0);
}

// What a producer says at a place and instant: up to three components (level, bed: [0]; current,
// wind: east, north; the sea's summary: Hs, Tp, from-degrees), the ones it says (`has`, a bit each),
// those it says only where what is composed so far is not positive (`fills`: a point forecast's Hs
// where the grid has none -- absence is not zero), and its provenance (a static string).
struct Sample {
    double v[3] = {0.0, 0.0, 0.0};
    uint8_t has = 0, fills = 0;
    const char* src = "-";
};

struct Producer {
    std::string name;
    Quantity q = Quantity::Level;
    Support support = Support::Analytic;
    TimeForm form = TimeForm::Static;
    double grainM = 0.0;    // the native ground; 0 = none (analytic)
    Clock clock{};
    uint64_t identity = 0;
    std::string reader;     // who reads it today (a declared producer Query does not evaluate)
    // The answer at (lat, lon, t, ground); false = no opinion here (weight 0). Empty: declared only.
    std::function<bool(double latDeg, double lonDeg, double unixT, double groundResM, Sample&)> at;
};

const char* Name(Quantity q);
const char* Name(Support s);
const char* Name(TimeForm f);

}  // namespace ga::water
