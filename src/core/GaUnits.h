// ================================================================================================
//  GaUnits - M9l: THE NORMALIZATION STAGE. The pipeline is
//
//      GA Load -> GA Unit/Scale/Projection normalize -> GA Compose -> GA physics -> sparse GPU
//
//  and this file is the second arrow. Until it existed that arrow was a COMMENT: DomainCompositor's
//  Add() said "a product carries one grade and one unit" while checking only grade and channel
//  count, and GeoRef::valueUnit was written by every loader, logged once at boot, and read by
//  nothing. A bed in "m NAVD88" and a survey in "ft MLLW" would have composed to full coverage and
//  a number that is neither.
//
//  WHY THIS CANNOT LIVE IN THE LOADER: a loader's job is to report what its file says, faithfully,
//  including units it cannot convert. Converting there would erase the provenance that makes a
//  refusal explainable -- "this file is feet" is the useful half of the error. And WHY IT CANNOT
//  LIVE IN THE COMPOSITOR: the compositor sees values, not files; by the time a float arrives its
//  unit is already gone. So normalization is its own stage with its own type, sitting between them,
//  and the compositor's job narrows to VERIFYING that what it was handed came through it.
//
//  WHAT CONVERTS AND WHAT IS REFUSED
//
//    converts      a scale factor and nothing else     ft -> m, cm/s -> m/s, sRGB byte -> unit
//    refused       a different quantity                m/s into m; no factor exists
//    refused       a different vertical datum          NAVD88 into MLLW with no stated offset
//
//  The datum rule is the one most likely to be waved through, and it is the one that matters. The
//  offset between NAVD88 and MLLW is a REAL, SITE-SPECIFIC number -- about 1.30 m at Newburyport,
//  different at every station, published per station by CO-OPS and not derivable from the grids.
//  A library that guessed it would put a plausible bed at the wrong depth everywhere, which is
//  strictly worse than one that stops. So: same datum composes, different datum refuses, and an
//  operator who KNOWS the offset states it through WithDatumShift and owns it.
//
//  That refusal is not an obstacle to the tide -- it is the tide's type signature. TideModel
//  already carries mllwMinusNavdM per station, the exact published link this rule demands, and
//  ComposeTree.h's water-level node is what supplies it. The rule turned a hand-carried constant
//  in BathyModel into a declared edge in the graph.
//
//  Scale and projection are already normalized and stay where they are: GeoRef::MetersPerTexelX
//  converts degrees to metres for the ladder, RasterSource::ToSourceCrs puts WGS84 into the
//  source's own CRS exactly. This file is the third leg -- the VALUE's unit, which had no home.
// ================================================================================================
#pragma once
#include <cctype>
#include <cstdint>
#include <string>

namespace ga {

// What a number IS, independent of which unit it is written in. Two fields compose only if these
// match -- no factor turns a speed into a depth, and pretending otherwise is the whole failure
// this enum exists to make unrepresentable.
enum class Quantity : uint8_t {
    Unknown,        // unparsed: composes only with itself, and says so out loud
    Dimensionless,  // coverage, masks, indices
    Length,         // depths, elevations, wave heights
    Velocity,       // currents, wind
    Angle,          // directions, phases
    Pressure,
    Temperature,
    Colour,         // imagery
};

inline const char* QuantityName(Quantity q) {
    switch (q) {
        case Quantity::Dimensionless: return "1";
        case Quantity::Length:        return "length";
        case Quantity::Velocity:      return "velocity";
        case Quantity::Angle:         return "angle";
        case Quantity::Pressure:      return "pressure";
        case Quantity::Temperature:   return "temperature";
        case Quantity::Colour:        return "colour";
        default:                      return "unknown";
    }
}

// ================================================================================================
//  UnitSpec -- a parsed valueUnit: which quantity, how to reach canonical, and (for a Length) the
//  vertical datum it is measured from.
//
//  Canonical is SI -- metres, m/s, radians, pascals, kelvin, colour on [0,1] -- because that is
//  what the solver and the shaders already assume. The point of a canonical form is that every
//  stage after this one stops asking.
// ================================================================================================
struct UnitSpec {
    Quantity quantity = Quantity::Unknown;
    double toCanonical = 1.0;      // canonical = raw * toCanonical + datumShiftM
    double datumShiftM = 0.0;      // Length only, and only where someone declared it
    std::string datum;             // "NAVD88", "MLLW", "MSL"; empty = not a levelled height
    std::string raw;               // what the file actually said, kept for the error message

    bool Known() const { return quantity != Quantity::Unknown; }
    bool IsCanonical() const { return toCanonical == 1.0 && datumShiftM == 0.0; }

    // Parse a loader's valueUnit. Deliberately forgiving about the descriptive tail -- loaders
    // write "m/s (u east, v north)" and "m NAVD88" -- and deliberately strict about the leading
    // token, because that is the part a wrong guess would silently corrupt.
    static UnitSpec Parse(const char* s) {
        UnitSpec u;
        if (!s || !*s) return u;
        u.raw = s;

        // Lowercase, and cut at the first parenthesis so a descriptive tail cannot affect a match.
        std::string t;
        for (const char* p = s; *p && *p != '('; ++p) {
            t.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(*p))));
        }
        while (!t.empty() && (t.back() == ' ' || t.back() == '\t')) t.pop_back();

        const size_t sp = t.find(' ');
        const std::string head = (sp == std::string::npos) ? t : t.substr(0, sp);
        const std::string tail = (sp == std::string::npos) ? std::string() : t.substr(sp + 1);

        struct Row { const char* tok; Quantity q; double f; };
        static const Row kTable[] = {
            {"m", Quantity::Length, 1.0},          {"meter", Quantity::Length, 1.0},
            {"meters", Quantity::Length, 1.0},     {"metre", Quantity::Length, 1.0},
            {"metres", Quantity::Length, 1.0},     {"cm", Quantity::Length, 0.01},
            {"mm", Quantity::Length, 0.001},       {"km", Quantity::Length, 1000.0},
            {"ft", Quantity::Length, 0.3048},      {"feet", Quantity::Length, 0.3048},
            {"foot", Quantity::Length, 0.3048},    {"fathom", Quantity::Length, 1.8288},
            {"fathoms", Quantity::Length, 1.8288},
            {"m/s", Quantity::Velocity, 1.0},      {"ms-1", Quantity::Velocity, 1.0},
            {"cm/s", Quantity::Velocity, 0.01},    {"kn", Quantity::Velocity, 0.514444},
            {"knot", Quantity::Velocity, 0.514444},{"knots", Quantity::Velocity, 0.514444},
            {"kt", Quantity::Velocity, 0.514444},  {"mph", Quantity::Velocity, 0.44704},
            {"km/h", Quantity::Velocity, 1.0 / 3.6},
            {"rad", Quantity::Angle, 1.0},         {"radian", Quantity::Angle, 1.0},
            {"radians", Quantity::Angle, 1.0},
            {"deg", Quantity::Angle, 3.14159265358979323846 / 180.0},
            {"degree", Quantity::Angle, 3.14159265358979323846 / 180.0},
            {"degrees", Quantity::Angle, 3.14159265358979323846 / 180.0},
            {"pa", Quantity::Pressure, 1.0},       {"hpa", Quantity::Pressure, 100.0},
            {"mbar", Quantity::Pressure, 100.0},   {"millibar", Quantity::Pressure, 100.0},
            {"k", Quantity::Temperature, 1.0},     {"kelvin", Quantity::Temperature, 1.0},
            {"srgb", Quantity::Colour, 1.0 / 255.0},
            {"rgb", Quantity::Colour, 1.0 / 255.0},
            {"byte", Quantity::Colour, 1.0 / 255.0},
            {"unorm", Quantity::Colour, 1.0},
            {"1", Quantity::Dimensionless, 1.0},   {"none", Quantity::Dimensionless, 1.0},
            {"fraction", Quantity::Dimensionless, 1.0},
            {"mask", Quantity::Dimensionless, 1.0},
        };
        for (const Row& r : kTable) {
            if (head == r.tok) { u.quantity = r.q; u.toCanonical = r.f; break; }
        }
        // "sRGB byte": the leading token names the encoding, the factor lives on the tail.
        if (u.quantity == Quantity::Colour && tail.find("byte") != std::string::npos) {
            u.toCanonical = 1.0 / 255.0;
        }

        // A levelled height names its datum; a wave height or a cell size does not. Only Length
        // carries one, because only Length has a zero that somebody had to choose.
        if (u.quantity == Quantity::Length && !tail.empty()) {
            static const char* kDatums[] = {"navd88", "navd", "mllw",  "mlw",  "msl",
                                            "mhhw",   "mhw",  "wgs84", "egm96"};
            for (const char* d : kDatums) {
                if (tail.find(d) != std::string::npos) {
                    u.datum.assign(d);
                    for (char& c : u.datum) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
                    if (u.datum == "NAVD") u.datum = "NAVD88";   // the engine writes both
                    break;
                }
            }
        }
        return u;
    }

    // Build a target by hand, for the cases with no file behind them.
    static UnitSpec Of(Quantity q, const char* datumName = "") {
        UnitSpec u;
        u.quantity = q;
        u.toCanonical = 1.0;
        u.datum = datumName ? datumName : "";
        u.raw = std::string(QuantityName(q)) + (u.datum.empty() ? "" : " " + u.datum);
        return u;
    }

    // An operator who knows the site's offset states it and owns it. Added AFTER scaling, and it
    // retargets the datum name so a later comparison sees the frame the values are now in.
    UnitSpec WithDatumShift(double metres, const char* toDatum) const {
        UnitSpec u = *this;
        u.datumShiftM = metres;
        u.datum = toDatum ? toDatum : "";
        return u;
    }

    // Can `other` become THIS by a scale and an offset? Quantity must match exactly, and a Length
    // must already sit on the same datum -- the offset between two datums is site data, not
    // arithmetic, so this returns false rather than inventing one.
    bool AcceptsFrom(const UnitSpec& other, std::string* why = nullptr) const {
        if (quantity != other.quantity) {
            if (why) {
                *why = std::string("quantity ") + QuantityName(other.quantity) + " into " +
                       QuantityName(quantity);
            }
            return false;
        }
        if (quantity == Quantity::Unknown) {
            if (why) *why = "unit not understood: '" + other.raw + "'";
            return false;
        }
        if (quantity == Quantity::Length && datum != other.datum) {
            if (why) {
                *why = "vertical datum " +
                       (other.datum.empty() ? std::string("(none)") : other.datum) + " into " +
                       (datum.empty() ? std::string("(none)") : datum) +
                       " -- that offset is published site data, declare it with WithDatumShift";
            }
            return false;
        }
        return true;
    }

    std::string Describe() const {
        std::string s = raw.empty() ? std::string("(unstated)") : raw;
        s += " [";
        s += QuantityName(quantity);
        if (!datum.empty()) s += " " + datum;
        s += "]";
        return s;
    }
};

}  // namespace ga
