// ================================================================================================
//  HeightStackSource - M9n: THE REALIZATION, MOVED INTO THE COMPOSITOR.
//
//  The GA bed disagreed with the engine's bed by 28.3 m, and the reason was not a bug in the GA
//  path -- it reproduced its FILE exactly. It is that the engine's bed is not a file. BathyModel
//  ::RealizeFromChannel OVERWRITES every cell with Compositor::SampleHeightStack, so the
//  authoritative bed is a six-layer composite:
//
//      L0 noaa.etopo2022        4.9 km   global floor
//      L1 noaa.etopo15s.ne      461 m    regional
//      L2 noaa.cudem.capeann    13.7 m
//      L3 noaa.cudem.boston     13.7 m
//      L4 noaa.cudem.merrimack  13.7 m   <- the one layer the GA path had
//      L5 survey.edits          5 m      hand-edit polygons -- "THE LAW"
//
//  So the fix was never "read the same bytes sparsely". It was to make the thing that edits the
//  bytes a SOURCE -- which is what this adapter does, one DomainSource per existing HeightSource.
//  Nothing is reimplemented: each layer still answers through the same Sample() the renderer
//  uses, so there is no second copy of the resampling, the feathering, or the edit polygons to
//  drift out of step. The compositor gains the layers; BathyModel keeps its loop; and the two
//  paths can finally be asked whether they agree.
//
//  THE BLEND RULE HAD TO MOVE TOO, and this was the subtler half. SampleHeightStack is a
//  sequential OVER in layer order:
//
//      h += (m - h) * w                 for each layer, bottom to top
//
//  while DomainCompositor's native rule is priority-bucketed weighted AVERAGE. Those agree only
//  when every weight is 0 or 1. Across a feather -- which is exactly where a survey edit meets
//  the grid under it, and exactly where the 28 m lived -- they differ, and averaging would have
//  pulled the edit toward the CUDEM beneath it instead of laying it over. Hence
//  DomainCompositor::Blend::LayeredOver, which is this rule and only this rule.
//
//  THE DATUM, which the unit stage would otherwise have refused outright and would have been
//  RIGHT to. CUDEM and the survey edits are NAVD88. The two ETOPO layers are MSL/geoid
//  referenced and are not. The engine has always blended them as one height.
//
//  Labelling that with a zero shift would have been documentation, not a fix, so the real number
//  is used instead -- and it turned out to be already on disk. CO-OPS publishes a station's
//  datums on one staff, and Boston's cached datums_metric.json carries both:
//
//      MSL 2.660      NAVD88 2.752      ->  NAVD88 zero sits 0.092 m ABOVE local MSL
//
//  so an MSL-referenced height becomes NAVD88 by SUBTRACTING 0.092 m. main::ResolveDatum already
//  derives this same quantity from the same data ("NAVD=MSL+0.092 (from Boston)"); it is passed
//  in here rather than recomputed, so there is one derivation and one provenance string.
//
//  Its LIMIT, stated because it is real: local MSL at a tide station is not the global geoid
//  ETOPO references -- sea-surface topography separates them, and the separation varies across
//  the domain. This is the best published number available without GEOID18 or VDatum, and at
//  0.092 m it is far inside ETOPO's own vertical error at 4.9 km. What makes that tolerable is
//  not the argument, though: it is the measurement. BuildBedBank runs a SENSITIVITY probe --
//  the stack composed twice, once with the ETOPO layers displaced by a known perturbation --
//  and reports how many metres of the finished bed actually move. Where CUDEM and the edits
//  paint at full weight the answer is zero, and the datum question is provably moot THERE
//  rather than argued away everywhere.
// ================================================================================================
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "compose/ComposeTree.h"
#include "compose/Compositor.h"
#include "compose/DomainSource.h"
#include "core/Common.h"

namespace ga {

// ================================================================================================
//  HeightLayerSource -- one Compositor height layer, wearing the DomainSource interface.
// ================================================================================================
class HeightLayerSource : public DomainSource {
public:
    HeightLayerSource(HeightSource* src, const char* valueUnit)
        : m_src(src),
          m_name(src ? src->Info().name : "height"),
          m_unit(UnitSpec::Parse(valueUnit)) {}

    const char* Name() const override { return m_name.c_str(); }
    SourceDomain Domain() const override { return SourceDomain::Raster; }
    uint8_t GradeSig() const override { return 1u; }   // kG0: a scalar height
    uint32_t Channels() const override { return 1; }
    const UnitSpec& Unit() const override { return m_unit; }
    const char* NodeKind() const override { return "load"; }

    bool SampleAt(const DomainQuery& q, DomainValue& out) const override {
        out.weight = 0.0f;
        if (!m_src) return false;
        // The layers answer in radians and take the output rung as their ground resolution --
        // the same two arguments RealizeFromChannel passes, so a coarser mip asks each source
        // for its box mean rather than a point probe (the resampling doctrine, unchanged).
        constexpr double kD2R = 3.14159265358979 / 180.0;
        float m = 0.0f;
        const float w = m_src->Sample(q.lat * kD2R, q.lon * kD2R, q.groundM, m);
        if (w <= 0.0f) return false;
        out.c[0] = m;
        out.weight = w;
        return true;
    }

private:
    HeightSource* m_src = nullptr;   // borrowed; main owns the sources
    std::string m_name;
    UnitSpec m_unit;
};

// ================================================================================================
//  BuildHeightStack -- every layer of a Compositor height channel as DomainSources, in stack
//  order, each declaring the datum its data is actually in.
//
//  Returns them bottom-to-top, which is the order Blend::LayeredOver requires: the survey edits
//  must be added LAST or they stop being the law.
// ================================================================================================
// mslToNavd88M: metres to ADD to an MSL-referenced height to express it in NAVD88. Derived from
//   published CO-OPS station datums (see the header note); -0.092 m in the Gulf of Maine.
// probeM: a deliberate extra displacement applied ONLY to the MSL-referenced layers, used by the
//   sensitivity probe to measure how far the finished bed moves when their datum is wrong. Zero
//   for the real stack.
inline std::vector<std::shared_ptr<DomainSource>> BuildHeightStack(const Compositor& comp,
                                                                   int heightChannel,
                                                                   double mslToNavd88M,
                                                                   const char* provenance,
                                                                   double probeM = 0.0) {
    std::vector<std::shared_ptr<DomainSource>> out;
    if (heightChannel < 0 || heightChannel >= comp.ChannelCount()) return out;
    const Compositor::Channel& ch = comp.ChannelAt(heightChannel);
    for (HeightSource* h : ch.height) {
        if (!h) continue;
        // ETOPO is MSL/geoid referenced; CUDEM and the edits are NAVD88. Declaring that honestly
        // is what forces the link to be a real, sourced number instead of an assumption.
        const std::string& n = h->Info().name;
        const bool msl = n.rfind("noaa.etopo", 0) == 0;
        auto layer = std::make_shared<HeightLayerSource>(h, msl ? "m MSL" : "m NAVD88");
        if (msl) {
            auto linked = DeclareDatumLink(layer, mslToNavd88M + probeM, "NAVD88", provenance);
            if (linked) { out.push_back(std::move(linked)); continue; }
        }
        out.push_back(std::move(layer));
    }
    return out;
}

}  // namespace ga
