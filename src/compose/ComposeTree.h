// ================================================================================================
//  ComposeTree - M9m: COMPOSITION AS A REVERSE TREE.
//
//  The user's framing, and it corrected a real flatness in the design: composing is not one node
//  with a list of sources. It is a DAG that CONVERGES -- many leaves, fewer nodes, one product --
//  read like a gantt chart, where each row also has a CADENCE and a VALIDITY, and a row cannot
//  start before the rows it depends on.
//
//      merrimack CUDEM  (static, one load) ---------+
//                                                    +--> water.depth  (per frame)
//      tide stations    (analytic, per instant) ----+
//
//  Before this file a DomainCompositor was a leaf-eater: sources in, page out, and nothing could
//  consume a composed product except the GPU. Three things follow from making a compositor a
//  SOURCE:
//
//    1. TREES. A composite is a source, so it feeds another compositor. The bed can be
//       "CUDEM under survey edits" -- itself a two-source blend -- and everything downstream sees
//       one bed and never learns it was two.
//    2. THE UNIT CHECK COMPOSES WITH IT. Each node declares a unit; a parent verifies its
//       children. A wrong datum four levels down is caught at the edge where it is introduced,
//       named, with both frames printed -- not as a wrong depth at the waterline.
//    3. CADENCE BECOMES VISIBLE. The bed composes once; the tide is a function of t and must be
//       re-evaluated every instant. Same tree, different rows, different clocks. That is the
//       gantt half, and PrintTree prints it.
//
//  THE TIDE STEP, and why it needed its own node.
//
//  BathyModel's header has carried this for months: "CUDEM elevations are NAVD88; the tide model
//  speaks MLLW. Near Newburyport MLLW sits ~1.30 m below NAVD88 zero ... tunable via --datum".
//  A hand-carried constant, in a comment, applied inside one class -- invisible to any compositor
//  and unavailable to any other product that might want the same water surface.
//
//  GaUnits made that constant illegal to guess: a Length in MLLW cannot become a Length in NAVD88
//  without a DECLARED offset. That refusal is not an obstacle to the tide, it is the tide's type
//  signature -- and the number it demands already exists, per station, as the CO-OPS datum link
//  TideStation::mllwMinusNavdM. So the tide arrives as a source that knows which frame it can
//  speak in, and a station with no published link (-999) answers with WEIGHT ZERO rather than a
//  plausible number. Absence, again, is the honest answer -- the same rule the ingest path uses
//  for nodata, applied to a vertical datum.
//
//  And then DEPTH is a subtraction inside the algebra, not a special case:
//
//      water.navd88 [length NAVD88]  -  bed.navd88 [length NAVD88]  =  depth [length, NO datum]
//
//  The output losing its datum is the unit algebra being RIGHT, not sloppy. Two levelled heights
//  differ by a thickness, and a thickness has no zero to be measured from -- so the result will
//  refuse to compose with anything still carrying a datum, which is exactly the mistake a person
//  would otherwise make once and never find.
// ================================================================================================
#pragma once
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "compose/DomainSource.h"
#include "core/Common.h"
#include "sim/TideModel.h"

namespace ga {

// ================================================================================================
//  CompositeSource -- a DomainCompositor wearing the DomainSource interface. THE tree edge.
// ================================================================================================
class CompositeSource : public DomainSource {
public:
    CompositeSource(std::string name, std::shared_ptr<DomainCompositor> comp, int priority = 0)
        : m_name(std::move(name)), m_comp(std::move(comp)), m_priority(priority) {}

    const char* Name() const override { return m_name.c_str(); }
    SourceDomain Domain() const override { return SourceDomain::Raster; }
    uint8_t GradeSig() const override { return m_comp ? m_comp->Grade() : 0; }
    uint32_t Channels() const override { return m_comp ? m_comp->Channels() : 0; }
    const UnitSpec& Unit() const override { return m_comp->Unit(); }
    int Priority() const override { return m_priority; }

    bool SampleAt(const DomainQuery& q, DomainValue& out) const override {
        return m_comp && m_comp->SampleBlended(q, out);
    }

    size_t InputCount() const override { return m_comp ? m_comp->SourceCount() : 0; }
    const DomainSource* Input(size_t i) const override {
        return m_comp ? m_comp->SourceAt(i) : nullptr;
    }
    const char* NodeKind() const override { return "compose"; }
    bool MayCover(double lon0, double lat0, double lon1, double lat1) const override {
        for (size_t i = 0; i < InputCount(); ++i) {
            if (Input(i)->MayCover(lon0, lat0, lon1, lat1)) return true;
        }
        return false;
    }
    // A composite's identity is its rule plus its inputs', in order: change any input, or the
    // order, or the blend, and every tile this node ever cached is correctly orphaned.
    std::string Identity() const override {
        std::string id = "compose|" + m_name + "|" +
                         (m_comp && m_comp->BlendMode() == DomainCompositor::Blend::LayeredOver
                              ? "over"
                              : "priavg");
        for (size_t i = 0; i < InputCount(); ++i) id += "|(" + Input(i)->Identity() + ")";
        return id;
    }
    const DomainCompositor* Comp() const { return m_comp.get(); }
    // The union of the inputs' footprints. An input with no declared footprint makes the
    // union unknown, which the caller treats as global.
    bool Footprint(double& lon0, double& lat0, double& lon1, double& lat1) const override {
        bool any = false;
        double a = 180, b = 90, c = -180, d = -90;
        for (size_t i = 0; i < InputCount(); ++i) {
            double p, q, r, s;
            if (!Input(i)->Footprint(p, q, r, s)) return false;
            a = (std::min)(a, p); b = (std::min)(b, q); c = (std::max)(c, r); d = (std::max)(d, s);
            any = true;
        }
        if (!any) return false;
        lon0 = a; lat0 = b; lon1 = c; lat1 = d;
        return true;
    }

private:
    std::string m_name;
    std::shared_ptr<DomainCompositor> m_comp;
    int m_priority = 0;
};

// ================================================================================================
//  GateSource -- M9ak/M9am: a layer's weight, MULTIPLIED by another source's coverage.
//
//  "The GIS mask gates, the height band refines." The survey says where water CAN be; the bed
//  classifier's alpha says where the waterline actually falls inside that. A compositor resolves
//  disagreement about ONE quantity, and BinaryFieldSource combines two quantities into a third;
//  this is a third kind of edge again: the VALUE is the layer's, untouched, and only the WEIGHT
//  is the product. Distinct so that it prints as what it is in the tree.
//
//  A gate with no coverage at a point has no opinion -- the layer passes at its own weight. A
//  gate that IS present and says zero blocks. Absent and empty are opposite answers, and the tile
//  cache below has to preserve that distinction too (see TileTree).
// ================================================================================================
class GateSource : public DomainSource {
public:
    GateSource(std::string name, std::shared_ptr<DomainSource> layer,
               std::shared_ptr<DomainSource> gate)
        : m_name(std::move(name)), m_layer(std::move(layer)), m_gate(std::move(gate)) {}

    const char* Name() const override { return m_name.c_str(); }
    SourceDomain Domain() const override { return m_layer->Domain(); }
    uint8_t GradeSig() const override { return m_layer->GradeSig(); }
    uint32_t Channels() const override { return m_layer->Channels(); }
    int Priority() const override { return m_layer->Priority(); }
    const UnitSpec& Unit() const override { return m_layer->Unit(); }
    const char* NodeKind() const override { return "gate"; }
    const char* Cadence() const override { return m_layer->Cadence(); }
    size_t InputCount() const override { return 2; }
    const DomainSource* Input(size_t i) const override {
        return i == 0 ? m_layer.get() : (i == 1 ? m_gate.get() : nullptr);
    }
    bool MayCover(double a, double b, double c, double d) const override {
        return m_layer->MayCover(a, b, c, d);
    }
    bool Footprint(double& a, double& b, double& c, double& d) const override {
        return m_layer->Footprint(a, b, c, d);
    }
    std::string Identity() const override {
        return "gate2|(" + m_layer->Identity() + ")<(" + m_gate->Identity() + ")";   // M9ay value gate
    }
    bool SampleAt(const DomainQuery& q, DomainValue& out) const override {
        if (!m_layer->SampleAt(q, out) || out.weight <= 0.0f) return false;
        DomainValue g;
        // Outside the gate's footprint it is not consulted: no opinion, the layer passes.
        const bool present = m_gate->MayCover(q.lon, q.lat, q.lon, q.lat);
        if (!present) return true;
        // M9ay: a VALUE gate. Present and silent (a void tile) still blocks. Where the gate
        // answers, its COVERAGE says whether it has an opinion (0 = none: the layer passes)
        // and its value channel IS the factor -- the survey's water coverage, 1 water, 0
        // land. Before this the weight was the factor, so land had to be painted as ABSENCE,
        // and a land texel and an unsurveyed texel were the same byte: the mask could gate
        // but could not be read. Now its tiles are the classifier's pages (AUDIT_WATER item 2).
        if (!m_gate->SampleAt(q, g)) {
            out.weight = 0.0f;
            return false;
        }
        if (g.weight > 0.0f) out.weight *= g.c[0];
        return out.weight > 0.0f;
    }

private:
    std::string m_name;
    std::shared_ptr<DomainSource> m_layer, m_gate;
};

// ================================================================================================
//  TideSource -- the analytic water level as a GA source. Point domain (stations are scattered),
//  grade 0, one channel, and TIME-VARYING: the same query at two instants is two answers, which
//  is what makes it a separate row on the gantt from the bed that never moves.
//
//  IT SPEAKS MLLW AND ONLY MLLW, because that is what the harmonics were fitted in. An earlier
//  version could also emit NAVD88 by applying each station's published CO-OPS link and dropping
//  any station that lacked one. That looked more rigorous and measured 1.73 m WORSE, for a
//  reason worth keeping written down:
//
//    only 2 of the 20 stations here carry a published NAVD link, and NEWBURYPORT -- the focus,
//    the station whose harmonics actually describe this estuary -- is not one of them. Dropping
//    it left the water level being interpolated from Riverside and Boston. The curve came from
//    the wrong place to avoid guessing a constant.
//
//  So: a source reports what its data says, in the frame its data is in. A missing datum link is
//  not a reason to discard a good tide curve -- it is a reason to make somebody DECLARE the link.
//  That is DeclareDatumLink below, and the declaration carries its provenance so the assumption
//  is readable in the log instead of buried in a class.
// ================================================================================================
class TideSource : public DomainSource {
public:
    // radiusDeg: how far a station's authority reaches. The Merrimack stations sit within a few
    // hundredths of a degree of each other, so this is generous by default and tunable.
    TideSource(const TideModel* tides, double radiusDeg = 0.35, int priority = 5)
        : m_tides(tides),
          m_radius(radiusDeg),
          m_priority(priority),
          m_unit(UnitSpec::Of(Quantity::Length, "MLLW")) {
        if (!m_tides) return;
        for (size_t i = 0; i < m_tides->Count(); ++i) m_use.push_back(i);
        Log("[tide-src] tide.mllw: %u stations, harmonic, m above MLLW", uint32_t(m_use.size()));
    }

    const char* Name() const override { return "tide.mllw"; }
    SourceDomain Domain() const override { return SourceDomain::Point; }
    uint8_t GradeSig() const override { return 1u; }   // kG0: a scalar height
    uint32_t Channels() const override { return 1; }
    int Priority() const override { return m_priority; }
    const UnitSpec& Unit() const override { return m_unit; }
    const char* Cadence() const override { return "per-instant (analytic)"; }
    const char* NodeKind() const override { return "load"; }

    bool SampleAt(const DomainQuery& q, DomainValue& out) const override {
        out.weight = 0.0f;
        if (!m_tides || m_use.empty()) return false;
        // Inverse-distance over the stations in range, in the frame this source speaks.
        double wsum = 0.0, acc = 0.0;
        for (size_t k : m_use) {
            const TideStation& st = m_tides->S(k);
            const double dLon = (q.lon - st.lon) * std::cos(st.lat * 3.14159265358979 / 180.0);
            const double dLat = q.lat - st.lat;
            const double d = std::sqrt(dLon * dLon + dLat * dLat);
            if (d > m_radius) continue;
            const double w = 1.0 / (d * d + 1e-9);
            // Stays in MLLW. Converting here would bury a site constant inside a
            // loader; DeclareDatumLink is where that assertion belongs.
            acc += w * m_tides->Height(k, q.unixT);   // metres above MLLW
            wsum += w;
        }
        if (wsum <= 0.0) return false;
        out.c[0] = static_cast<float>(acc / wsum);
        out.weight = 1.0f;   // a station in range is authoritative about the level, not partial
        return true;
    }

    bool MayCover(double lon0, double lat0, double lon1, double lat1) const override {
        if (!m_tides) return false;
        for (size_t k : m_use) {
            const TideStation& st = m_tides->S(k);
            if (st.lon >= lon0 - m_radius && st.lon <= lon1 + m_radius &&
                st.lat >= lat0 - m_radius && st.lat <= lat1 + m_radius) {
                return true;
            }
        }
        return false;
    }

private:
    const TideModel* m_tides = nullptr;
    double m_radius = 0.35;
    int m_priority = 5;
    UnitSpec m_unit;
    std::vector<size_t> m_use;
};

// ================================================================================================
//  DeclareDatumLink -- an operator states a vertical datum offset and OWNS it.
//
//  This is the escape hatch GaUnits::AcceptsFrom refuses to provide on its own, and the shape of
//  the hatch is the point. The offset cannot be derived, so somebody has to assert it; making
//  that an explicit call with a `provenance` string puts the assertion in the log, beside the
//  number, every run -- rather than leaving it a constant in a class where the next reader has
//  no way to tell it was ever a choice.
//
//  Here the caller passes main's ResolveDatum result, whose own provenance is already printed:
//  "no NAVD link at Newburyport; MLLW - NAVD88 = -1.396 m via NAVD=MSL+0.092 (from Boston)".
//  The estimate was always there. What is new is that it is an EDGE, with a type on both ends.
// ================================================================================================
inline std::shared_ptr<DomainSource> DeclareDatumLink(std::shared_ptr<DomainSource> src,
                                                      double shiftM, const char* toDatum,
                                                      const char* provenance) {
    if (!src) return nullptr;
    const UnitSpec from = src->Unit();
    if (from.quantity != Quantity::Length) {
        Log("[datum-link] %s REFUSED: only a length carries a vertical datum (%s)", src->Name(),
            from.Describe().c_str());
        return nullptr;
    }
    const UnitSpec target = UnitSpec::Of(Quantity::Length, toDatum);
    Log("[datum-link] %s: %s -> %s, %+.4f m -- declared, not derived (%s)", src->Name(),
        from.datum.empty() ? "(none)" : from.datum.c_str(), toDatum, shiftM, provenance);
    return std::make_shared<NormalizedSource>(std::move(src), from.WithDatumShift(shiftM, toDatum),
                                              target);
}

// ================================================================================================
//  BinaryFieldSource -- THE EXTRA COMPOSE STEP. Two fields, one algebraic operation, one result.
//
//  Distinct from a compositor because a compositor RESOLVES DISAGREEMENT between sources that
//  describe the same quantity (by priority, then by weight). This one COMBINES DIFFERENT
//  quantities into a third. Both are composition; only one of them is a blend, and running the
//  tide through the blender would have averaged the water surface with the seabed.
//
//  Coverage multiplies rather than averages: a depth is meaningful only where BOTH the level and
//  the bed are known. That is the strictest of the choices available and the only one that cannot
//  invent a shoreline.
// ================================================================================================
class BinaryFieldSource : public DomainSource {
public:
    enum class Op : uint8_t { Subtract, Add, Min, Max };

    BinaryFieldSource(std::string name, Op op, std::shared_ptr<DomainSource> a,
                      std::shared_ptr<DomainSource> b, int priority = 0)
        : m_name(std::move(name)), m_op(op), m_a(std::move(a)), m_b(std::move(b)),
          m_priority(priority) {
        m_ok = Check();
    }

    bool Valid() const { return m_ok; }
    const char* Name() const override { return m_name.c_str(); }
    SourceDomain Domain() const override { return m_a ? m_a->Domain() : SourceDomain::Raster; }
    uint8_t GradeSig() const override { return m_a ? m_a->GradeSig() : 0; }
    uint32_t Channels() const override { return m_a ? m_a->Channels() : 0; }
    int Priority() const override { return m_priority; }
    const UnitSpec& Unit() const override { return m_unit; }
    const char* NodeKind() const override { return OpName(); }
    const char* Cadence() const override {
        // A node is as live as its liveliest input -- the gantt row inherits the fastest clock.
        const char* ca = m_a ? m_a->Cadence() : "static";
        const char* cb = m_b ? m_b->Cadence() : "static";
        return (std::string(ca) == "static") ? cb : ca;
    }

    size_t InputCount() const override { return 2; }
    const DomainSource* Input(size_t i) const override {
        return i == 0 ? m_a.get() : (i == 1 ? m_b.get() : nullptr);
    }

    bool SampleAt(const DomainQuery& q, DomainValue& out) const override {
        out.weight = 0.0f;
        if (!m_ok) return false;
        DomainValue va, vb;
        if (!m_a->SampleAt(q, va) || va.weight <= 0.0f) return false;
        if (!m_b->SampleAt(q, vb) || vb.weight <= 0.0f) return false;
        const uint32_t n = Channels();
        for (uint32_t c = 0; c < n && c < 4; ++c) {
            switch (m_op) {
                case Op::Subtract: out.c[c] = va.c[c] - vb.c[c]; break;
                case Op::Add:      out.c[c] = va.c[c] + vb.c[c]; break;
                case Op::Min:      out.c[c] = (std::min)(va.c[c], vb.c[c]); break;
                case Op::Max:      out.c[c] = (std::max)(va.c[c], vb.c[c]); break;
            }
        }
        out.weight = va.weight * vb.weight;   // known only where both are known
        return out.weight > 0.0f;
    }

private:
    const char* OpName() const {
        switch (m_op) {
            case Op::Subtract: return "subtract";
            case Op::Add:      return "add";
            case Op::Min:      return "min";
            default:           return "max";
        }
    }

    bool Check() {
        if (!m_a || !m_b) return false;
        const UnitSpec& ua = m_a->Unit();
        const UnitSpec& ub = m_b->Unit();
        if (m_a->Channels() != m_b->Channels() || m_a->GradeSig() != m_b->GradeSig()) {
            Log("[compose] %s REFUSED: %s and %s differ in grade or channel count", m_name.c_str(),
                m_a->Name(), m_b->Name());
            return false;
        }
        std::string why;
        if (!ua.AcceptsFrom(ub, &why)) {
            Log("[compose] %s REFUSED: %s (%s vs %s)", m_name.c_str(), why.c_str(),
                ua.Describe().c_str(), ub.Describe().c_str());
            return false;
        }
        // The result's frame. Add/Subtract of two levelled heights is a THICKNESS: same
        // quantity, no datum, because a difference has no zero anyone chose. Min/Max pick one
        // of the operands and so keep theirs.
        m_unit = ua;
        if ((m_op == Op::Subtract || m_op == Op::Add) && !ua.datum.empty()) {
            m_unit.datum.clear();
            m_unit.raw = "m";   // a thickness: same quantity, no zero anyone chose
        }
        return true;
    }

    std::string m_name;
    Op m_op;
    std::shared_ptr<DomainSource> m_a, m_b;
    int m_priority = 0;
    bool m_ok = false;
    UnitSpec m_unit;
};

// ================================================================================================
//  PrintTree -- the reverse tree AND the gantt, as one indented table. The print is the truth:
//  it walks the live node graph rather than restating a design, so a node wired the wrong way
//  shows up here before it shows up in pixels. Same discipline as GaAst::Print, which does this
//  for the render edges; this is the compose half that had no picture.
// ================================================================================================
inline void PrintTreeNode(const DomainSource* n, int depth, const char* tag) {
    if (!n) return;
    std::string pad;
    for (int i = 0; i < depth; ++i) pad += "  ";
    const char* dom = "?";
    switch (n->Domain()) {
        case SourceDomain::Point:   dom = "point"; break;
        case SourceDomain::Profile: dom = "profile"; break;
        case SourceDomain::Raster:  dom = "raster"; break;
        case SourceDomain::Volume:  dom = "volume"; break;
    }
    std::string label = pad + tag + n->Name();
    if (label.size() > 26) label.resize(26);
    std::string unit = n->Unit().Describe();
    if (unit.size() > 26) unit.resize(26);
    Log("[tree] %-26s %-10s %-7s %-26s %s", label.c_str(), n->NodeKind(), dom, unit.c_str(),
        n->Cadence());
    for (size_t i = 0; i < n->InputCount(); ++i) {
        PrintTreeNode(n->Input(i), depth + 1, "+- ");
    }
}

inline void PrintTree(const char* product, const DomainSource* root) {
    Log("[tree] %s -- inputs converge upward; the last column is the gantt row's clock", product);
    Log("[tree] %-26s %-10s %-7s %-26s %s", "node", "kind", "domain", "unit", "cadence");
    PrintTreeNode(root, 0, "");
}

}  // namespace ga
