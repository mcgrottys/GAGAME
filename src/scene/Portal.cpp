// Portal - the Droste link as a node: the session's build block, verbatim (M12 step 5e).
#include "scene/Portal.h"

#include "compose/Compositor.h"
#include "core/Common.h"
#include "core/Pga.h"
#include "core/Space.h"
#include "scene/GlobeLayer.h"
#include "sim/GlobeModel.h"

#include <algorithm>
#include <cmath>

namespace ga::scene {

std::vector<std::string> Portal::Configure(const Wiring& w) {
    (void)w;   // no device object: the link is arithmetic
    return {};
}

bool Portal::Init(Gpu& gpu) {
    (void)gpu;
    return Build();
}

void Portal::Apply(const PropSet& props) {
    std::string why;
    if (!props.ApplyTo(&m_props, nullptr, &why)) {
        Log("[portal] '%s' apply refused: %s", m_props.name.c_str(), why.c_str());
        return;
    }
    enabled = m_props.enabled;
}

void Portal::Update(const FrameInfo& f) { (void)f; }

void Portal::SetDestination(double latDeg, double lonDeg) {
    m_hasTo = true;
    m_toLat = latDeg;
    m_toLon = lonDeg;
}

bool Portal::Carry(const double destPlanet[3], const double leafPlanet[3], const double east[3],
                   const double up[3], const double north[3], double twistRad,
                   double axisOut[3], double& angleOut, std::string* why) {
    // Both radials in the tangent rows: the frame BuildPortal, its fixed point and the cycle
    // speak. A radial is unit up to rounding; normalised again so the arc is a pure rotation.
    auto inRows = [&](const double v[3], double o[3]) {
        o[0] = east[0] * v[0] + east[1] * v[1] + east[2] * v[2];
        o[1] = up[0] * v[0] + up[1] * v[1] + up[2] * v[2];
        o[2] = north[0] * v[0] + north[1] * v[1] + north[2] * v[2];
        const double n = std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);
        for (int i = 0; i < 3; ++i) o[i] /= n;
    };
    double h[3], l[3];
    inRows(destPlanet, h);
    inRows(leafPlanet, l);
    // THE SHORTEST ARC, as a rotor: R = (1 + l h) / |1 + l h| in the geometric product -- in the
    // motor's layout (w, x, y, z) that is (1 + h.l, h x l) normalised, which QRotate turns h onto
    // l with (scenetest pins it). At the antipode 1 + h.l vanishes with h x l: no unique arc.
    const double d = h[0] * l[0] + h[1] * l[1] + h[2] * l[2];
    double qa[4] = {1.0 + d, h[1] * l[2] - h[2] * l[1], h[2] * l[0] - h[0] * l[2],
                    h[0] * l[1] - h[1] * l[0]};
    const double qn = std::sqrt(qa[0] * qa[0] + qa[1] * qa[1] + qa[2] * qa[2] + qa[3] * qa[3]);
    if (!(qn > 1e-9)) {
        if (why) *why = "the destination is the leaf's antipode: every great circle carries it there";
        return false;
    }
    for (double& c : qa) c /= qn;
    // THE TWIST after it: the same rotor about north BuildPortal always built (Motor::Rotation's
    // real part), and the composition is the rotor product -- the arc first, then the twist.
    const double ht = 0.5 * twistRad;
    const double qt[4] = {std::cos(ht), 0.0, 0.0, std::sin(ht)};
    double q[4];
    Motor::QMul(qt, qa, q);
    // One rotation of at most half a turn: q and -q are the same map, and the principal branch
    // is the one whose angle lies in [0, pi] (Space.h's Pow takes the same branch).
    if (q[0] < 0.0) {
        for (double& c : q) c = -c;
    }
    Placement::AxisAngle(q, axisOut, angleOut);
    return true;
}
void Portal::Record(const ViewContext& v) { (void)v; }

std::vector<std::string> Portal::Configure(const Observers& o) {
    m_o = o;
    std::vector<std::string> missing;
    if (!o.east || !o.up || !o.north) missing.push_back("rows");
    if (!(o.planetR > 0.0)) missing.push_back("planetR");
    if (!o.compositor || o.hgtCh < 0) missing.push_back("ground");
    if (!o.tangent) missing.push_back("tangent");
    if (!missing.empty()) {
        std::string list;
        for (const std::string& m : missing) list += (list.empty() ? "" : ", ") + m;
        Log("[portal] '%s' NOT wired: %s", m_props.name.c_str(), list.c_str());
    }
    return missing;
}

// ---- M10: THE DROSTE LINK (src/core/Droste.h). The root's ADDRESS, hung as a leaf:
// the quadtree leaf at (drosteLat, drosteLon, drosteLevel) gets the root as its child.
// What that means in space follows from the address alone -- the globe's diameter is
// the leaf's span (x fill), it rests on the composed ground at the leaf's centre, and
// the fixed point where the tower converges is then FORCED by the similarity. The link
// is one Cl(4,1) versor; everything per frame is its closed form.
bool Portal::Build() {
    m_link = droste::Portal{};
    m_cycle = Space{};
    const PortalProps& portalDecl = m_props;
    const bool portalOn = enabled && m_props.enabled;
    if (!(portalOn && m_o.globe && !m_o.marsMode)) return false;
    if (!m_o.east || !m_o.up || !m_o.north || !m_o.tangent) return false;
    const double* east0 = m_o.east;
    const double* oDir = m_o.up;
    const double* north0 = m_o.north;
    const double planetR = m_o.planetR;
    const int hgtCh = m_o.hgtCh;
    auto& portal = m_link;
    auto& drosteLeaf = m_cycle;
    double pd[3];
    GlobeModel::LatLonDir(portalDecl.lat, portalDecl.lon, pd);
    int lf = 0;
    uint32_t lix = 0, liy = 0;
    GlobeLayer::LeafOf(pd, portalDecl.level, lf, lix, liy);
    double ld[3];
    GlobeLayer::LeafDir(lf, portalDecl.level, lix, liy, ld);
    const double latC = std::asin(std::clamp(ld[1], -1.0, 1.0));
    const double lonC = std::atan2(ld[2], ld[0]);
    const double spanM =
        (3.14159265358979 / 2.0) * planetR / double(1u << portalDecl.level);
    const double ground =
        (hgtCh >= 0 && m_o.compositor)
            ? double(m_o.compositor->SampleHeightStack(hgtCh, latC, lonC, spanM * 0.25))
            : 0.0;
    const double twist = portalDecl.twistDeg * 3.14159265358979 / 180.0;
    const double axisN[3] = {0.0, 0.0, 1.0};   // the anchor's north, in the one frame
    // THE DESTINATION: the arc that carries it onto this leaf's place, then the twist, as ONE
    // rotation (Carry). Without a destination the numbers are the twist about north, as always.
    double axisQ[3] = {axisN[0], axisN[1], axisN[2]};
    double turnQ = twist;
    double toDir[3] = {0.0, 0.0, 0.0};
    if (m_hasTo) {
        GlobeModel::LatLonDir(m_toLat, m_toLon, toDir);
        std::string why;
        if (!Carry(toDir, ld, east0, oDir, north0, twist, axisQ, turnQ, &why)) {
            Log("[droste] portal '%s' destination %.5f N %.5f E refused: %s -- no tower",
                portalDecl.name.c_str(), m_toLat, m_toLon, why.c_str());
            return false;
        }
    }
    portal = droste::BuildPortal(lf, portalDecl.level, lix, liy, ld, east0, oDir, north0,
                                 planetR, ground, portalDecl.fill, axisQ, turnQ);
    Log("[droste] the root hangs at leaf (face %d, level %d, %u, %u) = %.5f N %.5f E: "
        "globe radius %.2f m (s %.4e, %.2f decades a level) resting on %.2f m, centre "
        "(%.2f, %.2f, %.2f), twist %.1f deg about north, fixed point (%.4f, %.4f, %.4f)",
        lf, portalDecl.level, lix, liy, latC * 57.29577951308232,
        lonC * 57.29577951308232, portal.radius, portal.s, -std::log10(portal.s), ground,
        portal.centre[0], portal.centre[1], portal.centre[2], portalDecl.twistDeg,
        portal.p[0], portal.p[1], portal.p[2]);
    if (m_hasTo) {
        const double arc = std::acos(std::clamp(
            toDir[0] * ld[0] + toDir[1] * ld[1] + toDir[2] * ld[2], -1.0, 1.0));
        Log("[droste] destination %.5f N %.5f E: carried %.3f deg of arc onto the leaf's place, then "
            "the twist -- one rotation of %.6f deg a level about (%.6f, %.6f, %.6f); an eye that "
            "dives in there re-roots at the destination",
            m_toLat, m_toLon, arc * 57.29577951308232, portal.twist * 57.29577951308232,
            portal.axis[0], portal.axis[1], portal.axis[2]);
    }
    // M12 step 4d: THE CYCLE, DECLARED. The root's tangent frame hung under its own leaf by
    // the portal's similarity, as a Space: Level(k) = S^k. Similar(p, s, axis, twist) takes
    // the twist in RADIANS, as Portal::twist holds it (radians per level); p, s and axis are
    // the portal's own (BuildPortal resolved the address into them, and stays the builder).
    drosteLeaf = Space::Cycle("droste.leaf", *m_o.tangent,
                              Placement::Similar(portal.p, portal.s, portal.axis, portal.twist));
    Log("[space] droste.leaf: %s hung under its own leaf, S = Similar(p, s %.4e, axis (%.0f, "
        "%.0f, %.0f), twist %.17g rad); Level(k) = S^k, unit length %.4g m",
        m_o.tangent->name.c_str(), portal.s, portal.axis[0], portal.axis[1], portal.axis[2],
        portal.twist, drosteLeaf.unitM);
    return true;
}

}  // namespace ga::scene
