// Gateway - the cuboid gate: one motor between two places on one planet (see Gateway.h).
#include "scene/Gateway.h"

#include "core/Common.h"
#include "scene/Pose.h"

#include <algorithm>
#include <cmath>

namespace ga::scene {

bool Gateway::Build(const Space& planet, const Space& source, double planetR, double x, double alt,
                 double z, double azDeg) {
    m_valid = false;
    m_source = &source;
    const double kDeg = 3.14159265358979323846 / 180.0;
    if (!(std::fabs(m_props.toLat) < 89.0) || !std::isfinite(m_props.toLon) ||
        !(m_props.size[0] > 0.0 && m_props.size[1] > 0.0 && m_props.size[2] > 0.0)) {
        Log("[gate] '%s' refused: a destination needs a place off the poles and the box a size",
            m_props.name.c_str());
        return false;
    }
    const double o[3] = {0.0, 0.0, 0.0}, upY[3] = {0.0, 1.0, 0.0};
    m_entry = Motor::Translation(x, alt, z) * Motor::Rotation(o, upY, azDeg * kDeg);
    m_entryInv = m_entry.Inverse();
    m_exit = Motor::Rotation(o, upY, m_props.toAz * kDeg);
    m_carry = m_exit * m_entryInv;

    // The destination: a tangent frame of the SAME planet, built by the rule the root's is.
    const PoseFrame fr = FrameFromAnchor(m_props.toLat, m_props.toLon, planetR);
    m_dest = Space{};
    m_dest.name = "gate." + m_props.name;
    m_dest.unitM = planetR;
    m_dest.extentM = 2.0 * planetR;
    m_dest.parent = &planet;
    const double anchor[3] = {fr.up[0] * planetR, fr.up[1] * planetR, fr.up[2] * planetR};
    m_dest.link = Placement::Frame(fr.east, fr.up, fr.north, anchor);
    std::string why;
    if (!fr.valid || !m_dest.Declare(&why)) {
        Log("[gate] '%s' refused: the destination frame did not declare (%s)", m_props.name.c_str(),
            why.c_str());
        return false;
    }
    const Placement dIn = m_dest.To(source);
    if (!(dIn.s > 0.0)) {
        Log("[gate] '%s' refused: the destination is not a proper placement in the source (s %g)",
            m_props.name.c_str(), dIn.s);
        return false;
    }
    m_destInSource = dIn.ToMotor();

    m_chart = Space::Anchor{};
    m_chart.latDeg = m_props.toLat;
    m_chart.lonDeg = m_props.toLon;
    m_chart.mPerLat = 110574.0;
    m_chart.mPerLon = 111320.0 * std::cos(m_props.toLat * kDeg);
    m_chart.linear = true;
    // M13 step 2: and the EXACT map beside it -- the destination frame's own rows (the same ones
    // the space's placement was built from) and the planet's radius, so a carried hull's water is
    // read at the place, not at the linear chart's drift from this anchor.
    for (int i = 0; i < 3; ++i) {
        m_chart.east[i] = fr.east[i];
        m_chart.up[i] = fr.up[i];
        m_chart.north[i] = fr.north[i];
    }
    m_chart.planetR = planetR;

    double ex = 0.0, ey = 0.0, ez = 0.0;
    m_destInSource.TransformPoint(ex, ey, ez);
    Log("[gate] '%s': a %.0f x %.0f x %.0f m box at (%.1f, %.1f, %.1f) heading %.1f carries to "
        "%.5f N %.5f E heading %.1f -- %.0f km away in the source frame",
        m_props.name.c_str(), m_props.size[0], m_props.size[1], m_props.size[2], x, alt, z, azDeg,
        m_props.toLat, m_props.toLon, m_props.toAz,
        std::sqrt(ex * ex + ey * ey + ez * ez) / 1000.0);
    m_valid = true;
    return true;
}

bool Gateway::SeenThrough(const double eye[3], const double p[3]) const {
    if (!m_valid) return false;
    // Both ends in the box's own frame, the segment e + t (f - e), t in [0, 1].
    double ex = eye[0], ey = eye[1], ez = eye[2];
    double fx = p[0], fy = p[1], fz = p[2];
    m_entryInv.TransformPoint(ex, ey, ez);
    m_entryInv.TransformPoint(fx, fy, fz);
    const double e[3] = {ex, ey, ez};
    const double d[3] = {fx - ex, fy - ey, fz - ez};
    double tEnter = -1e300, tExit = 1e300;
    for (int a = 0; a < 3; ++a) {
        const double h = 0.5 * m_props.size[a];
        const double da = (std::fabs(d[a]) < 1e-12) ? 1e-12 : d[a];
        const double t1 = (-h - e[a]) / da, t2 = (h - e[a]) / da;
        tEnter = (std::max)(tEnter, (std::min)(t1, t2));
        tExit = (std::min)(tExit, (std::max)(t1, t2));
    }
    return tEnter <= tExit && tExit >= 0.0 && tEnter <= 1.0;
}

bool Gateway::Inside(double px, double py, double pz) const {
    if (!m_valid) return false;
    m_entryInv.TransformPoint(px, py, pz);
    return std::fabs(px) <= 0.5 * m_props.size[0] && std::fabs(py) <= 0.5 * m_props.size[1] &&
           std::fabs(pz) <= 0.5 * m_props.size[2];
}

}  // namespace ga::scene
