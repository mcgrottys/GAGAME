// Gateway - the cuboid gate: one motor between two places on one planet (see Gateway.h).
#include "scene/Gateway.h"

#include "core/Common.h"
#include "scene/Pose.h"

#include <algorithm>
#include <cmath>

namespace ga::scene {

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;
}

bool Place::Build(const Space& planet, double planetR, double lat, double lon,
                  const std::string& name, std::string* why) {
    if (!(std::fabs(lat) < 89.0) || !std::isfinite(lon)) {
        if (why) *why = "a place needs a latitude off the poles and a finite longitude";
        return false;
    }
    // The tangent frame there, built by the rule the root's is, hung under the planet.
    const PoseFrame fr = FrameFromAnchor(lat, lon, planetR);
    space = Space{};
    space.name = name;
    space.unitM = planetR;
    space.extentM = 2.0 * planetR;
    space.parent = &planet;
    const double anchor[3] = {fr.up[0] * planetR, fr.up[1] * planetR, fr.up[2] * planetR};
    space.link = Placement::Frame(fr.east, fr.up, fr.north, anchor);
    if (!fr.valid) {
        if (why) *why = "no tangent frame at the place";
        return false;
    }
    if (!space.Declare(why)) return false;
    chart = Space::Anchor{};
    chart.latDeg = lat;
    chart.lonDeg = lon;
    chart.mPerLat = 110574.0;
    chart.mPerLon = 111320.0 * std::cos(lat * kDeg);
    chart.linear = true;
    // M13 step 2: and the EXACT map beside it -- the frame's own rows (the same ones the space's
    // placement was built from) and the planet's radius, so a carried hull's water is read at the
    // place, not at the linear chart's drift from this anchor.
    for (int i = 0; i < 3; ++i) {
        chart.east[i] = fr.east[i];
        chart.up[i] = fr.up[i];
        chart.north[i] = fr.north[i];
    }
    chart.planetR = planetR;
    latDeg = lat;
    lonDeg = lon;
    return true;
}

bool Gateway::Build(const Space& source, const Space& dest, const Space::Anchor& destChart,
                    const Space& root, double x, double alt, double z, double azDeg,
                    const double exit[3]) {
    m_valid = false;
    m_source = &source;
    m_dest = &dest;
    m_chart = &destChart;
    if (!(m_props.size[0] > 0.0 && m_props.size[1] > 0.0 && m_props.size[2] > 0.0)) {
        Log("[gate] '%s' refused: the box needs a size", m_props.name.c_str());
        return false;
    }
    const double o[3] = {0.0, 0.0, 0.0}, upY[3] = {0.0, 1.0, 0.0};
    m_entry = Motor::Translation(x, alt, z) * Motor::Rotation(o, upY, azDeg * kDeg);
    m_entryInv = m_entry.Inverse();
    m_exit = Motor::Translation(exit[0], exit[1], exit[2]) *
             Motor::Rotation(o, upY, m_props.toAz * kDeg);
    m_carry = m_exit * m_entryInv;

    const Placement dIn = dest.To(source);
    const Placement sRoot = source.To(root);
    const Placement dRoot = dest.To(root);
    if (!(dIn.s > 0.0) || !(sRoot.s > 0.0) || !(dRoot.s > 0.0)) {
        Log("[gate] '%s' refused: its two places are not proper placements of one planet "
            "(s %g, %g, %g)", m_props.name.c_str(), dIn.s, sRoot.s, dRoot.s);
        return false;
    }
    m_destInSource = dIn.ToMotor();
    const Motor srcInRoot = sRoot.ToMotor();
    m_destInRoot = dRoot.ToMotor();
    m_entryInRoot = srcInRoot * m_entry;
    m_entryInRootInv = m_entryInRoot.Inverse();
    m_window = m_destInRoot * m_carry * srcInRoot.Inverse();

    double bx = 0.0, by = 0.0, bz = 0.0;
    m_entryInRoot.TransformPoint(bx, by, bz);
    double ex = exit[0], ey = exit[1], ez = exit[2];
    m_destInRoot.TransformPoint(ex, ey, ez);
    Log("[gate] '%s': a %.0f x %.0f x %.0f m box at (%.1f, %.1f, %.1f) in %s heading %.1f carries "
        "to %.5f N %.5f E -- (%.1f, %.1f, %.1f) in %s, heading %.1f -- %.0f km away",
        m_props.name.c_str(), m_props.size[0], m_props.size[1], m_props.size[2], x, alt, z,
        source.name.c_str(), azDeg, destChart.latDeg, destChart.lonDeg, exit[0], exit[1], exit[2],
        dest.name.c_str(), m_props.toAz,
        std::sqrt((ex - bx) * (ex - bx) + (ey - by) * (ey - by) + (ez - bz) * (ez - bz)) / 1000.0);
    m_valid = true;
    return true;
}

bool Gateway::Build(const Space& planet, const Space& source, double planetR, double x, double alt,
                    double z, double azDeg) {
    m_valid = false;
    std::string why;
    if (!m_ownDest.Build(planet, planetR, m_props.toLat, m_props.toLon, "gate." + m_props.name,
                         &why)) {
        Log("[gate] '%s' refused: the destination is not a place (%s)", m_props.name.c_str(),
            why.c_str());
        return false;
    }
    const double exit[3] = {0.0, 0.0, 0.0};
    return Build(source, m_ownDest.space, m_ownDest.chart, source, x, alt, z, azDeg, exit);
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

bool Gateway::SeenThroughFrom(const Motor& boxInRoot, const double eye[3], const double p[3],
                              double tStart, double* tIn) const {
    if (!m_valid) return false;
    const Motor inv = boxInRoot.Inverse();
    double ex = eye[0], ey = eye[1], ez = eye[2];
    double fx = p[0], fy = p[1], fz = p[2];
    inv.TransformPoint(ex, ey, ez);
    inv.TransformPoint(fx, fy, fz);
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
    // The ray is in this window's world only from where it entered the last one.
    tEnter = (std::max)(tEnter, tStart);
    if (tIn) *tIn = tEnter;
    return tEnter <= tExit && tEnter <= 1.0;
}

bool Gateway::Inside(double px, double py, double pz) const {
    if (!m_valid) return false;
    m_entryInv.TransformPoint(px, py, pz);
    return std::fabs(px) <= 0.5 * m_props.size[0] && std::fabs(py) <= 0.5 * m_props.size[1] &&
           std::fabs(pz) <= 0.5 * m_props.size[2];
}

bool Gateway::InsideRoot(double px, double py, double pz) const {
    if (!m_valid) return false;
    m_entryInRootInv.TransformPoint(px, py, pz);
    return std::fabs(px) <= 0.5 * m_props.size[0] && std::fabs(py) <= 0.5 * m_props.size[1] &&
           std::fabs(pz) <= 0.5 * m_props.size[2];
}

// ---- THE VIEW THROUGH THE GATES ------------------------------------------------------------------
namespace {

constexpr double kInf = 1.0e300;
constexpr double kFront = 1.0e-3;   // m: a corner nearer the eye's plane than this is not in front

struct Candidate {
    Motor boxInRoot;
    double rect[4] = {0.0, 0.0, 0.0, 0.0};
    double zNear = 0.0;
    double zBox = kInf;   // the box's nearest corner along the view axis: the nearest box wins
    double dist = kInf;   // its centre's distance from the eye
    bool visible = false;
};

double Dot(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// Where a box stands for the true eye once the view has been pulled back through the windows so
// far, and which of the view's rays still reach it.
Candidate Evaluate(const Gateway& g, const Motor& pull, const ViewCone& v, const double rect[4],
                   double zNear, bool open) {
    Candidate c;
    c.boxInRoot = pull * g.EntryInRoot();
    double cc[3] = {0.0, 0.0, 0.0};
    c.boxInRoot.TransformPoint(cc[0], cc[1], cc[2]);
    const double dc[3] = {cc[0] - v.eye[0], cc[1] - v.eye[1], cc[2] - v.eye[2]};
    c.dist = std::sqrt(Dot(dc, dc));
    const double* sz = g.Declared().size;
    double zMin = kInf, zMax = -kInf, x0 = kInf, x1 = -kInf, y0 = kInf, y1 = -kInf;
    bool straddles = false;
    for (int k = 0; k < 8; ++k) {
        double p[3] = {((k & 1) ? 0.5 : -0.5) * sz[0], ((k & 2) ? 0.5 : -0.5) * sz[1],
                       ((k & 4) ? 0.5 : -0.5) * sz[2]};
        c.boxInRoot.TransformPoint(p[0], p[1], p[2]);
        const double rel[3] = {p[0] - v.eye[0], p[1] - v.eye[1], p[2] - v.eye[2]};
        const double vz = Dot(rel, v.fwd);
        zMin = (std::min)(zMin, vz);
        zMax = (std::max)(zMax, vz);
        if (vz <= kFront) {
            straddles = true;
            continue;
        }
        const double tx = Dot(rel, v.right) / vz, ty = Dot(rel, v.up) / vz;
        x0 = (std::min)(x0, tx);
        x1 = (std::max)(x1, tx);
        y0 = (std::min)(y0, ty);
        y1 = (std::max)(y1, ty);
    }
    c.zBox = zMin;
    // Wholly behind the eye, or wholly nearer than the windows it would have to be seen through.
    if (!open || zMax <= (std::max)(zNear, kFront)) return c;
    if (straddles) {
        // The box reaches round the eye's plane (the eye is in it, or beside it): its rays are not
        // bounded by a rectangle, so the view so far is the bound -- conservative, never short.
        for (int i = 0; i < 4; ++i) c.rect[i] = rect[i];
        c.zNear = zNear;
        c.visible = true;
        return c;
    }
    c.rect[0] = (std::max)(rect[0], x0);
    c.rect[1] = (std::min)(rect[1], x1);
    c.rect[2] = (std::max)(rect[2], y0);
    c.rect[3] = (std::min)(rect[3], y1);
    if (!(c.rect[0] < c.rect[1] && c.rect[2] < c.rect[3])) return c;
    c.zNear = (std::max)(zNear, zMin);
    c.visible = true;
    return c;
}

}  // namespace

std::vector<WindowLink> WindowChain(const std::vector<const Gateway*>& gates,
                                    const std::vector<const Gateway*>& forced,
                                    const ViewCone& view, int maxDepth, double reachM) {
    std::vector<WindowLink> out;
    Motor carry = Motor::Identity();
    Motor pull = Motor::Identity();
    double rect[4] = {-view.tanX, view.tanX, -view.tanY, view.tanY};
    double zNear = 0.0;
    bool open = true;   // the view so far still has rays
    for (int depth = 0; depth < maxDepth; ++depth) {
        const Gateway* pick = nullptr;
        Candidate best;
        if (depth < static_cast<int>(forced.size())) {
            pick = forced[size_t(depth)];
            if (!pick || !pick->Valid()) break;
            best = Evaluate(*pick, pull, view, rect, zNear, open);
        } else {
            if (!open) break;
            for (const Gateway* g : gates) {
                if (!g || !g->Valid()) continue;
                const Candidate c = Evaluate(*g, pull, view, rect, zNear, open);
                if (!c.visible || c.dist > reachM) continue;
                // THE SCREEN ENDS THE CORRIDOR: a window narrower than two pixels shows nothing.
                if ((std::min)(c.rect[1] - c.rect[0], c.rect[3] - c.rect[2]) < 2.0 * view.pixTan) {
                    continue;
                }
                if (pick && c.zBox >= best.zBox) continue;
                pick = g;
                best = c;
            }
            if (!pick) break;
        }
        WindowLink L;
        L.gate = pick;
        L.boxInRoot = best.boxInRoot;
        L.carry = pick->Window() * carry;   // the earlier windows first, then this one
        L.pull = L.carry.Inverse();
        L.visible = best.visible;
        if (best.visible) {
            for (int i = 0; i < 4; ++i) L.rect[i] = best.rect[i];
            L.zNear = best.zNear;
            for (int i = 0; i < 4; ++i) rect[i] = best.rect[i];
            zNear = best.zNear;
        } else {
            open = false;   // a window no ray reaches: nothing is seen past it
        }
        carry = L.carry;
        pull = L.pull;
        out.push_back(L);
    }
    return out;
}

int ChainDepth(const std::vector<WindowLink>& chain, const double eye[3], const double p[3]) {
    double t = 0.0;
    int k = 0;
    for (; k < static_cast<int>(chain.size()); ++k) {
        const WindowLink& L = chain[size_t(k)];
        double tIn = 0.0;
        if (!L.gate || !L.gate->SeenThroughFrom(L.boxInRoot, eye, p, t, &tIn)) break;
        t = tIn;
    }
    return k;
}

void LinkPlanes(const WindowLink& L, const ViewCone& v, double planes[5][4]) {
    auto set = [&](int i, const double n[3], double d) {
        const double len = std::sqrt(Dot(n, n));
        const double s = (len > 0.0) ? 1.0 / len : 0.0;
        planes[i][0] = n[0] * s;
        planes[i][1] = n[1] * s;
        planes[i][2] = n[2] * s;
        planes[i][3] = d * s;
    };
    if (!L.visible) {
        // No ray reaches it: a near plane past any distance culls the whole walk.
        for (int i = 0; i < 5; ++i) set(i, v.fwd, -1.0e30);
        return;
    }
    double n[3];
    for (int i = 0; i < 3; ++i) n[i] = v.right[i] - L.rect[0] * v.fwd[i];   // tan x >= x0
    set(0, n, 0.0);
    for (int i = 0; i < 3; ++i) n[i] = L.rect[1] * v.fwd[i] - v.right[i];   // tan x <= x1
    set(1, n, 0.0);
    for (int i = 0; i < 3; ++i) n[i] = v.up[i] - L.rect[2] * v.fwd[i];      // tan y >= y0
    set(2, n, 0.0);
    for (int i = 0; i < 3; ++i) n[i] = L.rect[3] * v.fwd[i] - v.up[i];      // tan y <= y1
    set(3, n, 0.0);
    set(4, v.fwd, -L.zNear);                                                // depth >= zNear
}

bool LinkHole(const WindowLink& L, const ViewCone& v, double planes[5][4]) {
    if (!L.gate || !L.visible) return false;
    // The eye in the box's own frame, and the face it sees best: outside that face's slab, the
    // largest area x distance / range^3 (the solid angle, to first order).
    double e[3] = {v.eye[0], v.eye[1], v.eye[2]};
    L.boxInRoot.Inverse().TransformPoint(e[0], e[1], e[2]);
    const double* sz = L.gate->Declared().size;
    const double h[3] = {0.5 * sz[0], 0.5 * sz[1], 0.5 * sz[2]};
    int a = -1;
    double best = 0.0, s = 1.0;
    for (int k = 0; k < 3; ++k) {
        if (std::fabs(e[k]) <= h[k]) continue;
        const int b = (k + 1) % 3, c = (k + 2) % 3;
        const double sk = (e[k] > 0.0) ? 1.0 : -1.0;
        const double off = e[k] - sk * h[k];
        const double rng2 = off * off + e[b] * e[b] + e[c] * e[c];
        const double w = 4.0 * h[b] * h[c] * std::fabs(off) / (rng2 * std::sqrt(rng2));
        if (w > best) {
            best = w;
            a = k;
            s = sk;
        }
    }
    if (a < 0) return false;   // the eye is in the box: every ray already starts in the window
    const int b = (a + 1) % 3, c = (a + 2) % 3;
    // The face's corners in order round it, and its centre -- true frame, eye-relative.
    const double sb[4] = {-1.0, 1.0, 1.0, -1.0}, sc[4] = {-1.0, -1.0, 1.0, 1.0};
    double P[4][3], C[3];
    for (int i = 0; i < 4; ++i) {
        double q[3] = {0.0, 0.0, 0.0};
        q[a] = s * h[a];
        q[b] = sb[i] * h[b];
        q[c] = sc[i] * h[c];
        L.boxInRoot.TransformPoint(q[0], q[1], q[2]);
        for (int j = 0; j < 3; ++j) P[i][j] = q[j] - v.eye[j];
    }
    {
        double q[3] = {0.0, 0.0, 0.0};
        q[a] = s * h[a];
        L.boxInRoot.TransformPoint(q[0], q[1], q[2]);
        for (int j = 0; j < 3; ++j) C[j] = q[j] - v.eye[j];
    }
    auto put = [&](int i, double n[3], double d) {
        const double len = std::sqrt(Dot(n, n));
        if (!(len > 0.0)) return false;
        for (int j = 0; j < 3; ++j) planes[i][j] = n[j] / len;
        planes[i][3] = d / len;
        return true;
    };
    // The four wedge planes through the eye and each edge, the face's centre on their inside.
    for (int i = 0; i < 4; ++i) {
        const double* p = P[i];
        const double* q = P[(i + 1) % 4];
        double n[3] = {p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0]};
        if (Dot(n, C) < 0.0) {
            for (double& x : n) x = -x;
        }
        if (!put(i, n, 0.0)) return false;
    }
    // The face's own plane, facing away from the eye: beyond it is inside.
    double ax[3] = {0.0, 0.0, 0.0};
    ax[a] = 1.0;
    L.boxInRoot.TransformDir(ax[0], ax[1], ax[2]);
    double n[3] = {-s * ax[0], -s * ax[1], -s * ax[2]};
    return put(4, n, -Dot(n, C));
}

}  // namespace ga::scene
