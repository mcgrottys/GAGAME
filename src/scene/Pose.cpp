// Pose - the session's pose maps, bodies verbatim from FrameLoop::Session (M12 step 5a).
#include "scene/Pose.h"

#include "render/Camera.h"
#include "sim/GlobeModel.h"

#include <algorithm>
#include <cmath>

namespace ga::scene {

Motor FromCamera(const Camera& c) {
    return Motor::Translation(c.px, c.py, c.pz) * c.rot;
}

void ToCamera(const Motor& m, Camera& c) {
    double px = 0, py = 0, pz = 0;
    m.TransformPoint(px, py, pz);
    double f[3] = {1, 0, 0};
    m.TransformDir(f[0], f[1], f[2]);
    c.px = px;
    c.py = py;
    c.pz = pz;
    c.Aim(f);   // the roll a slerp carried is taken off against the camera's up (horizon level)
}

namespace {

constexpr double kRad = 3.14159265358979323846 / 180.0;

// The site's frame on the planet: up the radial, east d(dir)/dlon, north east x up (this planet
// frame's own order: FrameFromAnchor's derivation).
void SiteFrame(double lat, double lon, double up[3], double east[3], double north[3]) {
    GlobeModel::LatLonDir(lat, lon, up);
    const double yl = std::sqrt(up[0] * up[0] + up[2] * up[2]);
    if (yl > 1e-12) {
        east[0] = -up[2] / yl; east[1] = 0.0; east[2] = up[0] / yl;
    } else {   // at a pole every direction is south (or north): take the meridian of 0
        east[0] = 0.0; east[1] = 0.0; east[2] = 1.0;
    }
    north[0] = east[1] * up[2] - east[2] * up[1];
    north[1] = east[2] * up[0] - east[0] * up[2];
    north[2] = east[0] * up[1] - east[1] * up[0];
}

// A camera standing at the pose a motor names, its attitude the motor's rotor exactly (no level).
Camera CameraOf(const Motor& m, const double up[3]) {
    Camera c;
    double f[3] = {1, 0, 0}, u[3] = {0, 1, 0};
    m.TransformPoint(c.px, c.py, c.pz);
    m.TransformDir(f[0], f[1], f[2]);
    m.TransformDir(u[0], u[1], u[2]);
    c.Orient(f, u);
    c.SetUp(up);
    return c;
}

// The rows' map of a direction, planet -> flat (east, up, north as rows) and back (as columns).
void ToFlat(const double e[3], const double o[3], const double n[3], const double d[3], double out[3]) {
    out[0] = d[0] * e[0] + d[1] * e[1] + d[2] * e[2];
    out[1] = d[0] * o[0] + d[1] * o[1] + d[2] * o[2];
    out[2] = d[0] * n[0] + d[1] * n[1] + d[2] * n[2];
}
void ToPlanet(const double e[3], const double o[3], const double n[3], const double d[3], double out[3]) {
    for (int i = 0; i < 3; ++i) out[i] = e[i] * d[0] + o[i] * d[1] + n[i] * d[2];
}

// The attitude and the up carried through a map of directions: the frame changes, the pose does not.
void CarryAttitude(const Camera& from, Camera& to, void (*map)(const double*, const double*, const double*,
                                                               const double*, double*),
                   const double e[3], const double o[3], const double n[3]) {
    double f[3] = {1, 0, 0}, u[3] = {0, 1, 0}, mf[3], mu[3], mup[3];
    from.rot.TransformDir(f[0], f[1], f[2]);
    from.rot.TransformDir(u[0], u[1], u[2]);
    map(e, o, n, f, mf);
    map(e, o, n, u, mu);
    map(e, o, n, from.upRef, mup);
    to.Orient(mf, mu);
    to.SetUp(mup);
}

}  // namespace

Camera LatLonPose(double lat, double lon, double altM, bool lookAt, double tLat, double tLon,
                  double headingDeg, double tiltDeg, double rollDeg, double rangeM, double planetR) {
    double up[3], east[3], north[3];
    SiteFrame(lat, lon, up, east, north);
    if (lookAt) {
        Camera c;
        const double rr = planetR + altM;
        c.px = up[0] * rr;
        c.py = up[1] * rr;
        c.pz = up[2] * rr;
        c.SetUp(up);
        double t[3];
        GlobeModel::LatLonDir(tLat, tLon, t);
        c.LookAt(t[0] * planetR, t[1] * planetR, t[2] * planetR);
        return c;
    }
    // R_site: the own forward onto the site's down, the own up onto its north.
    Camera site;
    const double down[3] = {-up[0], -up[1], -up[2]};
    site.Orient(down, north);
    const double ox[3] = {1, 0, 0}, oz[3] = {0, 0, 1}, nx[3] = {-1, 0, 0}, origin[3] = {0, 0, 0};
    // In the own axes at R_site the site's up is -x: heading turns about it, clockwise from north
    // seen from above; tilt about the own z (the right is -z), lifting the forward toward the up
    // the heading left; roll about the forward.
    const Motor M = Motor::Translation(up[0] * (planetR + altM), up[1] * (planetR + altM), up[2] * (planetR + altM)) *
                    site.rot * Motor::Rotation(origin, nx, headingDeg * kRad) *
                    Motor::Rotation(origin, oz, tiltDeg * kRad) * Motor::Rotation(origin, ox, rollDeg * kRad) *
                    Motor::Translation(-rangeM, 0.0, 0.0);
    double p[3] = {0, 0, 0};
    M.TransformPoint(p[0], p[1], p[2]);
    const double pl = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    const double eyeUp[3] = {p[0] / pl, p[1] / pl, p[2] / pl};
    return CameraOf(M, eyeUp);
}

Camera OrbitPose(double lat, double lon, double altM, double tLat, double tLon, double planetR) {
    return LatLonPose(lat, lon, altM, true, tLat, tLon, 0, 0, 0, 0, planetR);
}

Camera GlobeCamera(double lat, double lon, double altM, double planetR) {
    return LatLonPose(lat, lon, altM, false, 0, 0, 0, 0, 0, 0, planetR);
}

Camera PlanetToFlatPose(const Camera& g, const double east0[3], const double oDir[3],
                        const double north0[3], double planetR) {
    Camera f = g;
    const double p[3] = {g.px, g.py, g.pz};
    f.px = p[0] * east0[0] + p[1] * east0[1] + p[2] * east0[2];
    f.py = p[0] * oDir[0] + p[1] * oDir[1] + p[2] * oDir[2] - planetR;
    f.pz = p[0] * north0[0] + p[1] * north0[1] + p[2] * north0[2];
    CarryAttitude(g, f, ToFlat, east0, oDir, north0);
    return f;
}

Camera FlatToPlanetPose(const Camera& f, const double east0[3], const double oDir[3],
                        const double north0[3], double planetR) {
    Camera g = f;
    const double r = planetR + f.py;
    g.px = oDir[0] * r + east0[0] * f.px + north0[0] * f.pz;
    g.py = oDir[1] * r + east0[1] * f.px + north0[1] * f.pz;
    g.pz = oDir[2] * r + east0[2] * f.px + north0[2] * f.pz;
    CarryAttitude(f, g, ToPlanet, east0, oDir, north0);
    return g;
}

PoseFrame FrameFromAnchor(double latDeg, double lonDeg, double planetR) {
    PoseFrame fr;
    fr.planetR = planetR;
    double* oDir = fr.up;
    double* east0 = fr.east;
    double* north0 = fr.north;
    GlobeModel::LatLonDir(latDeg, lonDeg, oDir);
    const double yl = std::sqrt(oDir[0] * oDir[0] + oDir[2] * oDir[2]);
    east0[0] = -oDir[2] / yl; east0[1] = 0.0; east0[2] = oDir[0] / yl;   // d(dir)/dlon
    north0[0] = east0[1] * oDir[2] - east0[2] * oDir[1];
    north0[1] = east0[2] * oDir[0] - east0[0] * oDir[2];
    north0[2] = east0[0] * oDir[1] - east0[1] * oDir[0];
    const double det =
        east0[0] * (oDir[1] * north0[2] - oDir[2] * north0[1]) -
        east0[1] * (oDir[0] * north0[2] - oDir[2] * north0[0]) +
        east0[2] * (oDir[0] * north0[1] - oDir[1] * north0[0]);
    fr.valid = det >= 0.999;
    return fr;
}

}  // namespace ga::scene
