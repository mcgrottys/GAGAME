// Pose - the session's pose maps, bodies verbatim from FrameLoop::Session (M12 step 5a).
#include "scene/Pose.h"

#include "render/Camera.h"
#include "sim/GlobeModel.h"

#include <algorithm>
#include <cmath>

namespace ga::scene {

Motor FromCamera(const Camera& c) {
    const double org[3] = {0, 0, 0};
    const double yAxis[3] = {0, 1, 0};
    const double rAxis[3] = {std::sin(c.yaw), 0.0, -std::cos(c.yaw)};
    return Motor::Translation(c.px, c.py, c.pz) *
           Motor::Rotation(org, rAxis, -c.pitch) * Motor::Rotation(org, yAxis, -c.yaw);
}

void ToCamera(const Motor& m, Camera& c) {
    double px = 0, py = 0, pz = 0;
    m.TransformPoint(px, py, pz);
    double fx = 1, fy = 0, fz = 0;
    m.TransformDir(fx, fy, fz);
    c.px = px;
    c.py = py;
    c.pz = pz;
    c.yaw = static_cast<float>(std::atan2(fz, fx));
    const float lim = 3.14159265f / 2.0f - 0.0017f;
    c.pitch = std::clamp(
        static_cast<float>(std::atan2(fy, std::sqrt(fx * fx + fz * fz))), -lim, lim);
}

Camera OrbitPose(double lat, double lon, double altM, double tLat, double tLon, double planetR) {
    Camera c;
    double d[3];
    GlobeModel::LatLonDir(lat, lon, d);
    const double rr = planetR + altM;
    c.px = d[0] * rr;
    c.py = d[1] * rr;
    c.pz = d[2] * rr;
    double t[3];
    GlobeModel::LatLonDir(tLat, tLon, t);
    c.LookAt(t[0] * planetR, t[1] * planetR, t[2] * planetR);
    return c;
}

Camera GlobeCamera(double lat, double lon, double altM, double planetR) {
    Camera camGlobe;
    const double gR = planetR + altM;
    double d[3];
    GlobeModel::LatLonDir(lat, lon, d);
    camGlobe.px = d[0] * gR;
    camGlobe.py = d[1] * gR;
    camGlobe.pz = d[2] * gR;
    camGlobe.LookAt(0.0, 0.0, 0.0);
    return camGlobe;
}

Camera PlanetToFlatPose(const Camera& g, const double east0[3], const double oDir[3],
                        const double north0[3], double planetR) {
    Camera f = g;
    const double p[3] = {g.px, g.py, g.pz};
    f.px = p[0] * east0[0] + p[1] * east0[1] + p[2] * east0[2];
    f.py = p[0] * oDir[0] + p[1] * oDir[1] + p[2] * oDir[2] - planetR;
    f.pz = p[0] * north0[0] + p[1] * north0[1] + p[2] * north0[2];
    const DirectX::XMFLOAT3 ff = g.Forward();
    const double d[3] = {ff.x, ff.y, ff.z};
    const double fx = d[0] * east0[0] + d[1] * east0[1] + d[2] * east0[2];
    const double fy = d[0] * oDir[0] + d[1] * oDir[1] + d[2] * oDir[2];
    const double fz = d[0] * north0[0] + d[1] * north0[1] + d[2] * north0[2];
    f.yaw = static_cast<float>(std::atan2(fz, fx));
    const float lim = 3.14159265f / 2.0f - 0.0017f;
    f.pitch = std::clamp(
        static_cast<float>(std::atan2(fy, std::sqrt(fx * fx + fz * fz))), -lim, lim);
    return f;
}

Camera FlatToPlanetPose(const Camera& f, const double east0[3], const double oDir[3],
                        const double north0[3], double planetR) {
    Camera g = f;
    const double r = planetR + f.py;
    g.px = oDir[0] * r + east0[0] * f.px + north0[0] * f.pz;
    g.py = oDir[1] * r + east0[1] * f.px + north0[1] * f.pz;
    g.pz = oDir[2] * r + east0[2] * f.px + north0[2] * f.pz;
    const DirectX::XMFLOAT3 ff = f.Forward();
    const double d[3] = {ff.x, ff.y, ff.z};
    const double gx = east0[0] * d[0] + oDir[0] * d[1] + north0[0] * d[2];
    const double gy = east0[1] * d[0] + oDir[1] * d[1] + north0[1] * d[2];
    const double gz = east0[2] * d[0] + oDir[2] * d[1] + north0[2] * d[2];
    g.yaw = static_cast<float>(std::atan2(gz, gx));
    const float lim = 3.14159265f / 2.0f - 0.0017f;
    g.pitch = std::clamp(
        static_cast<float>(std::atan2(gy, std::sqrt(gx * gx + gz * gz))), -lim, lim);
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
