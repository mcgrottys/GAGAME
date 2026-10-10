#include "render/Camera.h"

#include "core/Window.h"
#include "scene/View.h"

#include <algorithm>
#include <cmath>

using namespace DirectX;

namespace ga {

namespace {

const double kOrigin[3] = {0.0, 0.0, 0.0};
constexpr double kDeg = 3.14159265358979323846 / 180.0;

double Dot(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
void Cross(const double a[3], const double b[3], double o[3]) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
bool Unit(double v[3]) {
    const double n = std::sqrt(Dot(v, v));
    if (!(n > 1e-300)) return false;
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
    return true;
}
// The rotor turning about a direction through the origin (right-handed).
Motor About(const double axis[3], double angle) {
    double d[3] = {axis[0], axis[1], axis[2]};
    if (!Unit(d)) return Motor::Identity();
    return Motor::Rotation(kOrigin, d, angle);
}
// The shortest-arc rotor taking unit a onto unit b; antiparallel, half a turn about a perpendicular.
Motor Between(const double a[3], const double b[3]) {
    double ax[3];
    Cross(a, b, ax);
    const double s = std::sqrt(Dot(ax, ax)), c = Dot(a, b);
    if (s < 1e-15) {
        if (c > 0.0) return Motor::Identity();
        const double t[3] = {std::fabs(a[0]) < 0.9 ? 1.0 : 0.0, std::fabs(a[0]) < 0.9 ? 0.0 : 1.0, 0.0};
        Cross(a, t, ax);
        return About(ax, 3.14159265358979323846);
    }
    return About(ax, std::atan2(s, c));
}
// A rotor kept unit (products drift) and pure (no translation part).
Motor Pure(const Motor& m) {
    const double n = std::sqrt(m.s * m.s + m.r23 * m.r23 + m.r31 * m.r31 + m.r12 * m.r12);
    Motor r;
    r.s = m.s / n;
    r.r23 = m.r23 / n;
    r.r31 = m.r31 / n;
    r.r12 = m.r12 / n;
    return r;
}
void Own(const Motor& r, double x, double y, double z, double o[3]) {
    o[0] = x;
    o[1] = y;
    o[2] = z;
    r.TransformDir(o[0], o[1], o[2]);
}

}  // namespace

void Camera::SetFromCompass(double eastM, double altM, double northM, float azimuthDeg,
                            float pitchDeg) {
    px = eastM;
    py = altM;
    pz = northM;
    // Compass azimuth is clockwise from north; a turn about +y is counter-clockwise from east. The
    // frame is (x = east, y = up, z = north): yaw first about the up, then pitch about the right
    // that yaw left -- FromCamera's own composition, so a compass pose and its motor agree.
    const double yaw = (90.0 - azimuthDeg) * kDeg, pitch = std::clamp(double(pitchDeg), -90.0, 90.0) * kDeg;
    const double y[3] = {0.0, 1.0, 0.0}, r[3] = {std::sin(yaw), 0.0, -std::cos(yaw)};
    rot = Pure(About(r, -pitch) * About(y, -yaw));
    m_written = true;
}

void Camera::LookAt(double tx, double ty, double tz) {
    const double f[3] = {tx - px, ty - py, tz - pz};
    Aim(f);
}

// THE LEVELLING LAW IS scene::View's (Basis: the LookTo Gram-Schmidt in double with M6j's smooth
// blend toward north as the aim closes on the up; Frame: the rotor from a forward and an up, aim
// then roll) -- written for this camera and pinned by the [view] gate; the camera reads it, it
// does not keep a second copy.
void Camera::Aim(const double fwd[3]) {
    double f[3] = {fwd[0], fwd[1], fwd[2]};
    if (!Unit(f)) return;
    rot = Pure(scene::View::Frame(kOrigin, f, upRef));
    Level();
    m_written = true;
}

void Camera::Orient(const double fwd[3], const double up[3]) {
    double f[3] = {fwd[0], fwd[1], fwd[2]};
    if (!Unit(f)) return;
    rot = Pure(scene::View::Frame(kOrigin, f, up));
}

void Camera::Level() {
    double f[3], r[3], u[3];
    scene::View::Basis(rot, upRef, f, r, u);
    rot = Pure(scene::View::Frame(kOrigin, f, u));
}

void Camera::SetUp(const double up[3]) {
    double u[3] = {up[0], up[1], up[2]};
    if (!Unit(u)) return;
    for (int i = 0; i < 3; ++i) upRef[i] = u[i];
}

void Camera::Transport(const double up[3]) {
    double u[3] = {up[0], up[1], up[2]};
    if (!Unit(u)) return;
    if (m_written) {   // a pose written outright: the up recorded, the roll taken off
        SetUp(u);
        Level();
        m_written = false;
        return;
    }
    rot = Pure(Between(upRef, u) * rot);
    for (int i = 0; i < 3; ++i) upRef[i] = u[i];
}

XMFLOAT3 Camera::Forward() const {
    double f[3];
    Own(rot, 1.0, 0.0, 0.0, f);
    return XMFLOAT3(static_cast<float>(f[0]), static_cast<float>(f[1]), static_cast<float>(f[2]));
}

XMFLOAT3 Camera::Right() const {
    double r[3];
    Own(rot, 0.0, 0.0, -1.0, r);
    return XMFLOAT3(static_cast<float>(r[0]), static_cast<float>(r[1]), static_cast<float>(r[2]));
}

void Camera::Update(const InputState& in, float dt) {
    if (in.rmb) {
        constexpr double kLookRate = 0.0032;   // radians per pixel
        // Heading about the local up; pitch about the own right, refused where the forward would
        // cross the up (the one bound: a view along the up has no heading left to turn by).
        if (in.mouseDx != 0.0f) rot = Pure(About(upRef, in.mouseDx * kLookRate) * rot);
        if (in.mouseDy != 0.0f) {
            double r[3], f[3];
            Own(rot, 0.0, 0.0, -1.0, r);
            const Motor next = Pure(About(r, in.mouseDy * kLookRate) * rot);
            Own(next, 1.0, 0.0, 0.0, f);
            if (std::fabs(Dot(f, upRef)) < 0.99995) rot = next;
        }
    }
    if (in.wheel != 0.0f) {
        speed = std::clamp(speed * std::pow(1.25f, in.wheel), 0.5f, 20000.0f);
    }

    float mult = 1.0f;
    if (in.keyDown[VK_SHIFT]) mult = 8.0f;
    if (in.keyDown[VK_CONTROL]) mult = 0.15f;
    const double step = double(speed) * mult * dt;

    double f[3], r[3];
    Own(rot, 1.0, 0.0, 0.0, f);
    Own(rot, 0.0, 0.0, -1.0, r);
    double d[3] = {0.0, 0.0, 0.0};
    auto add = [&](const double v[3], double s) {
        for (int i = 0; i < 3; ++i) d[i] += v[i] * s;
    };
    if (in.keyDown['W']) add(f, step);
    if (in.keyDown['S']) add(f, -step);
    if (in.keyDown['D']) add(r, step);
    if (in.keyDown['A']) add(r, -step);
    if (in.keyDown['E']) add(upRef, step);   // up and down are the local ones
    if (in.keyDown['Q']) add(upRef, -step);

    px += d[0];
    py += d[1];
    pz += d[2];
}

bool Camera::OrbitAboutLine(const double pivot[3], const double axis[3], double angleRad,
                            double minAltY) {
    double d[3] = {axis[0], axis[1], axis[2]};
    if (!Unit(d)) return false;
    const Motor m = Motor::Rotation(pivot, d, angleRad);
    double nx = px, ny = py, nz = pz;
    m.TransformPoint(nx, ny, nz);
    if (ny < minAltY) return false;   // reject, keep the current pose
    px = nx;
    py = ny;
    pz = nz;
    rot = Pure(About(d, angleRad) * rot);   // the same motor's rotor turns the attitude
    // ...and the up it was levelled against: the attitude and its up reference are ONE frame. Left
    // behind, Transport's shortest arc from the stale up to the new local up turned the attitude a
    // second time each frame -- unseen over the ground, and at 20,000 km a drag that should spin the
    // planet under the cursor turned the eye off it (2026-10-10).
    m.TransformDir(upRef[0], upRef[1], upRef[2]);
    return true;
}

void Camera::DollyToward(const double target[3], double frac, double minDist) {
    const double dx = target[0] - px, dy = target[1] - py, dz = target[2] - pz;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < 1e-6) return;
    double move = dist * frac;
    if (move > 0.0) move = std::min(move, dist - minDist);   // never punch through the ground
    if (move <= 0.0 && frac > 0.0) return;
    px += dx / dist * move;
    py += dy / dist * move;
    pz += dz / dist * move;
}

void Camera::ViewBasis(XMFLOAT3& fwd, XMFLOAT3& right, XMFLOAT3& up) const {
    double f[3], r[3], u[3];
    Own(rot, 1.0, 0.0, 0.0, f);
    Own(rot, 0.0, 0.0, -1.0, r);
    Own(rot, 0.0, 1.0, 0.0, u);
    fwd = XMFLOAT3(float(f[0]), float(f[1]), float(f[2]));
    right = XMFLOAT3(float(r[0]), float(r[1]), float(r[2]));
    up = XMFLOAT3(float(u[0]), float(u[1]), float(u[2]));
}

void Camera::CompassOf(float& azimuthDeg, float& pitchDeg) const {
    double f[3];
    Own(rot, 1.0, 0.0, 0.0, f);
    double az = 90.0 - std::atan2(f[2], f[0]) / kDeg;
    az = std::fmod(std::fmod(az, 360.0) + 360.0, 360.0);
    azimuthDeg = static_cast<float>(az);
    pitchDeg = static_cast<float>(std::atan2(f[1], std::sqrt(f[0] * f[0] + f[2] * f[2])) / kDeg);
}

XMMATRIX Camera::ViewRelative() const {
    XMFLOAT3 f, r, u;
    ViewBasis(f, r, u);
    return XMMatrixLookToLH(XMVectorZero(), XMLoadFloat3(&f), XMLoadFloat3(&u));
}

XMMATRIX Camera::Projection(float aspect) const {
    // Reversed-Z with an infinite far plane, row-vector convention to match DirectXMath.
    //   clip.z = nearZ,  clip.w = viewZ   =>   ndcZ = nearZ / viewZ
    // Depth clears to 0 and compares GREATER. Nothing ever gets clipped away in the distance.
    const float h = 1.0f / std::tan(fovY * 0.5f);
    const float w = h / aspect;
    return XMMATRIX(w, 0, 0, 0,
                    0, h, 0, 0,
                    0, 0, 0, 1,
                    0, 0, nearZ, 0);
}

}  // namespace ga
