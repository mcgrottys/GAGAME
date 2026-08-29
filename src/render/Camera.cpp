#include "render/Camera.h"

#include "core/Pga.h"
#include "core/Window.h"

#include <cmath>
#include <algorithm>

using namespace DirectX;

namespace ga {

void Camera::SetFromCompass(double eastM, double altM, double northM, float azimuthDeg,
                            float pitchDeg) {
    px = eastM;
    py = altM;
    pz = northM;
    // Compass azimuth is clockwise from north; our yaw is counter-clockwise from east. The frame
    // is (x = east, y = up, z = north).
    yaw = XMConvertToRadians(90.0f - azimuthDeg);
    // Clamp short of exactly +-90: at exactly -90 the forward vector is parallel to world up,
    // cross(fwd, up) is zero, and the view matrix comes out full of NaNs -- which renders nothing
    // and writes a zero-byte dump that reads like a crash rather than a degenerate basis.
    const float lim = XM_PIDIV2 - 0.0017f;   // ~89.9 degrees
    pitch = std::clamp(XMConvertToRadians(pitchDeg), -lim, lim);
}

void Camera::LookAt(double tx, double ty, double tz) {
    const double dx = tx - px, dy = ty - py, dz = tz - pz;
    const double horiz = std::sqrt(dx * dx + dz * dz);
    yaw = static_cast<float>(std::atan2(dz, dx));
    const float lim = XM_PIDIV2 - 0.0017f;
    pitch = std::clamp(static_cast<float>(std::atan2(dy, std::max(horiz, 1e-6))), -lim, lim);
}

XMFLOAT3 Camera::Forward() const {
    const float cp = std::cos(pitch), sp = std::sin(pitch);
    return XMFLOAT3(std::cos(yaw) * cp, sp, std::sin(yaw) * cp);
}

XMFLOAT3 Camera::Right() const {
    // Horizontal right, independent of pitch, so strafing never drifts vertically.
    return XMFLOAT3(std::sin(yaw), 0.0f, -std::cos(yaw));
}

void Camera::Update(const InputState& in, float dt) {
    if (in.rmb) {
        constexpr float kLookRate = 0.0032f;   // radians per pixel
        yaw -= in.mouseDx * kLookRate;
        pitch -= in.mouseDy * kLookRate;
        const float lim = XM_PIDIV2 - 0.01f;
        pitch = std::clamp(pitch, -lim, lim);
    }
    if (in.wheel != 0.0f) {
        speed = std::clamp(speed * std::pow(1.25f, in.wheel), 0.5f, 20000.0f);
    }

    float mult = 1.0f;
    if (in.keyDown[VK_SHIFT]) mult = 8.0f;
    if (in.keyDown[VK_CONTROL]) mult = 0.15f;
    const float step = speed * mult * dt;

    const XMFLOAT3 f = Forward();
    const XMFLOAT3 r = Right();
    double dx = 0, dy = 0, dz = 0;
    auto add = [&](const XMFLOAT3& v, float s) { dx += v.x * s; dy += v.y * s; dz += v.z * s; };
    if (in.keyDown['W']) add(f, step);
    if (in.keyDown['S']) add(f, -step);
    if (in.keyDown['D']) add(r, step);
    if (in.keyDown['A']) add(r, -step);
    if (in.keyDown['E']) dy += step;
    if (in.keyDown['Q']) dy -= step;

    px += dx;
    py += dy;
    pz += dz;
}

bool Camera::OrbitAboutLine(const double pivot[3], const double axis[3], double angleRad,
                            double minAltY) {
    const double len = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (len < 1e-9) return false;
    const double d[3] = {axis[0] / len, axis[1] / len, axis[2] / len};
    const Motor m = Motor::Rotation(pivot, d, angleRad);

    double nx = px, ny = py, nz = pz;
    m.TransformPoint(nx, ny, nz);
    const XMFLOAT3 f = Forward();
    double fx = f.x, fy = f.y, fz = f.z;
    m.TransformDir(fx, fy, fz);

    const double horiz = std::sqrt(fx * fx + fz * fz);
    if (ny < minAltY || horiz < 1e-6) return false;   // reject, keep the current pose
    px = nx;
    py = ny;
    pz = nz;
    yaw = static_cast<float>(std::atan2(fz, fx));
    const float lim = XM_PIDIV2 - 0.0017f;
    pitch = std::clamp(static_cast<float>(std::atan2(fy, horiz)), -lim, lim);
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

XMMATRIX Camera::ViewRelative() const {
    const XMFLOAT3 f = Forward();
    const XMVECTOR eye = XMVectorZero();                       // camera-relative: always origin
    const XMVECTOR dir = XMVector3Normalize(XMLoadFloat3(&f));
    // M6g: roll follows ANTI-GRAVITY (upHint). Near-parallel views fall back to any
    // perpendicular so the basis never degenerates.
    XMVECTOR up = XMVector3Normalize(XMVectorSet(upHint[0], upHint[1], upHint[2], 0));
    const float align = std::fabs(XMVectorGetX(XMVector3Dot(dir, up)));
    if (align > 0.999f) {
        up = (std::fabs(f.y) > 0.999f) ? XMVectorSet(0, 0, 1, 0) : XMVectorSet(0, 1, 0, 0);
    }
    return XMMatrixLookToLH(eye, dir, up);
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
