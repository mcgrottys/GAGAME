// ================================================================================================
//  Camera - CAMERA-RELATIVE, REVERSED-Z, INFINITE FAR PLANE, AND A ROTOR FOR AN ATTITUDE.
//
//  All three are here because of where this is going (a globe), not because of what it draws
//  today. Carried from vqview-inlet, whose header explains at length:
//
//  CAMERA-RELATIVE: position is a double3 in world metres; the view matrix has its translation
//  zeroed; nothing in a vertex shader ever sees an absolute world coordinate. At earth scale
//  float32 world coordinates are catastrophic (~0.5 m spacing at earth-radius magnitudes).
//  TRAP: once positions are camera-relative, normalize(worldPos) is NOT an outward normal.
//  Pass normals explicitly; never rebuild them from a relative position.
//
//  REVERSED-Z, INFINITE FAR: depth is 1 at the near plane falling to 0 at infinity; clear to 0,
//  compare GREATER. Floats dense near zero oppose 1/z dense near the near plane, which is what
//  buys usable precision from 1 m to 1000 km.
//
//  THE ATTITUDE IS A ROTOR (2026-10-09, the owner: "using PGA benefits"). The pose is the motor
//  T(p) R: p the position, R the rotor taking the camera's own axes -- forward +x, up +y, right -z,
//  FromCamera's frame -- into the world. It replaced a yaw and a pitch measured in the ANCHOR's
//  flat frame (clamped at 89.9 degrees, the roll re-derived every frame from a float up-hint):
//  correct at the anchor and ever less so away from it, until 97 degrees round the planet (Tokyo
//  from the Merrimack) "down" was nearly horizontal and the clamp, the hint and the controls
//  fought. Now every law is said in the eye's own frame and holds anywhere:
//    look      heading turns about the LOCAL up (a rotor on the left), pitch about the camera's own
//              right; the one bound is that forward never crosses the local up or down.
//    up        `upRef`, the local up (anti-gravity, set by the session each frame). Free flight
//              CARRIES the attitude as the up turns (Transport: the rotor taking the old up to the
//              new, on the left), so a heading and a tilt over the ground mean the same thing at
//              every place. A pose written outright this frame (a rail, a chase, LookAt) is
//              LEVELLED instead -- its roll taken off against the up -- the rails' standing law.
//    matrix    only at the boundary: ViewRelative builds the LookTo from the rotor's own axes.
// ================================================================================================
#pragma once

#include "core/Common.h"
#include "core/Pga.h"

#include <DirectXMath.h>

namespace ga {

struct InputState;

class Camera {
public:
    // Position in world metres. Double, deliberately -- see the header comment.
    double px = 0, py = 0, pz = 0;
    // The attitude: the camera's own axes (forward +x, up +y, right -z) into the world. A pure
    // rotor (no translation part); compose with Motor::Translation(p) for the pose.
    Motor rot = Motor::Identity();
    // The local up the attitude is held to (unit, world frame): anti-gravity, written each frame.
    double upRef[3] = {0.0, 1.0, 0.0};
    float fovY = 0.9f;  // radians, VERTICAL
    float nearZ = 0.25f;
    float speed = 30.0f;   // metres per second

    // The compass placement in the world's own axes (x east, y up, z north): azimuth clockwise
    // from north, pitch up from level. Levelled against +y.
    void SetFromCompass(double eastM, double altM, double northM, float azimuthDeg,
                        float pitchDeg);
    // Aim at a world point / along a world direction, the roll levelled against upRef (against
    // world north, +z, where the aim runs along the up itself).
    void LookAt(double tx, double ty, double tz);
    void Aim(const double fwd[3]);
    // The whole attitude from a forward and an up (the up need not be exactly perpendicular). Its
    // roll is kept: free flight transports it, nothing levels it.
    void Orient(const double fwd[3], const double up[3]);
    // The local up: SetUp only records it; Transport carries the attitude with it (free flight),
    // or, when the pose was written outright since the last call, records it and levels.
    void SetUp(const double up[3]);
    void Transport(const double up[3]);
    // Take the roll off: the right axis back into the plane perpendicular to upRef.
    void Level();
    void Update(const InputState& in, float dt);

    // Google-Earth-style orbit: ONE rigid rotation of the camera (position and aim together)
    // about an arbitrary world line, done with a PGA motor (core/Pga.h). Returns false -- and
    // changes nothing -- if the result would sink below minAltY.
    bool OrbitAboutLine(const double pivot[3], const double axis[3], double angleRad,
                        double minAltY);
    // Wheel zoom: close `frac` of the distance to a world target (negative backs away).
    void DollyToward(const double target[3], double frac, double minDist);

    DirectX::XMFLOAT3 Forward() const;
    DirectX::XMFLOAT3 Right() const;   // the camera's own right (level in free flight)
    // The render basis: the rotor's own axes, exactly orthonormal. Every ray-reconstructing
    // consumer (scene constants, the globe's sky shell) takes this.
    void ViewBasis(DirectX::XMFLOAT3& fwd, DirectX::XMFLOAT3& right,
                   DirectX::XMFLOAT3& up) const;
    // The compass angles of the forward in the world's own axes (degrees: azimuth clockwise from
    // +z, pitch up from the xz plane) -- what SetFromCompass takes back, for saved views.
    void CompassOf(float& azimuthDeg, float& pitchDeg) const;

    // View with translation removed: the camera sits at the origin of the render frame.
    DirectX::XMMATRIX ViewRelative() const;
    // Reversed-Z, infinite far. aspect = width / height.
    DirectX::XMMATRIX Projection(float aspect) const;

private:
    bool m_written = false;   // the attitude was set outright since the last Transport
};

}  // namespace ga
