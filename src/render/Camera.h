// ================================================================================================
//  Camera - CAMERA-RELATIVE, REVERSED-Z, INFINITE FAR PLANE.
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
// ================================================================================================
#pragma once

#include "core/Common.h"

#include <DirectXMath.h>

namespace ga {

struct InputState;

class Camera {
public:
    // Position in world metres. Double, deliberately -- see the header comment.
    double px = 0, py = 0, pz = 0;
    float yaw = 0;      // radians, 0 = +X (east), increasing toward +Z (north)
    float pitch = 0;    // radians, negative looks down
    float fovY = 0.9f;  // radians, VERTICAL
    float nearZ = 0.25f;
    float speed = 30.0f;   // metres per second

    void SetFromCompass(double eastM, double altM, double northM, float azimuthDeg,
                        float pitchDeg);
    // Aim at a world point.
    void LookAt(double tx, double ty, double tz);
    void Update(const InputState& in, float dt);

    // Google-Earth-style orbit: ONE rigid rotation of the camera (position and aim together)
    // about an arbitrary world line, done with a PGA motor (core/Pga.h). Returns false -- and
    // changes nothing -- if the result would sink below minAltY or aim degenerately vertical.
    bool OrbitAboutLine(const double pivot[3], const double axis[3], double angleRad,
                        double minAltY);
    // Wheel zoom: close `frac` of the distance to a world target (negative backs away).
    void DollyToward(const double target[3], double frac, double minDist);

    DirectX::XMFLOAT3 Forward() const;
    DirectX::XMFLOAT3 Right() const;

    // View with translation removed: the camera sits at the origin of the render frame.
    DirectX::XMMATRIX ViewRelative() const;
    // Reversed-Z, infinite far. aspect = width / height.
    DirectX::XMMATRIX Projection(float aspect) const;
};

}  // namespace ga
