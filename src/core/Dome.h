// ================================================================================================
//  Dome.h - THE SKY'S FRAME AT A PLACE (M13).
//
//  A sky is asked two questions and only two: how high above the horizon is this ray
//  (dot(d, +y)), and how near the sun is it (dot(d, sun))? Both are answered in a frame whose +y
//  is the ZENITH THERE -- so the whole of "whose sky is this" is one rotor: the shortest arc that
//  carries that place's radial onto +y, with the sun turned by the same rotor so the two answers
//  stay consistent. The identity at a tangent frame's own origin; 18.56 degrees for a place two
//  thousand kilometres out, which is the angle the ground turns, not an angle the sun moves.
//
//  Declared here because TWO eyes ask it: the camera's own dome, and the sky seen through a gate's
//  window at the other end of the carry. With two constructions a window would show a sky no
//  camera standing there could see -- and it did: the window's rays were asked for their elevation
//  against this place's +y and read 18.6 degrees below the horizon everywhere, a flat grey slab
//  over the destination's sea.
// ================================================================================================
#pragma once

#include "core/Pga.h"

#include <cmath>

namespace ga {

// THE ZENITH AT A POINT: the planet's radial through it, in the tangent frame the renderer speaks
// (the planet's centre sits at (0, -R, 0) there). The gravity-up the camera clamp writes is the
// same vector; the skies read it in doubles.
inline void ZenithAt(const double p[3], double planetR, double out[3]) {
    const double gy = p[1] + planetR;
    const double gl = std::sqrt(p[0] * p[0] + gy * gy + p[2] * p[2]);
    out[0] = p[0] / gl;
    out[1] = gy / gl;
    out[2] = p[2] / gl;
}

// The shortest arc from a unit up onto +y, as a quaternion: (1 + u.y, u x y) normalised
// (Portal::Carry's construction; Motor::QRotate turns u onto +y). False only at the antipode of
// +y, where the arc is not defined and no eye stands.
inline bool ZenithRotor(const double u[3], double q[4]) {
    q[0] = 1.0 + u[1];
    q[1] = -u[2];
    q[2] = 0.0;
    q[3] = u[0];
    const double qn = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (qn <= 1e-9) return false;
    for (int i = 0; i < 4; ++i) q[i] /= qn;
    return true;
}

// The dome at a place, as the shader takes it: rows that carry a direction of THIS frame into that
// dome's (column c is the image of axis c), and the sun said in the same frame. One rotor, four
// vectors. `u` and `sunWorld` must be given in the same frame -- for a window, that is the frame
// the window is DRAWN in, so both come through the carry's rotation first.
inline void DomeFrame(const double u[3], const float sunWorld[3], float rows[9], float sun[3]) {
    const float id[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    for (int i = 0; i < 9; ++i) rows[i] = id[i];
    for (int i = 0; i < 3; ++i) sun[i] = sunWorld[i];
    double q[4];
    if (!ZenithRotor(u, q)) return;
    for (int c = 0; c < 3; ++c) {
        double x = c == 0 ? 1.0 : 0.0, y = c == 1 ? 1.0 : 0.0, z = c == 2 ? 1.0 : 0.0;
        Motor::QRotate(q, x, y, z);
        rows[0 * 3 + c] = static_cast<float>(x);
        rows[1 * 3 + c] = static_cast<float>(y);
        rows[2 * 3 + c] = static_cast<float>(z);
    }
    double sx = sunWorld[0], sy = sunWorld[1], sz = sunWorld[2];
    Motor::QRotate(q, sx, sy, sz);
    sun[0] = static_cast<float>(sx);
    sun[1] = static_cast<float>(sy);
    sun[2] = static_cast<float>(sz);
}

}  // namespace ga
