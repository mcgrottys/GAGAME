// ================================================================================================
//  Minimap - A SECOND EYE IN A CORNER OF THE GLASS (scene `hud.minimap`).
//
//  Not a map and not a texture: an eye. Its pose is ONE motor in the root's tangent frame, the
//  same planet and the same tiles the first eye reads are walked again from it (GlobeLayer::
//  SetOtherView), and the renderer records it into a rectangle of the same target
//  (Renderer::ViewOf). Everything it does with the mouse is a motor composed onto that pose:
//
//      grab    the point of the sea-level sphere under the cursor stays under the cursor: one
//              rotor about the planet's centre takes the point now under it back to the grabbed
//              one (the first eye's orbital grab, FrameLoop). Exact at every altitude, so there
//              is no second law for low-altitude panning: turning the planet IS the pan.
//      wheel   a translator along the ray through the cursor, closing a fixed fraction of the
//              distance per notch: every approach is exponential, so the wheel crosses the
//              decades from orbit to the water at one rate (the floor is minAltM above the sea).
//      Reset   the screw from the pose to the home pose (homeAltM straight above the followed
//              entity, north up), walked at constant twist (Motor::Slerp) with a smoothstep in
//              time: rotation and translation arrive together.
//
//  The rasterizer gets a Camera at the last moment (BuildCamera), with the motor's own up as the
//  roll reference -- so looking straight down is an ordinary view, neither clamped nor blended
//  (Camera::ViewBasis); the [minimap] self-test holds the grab and home to the pixel.
//  Nothing here knows a boat, a sea or a layer: a remote player's eye or a movable portal's is
//  the same object with a different owner of its rectangle.
// ================================================================================================
#pragma once

#include "core/Pga.h"
#include "render/Camera.h"
#include "scene/SceneSchema.h"

#include <cstdint>

namespace ga {
struct InputState;
}

namespace ga::scene {

class Minimap {
public:
    struct Rect {
        float x = 0, y = 0, w = 0, h = 0;
        bool Contains(float px, float py) const {
            return px >= x && py >= y && px < x + w && py < y + h;
        }
    };

    // The declaration and the planet: its radius, and the pole's direction in the root's
    // tangent frame (north is up at home).
    void Configure(const MinimapProps& p, double planetR, const double poleFlat[3]);
    bool Enabled() const { return m_p.enabled; }

    // Where it stands on a target of W x H pixels, and its Reset button inside that.
    Rect Place(uint32_t W, uint32_t H) const;
    static Rect Button(const Rect& r);

    // The mouse, first: what lands on the rectangle (or continues a drag that began there) is
    // the minimap's, and is taken out of `in` so the first eye's gestures never see it.
    // Returns true when it took anything.
    bool Input(InputState& in, uint32_t W, uint32_t H);

    // Once a frame, before it is drawn: the followed entity's point in the root frame (null when
    // there is none), and the frame's wall time for the reset's walk.
    void Step(float dt, const double* subject);
    void Reset() { m_resetT = 0.0f; }
    bool Resetting() const { return m_resetT >= 0.0f; }

    // The rasterizer's camera, built from the motor.
    const Camera& Cam() const { return m_cam; }
    double Altitude() const;
    // A root-frame point on the rectangle: false when it is behind the eye, off the rectangle, or
    // hidden behind the planet's limb.
    bool Project(const double p[3], const Rect& r, float& sx, float& sy) const;
    // The inverse: the sea-level sphere's point under a pixel of the rectangle (the grab point,
    // on the limb for a pixel off the planet).
    void Pick(const Rect& r, float px, float py, double out[3]) const {
        double d[3];
        Ray(r, px, py, d);
        HitSphere(d, out);
    }

private:
    Motor Home(const double s[3]) const;
    void BuildCamera();
    void Ray(const Rect& r, float px, float py, double d[3]) const;
    bool HitSphere(const double d[3], double out[3]) const;

    MinimapProps m_p;
    double m_R = 6371000.0;
    double m_pole[3] = {0.0, 1.0, 0.0};
    Motor m_pose = Motor::Identity();
    Camera m_cam;
    bool m_placed = false;
    bool m_dragging = false;
    bool m_lmbWas = false;
    double m_grab[3] = {};
    float m_resetT = -1.0f;   // seconds into the walk home; < 0 = not resetting
    Motor m_resetFrom = Motor::Identity();
    double m_subject[3] = {};
    bool m_hasSubject = false;
};

}  // namespace ga::scene
