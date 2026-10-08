// ================================================================================================
//  View - M12 step 5b: A VIEW IS A COMPONENT, AND ITS PLACEMENT IS A MOTOR.
//
//  The engine has one camera and six ideas of what a view is (the four session cameras, the rail
//  poses, the Droste gauge, the chase cam, the predicted eye, the pickers), all of them Euler
//  angles in floats with the roll structurally dropped. This is the declaration those ideas
//  collapse into: a named component whose PLACEMENT is a motor in its parent space, whose optics
//  are three numbers, and whose rasterizer boundary is exactly two functions.
//
//  WHAT A VIEW DECLARES (the Schema IS the scene file's `views` section -- SceneSchema.h's
//  ViewSchema(), one table, so the file format, --print-scene and this component cannot drift):
//      name                 the view's name; `scene.view` names the one that starts
//      at                   the eye, in the placement sugar (a motor, rigid; absent = the mode's
//                           default, which is what every recorded recipe still means)
//      fovY, nearZ          the optics a Camera carries
//      reversedZ            depth 1 at the near plane falling to 0 at infinity (Camera.h's law);
//                           declared because it is a property of a VIEW, not of the engine, and
//                           the day a second view wants the ordinary sense it says so here
//      gauge                the SPACE the eye is expressed in. "root" today. Gauge() is its
//                           integer expression in the Droste tower -- today's camLevel, 0 at the
//                           root, k inside the k-th link -- the number the session keeps beside
//                           the camera and re-roots against every time the eye crosses a level.
//      target, viewport     where the recording lands: the named target chain ("main" = the
//                           renderer's own HDR -> tonemap -> backbuffer) and the rectangle of it
//      follow               the chase camera, as data: {target, back, up, aimLift}, the four
//                           numbers FrameLoop.cpp's `if (helming)` block holds as literals. An
//                           empty `target` is "this view follows nothing", which is every
//                           recorded recipe but the boat's.
//
//  THE RASTERIZER BOUNDARY IS ToCamera(), AND NOTHING ELSE. A Camera is Euler angles, floats and
//  a hard-coded projection -- the shape the rasterizer wants and the wrong shape for anything
//  else. So the View keeps the pose as a motor and hands the rasterizer a Camera at the last
//  moment; FromCamera() is the inverse, the session's own `poseMotor` (scene/Pose.h, moved
//  verbatim in 5a and called here, not re-derived). The optics cross that boundary through the
//  session's OWN conversion: the declaration is degrees, because that is what a file and --fov
//  say, and the resolved radians are what a Camera holds, so `scene fovY -> Camera` is bit-equal
//  to the flag path and `Camera -> View -> Camera` is the identity (FromCamera keeps the radians
//  it was given and prints the degrees).
//
//  LEVEL: THE RE-LEVELLING, AS ONE FUNCTION OF A MOTOR AND AN UP FIELD.
//  Today a view is kept level by ACCIDENT of its representation: a motor becomes yaw and pitch
//  (scene::ToCamera), the roll falls off the end because two Euler angles cannot hold it, and
//  Camera::ViewBasis then re-derives the third axis from `upHint` -- the anti-gravity FIELD the
//  frame loop writes every frame (radial from the planet's centre; +y at the estuary). Two steps,
//  a lossy float round trip in the middle, and the law spread across two files.
//
//  Level(pose, up) is that law as ONE pure function: take the motor's own forward, level it
//  against the up field, and build the rotor from the frame. The roll is removed BY
//  CONSTRUCTION -- the returned motor's Y axis is the levelled up, whatever roll the input
//  carried -- rather than discarded by an extraction that cannot represent it. It is introduced
//  BESIDE the existing extraction and NOTHING READS IT THIS STEP; the gate ([view] in
//  --selftest) is what earns it the right to be read: at the six recipe poses and a thousand
//  random ones it reproduces the frame Camera::ViewBasis builds (to the float basis' own
//  resolution), it reproduces `poseMotor` exactly where the up field is the world's up, it is
//  invariant to a roll of +-30 degrees, and it is idempotent.
//
//  Basis() is the same law returning the three vectors instead of the rotor: Camera::ViewBasis's
//  arithmetic in DOUBLE, degeneracy blend and all, so that a caller who wants axes and a caller
//  who wants a versor are reading one law and not two.
//
//  Prior art, named. The gauge -- an eye expressed in a space other than the root, re-rooted as
//  it moves -- is the floating origin of KSP and 64-bit-world practice (the plan's naming), and
//  the level index is that idea applied to a self-similar tower rather than to a single shift.
//  The re-levelling itself is NOT new and is not claimed to be: building an orientation from a
//  forward and a chosen up is gluLookAt / XMMatrixLookToLH, Unity's Quaternion.LookRotation
//  (forward, upwards) and Maya's aim constraint with a world-up vector or object, and "rebuild
//  the rotation from the forward and the up, then convert back" is the standard answer to
//  unwanted camera roll in the engine forums. What I did NOT find named anywhere is the up as a
//  FIELD rather than a constant vector -- the closest are astrodynamics' local-vertical /
//  local-horizontal (LVLH) frame and an aircraft's artificial horizon, both of which level an
//  attitude against the local gravity direction at the vehicle's own position. So: the operation
//  is old, the field is the engine's own (M6g), and the only thing this function adds is doing
//  both in one place, in double, on a motor.
// ================================================================================================
#pragma once

#include "core/Pga.h"
#include "core/Space.h"
#include "scene/Component.h"
#include "scene/SceneSchema.h"

#include <string>

namespace ga {
class Camera;
}

namespace ga::scene {

// A rigid Placement as the motor it is: the inverse of Placement::Rigid (Space.h), which is how
// the scene's placement sugar reaches a pose. Rigid(MotorOf(p)) == p for every rigid p.
Motor MotorOf(const Placement& p);

class View final : public Component {
public:
    View() = default;
    explicit View(std::string name);

    // ---- Component. A View records nothing: it IS the view, and the components that draw are
    // handed it (Record(const ViewContext&)). Init needs no device; Configure needs no observer.
    const char* Name() const override { return m_props.name.c_str(); }
    const Schema& Props() const override { return ViewSchema(); }
    std::vector<std::string> Configure(const Wiring& w) override;
    bool Init(Gpu& gpu) override;
    void Apply(const PropSet& props) override;
    void Update(const FrameInfo& f) override;
    void Record(const ViewContext& v) override;
    void ReloadShaders() override {}

    // ---- the declaration, as read from the file (and as --print-scene prints it back)
    const ViewProps& Declared() const { return m_props; }
    ViewProps& Declared() { return m_props; }

    // The frame a lat/lon placement resolves in (the session's tangent rows). A View given no
    // frame refuses to resolve `at` and says so; it never guesses one.
    void SetFrame(const PoseFrame& frame) { m_frame = frame; }
    const PoseFrame& Frame() const { return m_frame; }

    // ---- the placement: ONE motor, in the parent space
    const Motor& Pose() const { return m_pose; }
    void SetPose(const Motor& m) { m_pose = m; }

    // The space the eye is expressed in, as its level in the Droste tower: today's camLevel.
    int Gauge() const { return m_gauge; }
    void SetGauge(int level) { m_gauge = level; }

    // ---- THE RASTERIZER BOUNDARY (scene/Pose.h's two functions, and the optics with them)
    void FromCamera(const Camera& c);
    void ToCamera(Camera& c) const;

    // The optics as the Camera holds them: radians, converted by the session's own line.
    float FovYRad() const { return m_fovYRad; }
    void SetFovYRad(float rad);
    static float FovRadOf(float deg) { return deg * 3.14159265f / 180.0f; }
    static float FovDegOf(float rad) { return rad * 180.0f / 3.14159265f; }

    // ---- THE CHASE CAMERA (M12 step 5e). Pure: the eye `back` behind the target's heading
    // and `up` above it, aimed `aimLift` above the target's origin -- the follow's four
    // numbers, fed by the entity's pose (scene/Entity.h ChaseFrame). Gravity-up and roll-free
    // by choice: a camera that heels with the hull reads as the WORLD rolling, which is
    // nauseating and is not what a helmsman's inner ear reports.
    static void Follow(const FollowProps& follow, const double target[3], const double heading[3],
                       Camera& cam);

    // ---- THE RE-LEVELLING. Pure: no member is read or written.
    static Motor Level(const Motor& pose, const double up[3]);
    // The motor of the frame at p looking along unit f with unit up u (u perpendicular to f):
    // the rotor Level builds, for a caller that already holds the frame (a minimap's eye).
    static Motor Frame(const double p[3], const double f[3], const double u[3]);
    static void Basis(const Motor& pose, const double up[3], double fwd[3], double right[3],
                      double upOut[3]);

private:
    ViewProps m_props;
    PoseFrame m_frame;
    Motor m_pose;
    int m_gauge = 0;
    float m_fovYRad = FovRadOf(55.0f);
};

}  // namespace ga::scene
