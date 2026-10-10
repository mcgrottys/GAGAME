// View - the component, the rasterizer boundary, and the re-levelling (M12 step 5b).
#include "scene/View.h"

#include "core/Common.h"
#include "render/Camera.h"
#include "scene/Pose.h"

#include <algorithm>
#include <cmath>

namespace ga::scene {

namespace {

double Dot3(const double a[3], const double b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
void Cross3(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}
// Unit length, in place. False (and the vector untouched) when there is nothing to normalize.
bool Norm3(double v[3]) {
    const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (!(n > 0.0)) return false;
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
    return true;
}

}  // namespace

Motor MotorOf(const Placement& p) {
    Motor r;
    const double du[4] = {0.0, 0.0, 0.0, 0.0};
    r.SetParts(p.r, du);
    return Motor::Translation(p.t[0], p.t[1], p.t[2]) * r;
}

View::View(std::string name) { m_props.name = std::move(name); }

std::vector<std::string> View::Configure(const Wiring& w) {
    (void)w;   // a view owns no device object: nothing to wire, nothing missing
    return {};
}

bool View::Init(Gpu& gpu) {
    (void)gpu;
    return true;
}

void View::Apply(const PropSet& props) {
    // THE ONE PATH for load and hot-reload (Component.h's law). A placement resolves only in a
    // frame; a View that was never given one refuses `at` and says which key, rather than
    // resolving a lat/lon against a guessed planet.
    std::string why;
    const PoseFrame* frame = m_frame.valid ? &m_frame : nullptr;
    if (!props.ApplyTo(&m_props, frame, &why)) {
        Log("[view] '%s' apply refused: %s", m_props.name.c_str(), why.c_str());
        return;
    }
    m_fovYRad = FovRadOf(m_props.fovY);
    m_pose = MotorOf(m_props.at);
}

void View::Update(const FrameInfo& f) {
    (void)f;   // the follow is applied by the frame loop from the followed entity (Follow)
}

void View::Follow(const FollowProps& follow, const double p[3], const double f[3], Camera& cam) {
    const double back = follow.back, up = follow.up;
    cam.px = p[0] - f[0] * back;
    cam.py = p[1] + up;
    cam.pz = p[2] - f[2] * back;
    cam.LookAt(p[0], p[1] + follow.aimLift, p[2]);
}

void View::Record(const ViewContext& v) {
    (void)v;   // a View draws nothing: it IS the view the drawing components are handed
}

void View::SetFovYRad(float rad) {
    m_fovYRad = rad;
    m_props.fovY = FovDegOf(rad);
}

void View::FromCamera(const Camera& c) {
    // poseMotor, verbatim -- scene/Pose.h holds the body the session's lambda holds.
    m_pose = scene::FromCamera(c);
    m_props.at = Placement::Rigid(m_pose);
    // The optics keep the RADIANS the camera had (so Camera -> View -> Camera is the identity)
    // and print the degrees a file would have said.
    SetFovYRad(c.fovY);
    m_props.nearZ = c.nearZ;
}

void View::ToCamera(Camera& c) const {
    // motorPose, verbatim. Position and aim; the up (upRef) and speed are the session's, untouched.
    scene::ToCamera(m_pose, c);
    c.fovY = m_fovYRad;
    c.nearZ = m_props.nearZ;
}

void View::Basis(const Motor& pose, const double up[3], double fwd[3], double right[3],
                 double upOut[3]) {
    // The aim is the motor's own +X axis (scene::FromCamera puts it there).
    double d[3] = {1.0, 0.0, 0.0};
    pose.TransformDir(d[0], d[1], d[2]);
    if (!Norm3(d)) {
        d[0] = 1.0;
        d[1] = 0.0;
        d[2] = 0.0;
    }
    double u[3] = {up[0], up[1], up[2]};
    if (!Norm3(u)) {
        u[0] = 0.0;
        u[1] = 1.0;
        u[2] = 0.0;
    }
    // M6j's SMOOTH degeneracy blend, verbatim in double (Camera::ViewBasis): as the aim closes
    // on the up field the reference blends toward flat north projected perpendicular to the
    // view, so the roll transition spreads across a descent instead of popping in one frame.
    const double align = std::fabs(Dot3(d, u));
    const double t = std::clamp((align - 0.985) / (0.9995 - 0.985), 0.0, 1.0);
    if (t > 0.0) {
        double ref[3] = {-d[0] * d[2], -d[1] * d[2], 1.0 - d[2] * d[2]};   // north - d (d . north)
        // ref is zero only when the aim IS north and the up field is along it too, which is
        // where ViewBasis divides by zero. Keep the raw up there rather than propagate a NaN;
        // nothing in the engine reaches it, and the gate's sweep stays out of it.
        if (Norm3(ref)) {
            const double s = t * t * (3.0 - 2.0 * t);
            for (int i = 0; i < 3; ++i) u[i] = u[i] + (ref[i] - u[i]) * s;
            Norm3(u);
        }
    }
    // The exact Gram-Schmidt XMMatrixLookToLH performs, which ViewBasis performs: one law.
    Cross3(u, d, right);
    Norm3(right);
    Cross3(d, right, upOut);
    fwd[0] = d[0];
    fwd[1] = d[1];
    fwd[2] = d[2];
}

Motor View::Level(const Motor& pose, const double up[3]) {
    double f[3], r[3], u[3];
    Basis(pose, up, f, r, u);
    double p[3] = {0.0, 0.0, 0.0};
    pose.TransformPoint(p[0], p[1], p[2]);
    return Frame(p, f, u);
}

Motor View::Frame(const double p[3], const double f[3], const double u[3]) {
    // THE ROTOR FROM THE FRAME. The rotation takes (e1, e2, e3) to (f, u, -r) -- the frame
    // scene::FromCamera's two turns produce -- and it is built as the two turns that ARE the
    // re-levelling: aim, then roll about the aim. Both angles come from atan2, which is
    // accurate where acos is not (at 0 and at pi, exactly where a camera looks along an axis).
    const double org[3] = {0.0, 0.0, 0.0};
    const double a[3] = {0.0, -f[2], f[1]};   // e1 x f
    const double an = std::sqrt(a[1] * a[1] + a[2] * a[2]);
    Motor aim = Motor::Identity();
    if (an > 0.0) {
        const double ax[3] = {0.0, a[1] / an, a[2] / an};
        aim = Motor::Rotation(org, ax, std::atan2(an, f[0]));
    } else if (f[0] < 0.0) {
        const double ax[3] = {0.0, 1.0, 0.0};   // f = -e1: a half turn, any perpendicular axis
        aim = Motor::Rotation(org, ax, 3.14159265358979323846);
    }
    double u1[3] = {0.0, 1.0, 0.0};
    aim.TransformDir(u1[0], u1[1], u1[2]);   // the aimed frame's up, still carrying e2's roll
    double c[3];
    Cross3(u1, u, c);
    const Motor roll = Motor::Rotation(org, f, std::atan2(Dot3(f, c), Dot3(u1, u)));
    return Motor::Translation(p[0], p[1], p[2]) * roll * aim;
}

}  // namespace ga::scene
