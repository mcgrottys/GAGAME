#include "scene/Minimap.h"

#include "core/Window.h"
#include "scene/View.h"

#include <algorithm>
#include <cmath>

namespace ga::scene {

namespace {
constexpr float kFovY = 0.70f;          // radians: a little longer lens than the helm's
constexpr float kResetSec = 0.6f;       // the walk home
constexpr double kWheelStep = 0.8;      // each notch keeps this share of the distance
double Dot(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
void Unit(double v[3]) {
    const double l = std::sqrt(Dot(v, v));
    if (l > 0.0) {
        v[0] /= l;
        v[1] /= l;
        v[2] /= l;
    }
}
}  // namespace

void Minimap::Configure(const MinimapProps& p, double planetR, const double poleFlat[3]) {
    m_p = p;
    m_R = planetR;
    for (int i = 0; i < 3; ++i) m_pole[i] = poleFlat[i];
    Unit(m_pole);
    m_cam.fovY = kFovY;
    m_followAlt = p.homeAltM;
}

Minimap::Rect Minimap::Place(uint32_t W, uint32_t H) const {
    const float side = std::floor(static_cast<float>(m_p.size) * static_cast<float>(H));
    const float m = static_cast<float>(m_p.margin);
    return Rect{static_cast<float>(W) - m - side, static_cast<float>(H) - m - side, side, side};
}

Minimap::Rect Minimap::Button(const Rect& r) {
    const float b = std::floor((std::max)(24.0f, r.w * 0.1f));
    return Rect{r.x + r.w - b - 6.0f, r.y + 6.0f, b, b};
}

// The unit ray through a pixel of the rectangle, from the basis the picture is drawn with.
void Minimap::Ray(const Rect& r, float px, float py, double d[3]) const {
    DirectX::XMFLOAT3 f, rt, u;
    m_cam.ViewBasis(f, rt, u);
    const double th = std::tan(0.5 * m_cam.fovY);
    const double x = (2.0 * (px - r.x) / r.w - 1.0) * th * (r.w / r.h);
    const double y = (1.0 - 2.0 * (py - r.y) / r.h) * th;
    d[0] = f.x + rt.x * x + u.x * y;
    d[1] = f.y + rt.y * x + u.y * y;
    d[2] = f.z + rt.z * x + u.z * y;
    Unit(d);
}

// THE GRAB POINT: where the ray meets the sea-level sphere, or -- for a ray that misses it -- the
// sphere's point nearest the ray, which is on the limb. One law, continuous across the limb, so a
// drag that slides off the planet keeps turning it rather than stopping dead.
bool Minimap::HitSphere(const double d[3], double out[3]) const {
    const double e[3] = {m_cam.px, m_cam.py + m_R, m_cam.pz};   // the eye about the centre
    const double b = Dot(e, d);
    const double c = Dot(e, e) - m_R * m_R;
    const double disc = b * b - c;
    double q[3];
    if (disc >= 0.0 && -b - std::sqrt(disc) > 0.0) {
        const double t = -b - std::sqrt(disc);
        for (int i = 0; i < 3; ++i) q[i] = e[i] + d[i] * t;
    } else {
        const double t = (std::max)(-b, 0.0);   // the ray's nearest approach to the centre
        for (int i = 0; i < 3; ++i) q[i] = e[i] + d[i] * t;
        Unit(q);
        for (int i = 0; i < 3; ++i) q[i] *= m_R;
    }
    out[0] = q[0];
    out[1] = q[1] - m_R;
    out[2] = q[2];
    return true;
}

bool Minimap::Input(InputState& in, uint32_t W, uint32_t H) {
    if (!m_p.enabled) return false;
    const Rect r = Place(W, H);
    const bool over = r.Contains(in.mouseX, in.mouseY);
    const bool press = in.lmb && !m_lmbWas;
    m_lmbWas = in.lmb;
    bool took = false;
    if (press && over) {
        if (Button(r).Contains(in.mouseX, in.mouseY)) {
            Reset();
        } else {
            double d[3];
            Ray(r, in.mouseX, in.mouseY, d);
            HitSphere(d, m_grab);
            m_dragging = true;
        }
    }
    if (!in.lmb) m_dragging = false;
    if (m_dragging) {
        m_resetT = -1.0f;   // the hand takes over from the walk home
        double d[3], now[3];
        Ray(r, in.mouseX, in.mouseY, d);
        HitSphere(d, now);
        // The rotor about the planet's centre taking the point now under the cursor to the one
        // that was grabbed: turning the eye by it puts the grabbed point back under the cursor.
        double a[3] = {now[0], now[1] + m_R, now[2]};
        double b[3] = {m_grab[0], m_grab[1] + m_R, m_grab[2]};
        Unit(a);
        Unit(b);
        double ax[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                        a[0] * b[1] - a[1] * b[0]};
        const double s = std::sqrt(Dot(ax, ax));
        if (s > 1e-12) {
            Unit(ax);
            const double ctr[3] = {0.0, -m_R, 0.0};
            m_pose = Motor::Rotation(ctr, ax, std::atan2(s, Dot(a, b))) * m_pose;
            m_following = false;   // the hand turned the planet: the eye stays where it is put
            BuildCamera();
        }
    }
    if (over && in.wheel != 0.0f && m_following && !Resetting()) {
        // Following: the wheel is the follow altitude, so the entity stays at the centre.
        m_followAlt = std::clamp(m_followAlt * std::pow(kWheelStep, double(in.wheel)), m_p.minAltM,
                                 8.0 * m_R);
        m_pose = Home(m_subject, m_followAlt);
        BuildCamera();
        in.wheel = 0.0f;
        took = true;
    }
    if (over && in.wheel != 0.0f) {
        // The translator along the cursor's ray: each notch keeps kWheelStep of the distance to
        // the point under it, floored at minAltM above the sea and capped at eight radii out.
        double d[3], tgt[3];
        Ray(r, in.mouseX, in.mouseY, d);
        HitSphere(d, tgt);
        const double v[3] = {tgt[0] - m_cam.px, tgt[1] - m_cam.py, tgt[2] - m_cam.pz};
        const double dist = std::sqrt(Dot(v, v));
        if (dist > 0.0) {
            const double alt = Altitude();
            double next = dist * std::pow(kWheelStep, double(in.wheel));
            // the altitude falls with the distance along this ray; keep it above the floor
            const double floorDist = dist - (alt - m_p.minAltM);
            next = (std::max)(next, (std::min)(floorDist, dist));
            next = (std::min)(next, dist + 8.0 * m_R - alt);
            const double move = dist - next;
            m_pose = Motor::Translation(v[0] / dist * move, v[1] / dist * move,
                                        v[2] / dist * move) * m_pose;
            m_resetT = -1.0f;
            BuildCamera();
        }
        in.wheel = 0.0f;
        took = true;
    }
    if (over || m_dragging || press) {
        // the first eye's gestures never see a press, a drag or a wheel that was the minimap's
        if (press || m_dragging) in.lmb = false;
        in.mouseDx = 0.0f;
        in.mouseDy = 0.0f;
        in.wheel = 0.0f;
        took = true;
    }
    return took;
}

Motor Minimap::Home(const double s[3], double alt) const {
    double n[3] = {s[0], s[1] + m_R, s[2]};   // the radial at the subject
    const double rs = std::sqrt(Dot(n, n));
    Unit(n);
    const double eye[3] = {n[0] * (rs + alt), n[1] * (rs + alt) - m_R, n[2] * (rs + alt)};
    const double f[3] = {-n[0], -n[1], -n[2]};
    // north up: the pole's part across the radial (at a pole, the frame's east)
    double u[3];
    const double pn = Dot(m_pole, n);
    for (int i = 0; i < 3; ++i) u[i] = m_pole[i] - pn * n[i];
    if (Dot(u, u) < 1e-12) {
        const double east[3] = {1.0, 0.0, 0.0};
        const double en = Dot(east, n);
        for (int i = 0; i < 3; ++i) u[i] = east[i] - en * n[i];
    }
    Unit(u);
    return View::Frame(eye, f, u);
}

void Minimap::Step(float dt, const double* subject) {
    if (subject) {
        for (int i = 0; i < 3; ++i) m_subject[i] = subject[i];
        m_hasSubject = true;
    }
    if (!m_placed) {
        m_pose = Home(m_subject, m_followAlt);   // the anchor until there is a subject
        m_placed = true;
    }
    if (m_resetT >= 0.0f) {
        // the walk home ends on the moving subject, and the eye follows it from there
        m_resetT += dt;
        const double u = std::clamp(double(m_resetT) / kResetSec, 0.0, 1.0);
        m_pose = Motor::Slerp(m_resetFrom, Home(m_subject, m_p.homeAltM), u * u * (3.0 - 2.0 * u));
        if (u >= 1.0) {
            m_resetT = -1.0f;
            m_following = true;
            m_followAlt = m_p.homeAltM;
        }
    } else if (m_following) {
        m_pose = Home(m_subject, m_followAlt);
    }
    BuildCamera();
}

// THE RASTERIZER BOUNDARY, without ToCamera's pitch clamp. The clamp (89.9 degrees) exists for a
// camera whose roll reference is the world's up, which is degenerate straight down; this eye's
// roll reference is its motor's own up, perpendicular to its aim, so straight down is an ordinary
// view -- and the clamp turned it 0.1 degree, 10 km at home (the [minimap] gate caught 0.7 px).
void Minimap::BuildCamera() {
    double p[3] = {0.0, 0.0, 0.0}, f[3] = {1.0, 0.0, 0.0}, up[3] = {0.0, 1.0, 0.0};
    m_pose.TransformPoint(p[0], p[1], p[2]);
    m_pose.TransformDir(f[0], f[1], f[2]);
    m_pose.TransformDir(up[0], up[1], up[2]);
    m_cam.px = p[0];
    m_cam.py = p[1];
    m_cam.pz = p[2];
    m_cam.yaw = static_cast<float>(std::atan2(f[2], f[0]));
    m_cam.pitch = static_cast<float>(std::atan2(f[1], std::sqrt(f[0] * f[0] + f[2] * f[2])));
    for (int i = 0; i < 3; ++i) m_cam.upHint[i] = static_cast<float>(up[i]);
}

double Minimap::Altitude() const {
    const double e[3] = {m_cam.px, m_cam.py + m_R, m_cam.pz};
    return std::sqrt(Dot(e, e)) - m_R;
}

bool Minimap::Project(const double p[3], const Rect& r, float& sx, float& sy) const {
    DirectX::XMFLOAT3 f, rt, u;
    m_cam.ViewBasis(f, rt, u);
    const double d[3] = {p[0] - m_cam.px, p[1] - m_cam.py, p[2] - m_cam.pz};
    const double z = d[0] * f.x + d[1] * f.y + d[2] * f.z;
    if (z <= 0.0) return false;
    const double th = std::tan(0.5 * m_cam.fovY);
    const double x = (d[0] * rt.x + d[1] * rt.y + d[2] * rt.z) / (z * th * (r.w / r.h));
    const double y = (d[0] * u.x + d[1] * u.y + d[2] * u.z) / (z * th);
    if (std::fabs(x) > 1.0 || std::fabs(y) > 1.0) return false;
    // Behind the limb: the segment from the eye meets the sphere (a hundred metres under the sea,
    // so the point itself never hides itself) before it reaches the point.
    const double dl = std::sqrt(Dot(d, d));
    const double dn[3] = {d[0] / dl, d[1] / dl, d[2] / dl};
    const double e[3] = {m_cam.px, m_cam.py + m_R, m_cam.pz};
    const double Rs = m_R - 100.0;
    const double b = Dot(e, dn);
    const double disc = b * b - (Dot(e, e) - Rs * Rs);
    if (disc >= 0.0) {
        const double t = -b - std::sqrt(disc);
        if (t > 0.0 && t < dl) return false;
    }
    sx = static_cast<float>(r.x + (x * 0.5 + 0.5) * r.w);
    sy = static_cast<float>(r.y + (0.5 - y * 0.5) * r.h);
    return true;
}

}  // namespace ga::scene
