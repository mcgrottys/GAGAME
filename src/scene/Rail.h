// ================================================================================================
//  Rail - M12 step 5e: A CAMERA RAIL IS SEGMENTS, AS DATA, AND ONE PURE FUNCTION OF TIME.
//
//  The engine's rails were five hand tables and six lambdas in the session (railKeys, railPose,
//  diveFrom/diveAt, legU, diveU, keyedPose, gravityUp, drosteRailPose -- FrameLoop::Session at
//  c2813b8), one table per flag, the Droste dive's spiral and turn hard-wired around them. They
//  are now FILES -- scenes/rails/<name>.json, one per rail, `rails.active` naming the one flown
//  -- and this class is the reading of one file into segments and the evaluation At(t). The
//  values in the files were PRINTED from the tables (the [scene] gate still prints them, from
//  its own transcription of the tables, and holds the text equal to the checked-in file byte
//  for byte), and At(t) is held bitwise equal to the lambdas at a thousand instants.
//
//  THE SEGMENTS, in file order, each with a duration (0 = the rest of the rail), consumed by
//  subtracting durations in sequence -- which is how the hand-written dive rail walked its own
//  legs (`tau -= 14; if (tau < 2) ...`), and why a leg's own clock starts at zero:
//
//      keys     poses at times; between two keys the SCREW SLERP (Pga.h Motor::Slerp: M0
//               Exp(u Log(~M0 M1)), position and aim carried together on one helix) with the
//               leg eased at both ends (u -> u^2 (3 - 2u)); past the last key the pose holds.
//               `from` is the key time the segment starts at (the out-and-back's first leg is
//               the flood keys from 26 s). A key is a placement in the sugar, or `view` (the
//               eye a view declares -- the orbit start), or `pose` (a named tower pose).
//      spiral   pose(u) = S^u(base): the tower's own one-parameter subgroup, through the
//               portal's cycle (Space::LevelApply / LevelApplyDir -- the power about its fixed
//               point), with n = floor(u) handed to the frame and only f = u - n pushed through
//               the versor. u(t) is a LAW: `dive` (eased in over 3 s, then one level per
//               levelSec, held on a whole level when the frame says the dive ends on a helm) or
//               `leg` (u0 -> u1 over the leg with 3 s velocity ramps at both ends, LegU; a leg
//               that runs u DOWN is the same law with the ends swapped, and the arithmetic is
//               the hand-written `2 - legU(tau, D, 2)` exactly). `levels` gives the duration in
//               levels of the dive's pace (2 levels = 2 x levelSec).
//      hold     S^0(base) for the duration: the spiral at u = 0 (which is NOT the base pose bit
//               for bit -- p + (x - p) -- and was never meant to be: the hold is where the
//               probe watches the versor leave the helm).
//      turn     from one tower pose to another over the duration -- the same eased screw slerp
//               as two keys at 0 and D -- at a declared level, with a declared pose's up.
//
//  A TOWER rail (the two Droste rails) writes the camera's LEVEL and its UP every frame: the
//  spiral carries roll exactly (the up is Q^f of the base pose's own up), the turn holds the
//  helm's, and a keyed leg takes anti-gravity at the eye. A plain rail writes neither; the
//  frame's ground field supplies the up (FrameLoop, drosteRailUp).
//
//  THE POSES a tower rail names (`poses`): a camera in the sugar, optionally AIMED at the
//  portal's fixed point with the stand-off rule the dive was measured with (M10: the helm's
//  spiral clears the inner globe only if the twist lifts it over -- clearance/distance -0.21 at
//  0 deg, +0.012 at 90 -- so an untwisted tower dives from above, 300 m up and 300 m west of
//  the fixed point), or turned about the local vertical from another pose (the out-and-back's
//  return face of the helm). A pose resolves to the CAMERA the session built (`drosteHelm`),
//  its motor, and the spiral's base: the eye, Camera::Forward() and the ViewBasis up, exactly
//  the three the dive read.
//
//  WHAT At(t) RETURNS, AND WHY IT IS AN EYE AND AN AIM. The keyed legs interpolate MOTORS; the
//  spiral maps an eye and a forward through the versor. Both reach the rasterizer as the same
//  five numbers, and the hand code turned each into yaw and pitch by the one formula
//  (scene::ToCamera's atan2 pair) -- so the sample is the eye and the aim, the motor beside it
//  for the record, and AimCamera is that formula said once. No 4x4 anywhere; the algebra is
//  Pga.h's screw and Space.h's similarity power, as the plan requires.
//
//  Prior art, named. Keyframed camera paths are the animation curve of every DCC (Maya, Blender)
//  and the game engines' cinematic tracks (Unreal's Sequencer, Godot's AnimationPlayer: a track
//  is data, an evaluator samples it); the screw slerp between key motors is the dual-quaternion
//  interpolation of Kavan et al. 2008 (ScLERP), which this engine took from the motor manifold
//  directly. The spiral is Hart & DeFanti 1991 (a cycle in a scene graph, instanced until it is
//  smaller than a pixel -- the Droste tower itself), and the rail's data form follows Godot and
//  USD, where a scene's tracks are file contents rather than code. What is this engine's own:
//  the dive leg as a one-parameter subgroup of the portal's similarity, evaluated about its
//  fixed point (Space.h PowApply).
// ================================================================================================
#pragma once

#include "core/Json.h"
#include "core/Pga.h"
#include "core/Space.h"
#include "render/Camera.h"
#include "scene/Props.h"

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace ga::droste {
struct Portal;
}

namespace ga::scene {

// What a rail resolves IN: the frame the keys' sugar resolves in, the tower it is flown through,
// the dive's declared pace, the session camera's optics, and the views a key may name.
struct RailFrame {
    PoseFrame frame;                          // planetR + the tangent rows (the sugar's frame)
    float fovY = 0.9f;                        // the session camera's optics, carried by a tower pose
    const droste::Portal* portal = nullptr;   // the link; null or invalid = no tower
    const Space* cycle = nullptr;             // its Space::Cycle: Level(k) = S^k
    double twistDeg = 0.0;                    // the portal's declared twist (the stand-off rule)
    double levelSec = 16.0;                   // rails.droste.levelSec: the dive's pace
    int levels = 3;                           // rails.droste.levels: how deep the dive goes
    bool clampLevels = false;                 // the dive law ends ON a helm (rails.active == droste)
    // A view's declared eye, by name (the `view` key): null when the view declares none.
    std::function<const JsonValue*(const std::string&)> viewAt;
};

// A named pose of the tower, resolved: the camera the session built, its motor, and the
// spiral's base -- the eye, Camera::Forward() and the ViewBasis up, the three the dive read.
struct RailPose {
    std::string name;
    Camera cam;
    Motor motor;
    double c0[3] = {0.0, 0.0, 0.0};
    double f0[3] = {1.0, 0.0, 0.0};
    double up0[3] = {0.0, 1.0, 0.0};
};

// One instant of a rail: the pose as an eye and an aim, the level it is written in, the up
// the rail wrote (tower rails), and -- for a spiral sample -- u, f and the poses, so the
// frame loop's [droste] probe can evaluate the same versor beside it.
struct RailSample {
    double eye[3] = {0.0, 0.0, 0.0};
    double fwd[3] = {1.0, 0.0, 0.0};
    int level = 0;
    double up[3] = {0.0, 1.0, 0.0};
    bool upWritten = false;
    bool spiral = false;
    double u = 0.0, f = 0.0;
    const RailPose* base = nullptr;
    const RailPose* upOf = nullptr;
    Motor motor;   // the keyed legs' screw (identity for a spiral sample): the record
};

class Rail {
public:
    enum Kind { kKeys = 0, kSpiral = 1, kHold = 2, kTurn = 3 };
    enum Law { kDive = 0, kLeg = 1 };

    // scenes/rails/<name>.json: parsed, completed against the rail file's schema, validated (an
    // unknown key refuses naming its path), read. False = refused, `why` says.
    bool Load(const std::string& path, std::string* why);
    bool FromJson(const JsonValue& doc, const std::string& from, std::string* why);
    // The keys' motors and the tower poses, in a frame. Must precede At.
    bool Resolve(const RailFrame& frame, std::string* why);

    bool Loaded() const { return !m_segments.empty(); }
    bool Resolved() const { return m_resolved; }
    bool Tower() const { return m_tower; }
    const std::string& Name() const { return m_name; }
    const std::string& Path() const { return m_path; }
    // The declaration as read (completed): SceneBuilder::WriteJson of it is the canonical text.
    const JsonValue& Document() const { return m_doc; }
    const RailPose* Pose(const std::string& name) const;
    size_t SegmentCount() const { return m_segments.size(); }

    // THE RAIL AT t SECONDS. Pure: reads the resolved data and nothing else.
    RailSample At(double t) const;
    // The leading keys flown at absolute t, level and up untouched: what a tower rail is
    // without a valid portal (the hand code fell back to railPose(t)).
    RailSample KeysAt(double t) const;

    // ---- the laws, pure and exported (the [scene] gate and the [droste] sweep read them)
    // The dive's clock: eased in over the first 3 s (C1: the camera leaves the helm from rest),
    // then one level per levelSec; clamped at `levels` when the dive ends on a helm.
    static double DiveU(double tau, double levelSec, int levels, bool clamp);
    // A leg that moves u by dU in D seconds with 3 s velocity ramps at both ends.
    static double LegU(double tau, double D, double dU);
    // The eased screw slerp over a key table at time t; past the last key, the last key.
    static void KeyedPose(const std::vector<std::pair<double, Motor>>& keys, double t,
                          double eye[3], double fwd[3], Motor* motor);
    // Anti-gravity at an eye: radial from the planet's centre at (0, -R, 0).
    static void GravityUp(const double eye[3], double planetR, double up[3]);
    // The rasterizer boundary for a sample: the eye and the aim into the camera's five numbers
    // (scene::ToCamera's own atan2 pair); a spiral sample first takes its base camera whole,
    // as the hand code did (`out = base`).
    static void AimCamera(const RailSample& s, Camera& cam);

    // The rail file's vocabulary (scene/Props.h tables), for the tests and --print-scene.
    static const Schema& FileSchema();
    static const Schema& PoseSchema();
    static const Schema& SegmentSchema();

private:
    struct Segment {
        int kind = kKeys;
        double duration = 0.0;   // seconds; 0 = open-ended
        double from = 0.0;       // keys: the key time the segment starts at
        int up = 0;              // keys: none | gravity
        std::vector<std::pair<double, Motor>> keys;
        std::vector<double> keyT;
        std::vector<JsonValue> keyAt;   // the keys' sugar, as written (resolved in a frame)
        std::vector<bool> keyHasAt;
        std::vector<std::string> keyView, keyPose;
        std::string base, upOf, to;
        int law = kDive;
        double u0 = 0.0, u1 = 0.0, delay = 0.0;
        int levels = 0;          // spiral: the duration in levels (0 = `duration`)
        int level = 0;           // turn: the level the turn is written in
        // resolved
        double durationS = 0.0;
        double start = 0.0;      // the sum of the durations before it (the dive's clock origin)
        const RailPose* basePose = nullptr;
        const RailPose* upPose = nullptr;
        const RailPose* toPose = nullptr;
    };
    struct PoseDecl {
        std::string name;
        bool hasAt = false;
        JsonValue at;
        int aim = 0;                       // none | fixedPoint
        double standOffBelowTwistDeg = 0.0;
        double standOff[3] = {0.0, 0.0, 0.0};
        std::string turnOf;
        double turnYaw = 0.0, turnPitch = 0.0;
    };

    RailSample Eval(const Segment& s, double t, double tau) const;
    bool ResolvePose(const PoseDecl& d, const RailFrame& frame, RailPose& out,
                     std::string* why) const;
    bool KeyMotor(const Segment& s, size_t i, const RailFrame& frame, Motor& out,
                  std::string* why) const;

    std::string m_name, m_path;
    bool m_tower = false;
    bool m_resolved = false;
    JsonValue m_doc;
    std::vector<PoseDecl> m_poseDecls;
    std::vector<RailPose> m_poses;
    std::vector<Segment> m_segments;
    RailFrame m_frame;
};

// The rigid placement sugar as the MOTOR the session's pose maps make -- scene::FromCamera of
// the camera the spelling builds (Props.cpp ResolveMotor, before Placement::Rigid narrows it).
// A rail key must carry this motor and not the placement's: the round trip through Rigid and
// back re-unitizes nothing and the keys were the session's own motors.
bool ResolveMotorSugar(const JsonValue& sugar, const PoseFrame& frame, const std::string& path,
                       Motor& out, std::string* why);

}  // namespace ga::scene
