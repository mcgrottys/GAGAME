// ================================================================================================
//  Entity - M12 step 5e: ONE VESSEL, ITS STEP, AND THE WATER IT READS -- AS A SCENE NODE.
//
//  The boat lived in the session as eleven members and one lambda: the Vessel, its TreeWater,
//  its controls, `boatPlaced`, `helming`, `helmYawRef`, the two function-local statics
//  (quantaOwed, telTick) and `stepBoat`, called from both clock branches of the frame
//  (FrameLoop at c2813b8). Every one of those is PER-VESSEL state, and holding them in the loop
//  is what made a second hull a code change. This component is that state and that step, moved
//  VERBATIM: Update(f) is stepBoat's body, the members are the lambda's captures, and two boats
//  are two nodes -- each with its own TreeWater, whose memo is single-threaded by contract
//  (sim/WaterSurfaceTree.h) and must not be shared between hulls stepping in one tick.
//
//  WHAT A NODE DECLARES (SceneSchema.h EntitySchema, the `entities` section): `vessel` (the hull
//  kind, VesselRegistry), `at` (the spawn, a motor in the placement sugar -- {x, alt, z} is a
//  pure translation, which is how --campos spawned it), `controller` (helm: the keyboard drives
//  the levers and the outboard angle, Helm(); fixed: the declared throttle and steer every
//  step, --boat-drive).
//
//  THE SOLVER IS TRUTH (the water match, step 1 -- Mark, 2026-09-15). A hull queries the weather
//  manager every step (TreeWater -> WeatherManager::Query), and inside a solver's domain the
//  water's surface and current are the SOLVER'S. The solver lives on the GPU, so the hull ASKS for
//  the region around itself every frame (WeatherManager::RequestRegion: the texels copied through
//  the frame ring and delivered two frames later, never by a stall) and reads the delivered texels
//  through the kernel's own reconstruction. FrameInfo carries `asOf`, the instant those answers are
//  coherent at, and TreeWater::Describe reports it. This replaced the step 5e cadence
//  (`mirrorCadence`, a whole-field mirror read on a clock, default never): measured by
//  --water-probe, a hull that never read the solver rode 0.39 m below the sea the renderer drew.
//
//  The chase camera stays a Follow on the View (scene/View.h): the entity hands the view its
//  hull's origin and heading (ChaseFrame, which also keeps the look-steer's heading reference
//  here rather than on the live camera), and View::Follow places the eye. One residue, stated:
//  the vessel layer takes ONE list of hulls (VesselLayer::SetVessels) and this step keeps the
//  hand code's call from the step, so with two entities the layer draws the last one to step --
//  the loop should own that list when a second hull exists (a follow-on, not a silent change).
//
//  Prior art, named: the Entity-Component pattern of Unity (a MonoBehaviour owning a Rigidbody)
//  and Godot (a RigidBody node with its own physics state); Unreal's Pawn + PlayerController
//  split for the helm/fixed/net controllers the plan names. The freshness contract is the
//  ordinary "snapshot with a timestamp" of any simulation consumer; what is this engine's own is
//  that the cadence is scene data and absence is declared rather than zero.
// ================================================================================================
#pragma once

#include "core/Pga.h"
#include "scene/Component.h"
#include "scene/SceneSchema.h"
#include "sim/Vessel.h"
#include "sim/WaterSurfaceTree.h"

#include <memory>
#include <string>
#include <vector>

namespace ga {
class Gpu;
class SeaLayer;
class SeaState;
class VesselLayer;
class VesselRegistry;
class WaveField;
class WeatherManager;
struct InputState;
struct WaterSceneConfig;
}  // namespace ga

namespace ga::scene {

class Gateway;

class Entity final : public Component {
public:
    // The water the hull reads and the layer that draws it. Every one nullable: absence is
    // reported by Configure and TreeWater reports a missing term as valid = false, never as
    // flat water (WaterSurfaceTree.h's banner).
    struct Observers {
        WeatherManager* weather = nullptr;        // THE tree's point evaluator (and the solver's regions)
        const WaveField* waveField = nullptr;     // the solved field, when the session built one
        SeaLayer* sea = nullptr;                  // its cascades (Ocean()) and heightScale
        const SeaState* seaState = nullptr;
        const WaterSceneConfig* waterScene = nullptr;   // wfExag / wfChop, live (hot-reloaded)
        VesselLayer* vesselLayer = nullptr;
        Gpu* gpu = nullptr;                       // the device the hull's water lives beside
        // The gates a hull in the ROOT space can be carried through (scene/Gateway.h). Tested
        // inside the step, before the hull is published to the vessel layer, so the frame that
        // carries it draws it where it now is.
        const std::vector<std::unique_ptr<Gateway>>* gates = nullptr;
        // The swell shadow the bank reads, for the hull's water (compose/ExposurePage). Null: exposed.
        const PlaceField* swellShadow = nullptr;
        // The bed the water kernels read, for the hull's depth laws (compose/HeightPage). Null: the
        // slow field's bed.
        const PlaceField* bed = nullptr;
        // M13 step 2: the ROOT space's chart, with the renderer's own frame rows and the planet's
        // radius on it (Space::Anchor::PlaceOf). A hull in the root space reads its places through
        // this; a carried hull through its gate's, set at the carry. Null: the anchor-linear law,
        // which is what a hull got before the places were exact.
        const Space::Anchor* rootChart = nullptr;
        // ... and the two spaces the per-subject floating origin is built between: the planet a
        // hull's own tangent frame hangs under, and the root frame the scene is drawn in.
        const Space* planet = nullptr;
        const Space* rootSpace = nullptr;
    };

    // ---- Component -------------------------------------------------------------------------
    const char* Name() const override { return m_props.name.c_str(); }
    const Schema& Props() const override { return EntitySchema(); }
    std::vector<std::string> Configure(const Wiring& w) override;
    bool Init(Gpu& gpu) override;
    void Apply(const PropSet& props) override;
    // stepBoat, verbatim: the set-down, the fixed controller, the 60 Hz hull steps off the
    // clock's whole quanta (FrameInfo::quanta), the telemetry line a second, the vessel layer.
    void Update(const FrameInfo& f) override;
    void Record(const ViewContext& v) override;
    void ReloadShaders() override {}

    // ---- the declaration, the spawn, the boot ------------------------------------------------
    EntityProps& Declared() { return m_props; }
    const EntityProps& Declared() const { return m_props; }
    // The spawn in the FLAT world frame (the sugar's {x, alt, z}: a translation).
    void SetSpawn(double x, double y, double z);
    // ...and the bow's compass heading when the sugar says one (`az`: 0 north, 90 east).
    void SetSpawn(double x, double y, double z, double headingDeg);
    const Motor& Spawn() const { return m_spawn; }
    // The hull from the registry at the spawn: the session's boot block. False = not spawned.
    bool Spawn(const VesselRegistry& reg);
    // The water and the layer. Returns what it was NOT given (logged here too, once).
    std::vector<std::string> Configure(const Observers& o);

    // ---- the running state ------------------------------------------------------------------
    bool Active() const { return m_boat != nullptr; }
    const Vessel* Hull() const { return m_boat.get(); }
    bool Helming() const { return m_helming; }
    void SetHelming(bool on) { m_helming = on; }
    bool Placed() const { return m_placed; }   // set down on the water: the step ran
    double HelmYawRef() const { return m_helmYawRef; }
    VesselControls& Controls() { return m_ctl; }
    const TreeWater& Sea() const { return m_sea; }
    // THE HELM CONTROLLER: the keyboard block, verbatim (T detaches the camera; W/S drive both
    // levers, Q/E split them; A/D the outboard angle at its mechanical rate; SHIFT/CTRL trim).
    void Helm(const InputState& in, float dt);
    // THE CLOCK POLICY: a deliberate time jump resets the hull to rest at its last pose.
    void ResetAtRest();
    // The hull's origin and horizontal heading, for the chase camera; keeps helmYawRef.
    // In the hull's OWN space (the root's, until a gate carries it).
    void ChaseFrame(double p[3], double f[3]);

    // ---- THE GATE (scene/Gateway.h) --------------------------------------------------------------
    // The space the hull lives in: null is the root tangent space. After a gate, the destination.
    const Space* InSpace() const { return m_space; }
    // That space's placement in the root's frame (identity when it is the root).
    const Motor& SpaceInRoot() const { return m_spaceInRoot; }
    // Carry the hull through `gate`: pose and momenta by the gate's motor, the free surface onto
    // the free surface (the hull keeps its height above its own water), the water read through
    // the destination's chart. Returns false when there is no hull to carry.
    bool Teleport(const Gateway& gate, double simUnix);
    // WHICH GATE LAST CARRIED IT, and how many times it has been carried. The VIEW reads these:
    // a chase eye does not teleport with its subject, it follows the subject THROUGH the window
    // and crosses on its own when the box takes it (FrameLoop's m_eyeOwes), so it has to know
    // which window the subject went through and when that changed.
    const Gateway* LastGate() const { return m_lastGate; }
    uint32_t Carries() const { return m_carries; }
    // THE VIEW'S CARRY, set by the loop each frame before the step. While an eye still stands on
    // this side of a gate its subject has crossed, the subject is DRAWN at its apparent pose --
    // pulled back through the gate's motor, which is the same map the window's geometry uses --
    // and its pixels are kept only where the window shows them. Null pull = the hull draws in
    // the frame it actually stands in, which is every hull the eye shares a space with.
    void SetViewCarry(const Motor* pull, bool through) {
        m_viewPull = pull ? *pull : Motor::Identity();
        m_viewPullOn = pull != nullptr;
        m_viewThrough = through;
    }
    // M13 step 2: the per-subject floating origin -- the hull's space follows the hull.
    void Recentre(double simUnix);
    // Hand the hull to the vessel layer in the frame the VIEW is looking through this frame.
    // Called by the step, and again by the loop once the chase has decided (SetViewCarry).
    void PublishDraw();
    // The hull steps' wall time this Update (the frame loop's profiler slot).
    double LastStepMs() const { return m_stepMs; }

private:
    void StepTelemetry(TreeWater& boatSea, double simUnix);

public:

private:
    EntityProps m_props;
    Motor m_spawn;
    // The heading the set-down seats the hull at: the spawn's declared `az` on its first
    // placement, the heading it had on a re-seat after a time jump. A spawn that declared none
    // is seated by the translation alone on its first placement, byte for byte as before.
    bool m_spawnTurned = false;
    double m_spawnHeadingRad = 0.0;
    bool m_reseatKeepsYaw = false;
    Observers m_o;
    bool m_wired = false;

    // ---- stepBoat's captures, per hull -----------------------------------------------------
    std::unique_ptr<Vessel> m_boat;
    TreeWater m_sea;             // its own memo: single-threaded by contract
    VesselControls m_ctl;
    bool m_placed = false;       // set down on the surface at the first step, see Update
    bool m_helming = false;      // T detaches the camera; the physics never stops
    double m_helmYawRef = 0.0;   // the look-steer's heading reference -- see the note in
                                 // ChaseFrame for why it is NOT read off the live camera
    int m_quantaOwed = 0;        // the 240 Hz quanta owed to the 60 Hz hull step
    int m_telTick = 0;           // one telemetry line a second
    double m_stepMs = 0.0;
    const Space* m_space = nullptr;   // null = the root tangent space
    const Gateway* m_lastGate = nullptr;   // the gate that last carried it (the view reads this)
    uint32_t m_carries = 0;                // how many times, so a view can notice a new one
    Motor m_viewPull = Motor::Identity();  // the view's pull-back through the windows it owes
    bool m_viewPullOn = false;
    bool m_viewThrough = false;
    // M13 step 2: the space this hull carries with it (Recentre) and its chart. m_space points at
    // m_ownSpace once it has re-centred, or at a gate's destination after a carry.
    Space m_ownSpace;
    Space::Anchor m_ownChart, m_gateChart;
    bool m_ownValid = false;
    Motor m_spaceInRoot;               // identity until a gate carries the hull
};

}  // namespace ga::scene
