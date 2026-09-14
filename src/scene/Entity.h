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
//  step, --boat-drive), and `mirrorCadence` -- the freshness contract below.
//
//  THE FRESHNESS CONTRACT (the review's finding, confirmed in the source). A hull queries the
//  weather manager every step (TreeWater::SlowAt -> WeatherManager::Query), and Query refines
//  the tide's level and adds the solved current from a CPU MIRROR of the solver's fields --
//  but the mirror is filled only by RefreshMirrorsTo, whose callers were four tools. In ordinary
//  play the mirror was never read: the hull rode the analytic tide and the waves and never the
//  solved level or the current, with nothing declaring so. The contract now: FrameInfo carries
//  `asOf`, the instant the physics snapshot is coherent at; the entity DECLARES a cadence
//  (`mirrorCadence`, seconds) and drives the refresh -- when it exists, and only when the mirror
//  is older than the cadence (WeatherManager::RefreshOnCadence: one readback per cadence, never
//  one per hull step); and TreeWater::Describe reports the age it reads. The default is 0 =
//  NEVER, which is today's cost and today's physics exactly: the effective cadence the engine
//  shipped with was infinite. A cadence of 1 s makes the hull read the solved water, and that is
//  a PHYSICS change by design, measured on the helm boat's telemetry and reported, not slipped in.
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

class Entity final : public Component {
public:
    // The water the hull reads and the layer that draws it. Every one nullable: absence is
    // reported by Configure and TreeWater reports a missing term as valid = false, never as
    // flat water (WaterSurfaceTree.h's banner).
    struct Observers {
        WeatherManager* weather = nullptr;        // THE tree's point evaluator (and the mirror)
        const WaveField* waveField = nullptr;     // the solved field, when the session built one
        SeaLayer* sea = nullptr;                  // its cascades (Ocean()) and heightScale
        const SeaState* seaState = nullptr;
        const WaterSceneConfig* waterScene = nullptr;   // wfExag / wfChop, live (hot-reloaded)
        VesselLayer* vesselLayer = nullptr;
        Gpu* gpu = nullptr;                       // the mirror refresh's readback
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
    void ChaseFrame(double p[3], double f[3]);
    // The hull steps' wall time this Update (the frame loop's profiler slot).
    double LastStepMs() const { return m_stepMs; }
    // The declared cadence, seconds; 0 = never (today's behaviour).
    double MirrorCadence() const { return m_props.mirrorCadence; }

private:
    EntityProps m_props;
    Motor m_spawn;
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
};

}  // namespace ga::scene
