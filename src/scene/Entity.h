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
//  manager every step (TreeWater -> WaterManager::Query), and inside a solver's domain the
//  water's surface and current are the SOLVER'S. The solver lives on the GPU, so the hull ASKS for
//  the region around itself every frame (WaterManager::RequestRegion: the texels copied through
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

#include "core/OriginLattice.h"
#include "core/Pga.h"
#include "scene/Component.h"
#include "scene/SceneSchema.h"
#include "sim/Vessel.h"
#include "sim/WaterSurfaceTree.h"

#include <memory>
#include <string>
#include <vector>

namespace ga {
class BuildingLayer;
class BuildingStack;
class Gpu;
class SeaLayer;
class SeaState;
class VesselLayer;
class VesselRegistry;
class WaveField;
class WaterManager;
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
        WaterManager* weather = nullptr;        // THE tree's point evaluator (and the solver's regions)
        const WaveField* waveField = nullptr;     // the solved field, when the session built one
        SeaLayer* sea = nullptr;                  // its cascades (Ocean()) and heightScale
        const SeaState* seaState = nullptr;
        const WaterSceneConfig* waterScene = nullptr;   // wfExag / wfChop, live (hot-reloaded)
        VesselLayer* vesselLayer = nullptr;
        Gpu* gpu = nullptr;                       // the device the hull's water lives beside
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
        // THE SOLIDS (2026-10-12): the buildings stack, composed about the hull on the pool and
        // read by its ground law as prisms in its own frame (sim/Solids.h). Null: the bed alone.
        const BuildingStack* buildings = nullptr;
        // ...and the layer whose resident pages hold the bridge decks and ramps (DecksNear).
        const BuildingLayer* buildingLayer = nullptr;
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
    // The spawn was written about the scene's place: said in the root by its motor.
    void CarrySpawn(const Motor& toRoot) { m_spawn = toRoot * m_spawn; }
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
    // PLACED (2026-10-11): at a pose, at rest, controls cleared, seated on its surface by the next
    // Update facing that way -- ResetAtRest at a pose. The vehicle cycle's one verb.
    void PlaceAt(const Motor& pose);
    // ...over another entity: its CG and heading said in this hull's space, upright; the seat
    // then puts it at its own seatHeight over the surface there (a drone six metres up, a boat
    // on the water, a car on the ground, by the one law).
    void PlaceOver(const Entity& other);
    // ---- FORMS (parts plus forms, 2026-10-12): an entity declared with `forms` is one body whose
    // T / Y take the next form (Vessel::SetForm: the origin moves to the form's CG, the pose is
    // carried, the momenta rested) and whose seat then puts that form where it belongs -- the
    // drone six metres up, the boat at its draught, the car on its tyres -- by the one seat law.
    // False when the entity has one form (the loop then cycles entities).
    bool CycleForm(int dir);
    size_t FormCount() const;
    // The reference model of the CURRENT form (empty: none), for the layer every publish.
    const std::string& MeshFile() const;
    double MeshScale() const;
    double MeshYaw() const;
    // The CG and horizontal heading said in `s` (null: the root), through the common ancestor.
    bool FrameIn(const Space* s, double cg[3], double fwd[3]) const;
    // The hull's origin and horizontal heading, for the chase camera; keeps helmYawRef.
    // In the hull's OWN space (the root's, until a gate carries it).
    void ChaseFrame(double p[3], double f[3]);

    // ---- THE HULL'S SPACE (the discipline, 2026-10-11) -------------------------------------------
    // A hull stands on a point of the planet's cloud, its own, chosen by its position alone
    // (core/OriginLattice.h; Recentre): it knows no gate, no portal and no root threshold. A gate
    // is a motor between two spaces of the planet, and the frame loop carries whatever enters its
    // box by it (StepEntities); the hull's part is Carry, the same for any motor into any space.
    // The space the hull stands in: null until its first step (the root's, where it was spawned).
    const Space* InSpace() const { return m_space; }
    // That space's placement in the root's frame (identity until it has one of its own).
    const Motor& SpaceInRoot() const { return m_spaceInRoot; }
    // CARRIED BY A MOTOR INTO A SPACE: pose and momenta by `m` (a pose of the space it stands in
    // -> a pose of `into`), then the hull stands on `into`'s cloud point and re-picks its own by
    // its position, keeping its height above its own water across the whole of it (the free
    // surface onto the free surface: two seas, two tides). False when there is no hull.
    bool Carry(const Space& into, const Motor& m, double simUnix);
    // The centre of gravity said in `s`, through the common ancestor. False without a hull.
    bool CgIn(const Space& s, double out[3]) const;
    // The spec's reach from the CG, and the CG's speed -- what a gate's residency asks with.
    double Reach() const;
    double SpeedMps() const;
    // How many times it has been carried, so a view can notice a new one.
    uint32_t Carries() const { return m_carries; }
    // THE PER-SUBJECT FLOATING ORIGIN: the hull's space is the cloud's point under it.
    void Recentre(double simUnix);
    // WHAT THE VIEW DRAWS: Hull(), in the frame it stands in (its space's placement in the root,
    // identity in the root itself). The loop hands it to the vessel layer once per world the view
    // reaches through the gates -- after the view has decided what it looks through this frame --
    // so a boat is seen wherever a window shows its place, its own included.
    Motor DrawFrame() const { return m_space ? m_spaceInRoot : Motor::Identity(); }
    // The hull steps' wall time this Update (the frame loop's profiler slot).
    double LastStepMs() const { return m_stepMs; }

private:
    void StepTelemetry(TreeWater& boatSea, double simUnix);
    // The solids about the hull: ask the stack again when the hull has moved a third of the reach
    // (a pool job; published whole), and convert the set into the hull's frame when the set or the
    // hull's chart changed. The prisms the step reads.
    void SolidsStep(double simUnix);
    struct SolidsJob;
    std::shared_ptr<SolidsJob> m_solidsJob;
    std::shared_ptr<const void> m_solidsSeen;      // the set the prisms were made from
    std::vector<SolidPrism> m_prisms;
    uint32_t m_prismsChart = 0;                    // the chart generation they were made in
    uint64_t m_prismsPages = 0;                    // the layer's page set they were made from
    uint32_t m_stands = 0;                         // StandOn count: the chart's generation
    static constexpr double kSolidsReachM = 80.0;

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
    bool m_heldNoWater = false;  // the step is held: the water under the hull is absent (Update)
    int m_telTick = 0;           // one telemetry line a second
    double m_stepMs = 0.0;
    const Space* m_space = nullptr;   // null until the first step: the root's, where it was spawned
    uint32_t m_carries = 0;           // how many times a motor has carried it
    // The hull's own space: a point of the cloud (StandOn), its chart the point's tangent frame.
    Space m_ownSpace;
    Space::Anchor m_ownChart;
    OriginLattice::Origin m_origin{};
    bool m_ownValid = false;
    bool m_standRefused = false;      // a stand Declare refused: said once, then the hull keeps its frame
    Motor m_spaceInRoot;              // own -> root; identity until it has a space of its own
    // Stand on a space of the cloud (its stand IS the origin): the members above, the water's chart.
    bool StandOn(const Space& s);
};

}  // namespace ga::scene
