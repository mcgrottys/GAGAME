// ================================================================================================
//  Scene - M12 step 5d: THE RESOLVED SCENE, TYPED. What the boot reads instead of `Options`.
//
//  Step 5a built the document (scene/SceneBuilder.h: defaults < the base file < each `include`
//  overlay < every --set, in command-line order) and the shim that writes a legacy flag line into
//  it (app/Options.cpp Options::ToSets); nothing read it. This is the read: ONE pass over the
//  resolved JsonValue into the section structs SceneSchema.h already declares, plus the six
//  named lists a document carries, so the assembly and the frame loop name a PROPERTY PATH
//  (`S.water.foam`, `S.capture.frames`, `S.streaming.tileBudget`) where they named a flag.
//
//  WHY THE SECTIONS COME BACK AS THE SCHEMA'S OWN STRUCTS. The table in SceneSchema.h binds every
//  key to a field of SceneDocument with its type, its quantity and its unit; PropSet::Merge is
//  the one parser (the number law, "12 kn", the enum names, the refusals) and ApplyTo the one
//  writer. Reading the document with a second hand-rolled reader is exactly the drift the water
//  scene's two readers had (Props.h's banner), so there is no second reader here: Scene IS a
//  SceneDocument, filled through its own table.
//
//  WHY THE LISTS ARE NOT. A PropSet does not carry a List (5a's law: lists are the document's
//  business), and an element's `at` is a PLACEMENT, which cannot resolve without the tangent
//  frame the Assembly only has after the surface is built. So each element keeps its declared
//  scalars (read through the element's own schema chain, by the SAME parser) and its `at` AS
//  WRITTEN, and the consumer reads the sugar's numbers where it stands -- scene::ReadPoseSugar,
//  the four spellings in their declared units. The session then calls SetFromCompass and
//  GlobeCamera with those numbers, which is what it called with the flags' numbers: 5d moves
//  WHERE a value comes from, not what is done with it.
//
//  WHAT STAYS IN `Options`. The pure instruments (--pix, --gpu-time, --bench, --no-vsync,
//  --res-trace, --thread-audit, --jobs-inline, --lens, --wireframe, --stencil, --albedo, --viz,
//  --inject, --debug, --predict-inline, --ds-serial, --dump-both, --dump-fibers, --dump-meshlets,
//  --probe-cull-far, --mesh-stats, --no-ms, --res-trace-frames) and the one-shot tools' own
//  arguments. Every other field the boot reads is a path in this document.
// ================================================================================================
#pragma once

#include "core/Json.h"
#include "scene/Props.h"
#include "scene/SceneSchema.h"

#include <string>
#include <vector>

namespace ga::app {

// ---- the named lists. Each element is its declaration plus, where it has one, the placement
// sugar exactly as written (a Placement needs a frame; see the banner).
struct SceneView {
    scene::ViewProps p;
    bool hasAt = false;
    JsonValue at;
    std::vector<std::string> interests;   // views[].interests, by name, in file order
};
struct ScenePortal {
    scene::PortalProps p;
    bool hasTo = false;   // toLat AND toLon declared: the portal has a destination
};
struct SceneGate {
    scene::GateProps p;
    bool hasAt = false;
    JsonValue at;     // the box's sugar as written (read where the flat frame exists)
    bool hasFrom = false;   // fromLat AND fromLon declared: the box stands at that place
    bool hasTo = false;     // toLat AND toLon declared: it leads there
    bool hasToAt = false;
    JsonValue toAt;   // where it comes out, as written (read where the destination's frame exists)
};
struct SceneEntity {
    scene::EntityProps p;
    bool hasAt = false;
    JsonValue at;
};
struct SceneInterest {
    scene::InterestProps p;
    bool hasAt = false;
    JsonValue at;   // the fixed place's sugar as written (read where the flat frame exists)
};
struct SceneEffect {
    scene::EffectProps p;
    double d = 0.0;          // slice.plane's cutaway offset (the only typed effect today)
};
struct SceneLayer {
    scene::LayerEntry p;
    float exaggeration = 60.0f;   // the tide layer's, its one typed key
    double levelHeight = 3.0, defaultHeight = 6.0, radius = 3000.0;   // the buildings layer's
    std::string lod;                                                  // ...and its far boxes'
    double lodRho0 = 4.0, lodPixels = 1.0;
};
struct SceneTool {
    std::string name, args;
};

// ================================================================================================
//  Scene - the resolved document as the boot reads it. The sections come from SceneDocument (so
//  `S.capture.frames`, `S.water.swe.spinupH`, `S.sea.storm.hs` are the file's own spelling); the
//  lists and the raw `water` section hang beside them.
// ================================================================================================
struct Scene : scene::SceneDocument {
    // The enum spellings, in the order the schema declares their names (SceneSchema.cpp).
    enum Mode { kChart = 0, kWorld = 1, kGulf = 2 };
    enum SunSource { kEphemeris = 0, kPinned = 1, kEarth = 2 };
    enum Rail { kRailNone = 0, kRailClassic, kRailZoom, kRailFlood, kRailJetty, kRailDroste,
                kRailDrosteOut };
    enum Controller { kHelm = 0, kFixed = 1 };
    enum Lighting { kRealistic = 0, kAppealing = 1 };

    std::string path;                 // the scene file this run resolved from
    // The wide spellings the engine's file APIs take, widened ONCE here (the legacy fields were
    // std::wstring and the shim narrowed them the same way: ASCII, character for character).
    std::wstring shadersW, dumpW, hdrW, railDirW;
    std::vector<SceneView> views;
    std::vector<ScenePortal> portals;
    std::vector<SceneGate> gates;
    std::vector<SceneEntity> entities;
    std::vector<SceneInterest> interests;
    std::vector<SceneEffect> effects;
    std::vector<SceneLayer> layers;   // THE REGISTRATION ORDER (= the draw order), as data
    std::vector<SceneTool> tools;
    std::vector<scene::SourceProps> sources;   // rasters that are sources by being files
    std::vector<scene::StationProps> stations;   // PHASE C3: stations placed by the scene
    JsonValue waterDoc;               // the `water` section as written (WaterComponent reads it)

    const SceneView* View(const std::string& name) const;
    const SceneView* Start() const;   // the view `scene.view` names, or null
    const ScenePortal* Portal(const char* name) const;
    const SceneEntity* Entity(const char* name) const;
    const SceneEffect* Effect(const char* name) const;
    const SceneEffect* EffectOfType(const char* type) const;   // the first enabled one
    const SceneLayer* Layer(const char* name) const;
    const SceneTool* Tool(const char* name) const;
    // Declared AND enabled: a layer the list does not carry, or carries disabled, is not built
    // and not registered. The list's ORDER is checked against the order the Assembly registers
    // in (LayerOrder / CheckLayerOrder), so the data and the code cannot drift silently.
    bool LayerOn(const char* name) const;
    std::vector<std::string> LayerOrder() const;   // the enabled names, in file order
};

// The resolved document, read into the typed scene. False = refused, `why` names the path.
bool ReadScene(const JsonValue& resolved, Scene& out, std::string* why);

// The `water` section reduced to what WaterComponent's table declares (wavefield, closures,
// fleet) -- the scene's spelling of data/wave_scene.json, which reaches it as an `include`.
JsonValue WaterSceneDoc(const JsonValue& waterSection);

// A one-shot mode's ARGUMENTS, as the flag would have set them: `--tool name:args` fills the
// Options field the legacy flag filled, so a tool reads one place whichever spelling ran it.
struct Options;
void ToolArgs(const Scene& S, Options& o);

// The registered draw order against the declared one. False = they disagree, `why` names the
// layer; the boot refuses with it.
bool CheckLayerOrder(const Scene& S, const std::vector<std::string>& registered, std::string* why);

}  // namespace ga::app
