// ================================================================================================
//  WeatherManager - M6x: THE GLOBAL WEATHER/PHYSICS MANAGER -- the Vatican client for water
//  and near-surface weather. ONE query surface over every product the monasteries publish:
//
//      Query(lat, lon, t, groundResM) -> { level, current, waves, wind, bed, depth }
//
//  each component answered by the FINEST RESIDENT RUNG that covers the point -- the texture
//  doctrine applied to physics. Stateless global fields carry the world (the tide atlas'
//  constituent rotors, GFS wind vectors, GFS-Wave Hs, GoMOFS surface currents); STATEFUL
//  solver windows refine exactly where they are resident (the Merrimack SWE; Boston Harbor
//  spins up when the camera arrives -- detail rises on zoom because RESIDENCY rises on zoom,
//  the same statement the tiled atlas makes about texels). Every component reports its
//  PROVENANCE: which source, which rung -- heterogeneity is queryable, not hidden.
//
//  "NOAA carries the state, the GPU carries the phase": the manager is overwhelmingly a
//  stateless evaluator; the solver windows are the sparse exception, and their absence is
//  not an error -- it is the tide plane being exactly right.
// ================================================================================================
#pragma once

#include "compose/Compositor.h"
#include "compose/WaterAtlas.h"
#include "sim/BathyModel.h"
#include "sim/CurrentModel.h"
#include "sim/GlobeModel.h"
#include "sim/SeaState.h"
#include "sim/SweSolver.h"
#include "sim/TideModel.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ga {

struct WeatherSample {
    // water
    double levelNavd = 0;             // surface elevation (tide + solver deviation)
    float u = 0, v = 0;               // surface current, m/s east/north
    float hs = 0, tp = 0, dirDeg = 0; // waves: significant height, peak period, FROM deg
    float bedNavd = 0;                // the one bed (the composed height stack)
    float depthM = 0;                 // level - bed; <= 0 means dry here, now
    // air
    float windU = 0, windV = 0;       // 10 m wind, m/s
    // provenance: the rung that answered each component (static strings)
    const char* levelSrc = "-";
    const char* currentSrc = "-";
    const char* waveSrc = "-";
    const char* windSrc = "-";
    const char* bedSrc = "-";
};

class WeatherManager {
public:
    void Init(Compositor* comp, int heightChannel, const WaterAtlas* atlas,
              const TideModel* tides, const GlobeModel* globe, const SeaState* sea,
              const CurrentModel* currents);

    // A stateful window. External windows (the Merrimack: driven by the render loop) are
    // mirrored only; OWNED windows (Boston) are advanced by Update() and spun up lazily
    // when the camera enters their footprint below the activation altitude.
    void AddExternalWindow(const char* name, SweSolver* solver, const BathyModel* bathy,
                           std::function<double(double)> oceanAt);
    void AddDormantWindow(const char* name, BathyModel* bathy, const SweConfig& cfg,
                          std::function<double(double)> oceanAt, double spinupHours);

    // Per frame (or before a physics batch): lazy activation, owned-solver advancement,
    // mirror refresh (full-field readbacks at most every kMirrorDt sim-seconds).
    void Update(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir, double simUnix,
                double camLatDeg, double camLonDeg, double camAltM);
    // Force a dormant window up (the probe harness; interactive uses the camera rule).
    bool Activate(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir,
                  const char* name, double simUnix);

    WeatherSample Query(double latDeg, double lonDeg, double unixT,
                        double groundResM = 500.0) const;

    int ActiveWindows() const;
    std::string stats;   // "wx 2 windows (merrimack, boston)" for the title bar

private:
    struct Window {
        std::string name;
        SweSolver* solver = nullptr;              // external, or owned.get()
        std::unique_ptr<SweSolver> owned;
        std::unique_ptr<GpuTexture> ownedBathyTex;
        const BathyModel* bathy = nullptr;
        BathyModel* bathyMut = nullptr;       // dormant windows: realized from the channel
                                              // on activation (the one bed, lazily)
        std::function<double(double)> oceanAt;    // NAVD level clock for forcing + queries
        SweConfig cfg;
        double spinupHours = 0.5;
        bool active = false;
        // CPU mirrors
        std::vector<float> eta, uv4;
        uint32_t etaW = 0, etaH = 0, uvW = 0, uvH = 0;
        double mirrorT = -1.0e18;
        char levelTag[48] = {0};
        char currentTag[48] = {0};
    };
    static constexpr double kMirrorDt = 2.0;      // sim-seconds between mirror refreshes
    static constexpr double kActivateAltM = 30000.0;

    const Window* WindowAt(double latDeg, double lonDeg) const;
    void RefreshMirror(Gpu& gpu, Window& w, double simUnix);

    Compositor* m_comp = nullptr;
    int m_hgtCh = -1;
    const WaterAtlas* m_atlas = nullptr;
    const TideModel* m_tides = nullptr;
    const GlobeModel* m_globe = nullptr;
    const SeaState* m_sea = nullptr;
    const CurrentModel* m_currents = nullptr;
    std::vector<Window> m_windows;
};

}  // namespace ga
