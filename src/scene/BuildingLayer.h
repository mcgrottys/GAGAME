// ================================================================================================
//  BuildingLayer - the composed building SOLIDS (compose/BuildingSolids.h), drawn as prisms: walls
//  from the ring's edges, a flat roof ear-clipped from its rings, each solid stood on the composed
//  ground at the lowest point of its footprint (OSM's datum for `height`).
//
//  A SEPARATE LAYER, by the owner's decision (2026-10-09): the solids never enter earth.height, so
//  the water solver's bed (which IS earth.height) does not see them. The water will meet the same
//  solids later through its own sparse voxels; nothing here is the picture's private copy of them.
//
//  PRECISION, the vessel layer's law: every vertex is built in DOUBLES in the flat frame, then
//  stored as a float offset from its CELL's origin (cells of kCellDeg, about a kilometre, so an
//  offset is never more than ~1 km and its float step is ~0.06 mm). At Render each cell's origin is
//  taken relative to the eye in doubles and only that difference is cast. One static buffer; one
//  draw a cell, its offset in the constants.
//
//  Drawn with no culling: a footprint's winding is the file's, and a solid's inside is occluded by
//  its own walls anyway. Procedural fetch by SV_VertexID, as every layer here (no input assembler).
// ================================================================================================
#pragma once

#include "compose/BuildingSolids.h"
#include "scene/Layer.h"

#include <functional>
#include <string>
#include <vector>

namespace ga {

class BuildingLayer : public Layer {
public:
    // A point of the planet (degrees, metres on the height datum) in the flat frame, doubles.
    using Place = std::function<void(double latDeg, double lonDeg, double h, double out[3])>;
    // The composed ground (metres on the height datum) under a point, degrees.
    using Ground = std::function<double(double latDeg, double lonDeg)>;

    void Configure(const std::wstring& shaderDir, std::vector<BuildingSolid> solids, Place place,
                   Ground ground) {
        m_shaderDir = shaderDir;
        m_solids = std::move(solids);
        m_place = std::move(place);
        m_ground = std::move(ground);
    }

    const char* Name() const override { return "buildings"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields, hal::RootSignature rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;

    // The geometry, once the flat frame's rows exist (FrameLoop derives them after the layers are
    // registered): `place` reads them, so the solids are built where the frame is real.
    void Build(Gpu& gpu);

    size_t Solids() const { return m_solids.size(); }
    uint64_t Vertices() const { return m_vertexCount; }

    // Mirrors `struct BuildingVertex` in shaders/Buildings.hlsl (priors 22).
    struct Vertex {
        float pos[3];   // metres from the cell's origin, flat frame
        float kind;     // 0 wall, 1 roof (+2 for a building:part)
        float n[3];     // outward unit normal, flat frame
        float pad;
    };
    static constexpr double kCellDeg = 0.01;

private:
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);

    struct Cell {
        double origin[3];          // flat frame, doubles
        uint32_t first, count;     // vertices in the one buffer
    };

    std::wstring m_shaderDir;
    std::vector<BuildingSolid> m_solids;
    Place m_place;
    Ground m_ground;
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_pso;
    GpuBuffer m_vb;
    std::vector<Cell> m_cells;
    uint64_t m_vertexCount = 0;
};

}  // namespace ga
