// ================================================================================================
//  FieldLoader - M9h: THE PLUGIN SEAM. `plugin.LoadFileTypeX("path")`, in C++.
//
//  The goal, in the user's words: adding a new dataset should cost a loader and a grade
//  declaration -- not a bespoke pipeline, and not a perl script. Register a loader for a file
//  type, open a file, get a sparse GA object whose projection, units and range came from the
//  data rather than from a comment.
//
//      registry.Register("tif", MakeGeoTiffLoader);
//      auto bed  = registry.Open("data/bathy/merrimack.tif");     // GeoRef read FROM the file
//      auto flow = registry.Open("data/currents/gomofs.nc");
//      // -> GradeBank::FillFrom(*bed) allocates only the tiles that carry data
//
//  THREE THINGS A LOADER MUST ANSWER, and they are the whole contract:
//
//    1. WHERE  -- a GeoRef, narrowed as far as the source allows: embedded (GeoTIFF tags),
//                 by convention (a WMTS pyramid), or declared by the operator. The provenance
//                 travels with it so a report can say which.
//    2. WHAT   -- a grade signature. A bathymetry raster is grade 0. A current field is grade 1.
//                 A velocity-gradient product is grade 0 | grade 2 and the ALGEBRA says so
//                 (GradeField.h) rather than the loader claiming it.
//    3. WHERE NOT -- which tiles are entirely nodata. This is the load-bearing one: a tile that
//                 is all absence is never allocated, so Tier-2's read-zero MEANS "no data
//                 here" and no sentinel value (-9999, NaN) can reach a shader disguised as
//                 terrain. Lossless, because no real sample was touched; sparse, because
//                 absence costs nothing. See GeoRef.h's ingest rule.
//
//  A loader is deliberately NOT responsible for reprojection. It reports the CRS it found; the
//  compositor resolves sources into the exchange frame with the exact formulas in
//  Projections.h. A loader that guessed would be the one place a silent misalignment could
//  enter, and this project's orientation bugs have all been declaration bugs (priors 10).
// ================================================================================================
#pragma once

#include "core/GeoRef.h"
#include "core/GradeField.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ga {

// One tile's decoded samples, channels interleaved, row-major in the SOURCE's row order (the
// GeoRef says which way that runs -- the consumer flips, the loader never does).
struct TilePayload {
    std::vector<float> data;
    uint32_t width = 0, height = 0, channels = 0;
    // True when every sample was nodata. The caller must then leave the tile NULL rather than
    // allocating a tile of zeros -- those are different statements and the atlas relies on it.
    bool allNoData = true;
    // How much of the tile carried real samples, for residency budgeting and reporting.
    float coverage = 0.0f;
};

// ================================================================================================
//  The interface a file format implements. One virtual call per TILE, never per texel -- the
//  polymorphism is at the seam where it costs nothing.
// ================================================================================================
class FieldLoader {
public:
    virtual ~FieldLoader() = default;

    virtual const char* Name() const = 0;        // "noaa.cudem.merrimack"
    virtual const char* Structure() const = 0;   // "geotiff int16 windowed"
    virtual const GeoRef& Ref() const = 0;       // narrowed as far as the source allows
    virtual uint8_t GradeSig() const = 0;        // kG0 / kG1 / kG2 combinations
    virtual uint32_t Channels() const = 0;       // samples per texel in the payload

    // Decode one tile. Returns false when the tile is entirely absent -- the caller leaves it
    // NULL. Returning true with allNoData set is the same statement; both are honoured.
    virtual bool LoadTile(uint32_t tileX, uint32_t tileY, uint32_t tileW, uint32_t tileH,
                          TilePayload& out) = 0;

    // Optional fast path: answer residency WITHOUT decoding. A GeoTIFF with a nodata mask, a
    // tile pyramid with a coverage index, or a survey footprint polygon can all answer this
    // cheaply, which is what lets a policy be evaluated before any I/O happens.
    virtual bool MayHaveData(uint32_t /*tileX*/, uint32_t /*tileY*/) const { return true; }

    // The residency class -- the third axis beyond FIELD/TEXTURE. It decides what an eviction
    // COSTS, which is the thing that makes water different from terrain.
    enum class Residence : uint8_t {
        Streamable,     // reload from disk on demand (imagery, bathymetry)
        Recomputable,   // regenerate from a cache key (the solved wave field)
        Volatile,       // exists only on the GPU; evicting LOSES it (churn, foam memory)
    };
    virtual Residence ResidenceClass() const { return Residence::Streamable; }
};

// ================================================================================================
//  The registry -- `plugin`. Register a factory per file type; Open() picks by extension unless
//  the caller names the type. A declared GeoRef may be supplied for bare rasters that carry no
//  georeference of their own; loaders that find one embedded MUST prefer the file's own and say
//  so through GeoRef::provenance.
// ================================================================================================
using LoaderFactory =
    std::function<std::unique_ptr<FieldLoader>(const std::string& path, const GeoRef* declared)>;

class LoaderRegistry {
public:
    void Register(const std::string& ext, LoaderFactory make) {
        m_byExt[Lower(ext)] = std::move(make);
    }

    bool Knows(const std::string& ext) const { return m_byExt.count(Lower(ext)) != 0; }

    std::unique_ptr<FieldLoader> Open(const std::string& path,
                                      const GeoRef* declared = nullptr) const {
        const size_t dot = path.find_last_of('.');
        if (dot == std::string::npos) return nullptr;
        auto it = m_byExt.find(Lower(path.substr(dot + 1)));
        if (it == m_byExt.end()) return nullptr;
        return it->second(path, declared);
    }

    // Named explicitly, for sources whose extension lies (a .bin that is a tile pyramid) or for
    // protocols with no file at all.
    std::unique_ptr<FieldLoader> OpenAs(const std::string& type, const std::string& path,
                                        const GeoRef* declared = nullptr) const {
        auto it = m_byExt.find(Lower(type));
        return (it == m_byExt.end()) ? nullptr : it->second(path, declared);
    }

    std::vector<std::string> Types() const {
        std::vector<std::string> v;
        v.reserve(m_byExt.size());
        for (const auto& kv : m_byExt) v.push_back(kv.first);
        return v;
    }

private:
    static std::string Lower(std::string s) {
        for (char& c : s) c = (c >= 'A' && c <= 'Z') ? char(c - 'A' + 'a') : c;
        return s;
    }
    std::map<std::string, LoaderFactory> m_byExt;
};

}   // namespace ga
