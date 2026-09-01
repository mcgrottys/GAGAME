// ================================================================================================
//  GeoGridLoader - M9h: the first CONCRETE FieldLoader, so `plugin.Open(path)` works end to end.
//
//  What it loads: the harvester's georeferenced grid pair -- `<name>.f32` (float32 samples,
//  row 0 north) beside `<name>.json` (nx, ny, lon0, lat1, dlon, dlat, row0, nodata, provenance).
//  That pair IS what this engine consumes; harvester/geotiff.py already reads the real GeoTIFF
//  (int16/float32, LZW/Deflate, horizontal and floating-point predictors, windowed) and writes
//  the georeference out beside the samples.
//
//  WHY NOT PARSE THE .tif HERE. Two reasons, and the second is the honest one:
//    * the georeference survives the conversion intact -- lon0/lat1/dlon/dlat/row0/nodata are
//      exactly the ModelTiepoint, ModelPixelScale and GDAL nodata tags, so provenance is still
//      Embedded, just read once by the harvester instead of on every load;
//    * a C++ TIFF reader worth having needs LZW, Deflate and both predictors. geotiff.py has
//      all of that and is exercised on every harvest. Writing a half version here that handles
//      only uncompressed strips would look like the feature and fail on the actual files.
//  When a direct reader is wanted, it is a port of geotiff.py behind this same interface, and
//  nothing above it changes -- which is the point of the seam.
//
//  THE INGEST RULE, applied (GeoRef.h): nodata is absence. MayHaveData() answers residency by
//  scanning a tile for a single real sample, so an entirely-unsurveyed tile is never allocated
//  and never decoded. A survey composite is mostly absence, and this is where that pays.
// ================================================================================================
#pragma once

#include "core/FieldLoader.h"
#include "core/Json.h"

#include <cstdio>
#include <string>
#include <vector>

namespace ga {

class GeoGridLoader : public FieldLoader {
public:
    // path may be either the .json sidecar or the .f32 itself; the pair is found from the stem.
    static std::unique_ptr<FieldLoader> Open(const std::string& path, const GeoRef* declared) {
        auto self = std::unique_ptr<GeoGridLoader>(new GeoGridLoader());
        return self->Init(path, declared) ? std::unique_ptr<FieldLoader>(self.release())
                                          : nullptr;
    }

    const char* Name() const override { return m_name.c_str(); }
    const char* Structure() const override { return "harvester grid float32 (row 0 north)"; }
    const GeoRef& Ref() const override { return m_ref; }
    uint8_t GradeSig() const override { return kG0; }   // one scalar per cell
    uint32_t Channels() const override { return 1; }
    Residence ResidenceClass() const override { return Residence::Streamable; }

    // Absence without decoding: one pass looking for a single real sample.
    bool MayHaveData(uint32_t tileX, uint32_t tileY) const override {
        return TileHasData(tileX, tileY, m_probeW, m_probeH);
    }

    bool LoadTile(uint32_t tileX, uint32_t tileY, uint32_t tileW, uint32_t tileH,
                  TilePayload& out) override {
        out.width = tileW;
        out.height = tileH;
        out.channels = 1;
        out.data.assign(size_t(tileW) * tileH, 0.0f);
        out.allNoData = true;
        uint32_t real = 0;
        for (uint32_t r = 0; r < tileH; ++r) {
            const uint32_t sy = tileY * tileH + r;
            if (sy >= m_ref.height) break;
            for (uint32_t c = 0; c < tileW; ++c) {
                const uint32_t sx = tileX * tileW + c;
                if (sx >= m_ref.width) break;
                const float v = m_samples[size_t(sy) * m_ref.width + sx];
                if (m_ref.IsNoData(double(v))) continue;   // absence stays zero
                out.data[size_t(r) * tileW + c] = v;
                out.allNoData = false;
                ++real;
            }
        }
        out.coverage = float(real) / float(tileW * tileH);
        return !out.allNoData;
    }

    // For callers that want the whole grid (the existing dense consumers).
    const std::vector<float>& Samples() const { return m_samples; }

    // Residency mask at a given tile size -- the ingest rule turned into a policy input.
    std::vector<uint8_t> CoverageMask(uint32_t tileW, uint32_t tileH, uint32_t tilesX,
                                      uint32_t tilesY) const {
        std::vector<uint8_t> m(size_t(tilesX) * tilesY, 0);
        for (uint32_t ty = 0; ty < tilesY; ++ty) {
            for (uint32_t tx = 0; tx < tilesX; ++tx) {
                m[size_t(ty) * tilesX + tx] = TileHasData(tx, ty, tileW, tileH) ? 1u : 0u;
            }
        }
        return m;
    }

private:
    GeoGridLoader() = default;

    bool TileHasData(uint32_t tx, uint32_t ty, uint32_t tw, uint32_t th) const {
        if (!tw || !th) return true;
        for (uint32_t r = 0; r < th; ++r) {
            const uint32_t sy = ty * th + r;
            if (sy >= m_ref.height) break;
            const uint32_t x0 = tx * tw;
            if (x0 >= m_ref.width) break;
            const uint32_t x1 = (x0 + tw < m_ref.width) ? x0 + tw : m_ref.width;
            const float* row = &m_samples[size_t(sy) * m_ref.width];
            for (uint32_t x = x0; x < x1; ++x) {
                if (!m_ref.IsNoData(double(row[x]))) return true;
            }
        }
        return false;
    }

    bool Init(const std::string& path, const GeoRef* declared) {
        const size_t dot = path.find_last_of('.');
        const std::string stem = (dot == std::string::npos) ? path : path.substr(0, dot);
        const std::string jsonPath = stem + ".json";

        std::string text;
        if (FILE* f = fopen(jsonPath.c_str(), "rb")) {
            char buf[4096];
            size_t n;
            while ((n = fread(buf, 1, sizeof(buf), f)) > 0) text.append(buf, n);
            fclose(f);
        }
        std::string err;
        const JsonValue root = text.empty() ? JsonValue{} : JsonParser::Parse(text, &err);
        if (!err.empty()) {
            Log("[loader] %s: %s", jsonPath.c_str(), err.c_str());
            return false;
        }

        const int nx = static_cast<int>(root.Num("nx", 0));
        const int ny = static_cast<int>(root.Num("ny", 0));
        if (nx <= 0 || ny <= 0) {
            // No sidecar: the operator must declare the georeference, which is exactly the
            // third provenance class and why it exists.
            if (!declared) {
                Log("[loader] %s: no sidecar and no declared GeoRef -- refusing to guess",
                    jsonPath.c_str());
                return false;
            }
            m_ref = *declared;
        } else {
            // The sidecar carries what the GeoTIFF's own tags said, so this stays EMBEDDED
            // provenance: read once by the harvester, not invented here.
            m_ref = GeoRef::Declared(4326, CrsKind::Geographic, root.Num("lon0", 0.0),
                                     root.Num("lat1", 0.0), root.Num("dlon", 0.0),
                                     -root.Num("dlat", 0.0), uint32_t(nx), uint32_t(ny));
            m_ref.provenance = CrsProvenance::Embedded;
            m_ref.centers = false;   // harvester grids are corner-anchored, like the tiepoint
            m_ref.linearUnit = "deg";
            m_ref.valueUnit = "m NAVD88";
            m_ref.hasNoData = true;
            m_ref.noData = root.Num("nodata", -9999.0);
            m_ref.valueMin = root.Num("min_m", -100.0);
            m_ref.valueMax = root.Num("max_m", 100.0);
            if (root.Str("row0", "north") != "north") m_ref.scaleY = -m_ref.scaleY;
        }

        std::string f32 = root.Str("file", "");
        if (f32.empty()) f32 = stem.substr(stem.find_last_of("/\\") + 1) + ".f32";
        const size_t slash = jsonPath.find_last_of("/\\");
        const std::string dir = (slash == std::string::npos) ? "" : jsonPath.substr(0, slash + 1);
        const std::string binPath = dir + f32;

        FILE* bf = fopen(binPath.c_str(), "rb");
        if (!bf) {
            Log("[loader] %s: samples not found", binPath.c_str());
            return false;
        }
        m_samples.resize(size_t(m_ref.width) * m_ref.height);
        const size_t got = fread(m_samples.data(), sizeof(float), m_samples.size(), bf);
        fclose(bf);
        if (got != m_samples.size()) {
            Log("[loader] %s: short read (%zu of %zu samples)", binPath.c_str(), got,
                m_samples.size());
            return false;
        }
        m_name = stem.substr(stem.find_last_of("/\\") + 1);
        Log("[loader] %s: %ux%u %s, nodata %.0f, %s", m_name.c_str(), m_ref.width, m_ref.height,
            m_ref.Describe().c_str(), m_ref.noData, root.Str("source", "").substr(0, 60).c_str());
        return true;
    }

    std::string m_name;
    GeoRef m_ref;
    std::vector<float> m_samples;
    uint32_t m_probeW = 128, m_probeH = 128;
};

}   // namespace ga
