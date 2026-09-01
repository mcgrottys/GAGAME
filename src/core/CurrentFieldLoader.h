// ================================================================================================
//  CurrentFieldLoader - M9i: the regional current as a GA object, through the plugin seam.
//
//  This is the second concrete FieldLoader, and it exists to prove the seam takes a SECOND file
//  type without anything above it changing. The first attempt at getting GoMOFS into the tree
//  did not go through here at all: it reached into GulfLayer's private texture, added a kernel
//  to Swe.hlsl and wired special-case descriptors into SweSolver so one subsystem could feed
//  one bank. That is the bespoke pipeline this design exists to delete, and it was backed out.
//
//  What a loader answers, and all it answers (FieldLoader.h):
//    WHERE     data/currents/currents.json's `field` block -- lon0/lat0/dlon/dlat straight from
//              the harvester, which read them from the GoMOFS NetCDF. Provenance EMBEDDED.
//    WHAT      grade 1. A current is a vector field; the ALGEBRA decides what grad() of it is
//              (kG0 | kG2 -- divergence and vorticity), not this file and not a call site.
//    WHERE NOT samples <= -900 are land. Absence, not a value: an all-land tile is never
//              allocated, so Tier-2's read-zero MEANS "no current here" and no sentinel ever
//              reaches a shader disguised as a measurement.
//
//  ROW ORDER, DERIVED RATHER THAN DECLARED. CurrentField::Sample computes
//  fy = (lat - lat0) / dlat with dlat POSITIVE, so row 0 is the SOUTH edge -- the opposite of
//  the bathy grids, which are row-0-north. Nothing here types that fact: scaleY is set positive
//  and GeoRef::VNorth() reads it off the affine, so the source cannot disagree with its own
//  georeference (priors 10, where a hand-declared flip was the bug).
//
//  Storage note: the .f32 is PLANAR -- the whole u plane, then the whole v plane -- while a
//  TilePayload is interleaved. The gather happens here, which is the point of a loader: the
//  file's layout stops at this boundary.
// ================================================================================================
#pragma once

#include "core/FieldLoader.h"
#include "core/Json.h"

#include <cstdio>
#include <string>
#include <vector>

namespace ga {

class CurrentFieldLoader : public FieldLoader {
public:
    static std::unique_ptr<FieldLoader> Open(const std::string& path, const GeoRef* declared) {
        auto self = std::unique_ptr<CurrentFieldLoader>(new CurrentFieldLoader());
        return self->Init(path, declared) ? std::unique_ptr<FieldLoader>(self.release())
                                          : nullptr;
    }

    const char* Name() const override { return m_name.c_str(); }
    const char* Structure() const override { return "gomofs uv float32 planar (row 0 south)"; }
    const GeoRef& Ref() const override { return m_ref; }
    // A current is a VECTOR. grad() of it is kG0 | kG2 by the Cayley closure, computed in the
    // type -- this loader never has to know that, which is the point.
    uint8_t GradeSig() const override { return kG1; }
    uint32_t Channels() const override { return 2; }
    Residence ResidenceClass() const override { return Residence::Streamable; }

    bool MayHaveData(uint32_t tileX, uint32_t tileY) const override {
        return TileHasData(tileX, tileY, m_probe, m_probe);
    }

    bool LoadTile(uint32_t tileX, uint32_t tileY, uint32_t tileW, uint32_t tileH,
                  TilePayload& out) override {
        out.width = tileW;
        out.height = tileH;
        out.channels = 2;
        out.data.assign(size_t(tileW) * tileH * 2, 0.0f);
        out.allNoData = true;
        uint32_t real = 0;
        for (uint32_t r = 0; r < tileH; ++r) {
            const uint32_t sy = tileY * tileH + r;
            if (sy >= m_ref.height) break;
            for (uint32_t c = 0; c < tileW; ++c) {
                const uint32_t sx = tileX * tileW + c;
                if (sx >= m_ref.width) break;
                const size_t k = size_t(sy) * m_ref.width + sx;
                if (m_u[k] <= -900.0f) continue;   // land: absence stays zero
                const size_t o = (size_t(r) * tileW + c) * 2;
                out.data[o + 0] = m_u[k];
                out.data[o + 1] = m_v[k];
                out.allNoData = false;
                ++real;
            }
        }
        out.coverage = float(real) / float(tileW * tileH);
        return !out.allNoData;
    }

    // Residency without decoding, for a policy that must be evaluated before any I/O.
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
    CurrentFieldLoader() = default;

    bool TileHasData(uint32_t tx, uint32_t ty, uint32_t tw, uint32_t th) const {
        if (!tw || !th) return true;
        for (uint32_t r = 0; r < th; ++r) {
            const uint32_t sy = ty * th + r;
            if (sy >= m_ref.height) break;
            const uint32_t x0 = tx * tw;
            if (x0 >= m_ref.width) break;
            const uint32_t x1 = (x0 + tw < m_ref.width) ? x0 + tw : m_ref.width;
            const float* row = &m_u[size_t(sy) * m_ref.width];
            for (uint32_t x = x0; x < x1; ++x) {
                if (row[x] > -900.0f) return true;
            }
        }
        return false;
    }

    bool Init(const std::string& path, const GeoRef* declared) {
        std::string text;
        if (FILE* f = fopen(path.c_str(), "rb")) {
            char buf[8192];
            size_t n;
            while ((n = fread(buf, 1, sizeof(buf), f)) > 0) text.append(buf, n);
            fclose(f);
        }
        if (text.empty()) return false;
        std::string err;
        const JsonValue root = JsonParser::Parse(text, &err);
        if (!err.empty()) {
            Log("[loader] %s: %s", path.c_str(), err.c_str());
            return false;
        }
        const JsonValue* jf = root.Get("field");
        if (!jf) return false;

        const int nx = static_cast<int>(jf->Num("nx", 0));
        const int ny = static_cast<int>(jf->Num("ny", 0));
        if (nx <= 0 || ny <= 0) {
            if (!declared) return false;
            m_ref = *declared;
        } else {
            // scaleY POSITIVE: row 0 is south. Derived from the file's own convention, never
            // typed -- VNorth() then reads it off the affine.
            m_ref = GeoRef::Declared(4326, CrsKind::Geographic, jf->Num("lon0", 0.0),
                                     jf->Num("lat0", 0.0), jf->Num("dlon", 0.0),
                                     jf->Num("dlat", 0.0), uint32_t(nx), uint32_t(ny));
            m_ref.provenance = CrsProvenance::Embedded;   // harvester read it from the NetCDF
            m_ref.centers = true;                          // Sample() offsets by -0.5: centres
            m_ref.linearUnit = "deg";
            m_ref.valueUnit = "m/s (u east, v north)";
            m_ref.hasNoData = true;
            m_ref.noData = -999.0;
            m_ref.valueMin = -3.0;
            m_ref.valueMax = 3.0;
        }

        const size_t slash = path.find_last_of("/\\");
        const std::string dir = (slash == std::string::npos) ? "" : path.substr(0, slash + 1);
        const std::string bin = dir + jf->Str("file", "gomofs_uv.f32");
        FILE* bf = fopen(bin.c_str(), "rb");
        if (!bf) {
            Log("[loader] %s: samples not found", bin.c_str());
            return false;
        }
        const size_t n = size_t(m_ref.width) * m_ref.height;
        m_u.resize(n);
        m_v.resize(n);
        const size_t gu = fread(m_u.data(), sizeof(float), n, bf);
        const size_t gv = fread(m_v.data(), sizeof(float), n, bf);
        fclose(bf);
        if (gu != n || gv != n) {
            Log("[loader] %s: short read (%zu + %zu of %zu)", bin.c_str(), gu, gv, n);
            return false;
        }
        m_name = "noaa.gomofs.current";
        Log("[loader] %s: %ux%u %s, %s", m_name.c_str(), m_ref.width, m_ref.height,
            m_ref.Describe().c_str(), jf->Str("source", "").substr(0, 52).c_str());
        return true;
    }

    std::string m_name;
    GeoRef m_ref;
    std::vector<float> m_u, m_v;
    uint32_t m_probe = 32;
};

}   // namespace ga
