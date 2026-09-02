// ================================================================================================
//  MemGridLoader - M9q: GA LOAD, for data that is already in RAM.
//
//  The process is GA Load -> normalize -> GA Compose -> GA physics -> sparse GPU, and every
//  loader so far has read a FILE. But several fields arrive decoded already -- GlobeModel holds
//  gfswave Hs and wind, GFS sea ice, and the three ocean-colour retrievals as plain float
//  vectors, parsed out of GRIB2/NetCDF long before any layer sees them. Those went straight to
//  CreateTexture2D + UploadTexture, which is the shortcut the process exists to forbid: a
//  committed texture with no georeference anyone can check, no coverage, no mip chain that
//  understands absence, and no way into the sparse tree.
//
//  This is the seam that fixes that WITHOUT re-parsing anything. It is a FieldLoader over
//  borrowed float arrays, so the same compositor, the same unit stage and the same paged bank
//  serve in-memory fields and on-disk ones identically. GlobeLayer's five planes go through it;
//  GulfLayer's currents and WeatherManager's mirrors are the same shape and can follow.
//
//  CHANNEL PACKING IS A LOAD CONCERN, not a layer's. GlobeLayer used to interleave chl / Kd490 /
//  SPM / valid into one RGBA buffer by hand before uploading. That loop is composition performed
//  in the wrong place -- it hard-codes the channel order at the call site, where nothing declares
//  it. Here the loader takes up to four planes and states what each one is, so the packing has a
//  name and the compositor sees four honest channels.
//
//  NODATA IS A PREDICATE, not a magic number the caller remembers. Each plane carries its own
//  sentinel test (gfswave writes -1 for land; ocean colour has OcNull for polar night and the
//  gap fill's edges), and a tile that is entirely sentinel returns allNoData so the atlas leaves
//  it NULL rather than allocating a tile of lies.
//
//  ONE DELIBERATE CONSERVATISM. The sentinel is ALSO what the consumer expects to read: Globe.hlsl
//  tests `hsS >= 0` for land, so turning -1 into absence and filling the hole with something else
//  would change the picture. So the bank is composed with that same sentinel as its nodata fill --
//  identical bytes for the shader, honest coverage for the tree. Once a consumer reads the
//  coverage plane instead of sniffing a magic value, the sentinel can retire.
// ================================================================================================
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "core/FieldLoader.h"
#include "core/GeoRef.h"

namespace ga {

class MemGridLoader : public FieldLoader {
public:
    // A plane: the samples, and what counts as "no retrieval here".
    struct Plane {
        const std::vector<float>* data = nullptr;
        // Sentinel test. Default: everything is real. `nodataBelow` covers both conventions in
        // use here -- gfswave's -1 land flag and ocean colour's OcNull floor.
        float nodataBelow = -1e30f;
        // What an absent sample becomes in the payload. Kept equal to the sentinel by default so
        // the composed bank is byte-identical to the texture it replaces.
        float fill = 0.0f;
        // A DERIVED channel: 1 where every data plane retrieved, 0 where any did not, and no
        // array of its own. The ocean-colour pack needs exactly this, and computing it here is
        // the point -- "was this texel retrieved" is a fact about the LOAD, and the hand-rolled
        // version in GlobeLayer had to restate all three sentinel tests to get it. Last in the
        // struct so the common {data, sentinel, fill} form still reads naturally.
        bool validityFlag = false;
    };

    MemGridLoader(std::string name, std::string structure, const GeoRef& ref,
                  std::vector<Plane> planes)
        : m_name(std::move(name)),
          m_structure(std::move(structure)),
          m_ref(ref),
          m_planes(std::move(planes)) {}

    bool Valid() const {
        if (m_planes.empty() || m_planes.size() > 4) return false;
        const size_t need = size_t(m_ref.width) * m_ref.height;
        bool anyData = false;
        for (const Plane& p : m_planes) {
            if (p.validityFlag) continue;   // derived: it has no array of its own
            if (!p.data || p.data->size() < need) return false;
            anyData = true;
        }
        return anyData;
    }

    const char* Name() const override { return m_name.c_str(); }
    const char* Structure() const override { return m_structure.c_str(); }
    const GeoRef& Ref() const override { return m_ref; }
    uint8_t GradeSig() const override { return 1u; }   // kG0: these are all scalars per channel
    uint32_t Channels() const override { return uint32_t(m_planes.size()); }
    // In RAM already and owned elsewhere: evicting a tile costs a recopy, never a re-parse.
    Residence ResidenceClass() const override { return Residence::Recomputable; }

    bool LoadTile(uint32_t tileX, uint32_t tileY, uint32_t tileW, uint32_t tileH,
                  TilePayload& out) override {
        const uint32_t ch = Channels();
        out.width = tileW;
        out.height = tileH;
        out.channels = ch;
        out.data.assign(size_t(tileW) * tileH * ch, 0.0f);
        out.allNoData = true;
        uint32_t real = 0;

        const uint32_t x0 = tileX * tileW, y0 = tileY * tileH;
        for (uint32_t y = 0; y < tileH; ++y) {
            const uint32_t sy = y0 + y;
            if (sy >= m_ref.height) continue;
            for (uint32_t x = 0; x < tileW; ++x) {
                const uint32_t sx = x0 + x;
                if (sx >= m_ref.width) continue;
                const size_t si = size_t(sy) * m_ref.width + sx;
                const size_t di = (size_t(y) * tileW + x) * ch;
                // A texel is REAL only where every plane retrieved. That is the honest rule for
                // a packed field: the ocean model consumes chl, Kd and SPM together, so a texel
                // missing one of them is not a partially-good sample, it is absent.
                bool ok = true;
                for (uint32_t c = 0; c < ch; ++c) {
                    const Plane& pl = m_planes[c];
                    if (pl.validityFlag) continue;
                    if ((*pl.data)[si] <= pl.nodataBelow) { ok = false; break; }
                }
                for (uint32_t c = 0; c < ch; ++c) {
                    const Plane& pl = m_planes[c];
                    if (pl.validityFlag) {
                        out.data[di + c] = ok ? 1.0f : 0.0f;
                    } else {
                        out.data[di + c] = ok ? (*pl.data)[si] : pl.fill;
                    }
                }
                if (ok) { ++real; out.allNoData = false; }
            }
        }
        out.coverage = float(real) / float((std::max)(1u, tileW * tileH));
        return !out.allNoData;
    }

private:
    std::string m_name, m_structure;
    GeoRef m_ref;
    std::vector<Plane> m_planes;   // borrowed; GlobeModel outlives the bank
};

}  // namespace ga
