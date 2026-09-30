// ================================================================================================
//  ColorStackSource - M9ac: THE IMAGERY, THROUGH THE PROCESS.
//
//  Colour was the last channel still shaped like a traditional LOD system, and the shape was
//  forced rather than chosen. D3D12 caps a single texture at 16384, so one cube face bottoms out
//  at ~611 m/texel; reaching 9.5 m and 1.2 m over the inlet meant TWO MORE TEXTURES -- a z14
//  Mercator window and a z17 detail window -- each with its own frame, its own residency budget,
//  and a hand-off gate in the shader to fade between them.
//
//  That produced exactly the artefacts reported: a 64x resolution cliff at the window's edge
//  with nothing in between, and far imagery starved because three tenants bid separately for
//  tiles while the one that covers the whole planet was already at its finest.
//
//  THE CAP IS ON A PAGE, NOT ON THE ADDRESS SPACE. PageAddr/LevelLadder already model the
//  planet as PageAddr{level, x, y} with 16384-texel pages and a strict halving per level, so a
//  ladder can run from centimetres to the whole globe in ONE space with pages resident only
//  where data exists. Sparse residency IS the level-of-detail mechanism; the rungs were a
//  workaround for a limit that the page address space had already removed.
//
//  So imagery joins every other field on the standard path, and this file is its first arrow:
//
//      GA Load (here)  ->  normalize  ->  GA Compose  ->  Sparse GA Bank  ->  GA Atlas
//
//  Each existing ColorSource -- the Google tile tree, the MassGIS orthos, the overlay -- becomes
//  a DomainSource. Nothing is reimplemented: they still answer through the same Sample() the
//  painter calls, so there is no second copy of the reprojection, the zoom pick or the feather
//  to drift. What changes is where the answers go.
//
//  UNIT: "sRGB byte", which GaUnits parses as Colour with a 1/255 factor. That is not ceremony.
//  The compositor now refuses to blend a colour with a length, and an 8-bit source that forgot
//  to say it was 8-bit would compose 255x too bright against a float one.
//
//  BLEND: LayeredOver, the same rule the height stack needed. An imagery stack is an authority
//  order -- orthos over tiles, overlay over orthos -- painted with a feather at the edges, which
//  is compositing, not averaging. Priority-bucketed weighted mean would have pulled a 15 cm
//  ortho back toward the 611 m tile underneath it wherever its feather was partial.
// ================================================================================================
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "compose/ComposeTree.h"
#include "compose/Compositor.h"
#include "compose/DomainSource.h"
#include "core/Common.h"

namespace ga {

// ================================================================================================
//  ColorLayerSource -- one Compositor colour layer, wearing the DomainSource interface.
//
//  Four channels: RGB plus the source's own paint WEIGHT carried in alpha. The weight has to
//  ride through as data because the stack's feather lives in it -- a texel a source only partly
//  claims must stay partly claimed all the way to the atlas, or every seam it was smoothing
//  turns back into an edge.
// ================================================================================================
class ColorLayerSource : public DomainSource {
public:
    explicit ColorLayerSource(ColorSource* src)
        : m_src(src),
          m_name(src ? src->Info().name : "color"),
          m_unit(UnitSpec::Parse("sRGB byte")) {}

    const char* Name() const override { return m_name.c_str(); }
    SourceDomain Domain() const override { return SourceDomain::Raster; }
    uint8_t GradeSig() const override { return 1u; }   // kG0 per channel: colour is not a rotor
    uint32_t Channels() const override { return 4; }
    const UnitSpec& Unit() const override { return m_unit; }
    const char* NodeKind() const override { return "load"; }
    // name|structure -- the same identity Compositor's soak rule hashes, so a tree on disk and
    // a composed cache entry agree about what a source IS.
    std::string Identity() const override {
        return m_src ? m_src->Info().name + "|" + m_src->Info().structure : m_name;
    }
    bool MayCover(double lon0, double lat0, double lon1, double lat1) const override {
        if (!m_src) return false;
        const SourceInfo& si = m_src->Info();
        return !(si.lon1 < lon0 || si.lon0 > lon1 || si.lat1 < lat0 || si.lat0 > lat1);
    }
    bool Footprint(double& lon0, double& lat0, double& lon1, double& lat1) const override {
        if (!m_src) return false;
        const SourceInfo& si = m_src->Info();
        lon0 = si.lon0; lat0 = si.lat0; lon1 = si.lon1; lat1 = si.lat1;
        return true;
    }
    // The tile-wise path needs the per-tile context the point API cannot carry: BeginTile is
    // where the vector GIS mask sweeps its rings once per tile. Exposed so TileTree can run it.
    ColorSource* Raw() const { return m_src; }

    bool SampleAt(const DomainQuery& q, DomainValue& out) const override {
        out.weight = 0.0f;
        if (!m_src) return false;
        constexpr double kD2R = 3.14159265358979 / 180.0;
        // PaintCtx is per-tile, per-source state the REALIZATION owns, and sources are shared
        // across worker threads. A default context is honest here: the only thing it carries is
        // the grade-normalization gain, which BeginTile sets per tile -- a page-at-a-time
        // compose has no tile to set it from, so it must not pretend to.
        PaintCtx ctx;
        uint8_t rgba[4] = {0, 0, 0, 0};
        const float w = m_src->Sample(q.lat * kD2R, q.lon * kD2R, q.groundM, ctx, rgba);
        if (w <= 0.0f) return false;
        out.c[0] = float(rgba[0]);
        out.c[1] = float(rgba[1]);
        out.c[2] = float(rgba[2]);
        out.c[3] = float(rgba[3]);
        out.weight = w;
        return true;
    }

private:
    ColorSource* m_src = nullptr;   // borrowed; main owns the sources
    std::string m_name;
    UnitSpec m_unit;
};

// ================================================================================================
//  BuildColorStack -- every layer of a Compositor colour channel as DomainSources, bottom to
//  top, which is the order Blend::LayeredOver requires: the overlay must be added LAST or it
//  stops being on top.
// ================================================================================================
inline std::vector<std::shared_ptr<DomainSource>> BuildColorStack(const Compositor& comp,
                                                                  int colorChannel) {
    std::vector<std::shared_ptr<DomainSource>> out;
    if (colorChannel < 0 || colorChannel >= comp.ChannelCount()) return out;
    const Compositor::Channel& ch = comp.ChannelAt(colorChannel);
    for (ColorSource* c : ch.color) {
        if (!c) continue;
        auto layer = std::make_shared<ColorLayerSource>(c);
        // Normalize to canonical colour on [0,1]. Every layer is 8-bit today so this is one
        // shared factor, but the stage is what makes that a CHECKED fact rather than a habit:
        // a float or 16-bit source added later converts instead of blowing out.
        auto norm = NormalizeToSi(layer);
        out.push_back(norm ? std::move(norm) : std::move(layer));
    }
    return out;
}

}  // namespace ga
