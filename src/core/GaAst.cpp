#include "GaAst.h"

#include <cstdio>
#include <cstring>

#include "Common.h"

namespace ga::ast {

namespace {
std::vector<Edge>& Reg() {
    static std::vector<Edge> r;
    return r;
}
}  // namespace

void Register(const Edge& e) {
    for (Edge& x : Reg()) {
        if (!std::strcmp(x.from, e.from) && !std::strcmp(x.to, e.to) &&
            !std::strcmp(x.field, e.field)) {
            x = e;
            return;
        }
    }
    Reg().push_back(e);
}

void SetActive(const char* to, bool active) {
    for (Edge& x : Reg()) {
        if (!std::strcmp(x.to, to)) x.active = active;
    }
}

const std::vector<Edge>& Edges() { return Reg(); }

void Print() {
    Log("[gaast] ---- THE STATE DIAGRAM (%zu edges; frames are law) ----", Reg().size());
    const char* last = "";
    for (const Edge& e : Reg()) {
        if (std::strcmp(last, e.from)) {
            Log("[gaast] node %s", e.from);
            last = e.from;
        }
        char sf[96], df[96];
        snprintf(sf, sizeof(sf), "%s%s", e.src.space, e.src.vNorth ? "(+v=N)" : "(+v=S)");
        snprintf(df, sizeof(df), "%s%s", e.dst.space, e.dst.vNorth ? "(+v=N)" : "(+v=S)");
        char geo[128] = "";
        if (e.src.metersPerUnit > 0.0) {
            snprintf(geo, sizeof(geo), "  org(%.0f,%.0f) %.4gm/u", e.src.orgX, e.src.orgY,
                     e.src.metersPerUnit);
        }
        Log("[gaast]   -> %-12s %-14s %-22s->%-22s%s%s  %s in %s  x%.3g%s  [%s]", e.to,
            e.field, sf, df, e.flip ? "  FLIP" : "      ", geo, e.range, e.units, e.gain,
            e.active ? "" : "  (INACTIVE)", e.code);
    }
}

bool Validate() {
    bool ok = true;
    for (const Edge& e : Reg()) {
        // THE FLIP RULE: frames that disagree about +v need exactly one flip in the code.
        const bool need = e.src.vNorth != e.dst.vNorth;
        if (need != e.flip) {
            Log("[gaast] FAIL flip rule: %s -> %s '%s': src %s dst %s but code %s  [%s]",
                e.from, e.to, e.field, e.src.vNorth ? "+v=N" : "+v=S",
                e.dst.vNorth ? "+v=N" : "+v=S", e.flip ? "FLIPS" : "does not flip", e.code);
            ok = false;
        }
    }
    // THE ORPHAN RULE: a field someone produces that nothing active consumes is dead physics
    // walking -- the swell shadow fed only the retired SeaLayer draw for a whole milestone.
    for (const Edge& e : Reg()) {
        if (e.active) continue;
        bool live = false;
        for (const Edge& x : Reg()) {
            if (!std::strcmp(x.from, e.from) && !std::strcmp(x.field, e.field) && x.active) {
                live = true;
                break;
            }
        }
        if (!live) {
            Log("[gaast] WARN orphan: %s '%s' has no ACTIVE consumer (only %s, inactive)",
                e.from, e.field, e.to);
        }
    }
    if (ok) Log("[gaast] validate: %zu edges, flip rule holds on every one", Reg().size());
    return ok;
}

void RegisterKnownWaterEdges() {
    const Frame worldM{"world.m", true, 0, 0, 0};
    const Frame wrap{"patch.wrap", true, 0, 0, 0};
    const Frame atlasN{"atlas.texel", true, 0, 0, 0};
    const Frame rowS{"raster.row0N", false, 0, 0, 0};
    // The one-water chain. Row-0-north rasters (SWE atlases, CUDEM, the shadow) demand a
    // flip into +v=north consumers; the wrap cascades and the bank/churn atlases agree
    // with world +z and demand none. These lines ARE the orientation ledger, as code.
    Register({"ocean.fft", "water.bank", "cascade.disp", wrap, atlasN, false,
              "m disp + jacobian foam", "+-Hs/2", 1.0, "WaterBank.hlsl CsBankFill wrap"});
    Register({"ocean.fft", "globe.ps", "cascade.deriv", wrap, atlasN, false, "slope",
              "+-0.3", 1.0, "Globe.hlsl detail loop"});
    Register({"swe.solver", "water.bank", "eta", rowS, atlasN, true, "m dEta", "+-1.5", 1.0,
              "WaterBank.hlsl CsBankFill (1-uv.y)"});
    Register({"swe.solver", "water.bank", "uv", rowS, atlasN, true, "m/s", "+-2.5", 1.0,
              "WaterBank.hlsl CsBankFill (1-uv.y)"});
    Register({"swe.solver", "water.bank", "shadow", rowS, atlasN, true, "0..1 exposure",
              "0.12..1", 1.0, "WaterBank.hlsl CsBankFill (1-uv.y), floor 0.18"});
    Register({"churn.kernel", "water.bank", "churn", atlasN, atlasN, false,
              "0..1 aeration", "0..1", 1.05, "WaterBank.hlsl CsBankFill flat"});
    Register({"bathy.cudem", "churn.kernel", "bed", rowS, atlasN, true, "m NAVD", "-40..15",
              1.0, "SeaChurn.hlsl suv flip"});
    Register({"bathy.cudem", "sea.ps", "bed", rowS, atlasN, true, "m NAVD", "-40..15", 1.0,
              "Sea.hlsl:71 (uv.x, 1-uv.y)"});
    Register({"swe.solver", "sea.ps", "eta", rowS, atlasN, true, "m dEta", "+-1.5", 1.0,
              "Sea.hlsl SweDEta (1-uv.y)"});
    Register({"swe.solver", "sea.ps", "shadow", rowS, atlasN, true, "0..1 exposure",
              "0.12..1", 1.0, "Sea.hlsl SweShadow (uv.x, 1-uv.y)"});
    Register({"churn.kernel", "sea.ps", "churn", atlasN, atlasN, false, "0..1 aeration",
              "0..1", 1.05, "Sea.hlsl cuv flat"});
    Register({"compose.stack", "water.bank", "corners", worldM, worldM, false,
              "m NAVD level/bed + hsScale", "hsScale 0.15..3", 1.0,
              "WaterBankLayer CornerParams (CPU)"});
    Register({"water.bank", "globe.ps", "disp/param/detail", atlasN, atlasN, false,
              "m / sigma2 / m/s / hsScale*expo", "rings 4.8..154 m/texel", 1.0,
              "Globe.hlsl BankSample manual bilinear"});
    Register({"water.bank", "globe.mesh", "disp+level", atlasN, atlasN, false, "m NAVD",
              "+-4", 1.0, "GlobeMesh.hlsl BankSample"});
}

}  // namespace ga::ast
