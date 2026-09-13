// LoadField - --load-field PATH.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "core/Common.h"
#include "core/FieldLoader.h"
#include "core/GeoGridLoader.h"
#include "core/GeoRef.h"
#include "core/GradeField.h"
#include "hal/Gpu.h"

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace ga::app::tools {

// M9h: --load-field -- the plugin path, end to end and standalone. Register a loader
// for a file type, open a file, and the sparse bank falls out of the ingest rule:
// nodata is absence, absence is a NULL tile, and the tiles that were never allocated
// are the measurement. No renderer, no scene -- just the road.
int RunLoadField(const Options& opt) {
    Gpu gpu;
    gpu.Init(nullptr, 64, 64, opt.debugLayer);
    LoaderRegistry plugin;
    plugin.Register("f32", GeoGridLoader::Open);
    plugin.Register("json", GeoGridLoader::Open);
    std::unique_ptr<FieldLoader> ld = plugin.Open(opt.loadField);
    if (!ld) {
        Log("[loader] cannot open %s (known types: f32, json)", opt.loadField.c_str());
        return 1;
    }
    const GeoRef& gr = ld->Ref();
    GradeBankDesc d;
    d.name = std::string("loaded.") + ld->Name();
    d.width = gr.width;
    d.height = gr.height;
    d.fmt = DXGI_FORMAT_R32_FLOAT;
    d.gradeSig = ld->GradeSig();
    d.metersPerTexel = gr.MetersPerTexelX();
    d.vNorth = gr.VNorth();
    d.units = gr.valueUnit;
    GradeBank bank;
    bank.Init(gpu, d, policy::None());
    const uint32_t tX = bank.TilesX(), tY = bank.TilesY();
    auto* grid = static_cast<GeoGridLoader*>(ld.get());
    const std::vector<uint8_t> mask =
        grid->CoverageMask(bank.TileW(), bank.TileH(), tX, tY);
    bank.SetPolicy(policy::Mask(mask, tX));
    bank.Update(gpu);
    bank.UploadDenseTiles(gpu, reinterpret_cast<const uint8_t*>(grid->Samples().data()),
                          gr.width, 4u, bank.ResidentList());
    const uint32_t all = tX * tY, res = bank.ResidentCount();
    char sig[16];
    snprintf(sig, sizeof(sig), "%s%s%s", (d.gradeSig & kG0) ? "g0" : "",
             (d.gradeSig & kG1) ? "g1" : "", (d.gradeSig & kG2) ? "g2" : "");
    Log("[loader] %s -> bank '%s' [%s]: %u/%u tiles resident (%.1f%%), "
        "%.1f of %.1f MB, %u tiles were pure absence",
        opt.loadField.c_str(), d.name.c_str(), sig, res, all,
        all ? 100.0 * res / all : 0.0, bank.ResidentBytes() / 1048576.0,
        bank.VirtualBytes() / 1048576.0, all - res);
    Log("[loader] georeference: %s, %.3g deg/texel (%.2f m), value %s",
        gr.Describe().c_str(), gr.scaleX, gr.MetersPerTexelX(), gr.valueUnit);
    gpu.WaitIdle();
    return 0;
}

}  // namespace ga::app::tools
