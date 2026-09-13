// SeaVerify - --sea-verify.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "core/Common.h"
#include "hal/Gpu.h"
#include "scene/SeaLayer.h"

namespace ga::app::tools {

void RunSeaVerify(const Options&, Gpu& gpu, SeaLayer* sea) {
    const double hr = sea->MeasureRenderedHs(gpu);
    Log("[verify] sea: model Hs %.3f m, RENDERED Hs %.3f m "
        "(one realization; agreement within ~10%% passes)", sea->hsModel, hr);
}

}  // namespace ga::app::tools
