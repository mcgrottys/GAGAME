// TreePrune - --tree-prune / --tool tree-prune: the scene's prune.* keys handed to the tool
// (compose/TreePrune.h, whose banner says what it refuses). Declared in app/Tools.h.
#include "app/Tools.h"

#include "compose/TreePrune.h"
#include "scene/SceneSchema.h"

namespace ga::app::tools {

int RunTreePrune(const scene::PruneSection& p) {
    prune::Request q;
    // The enum's order is the schema's: list | retire | purge (scene/SceneSchema.cpp).
    q.mode = p.mode == 1 ? prune::Mode::Retire : p.mode == 2 ? prune::Mode::Purge : prune::Mode::List;
    q.root = p.root;
    q.ageDays = p.ageDays;
    q.purgeDays = p.purgeDays;
    q.confirm = p.confirm;
    return prune::ExitCode(prune::Run(q).status);
}

}  // namespace ga::app::tools
