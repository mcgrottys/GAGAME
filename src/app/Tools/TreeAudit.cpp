// TreeAudit - --tree-audit N / --pack-trees / --warm-trees.
// Moved verbatim from main.cpp (M12 step 1b); declared in app/Tools.h.
#include "app/Tools.h"

#include "compose/Compositor.h"
#include "compose/TileTree.h"
#include "core/Common.h"
#include "hal/Residency.h"
#include "core/ThreadAudit.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ga::app::tools {

// The gate. Every realization the render path uses, on tiles the shipped
// paint loop already wrote, with the worst per-channel disagreement printed.
int RunTreeAudit(const Options& opt, Compositor& compositor, int hgtCh,
                 ResidencyManager& resMgr, double det17OrgX, double det17OrgY, int colCh,
                 const std::unique_ptr<TileTree>& megaTree,
                 const std::unique_ptr<TileTree>& heightTree) {
    const ColorFrame frames[] = {
        ColorFrame::Cube(Compositor::kFaceDim),
        ColorFrame::Window(1263360, 1538048, 14),
        ColorFrame::Window(static_cast<long long>(det17OrgX),
                           static_cast<long long>(det17OrgY), 17),
    };
    const ColorFrame hframes[] = {
        ColorFrame::Cube(Compositor::kFaceDim, 256, 128),
        ColorFrame::Window(1263360, 1538048, 14, 256, 128),
    };
    if (opt.packTrees) {
        std::vector<std::string> tags;
        for (const ColorFrame& f : frames) tags.push_back(f.Tag());
        uint32_t n = megaTree->Pack(tags);
        if (heightTree) {
            std::vector<std::string> htags;
            for (const ColorFrame& f : hframes) htags.push_back(f.Tag());
            n += heightTree->Pack(htags);
        }
        Log("[tiletree] packed %u tiles across the megatexture tree; "
            "references resolve into the archives they name",
            n);
        ga::threadaudit::Report();
        resMgr.Shutdown();
        return 0;
    }
    TreeAudit all;
    if (heightTree) {
        for (const ColorFrame& f : hframes) {
            TreeAudit ha;
            AuditTileTree(compositor, hgtCh, *heightTree, f, opt.treeAudit,
                          ha, opt.warmTrees, true, "windowH");
            Log("[tree-audit] earth.height/%s: %u tiles, %u exact, worst "
                "|direct - tree| = %.3f m, %llu of %llu texels differ",
                f.Tag().c_str(), ha.tiles, ha.exact, ha.worstDelta / 1000.0,
                static_cast<unsigned long long>(ha.difTexels),
                static_cast<unsigned long long>(ha.texels));
        }
        Log("[trees]\n%s", heightTree->Stats().c_str());
    }
    for (const ColorFrame& f : frames) {
        TreeAudit a;
        AuditTileTree(compositor, colCh, *megaTree, f, opt.treeAudit, a,
                      opt.warmTrees);
        all.tiles += a.tiles;
        all.exact += a.exact;
        all.difTexels += a.difTexels;
        all.texels += a.texels;
        all.alphaDif += a.alphaDif;
        all.worstDelta = (std::max)(all.worstDelta, a.worstDelta);
    }
    Log("[tree-audit] TOTAL: %u tiles, %u byte-identical (%.1f%%), worst "
        "|direct - tree| = %u/255, %llu of %llu texels differ (%.4f%%), "
        "%u cover mismatches",
        all.tiles, all.exact,
        all.tiles ? 100.0 * double(all.exact) / double(all.tiles) : 0.0,
        all.worstDelta,
        static_cast<unsigned long long>(all.difTexels),
        static_cast<unsigned long long>(all.texels),
        all.texels ? 100.0 * double(all.difTexels) / double(all.texels)
                   : 0.0,
        all.alphaDif);
    Log("[trees]\n%s", megaTree->Stats().c_str());
    ga::threadaudit::Report();   // the tree runs hammer it hardest
    resMgr.Shutdown();
    return 0;
}

}  // namespace ga::app::tools
