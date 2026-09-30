#include "compose/SurfaceFrame.h"

#include "compose/Compositor.h"
#include "core/GaAst.h"
#include "hal/Residency.h"
#include "hal/Tenant.h"
#include "sim/BathyModel.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace ga {

bool SurfaceFrame::DeclareBlocks(const std::string& key) {
    blocks.clear();
    struct Entry {
        FaceWindow win;
        double lat, lon, inside;   // how far inside the block's nearest edge, texels of its rung
    };
    std::vector<Entry> got;
    std::string text;
    for (const char ch : key) {
        if (ch != '"') text += ch;   // a value quoted on a command line arrives with its quotes
    }
    size_t at = 0;
    while (at < text.size()) {
        const size_t end = (std::min)(text.find(';', at), text.size());
        const std::string e = text.substr(at, end - at);
        at = end + 1;
        if (e.find_first_not_of(" \t") == std::string::npos) continue;
        double lon = 0.0, lat = 0.0;
        int rung = -1;
        if (std::sscanf(e.c_str(), " %lf , %lf , %d", &lon, &lat, &rung) != 3 || rung < 0 ||
            rung > 17 || lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
            Log("[surface] streaming.faceWindows: '%s' is not lon,lat,rung (degrees, a rung of 0 "
                "to 17) -- the key is REFUSED and the Mercator windows stand",
                e.c_str());
            return false;
        }
        // The block of that rung holding the point: the point's texel on its face's lattice at
        // the rung, divided into blocks of 16384 (FaceWindow's anchor is the block's origin).
        const double kDeg = 3.141592653589793 / 180.0;
        const double d[3] = {std::cos(lat * kDeg) * std::cos(lon * kDeg), std::sin(lat * kDeg),
                             std::cos(lat * kDeg) * std::sin(lon * kDeg)};
        double uv[2];
        const uint32_t face = CubeFaceOfDir(d, uv);
        const double N = std::ldexp(double(Lattice::kFaceDim), rung), dim = Lattice::kFaceDim;
        const double X = uv[0] * N, Y = uv[1] * N;
        const long long last = (1ll << rung) - 1;
        const long long bx = (std::min)(last, static_cast<long long>(std::floor(X / dim)));
        const long long by = (std::min)(last, static_cast<long long>(std::floor(Y / dim)));
        const FaceWindow w{face, rung, bx * Lattice::kFaceDim, by * Lattice::kFaceDim};
        const double inside = (std::min)((std::min)(X - w.anchorX, w.anchorX + dim - X),
                                         (std::min)(Y - w.anchorY, w.anchorY + dim - Y));
        bool twice = false;
        for (const Entry& g : got) {
            twice = twice || (g.win.face == w.face && g.win.rung == w.rung &&
                              g.win.anchorX == w.anchorX && g.win.anchorY == w.anchorY);
        }
        if (twice) {
            Log("[surface] streaming.faceWindows: %.5f N %.5f E at rung %d names a block already "
                "declared -- once is enough",
                lat, lon, rung);
            continue;
        }
        got.push_back({w, lat, lon, inside});
    }
    if (got.size() > kMaxBlocks) {
        Log("[surface] streaming.faceWindows: %zu blocks, the rows carry %u -- the key is REFUSED "
            "and the Mercator windows stand",
            got.size(), kMaxBlocks);
        return false;
    }
    std::stable_sort(got.begin(), got.end(),
                     [](const Entry& a, const Entry& b) { return a.win.rung < b.win.rung; });
    // HIERARCHY 4.17 commit 4: THE DIRECTORY'S LAW. Only a rank's rung (3, 6, 9, 12, 15) makes a
    // block an eighth of its parent, so that a directory cell lies in one block of the next rank or
    // in none; and every block but rank 1's needs its parent declared, or no walk reaches it
    // (rank 1 cannot be skipped).
    for (const Entry& e : got) {
        const FaceWindow& w = e.win;
        const char* why = nullptr;
        if (w.rung < 3 || w.rung % 3 != 0 || w.rung > 3 * int(kMaxRanks)) {
            why = "its rung is not a rank's (3, 6, 9, 12 or 15)";
        } else if (w.rung > 3) {
            const long long px = w.anchorX / Lattice::kFaceDim / 8 * Lattice::kFaceDim;
            const long long py = w.anchorY / Lattice::kFaceDim / 8 * Lattice::kFaceDim;
            bool parent = false;
            for (const Entry& g : got) {
                parent = parent || (g.win.face == w.face && g.win.rung == w.rung - 3 &&
                                    g.win.anchorX == px && g.win.anchorY == py);
            }
            if (!parent) why = "the block of the rank above, which a walk must pass, is not declared";
        }
        if (why) {
            Log("[surface] streaming.faceWindows: %.5f N %.5f E at rung %d: %s -- the key is REFUSED "
                "and the Mercator windows stand",
                e.lat, e.lon, w.rung, why);
            return false;
        }
    }
    for (size_t i = 0; i < got.size(); ++i) {
        const FaceWindow& w = got[i].win;
        blocks.push_back(w);
        const double g0 = Block(i).GroundRes(0);
        Log("[surface] streaming.faceWindows: %.5f N %.5f E at rung %d -> slice %zu = face %u "
            "block (%lld,%lld) of the pyramid, %.4g m a texel at its mip 0; the point stands %.0f "
            "texels (%.2f km nominal) inside its nearest edge",
            got[i].lat, got[i].lon, w.rung, 6 + i, w.face, w.anchorX / Lattice::kFaceDim,
            w.anchorY / Lattice::kFaceDim, g0, got[i].inside, got[i].inside * g0 / 1000.0);
    }
    BuildDirectory();
    return true;
}

std::string SurfaceFrame::AutoKey(std::vector<BlockWant> wants) {
    std::stable_sort(wants.begin(), wants.end(),
                     [](const BlockWant& a, const BlockWant& b) { return a.grainM < b.grainM; });
    const double kDeg = 3.141592653589793 / 180.0;
    std::vector<FaceWindow> got;
    std::string key;
    for (const BlockWant& w : wants) {
        // THE RANK THAT HOLDS ITS GRAIN: the first whose mip-0 texel, read as every source reads
        // the ground it is asked at (the pyramid's GroundRes x cos(lat)), is no coarser than it.
        const double c = std::cos(0.5 * (w.lat0 + w.lat1) * kDeg);
        auto ground = [&](uint32_t rung) {
            return hal::BlockBinding{0u, int(rung), 0u, 0u}.GroundRes(0) * c;
        };
        uint32_t rank = 1;
        while (rank < kMaxRanks && ground(3 * rank) > w.grainM) ++rank;
        if (ground(3 * rank) > w.grainM) {
            const int need = int(std::ceil(std::log2(ground(0) / w.grainM)));
            Log("[surface] faceWindows auto: %s's grain %.4g m is finer than the finest rank the "
                "directory walks (rank %u, rung %u: %.4g m a texel here); it would need rung %d%s "
                "-- painted at rung %u",
                w.name.c_str(), w.grainM, rank, 3 * rank, ground(3 * rank), need,
                need > hal::BlockBinding::kFinestRung ? ", past the pyramid's own finest too" : "",
                3 * rank);
        }
        for (uint32_t r = 1; r <= rank; ++r) {
            // The blocks of the rank's rung its footprint touches: per face, the span of block
            // indices over a grid of its points.
            const long long side = 1ll << (3 * r);   // blocks a face side at this rung
            long long x0[6], y0[6], x1[6], y1[6];
            for (int f = 0; f < 6; ++f) {
                x0[f] = y0[f] = LLONG_MAX;
                x1[f] = y1[f] = LLONG_MIN;
            }
            const int n = 32;
            for (int i = 0; i <= n * (n + 1) + n; ++i) {
                const double lon = (w.lon0 + (w.lon1 - w.lon0) * (i % (n + 1)) / n) * kDeg;
                const double lat = (w.lat0 + (w.lat1 - w.lat0) * (i / (n + 1)) / n) * kDeg;
                const double d[3] = {std::cos(lat) * std::cos(lon), std::sin(lat),
                                     std::cos(lat) * std::sin(lon)};
                double uv[2];
                const uint32_t f = CubeFaceOfDir(d, uv);
                const long long bx = (std::min)(side - 1, static_cast<long long>(std::floor(uv[0] * side)));
                const long long by = (std::min)(side - 1, static_cast<long long>(std::floor(uv[1] * side)));
                x0[f] = (std::min)(x0[f], bx);
                x1[f] = (std::max)(x1[f], bx);
                y0[f] = (std::min)(y0[f], by);
                y1[f] = (std::max)(y1[f], by);
            }
            unsigned long long missed = 0;
            for (uint32_t f = 0; f < 6; ++f) {
                if (x0[f] == LLONG_MAX) continue;
                const unsigned long long cols = x1[f] - x0[f] + 1, count = cols * (y1[f] - y0[f] + 1);
                for (unsigned long long k = 0; k < count; ++k) {
                    if (got.size() >= kMaxBlocks) {
                        missed += count - k;   // (at most: one of these may be declared already)
                        break;
                    }
                    const long long bx = x0[f] + static_cast<long long>(k % cols);
                    const long long by = y0[f] + static_cast<long long>(k / cols);
                    const FaceWindow b{f, int(3 * r), bx * Lattice::kFaceDim, by * Lattice::kFaceDim};
                    bool have = false;
                    for (const FaceWindow& g : got) {
                        have = have || (g.face == b.face && g.rung == b.rung &&
                                        g.anchorX == b.anchorX && g.anchorY == b.anchorY);
                    }
                    if (have) continue;
                    got.push_back(b);
                    double d[3];
                    ComposeCubeDir(f, (bx + 0.5) / double(side), (by + 0.5) / double(side), d);
                    char e[96];
                    snprintf(e, sizeof(e), "%.9f,%.9f,%u;", std::atan2(d[2], d[0]) / kDeg,
                             std::atan2(d[1], std::hypot(d[0], d[2])) / kDeg, 3 * r);
                    key += e;
                }
            }
            if (missed) {
                Log("[surface] faceWindows auto: %s: %llu block(s) of rank %u over its footprint "
                    "(%.6f..%.6f E, %.6f..%.6f N) did not fit -- the rows carry %u",
                    w.name.c_str(), missed, r, w.lon0, w.lon1, w.lat0, w.lat1, kMaxBlocks);
            }
        }
    }
    return key;
}

void SurfaceFrame::BuildDirectory() {
    const uint32_t slices = 6u + uint32_t(blocks.size());
    directory.assign(size_t(slices) * kCells * kCells, kNone);
    for (uint32_t s = 0; s < slices; ++s) {
        const bool isFace = s < 6u;
        const uint32_t face = isFace ? s : blocks[s - 6].face;
        const int rung = isFace ? 0 : blocks[s - 6].rung;
        const long long ox = isFace ? 0 : blocks[s - 6].anchorX / Lattice::kFaceDim;
        const long long oy = isFace ? 0 : blocks[s - 6].anchorY / Lattice::kFaceDim;
        for (uint32_t cy = 0; cy < kCells; ++cy) {
            for (uint32_t cx = 0; cx < kCells; ++cx) {
                // The next rank's block under this cell: 8 a side in this block, a cell half of one.
                const long long cbx = (ox * 8 + cx / 2) * Lattice::kFaceDim;
                const long long cby = (oy * 8 + cy / 2) * Lattice::kFaceDim;
                for (size_t i = 0; i < blocks.size(); ++i) {
                    const FaceWindow& b = blocks[i];
                    if (b.face == face && b.rung == rung + 3 && b.anchorX == cbx && b.anchorY == cby) {
                        directory[(size_t(s) * kCells + cy) * kCells + cx] = uint16_t(6 + i);
                    }
                }
            }
        }
    }
}

uint32_t SurfaceFrame::Walk(const double P[3], WalkStep out[kMaxRanks]) const {
    if (blocks.empty() || directory.empty()) return 0;
    auto cell = [&](uint32_t slice, double u, double v) {
        const int cx = (std::max)(0, (std::min)(int(kCells) - 1, int(std::floor(u * kCells))));
        const int cy = (std::max)(0, (std::min)(int(kCells) - 1, int(std::floor(v * kCells))));
        return directory[(size_t(slice) * kCells + cy) * kCells + cx];
    };
    double uv[2];
    const uint32_t face = CubeFaceOfDir(P, uv);
    uint16_t s = cell(face, uv[0], uv[1]);
    uint32_t n = 0;
    while (n < kMaxRanks && s != kNone && s >= 6u && size_t(s - 6u) < blocks.size()) {
        double x = 0.0, y = 0.0;
        blocks[s - 6u].TexelOf(P, x, y);   // relative to the block's own origin
        out[n++] = {s, x / Lattice::kFaceDim, y / Lattice::kFaceDim};
        s = cell(s, x / Lattice::kFaceDim, y / Lattice::kFaceDim);
    }
    return n;
}

void SurfaceFrame::BlockRows(const FaceWindow& block, const Placement& own, const double eye[3],
                             FaceWindow::Planes& rows, float off[2]) {
    double n[3], a[3], b[3];
    CubeFaceAxes(block.face, n, a, b);
    const bool facing = eye[0] * n[0] + eye[1] * n[1] + eye[2] * n[2] > 0.0;
    const FaceWindow anchored = facing ? block.Nearest(eye) : block;
    rows = anchored.PlanesIn(own);
    off[0] = static_cast<float>((anchored.anchorX - block.anchorX) / Lattice::kFaceDim);
    off[1] = static_cast<float>((anchored.anchorY - block.anchorY) / Lattice::kFaceDim);
}

hal::BlockBinding SurfaceFrame::Block(size_t i) const {
    const FaceWindow& w = blocks[i];
    return hal::BlockBinding{w.face, w.rung, uint32_t(w.anchorX / Lattice::kFaceDim),
                             uint32_t(w.anchorY / Lattice::kFaceDim)};
}

SurfaceFrame SurfaceFrame::Merrimack(double planetR, bool stencil) {
    SurfaceFrame s;
    s.planetR = planetR;
    s.stencil = stencil;
    // M12 step 4b: the world.flat chart -- BathyModel.h's anchor (the ACT0816 entrance
    // station) and its two frozen metres-per-degree, the constants the kernels' geoA row was
    // cast from at three sites.
    s.flat.latDeg = BathyModel::kOrgLat;
    s.flat.lonDeg = BathyModel::kOrgLon;
    s.flat.mPerLat = BathyModel::kMPerLat;
    s.flat.mPerLon = BathyModel::kMPerLon;
    s.flat.linear = true;
    // The 16k quad-sphere, and the Merrimack z14 window: tile (4935, 6008) of the z14 tile
    // grid, 16384 texels a side (was Assembly.h's `4935.0 * 256.0, 6008.0 * 256.0`, M6i).
    s.cube = Lattice::Cube(Lattice::kFaceDim);
    s.win = Lattice::Window(1263360, 1538048, 14);
    s.cubeH = Lattice::Cube(Lattice::kFaceDim, 256, 128);
    s.winH = Lattice::Window(1263360, 1538048, 14, 256, 128);
    // M7f: the z17 detail window, 16384 z17 pixels a side centred on the estuary
    // (42.8160 N, 70.8125 W) -- the closed Mercator form floored to a pixel, verbatim from
    // the colour tenant's block in Assembly.cpp.
    const double n17 = 16384.0 * 256.0 * 8.0;
    {
        const double piD = 3.14159265358979;
        const double lonC = -70.8125, latC = 42.8160 * piD / 180.0;
        const double mx = (lonC + 180.0) / 360.0 * n17;
        const double my =
            (0.5 - std::log(std::tan(piD * 0.25 + latC * 0.5)) /
                       (2.0 * piD)) *
            n17;
        const double det17OrgX = std::floor(mx - 8192.0);
        const double det17OrgY = std::floor(my - 8192.0);
        s.det = Lattice::Window(static_cast<long long>(det17OrgX),
                                static_cast<long long>(det17OrgY), 17);
    }
    return s;
}

void SurfaceFrame::Declare(const hal::Tenant& color, const hal::Tenant& height,
                           const hal::Tenant& mask) {
    colorT = color.Id();
    // HIERARCHY 4.17 commit 2: with standing blocks the colour declares no Mercator page, and
    // SliceOf would route an undeclared tag to the cube's slice 0 -- so it is not asked.
    winSlice = (color.Valid() && blocks.empty()) ? color.SliceOf(win.Tag()) : UINT32_MAX;
    detSlice = (color.Valid() && blocks.empty()) ? color.SliceOf(det.Tag()) : UINT32_MAX;
    detT = colorT;   // M9ap: the z17 page is a slice of the colour tenant (was detTenant)
    hgtT = height.Id();
    hgtWinSlice = height.Valid() ? height.SliceOf(winH.Tag()) : UINT32_MAX;
    maskT = mask.Id();
    // M12 step 4c: the tenants' own words for the diagram, read off the same declarations.
    // An empty Tenant's Desc() is the empty declaration: no node, no bindings, no row.
    auto words = [](const hal::Tenant& t) {
        AstTenant a;
        a.node = t.Desc().astNode;
        for (const hal::SliceBinding& b : t.Desc().bindings) {
            a.bindings.push_back({b.astField, b.first, b.count, b.lattice});
        }
        return a;
    };
    colorAst = words(color);
    hgtAst = words(height);
}

void SurfaceFrame::RegisterEdges() const {
    // THE COMPOSITOR PILLAR, FROM THE DECLARATIONS. A paint provider iterates a lattice's
    // texels and resolves each to lat/lon (Lattice::Texel: ComposeCubeDir for a face, the
    // Mercator inverse for a page), so every paint row runs from the exchange frame the
    // sources answer in to the lattice's own frame (Lattice::AstFrame), and whether the code
    // flips is the flip rule on those two frames (ast::NeedsFlip -- GeoRef.h's NeedsFlipInto
    // said for frames), derived here and checked again by the validator, never typed. The
    // row's triple is the tenant's node and the binding's edge; bindings that realize one edge
    // (the colour's z14 and z17 pages) fold into one row whose range lists each slice with the
    // lattice tag SliceOf resolves it by, then the tile shape; the anchor names the provider on
    // each lattice. The quantity is the table's word for the fiber, as it was.
    const ast::Frame latlon{"latlon.deg", true, 0, 0, 0};
    auto rows = [&](const AstTenant& t, const char* units) {
        for (size_t i = 0; i < t.bindings.size(); ++i) {
            const AstBinding& b = t.bindings[i];
            bool folded = false;   // an earlier binding of the same edge wrote this row
            for (size_t j = 0; j < i && !folded; ++j) {
                folded = !std::strcmp(t.bindings[j].field, b.field);
            }
            if (folded) continue;
            const ast::Frame dst = b.lattice.AstFrame();
            std::string range, tags;
            for (size_t j = i; j < t.bindings.size(); ++j) {
                const AstBinding& x = t.bindings[j];
                if (std::strcmp(x.field, b.field)) continue;
                const std::string tag = x.lattice.Tag();
                char buf[160];
                if (x.count > 1) {
                    snprintf(buf, sizeof(buf), "slices %u..%u = %s", x.first,
                             x.first + x.count - 1, tag.c_str());
                } else {
                    snprintf(buf, sizeof(buf), "slice %u = %s", x.first, tag.c_str());
                }
                range += (range.empty() ? "" : ", ") + std::string(buf);
                tags += (tags.empty() ? "" : ", ") + tag;
            }
            char tile[48];
            snprintf(tile, sizeof(tile), "; tile %ux%u", b.lattice.texW, b.lattice.texH);
            range += tile;
            const bool cube = b.lattice.kind == Lattice::Kind::Cube;
            const std::string code =
                "TileTree::Provider(" + tags + ") / " +
                (cube ? "ComposeCubeDir (composetest-pinned)"
                      : "Lattice::Texel (merc inverse per texel)");
            ast::Register({"compose.stack", t.node, b.field, latlon, dst,
                           ast::NeedsFlip(latlon, dst), units, range, 1.0, code});
        }
    };
    rows(colorAst, "sRGB bytes");
    rows(hgtAst, "m NAVD (R16F)");
}

void SurfaceFrame::Fill(ComposedSurfaceCb& cb, const ResidencyManager& rm) const {
    // FillComposedCb's parameters, read from the declaration (the banner on what `window`
    // is). rm is a reference now, so the old `rm &&` guards are gone and rm-> is rm.
    const int colorCube = colorT, heightCube = hgtT;
    const int window = winSlice != UINT32_MAX ? colorT : -1;          // the z14 page, if declared
    const int heightWindow = hgtWinSlice != UINT32_MAX ? hgtT : -1;   // Mars has none
    const int maskPages = maskT, detailWin = detT;
    const double orgPxX = static_cast<double>(win.orgPxX);
    const double orgPxY = static_cast<double>(win.orgPxY);
    const double sizePx = static_cast<double>(win.faceDim);
    const int zBase = win.zBase;
    const double detOrgPx[2] = {static_cast<double>(det.orgPxX),
                                static_cast<double>(det.orgPxY)};
    const int detailZ = det.zBase;
    const bool stencilOverlay = stencil;
    // ---- the body (Compositor.cpp's FillComposedCb, M9ap..M9ay), as it was ---------------
    const bool cubeOn = colorCube >= 0;
    const bool winOn = window >= 0;
    const bool hgtOn = heightCube >= 0;
    const bool hgtWinOn = heightWindow >= 0;
    // M9ap: the pages path. One tenant; the cube views cover slices 0..5, the array view
    // carries the Mercator pages. The old window/detail SRVs are left unset so nothing can
    // read a second texture by accident.
    // HIERARCHY 4.17 commit 2: standing blocks are pages too (the array SRV and its residency).
    const bool pages = cubeOn && ((winOn && window == colorCube && winSlice != UINT32_MAX) ||
                                  !blocks.empty());
    cb.u5[0] = pages ? rm.TextureSrv(colorCube) : UINT32_MAX;
    cb.u5[1] = pages ? rm.ResidencySrv(colorCube) : UINT32_MAX;
    cb.u5[2] = pages ? winSlice : UINT32_MAX;
    cb.u5[3] = pages ? detSlice : UINT32_MAX;
    cb.u[0] = cubeOn ? (pages ? rm.TextureSrvCube(colorCube) : rm.TextureSrv(colorCube))
                     : UINT32_MAX;
    cb.u[1] = cubeOn ? (pages ? rm.ResidencySrvCube(colorCube) : rm.ResidencySrv(colorCube))
                     : UINT32_MAX;
    cb.u[2] = (winOn && !pages) ? rm.TextureSrv(window) : UINT32_MAX;
    cb.u[3] = (winOn && !pages) ? rm.ResidencySrv(window) : UINT32_MAX;
    // M9aq: height pages -- one tenant, cube views over slices 0..5, the array view carrying
    // the z14 page. The old window SRVs are left unset so nothing can read a second texture.
    const bool hpages = hgtOn && hgtWinOn && heightWindow == heightCube && hgtWinSlice != UINT32_MAX;
    cb.u6[0] = hpages ? rm.TextureSrv(heightCube) : UINT32_MAX;
    cb.u6[1] = hpages ? rm.ResidencySrv(heightCube) : UINT32_MAX;
    cb.u6[2] = hpages ? hgtWinSlice : UINT32_MAX;
    cb.u6[3] = UINT32_MAX;
    cb.u2[0] = hgtOn ? (hpages ? rm.TextureSrvCube(heightCube) : rm.TextureSrv(heightCube))
                     : UINT32_MAX;
    cb.u2[1] = hgtOn ? (hpages ? rm.ResidencySrvCube(heightCube) : rm.ResidencySrv(heightCube))
                     : UINT32_MAX;
    cb.u2[2] = (hgtWinOn && !hpages) ? rm.TextureSrv(heightWindow) : UINT32_MAX;
    cb.u2[3] = (hgtWinOn && !hpages) ? rm.ResidencySrv(heightWindow) : UINT32_MAX;
    // M9ay: the survey MASK PAGES (gis.landsea's tree as a page tenant): array SRV + residency
    // for the Mercator pages (slices 6, 7), cube views for the faces. r = water coverage,
    // b = edited, a = surveyed. UINT32_MAX = no survey: the classifier uses the height sign.
    const bool maskOn = maskPages >= 0;
    cb.u3[0] = maskOn ? rm.TextureSrv(maskPages) : UINT32_MAX;
    cb.u3[1] = maskOn ? rm.ResidencySrv(maskPages) : UINT32_MAX;
    cb.u3[2] = maskOn ? rm.TextureSrvCube(maskPages) : UINT32_MAX;
    cb.u3[3] = maskOn ? rm.ResidencySrvCube(maskPages) : UINT32_MAX;
    cb.f[0] = cubeOn ? 1.0f : 0.0f;
    cb.f[1] = winOn ? 1.0f : 0.0f;
    cb.f[2] = hgtOn ? 1.0f : 0.0f;
    cb.f[3] = static_cast<float>(planetR);
    // The merc row: Lattice::Rows() is the old four casts bit for bit (the banner).
    win.Rows(cb.merc);
    // R16F pyramids stop at the one-tile-ish mip (16384 -> 256 = 7 levels, max lod 6); one
    // cube texel spans (pi/2)/16384 radians of arc along a face's midline.
    cb.g[0] = 6.0f;
    cb.g[1] = static_cast<float>(3.14159265358979 * 0.5 / Compositor::kFaceDim);
    cb.g[2] = 6.0f;
    cb.g[3] = stencilOverlay ? 1.0f : 0.0f;
    for (int i = 0; i < 3; ++i) {
        cb.r0[i] = static_cast<float>(east[i]);
        cb.r1[i] = static_cast<float>(up[i]);
        cb.r2[i] = static_cast<float>(north[i]);
    }
    cb.r0[3] = cb.r1[3] = cb.r2[3] = 0.0f;
    // M7f: the DETAIL color window (z17) -- the third rung of the one ladder -- plus the
    // fine edit mask (surveyed structures at ~1 m over their own bbox).
    const bool detOn = detailWin >= 0;   // detOrgPx is always here: the det lattice is declared
    cb.u4[0] = (detOn && !pages) ? rm.TextureSrv(detailWin) : UINT32_MAX;
    cb.u4[1] = (detOn && !pages) ? rm.ResidencySrv(detailWin) : UINT32_MAX;
    cb.u4[2] = UINT32_MAX;   // M9ay: the fine edit raster is gone; edits ride the mask pages
    cb.u4[3] = UINT32_MAX;
    cb.det[0] = cb.det[1] = cb.det[2] = 0.0f;
    if (detOn && zBase > 0 && sizePx > 0.0) {
        const double f = static_cast<double>(1ll << (detailZ - zBase));
        cb.det[0] = static_cast<float>((orgPxX * f - detOrgPx[0]) / 16384.0);
        cb.det[1] = static_cast<float>((orgPxY * f - detOrgPx[1]) / 16384.0);
        cb.det[2] = static_cast<float>(sizePx * f / 16384.0);
    }
    cb.det[3] = 0.0f;
    // M12 step 4f: THE GROUND ROW. The mip-0 ground texel of the three rungs, from the lattices
    // themselves (Lattice::GroundRes(0): kMercCirc / (4 faceDim) for the cube, kMercCirc / world
    // px for a window) -- 611.496.., 9.5546.., 1.1943.. -- where the shaders spelled 611, 9.55
    // and 1.19. Exact, the z14 page's mip 6 IS the cube's mip 0 (and the z17's mip 3 the z14's
    // mip 0), and PageWins takes the page there; the literals (9.55 * 64 = 611.2 > 611) took the
    // cube. The pixels that move are the ones at that boundary, measured.
    cb.ground[0] = static_cast<float>(cube.GroundRes(0));
    cb.ground[1] = static_cast<float>(win.GroundRes(0));
    cb.ground[2] = static_cast<float>(det.GroundRes(0));
    cb.ground[3] = 0.0f;
    // HIERARCHY 4.17: THE STANDING BLOCKS' ROWS, appended. Per block (commit 3) PageTexelUv's
    // three planes about the eye's own tangent frame (east, up, north at the anchor, origin the
    // eye: the frame the mesh stage's undisplaced point geo lives in), taken in doubles every
    // frame on the anchor nearest the eye, and the whole blocks back to its own origin; its
    // ground at mip 0 and its slice; the eye about the planet's centre in those axes; then the
    // count. All zero with no blocks.
    const uint32_t nb = pages ? uint32_t((std::min)(blocks.size(), size_t(kMaxBlocks))) : 0u;
    const Placement own = Placement::Frame(east, up, north, eye);
    for (uint32_t i = 0; i < kMaxBlocks; ++i) {
        FaceWindow::Planes pl{};
        float off[2] = {0.0f, 0.0f};
        if (i < nb) BlockRows(blocks[i], own, eye, pl, off);
        for (int k = 0; k < 4; ++k) {
            cb.blkU[4 * i + k] = pl.u[k];
            cb.blkV[4 * i + k] = pl.v[k];
            cb.blkW[4 * i + k] = pl.w[k];
        }
        cb.blkG[i] = i < nb ? static_cast<float>(Block(i).GroundRes(0)) : 0.0f;
        cb.blkS[i] = i < nb ? 6u + i : 0u;
        cb.blkO[2 * i] = off[0];
        cb.blkO[2 * i + 1] = off[1];
    }
    const double* const axes[3] = {east, up, north};
    for (int k = 0; k < 3; ++k) {
        const double* x = axes[k];
        cb.blkE[k] = nb ? static_cast<float>(x[0] * eye[0] + x[1] * eye[1] + x[2] * eye[2]) : 0.0f;
    }
    cb.blkE[3] = 0.0f;
    cb.blkN[0] = nb;
    cb.blkN[1] = cb.blkN[2] = cb.blkN[3] = 0u;
    // HIERARCHY 4.17 commit 4: the directory the shader walks (Walk.hlsli), and its slices.
    cb.dirU[0] = (nb && dirSrv != UINT32_MAX) ? dirSrv : UINT32_MAX;
    cb.dirU[1] = nb ? 6u + nb : 0u;
    cb.dirU[2] = cb.dirU[3] = 0u;
}

}  // namespace ga
