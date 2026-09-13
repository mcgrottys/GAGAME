#include "compose/SurfaceFrame.h"

#include "compose/Compositor.h"
#include "hal/Residency.h"
#include "hal/Tenant.h"
#include "sim/BathyModel.h"

#include <cmath>

namespace ga {

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
    winSlice = color.Valid() ? color.SliceOf(win.Tag()) : UINT32_MAX;
    detSlice = color.Valid() ? color.SliceOf(det.Tag()) : UINT32_MAX;
    detT = colorT;   // M9ap: the z17 page is a slice of the colour tenant (was detTenant)
    hgtT = height.Id();
    hgtWinSlice = height.Valid() ? height.SliceOf(winH.Tag()) : UINT32_MAX;
    maskT = mask.Id();
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
    const bool pages = cubeOn && winOn && window == colorCube && winSlice != UINT32_MAX;
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
    for (int i = 0; i < 4; ++i) cb.ed[i] = 0.0f;
}

}  // namespace ga
