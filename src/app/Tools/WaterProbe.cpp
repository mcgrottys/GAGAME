// WaterProbe - --water-probe N.
#include "app/Tools.h"

#include "compose/ExposureSource.h"
#include "compose/WaterAtlas.h"
#include "core/Common.h"
#include "core/Pga.h"
#include "hal/Gpu.h"
#include "render/Camera.h"
#include "render/Renderer.h"
#include "scene/Entity.h"
#include "scene/VesselLayer.h"
#include "scene/WaterBankLayer.h"
#include "sim/Vessel.h"
#include "sim/WaterSurfaceTree.h"

#include <DirectXMath.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace ga::app::tools {

// ==================================================================================================
//  --water-probe N: DOES THE SEA WE DRAW STAND WHERE THE SEA A HULL READS STANDS?
//
//  Every N recorded frames, for every hull the sim is stepping. Three answers at the same points:
//
//    DRAWN   the rasterized surface itself. The scene depth is read back, every sampled pixel lifted
//            to its point P in doubles (the eye, the view basis ViewRelative builds, viewZ = nearZ /
//            depth -- Camera::Projection's reversed Z), and its height is |P - C| - R over the
//            planet's centre C = (0, -R, 0): the drawn surface's own NAVD height, whatever the mesh,
//            the rings, the morph or the lateral displacement did to put it there.
//    PHYS    the hull's OWN water, in its play state: Entity::Sea(), at the point's place in the
//            hull's frame (its space's placement undone), at this frame's instant. No mirror refresh
//            -- a hull in play reads none (the blind spot of --twin-surface, which refreshes first).
//    BANK    the kernel's answer at the point (WaterBankLayer::ReadBankPoints: level + dispY, the
//            finest ring's nearest texel), so a difference can be split into what the kernel fed the
//            mesh (BANK - PHYS) and what the raster did with it (DRAWN - BANK).
//
//  Pixels that land on a hull (VesselLayer::Occupies) and points the hull's water calls dry or does
//  not cover are skipped. Statistics are binned by horizontal distance from the hull, because the
//  camera-anchored rings make range the axis the answer should depend on and the waterline is the
//  one place the eye judges it. The hull's own line reports its freeboard in its water and the
//  plane-over-sphere lift its flat frame puts under it (the drawn hull stands that much higher than
//  its flat y says).
//
//  An instrument: two readbacks and a WaitIdle per probe frame. Never on in play.
// ==================================================================================================
namespace {

struct Bin {
    const double r0, r1;
    int n = 0;
    double sumD = 0, sumD2 = 0, maxD = 0;         // drawn - phys
    double sumMesh = 0, sumMesh2 = 0;             // drawn - bank
    int nBank = 0;
    double sumL = 0, sumL2 = 0;                   // bank level - phys level
    double sumW2 = 0;                             // (bank waves - phys waves)^2
    double wc = 0, wg = 0, wcg = 0;               // wave rms and correlation, phys vs bank
};

}  // namespace

void RunWaterProbe(Gpu& gpu, Renderer& renderer, const Camera& cam, double planetR,
                   const std::vector<std::unique_ptr<scene::Entity>>& entities,
                   WaterBankLayer* waterBank, const VesselLayer* vessels,
                   const WaterAtlas* atlas, const ExposureSource* exposure, double oceanNow,
                   double simUnix, uint32_t recFrame) {
    std::vector<float> depth;
    if (!renderer.ReadDepth(depth)) {
        Log("[wprobe] rec%u: the depth readback failed", recFrame);
        return;
    }
    const uint32_t W = renderer.Width(), H = renderer.Height();
    const double aspect = double(W) / double((std::max)(H, 1u));
    const double tanH = std::tan(0.5 * double(cam.fovY));
    DirectX::XMFLOAT3 f3, r3, u3;
    cam.ViewBasis(f3, r3, u3);
    const double fw[3] = {f3.x, f3.y, f3.z}, rt[3] = {r3.x, r3.y, r3.z}, up[3] = {u3.x, u3.y, u3.z};
    const double eye[3] = {cam.px, cam.py, cam.pz};
    // THE CHECK THAT THE LIFT IS THE RENDERER'S: every sample re-projected through the matrices the
    // frame was drawn with must land back on its own pixel centre. A sign or handedness slip in the
    // basis shows here as whole pixels, before a single height is believed.
    const DirectX::XMMATRIX vp = cam.ViewRelative() * cam.Projection(float(aspect));

    // The samples: a grid over the frame, every `step` pixels, where something was drawn.
    const uint32_t step = (std::max)(4u, W / 160u);
    struct Sample { double p[3]; };
    std::vector<Sample> samples;
    samples.reserve((W / step) * (H / step));
    double reprojMax = 0.0;
    for (uint32_t py = step / 2; py < H; py += step) {
        for (uint32_t px = step / 2; px < W; px += step) {
            const float d = depth[size_t(py) * W + px];
            if (!(d > 0.0f)) continue;   // nothing drawn: the clear value
            const double viewZ = double(cam.nearZ) / double(d);
            const double nx = (double(px) + 0.5) / double(W) * 2.0 - 1.0;
            const double ny = 1.0 - (double(py) + 0.5) / double(H) * 2.0;
            Sample s;
            for (int i = 0; i < 3; ++i) {
                s.p[i] = eye[i] + viewZ * (fw[i] + nx * tanH * aspect * rt[i] + ny * tanH * up[i]);
            }
            const DirectX::XMVECTOR rel = DirectX::XMVectorSet(
                float(s.p[0] - eye[0]), float(s.p[1] - eye[1]), float(s.p[2] - eye[2]), 1.0f);
            const DirectX::XMVECTOR c = DirectX::XMVector3TransformCoord(rel, vp);
            const double rx = (double(DirectX::XMVectorGetX(c)) + 1.0) * 0.5 * double(W) - 0.5;
            const double ry = (1.0 - double(DirectX::XMVectorGetY(c))) * 0.5 * double(H) - 0.5;
            reprojMax = (std::max)(reprojMax, std::hypot(rx - double(px), ry - double(py)));
            if (vessels && vessels->Occupies(s.p[0], s.p[1], s.p[2], 0.05)) continue;
            samples.push_back(s);
        }
    }

    // The bank at every sample, once: its rings are indexed by the point's place on the tangent
    // plane, R (east.dir, north.dir) -- the mesh's own bankXZ (GlobeMesh.hlsl).
    std::vector<double> xz(samples.size() * 2);
    for (size_t k = 0; k < samples.size(); ++k) {
        const double* p = samples[k].p;
        const double gy = p[1] + planetR;
        const double gl = std::sqrt(p[0] * p[0] + gy * gy + p[2] * p[2]);
        xz[k * 2] = planetR * p[0] / gl;
        xz[k * 2 + 1] = planetR * p[2] / gl;
    }
    std::vector<WaterBankLayer::BankPoint> bank(samples.size());
    if (waterBank && !samples.empty()) {
        waterBank->ReadBankPoints(gpu, xz.data(), int(samples.size()), bank.data());
    }

    for (const auto& e : entities) {
        if (!e->Active() || !e->Placed() || !e->Hull()) continue;
        const TreeWater& sea = e->Sea();
        // The hull's frame: its space's placement undone (identity for the root's own hulls).
        const Motor toLocal = e->InSpace() ? e->SpaceInRoot().Inverse() : Motor::Identity();
        double cg[3] = {0.0, 0.0, 0.0};
        e->Hull()->Body().pose.TransformPoint(cg[0], cg[1], cg[2]);   // the CG, the hull's frame
        double cgRoot[3] = {cg[0], cg[1], cg[2]};
        if (e->InSpace()) e->SpaceInRoot().TransformPoint(cgRoot[0], cgRoot[1], cgRoot[2]);
        const double cgy = cgRoot[1] + planetR;
        const double cgDrawn =
            std::sqrt(cgRoot[0] * cgRoot[0] + cgy * cgy + cgRoot[2] * cgRoot[2]) - planetR;
        const SurfaceSample atCg = sea.At(cg[0], cg[2], simUnix);

        Bin bins[] = {{0.0, 10.0}, {10.0, 30.0}, {30.0, 100.0}, {100.0, 300.0}, {300.0, 1e30}};
        int skippedDry = 0;
        // THE LEVEL, ATTRIBUTED AT THE HULL without touching play (no mirror refresh): what the hull
        // reads (the atlas at 1 m, or the mirror if its cadence filled one), the bank's corner law
        // (the atlas at 500 m -- CornerParams), and the ocean station the solver is forced by. The
        // bank's level minus the corner law is then the solver's live deviation (dEta) plus the
        // corner lerp; the corner law minus the hull's is the atlas's resolution.
        double cgLat = 0.0, cgLon = 0.0;
        sea.PlaceOf(cg[0], cg[2], cgLat, cgLon);
        const double levelHull = sea.MeanLevelAt(cg[0], cg[2], simUnix);
        const double levelCorner = (atlas && atlas->Ready())
                                       ? atlas->MslNavd(cgLat, cgLon) +
                                             atlas->Level(cgLat, cgLon, simUnix, 500.0)
                                       : std::nan("");
        double bankLevelNear = 0.0;
        int nBankNear = 0;
        for (size_t k = 0; k < samples.size(); ++k) {
            const double* p = samples[k].p;
            // The drawn height: over the sphere, in doubles.
            const double gy = p[1] + planetR;
            const double gl = std::sqrt(p[0] * p[0] + gy * gy + p[2] * p[2]);
            const double hDrawn = gl - planetR;
            // The same place in the hull's frame, on its tangent plane (where its chart is read).
            double q[3] = {p[0], p[1], p[2]};
            toLocal.TransformPoint(q[0], q[1], q[2]);
            const double qy = q[1] + planetR;
            const double ql = std::sqrt(q[0] * q[0] + qy * qy + q[2] * q[2]);
            const double lx = planetR * q[0] / ql, lz = planetR * q[2] / ql;
            const SurfaceSample s = sea.At(lx, lz, simUnix);
            if (!s.valid || s.depthM < 0.25) { ++skippedDry; continue; }
            const double dD = hDrawn - s.heightNavd;
            const double r = std::hypot(lx - cg[0], lz - cg[2]);
            for (Bin& b : bins) {
                if (r < b.r0 || r >= b.r1) continue;
                ++b.n;
                b.sumD += dD;
                b.sumD2 += dD * dD;
                if (std::abs(dD) > std::abs(b.maxD)) b.maxD = dD;
                const WaterBankLayer::BankPoint& bp = bank[k];
                if (bp.valid && r < 10.0) {
                    bankLevelNear += double(bp.level);
                    ++nBankNear;
                }
                if (bp.valid) {
                    const double level = sea.MeanLevelAt(lx, lz, simUnix);
                    const double bankH = double(bp.level) + double(bp.dispY);
                    const double mesh = hDrawn - bankH;
                    const double wPhys = s.heightNavd - level, wBank = double(bp.dispY);
                    ++b.nBank;
                    b.sumMesh += mesh;
                    b.sumMesh2 += mesh * mesh;
                    b.sumL += double(bp.level) - level;
                    b.sumL2 += (double(bp.level) - level) * (double(bp.level) - level);
                    b.sumW2 += (wBank - wPhys) * (wBank - wPhys);
                    b.wc += wPhys * wPhys;
                    b.wg += wBank * wBank;
                    b.wcg += wPhys * wBank;
                }
                break;
            }
        }
        Log("[wprobe] rec%u '%s' cg (%.1f, %+.3f, %.1f) | freeboard in its water %+.3f m | "
            "plane-over-sphere lift %+.3f m | %zu samples (%d dry or uncovered) | reprojection max "
            "%.3f px",
            recFrame, e->Name(), cg[0], cg[1], cg[2], atCg.valid ? cg[1] - atCg.heightNavd : 0.0,
            cgDrawn - cg[1], samples.size(), skippedDry, reprojMax);
        Log("[wprobe]   level at the hull (%.5f, %.5f): hull reads %+.3f | corner law (atlas @500 m) "
            "%+.3f | bank within 10 m %+.3f (n=%d) | ocean station %+.3f -> bank-corner %+.3f, "
            "corner-hull %+.3f",
            cgLat, cgLon, levelHull, levelCorner, nBankNear ? bankLevelNear / nBankNear : 0.0,
            nBankNear, oceanNow, nBankNear ? bankLevelNear / nBankNear - levelCorner : 0.0,
            levelCorner - levelHull);
        // THE WAVE GAINS AT THE HULL that only the kernel applies today: the swell shadow (the node's
        // own march, once -- an instrument can afford it) and the solved window's weight.
        Log("[wprobe]   wave gains at the hull: exposure %.3f (the kernel's floor 0.18) | solved window "
            "weight %.3f",
            exposure ? double(exposure->At(cgLat, cgLon)) : 1.0,
            e->InSpace() ? 0.0 : sea.WindowWeight(cg[0], cg[2]));
        for (const Bin& b : bins) {
            if (b.n == 0) continue;
            const double mean = b.sumD / b.n, rms = std::sqrt(b.sumD2 / b.n);
            if (b.nBank > 0) {
                const double wcr = std::sqrt(b.wc / b.nBank), wgr = std::sqrt(b.wg / b.nBank);
                Log("[wprobe]   %4.0f-%-5.0f m n=%5d  DRAWN-PHYS mean %+.3f rms %.3f worst %+.3f | "
                    "BANK-PHYS level %+.3f (rms %.3f) waves rms %.3f [phys %.3f bank %.3f corr %+.2f] | "
                    "DRAWN-BANK mean %+.3f rms %.3f (bank n=%d)",
                    b.r0, (std::min)(b.r1, 99999.0), b.n, mean, rms, b.maxD, b.sumL / b.nBank,
                    std::sqrt(b.sumL2 / b.nBank), std::sqrt(b.sumW2 / b.nBank), wcr, wgr,
                    (wcr > 1e-9 && wgr > 1e-9) ? (b.wcg / b.nBank) / (wcr * wgr) : 0.0,
                    b.sumMesh / b.nBank, std::sqrt(b.sumMesh2 / b.nBank), b.nBank);
            } else {
                Log("[wprobe]   %4.0f-%-5.0f m n=%5d  DRAWN-PHYS mean %+.3f rms %.3f worst %+.3f | "
                    "no bank here",
                    b.r0, (std::min)(b.r1, 99999.0), b.n, mean, rms, b.maxD);
            }
        }
    }
}

}  // namespace ga::app::tools
