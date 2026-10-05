// HullProbe - --hull-probe N (the Phase C water audit, 2026-10-04). An instrument, never in play.
//
// Every N frames, for every hull the sim steps, at the hull:
//   TWIN   the hull's own water (Entity::Sea): At (the displaced surface the hull stands on, and the
//          slope [vessel] water: prints) and AtLabel (the particle whose LABEL is the hull's place,
//          where the bank's texel stands).
//   BANK   the kernel's answer at the hull's place on the tangent plane (ReadBankPoints: level +
//          dispY through the finest ring holding it).
//   DRAWN  the scene depth lifted to the planet in doubles (as --water-probe does) at every water
//          pixel within 15 m of the hull (pixels the hull covers and pixels of another level are
//          left out): drawn height against the twin's At and the bank at the SAME point.
//   and the level source, the sea-state source, the band scale and the exposure at the hull.
// Then a 48 x 48 grid at 40 m about the hull: twin and bank wave eta classed by the twin's depth.
#include "app/Tools.h"

#include "compose/ExposureSource.h"
#include "core/Common.h"
#include "core/Pga.h"
#include "hal/Gpu.h"
#include "render/Camera.h"
#include "render/Renderer.h"
#include "scene/Entity.h"
#include "scene/Gateway.h"
#include "scene/VesselLayer.h"
#include "scene/WaterBankLayer.h"
#include "sim/Vessel.h"
#include "sim/WaterSurfaceTree.h"

#include <DirectXMath.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace ga::app::tools {

namespace {
double SlopeDeg(const SurfaceSample& s) {
    const double sl = std::sqrt(s.nx * s.nx + s.nz * s.nz) /
                      ((std::abs(s.ny) > 1e-9) ? std::abs(s.ny) : 1e-9);
    return std::atan(sl) * 57.2957795;
}
struct Cls {
    int n = 0;
    double bw2 = 0, tw2 = 0, d2 = 0, slMax = 0;
};
}  // namespace

void RunHullProbe(Gpu& gpu, Renderer& renderer, const Camera& cam, double planetR,
                  const std::vector<std::unique_ptr<scene::Entity>>& entities,
                  WaterBankLayer* waterBank, const VesselLayer* vessels,
                  const ExposureSource* exposure, double simUnix, uint32_t frame,
                  const std::vector<scene::WindowLink>* windows) {
    std::vector<float> depth;
    const bool haveDepth = renderer.ReadDepth(depth);
    const uint32_t W = renderer.Width(), H = renderer.Height();
    const double aspect = double(W) / double((std::max)(H, 1u));
    const double tanH = std::tan(0.5 * double(cam.fovY));
    DirectX::XMFLOAT3 f3, r3, u3;
    cam.ViewBasis(f3, r3, u3);
    const double fw[3] = {f3.x, f3.y, f3.z}, rt[3] = {r3.x, r3.y, r3.z}, up[3] = {u3.x, u3.y, u3.z};
    const double eye[3] = {cam.px, cam.py, cam.pz};
    const DirectX::XMMATRIX vp = cam.ViewRelative() * cam.Projection(float(aspect));
    auto tangentXz = [&](const double p[3], double* xz) {
        const double gy = p[1] + planetR;
        const double gl = std::sqrt(p[0] * p[0] + gy * gy + p[2] * p[2]);
        xz[0] = planetR * p[0] / gl;
        xz[1] = planetR * p[2] / gl;
    };

    for (const auto& e : entities) {
        if (!e->Active() || !e->Placed() || !e->Hull()) continue;
        const TreeWater& sea = e->Sea();
        const Motor toLocal = e->InSpace() ? e->SpaceInRoot().Inverse() : Motor::Identity();
        double cg[3] = {0.0, 0.0, 0.0};
        e->Hull()->Body().pose.TransformPoint(cg[0], cg[1], cg[2]);
        double cgRoot[3] = {cg[0], cg[1], cg[2]};
        if (e->InSpace()) e->SpaceInRoot().TransformPoint(cgRoot[0], cgRoot[1], cgRoot[2]);

        // ---- TWIN at the hull.
        const SurfaceSample tw = sea.At(cg[0], cg[2], simUnix);
        const SurfaceSample tl = sea.AtLabel(cg[0], cg[2], simUnix);
        const double lev = sea.MeanLevelAt(cg[0], cg[2], simUnix);
        double cgLat = 0.0, cgLon = 0.0;
        sea.PlaceOf(cg[0], cg[2], cgLat, cgLon);
        const WeatherSample q = sea.MeanStateAt(cg[0], cg[2], simUnix);
        const double expo = exposure ? double(exposure->At(cgLat, cgLon)) : -1.0;
        double gT[OceanCpu::kCascades] = {0, 0, 0}, dryT = 0.0;
        sea.BandGains(cg[0], cg[2], simUnix, gT, dryT);

        // ---- DRAWN pixels near the hull: a box about the CG's pixel.
        struct Px { double p[3]; double lx, lz, r, hDrawn; };
        std::vector<Px> nearPx;
        double cgPx = -1, cgPy = -1;
        {
            const double rel[3] = {cgRoot[0] - eye[0], cgRoot[1] - eye[1], cgRoot[2] - eye[2]};
            const double vz = rel[0] * fw[0] + rel[1] * fw[1] + rel[2] * fw[2];
            if (haveDepth && vz > 0.5) {
                const DirectX::XMVECTOR c = DirectX::XMVector3TransformCoord(
                    DirectX::XMVectorSet(float(rel[0]), float(rel[1]), float(rel[2]), 1.0f), vp);
                cgPx = (double(DirectX::XMVectorGetX(c)) + 1.0) * 0.5 * double(W) - 0.5;
                cgPy = (1.0 - double(DirectX::XMVectorGetY(c))) * 0.5 * double(H) - 0.5;
                // 15 m at range vz spans this many pixels; scan a bit beyond.
                const double pxPerM = double(H) / (2.0 * tanH * vz);
                const int half = int((std::min)(400.0, 18.0 * pxPerM + 4.0));
                const int stepP = (std::max)(1, half / 40);
                for (int py = int(cgPy) - half; py <= int(cgPy) + half; py += stepP) {
                    if (py < 0 || py >= int(H)) continue;
                    for (int px = int(cgPx) - half; px <= int(cgPx) + half; px += stepP) {
                        if (px < 0 || px >= int(W)) continue;
                        const float d = depth[size_t(py) * W + size_t(px)];
                        if (!(d > 0.0f)) continue;
                        const double viewZ = double(cam.nearZ) / double(d);
                        const double nx = (double(px) + 0.5) / double(W) * 2.0 - 1.0;
                        const double ny = 1.0 - (double(py) + 0.5) / double(H) * 2.0;
                        Px s;
                        for (int i = 0; i < 3; ++i)
                            s.p[i] = eye[i] + viewZ * (fw[i] + nx * tanH * aspect * rt[i] + ny * tanH * up[i]);
                        if (vessels && vessels->Occupies(s.p[0], s.p[1], s.p[2], 0.05)) continue;
                        if (windows && !windows->empty() && scene::ChainDepth(*windows, eye, s.p) != 0) continue;
                        const double gy = s.p[1] + planetR;
                        s.hDrawn = std::sqrt(s.p[0] * s.p[0] + gy * gy + s.p[2] * s.p[2]) - planetR;
                        double qq[3] = {s.p[0], s.p[1], s.p[2]};
                        toLocal.TransformPoint(qq[0], qq[1], qq[2]);
                        const double qy = qq[1] + planetR;
                        const double ql = std::sqrt(qq[0] * qq[0] + qy * qy + qq[2] * qq[2]);
                        s.lx = planetR * qq[0] / ql;
                        s.lz = planetR * qq[2] / ql;
                        s.r = std::hypot(s.lx - cg[0], s.lz - cg[2]);
                        if (s.r > 15.0) continue;
                        nearPx.push_back(s);
                    }
                }
            }
        }

        // ---- THE GRID about the hull (hull frame), its tangent-plane places in the root.
        const int G = 64;
        const double GS = 80.0;
        std::vector<double> gx(size_t(G) * G), gz(size_t(G) * G);
        for (int j = 0; j < G; ++j)
            for (int i = 0; i < G; ++i) {
                gx[size_t(j) * G + i] = cg[0] + (i - G / 2) * GS;
                gz[size_t(j) * G + i] = cg[2] + (j - G / 2) * GS;
            }

        // ---- ONE bank readback: the CG, the near pixels, the grid.
        const size_t nPts = 1 + nearPx.size() + gx.size();
        std::vector<double> xz(nPts * 2);
        tangentXz(cgRoot, &xz[0]);
        for (size_t k = 0; k < nearPx.size(); ++k) tangentXz(nearPx[k].p, &xz[(1 + k) * 2]);
        for (size_t k = 0; k < gx.size(); ++k) {
            double p[3] = {gx[k], lev, gz[k]};
            if (e->InSpace()) e->SpaceInRoot().TransformPoint(p[0], p[1], p[2]);
            tangentXz(p, &xz[(1 + nearPx.size() + k) * 2]);
        }
        std::vector<WaterBankLayer::BankPoint> bank(nPts);
        if (waterBank) waterBank->ReadBankPoints(gpu, xz.data(), int(nPts), bank.data());
        const WaterBankLayer::BankPoint& bc = bank[0];
        const double bankEta = bc.valid ? double(bc.level) + double(bc.dispY) : std::nan("");

        // ---- DRAWN near the hull.
        int nD = 0, nDB = 0;
        double sDT = 0, sDT2 = 0, sDB = 0, sDB2 = 0, sDraw = 0, nearR = 1e9, nearDT = 0, nearDB = 0, nearH = 0;
        for (size_t k = 0; k < nearPx.size(); ++k) {
            const Px& s = nearPx[k];
            const SurfaceSample ts = sea.At(s.lx, s.lz, simUnix);
            if (!ts.valid) continue;
            const double dT = s.hDrawn - ts.heightNavd;
            ++nD;
            sDT += dT;
            sDT2 += dT * dT;
            sDraw += s.hDrawn;
            const WaterBankLayer::BankPoint& b = bank[1 + k];
            double dB = std::nan("");
            if (b.valid) {
                dB = s.hDrawn - (double(b.level) + double(b.dispY));
                ++nDB;
                sDB += dB;
                sDB2 += dB * dB;
            }
            if (s.r < nearR) { nearR = s.r; nearDT = dT; nearDB = dB; nearH = s.hDrawn; }
        }

        Log("[hprobe] f%u '%s' cg %.1f %.1f inSpace %d lat %.6f lon %.6f | twin At %+.3f slope %.2f deg depth %.2f "
            "| twin Label %+.3f level %+.3f | bank %d ring %d level %+.3f dispY %+.3f eta %+.3f dry %.2f "
            "| near n %d drawn-twin mean %+.3f rms %.3f | drawn-bank n %d mean %+.3f rms %.3f | drawn mean %+.3f "
            "| nearest r %.2f drawn %+.3f d-twin %+.3f d-bank %+.3f | cg pixel %.0f %.0f",
            frame, e->Name(), cg[0], cg[2], e->InSpace() ? 1 : 0, cgLat, cgLon, tw.heightNavd, SlopeDeg(tw),
            double(tw.depthM), tl.heightNavd, lev, bc.valid ? 1 : 0, bc.ring, double(bc.level), double(bc.dispY),
            bankEta, double(bc.dry), nD, nD ? sDT / nD : 0.0, nD ? std::sqrt(sDT2 / nD) : 0.0, nDB,
            nDB ? sDB / nDB : 0.0, nDB ? std::sqrt(sDB2 / nDB) : 0.0, nD ? sDraw / nD : 0.0,
            nearR < 1e8 ? nearR : -1.0, nearH, nearDT, nearDB, cgPx, cgPy);
        Log("[hprobe] f%u src | level %s %+.3f | current %s (%+.2f, %+.2f) | sea %s Hs %.2f Tp %.1f from %.0f "
            "| band scale x%.3f ref Hs %.3f | exposure %.3f | gains kernel %.3f %.3f %.3f twin %.3f %.3f %.3f",
            frame, q.levelSrc, q.levelNavd, q.currentSrc, double(q.u), double(q.v), q.waveSrc, double(q.hs),
#ifdef GA_HULLPROBE_B
            double(q.tp), double(q.dirDeg), sea.SeaScaleAtT(cgLat, cgLon, simUnix), sea.SeaScaleRefT(simUnix), expo,
#else
            double(q.tp), double(q.dirDeg), sea.SeaScaleAt(cgLat, cgLon), sea.SeaScaleRef(), expo,
#endif
            double(bc.gain0), double(bc.gain1), double(bc.gain2), gT[0], gT[1], gT[2]);

        // ---- THE GRID by depth: < 7 m, 7..20 m, > 20 m.
        Cls cls[3];
        int gTinv = 0, gTdry = 0, gBinv = 0, gRing[8] = {};
        for (size_t k = 0; k < gx.size(); ++k) {
            const SurfaceSample sl = sea.AtLabel(gx[k], gz[k], simUnix);
            if (!sl.valid) { ++gTinv; continue; }
            if (sl.depthM < 0.25) { ++gTdry; continue; }
            const WaterBankLayer::BankPoint& b = bank[1 + nearPx.size() + k];
            if (!b.valid) { ++gBinv; continue; }
            if (b.ring >= 0 && b.ring < 8) ++gRing[b.ring];
            const double l = sea.MeanLevelAt(gx[k], gz[k], simUnix);
            const SurfaceSample sa = sea.At(gx[k], gz[k], simUnix);
            const double dep = double(sl.depthM);
            Cls& c = cls[dep < 7.0 ? 0 : (dep <= 20.0 ? 1 : 2)];
            // bank: its own wave (dispY about its own level); twin: Label about its own mean level;
            // diff: the whole eta, bank (level + dispY) against the twin's Label.
            const double bw = double(b.dispY), twv = sl.heightNavd - l;
            const double dd = double(b.level) + double(b.dispY) - sl.heightNavd;
            ++c.n;
            c.bw2 += bw * bw;
            c.tw2 += twv * twv;
            c.d2 += dd * dd;
            if (sa.valid) c.slMax = (std::max)(c.slMax, SlopeDeg(sa));
        }
        Log("[hprobe] f%u grid | d<7 n %d bank %.3f twin %.3f diff %.3f slmax %.1f | 7-20 n %d bank %.3f twin %.3f "
            "diff %.3f slmax %.1f | d>20 n %d bank %.3f twin %.3f diff %.3f slmax %.1f",
            frame, cls[0].n, cls[0].n ? std::sqrt(cls[0].bw2 / cls[0].n) : 0.0,
            cls[0].n ? std::sqrt(cls[0].tw2 / cls[0].n) : 0.0, cls[0].n ? std::sqrt(cls[0].d2 / cls[0].n) : 0.0,
            cls[0].slMax, cls[1].n, cls[1].n ? std::sqrt(cls[1].bw2 / cls[1].n) : 0.0,
            cls[1].n ? std::sqrt(cls[1].tw2 / cls[1].n) : 0.0, cls[1].n ? std::sqrt(cls[1].d2 / cls[1].n) : 0.0,
            cls[1].slMax, cls[2].n, cls[2].n ? std::sqrt(cls[2].bw2 / cls[2].n) : 0.0,
            cls[2].n ? std::sqrt(cls[2].tw2 / cls[2].n) : 0.0, cls[2].n ? std::sqrt(cls[2].d2 / cls[2].n) : 0.0,
            cls[2].slMax);
        Log("[hprobe] f%u gridcount | %d points at %.0f m | twin invalid %d | twin dry %d | bank invalid %d | "
            "rings %d %d %d %d %d %d %d %d",
            frame, G * G, GS, gTinv, gTdry, gBinv, gRing[0], gRing[1], gRing[2], gRing[3], gRing[4], gRing[5],
            gRing[6], gRing[7]);
    }
}

}  // namespace ga::app::tools
