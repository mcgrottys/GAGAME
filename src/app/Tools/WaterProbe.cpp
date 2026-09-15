// WaterProbe - --water-probe N.
#include "app/Tools.h"

#include "compose/ExposureSource.h"
#include "compose/SurfaceFrame.h"
#include "compose/WaterAtlas.h"
#include "core/Common.h"
#include "core/Pga.h"
#include "hal/Gpu.h"
#include "render/Camera.h"
#include "render/Renderer.h"
#include "scene/Entity.h"
#include "scene/SeaLayer.h"
#include "scene/VesselLayer.h"
#include "scene/WaterBankLayer.h"
#include "sim/OceanCpu.h"
#include "sim/SeaState.h"
#include "sim/Vessel.h"
#include "sim/WaterSurfaceTree.h"
#include "sim/WaveField.h"

#include <DirectXMath.h>

#include <algorithm>
#include <chrono>
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
//            TreeWater::At -- the displaced surface, what a hull stands on; the kernel column below
//            compares the twin at the LABEL (TreeWater::AtLabel), where the bank's texel stands.
//    BANK    the kernel's answer at the point (WaterBankLayer::ReadBankPoints: level + dispY through
//            the mesh's own reconstruction of the finest ring holding it -- the cubic for disp, the
//            tent for level), so a difference can be split into what the kernel fed the mesh
//            (BANK - PHYS) and what the raster did with it (DRAWN - BANK).
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

namespace {

double Bearing(double east, double north) {
    double b = std::atan2(east, north) * 180.0 / 3.14159265358979;
    return (b < 0.0) ? b + 360.0 : b;
}

// THE PLACE OF A ROOT-FRAME POINT, exactly: the tangent frame's rows carry it into the planet frame
// (planet = east*x + up*(R + y) + north*z, the walk's own expression), and lat/lon is the inverse of
// GlobeModel::LatLonDir on that direction (this planet frame has x at 0N 0E, y at the pole, z at
// 90E). No metres-per-degree anywhere: this is the truth the water's linear chart is measured
// against (M13 step 0).
void PlaceOfRoot(const SurfaceFrame& s, double planetR, const double p[3], double& latDeg,
                 double& lonDeg) {
    const double ry = planetR + p[1];
    const double d[3] = {s.east[0] * p[0] + s.up[0] * ry + s.north[0] * p[2],
                         s.east[1] * p[0] + s.up[1] * ry + s.north[1] * p[2],
                         s.east[2] * p[0] + s.up[2] * ry + s.north[2] * p[2]};
    const double len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    const double r2d = 180.0 / 3.14159265358979;
    latDeg = std::asin((std::min)((std::max)(d[1] / (std::max)(len, 1e-12), -1.0), 1.0)) * r2d;
    lonDeg = std::atan2(d[2], d[0]) * r2d;
}
double WrapPi(double a) {
    while (a > 3.14159265358979) a -= 6.28318530717959;
    while (a < -3.14159265358979) a += 6.28318530717959;
    return a;
}

// The dominant axis of a field's gradients over a grid (the structure tensor's major eigenvector):
// for a wave train it is the propagation axis, as a bearing modulo 180, with its coherence
// (1 = every gradient on one axis, 0 = isotropic).
template <class F>
void GradientAxis(double cx, double cz, int n, double step, F height, double& axisDeg,
                  double& coherence) {
    double jxx = 0.0, jzz = 0.0, jxz = 0.0;
    std::vector<double> h(size_t(n) * n);
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            h[size_t(j) * n + i] = height(cx + (i - n / 2) * step, cz + (j - n / 2) * step);
        }
    }
    for (int j = 1; j + 1 < n; ++j) {
        for (int i = 1; i + 1 < n; ++i) {
            const double gx = (h[size_t(j) * n + i + 1] - h[size_t(j) * n + i - 1]) / (2.0 * step);
            const double gz = (h[size_t(j + 1) * n + i] - h[size_t(j - 1) * n + i]) / (2.0 * step);
            jxx += gx * gx;
            jzz += gz * gz;
            jxz += gx * gz;
        }
    }
    // The major axis angle from +x (east) toward +z (north), then as a compass bearing mod 180.
    const double ang = 0.5 * std::atan2(2.0 * jxz, jxx - jzz);
    axisDeg = std::fmod(Bearing(std::cos(ang), std::sin(ang)), 180.0);
    const double tr = jxx + jzz;
    const double dif = std::sqrt((jxx - jzz) * (jxx - jzz) + 4.0 * jxz * jxz);
    coherence = (tr > 1e-12) ? dif / tr : 0.0;
}

}  // namespace

// THE ORIENTATION AT THE HULL (Mark, 2026-09-15: "something seems sus about the vertical bands"). The
// probe's own comparison cannot see a rotated dataset: the drawn field and the hull's agree with each
// other (both read the same trees), so a rotation they SHARE is invisible to it. The truth to hold them
// to is what the data DECLARES: each solved component carries its propagation direction and wavenumber,
// so the gradient of its stored phase must point along that direction with magnitude k; and the ambient
// sea's gradients must lie along its declared peak direction.
void ReportOrientation(const scene::Entity& e, const WaveField* waveField, const SeaLayer* sea,
                       const SeaState* seaState, double rootX, double rootZ, double simUnix,
                       uint32_t recFrame) {
    if (!e.Hull()) return;
    double cg[3] = {0.0, 0.0, 0.0};
    e.Hull()->Body().pose.TransformPoint(cg[0], cg[1], cg[2]);
    // The declared peak (the sea layer's propagation direction, east/north).
    const bool peakValid = sea && sea->PeakDirValid();
    const double peakBearing = peakValid ? Bearing(sea->PeakDirX(), sea->PeakDirZ()) : -1.0;
    Log("[wprobe]   orientation at the hull, rec%u: the declared peak TRAVELS toward bearing %.1f%s",
        recFrame, peakBearing, peakValid ? "" : " (no valid peak)");
    // ---- the solved field: phase gradient against each component's declared direction.
    // The solved field is addressed in the ROOT chart's metres (the caller hands them in, the
    // identity for a root-space hull): a carried hull simply finds no valid probe there, which is
    // an answer, where before it was not asked at all.
    if (waveField && waveField->Ready()) {
        const WaveField::GpuTable& tab = waveField->Table();
        const double h = 0.25;
        const WaveField::Probe p0 = waveField->ProbeAt(rootX, rootZ, simUnix);
        const WaveField::Probe px1 = waveField->ProbeAt(rootX + h, rootZ, simUnix);
        const WaveField::Probe px0 = waveField->ProbeAt(rootX - h, rootZ, simUnix);
        const WaveField::Probe pz1 = waveField->ProbeAt(rootX, rootZ + h, simUnix);
        const WaveField::Probe pz0 = waveField->ProbeAt(rootX, rootZ - h, simUnix);
        if (p0.valid && px1.valid && px0.valid && pz1.valid && pz0.valid) {
            double wSum = 0.0, errSum = 0.0, ratioSum = 0.0;
            int logged = 0;
            for (uint32_t c = 0; c < tab.nUsed && c < uint32_t(WaveField::kMaxComp); ++c) {
                if (!(p0.a[c] > 0.01f)) continue;
                const double gx = WrapPi(double(px1.phase[c]) - double(px0.phase[c])) / (2.0 * h);
                const double gz = WrapPi(double(pz1.phase[c]) - double(pz0.phase[c])) / (2.0 * h);
                const double gl = std::sqrt(gx * gx + gz * gz);
                if (!(gl > 1e-9)) continue;
                const double dirB = Bearing(tab.dirX[c], tab.dirZ[c]);
                const double gradB = Bearing(gx, gz);
                double err = std::fabs(gradB - dirB);
                if (err > 180.0) err = 360.0 - err;
                const double ratio = gl / (std::max)(double(p0.k[c]), 1e-9);
                wSum += p0.a[c];
                errSum += p0.a[c] * err;
                ratioSum += p0.a[c] * ratio;
                if (logged < 6) {
                    Log("[wprobe]     solved comp %2u: a %.3f m, lambda %.1f m, declared travel %.1f | "
                        "grad(phase) %.1f, |grad|/k %.2f -> %.1f deg apart",
                        c, double(p0.a[c]), 6.28318530717959 / (std::max)(double(p0.k[c]), 1e-9),
                        dirB, gradB, ratio, err);
                    ++logged;
                }
            }
            if (wSum > 0.0) {
                Log("[wprobe]     solved field: amplitude-weighted |grad(phase) - declared| %.1f deg, "
                    "|grad|/k %.2f (a correct field: ~0 deg, ~1.00)",
                    errSum / wSum, ratioSum / wSum);
            }
            // The envelope: which way the solved field's amplitude varies around the hull.
            double envAxis = 0.0, envCoh = 0.0;
            GradientAxis(rootX, rootZ, 40, 4.0,
                         [&](double x, double z) {
                             const WaveField::Probe p = waveField->ProbeAt(x, z, simUnix);
                             return p.valid ? double(p.rms) : 0.0;
                         },
                         envAxis, envCoh);
            double etaAxis = 0.0, etaCoh = 0.0;
            GradientAxis(rootX, rootZ, 64, 1.5,
                         [&](double x, double z) {
                             const WaveField::Probe p = waveField->ProbeAt(x, z, simUnix);
                             return p.valid ? double(p.eta) : 0.0;
                         },
                         etaAxis, etaCoh);
            Log("[wprobe]     solved eta gradients lie along bearing %.1f (mod 180, coherence %.2f); its "
                "rms ENVELOPE varies along %.1f (coherence %.2f)",
                etaAxis, etaCoh, envAxis, envCoh);
        }
    }
    // ---- the ambient cascades (the CPU twin the kernel's textures mirror), outside any window.
    if (sea && seaState && sea->Ocean().Ready()) {
        const double tSec =
            static_cast<double>(static_cast<float>(simUnix - seaState->CycleUnix()));
        double casAxis = 0.0, casCoh = 0.0;
        GradientAxis(cg[0], cg[2], 64, 2.0,
                     [&](double x, double z) {
                         OceanSample o;
                         sea->Ocean().Sample(x, z, tSec, o);
                         return o.h;
                     },
                     casAxis, casCoh);
        Log("[wprobe]     cascade eta gradients lie along bearing %.1f (mod 180, coherence %.2f); the "
            "declared peak's axis is %.1f",
            casAxis, casCoh, peakValid ? std::fmod(peakBearing, 180.0) : -1.0);
    }
}

void RunWaterProbe(Gpu& gpu, Renderer& renderer, const Camera& cam, double planetR,
                   const std::vector<std::unique_ptr<scene::Entity>>& entities,
                   WaterBankLayer* waterBank, const VesselLayer* vessels,
                   const WaterAtlas* atlas, const ExposureSource* exposure,
                   const WaveField* waveField, const SeaLayer* seaLayer, const SeaState* seaState,
                   const SurfaceFrame& surface, double oceanNow, double simUnix,
                   uint32_t recFrame) {
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
                    const SurfaceSample sl = sea.AtLabel(lx, lz, simUnix);   // the texel's particle
                    const double bankH = double(bp.level) + double(bp.dispY);
                    const double mesh = hDrawn - bankH;
                    const double wPhys = sl.heightNavd - level, wBank = double(bp.dispY);
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
        // (Timed: what one texel of the page costs the node to paint at 76 m -- the price of any CPU
        // evaluation of the field itself rather than a copy of the texels.)
        const auto marchT0 = std::chrono::steady_clock::now();
        const double expoNode = exposure ? double(exposure->At(cgLat, cgLon)) : 1.0;
        const double marchMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - marchT0).count();
        // The solved window lives in the ROOT chart's metres, so a carried hull asks there too
        // (RootOf; the identity for a root-space hull) instead of being skipped.
        double cgRootX = cg[0], cgRootZ = cg[2];
        sea.RootOf(cg[0], cg[2], cgRootX, cgRootZ);
        Log("[wprobe]   wave gains at the hull: exposure %.3f (the node's march, %.2f ms; the kernel's "
            "floor 0.18) | solved window weight %.3f",
            expoNode, marchMs, sea.WindowWeight(cgRootX, cgRootZ));
        // THE REGISTRATION (M13 step 0): the water's linear chart against the sphere the mesh is
        // drawn on. Both answer for the same point; the offset is how far the water's data stand
        // from the ground that carries them, and it GROWS WITH RANGE from the space's anchor -- the
        // chart is exact at its own origin -- so it is reported at the hull and at the reach of the
        // rings that draw the water around it. 0 everywhere would mean the chart IS the sphere.
        {
            const double d2r = 3.14159265358979 / 180.0;
            const double legs[4][2] = {{0.0, 0.0}, {0.0, 5000.0}, {5000.0, 0.0}, {0.0, -5000.0}};
            const char* names[4] = {"at the hull", "5 km north", "5 km east", "5 km south"};
            char line[512];
            int n = snprintf(line, sizeof(line), "[wprobe]   registration (chart - sphere):");
            for (int L = 0; L < 4; ++L) {
                double q[3] = {cg[0] + legs[L][0], cg[1], cg[2] + legs[L][1]};
                double qLat = 0.0, qLon = 0.0;
                sea.PlaceOf(q[0], q[2], qLat, qLon);   // the water's chart, in the hull's space
                if (e->InSpace()) e->SpaceInRoot().TransformPoint(q[0], q[1], q[2]);
                double sLat = 0.0, sLon = 0.0;
                PlaceOfRoot(surface, planetR, q, sLat, sLon);   // the sphere the mesh is drawn on
                n += snprintf(line + n, sizeof(line) - size_t(n), "  %s %+.1f N %+.1f E m;", names[L],
                              (qLat - sLat) * d2r * planetR,
                              (qLon - sLon) * d2r * planetR * std::cos(sLat * d2r));
            }
            Log("%s", line);
        }
        // THE BAND LAW AT THE HULL, AS EACH PROCESSOR APPLIED IT (step 3). The bank kernel writes its
        // full-closure per-band gains and its dry weight to the detail plane; the hull's twin states
        // the same law (TreeWater::BandGains). A drawn sea whose waves stand at a fraction of the
        // hull's with the phases agreeing is a GAIN, and this names which band carries it -- the node's
        // exposure above is the source's own march, not the page texels the kernel read.
        if (waterBank) {
            // The bank is addressed by the point's place on the TANGENT PLANE (the mesh's own
            // bankXZ), so a carried hull's CG is asked for there -- in the ROOT frame, where its
            // rings are anchored -- and not skipped as it was through the interests commit.
            const double cgy2 = cgRoot[1] + planetR;
            const double cgl = std::sqrt(cgRoot[0] * cgRoot[0] + cgy2 * cgy2 + cgRoot[2] * cgRoot[2]);
            const double cgXz[2] = {planetR * cgRoot[0] / cgl, planetR * cgRoot[2] / cgl};
            WaterBankLayer::BankPoint bc;
            waterBank->ReadBankPoints(gpu, cgXz, 1, &bc);
            double g[OceanCpu::kCascades] = {0.0, 0.0, 0.0}, dryC = 0.0;
            const bool haveC = sea.BandGains(cg[0], cg[2], simUnix, g, dryC);
            Log("[wprobe]   band gains at the hull (swell, wind sea, chop; dry): kernel %s%.3f %.3f %.3f; "
                "%.3f | hull %s%.3f %.3f %.3f; %.3f",
                bc.valid ? "" : "(no ring) ", double(bc.gain0), double(bc.gain1), double(bc.gain2),
                double(bc.dry), haveC ? "" : "(no water) ", g[0], g[1], g[2], dryC);

        }
        ReportOrientation(*e, waveField, seaLayer, seaState, cgRootX, cgRootZ, simUnix, recFrame);
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
