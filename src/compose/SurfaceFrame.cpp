#include "compose/SurfaceFrame.h"

#include "compose/Compositor.h"
#include "core/GaAst.h"
#include "hal/Residency.h"
#include "hal/Tenant.h"

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
            Log("[surface] standing blocks: '%s' is not lon,lat,rung (degrees, a rung of 0 "
                "to 17) -- the key is REFUSED",
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
            Log("[surface] standing blocks: %.5f N %.5f E at rung %d names a block already "
                "declared -- once is enough",
                lat, lon, rung);
            continue;
        }
        got.push_back({w, lat, lon, inside});
    }
    if (got.size() > kMaxBlocks) {
        Log("[surface] standing blocks: %zu blocks, the rows carry %u -- the key is REFUSED "
            "",
            got.size(), kMaxBlocks);
        return false;
    }
    std::stable_sort(got.begin(), got.end(),
                     [](const Entry& a, const Entry& b) { return a.win.rung < b.win.rung; });
    // HIERARCHY 4.17: only a rank's rung (3, 6, 9, 12, 15) makes a block an eighth of its parent,
    // and every block but rank 1's needs its parent declared (rank 1 cannot be skipped).
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
            if (!parent) why = "the block of the rank above is not declared";
        }
        if (why) {
            Log("[surface] standing blocks: %.5f N %.5f E at rung %d: %s -- the list is REFUSED",
                e.lat, e.lon, w.rung, why);
            return false;
        }
    }
    for (size_t i = 0; i < got.size(); ++i) {
        const FaceWindow& w = got[i].win;
        blocks.push_back(w);
        const double g0 = Block(i).GroundRes(0);
        Log("[surface] standing blocks: %.5f N %.5f E at rung %d -> slice %zu = face %u "
            "block (%lld,%lld) of the pyramid, %.4g m a texel at its mip 0; the point stands %.0f "
            "texels (%.2f km nominal) inside its nearest edge",
            got[i].lat, got[i].lon, w.rung, 6 + i, w.face, w.anchorX / Lattice::kFaceDim,
            w.anchorY / Lattice::kFaceDim, g0, got[i].inside, got[i].inside * g0 / 1000.0);
    }
    return true;
}

void SurfaceFrame::BlockRows(const FaceWindow& block, const Placement& own, const double eye[3],
                             FaceWindow::Planes& rows, float off[2]) {
    double n[3], a[3], b[3];
    CubeFaceAxes(block.face, n, a, b);
    const bool facing = eye[0] * n[0] + eye[1] * n[1] + eye[2] * n[2] > 0.0;
    // A face the eye stands behind is anchored on the multiple of 16384 at or below the origin
    // (a block's own origin; a window's block), so the address stays the modulo one either way.
    FaceWindow below = block;
    below.anchorX -= block.anchorX % Lattice::kFaceDim;
    below.anchorY -= block.anchorY % Lattice::kFaceDim;
    const FaceWindow anchored = facing ? block.Nearest(eye) : below;
    rows = anchored.PlanesIn(own);
    // In doubles: a window's origin is not a multiple of 16384 (an integer division here truncated
    // it, which the chain's selftest caught), and the quotient is exact in a float (a multiple of
    // 1024 over 16384).
    off[0] = static_cast<float>(double(anchored.anchorX - block.anchorX) / Lattice::kFaceDim);
    off[1] = static_cast<float>(double(anchored.anchorY - block.anchorY) / Lattice::kFaceDim);
}

int SurfaceFrame::RanksAt(const double eye[3], double R, double pixAng, uint32_t* faceOut,
                          double* Lout) {
    const double rE = std::sqrt(eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2]);
    if (!(rE > 0.0)) return 0;
    const double d[3] = {eye[0] / rE, eye[1] / rE, eye[2] / rE};
    double uv[2];
    const uint32_t face = CubeFaceOfDir(d, uv);
    // Rank 1's texel on the ground at the eye's point: rung 3's two steps in metres, their
    // geometric mean.
    const double N3 = std::ldexp(double(Lattice::kFaceDim), 3);
    double d0[3], dx[3], dy[3];
    ComposeCubeDir(face, uv[0], uv[1], d0);
    ComposeCubeDir(face, uv[0] + 1.0 / N3, uv[1], dx);
    ComposeCubeDir(face, uv[0], uv[1] + 1.0 / N3, dy);
    auto ang = [](const double a[3], const double b[3]) {
        const double c[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                             a[0] * b[1] - a[1] * b[0]};
        return std::atan2(std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]),
                          a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
    };
    const double g3 = R * std::sqrt(ang(d0, dx) * ang(d0, dy));
    const double alt = (std::max)(rE - R, 0.01);
    const double L = std::log2(alt * pixAng / g3);
    if (faceOut) *faceOut = face;
    if (Lout) *Lout = L;
    return std::clamp(1 + int(std::ceil(-L / 3.0)), 0, int(kMaxRanks));
}

void SurfaceFrame::Assign(uint32_t n, const double eyes[][3], double pixAng) {
    n = (std::min)(n, kWindowSlots);
    slotsLive = n;
    uint32_t prev[kWindowSlots];
    for (uint32_t s = 0; s < kWindowSlots; ++s) prev[s] = slotSet[s];
    const double half = 0.5 * Lattice::kFaceDim;
    // The score of every (slot, set): the ranks THE SLOT WANTS (its own K, RanksAt) whose box in
    // the set already holds the slot's eye (4.19's held-set law on the box), then the set the slot
    // held, then a set nobody holds. A slot that wants no rank claims nothing by ground (the
    // measured fault without it: the camera's slot at 3,400 km over the Merrimack, K 0, took the
    // set of the Droste level standing at 71 m there, every other frame).
    int score[kWindowSlots][kWindowSlots] = {};
    for (uint32_t s = 0; s < n; ++s) {
        for (int i = 0; i < 3; ++i) slotEye[s][i] = eyes[s][i];
        const double rE = std::sqrt(eyes[s][0] * eyes[s][0] + eyes[s][1] * eyes[s][1] + eyes[s][2] * eyes[s][2]);
        const double d[3] = {eyes[s][0] / rE, eyes[s][1] / rE, eyes[s][2] / rE};
        uint32_t face = 0;
        const uint32_t K = uint32_t(RanksAt(eyes[s], planetR, pixAng, &face));
        for (uint32_t w = 0; w < kWindowSlots; ++w) {
            int holds = 0;
            for (uint32_t k = 0; k < (std::min)(K, bound[w].K); ++k) {
                const hal::BlockBinding& b = bound[w].box[k];
                if (b.face != face) break;
                double X = 0.0, Y = 0.0;
                FaceWindow{face, b.rung, 0, 0}.TexelOf(d, X, Y);
                if (std::abs(X - (double(b.OrgX()) + half)) > half ||
                    std::abs(Y - (double(b.OrgY()) + half)) > half) {
                    break;
                }
                ++holds;
            }
            score[s][w] = 4 * holds + (prev[s] == w ? 2 : 0) + (bound[w].K == 0 ? 1 : 0);
        }
    }
    // The claims, the best pair first (ties: the lower slot, then the lower set).
    bool slotDone[kWindowSlots] = {}, setTaken[kWindowSlots] = {};
    for (uint32_t s = 0; s < kWindowSlots; ++s) slotSet[s] = kNoSet;
    for (uint32_t c = 0; c < n; ++c) {
        int best = -1;
        uint32_t bs = 0, bw = 0;
        for (uint32_t s = 0; s < n; ++s) {
            if (slotDone[s]) continue;
            for (uint32_t w = 0; w < kWindowSlots; ++w) {
                if (!setTaken[w] && score[s][w] > best) {
                    best = score[s][w];
                    bs = s;
                    bw = w;
                }
            }
        }
        slotSet[bs] = bw;
        slotDone[bs] = setTaken[bw] = true;
    }
}

void SurfaceFrame::Follow(uint32_t set, const double eye[3], double pixAng,
                          std::vector<Moved>& moved) {
    if (set >= kWindowSlots) return;
    const uint32_t slot = set;   // the slices are the set's
    EyeWindows& w = bound[set];
    drawn[set] = w;
    if (!eye) {
        w.K = 0;
        return;
    }
    uint32_t face = 0;
    w.K = uint32_t(RanksAt(eye, planetR, pixAng, &face));
    const double rE = std::sqrt(eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2]);
    const double d[3] = {eye[0] / rE, eye[1] / rE, eye[2] / rE};
    // PHASE B2 (D1): the step per axis, a whole tile at the floor of every tenant sharing the windows.
    const double half = 0.5 * Lattice::kFaceDim, stepX = step[0], stepY = step[1];
    for (uint32_t k = 1; k <= w.K; ++k) {
        hal::BlockBinding& b = w.box[k - 1];
        const int rung = int(3 * k);
        double X = 0.0, Y = 0.0;
        FaceWindow{face, rung, 0, 0}.TexelOf(d, X, Y);   // the eye's own address, texels of the rung
        const double top = std::ldexp(double(Lattice::kFaceDim), rung) - Lattice::kFaceDim;
        auto centred = [&](double v, double s) {
            return uint64_t(std::clamp(std::floor((v - half) / s + 0.5) * s, 0.0, top));
        };
        const uint64_t cx = centred(X, stepX), cy = centred(Y, stepY);
        const bool other = b.face != face || b.rung != rung;
        const bool drift = std::abs(X - (double(b.OrgX()) + half)) > stepX ||
                           std::abs(Y - (double(b.OrgY()) + half)) > stepY;
        if (!other && !(drift && (cx != b.OrgX() || cy != b.OrgY()))) continue;
        b = hal::BlockBinding::At(face, rung, cx, cy);
        moved.push_back({WindowSlice(slot, k), b});
    }
}

void SurfaceFrame::ShareWindows(uint32_t texW, uint32_t texH) {
    step[0] = (std::max)(step[0], texW << hal::BlockBinding::kFloorMip);
    step[1] = (std::max)(step[1], texH << hal::BlockBinding::kFloorMip);
}

bool SurfaceFrame::StandAbout(double latDeg, double lonDeg, uint32_t rank) {
    constexpr double kD2R = 3.14159265358979323846 / 180.0;
    const double la = latDeg * kD2R, lo = lonDeg * kD2R;
    const double d[3] = {std::cos(la) * std::cos(lo), std::sin(la), std::cos(la) * std::sin(lo)};
    double uv[2];
    const uint32_t face = CubeFaceOfDir(d, uv);
    const int rung = int(3 * rank);
    double X = 0.0, Y = 0.0;
    FaceWindow{face, rung, 0, 0}.TexelOf(d, X, Y);
    const double half = 0.5 * Lattice::kFaceDim;
    const double top = std::ldexp(double(Lattice::kFaceDim), rung) - Lattice::kFaceDim;
    auto centred = [&](double v, double s) {
        return uint64_t(std::clamp(std::floor((v - half) / s + 0.5) * s, 0.0, top));
    };
    standing = hal::BlockBinding::At(face, rung, centred(X, step[0]), centred(Y, step[1]));
    standingRank = rank;
    standingCentre[0] = d[0] * planetR;
    standingCentre[1] = d[1] * planetR;
    standingCentre[2] = d[2] * planetR;
    Log("[surface] the standing window: rank %u (rung %d) on face %u, origin (%llu, %llu), about "
        "%.5f N %.5f E -- slice %u of every tenant, %.0f texels of the rung from its nearest edge",
        rank, rung, face, static_cast<unsigned long long>(standing.OrgX()),
        static_cast<unsigned long long>(standing.OrgY()), latDeg, lonDeg, kStandingSlice,
        (std::min)((std::min)(X - double(standing.OrgX()), double(standing.OrgX()) + 2 * half - X),
                   (std::min)(Y - double(standing.OrgY()), double(standing.OrgY()) + 2 * half - Y)));
    return true;
}

SurfaceFrame::ChainRows SurfaceFrame::SlotRows(uint32_t slot) const {
    if (slot >= slotsLive || slotSet[slot] == kNoSet) return ChainRows{};
    const uint32_t set = slotSet[slot];
    return RowsOf(bound[set], set, Placement::Frame(east, up, north, slotEye[slot]), slotEye[slot],
                  slice[set]);
}

void SurfaceFrame::Share() {
    bool claimed[kWindowSlots] = {};
    for (uint32_t s = 0; s < slotsLive; ++s) {
        if (slotSet[s] != kNoSet) claimed[slotSet[s]] = true;
    }
    sharedRanks = 0;
    for (uint32_t w = 0; w < kWindowSlots; ++w) {
        for (uint32_t k = 0; k < kMaxRanks; ++k) {
            uint32_t owner = w;
            if (claimed[w] && k < bound[w].K) {
                for (uint32_t v = 0; v < w; ++v) {   // the lowest claimed set at the same address
                    if (claimed[v] && k < bound[v].K && bound[v].box[k] == bound[w].box[k]) {
                        owner = v;
                        break;
                    }
                }
            }
            slice[w][k] = WindowSlice(owner, k + 1);
            if (owner != w) ++sharedRanks;
        }
    }
}

uint32_t SurfaceFrame::SlotNear(const double p[3], double reachM) const {
    uint32_t best = UINT32_MAX;
    double bd = reachM;
    for (uint32_t s = 0; s < slotsLive; ++s) {
        const double d = std::sqrt((p[0] - slotEye[s][0]) * (p[0] - slotEye[s][0]) +
                                   (p[1] - slotEye[s][1]) * (p[1] - slotEye[s][1]) +
                                   (p[2] - slotEye[s][2]) * (p[2] - slotEye[s][2]));
        if (d < bd) {
            bd = d;
            best = s;
        }
    }
    return best;
}

void SurfaceFrame::WindowBlocks(std::vector<hal::BlockSlice>& out, const TileProviderFn& provider,
                                const char* astField) const {
    for (uint32_t s = 0; s < kWindowSlots; ++s) {
        for (uint32_t k = 1; k <= kMaxRanks; ++k) {
            out.push_back({WindowSlice(s, k), hal::BlockBinding{0u, int(3 * k), 0u, 0u}, provider, astField});
        }
    }
    // The standing window where it stands, or (no solver) rank 2's first block, never wanted.
    out.push_back({kStandingSlice, standingRank ? standing : hal::BlockBinding{0u, 6, 0u, 0u}, provider,
                   astField});
}

Placement SurfaceFrame::StandingFrame() const {
    const double r = std::sqrt(standingCentre[0] * standingCentre[0] + standingCentre[1] * standingCentre[1] +
                               standingCentre[2] * standingCentre[2]);
    const double u[3] = {standingCentre[0] / r, standingCentre[1] / r, standingCentre[2] / r};
    const double rho = std::sqrt(u[0] * u[0] + u[2] * u[2]);
    const double e[3] = {-u[2] / rho, 0.0, u[0] / rho};
    const double n[3] = {-u[1] * u[0] / rho, rho, -u[1] * u[2] / rho};
    return Placement::Frame(e, u, n, standingCentre);
}

SurfaceFrame::ChainRows SurfaceFrame::StandingRows(
    const Placement& own, const double origin[3]) const {
    ChainRows r;
    if (!standingRank) return r;
    r.K = 1;
    const FaceWindow box{standing.face, standing.rung, static_cast<long long>(standing.OrgX()),
                         static_cast<long long>(standing.OrgY())};
    BlockRows(box, own, origin, r.pl[0], r.off[0]);
    r.slice[0] = kStandingSlice;
    r.ground[0] = standing.GroundRes(0);
    r.rank0 = standingRank - 1;
    return r;
}

int SurfaceFrame::SliceRects(const hal::BlockBinding& b, double x0, double y0, double x1, double y1,
                             float out[4][4]) {
    const double dim = Lattice::kFaceDim;
    const double ox = double(b.OrgX()), oy = double(b.OrgY());
    // Clip to the box, in texels from its origin.
    const double c0[2] = {(std::max)(x0 - ox, 0.0), (std::max)(y0 - oy, 0.0)};
    const double c1[2] = {(std::min)(x1 - ox, dim), (std::min)(y1 - oy, dim)};
    if (c1[0] <= c0[0] || c1[1] <= c0[1]) return 0;
    double lo[2][2], hi[2][2];
    int pieces[2];
    const double org[2] = {std::fmod(ox, dim), std::fmod(oy, dim)};
    for (int ax = 0; ax < 2; ++ax) {
        const double a = org[ax] + c0[ax];
        const double s0 = a >= dim ? a - dim : a, s1 = s0 + (c1[ax] - c0[ax]);
        pieces[ax] = s1 > dim ? 2 : 1;
        lo[ax][0] = s0 / dim;
        hi[ax][0] = (std::min)(s1, dim) / dim;
        lo[ax][1] = 0.0;
        hi[ax][1] = (s1 - dim) / dim;
    }
    int n = 0;
    for (int py = 0; py < pieces[1]; ++py) {
        for (int px = 0; px < pieces[0]; ++px) {
            out[n][0] = float(lo[0][px]);
            out[n][1] = float(lo[1][py]);
            out[n][2] = float(hi[0][px]);
            out[n][3] = float(hi[1][py]);
            ++n;
        }
    }
    return n;
}

int SurfaceFrame::SliceRectsAbout(const hal::BlockBinding& b, const double dir[3], double halfM,
                                  double planetR, float out[4][4]) {
    // The ground's texel at the rung, and the texel's size there (the rung's ground at the face's
    // middle is 2^-rung of the cube's; a texel shrinks toward a face's edge, so this over-reaches).
    const FaceWindow w{b.face, b.rung, 0, 0};
    double n[3], a[3], c[3];
    CubeFaceAxes(b.face, n, a, c);
    if (dir[0] * n[0] + dir[1] * n[1] + dir[2] * n[2] <= 1e-6) return 0;
    double X = 0.0, Y = 0.0;
    w.TexelOf(dir, X, Y);
    const double texel = 0.25 * 3.14159265358979 * planetR * 2.0 / std::ldexp(double(Lattice::kFaceDim), b.rung);
    const double r = halfM / (0.5 * texel);   // generous: half the nominal texel
    return SliceRects(b, X - r, Y - r, X + r, Y + r, out);
}

int SurfaceFrame::SliceRectsLL(const hal::BlockBinding& b, double lat0, double lon0, double lat1,
                               double lon1, float out[4][4]) {
    constexpr double kD2R = 3.14159265358979323846 / 180.0;
    const FaceWindow w{b.face, b.rung, 0, 0};
    double n[3], a[3], c[3];
    CubeFaceAxes(b.face, n, a, c);
    double xmin = 1e300, ymin = 1e300, xmax = -1e300, ymax = -1e300;
    for (int j = 0; j < 3; ++j) {
        for (int i = 0; i < 3; ++i) {
            const double la = (lat0 + (lat1 - lat0) * 0.5 * j) * kD2R, lo = (lon0 + (lon1 - lon0) * 0.5 * i) * kD2R;
            const double d[3] = {std::cos(la) * std::cos(lo), std::sin(la), std::cos(la) * std::sin(lo)};
            if (d[0] * n[0] + d[1] * n[1] + d[2] * n[2] <= 1e-6) return 0;
            double X = 0.0, Y = 0.0;
            w.TexelOf(d, X, Y);
            xmin = (std::min)(xmin, X);
            ymin = (std::min)(ymin, Y);
            xmax = (std::max)(xmax, X);
            ymax = (std::max)(ymax, Y);
        }
    }
    return SliceRects(b, xmin, ymin, xmax, ymax, out);
}

SurfaceFrame::ChainRows SurfaceFrame::RowsOf(const EyeWindows& w, uint32_t slot,
                                             const Placement& own, const double eye[3],
                                             const uint32_t* slices) {
    ChainRows r;
    r.K = (std::min)(w.K, kMaxRanks);
    for (uint32_t i = 0; i < r.K; ++i) {
        const hal::BlockBinding& b = w.box[i];
        const FaceWindow box{b.face, b.rung, static_cast<long long>(b.OrgX()),
                             static_cast<long long>(b.OrgY())};
        BlockRows(box, own, eye, r.pl[i], r.off[i]);
        r.slice[i] = slices ? slices[i] : WindowSlice(slot, i + 1);
        r.ground[i] = b.GroundRes(0);
    }
    return r;
}

uint32_t SurfaceFrame::Chain(const float p[3], const ChainRows& rows, WalkStep out[kMaxRanks]) {
    uint32_t n = 0;
    bool in = true;
    for (uint32_t k = 0; k < kMaxRanks; ++k) {
        float x = 0.0f, y = 0.0f;
        FaceWindow::PageTexel(p, rows.pl[k], x, y);
        const float u = x / 16384.0f, v = y / 16384.0f;
        const float bu = u + rows.off[k][0], bv = v + rows.off[k][1];
        in = in && k < rows.K && bu >= 0.0f && bv >= 0.0f && bu < 1.0f && bv < 1.0f;
        out[k] = {rows.slice[k], double(u), double(v)};
        if (in) n = k + 1;
    }
    return n;
}

void SurfaceFrame::KernelRows(const ChainRows& rows, KernelWindowRows& out) {
    out = KernelWindowRows{};
    for (uint32_t k = 0; k < kMaxRanks; ++k) {
        const bool on = k < rows.K;
        for (int c = 0; c < 4; ++c) {
            out.u[4 * k + c] = on ? rows.pl[k].u[c] : 0.0f;
            out.v[4 * k + c] = on ? rows.pl[k].v[c] : 0.0f;
            out.w[4 * k + c] = on ? rows.pl[k].w[c] : 0.0f;
        }
        out.o[2 * k] = on ? rows.off[k][0] : 0.0f;
        out.o[2 * k + 1] = on ? rows.off[k][1] : 0.0f;
        out.s[k] = on ? rows.slice[k] : 0u;
    }
    out.s[5] = rows.K;
    out.s[6] = rows.rank0;   // PHASE B2: the rank of the chain's first entry, less one
}

hal::BlockBinding SurfaceFrame::Block(size_t i) const {
    const FaceWindow& w = blocks[i];
    return hal::BlockBinding{w.face, w.rung, uint32_t(w.anchorX / Lattice::kFaceDim),
                             uint32_t(w.anchorY / Lattice::kFaceDim)};
}

SurfaceFrame SurfaceFrame::About(double planetR, bool stencil, double latDeg, double lonDeg) {
    SurfaceFrame s;
    s.planetR = planetR;
    s.stencil = stencil;
    // PHASE C4: the world.flat chart is the tangent plane about the SCENE's place (place.anchor),
    // exact: a ratio of planes in doubles, no metres-per-degree.
    s.flat = Space::Anchor::About(latDeg, lonDeg, planetR);
    // The 16k quad-sphere in the two tile shapes.
    s.cube = Lattice::Cube(Lattice::kFaceDim);
    s.cubeH = Lattice::Cube(Lattice::kFaceDim, 256, 128);
    return s;
}

void SurfaceFrame::Declare(const hal::Tenant& color, const hal::Tenant& height,
                           const hal::Tenant& mask) {
    colorT = color.Id();
    hgtT = height.Id();
    hgtWindows = height.Valid() && !height.Desc().blocks.empty();
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
    // PHASE B3: the pages are the eye's windows alone -- the Mercator pages, their rows (merc, det,
    // u4, the anchor's eyeA/E/N/U and eyePx) and their slices' lanes are deleted.
    const int colorCube = colorT, heightCube = hgtT, maskPages = maskT;
    const bool cubeOn = colorCube >= 0;
    const bool hgtOn = heightCube >= 0;
    // M9ap: the pages path. One tenant; the cube views cover slices 0..5, the array view the
    // windows. PHASE A1: the colour is pages always.
    const bool pages = cubeOn;
    cb.u5[0] = pages ? rm.TextureSrv(colorCube) : UINT32_MAX;
    cb.u5[1] = pages ? rm.ResidencySrv(colorCube) : UINT32_MAX;
    cb.u5[2] = cb.u5[3] = UINT32_MAX;
    cb.u[0] = cubeOn ? rm.TextureSrvCube(colorCube) : UINT32_MAX;
    cb.u[1] = cubeOn ? rm.ResidencySrvCube(colorCube) : UINT32_MAX;
    cb.u[2] = cb.u[3] = UINT32_MAX;
    // M9aq / PHASE B2: the height's pages are the eye's windows, the colour's slices; a height cube
    // alone (Mars's MOLA, an AddTextureCube) has no array view.
    const bool hpages = hgtOn && hgtWindows;
    cb.u6[0] = hpages ? rm.TextureSrv(heightCube) : UINT32_MAX;
    cb.u6[1] = hpages ? rm.ResidencySrv(heightCube) : UINT32_MAX;
    cb.u6[2] = cb.u6[3] = UINT32_MAX;
    cb.u2[0] = hgtOn ? (hpages ? rm.TextureSrvCube(heightCube) : rm.TextureSrv(heightCube))
                     : UINT32_MAX;
    cb.u2[1] = hgtOn ? (hpages ? rm.ResidencySrvCube(heightCube) : rm.ResidencySrv(heightCube))
                     : UINT32_MAX;
    cb.u2[2] = cb.u2[3] = UINT32_MAX;
    // M9ay: the survey MASK PAGES (gis.landsea's tree as a page tenant): array SRV + residency
    // for the windows, cube views for the faces. r = water coverage, b = edited, a = surveyed.
    // UINT32_MAX = no survey: the classifier uses the height sign.
    const bool maskOn = maskPages >= 0;
    cb.u3[0] = maskOn ? rm.TextureSrv(maskPages) : UINT32_MAX;
    cb.u3[1] = maskOn ? rm.ResidencySrv(maskPages) : UINT32_MAX;
    cb.u3[2] = maskOn ? rm.TextureSrvCube(maskPages) : UINT32_MAX;
    cb.u3[3] = maskOn ? rm.ResidencySrvCube(maskPages) : UINT32_MAX;
    cb.f[0] = cubeOn ? 1.0f : 0.0f;
    cb.f[1] = 0.0f;   // the old window: none
    cb.f[2] = hgtOn ? 1.0f : 0.0f;
    cb.f[3] = static_cast<float>(planetR);
    // R16F pyramids stop at the one-tile-ish mip (16384 -> 256 = 7 levels, max lod 6); one
    // cube texel spans (pi/2)/16384 radians of arc along a face's midline.
    cb.g[0] = 6.0f;
    cb.g[1] = static_cast<float>(3.14159265358979 * 0.5 / Compositor::kFaceDim);
    cb.g[2] = 6.0f;
    cb.g[3] = stencil ? 1.0f : 0.0f;
    for (int i = 0; i < 3; ++i) {
        cb.r0[i] = static_cast<float>(east[i]);
        cb.r1[i] = static_cast<float>(up[i]);
        cb.r2[i] = static_cast<float>(north[i]);
    }
    cb.r0[3] = cb.r1[3] = cb.r2[3] = 0.0f;
    // M12 step 4f: THE GROUND ROW, the cube's mip-0 ground from its lattice (611.496..).
    cb.ground[0] = static_cast<float>(cube.GroundRes(0));
    cb.ground[1] = cb.ground[2] = cb.ground[3] = 0.0f;
    // THE CAMERA'S EYE in the tangent axes less (0, R, 0), from doubles: CsPointOfDir's.
    const double* const axes[3] = {east, up, north};
    for (int k = 0; k < 3; ++k) {
        cb.eyeT[k] = static_cast<float>(axes[k][0] * eye[0] + axes[k][1] * eye[1] + axes[k][2] * eye[2]);
    }
    cb.eyeT[1] = static_cast<float>(up[0] * eye[0] + up[1] * eye[1] + up[2] * eye[2] - planetR);
    cb.eyeT[3] = 0.0f;
    // PHASE A2: THE EYE'S WINDOWS, PER LEVEL. Slot s's rows are the same rows -- RowsOf, the one
    // function -- taken about THAT slot's own eye (its tangent frame's axes, the root's, origin the
    // eye Follow took): the frame its mesh records' geo is relative to (GlobeLayer's records are
    // anchored at each world's own eye). The windows drawn are the step's frame before (Follow).
    for (uint32_t s = 0; s < kWindowSlots; ++s) {
        // Slot s reads the set it claimed (Assign), drawn as the frame before left it.
        const uint32_t set = s < slotsLive ? slotSet[s] : kNoSet;
        const Placement ownS = Placement::Frame(east, up, north, slotEye[s]);
        const ChainRows rows = (pages && set != kNoSet) ? RowsOf(drawn[set], set, ownS, slotEye[s]) : ChainRows{};
        for (uint32_t k = 0; k < kMaxRanks; ++k) {
            const uint32_t j = s * kMaxRanks + k;
            const bool on = k < rows.K;
            for (int c = 0; c < 4; ++c) {
                cb.winU[4 * j + c] = on ? rows.pl[k].u[c] : 0.0f;
                cb.winV[4 * j + c] = on ? rows.pl[k].v[c] : 0.0f;
                cb.winW[4 * j + c] = on ? rows.pl[k].w[c] : 0.0f;
            }
            cb.winO[2 * j] = on ? rows.off[k][0] : 0.0f;
            cb.winO[2 * j + 1] = on ? rows.off[k][1] : 0.0f;
            cb.winS[j] = on ? rows.slice[k] : 0u;
        }
        cb.winK[s] = rows.K;
    }
    for (uint32_t k = 0; k < 8; ++k) {
        cb.rankG[k] = k < kMaxRanks ? static_cast<float>(hal::BlockBinding{0u, int(3 * (k + 1)), 0u, 0u}.GroundRes(0))
                                    : 0.0f;
    }
}

}  // namespace ga
