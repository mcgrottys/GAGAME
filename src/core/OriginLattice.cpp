// OriginLattice - the dyadic icosahedral lattice of floating origins (OriginLattice.h).
#include "core/OriginLattice.h"

#include <algorithm>

namespace ga {

namespace {
constexpr double kPi = 3.14159265358979323846;
double Dot(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// THE ICOSAHEDRON, the polar axis on y as the planet frame has it: the twelve vertices
// (0, +-1, +-phi) and their cyclic permutations, unit; the twenty faces counter-clockwise seen
// from outside (the standard icosphere table; the self-test checks every face outward).
struct Ico {
    double v[12][3];
    int f[20][3];
    double n[20][3];      // unit face centres
    double inv[20][3][3]; // the inverse of the face's corner matrix: gnomonic barycentrics
    int ownerV[12];       // the lowest face holding a vertex
    int ownerE[12][12];   // the lowest face holding an edge (by its two vertices), -1 if none
    Ico() {
        const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
        const double raw[12][3] = {{-1, phi, 0}, {1, phi, 0}, {-1, -phi, 0}, {1, -phi, 0},
                                   {0, -1, phi}, {0, 1, phi}, {0, -1, -phi}, {0, 1, -phi},
                                   {phi, 0, -1}, {phi, 0, 1}, {-phi, 0, -1}, {-phi, 0, 1}};
        for (int i = 0; i < 12; ++i) {
            const double l = std::sqrt(raw[i][0] * raw[i][0] + raw[i][1] * raw[i][1] + raw[i][2] * raw[i][2]);
            for (int k = 0; k < 3; ++k) v[i][k] = raw[i][k] / l;
        }
        const int faces[20][3] = {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
                                  {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
                                  {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
                                  {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
        for (int i = 0; i < 12; ++i) ownerV[i] = -1;
        for (int a = 0; a < 12; ++a)
            for (int b = 0; b < 12; ++b) ownerE[a][b] = -1;
        for (int fi = 0; fi < 20; ++fi) {
            for (int k = 0; k < 3; ++k) f[fi][k] = faces[fi][k];
            const double *A = v[f[fi][0]], *B = v[f[fi][1]], *C = v[f[fi][2]];
            double c[3] = {A[0] + B[0] + C[0], A[1] + B[1] + C[1], A[2] + B[2] + C[2]};
            const double l = std::sqrt(Dot(c, c));
            for (int k = 0; k < 3; ++k) n[fi][k] = c[k] / l;
            // The corner matrix M = [A B C] (columns), inverted: w = M^-1 d gives d = wA A + wB B + wC C.
            const double m[3][3] = {{A[0], B[0], C[0]}, {A[1], B[1], C[1]}, {A[2], B[2], C[2]}};
            const double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
            inv[fi][0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det;
            inv[fi][0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det;
            inv[fi][0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det;
            inv[fi][1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det;
            inv[fi][1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det;
            inv[fi][1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det;
            inv[fi][2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det;
            inv[fi][2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det;
            inv[fi][2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det;
            for (int k = 0; k < 3; ++k) {
                const int a = f[fi][k], b = f[fi][(k + 1) % 3];
                if (ownerV[a] < 0) ownerV[a] = fi;
                if (ownerE[a][b] < 0) ownerE[a][b] = ownerE[b][a] = fi;
            }
        }
    }
};
const Ico& Icosa() {
    static const Ico ico;
    return ico;
}

constexpr int64_t kNmax = int64_t(1) << OriginLattice::kLmax;
// The index packs (face: 5 bits, i: 26 bits, j: 26 bits) -- i, j in [0, 2^24].
int64_t Pack(int face, int64_t i, int64_t j) { return (int64_t(face) << 52) | (i << 26) | j; }
void Unpack(int64_t index, int& face, int64_t& i, int64_t& j) {
    face = int(index >> 52);
    i = (index >> 26) & ((int64_t(1) << 26) - 1);
    j = index & ((int64_t(1) << 26) - 1);
}

// The canonical form of a finest-level point (f, i, j): a corner is owned by the lowest face
// holding its vertex, an edge point by the lowest face holding its edge, each with its own
// coordinates there; an interior point is its own.
int64_t Canonical(int face, int64_t i, int64_t j) {
    const Ico& ico = Icosa();
    const int64_t w[3] = {kNmax - i - j, i, j};   // the weights of corners A, B, C
    int zeros = 0;
    for (int k = 0; k < 3; ++k) zeros += (w[k] == 0);
    if (zeros == 0) return Pack(face, i, j);
    if (zeros == 2) {   // a corner
        int k = 0;
        while (w[k] == 0) ++k;
        const int vert = ico.f[face][k];
        const int owner = ico.ownerV[vert];
        int64_t oi = 0, oj = 0;
        for (int m = 0; m < 3; ++m) {
            if (ico.f[owner][m] != vert) continue;
            if (m == 1) oi = kNmax;
            if (m == 2) oj = kNmax;
        }
        return Pack(owner, oi, oj);
    }
    // An edge: the two corners with weight, the owner face, the weights placed on its corners.
    int ka = -1, kb = -1;
    for (int k = 0; k < 3; ++k) {
        if (w[k] == 0) continue;
        if (ka < 0) ka = k; else kb = k;
    }
    const int va = ico.f[face][ka], vb = ico.f[face][kb];
    const int owner = ico.ownerE[va][vb];
    int64_t ow[3] = {0, 0, 0};
    for (int m = 0; m < 3; ++m) {
        if (ico.f[owner][m] == va) ow[m] = w[ka];
        if (ico.f[owner][m] == vb) ow[m] = w[kb];
    }
    return Pack(owner, ow[1], ow[2]);
}
}  // namespace

int64_t OriginLattice::Encode(int level, int face, int64_t i, int64_t j) {
    const int64_t s = int64_t(1) << (kLmax - level);
    return Canonical(face, i * s, j * s);
}
void OriginLattice::Decode(int64_t index, int& face, int64_t& i, int64_t& j) { Unpack(index, face, i, j); }

void OriginLattice::Direction(int64_t index, double d[3]) {
    const Ico& ico = Icosa();
    int face;
    int64_t i, j;
    Unpack(index, face, i, j);
    // The weights as exact binary fractions of 2^kLmax: a level-L point's doubled coordinates give
    // the same weights to the bit, so a coarse point and its fine twin are one direction.
    const double wa = double(kNmax - i - j) / double(kNmax), wb = double(i) / double(kNmax), wc = double(j) / double(kNmax);
    const double *A = ico.v[ico.f[face][0]], *B = ico.v[ico.f[face][1]], *C = ico.v[ico.f[face][2]];
    double p[3];
    for (int k = 0; k < 3; ++k) p[k] = wa * A[k] + wb * B[k] + wc * C[k];
    const double l = std::sqrt(Dot(p, p));
    for (int k = 0; k < 3; ++k) d[k] = p[k] / l;
}

int64_t OriginLattice::InverseIndex(const double d[3]) const {
    const Ico& ico = Icosa();
    // The face: the one whose centre is nearest the direction.
    int face = 0;
    double best = -2.0;
    for (int f = 0; f < 20; ++f) {
        const double dd = Dot(d, ico.n[f]);
        if (dd > best) {
            best = dd;
            face = f;
        }
    }
    // The gnomonic barycentrics: d = wA A + wB B + wC C up to scale, normalised to sum 1.
    double w[3];
    for (int r = 0; r < 3; ++r) w[r] = ico.inv[face][r][0] * d[0] + ico.inv[face][r][1] * d[1] + ico.inv[face][r][2] * d[2];
    const double sum = w[0] + w[1] + w[2];
    for (double& x : w) x = std::max(0.0, x / sum);
    const int64_t N = int64_t(1) << level;
    const double fi = w[1] * double(N), fj = w[2] * double(N);
    // The cell's corners around (fi, fj), the nearest by angle: i, j in [0, N], i + j <= N.
    int64_t bi = 0, bj = 0;
    double bd = -2.0;
    const int64_t i0 = int64_t(std::floor(fi)), j0 = int64_t(std::floor(fj));
    for (int s = 0; s < 4; ++s) {
        const int64_t i = std::clamp(i0 + (s & 1), int64_t(0), N), j = std::clamp(j0 + (s >> 1), int64_t(0), N);
        if (i + j > N) continue;
        double q[3];
        Direction(Encode(level, face, i, j), q);
        const double dd = Dot(d, q);
        if (dd > bd) {
            bd = dd;
            bi = i;
            bj = j;
        }
    }
    return Encode(level, face, bi, bj);
}

int64_t OriginLattice::IndexOfOrdinal(int64_t ordinal) const {
    const int64_t N = int64_t(1) << level, perFace = (N + 1) * (N + 2) / 2;
    if (ordinal < 0 || ordinal >= 20 * perFace) return -1;
    const int face = int(ordinal / perFace);
    int64_t rem = ordinal % perFace, i = 0;
    while (rem > N - i) {
        rem -= N - i + 1;
        ++i;
    }
    const int64_t j = rem;
    const int64_t idx = Encode(level, face, i, j);
    int cf;
    int64_t ci, cj;
    Unpack(idx, cf, ci, cj);
    const int64_t s = int64_t(1) << (kLmax - level);
    return (cf == face && ci == i * s && cj == j * s) ? idx : -1;
}

OriginLattice::Origin OriginLattice::At(int64_t shell, int64_t index) const {
    Origin o;
    if (index < 0) {
        o.centre = true;
        o.shell = 0;
        o.index = -1;
        o.r = 0.0;
        // The identity frame: east x, up y, north z -- the planet frame's own axes.
        o.anchor.east[0] = 1.0; o.anchor.east[1] = 0.0; o.anchor.east[2] = 0.0;
        o.anchor.up[0] = 0.0; o.anchor.up[1] = 1.0; o.anchor.up[2] = 0.0;
        o.anchor.north[0] = 0.0; o.anchor.north[1] = 0.0; o.anchor.north[2] = 1.0;
        o.anchor.planetR = 0.0;
        return o;
    }
    o.shell = shell;
    o.index = index;
    o.r = RadiusOf(shell);
    double d[3];
    Direction(index, d);
    const double r2d = 180.0 / kPi;
    const double lat = std::asin(std::clamp(d[1], -1.0, 1.0)) * r2d, lon = std::atan2(d[2], d[0]) * r2d;
    o.anchor = Space::Anchor::About(lat, lon, o.r);
    for (int i = 0; i < 3; ++i) o.c[i] = o.anchor.up[i] * o.r;   // the anchor's up IS its direction
    return o;
}

OriginLattice::Origin OriginLattice::Nearest(const double P[3]) const {
    const double r = std::sqrt(Dot(P, P));
    if (r < rMin) return At(0, -1);
    const int64_t k = ShellOf(r);
    if (RadiusOf(k) < rMin) return At(0, -1);
    const double d[3] = {P[0] / r, P[1] / r, P[2] / r};
    return At(k, InverseIndex(d));
}

OriginLattice::Origin OriginLattice::Surface(const double P[3]) const {
    const double r = std::sqrt(Dot(P, P));
    if (!(r > 0.0)) return At(0, -1);
    const double d[3] = {P[0] / r, P[1] / r, P[2] / r};
    return At(0, InverseIndex(d));
}

OriginLattice::Origin OriginLattice::SurfaceAt(double latDeg, double lonDeg) const {
    const double d2r = kPi / 180.0;
    const double la = latDeg * d2r, lo = lonDeg * d2r;
    const double d[3] = {std::cos(la) * std::cos(lo), std::sin(la), std::cos(la) * std::sin(lo)};
    return At(0, InverseIndex(d));
}

OriginLattice::Origin OriginLattice::PickSurface(const double P[3], const Origin* current, double margin) const {
    const Origin best = Surface(P);
    if (!current || current->centre || current->Same(best)) return best;
    const double r = std::sqrt(Dot(P, P));
    const double S[3] = {P[0] / r * R, P[1] / r * R, P[2] / r * R};   // the point brought to the shell
    const double dn[3] = {S[0] - best.c[0], S[1] - best.c[1], S[2] - best.c[2]};
    const double dc[3] = {S[0] - current->c[0], S[1] - current->c[1], S[2] - current->c[2]};
    return std::sqrt(Dot(dc, dc)) <= (1.0 + margin) * std::sqrt(Dot(dn, dn)) ? *current : best;
}

OriginLattice::Origin OriginLattice::Pick(const double P[3], const Origin* current, double margin) const {
    const Origin best = Nearest(P);
    if (!current || current->Same(best)) return best;
    const double dn[3] = {P[0] - best.c[0], P[1] - best.c[1], P[2] - best.c[2]};
    const double dc[3] = {P[0] - current->c[0], P[1] - current->c[1], P[2] - current->c[2]};
    const double ln = std::sqrt(Dot(dn, dn)), lc = std::sqrt(Dot(dc, dc));
    return lc <= (1.0 + margin) * ln ? *current : best;
}

void OriginLattice::Floats(const Origin& o, const double P[3], double out[3]) {
    const double d[3] = {P[0] - o.c[0], P[1] - o.c[1], P[2] - o.c[2]};
    out[0] = Dot(o.anchor.east, d);
    out[1] = Dot(o.anchor.up, d);
    out[2] = Dot(o.anchor.north, d);
}

void OriginLattice::Point(const Origin& o, const double f[3], double P[3]) {
    for (int i = 0; i < 3; ++i) P[i] = o.c[i] + o.anchor.east[i] * f[0] + o.anchor.up[i] * f[1] + o.anchor.north[i] * f[2];
}

Space SpaceAt(const Space& parent, const OriginLattice::Origin& o, const std::string& name) {
    Space s;
    s.name = name;
    s.unitM = parent.unitM;
    s.extentM = parent.extentM;
    s.parent = &parent;
    s.link = o.centre ? Placement::Identity() : Placement::Frame(o.anchor.east, o.anchor.up, o.anchor.north, o.c);
    s.stand.shell = o.shell;
    s.stand.index = o.index;
    s.stand.valid = true;
    return s;
}

Motor OriginLattice::Between(const Origin& from, const Origin& to) {
    // floats_to = R_to^T (R_from floats_from + c_from - c_to), R the frame's columns (east, up,
    // north): a rotation Q = R_to^T R_from and a translation t = R_to^T (c_from - c_to), as one
    // motor T(t) Rot(Q). Q's axis and angle from its quaternion (Shepperd's choice of the largest
    // diagonal keeps it exact for every attitude).
    const double* A[3] = {from.anchor.east, from.anchor.up, from.anchor.north};
    const double* B[3] = {to.anchor.east, to.anchor.up, to.anchor.north};
    double Q[3][3];   // Q[r][c] = B_r . A_c
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) Q[r][c] = Dot(B[r], A[c]);
    const double dc[3] = {from.c[0] - to.c[0], from.c[1] - to.c[1], from.c[2] - to.c[2]};
    const double t[3] = {Dot(B[0], dc), Dot(B[1], dc), Dot(B[2], dc)};
    const double tr = Q[0][0] + Q[1][1] + Q[2][2];
    double x, y, z, w;
    if (tr > 0.0) {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s; x = (Q[2][1] - Q[1][2]) / s; y = (Q[0][2] - Q[2][0]) / s; z = (Q[1][0] - Q[0][1]) / s;
    } else if (Q[0][0] > Q[1][1] && Q[0][0] > Q[2][2]) {
        const double s = std::sqrt(1.0 + Q[0][0] - Q[1][1] - Q[2][2]) * 2.0;
        w = (Q[2][1] - Q[1][2]) / s; x = 0.25 * s; y = (Q[0][1] + Q[1][0]) / s; z = (Q[0][2] + Q[2][0]) / s;
    } else if (Q[1][1] > Q[2][2]) {
        const double s = std::sqrt(1.0 + Q[1][1] - Q[0][0] - Q[2][2]) * 2.0;
        w = (Q[0][2] - Q[2][0]) / s; x = (Q[0][1] + Q[1][0]) / s; y = 0.25 * s; z = (Q[1][2] + Q[2][1]) / s;
    } else {
        const double s = std::sqrt(1.0 + Q[2][2] - Q[0][0] - Q[1][1]) * 2.0;
        w = (Q[1][0] - Q[0][1]) / s; x = (Q[0][2] + Q[2][0]) / s; y = (Q[1][2] + Q[2][1]) / s; z = 0.25 * s;
    }
    const double sh = std::sqrt(x * x + y * y + z * z);
    const double org[3] = {0.0, 0.0, 0.0};
    Motor rot = Motor::Identity();
    if (sh > 1e-15) {
        const double axis[3] = {x / sh, y / sh, z / sh};
        rot = Motor::Rotation(org, axis, 2.0 * std::atan2(sh, w));
    }
    return Motor::Translation(t[0], t[1], t[2]) * rot;
}

}  // namespace ga
