#include "sim/SeaGrid.h"

#include "core/Common.h"
#include "core/FileText.h"
#include "core/Json.h"

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>

namespace ga {

namespace {
constexpr double kD2R = 3.14159265358979323846 / 180.0;
}  // namespace

SeaGrid::~SeaGrid() {
    if (m_data) UnmapViewOfFile(m_data);
    if (m_map) CloseHandle(static_cast<HANDLE>(m_map));
    if (m_file && m_file != INVALID_HANDLE_VALUE) CloseHandle(static_cast<HANDLE>(m_file));
}

bool SeaGrid::Load(const std::string& seastateJson) {
    const std::string text = ReadFileText(seastateJson);
    if (text.empty()) return false;
    std::string err;
    const JsonValue root = JsonParser::Parse(text, &err);
    const JsonValue* g = root.Get("grid");
    if (!err.empty() || !g || g->type != JsonValue::Type::Object) return false;
    if (g->Str("format") != "f16" || g->Str("order") != "hour plane row col") {
        Log("[seagrid] %s: a grid of format '%s', order '%s' -- not read", seastateJson.c_str(), g->Str("format").c_str(),
            g->Str("order").c_str());
        return false;
    }
    m_nx = int(g->Num("nx", 0));
    m_ny = int(g->Num("ny", 0));
    m_lat1 = g->Num("lat1", 90.0);
    m_lon1 = g->Num("lon1", 0.0);
    m_dlat = g->Num("dlat", 0.25);
    m_dlon = g->Num("dlon", 0.25);
    m_cycleUnix = g->Num("cycle_unix", 0.0);
    m_cycleLabel = g->Str("cycle_label", "gfswave ?");
    if (const JsonValue* h = g->Get("hours"); h && h->type == JsonValue::Type::Array) {
        for (const JsonValue& v : h->arr) m_fh.push_back(v.number);
    }
    if (const JsonValue* p = g->Get("planes"); p && p->type == JsonValue::Type::Array) {
        for (const JsonValue& v : p->arr) m_planes.push_back(v.str);
    }
    m_nodes = size_t(m_nx) * size_t(m_ny);
    if (m_nx < 2 || m_ny < 2 || m_fh.empty() || m_planes.empty()) return false;
    m_pHs = PlaneOf("HTSGW");
    m_pTp = PlaneOf("PERPW");
    m_pDir = PlaneOf("DIRPW");
    m_pWsHs = PlaneOf("WVHGT");
    m_pWsTp = PlaneOf("WVPER");
    m_pWsDir = PlaneOf("WVDIR");
    m_pWind = PlaneOf("WIND");
    m_pWdir = PlaneOf("WDIR");
    for (int k = 0; k < 3; ++k) {
        const std::string n = std::to_string(k + 1);
        m_pSwHs[k] = PlaneOf(("SWELL_" + n).c_str());
        m_pSwTp[k] = PlaneOf(("SWPER_" + n).c_str());
        m_pSwDir[k] = PlaneOf(("SWDIR_" + n).c_str());
    }
    if (m_pHs < 0) return false;

    // The file beside the json, mapped whole.
    std::string dir = seastateJson;
    const size_t slash = dir.find_last_of("/\\");
    dir = (slash == std::string::npos) ? std::string() : dir.substr(0, slash + 1);
    const std::string path = dir + g->Str("file", "grid.f16");
    const uint64_t want = uint64_t(m_fh.size()) * m_planes.size() * m_nodes * 2u;
    HANDLE f = CreateFileA(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL | FILE_FLAG_RANDOM_ACCESS, nullptr);
    if (f == INVALID_HANDLE_VALUE) {
        Log("[seagrid] %s: cannot open", path.c_str());
        return false;
    }
    LARGE_INTEGER sz{};
    if (!GetFileSizeEx(f, &sz) || uint64_t(sz.QuadPart) < want) {
        Log("[seagrid] %s: %lld bytes, the header wants %llu -- not read", path.c_str(), static_cast<long long>(sz.QuadPart),
            static_cast<unsigned long long>(want));
        CloseHandle(f);
        return false;
    }
    HANDLE m = CreateFileMappingA(f, nullptr, PAGE_READONLY, 0, 0, nullptr);
    const void* v = m ? MapViewOfFile(m, FILE_MAP_READ, 0, 0, 0) : nullptr;
    if (!v) {
        if (m) CloseHandle(m);
        CloseHandle(f);
        Log("[seagrid] %s: cannot map", path.c_str());
        return false;
    }
    m_file = f;
    m_map = m;
    m_data = static_cast<const uint16_t*>(v);

    // B2: every node's reading node, from the model's mask at the first hour.
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<uint8_t> wetMask(m_nodes, 0);
    size_t wet = 0, filled = 0;
    for (size_t n = 0; n < m_nodes; ++n) {
        if (std::isfinite(Val(0, m_pHs, int(n)))) {
            wetMask[n] = 1;
            ++wet;
        }
    }
    m_src = NearestWet(wetMask, m_nx, m_ny, kReachNodes, &filled);
    Log("[seagrid] %s: %d x %d nodes, %zu hours x %zu planes, %zu with sea; %zu more read their nearest sea "
        "within %d nodes (%.0f ms)",
        m_cycleLabel.c_str(), m_nx, m_ny, m_fh.size(), m_planes.size(), wet, filled, kReachNodes,
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    return true;
}

std::vector<int32_t> SeaGrid::NearestWet(const std::vector<uint8_t>& wet, int nx, int ny, int reach, size_t* filled) {
    const size_t nodes = size_t(nx) * size_t(ny);
    std::vector<int32_t> src(nodes, -1);
    std::vector<uint8_t> depth(nodes, 255);
    std::deque<int32_t> q;
    for (size_t n = 0; n < nodes && n < wet.size(); ++n) {
        if (!wet[n]) continue;
        src[n] = int32_t(n);
        depth[n] = 0;
        q.push_back(int32_t(n));
    }
    size_t f = 0;
    while (!q.empty()) {
        const int32_t n = q.front();
        q.pop_front();
        if (depth[size_t(n)] >= reach) continue;
        const int r = n / nx, c = n % nx;
        const int nb[4][2] = {{r - 1, c}, {r + 1, c}, {r, (c + nx - 1) % nx}, {r, (c + 1) % nx}};
        for (const auto& rc : nb) {
            if (rc[0] < 0 || rc[0] >= ny) continue;
            const size_t k = size_t(rc[0]) * size_t(nx) + size_t(rc[1]);
            if (src[k] >= 0) continue;
            src[k] = src[size_t(n)];
            depth[k] = uint8_t(depth[size_t(n)] + 1);
            q.push_back(int32_t(k));
            ++f;
        }
    }
    if (filled) *filled = f;
    return src;
}

int SeaGrid::PlaneOf(const char* name) const {
    for (size_t i = 0; i < m_planes.size(); ++i) {
        if (m_planes[i] == name) return int(i);
    }
    return -1;
}

float SeaGrid::Val(int hour, int plane, int node) const {
    if (plane < 0) return std::numeric_limits<float>::quiet_NaN();
    const size_t at = (size_t(hour) * m_planes.size() + size_t(plane)) * m_nodes + size_t(node);
    return HalfToFloat(m_data[at]);
}

int SeaGrid::HourIndex(double simUnix) const {
    const double fh = (simUnix - m_cycleUnix) / 3600.0;
    int best = 0;
    double bestD = 1e30;
    for (size_t i = 0; i < m_fh.size(); ++i) {
        const double d = std::abs(m_fh[i] - fh);
        if (d < bestD) {
            bestD = d;
            best = int(i);
        }
    }
    return best;
}

void SeaGrid::FillPlane(int hour, int plane, std::vector<float>& out) const {
    out.assign(m_nodes, -1.0f);
    if (!Ready() || plane < 0) return;
    for (size_t n = 0; n < m_nodes; ++n) {
        if (m_src[n] < 0) continue;
        const float v = Val(hour, plane, m_src[n]);
        if (std::isfinite(v) && v >= 0.0f) out[n] = v;
    }
}

void SeaGrid::HsPlane(int hour, std::vector<float>& out) const { FillPlane(hour, m_pHs, out); }
void SeaGrid::WindPlane(int hour, std::vector<float>& out) const { FillPlane(hour, m_pWind, out); }

bool SeaGrid::HourAt(double latDeg, double lonDeg, int hour, SeaHour& out) const {
    out = SeaHour{};
    if (!Ready()) return false;
    double fx = (lonDeg - m_lon1) / m_dlon;
    fx -= std::floor(fx / double(m_nx)) * double(m_nx);
    const double fy = std::clamp((m_lat1 - latDeg) / m_dlat, 0.0, double(m_ny - 1));
    const int i0 = (std::min)(int(fx), m_nx - 1), j0 = (std::min)(int(fy), m_ny - 2);
    const int i1 = (i0 + 1) % m_nx, j1 = j0 + 1;
    const double tx = fx - double(i0), ty = fy - double(j0);
    const int node[4] = {j0 * m_nx + i0, j0 * m_nx + i1, j1 * m_nx + i0, j1 * m_nx + i1};
    const double wb[4] = {(1 - tx) * (1 - ty), tx * (1 - ty), (1 - tx) * ty, tx * ty};
    int src[4];
    double wsum = 0.0;
    for (int k = 0; k < 4; ++k) {
        src[k] = m_src[size_t(node[k])];
        if (src[k] >= 0 && std::isfinite(Val(hour, m_pHs, src[k]))) wsum += wb[k];
        else src[k] = -1;
    }
    if (wsum <= 1e-9) return false;
    // One partition (its Hs, period and from-direction planes), blended by energy.
    const auto blend = [&](int pHs, int pTp, int pDir, double& hs, double& tp, double& fromDeg) {
        hs = tp = fromDeg = 0.0;
        if (pHs < 0) return;
        double e = 0.0, et = 0.0, ex = 0.0, ey = 0.0;
        for (int k = 0; k < 4; ++k) {
            if (src[k] < 0) continue;
            const double h = Val(hour, pHs, src[k]);
            if (!std::isfinite(h) || h <= 0.0) continue;
            const double w = wb[k] / wsum * h * h;
            e += w;
            const double t = Val(hour, pTp, src[k]);
            if (std::isfinite(t)) et += w * t;
            const double d = Val(hour, pDir, src[k]);
            if (std::isfinite(d)) {
                ex += w * std::sin(d * kD2R);
                ey += w * std::cos(d * kD2R);
            }
        }
        if (e <= 1e-12) return;
        hs = std::sqrt(e);
        tp = et / e;
        fromDeg = std::atan2(ex, ey) / kD2R;
        if (fromDeg < 0.0) fromDeg += 360.0;
    };
    out.fh = HourFh(hour);
    blend(m_pHs, m_pTp, m_pDir, out.combinedHs, out.combinedTp, out.combinedFromDeg);
    {
        SeaPartition p;
        blend(m_pWsHs, m_pWsTp, m_pWsDir, p.hs, p.tp, p.fromDeg);
        p.windsea = true;
        if (p.hs > 0.01 && p.tp > 0.5) out.parts.push_back(p);
    }
    for (int k = 0; k < 3; ++k) {
        SeaPartition p;
        blend(m_pSwHs[k], m_pSwTp[k], m_pSwDir[k], p.hs, p.tp, p.fromDeg);
        if (p.hs > 0.01 && p.tp > 0.5) out.parts.push_back(p);
    }
    // The wind: its speed a plain bilinear mean, its direction the mean of unit vectors.
    double ws = 0.0, wx = 0.0, wy = 0.0, wn = 0.0;
    for (int k = 0; k < 4; ++k) {
        if (src[k] < 0) continue;
        const double s = Val(hour, m_pWind, src[k]), d = Val(hour, m_pWdir, src[k]);
        if (!std::isfinite(s)) continue;
        const double w = wb[k] / wsum;
        ws += w * s;
        wn += w;
        if (std::isfinite(d)) {
            wx += w * s * std::sin(d * kD2R);
            wy += w * s * std::cos(d * kD2R);
        }
    }
    out.windMs = wn > 0.0 ? ws / wn : 0.0;
    out.windFromDeg = std::atan2(wx, wy) / kD2R;
    if (out.windFromDeg < 0.0) out.windFromDeg += 360.0;
    return true;
}

double SeaGrid::HsBilinear(double latDeg, double lonDeg, int hour) const {
    if (!Ready()) return -1.0;
    double fx = (lonDeg - m_lon1) / m_dlon;
    fx -= std::floor(fx / double(m_nx)) * double(m_nx);
    const double fy = std::clamp((m_lat1 - latDeg) / m_dlat, 0.0, double(m_ny - 1));
    const int i0 = (std::min)(int(fx), m_nx - 1), j0 = (std::min)(int(fy), m_ny - 2);
    const int i1 = (i0 + 1) % m_nx, j1 = j0 + 1;
    const double tx = fx - double(i0), ty = fy - double(j0);
    const int node[4] = {j0 * m_nx + i0, j0 * m_nx + i1, j1 * m_nx + i0, j1 * m_nx + i1};
    const double wb[4] = {(1 - tx) * (1 - ty), tx * (1 - ty), (1 - tx) * ty, tx * ty};
    double s = 0.0, w = 0.0;
    for (int k = 0; k < 4; ++k) {
        const int32_t src = m_src[size_t(node[k])];
        if (src < 0) continue;
        const float h = Val(hour, m_pHs, src);
        if (!std::isfinite(h) || h < 0.0f) continue;
        s += wb[k] * h;
        w += wb[k];
    }
    return w > 1e-9 ? s / w : -1.0;
}

bool SeaGrid::NearestSea(double& latDeg, double& lonDeg) const {
    if (!Ready()) return false;
    double fx = (lonDeg - m_lon1) / m_dlon;
    fx -= std::floor(fx / double(m_nx)) * double(m_nx);
    const int ci = int(std::lround(fx)) % m_nx;
    const int cj = std::clamp(int(std::lround((m_lat1 - latDeg) / m_dlat)), 0, m_ny - 1);
    // Rings of nodes outward from the place's own, the first holding a node with sea.
    for (int r = 0; r < (std::max)(m_nx, m_ny); ++r) {
        double bestD = 1e30;
        int best = -1;
        for (int dj = -r; dj <= r; ++dj) {
            for (int di = -r; di <= r; ++di) {
                if ((std::max)(std::abs(di), std::abs(dj)) != r) continue;
                const int j = cj + dj;
                if (j < 0 || j >= m_ny) continue;
                const int i = ((ci + di) % m_nx + m_nx) % m_nx;
                const int n = j * m_nx + i;
                if (m_src[size_t(n)] != n) continue;   // a node with sea of its own
                const double d = double(di) * di + double(dj) * dj;
                if (d < bestD) {
                    bestD = d;
                    best = n;
                }
            }
        }
        if (best >= 0) {
            latDeg = m_lat1 - double(best / m_nx) * m_dlat;
            const double lon = m_lon1 + double(best % m_nx) * m_dlon;
            lonDeg = lon > 180.0 ? lon - 360.0 : lon;
            return true;
        }
    }
    return false;
}

}  // namespace ga
