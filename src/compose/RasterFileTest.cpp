// RasterFileTest - --selftest's block for "a raster is a source by being a file"
// (compose/RasterFileSource.h, core/ImageLoader.h). Pure CPU. It writes its own rasters under
// out\rastertest -- uncompressed GeoTIFFs by hand (their header and tags are the point), PNGs
// through WIC -- and reads nothing else. Every expected ground point comes from the test's own
// closed forms in doubles (Snyder's UTM INVERSE here; the engine only ever runs the forward).
#include "compose/RasterFileSource.h"

#include "compose/ColorStackSource.h"
#include "compose/DomainSource.h"
#include "compose/SurfaceFrame.h"
#include "core/Common.h"
#include "core/Image.h"
#include "core/ImageLoader.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace ga {

namespace {

constexpr double kPi = 3.14159265358979323846, kD2R = kPi / 180.0, kA = 6378137.0;
constexpr uint32_t kW = 256, kCell = 16;   // a raster is 16 x 16 cells of 16 texels
int gChecks = 0, gFails = 0;

void Check(bool ok, const char* fmt, ...) {
    char b[768];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    ++gChecks;
    if (!ok) ++gFails;
    Log("[rastertest] %s %s", ok ? "ok  " : "FAIL", b);
}

// A cell's colour says where it is: red its row, green its column, blue the checker (or `blue`).
// Cells from row `hole` down are transparent (alpha 0; all black with 3 channels), and the cell
// (blackRow, blackCol) is black.
std::vector<uint8_t> Checker(uint32_t spp, int blue = -1, uint32_t hole = 99, int blackRow = -1,
                             int blackCol = -1) {
    std::vector<uint8_t> px(size_t(kW) * kW * spp);
    for (uint32_t y = 0; y < kW; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const uint32_t row = y / kCell, col = x / kCell;
            uint8_t c[4] = {uint8_t(8 + 15 * row), uint8_t(8 + 15 * col),
                            uint8_t(blue >= 0 ? blue : ((row + col) & 1) ? 224 : 32), 255};
            if (row >= hole) c[3] = 0;
            if (row >= hole && spp == 3) c[0] = c[1] = c[2] = 0;
            if (int(row) == blackRow && int(col) == blackCol) c[0] = c[1] = c[2] = 0;
            std::memcpy(&px[(size_t(y) * kW + x) * spp], c, spp);
        }
    }
    return px;
}

// ---- an uncompressed GeoTIFF: one strip an image, the GeoTIFF tags on the first ----------------
struct TiffImage {
    uint32_t w, h, spp, bps;
    std::vector<uint8_t> px;
    uint32_t subfile;   // NewSubfileType: 1 = a reduced-resolution copy (an overview)
};
struct GeoTags {
    std::vector<double> scale, tie;
    std::vector<uint16_t> keys;
    std::string nodata;
};

bool WriteTiff(const std::string& path, const std::vector<TiffImage>& imgs, const GeoTags& g) {
    std::vector<uint8_t> b{'I', 'I', 42, 0, 0, 0, 0, 0};
    auto blob = [&](const void* p, size_t n) {
        while (b.size() & 1) b.push_back(0);
        const uint32_t at = uint32_t(b.size());
        b.insert(b.end(), static_cast<const uint8_t*>(p), static_cast<const uint8_t*>(p) + n);
        return at;
    };
    auto put = [&](size_t at, uint32_t v, int n) {
        for (int i = 0; i < n; ++i) b[at + i] = uint8_t(v >> (8 * i));
    };
    size_t link = 4;   // where the next IFD's offset is written
    for (size_t k = 0; k < imgs.size(); ++k) {
        const TiffImage& im = imgs[k];
        struct E {
            uint16_t tag, type;
            uint32_t count, value;
        };
        std::vector<E> e;
        auto arr = [&](uint16_t tag, uint16_t type, uint32_t count, const void* p, size_t n) {
            uint32_t v = 0;
            if (n <= 4) std::memcpy(&v, p, n);   // four bytes or fewer stand in the entry itself
            else v = blob(p, n);
            e.push_back({tag, type, count, v});
        };
        const uint32_t data = blob(im.px.data(), im.px.size());
        const std::vector<uint16_t> bits(im.spp, uint16_t(im.bps));
        e.push_back({254, 4, 1, im.subfile});
        e.push_back({256, 4, 1, im.w});
        e.push_back({257, 4, 1, im.h});
        arr(258, 3, im.spp, bits.data(), bits.size() * 2);
        e.push_back({259, 3, 1, 1});                        // no compression
        e.push_back({262, 3, 1, im.spp >= 3 ? 2u : 1u});    // RGB, or black is zero
        e.push_back({273, 4, 1, data});
        e.push_back({277, 3, 1, im.spp});
        e.push_back({278, 4, 1, im.h});
        e.push_back({279, 4, 1, uint32_t(im.px.size())});
        e.push_back({284, 3, 1, 1});
        if (im.spp == 4) e.push_back({338, 3, 1, 2});       // unassociated alpha
        if (k == 0 && !g.scale.empty()) arr(33550, 12, uint32_t(g.scale.size()), g.scale.data(), g.scale.size() * 8);
        if (k == 0 && !g.tie.empty()) arr(33922, 12, uint32_t(g.tie.size()), g.tie.data(), g.tie.size() * 8);
        if (k == 0 && !g.keys.empty()) arr(34735, 3, uint32_t(g.keys.size()), g.keys.data(), g.keys.size() * 2);
        if (k == 0 && !g.nodata.empty()) arr(42113, 2, uint32_t(g.nodata.size() + 1), g.nodata.c_str(), g.nodata.size() + 1);
        while (b.size() & 1) b.push_back(0);
        const size_t ifd = b.size();
        put(link, uint32_t(ifd), 4);
        b.resize(ifd + 2 + e.size() * 12 + 4, 0);
        put(ifd, uint32_t(e.size()), 2);
        for (size_t i = 0; i < e.size(); ++i) {
            put(ifd + 2 + i * 12, e[i].tag, 2);
            put(ifd + 4 + i * 12, e[i].type, 2);
            put(ifd + 6 + i * 12, e[i].count, 4);
            put(ifd + 10 + i * 12, e[i].value, 4);
        }
        link = ifd + 2 + e.size() * 12;
    }
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(b.data()), std::streamsize(b.size()));
    return bool(f);
}

void WriteText(const std::string& path, const std::string& text) {
    std::ofstream(path, std::ios::binary | std::ios::trunc) << text;
}

// ---- where a test raster is, and the test's own way back from a texel to the ground ----------
struct Raster {
    std::string file;
    int epsg;
    double ox, oy, sx, sy;   // the outer corner of texel (0, 0) and the signed scale, CRS units
};

// Snyder (1987) eqs. 8-18 to 8-25, the footpoint-latitude INVERSE, WGS84, northern zones.
void UtmInverse(double E, double N, int zone, double& lat, double& lon) {
    const double f = 1.0 / 298.257223563, k0 = 0.9996, e2 = f * (2.0 - f), ep2 = e2 / (1.0 - e2);
    const double x = E - 500000.0, M = N / k0;
    const double mu = M / (kA * (1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0));
    const double r = std::sqrt(1.0 - e2), e1 = (1.0 - r) / (1.0 + r);
    const double p = mu + (1.5 * e1 - 27.0 * std::pow(e1, 3) / 32.0) * std::sin(2.0 * mu) +
                     (21.0 * e1 * e1 / 16.0 - 55.0 * std::pow(e1, 4) / 32.0) * std::sin(4.0 * mu) +
                     (151.0 * std::pow(e1, 3) / 96.0) * std::sin(6.0 * mu) +
                     (1097.0 * std::pow(e1, 4) / 512.0) * std::sin(8.0 * mu);
    const double s = std::sin(p), c = std::cos(p), t = std::tan(p);
    const double C1 = ep2 * c * c, T1 = t * t, N1 = kA / std::sqrt(1.0 - e2 * s * s);
    const double R1 = kA * (1.0 - e2) / std::pow(1.0 - e2 * s * s, 1.5), D = x / (N1 * k0);
    lat = p - (N1 * t / R1) *
                  (D * D / 2.0 - (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1 - 9.0 * ep2) * std::pow(D, 4) / 24.0 +
                   (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1 - 252.0 * ep2 - 3.0 * C1 * C1) * std::pow(D, 6) / 720.0);
    lon = (-183.0 + 6.0 * zone) * kD2R +
          (D - (1.0 + 2.0 * T1 + C1) * std::pow(D, 3) / 6.0 +
           (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1 + 8.0 * ep2 + 24.0 * T1 * T1) * std::pow(D, 5) / 120.0) / c;
}

// Texel coordinates (0 = the outer edge of texel 0) -> WGS84 radians.
void PixelLatLon(const Raster& r, double px, double py, double& lat, double& lon) {
    const double X = r.ox + px * r.sx, Y = r.oy + py * r.sy;
    if (r.epsg == 4326) {
        lat = Y * kD2R;
        lon = X * kD2R;
    } else if (r.epsg == 3857) {
        lon = X / kA;
        lat = 2.0 * std::atan(std::exp(Y / kA)) - 0.5 * kPi;
    } else {
        UtmInverse(X, Y, r.epsg - 32600, lat, lon);
    }
}

// kW texels of `g` metres, centred on 25.8997 N, 80.1239 W (Haulover), in the CRS asked for.
Raster Make(int epsg, double g, const std::string& file) {
    const double lat = 25.8997, lon = -80.1239, half = kW / 2.0;
    Raster r{file, epsg, 0, 0, g, -g};
    if (epsg == 4326) {
        const double d = g / (kA * kD2R);
        r.sx = d / std::cos(lat * kD2R);
        r.sy = -d;
        r.ox = lon - half * r.sx;
        r.oy = lat - half * r.sy;
    } else if (epsg == 3857) {
        r.sx = g / std::cos(lat * kD2R);
        r.sy = -r.sx;
        r.ox = kA * lon * kD2R - half * r.sx;
        r.oy = kA * std::log(std::tan(0.25 * kPi + 0.5 * lat * kD2R)) - half * r.sy;
    } else {   // (587754, 2864869) is the point in UTM 17N, to a metre
        r.ox = 587754.0 - half * g;
        r.oy = 2864869.0 + half * g;
    }
    return r;
}

GeoTags TagsOf(const Raster& r, int epsg, const std::string& nodata = "") {
    const bool geo = epsg == 4326;
    GeoTags t;
    t.scale = {r.sx, -r.sy, 0.0};
    t.tie = {0.0, 0.0, 0.0, r.ox, r.oy, 0.0};
    t.keys = {1, 1, 0, 3, 1024, 0, 1, uint16_t(geo ? 2 : 1), 1025, 0, 1, 1,
              uint16_t(geo ? 2048 : 3072), 0, 1, uint16_t(epsg)};
    t.nodata = nodata;
    return t;
}

std::string World(const Raster& r) {   // A D B E C F, (C, F) the CENTRE of texel (0, 0)
    char b[256];
    snprintf(b, sizeof(b), "%.17g\n0\n0\n%.17g\n%.17g\n%.17g\n", r.sx, r.sy, r.ox + 0.5 * r.sx,
             r.oy + 0.5 * r.sy);
    return b;
}

const char* kPrj32617 =
    "PROJCS[\"WGS 84 / UTM zone 17N\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\","
    "6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM["
    "\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]],"
    "PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"central_meridian\",-81],UNIT[\"metre\",1],"
    "AUTHORITY[\"EPSG\",\"32617\"]]";

// Every anchor (the four corners, the centre, a cell corner in each quadrant), asked a quarter
// texel off it in the four diagonal directions: inside the file the source must answer the cell
// that lies there -- a placement off by a quarter texel toward any cell edge, either axis, lands
// in the neighbour -- and outside it nothing. Returns the points that failed.
int Misses(RasterFileSource& s, const Raster& r, std::string* first) {
    const double q = kW / 4.0;
    const double anchors[9][2] = {{0, 0}, {4 * q, 0}, {0, 4 * q}, {4 * q, 4 * q}, {2 * q, 2 * q},
                                  {q, q}, {3 * q, q}, {q, 3 * q}, {3 * q, 3 * q}};
    int bad = 0;
    for (const auto& a : anchors) {
        for (int k = 0; k < 4; ++k) {
            const double px = a[0] + ((k & 1) ? 0.25 : -0.25), py = a[1] + ((k & 2) ? 0.25 : -0.25);
            double lat = 0.0, lon = 0.0;
            PixelLatLon(r, px, py, lat, lon);
            uint8_t c[4] = {0, 0, 0, 0};
            const float w = s.Sample(lat, lon, 1e-4, PaintCtx{}, c);
            const bool inside = px > 0.0 && py > 0.0 && px < kW && py < kW;
            const int row = int(std::lround((c[0] - 8) / 15.0)), col = int(std::lround((c[1] - 8) / 15.0));
            const bool ok = inside ? (w > 0.0f && row == int(py) / int(kCell) && col == int(px) / int(kCell))
                                   : w <= 0.0f;
            if (!ok && first && first->empty()) {
                char b[160];
                snprintf(b, sizeof(b), "texel (%.2f, %.2f): weight %.3f, cell (%d, %d)", px, py, w, row, col);
                *first = b;
            }
            bad += ok ? 0 : 1;
        }
    }
    return bad;
}

}  // namespace

bool RunRasterFileSelfTest() {
    gChecks = gFails = 0;
    std::error_code ec;
    std::filesystem::create_directories("out/rastertest/folder", ec);
    Log("[rastertest] ---- a raster is a source by being a file (compose/RasterFileSource.h): its "
        "own GeoTIFFs and PNGs under out\\rastertest");
    LoaderRegistry reg;
    RegisterImageLoaders(reg);
    const RasterEntry none;
    std::string why;

    // ---- 1. PLACEMENT: a checkerboard at 0.5 m and 0.05 m, in 4326, 3857, 32617, PNG + world --
    struct Case {
        const char* name;
        int epsg;
        double g;
        const char* form;
    };
    const Case cases[] = {{"geo4326_050", 4326, 0.5, "tif"},     {"geo4326_005", 4326, 0.05, "tif"},
                          {"merc3857_050", 3857, 0.5, "tif"},    {"merc3857_005", 3857, 0.05, "tif rgb nodata 0"},
                          {"utm32617_050", 32617, 0.5, "tif"},   {"utm32617_005", 32617, 0.05, "tif"},
                          {"png32617_050", 32617, 0.5, "png .pgw .prj"},
                          {"png3857_005", 3857, 0.05, "png .wld, crs declared"}};
    std::vector<std::unique_ptr<RasterFileSource>> srcs;
    std::vector<Raster> rasters;
    for (const Case& c : cases) {
        const std::string base = std::string("out/rastertest/") + c.name;
        const bool png = c.form[0] == 'p', rgb = std::strstr(c.form, "rgb") != nullptr;
        Raster r = Make(c.epsg, c.g, base + (png ? ".png" : ".tif"));
        RasterEntry e;
        if (!png) {
            const uint32_t spp = rgb ? 3 : 4;
            WriteTiff(r.file, {{kW, kW, spp, 8, Checker(spp, -1, 99, rgb ? 5 : -1, rgb ? 10 : -1), 0}},
                      TagsOf(r, c.epsg, rgb ? "0" : ""));
        } else {
            const std::vector<uint8_t> px = Checker(4);
            SavePng(std::wstring(r.file.begin(), r.file.end()), px.data(), kW, kW, kW * 4, px.size());
            const bool prj = std::strstr(c.form, ".prj") != nullptr;
            WriteText(base + (prj ? ".pgw" : ".wld"), World(r));
            if (prj) WriteText(base + ".prj", kPrj32617);
            else e.crs = "EPSG:" + std::to_string(c.epsg);
        }
        auto s = std::make_unique<RasterFileSource>();
        why.clear();
        const bool loaded = s->Load(reg, r.file, e, &why);
        std::string first;
        const int bad = loaded ? Misses(*s, r, &first) : -1;
        Check(loaded && bad == 0, "%s (EPSG:%d, %.2f m a texel, %s): %s", c.name, c.epsg, c.g, c.form,
              !loaded ? why.c_str()
                      : bad ? (std::to_string(bad) + " of 36 points wrong, first " + first).c_str()
                            : "36 points: each the cell that lies there, within a quarter texel; outside the file, nothing");
        if (loaded && rgb) {   // the black cell (5, 10) of an RGB file whose nodata is 0
            double lat = 0.0, lon = 0.0;
            uint8_t k[4];
            PixelLatLon(r, 10 * 16 + 8, 5 * 16 + 8, lat, lon);
            const float w0 = s->Sample(lat, lon, 1e-4, PaintCtx{}, k);
            PixelLatLon(r, 11 * 16 + 8, 5 * 16 + 8, lat, lon);
            const float w1 = s->Sample(lat, lon, 1e-4, PaintCtx{}, k);
            Check(w0 == 0.0f && w1 == 1.0f, "%s: GDAL nodata 0 is alpha 0 -- the black cell weighs %.3f, its neighbour %.3f",
                  c.name, w0, w1);
        }
        srcs.push_back(std::move(s));
        rasters.push_back(r);
    }

    // ---- 2. PLANTS, on the zone 17 file: each must be caught ---------------------------------
    {
        RasterFileSource& u = *srcs[4];
        const Raster& ur = rasters[4];
        const GeoRef good = u.Ref();
        GeoRef p = good;
        p.epsg = 32619;
        u.Place(p, nullptr);
        int bad = Misses(u, ur, nullptr);
        Check(bad > 0, "PLANT zone 19 assumed for the zone 17 file: %s (%d of 36 points wrong)", bad ? "CAUGHT" : "NOT caught", bad);
        p = good;
        p.originY = good.originY + good.scaleY * good.height;
        p.scaleY = -good.scaleY;
        u.Place(p, nullptr);
        bad = Misses(u, ur, nullptr);
        Check(bad > 0, "PLANT rows read bottom-up: %s (%d of 36 points wrong)", bad ? "CAUGHT" : "NOT caught", bad);
        p = good;
        p.originX += 0.5 * good.scaleX;
        p.originY += 0.5 * good.scaleY;
        u.Place(p, nullptr);
        bad = Misses(u, ur, nullptr);
        Check(bad > 0, "PLANT centre taken for corner (half a texel): %s (%d of 36 points wrong)", bad ? "CAUGHT" : "NOT caught", bad);
        u.Place(good, nullptr);
        bad = Misses(u, ur, nullptr);
        Check(bad == 0, "the file's own georeference placed again: %d of 36 points wrong", bad);
    }

    // ---- 3. TWO THAT OVERLAP: the finer over the coarser; where its alpha is 0 the lower shows --
    {
        RasterFileSource& coarse = *srcs[4];   // 0.5 m, 128 m a side
        const Raster fr = Make(32617, 0.05, "out/rastertest/fine_over.tif");
        WriteTiff(fr.file, {{kW, kW, 4, 8, Checker(4, 128, 8), 0}}, TagsOf(fr, 32617));
        RasterFileSource fine;
        const bool loaded = fine.Load(reg, fr.file, none, &why);
        std::vector<ColorSource*> st{&fine, &coarse};   // handed over upside down
        StackOrder(st);
        Check(loaded && st[0] == &coarse && st[1] == &fine,
              "the default order puts the 0.5 m file under the 0.05 m file (handed over the other way round)");
        DomainCompositor dc;
        dc.SetBlend(DomainCompositor::Blend::LayeredOver);
        std::vector<std::shared_ptr<DomainSource>> keep;
        for (ColorSource* c : st) {   // Assembly's leaf(): the layer, normalized
            auto l = std::make_shared<ColorLayerSource>(c);
            keep.push_back(l);
            auto n = NormalizeToSi(l);
            dc.Add(n ? n : l);
        }
        const char* where[2] = {"in the fine file's opaque half: the fine file's texel",
                                "in its alpha-0 half: the coarse file's texel"};
        for (int k = 0; k < 2; ++k) {
            double lat = 0.0, lon = 0.0;
            PixelLatLon(fr, 72.0, k ? 200.0 : 40.0, lat, lon);
            DomainQuery q;
            q.lat = lat / kD2R;
            q.lon = lon / kD2R;
            q.groundM = 1e-4;
            DomainValue v;
            const bool got = dc.SampleBlended(q, v);
            uint8_t want[4] = {0, 0, 0, 0};
            (k ? static_cast<ColorSource&>(coarse) : fine).Sample(lat, lon, 1e-4, PaintCtx{}, want);
            double worst = 0.0;
            for (int i = 0; i < 3; ++i) worst = (std::max)(worst, std::abs(v.c[i] * 255.0 - want[i]));
            Check(got && worst <= 1.0 && v.weight > 0.99f,
                  "composed %s (%u,%u,%u), worst |d| %.2f of 255, weight %.3f", where[k], want[0],
                  want[1], want[2], worst, v.weight);
        }
    }

    // ---- 4. IDENTITY: a file replaced by another of the same size is another tree ------------
    {
        const Raster ir = Make(4326, 0.5, "out/rastertest/identity.tif");
        std::vector<uint8_t> px = Checker(4);
        WriteTiff(ir.file, {{kW, kW, 4, 8, px, 0}}, TagsOf(ir, 4326));
        const auto size0 = std::filesystem::file_size(ir.file, ec);
        RasterFileSource a;
        a.Load(reg, ir.file, none, &why);
        const std::string id0 = ColorLayerSource(&a).Identity();
        px[(size_t(100) * kW + 100) * 4] ^= 1u;   // one bit of one texel
        WriteTiff(ir.file, {{kW, kW, 4, 8, px, 0}}, TagsOf(ir, 4326));
        const auto size1 = std::filesystem::file_size(ir.file, ec);
        RasterFileSource b;
        b.Load(reg, ir.file, none, &why);
        const std::string id1 = ColorLayerSource(&b).Identity();
        const size_t cut = id0.rfind("fnv64");
        Check(size0 == size1 && !id0.empty() && id0 != id1,
              "a file replaced by one of the same size (%llu bytes, one bit apart) is a new identity: "
              "...%s -> ...%s", static_cast<unsigned long long>(size1),
              id0.substr(cut == std::string::npos ? 0 : cut).c_str(),
              id1.substr(cut == std::string::npos ? 0 : cut).c_str());
    }

    // ---- 5. REFUSALS: by name, and the run goes on -------------------------------------------
    {
        const Raster br = Make(32617, 0.5, "out/rastertest/folder/b_state_plane.tif");
        WriteTiff(br.file, {{kW, kW, 4, 8, Checker(4), 0}}, TagsOf(br, 2249));
        const Raster gr = Make(4326, 0.5, "out/rastertest/folder/a_good.tif");
        WriteTiff(gr.file, {{kW, kW, 4, 8, Checker(4), 0}}, TagsOf(gr, 4326));
        RasterFileSource bad;
        why.clear();
        const bool took = bad.Load(reg, br.file, none, &why);
        Check(!took && why.find("EPSG:2249") != std::string::npos,
              "a file in EPSG:2249 (a state plane) is refused by name: %s", why.c_str());
        RasterEntry fe;
        fe.folder = "out/rastertest/folder";
        fe.match = "*.tif";
        fe.name = "folder";
        const auto got = LoadRasterSources({fe});
        Check(got.size() == 1 && got[0]->Info().name == "folder.a_good",
              "a folder of two, one refused: the run goes on with the other (%zu source(s)%s%s)",
              got.size(), got.empty() ? "" : ": ", got.empty() ? "" : got[0]->Info().name.c_str());
        const Raster hr = Make(4326, 0.5, "out/rastertest/height16.tif");
        WriteTiff(hr.file, {{kW, kW, 1, 16, std::vector<uint8_t>(size_t(kW) * kW * 2, 7), 0}}, TagsOf(hr, 4326));
        RasterFileSource h;
        why.clear();
        const bool heightTook = h.Load(reg, hr.file, none, &why);
        Check(!heightTook && why.find("height") != std::string::npos,
              "one channel of 16 bits is kind height, refused by name: %s", why.c_str());
    }

    // ---- 6. A FILE WITH AN OVERVIEW: WIC shows no reduced-resolution IFD (core/ImageLoader.h), so
    // the file loads whole and its level 1 is the source's own 2x2 mean -- the cell -- and never the
    // overview's magenta. If a WIC ever shows it, this line says so. --------------------------------
    {
        const Raster orr = Make(32617, 0.5, "out/rastertest/overview.tif");
        std::vector<uint8_t> magenta(size_t(kW / 2) * (kW / 2) * 4);
        for (size_t i = 0; i < magenta.size(); i += 4) {
            magenta[i] = 255;
            magenta[i + 1] = 0;
            magenta[i + 2] = 255;
            magenta[i + 3] = 255;
        }
        WriteTiff(orr.file, {{kW, kW, 4, 8, Checker(4), 0}, {kW / 2, kW / 2, 4, 8, magenta, 1}}, TagsOf(orr, 32617));
        RasterFileSource ov;
        const bool loaded = ov.Load(reg, orr.file, none, &why);
        double lat = 0.0, lon = 0.0;
        PixelLatLon(orr, 4 * 16 + 8, 2 * 16 + 8, lat, lon);
        uint8_t fine[4] = {0, 0, 0, 0}, coarse[4] = {0, 0, 0, 0};
        ov.Sample(lat, lon, 1e-4, PaintCtx{}, fine);
        ov.Sample(lat, lon, 2.5 * 0.5 / std::cos(lat), PaintCtx{}, coarse);   // asked at 1.25 m: level 1
        Check(loaded && fine[0] == 38 && fine[1] == 68 && coarse[0] == 38 && coarse[1] == 68 && coarse[2] == 32,
              "a file with an overview IFD loads; WIC does not show the overview, so at 1.25 m (level 1) "
              "it answers its own mean of the cell (%u,%u,%u), not the overview's magenta; at its "
              "grain (%u,%u,%u)", coarse[0], coarse[1], coarse[2], fine[0], fine[1], fine[2]);
    }

    // ---- 7. faceWindows auto: the blocks the grains ask for, walked -----------------------------
    {
        const double lat = 25.8547, lon = -80.19, dLat = 1000.0 / 111320.0,
                     dLon = dLat / std::cos(lat * kD2R);
        const std::string key = SurfaceFrame::AutoKey(
            {{"two_km_050", lon - dLon, lat - dLat, lon + dLon, lat + dLat, 0.5},
             {"two_hundred_m_005", lon - 0.1 * dLon, lat - 0.1 * dLat, lon + 0.1 * dLon, lat + 0.1 * dLat, 0.05}});
        SurfaceFrame sf;
        const bool declared = sf.DeclareBlocks(key);
        std::string rungs;
        for (const FaceWindow& b : sf.blocks) rungs += std::to_string(b.rung) + " ";
        auto walk = [&](double la, double lo) {
            const double d[3] = {std::cos(la * kD2R) * std::cos(lo * kD2R), std::sin(la * kD2R),
                                 std::cos(la * kD2R) * std::sin(lo * kD2R)};
            SurfaceFrame::WalkStep steps[SurfaceFrame::kMaxRanks];
            return sf.Walk(d, steps);
        };
        const uint32_t atCentre = walk(lat, lon), atCorner = walk(lat + 0.9 * dLat, lon + 0.9 * dLon);
        Check(declared && sf.blocks.size() <= SurfaceFrame::kMaxBlocks && atCentre == 5 && atCorner >= 3,
              "auto: %zu blocks (rungs %s); the 0.05 m file's centre walks %u ranks, the 0.5 m file's "
              "corner %u (4 where its rank-4 block found a row)", sf.blocks.size(), rungs.c_str(),
              atCentre, atCorner);
    }

    Log("[rastertest] ---- %s: %d checks, %d FAIL", gFails ? "FAIL" : "PASS", gChecks, gFails);
    return gFails == 0;
}

}  // namespace ga
