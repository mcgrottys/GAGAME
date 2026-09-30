#include "core/ImageLoader.h"

#include "core/Common.h"
#include "core/Json.h"

#include <windows.h>
#include <wincodec.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <mutex>
#include <vector>

namespace ga {

namespace {

thread_local std::string tWhy;
const char* kSources = "cache/sources";   // the index and the raw rows of sequential formats

bool ReadAll(const std::string& path, std::vector<uint8_t>& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
}

// FNV-1a 64, folded over each input in turn: the file, its sidecars, the declared crs.
void Fnv(uint64_t& h, const void* p, size_t n) {
    const uint8_t* b = static_cast<const uint8_t*>(p);
    for (size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 1099511628211ull;
    }
}

std::string Hr(const char* what, HRESULT hr) {
    char b[128];
    snprintf(b, sizeof(b), "%s (HRESULT 0x%08lX)", what, static_cast<unsigned long>(hr));
    return b;
}

std::string Hex(uint64_t v) {
    char b[24];
    snprintf(b, sizeof(b), "%016llx", static_cast<unsigned long long>(v));
    return b;
}

// Written beside, then moved over: a reader sees the whole old file or the whole new one.
bool Publish(const std::string& path, const void* p, size_t n) {
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        f.write(static_cast<const char*>(p), std::streamsize(n));
        if (!f) return false;
    }
    return MoveFileExA(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING) != 0;
}

// ---- the shapes the GeoTIFF tags arrive in through WIC's metadata query reader ------------------
std::vector<double> QueryDoubles(IWICMetadataQueryReader* q, const wchar_t* name) {
    std::vector<double> out;
    PROPVARIANT v;
    PropVariantInit(&v);
    if (SUCCEEDED(q->GetMetadataByName(name, &v))) {
        if (v.vt == (VT_VECTOR | VT_R8)) out.assign(v.cadbl.pElems, v.cadbl.pElems + v.cadbl.cElems);
        else if (v.vt == VT_R8) out.push_back(v.dblVal);
    }
    PropVariantClear(&v);
    return out;
}
std::vector<uint16_t> QueryShorts(IWICMetadataQueryReader* q, const wchar_t* name) {
    std::vector<uint16_t> out;
    PROPVARIANT v;
    PropVariantInit(&v);
    if (SUCCEEDED(q->GetMetadataByName(name, &v))) {
        if (v.vt == (VT_VECTOR | VT_UI2)) out.assign(v.caui.pElems, v.caui.pElems + v.caui.cElems);
        else if (v.vt == VT_UI2) out.push_back(v.uiVal);
    }
    PropVariantClear(&v);
    return out;
}
std::string QueryText(IWICMetadataQueryReader* q, const wchar_t* name) {
    std::string out;
    PROPVARIANT v;
    PropVariantInit(&v);
    if (SUCCEEDED(q->GetMetadataByName(name, &v))) {
        if (v.vt == VT_LPSTR && v.pszVal) out = v.pszVal;
        if (v.vt == VT_LPWSTR && v.pwszVal) {
            for (const wchar_t* c = v.pwszVal; *c; ++c) out += static_cast<char>(*c);
        }
    }
    PropVariantClear(&v);
    return out;
}
// The sidecar beside `path` with the first of these extensions that exists, and its bytes.
bool Sidecar(const std::string& path, std::initializer_list<const char*> exts, std::string& found,
             std::vector<uint8_t>& bytes) {
    const size_t dot = path.find_last_of('.');
    const std::string stem = dot == std::string::npos ? path : path.substr(0, dot);
    for (const char* e : exts) {
        if (ReadAll(stem + e, bytes)) {
            found = stem + e;
            return true;
        }
    }
    return false;
}

// A .prj's EPSG: the LAST authority in its text, which in WKT1 (AUTHORITY["EPSG","n"]) and WKT2
// (ID["EPSG",n]) alike is the outermost element's -- the CRS's own, not its datum's.
int PrjEpsg(const std::string& t) {
    size_t at = std::string::npos;
    for (const char* key : {"AUTHORITY[\"EPSG\",", "ID[\"EPSG\","}) {
        const size_t k = t.rfind(key);
        if (k != std::string::npos && (at == std::string::npos || k > at)) at = k;
    }
    if (at == std::string::npos) return 0;
    size_t i = t.find(',', at) + 1;
    while (i < t.size() && (t[i] == '"' || t[i] == ' ')) ++i;
    return atoi(t.c_str() + i);
}

// Rows as they lie in a file: `nc` bytes a texel, `w` texels a row, from `base`. A read is a seek
// (a positional ReadFile), so threads read one file at once without a lock.
struct RowFile {
    HANDLE h = INVALID_HANDLE_VALUE;
    uint64_t base = 0;
    uint32_t w = 0, nc = 0;
    ~RowFile() {
        if (h != INVALID_HANDLE_VALUE) CloseHandle(h);
    }
    bool Open(const std::string& path, uint64_t at, uint32_t width, uint32_t height, uint32_t chans) {
        std::error_code ec;
        if (std::filesystem::file_size(path, ec) < at + uint64_t(width) * height * chans || ec) return false;
        h = CreateFileA(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING,
                        FILE_ATTRIBUTE_NORMAL, nullptr);
        base = at;
        w = width;
        nc = chans;
        return h != INVALID_HANDLE_VALUE;
    }
    bool Read(uint32_t x, uint32_t y, uint32_t n, uint8_t* dst) const {
        const uint64_t at = base + (uint64_t(y) * w + x) * nc;
        OVERLAPPED o{};
        o.Offset = DWORD(at);
        o.OffsetHigh = DWORD(at >> 32);
        DWORD got = 0;
        return ReadFile(h, dst, n * nc, &got, &o) && got == n * nc;
    }
};

// One file of any size: its decoder kept on the file and asked for a window (a TIFF), or its raw
// rows (a manifest's file; a PNG or a JPEG decoded once).
class ImageLoader : public FieldLoader {
public:
    const char* Name() const override { return m_name.c_str(); }
    const char* Structure() const override { return m_structure.c_str(); }
    const GeoRef& Ref() const override { return m_ref; }
    uint8_t GradeSig() const override { return kG0; }
    uint32_t Channels() const override { return m_height ? 1 : 4; }
    // RGBA as 0..255 floats; a texel the file calls nodata, or gives alpha 0, stays all zero. A
    // height: its one channel, NaN where the file says nodata.
    bool LoadTile(uint32_t tx, uint32_t ty, uint32_t tw, uint32_t th, TilePayload& out) override {
        out.width = tw;
        out.height = th;
        out.channels = Channels();
        out.data.assign(size_t(tw) * th * out.channels, m_height ? NAN : 0.0f);
        out.allNoData = true;
        out.coverage = 0.0f;
        const uint64_t x0 = uint64_t(tx) * tw, y0 = uint64_t(ty) * th;
        if (x0 >= m_ref.width || y0 >= m_ref.height) return false;
        const uint32_t w = uint32_t((std::min)(uint64_t(tw), m_ref.width - x0));
        const uint32_t h = uint32_t((std::min)(uint64_t(th), m_ref.height - y0));
        std::vector<uint8_t> px(size_t(w) * h * 4);
        if (!Pixels(uint32_t(x0), uint32_t(y0), w, h, px.data())) return false;
        uint32_t real = 0;
        for (uint32_t r = 0; r < h; ++r) {
            for (uint32_t c = 0; c < w; ++c) {
                uint8_t* p = &px[(size_t(r) * w + c) * 4];
                if (m_height) {
                    float v;
                    std::memcpy(&v, p, 4);
                    if (std::isnan(v) || m_ref.IsNoData(v)) continue;
                    out.data[size_t(r) * tw + c] = v;
                    ++real;
                    continue;
                }
                if (m_dataBand) p[3] = 255;   // a fourth band the file does not call alpha
                const bool nodata = m_ref.hasNoData && m_ref.IsNoData(p[0]) &&
                                    m_ref.IsNoData(p[1]) && m_ref.IsNoData(p[2]);
                if (nodata || p[3] == 0) continue;
                float* d = &out.data[(size_t(r) * tw + c) * 4];
                for (int k = 0; k < 4; ++k) d[k] = p[k];
                ++real;
            }
        }
        out.allNoData = real == 0;
        out.coverage = float(real) / float(tw * th);
        return real > 0;
    }
    // A window's four bytes a texel -- RGBA8, or a height's float32 -- from WIC for the rectangle,
    // one decode at a time (a height from the frame itself, 16 bits widened in place from the
    // last texel); or the rows.
    bool Pixels(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint8_t* dst) {
        if (m_rows.h != INVALID_HANDLE_VALUE) {
            std::vector<uint8_t> row(size_t(w) * m_rows.nc);
            for (uint32_t r = 0; r < h; ++r) {
                if (!m_rows.Read(x, y + r, w, row.data())) return false;
                uint8_t* d = dst + size_t(r) * w * 4;
                for (uint32_t c = 0; c < w; ++c) {
                    for (uint32_t k = 0; k < 3; ++k) d[c * 4 + k] = row[c * m_rows.nc + k];
                    d[c * 4 + 3] = m_rows.nc == 4 ? row[c * 4 + 3] : 255;
                }
            }
            return true;
        }
        static thread_local const HRESULT co = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        (void)co;
        std::lock_guard<std::mutex> lk(m_mx);
        const WICRect rc{INT(x), INT(y), INT(w), INT(h)};
        if (m_bits != 16) return SUCCEEDED(m_conv->CopyPixels(&rc, w * 4, w * h * 4, dst));
        if (FAILED(m_conv->CopyPixels(&rc, w * 2, w * h * 2, dst))) return false;
        for (size_t i = size_t(w) * h; i-- > 0;) {
            uint16_t u;
            std::memcpy(&u, dst + 2 * i, 2);
            const float f = m_signed ? float(int16_t(u)) : float(u);
            std::memcpy(dst + 4 * i, &f, 4);
        }
        return true;
    }

    std::string m_name, m_structure, m_unit;
    GeoRef m_ref;
    Com<IWICBitmapSource> m_conv;   // the decoder's frame: converted to RGBA8, or a height's own
    std::mutex m_mx;
    RowFile m_rows;
    bool m_dataBand = false, m_height = false, m_signed = false;
    UINT m_bits = 32;
};

// A VERTICAL CRS (GeoKey 4096) as the unit and datum UnitSpec reads (core/GaUnits.h); 4099, where the
// file has it, names the unit instead. Every other code is refused by name: a depth's sign, a geoid's
// separation and a user's datum are never guessed.
struct Vertical {
    int epsg;
    const char* unit;
};
const Vertical kVertical[] = {{5703, "m NAVD88"},  {6360, "ftUS NAVD88"}, {8228, "ft NAVD88"},
                              {5773, "m EGM96"},   {3855, "m EGM2008"},   {5714, "m MSL"},
                              {4979, "m WGS84"}};

}  // namespace

const std::string& ImageLoaderWhy() { return tWhy; }

uint64_t ContentHash(const std::string& path, bool* indexed) {
    static std::mutex mx;
    std::lock_guard<std::mutex> lk(mx);
    std::error_code ec;
    const std::string key = std::filesystem::absolute(path, ec).generic_string();
    const std::string size = std::to_string(std::filesystem::file_size(path, ec));
    if (ec) return 0;
    const std::string time =
        std::to_string(std::filesystem::last_write_time(path, ec).time_since_epoch().count());
    const std::string index = std::string(kSources) + "/identity.json";
    std::vector<uint8_t> text;
    ReadAll(index, text);
    const JsonValue doc = JsonParser::Parse(std::string(text.begin(), text.end()), nullptr);
    const JsonValue* files = doc.Get("files");
    for (size_t i = 0; files && i < files->arr.size(); ++i) {
        const JsonValue& f = files->arr[i];
        if (f.Str("path") == key && f.Str("size") == size && f.Str("time") == time) {
            if (indexed) *indexed = true;
            return std::strtoull(f.Str("fnv64").c_str(), nullptr, 16);
        }
    }
    uint64_t h = 1469598103934665603ull;
    std::ifstream in(path, std::ios::binary);
    std::vector<char> buf(size_t(4) << 20);
    while (in) {
        in.read(buf.data(), std::streamsize(buf.size()));
        Fnv(h, buf.data(), size_t(in.gcount()));
    }
    std::string out = "{\"files\": [\n";
    auto row = [&out](const std::string& p, const std::string& s, const std::string& t,
                      const std::string& v) {
        out += (out.size() > 13 ? ",\n" : "") + std::string("  {\"path\": \"") + p + "\", \"size\": \"" +
               s + "\", \"time\": \"" + t + "\", \"fnv64\": \"" + v + "\"}";
    };
    for (size_t i = 0; files && i < files->arr.size(); ++i) {
        const JsonValue& f = files->arr[i];
        if (f.Str("path") != key) row(f.Str("path"), f.Str("size"), f.Str("time"), f.Str("fnv64"));
    }
    row(key, size, time, Hex(h));
    out += "\n]}\n";
    std::filesystem::create_directories(kSources, ec);
    Publish(index, out.data(), out.size());
    if (indexed) *indexed = false;
    return h;
}

bool CrsOfEpsg(int epsg, CrsKind& kind, int& zone, bool& south, std::string* why) {
    zone = 0;
    south = false;
    kind = CrsKind::Unknown;
    if (epsg == 4326 || epsg == 4269 || epsg == 6318) kind = CrsKind::Geographic;   // NAD83 as WGS84
    if (epsg == 3857) kind = CrsKind::WebMercator;
    if (epsg >= 32601 && epsg <= 32660) zone = epsg - 32600;
    if (epsg >= 32701 && epsg <= 32760) {
        zone = epsg - 32700;
        south = true;
    }
    if (epsg >= 26901 && epsg <= 26923) zone = epsg - 26900;   // NAD83 / UTM zone zzN
    if (epsg >= 6330 && epsg <= 6348) zone = epsg - 6329;      // NAD83(2011) / UTM zone 1N..19N
    if (zone) kind = CrsKind::TransverseMercator;
    if (kind == CrsKind::Unknown && why) {
        *why = "EPSG:" + std::to_string(epsg) +
               " is not a projection this engine evaluates (4326, 4269, 6318, 3857, UTM "
               "32601-32660, 32701-32760, 26901-26923, 6330-6348)";
    }
    return kind != CrsKind::Unknown;
}

int ParseEpsg(const std::string& t) {
    const size_t i = (t.size() > 5 && _strnicmp(t.c_str(), "EPSG:", 5) == 0) ? 5 : 0;
    if (i >= t.size()) return 0;
    for (size_t k = i; k < t.size(); ++k) {
        if (!std::isdigit(static_cast<unsigned char>(t[k]))) return 0;
    }
    return atoi(t.c_str() + i);
}

std::unique_ptr<FieldLoader> OpenImageFile(const std::string& path, const GeoRef* declared) {
    tWhy.clear();
    auto refuse = [](std::string why) -> std::unique_ptr<FieldLoader> {
        tWhy = std::move(why);
        return nullptr;
    };
    bool indexed = false;
    const uint64_t own = ContentHash(path, &indexed);   // the file's bytes, not held
    if (own == 0) return refuse("the file cannot be read");
    static thread_local const HRESULT co = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    (void)co;   // RPC_E_CHANGED_MODE: the thread's COM is someone else's, and WIC still works
    auto L = std::make_unique<ImageLoader>();
    Com<IWICImagingFactory> fac;
    Com<IWICBitmapDecoder> dec;
    Com<IWICBitmapFrameDecode> f0;
    HRESULT hr = CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                  IID_PPV_ARGS(&fac));
    const std::wstring wpath(path.begin(), path.end());
    if (SUCCEEDED(hr)) {
        hr = fac->CreateDecoderFromFilename(wpath.c_str(), nullptr, GENERIC_READ,
                                            WICDecodeMetadataCacheOnDemand, &dec);
    }
    if (SUCCEEDED(hr)) hr = dec->GetFrame(0, &f0);
    if (FAILED(hr)) return refuse(Hr("WIC cannot decode it", hr));
    GUID container{};
    dec->GetContainerFormat(&container);
    const bool tiff = container == GUID_ContainerFormatTiff;
    UINT w = 0, h = 0;
    f0->GetSize(&w, &h);
    std::string ext = std::filesystem::path(path).extension().string();
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    // ---- WHAT: colour, by the entry's word or by the pixels ------------------------------------
    WICPixelFormatGUID pf{};
    f0->GetPixelFormat(&pf);
    UINT chan = 0, bpp = 0;
    WICPixelFormatNumericRepresentation rep = WICPixelFormatNumericRepresentationUnspecified;
    Com<IWICComponentInfo> ci;
    Com<IWICPixelFormatInfo2> pi;
    if (SUCCEEDED(fac->CreateComponentInfo(pf, &ci)) && SUCCEEDED(ci.As(&pi))) {
        pi->GetChannelCount(&chan);
        pi->GetBitsPerPixel(&bpp);
        pi->GetNumericRepresentation(&rep);
    }
    const std::string said = (declared && declared->valueUnit) ? declared->valueUnit : "";
    const bool colour = said.empty() ? ((chan == 3 || chan == 4) && bpp == 8 * chan)
                                     : said == "sRGB byte";
    const bool height = said.empty() ? (chan == 1 && (bpp == 16 || rep == WICPixelFormatNumericRepresentationFloat))
                                     : !colour;
    if (height && (chan != 1 || (bpp != 16 && !(bpp == 32 && rep == WICPixelFormatNumericRepresentationFloat)))) {
        return refuse("kind height, but " + std::to_string(chan) + " channel(s) of " +
                      std::to_string(bpp) + " bits: a height is one channel of 32-bit float or 16-bit integer");
    }
    L->m_height = height;
    L->m_bits = height ? bpp : 32;   // a colour window is the converter's RGBA8
    if (!colour && !height) {
        return refuse(std::to_string(chan) + " channels in " + std::to_string(bpp) +
                      " bits is neither colour (8-bit, 3 or 4 channels) nor height (one channel "
                      "of 16-bit or float); an entry that says kind colour paints it as colour");
    }

    // ---- WHERE: the file's tags, else its world file; the CRS from its GeoKeys, else its .prj,
    // else the entry's -----------------------------------------------------------------------------
    uint64_t fnv = own;
    GeoRef g;
    g.provenance = CrsProvenance::Embedded;
    g.width = w;
    g.height = h;
    g.centers = true;
    bool affine = false;
    int epsg = 0, vert = 0, vunit = 0;
    std::string from, side;
    Com<IWICMetadataQueryReader> q;
    if (tiff && SUCCEEDED(f0->GetMetadataQueryReader(&q))) {
        const std::vector<double> ps = QueryDoubles(q.Get(), L"/ifd/{ushort=33550}");
        const std::vector<double> tp = QueryDoubles(q.Get(), L"/ifd/{ushort=33922}");
        const std::vector<double> mt = QueryDoubles(q.Get(), L"/ifd/{ushort=34264}");
        const std::vector<uint16_t> keys = QueryShorts(q.Get(), L"/ifd/{ushort=34735}");
        int model = 0, raster = 1, geo = 0, proj = 0;
        for (size_t i = 4; keys.size() >= 4 && i + 3 < keys.size() && (i - 4) / 4 < keys[3]; i += 4) {
            if (keys[i + 1] != 0) continue;   // held in another tag: none of the six read here
            if (keys[i] == 1024) model = keys[i + 3];
            if (keys[i] == 1025) raster = keys[i + 3];
            if (keys[i] == 2048) geo = keys[i + 3];
            if (keys[i] == 3072) proj = keys[i + 3];
            if (keys[i] == 4096) vert = keys[i + 3];
            if (keys[i] == 4099) vunit = keys[i + 3];
        }
        const std::vector<uint16_t> fmt = QueryShorts(q.Get(), L"/ifd/{ushort=339}");
        L->m_signed = !fmt.empty() && fmt[0] == 2;   // SampleFormat: 2 is a signed integer
        if (!keys.empty()) {
            epsg = model == 2 ? geo : (proj ? proj : geo);
            if (epsg == 0 || epsg == 32767) {
                return refuse("its GeoKeys name no EPSG code (model type " +
                              std::to_string(model) + "; a user-defined CRS is not read)");
            }
            from = "its GeoKeys";
        }
        if (mt.size() >= 16) {
            if (mt[1] != 0.0 || mt[4] != 0.0) return refuse("its ModelTransformation rotates or shears it");
            g.scaleX = mt[0];
            g.originX = mt[3];
            g.scaleY = mt[5];
            g.originY = mt[7];
            affine = true;
        } else if (ps.size() >= 2 && tp.size() >= 6) {
            g.scaleX = ps[0];
            g.scaleY = -ps[1];   // GeoTIFF: a positive pixel height, rows running south
            g.originX = tp[3] - tp[0] * g.scaleX;
            g.originY = tp[4] - tp[1] * g.scaleY;
            affine = true;
        }
        if (affine && raster == 2) {   // PixelIsPoint: raster (0, 0) is texel 0's CENTRE
            g.originX -= 0.5 * g.scaleX;
            g.originY -= 0.5 * g.scaleY;
        }
        const std::string nd = QueryText(q.Get(), L"/ifd/{ushort=42113}");
        if (!nd.empty()) {
            g.hasNoData = true;
            g.noData = std::strtod(nd.c_str(), nullptr);   // "nan" reads as NaN, which IsNoData knows
        }
        // A FOURTH BAND is alpha only where ExtraSamples (338) says 1 or 2.
        const std::vector<uint16_t> extra = QueryShorts(q.Get(), L"/ifd/{ushort=338}");
        L->m_dataBand = chan == 4 && (extra.empty() || (extra[0] != 1 && extra[0] != 2));
    }
    std::vector<uint8_t> sb;
    if (!affine) {
        const char* sc = ext == ".png" ? ".pgw" : (ext == ".jpg" || ext == ".jpeg") ? ".jgw" : ".tfw";
        if (!Sidecar(path, {sc, ".wld"}, side, sb)) {
            return refuse(std::string("no georeference: no GeoTIFF tags, and no world file (") +
                          sc + " or .wld) beside it");
        }
        Fnv(fnv, sb.data(), sb.size());
        const std::string t(sb.begin(), sb.end());
        double v[6];
        if (sscanf_s(t.c_str(), "%lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3], &v[4],
                     &v[5]) != 6) {
            return refuse(side + " is not six numbers");
        }
        if (v[1] != 0.0 || v[2] != 0.0) return refuse(side + " rotates the raster");
        g.scaleX = v[0];
        g.scaleY = v[3];
        g.originX = v[4] - 0.5 * v[0];   // (C, F) names texel 0's CENTRE
        g.originY = v[5] - 0.5 * v[3];
    }
    if (epsg == 0 && Sidecar(path, {".prj"}, side, sb)) {
        Fnv(fnv, sb.data(), sb.size());
        epsg = PrjEpsg(std::string(sb.begin(), sb.end()));
        if (epsg == 0) return refuse(side + " names no EPSG code (no AUTHORITY or ID)");
        from = side;
    }
    const int entryEpsg = declared ? declared->epsg : 0;
    if (epsg == 0) {
        if (entryEpsg <= 0) return refuse("it carries no CRS, and the entry declares none (crs)");
        epsg = entryEpsg;
        from = "the entry's crs";
        g.provenance = CrsProvenance::Declared;
        Fnv(fnv, &epsg, sizeof(epsg));
    } else if (entryEpsg > 0 && entryEpsg != epsg) {
        Log("[raster] %s: the file's own EPSG:%d (%s) stands; the entry's EPSG:%d is not read",
            path.c_str(), epsg, from.c_str(), entryEpsg);
    }
    int zone = 0;
    bool south = false;
    std::string why;
    if (!CrsOfEpsg(epsg, g.kind, zone, south, &why)) return refuse(why);
    if (epsg == 4269 || epsg == 6318) {
        Log("[raster] %s: EPSG:%d is NAD83 geographic, read as WGS84 (they differ by about 1 m)",
            path.c_str(), epsg);
    }
    g.epsg = epsg;
    g.linearUnit = g.kind == CrsKind::Geographic ? "deg" : "m";
    g.valueUnit = "sRGB byte";
    if (!(g.scaleX > 0.0) || g.scaleY == 0.0) {
        return refuse("its pixel is not a positive width by a non-zero height");
    }
    if (height) {   // ---- A HEIGHT'S UNIT AND DATUM, as the file names them ("" where it does not)
        const char* v = "";
        for (const Vertical& k : kVertical) v = k.epsg == vert ? k.unit : v;
        if (!*v && vert != 0 && vert != 32767) {
            return refuse("its vertical CRS EPSG:" + std::to_string(vert) +
                          " is not one this engine names (5703, 6360, 8228, 5773, 3855, 5714, 4979)");
        }
        const char* u = vunit == 9001 ? "m" : vunit == 9002 ? "ft" : vunit == 9003 ? "ftUS" : "";
        if (!*u && vunit != 0) return refuse("its vertical unit (GeoKey 4099 = " + std::to_string(vunit) + ") is not m, ft or US ft");
        L->m_unit = *u ? std::string(u) + (std::strchr(v, ' ') ? std::strchr(v, ' ') : "") : v;
        g.valueUnit = L->m_unit.c_str();
    }

    // ---- THE PIXELS: a converter to RGBA8 kept on the frame, or a height's frame as it is. A TIFF
    // decodes the window asked for; a PNG or a JPEG decodes from its first row every time, so it is
    // decoded once, in bands, to raw rows named by its content's hash, and read by a seek from there.
    Com<IWICFormatConverter> conv;
    if (!height) hr = fac->CreateFormatConverter(&conv);
    if (!height && SUCCEEDED(hr)) {
        hr = conv->Initialize(f0.Get(), GUID_WICPixelFormat32bppRGBA, WICBitmapDitherTypeNone,
                              nullptr, 0.0, WICBitmapPaletteTypeCustom);
    }
    if (FAILED(hr)) return refuse(Hr("WIC cannot convert its pixels to RGBA", hr));
    if (height) L->m_conv = f0;
    else L->m_conv = conv;
    std::string rows;
    if (!tiff) {
        rows = std::string(kSources) + "/" + Hex(own) + (height ? ".f32" : ".rgba");
        std::error_code ec;
        if (std::filesystem::file_size(rows, ec) != uint64_t(w) * h * 4 || ec) {
            std::filesystem::create_directories(kSources, ec);
            std::ofstream f(rows + ".tmp", std::ios::binary | std::ios::trunc);
            std::vector<uint8_t> band(size_t(w) * 256 * 4);
            for (UINT y = 0; y < h && f; y += 256) {
                const UINT n = (std::min)(256u, h - y);
                if (!L->Pixels(0, y, w, n, band.data())) return refuse("WIC cannot decode its rows");
                f.write(reinterpret_cast<const char*>(band.data()), std::streamsize(size_t(w) * n * 4));
            }
            f.close();
            if (!f || !MoveFileExA((rows + ".tmp").c_str(), rows.c_str(), MOVEFILE_REPLACE_EXISTING)) {
                return refuse("its rows cannot be written to " + rows);
            }
        }
        if (!L->m_rows.Open(rows, 0, w, h, 4)) return refuse("its rows cannot be read from " + rows);
        L->m_conv.Reset();
    }
    L->m_name = std::filesystem::path(path).stem().string();
    L->m_ref = g;
    char s[400];
    const std::string what = !height ? std::string("rgba8") + (L->m_dataBand ? " (band 4 data)" : "")
                             : std::string(bpp == 16 ? (L->m_signed ? "int16" : "uint16") : "float32") +
                                   " height (" + (L->m_unit.empty() ? "no unit or datum of its own" : L->m_unit) + ")";
    snprintf(s, sizeof(s), "%s %ux%u %s, %s from %s, fnv64 %016llx (r2)",
             tiff ? "geotiff" : ext.c_str() + 1, w, h, what.c_str(),
             g.Describe().c_str(), from.c_str(), static_cast<unsigned long long>(fnv));
    L->m_structure = s;
    Log("[raster] %s: its bytes' hash %s%s", path.c_str(), indexed ? "from the index" : "taken, and indexed",
        rows.empty() ? "; windows decoded from the file" : ("; decoded once to " + rows).c_str());
    return L;
}

std::vector<std::unique_ptr<FieldLoader>> OpenManifest(const std::string& path, std::string* why) {
    std::vector<std::unique_ptr<FieldLoader>> out;
    std::vector<uint8_t> text;
    if (!ReadAll(path, text) || text.empty()) {
        if (why) *why = "the manifest cannot be read";
        return out;
    }
    std::string err;
    const JsonValue v = JsonParser::Parse(std::string(text.begin(), text.end()), &err);
    const std::string crs = v.Str("crs");
    const size_t at = crs.find("EPSG:");
    const int epsg = at == std::string::npos ? 0 : atoi(crs.c_str() + at + 5);
    CrsKind kind = CrsKind::Unknown;
    int zone = 0;
    bool south = false;
    std::string w = "its crs '" + crs + "' names no EPSG code";
    const JsonValue* tiles = v.Get("tiles");
    if (!CrsOfEpsg(epsg, kind, zone, south, epsg ? &w : nullptr) || !tiles || tiles->arr.empty()) {
        if (why) *why = (tiles && !tiles->arr.empty()) ? w : "it names no tiles" + (err.empty() ? "" : " (" + err + ")");
        return out;
    }
    uint64_t mfnv = 1469598103934665603ull;
    Fnv(mfnv, text.data(), text.size());
    const std::string dir = path.substr(0, path.find_last_of("/\\") + 1);
    for (const JsonValue& t : tiles->arr) {
        const std::string file = dir + t.Str("file");
        const uint32_t px = static_cast<uint32_t>(t.Num("px", 0));
        const uint32_t nc = static_cast<uint32_t>(t.Num("channels", 3));
        const JsonValue* mips = t.Get("mips");
        const uint64_t off = (mips && !mips->arr.empty()) ? uint64_t(mips->arr[0].Num("offset", 0)) : 0;
        auto L = std::make_unique<ImageLoader>();
        uint64_t fnv = mfnv;
        const uint64_t own = ContentHash(file);
        Fnv(fnv, &own, sizeof(own));
        if (!px || (nc != 3 && nc != 4) || !own || !L->m_rows.Open(file, off, px, px, nc)) {
            Log("[raster] %s: its tile %s cannot be read as %u x %u x %u rows; the set goes on without it",
                path.c_str(), file.c_str(), px, px, nc);
            continue;
        }
        GeoRef& g = L->m_ref;
        g.provenance = CrsProvenance::Declared;
        g.epsg = epsg;
        g.kind = kind;
        g.width = g.height = px;
        g.centers = true;
        g.originX = t.Num("utm_e0", 0);
        g.scaleX = (t.Num("utm_e1", 0) - g.originX) / px;
        g.originY = t.Num("utm_n1", 0);
        g.scaleY = -(g.originY - t.Num("utm_n0", 0)) / px;
        g.linearUnit = kind == CrsKind::Geographic ? "deg" : "m";
        g.valueUnit = "sRGB byte";
        L->m_name = std::filesystem::path(file).stem().string();
        char s[400];
        snprintf(s, sizeof(s), "rows %ux%u x%u at %llu, %s from its manifest, fnv64 %016llx (r2)", px,
                 px, nc, static_cast<unsigned long long>(off), g.Describe().c_str(),
                 static_cast<unsigned long long>(fnv));
        L->m_structure = s;
        out.push_back(std::move(L));
    }
    if (out.empty() && why) *why = "none of its tiles can be read";
    return out;
}

void RegisterImageLoaders(LoaderRegistry& reg) {
    for (const char* e : {"tif", "tiff", "png", "jpg", "jpeg"}) reg.Register(e, OpenImageFile);
}

}  // namespace ga
