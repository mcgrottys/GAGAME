#include "core/ImageLoader.h"

#include "core/Common.h"

#include <windows.h>
#include <wincodec.h>

#include <cctype>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <vector>

namespace ga {

namespace {

thread_local std::string tWhy;

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

// One frame, straight RGBA8, through WIC's converter.
bool DecodeRgba(IWICImagingFactory* fac, IWICBitmapFrameDecode* frame, uint32_t w, uint32_t h,
                std::vector<uint8_t>& out, std::string& why) {
    const unsigned long long bytes = 4ull * w * h;
    if (bytes > UINT_MAX) {
        why = "it decodes to more than 4 GB, and this slice decodes a file whole";
        return false;
    }
    Com<IWICFormatConverter> conv;
    HRESULT hr = fac->CreateFormatConverter(&conv);
    if (SUCCEEDED(hr)) {
        hr = conv->Initialize(frame, GUID_WICPixelFormat32bppRGBA, WICBitmapDitherTypeNone,
                              nullptr, 0.0, WICBitmapPaletteTypeCustom);
    }
    if (SUCCEEDED(hr)) {
        out.resize(static_cast<size_t>(bytes));
        hr = conv->CopyPixels(nullptr, 4 * w, static_cast<UINT>(bytes), out.data());
    }
    if (FAILED(hr)) why = Hr("WIC cannot decode its pixels", hr);
    return SUCCEEDED(hr);
}

class ImageLoader : public FieldLoader {
public:
    const char* Name() const override { return m_name.c_str(); }
    const char* Structure() const override { return m_structure.c_str(); }
    const GeoRef& Ref() const override { return m_ref; }
    uint8_t GradeSig() const override { return kG0; }
    uint32_t Channels() const override { return 4; }
    // RGBA as 0..255 floats; a texel the file calls nodata, or gives alpha 0, stays all zero.
    bool LoadTile(uint32_t tx, uint32_t ty, uint32_t tw, uint32_t th, TilePayload& out) override {
        out.width = tw;
        out.height = th;
        out.channels = 4;
        out.data.assign(size_t(tw) * th * 4, 0.0f);
        uint32_t real = 0;
        for (uint32_t r = 0; r < th && ty * th + r < m_ref.height; ++r) {
            for (uint32_t c = 0; c < tw && tx * tw + c < m_ref.width; ++c) {
                const uint8_t* p = &m_px[(size_t(ty * th + r) * m_ref.width + tx * tw + c) * 4];
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

    std::string m_name, m_structure;
    GeoRef m_ref;
    std::vector<uint8_t> m_px;   // RGBA8, straight alpha, row 0 = the file's first row
};

}  // namespace

const std::string& ImageLoaderWhy() { return tWhy; }

bool CrsOfEpsg(int epsg, CrsKind& kind, int& zone, bool& south, std::string* why) {
    zone = 0;
    south = false;
    kind = CrsKind::Unknown;
    if (epsg == 4326) kind = CrsKind::Geographic;
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
               " is not a projection this engine evaluates (4326, 3857, UTM 32601-32660, "
               "32701-32760, 26901-26923, 6330-6348)";
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
    std::vector<uint8_t> bytes;
    if (!ReadAll(path, bytes) || bytes.empty()) return refuse("the file cannot be read");
    if (bytes.size() > UINT_MAX) return refuse("the file is over 4 GB, and this slice reads a file whole");
    static thread_local const HRESULT co = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    (void)co;   // RPC_E_CHANGED_MODE: the thread's COM is someone else's, and WIC still works
    Com<IWICImagingFactory> fac;
    Com<IWICStream> stream;
    Com<IWICBitmapDecoder> dec;
    Com<IWICBitmapFrameDecode> f0;
    HRESULT hr = CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                  IID_PPV_ARGS(&fac));
    if (SUCCEEDED(hr)) hr = fac->CreateStream(&stream);
    if (SUCCEEDED(hr)) hr = stream->InitializeFromMemory(bytes.data(), DWORD(bytes.size()));
    if (SUCCEEDED(hr)) {
        hr = fac->CreateDecoderFromStream(stream.Get(), nullptr, WICDecodeMetadataCacheOnDemand,
                                          &dec);
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
    if (height) {
        return refuse("kind height (" + std::to_string(chan) + " channel of " +
                      std::to_string(bpp) + " bits): no stack takes a height from a file yet, "
                      "and this slice paints colour");
    }
    if (!colour) {
        return refuse(std::to_string(chan) + " channels in " + std::to_string(bpp) +
                      " bits is neither colour (8-bit, 3 or 4 channels) nor height (one channel "
                      "of 16-bit or float); an entry that says kind colour paints it as colour");
    }

    // ---- WHERE: the file's tags, else its world file; the CRS from its GeoKeys, else its .prj,
    // else the entry's -----------------------------------------------------------------------------
    uint64_t fnv = 1469598103934665603ull;
    Fnv(fnv, bytes.data(), bytes.size());
    GeoRef g;
    g.provenance = CrsProvenance::Embedded;
    g.width = w;
    g.height = h;
    g.centers = true;
    bool affine = false;
    int epsg = 0;
    std::string from, side;
    Com<IWICMetadataQueryReader> q;
    if (tiff && SUCCEEDED(f0->GetMetadataQueryReader(&q))) {
        const std::vector<double> ps = QueryDoubles(q.Get(), L"/ifd/{ushort=33550}");
        const std::vector<double> tp = QueryDoubles(q.Get(), L"/ifd/{ushort=33922}");
        const std::vector<double> mt = QueryDoubles(q.Get(), L"/ifd/{ushort=34264}");
        const std::vector<uint16_t> keys = QueryShorts(q.Get(), L"/ifd/{ushort=34735}");
        int model = 0, raster = 1, geo = 0, proj = 0;
        for (size_t i = 4; keys.size() >= 4 && i + 3 < keys.size() && (i - 4) / 4 < keys[3]; i += 4) {
            if (keys[i + 1] != 0) continue;   // held in another tag: none of the four read here
            if (keys[i] == 1024) model = keys[i + 3];
            if (keys[i] == 1025) raster = keys[i + 3];
            if (keys[i] == 2048) geo = keys[i + 3];
            if (keys[i] == 3072) proj = keys[i + 3];
        }
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
    }
    std::vector<uint8_t> sb;
    if (!affine) {
        const char* own = ext == ".png" ? ".pgw" : (ext == ".jpg" || ext == ".jpeg") ? ".jgw" : ".tfw";
        if (!Sidecar(path, {own, ".wld"}, side, sb)) {
            return refuse(std::string("no georeference: no GeoTIFF tags, and no world file (") +
                          own + " or .wld) beside it");
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
    g.epsg = epsg;
    g.linearUnit = g.kind == CrsKind::Geographic ? "deg" : "m";
    g.valueUnit = "sRGB byte";
    if (!(g.scaleX > 0.0) || g.scaleY == 0.0) {
        return refuse("its pixel is not a positive width by a non-zero height");
    }

    auto L = std::make_unique<ImageLoader>();
    if (!DecodeRgba(fac.Get(), f0.Get(), w, h, L->m_px, why)) return refuse(why);
    L->m_name = std::filesystem::path(path).stem().string();
    L->m_ref = g;
    char s[320];
    snprintf(s, sizeof(s), "%s %ux%u rgba8, %s from %s, fnv64 %016llx (r1)",
             tiff ? "geotiff" : ext.c_str() + 1, w, h, g.Describe().c_str(), from.c_str(),
             static_cast<unsigned long long>(fnv));
    L->m_structure = s;
    return L;
}

void RegisterImageLoaders(LoaderRegistry& reg) {
    for (const char* e : {"tif", "tiff", "png", "jpg", "jpeg"}) reg.Register(e, OpenImageFile);
}

}  // namespace ga
