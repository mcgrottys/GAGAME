#include "core/Image.h"

#include <wincodec.h>

namespace ga {

namespace {

class WicFactory {
public:
    static IWICImagingFactory* Get() {
        static WicFactory s;
        return s.m_factory.Get();
    }

private:
    WicFactory() {
        // Apartment-threaded is what WIC documentation assumes; RPC_E_CHANGED_MODE just means
        // someone already initialised COM differently on this thread, which is fine.
        const HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
        m_uninit = SUCCEEDED(hr);
        if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                    IID_PPV_ARGS(&m_factory)))) {
            Log("[image] WIC factory creation failed");
        }
    }
    ~WicFactory() {
        m_factory.Reset();
        if (m_uninit) CoUninitialize();
    }
    Com<IWICImagingFactory> m_factory;
    bool m_uninit = false;
};

}  // namespace

ImageData LoadPng(const std::wstring& path) {
    ImageData out;
    IWICImagingFactory* wic = WicFactory::Get();
    if (!wic) return out;

    Com<IWICBitmapDecoder> dec;
    HRESULT hr = wic->CreateDecoderFromFilename(path.c_str(), nullptr, GENERIC_READ,
                                                WICDecodeMetadataCacheOnDemand, &dec);
    if (FAILED(hr)) {
        Log("[image] cannot open %S : %s", path.c_str(), HrString(hr).c_str());
        return out;
    }
    Com<IWICBitmapFrameDecode> frame;
    if (FAILED(dec->GetFrame(0, &frame))) return out;

    UINT w = 0, h = 0;
    frame->GetSize(&w, &h);
    WICPixelFormatGUID srcFmt{};
    frame->GetPixelFormat(&srcFmt);

    // Only 16-bit greyscale is worth preserving at full precision; everything else this project
    // loads is 8-bit colour and gets normalised to RGBA8.
    const bool grey16 = (srcFmt == GUID_WICPixelFormat16bppGray);
    const WICPixelFormatGUID dstFmt = grey16 ? GUID_WICPixelFormat16bppGray
                                             : GUID_WICPixelFormat32bppRGBA;
    const uint32_t bpp = grey16 ? 2u : 4u;

    Com<IWICBitmapSource> src;
    if (srcFmt == dstFmt) {
        src = frame;
    } else {
        Com<IWICFormatConverter> conv;
        if (FAILED(wic->CreateFormatConverter(&conv))) return out;
        if (FAILED(conv->Initialize(frame.Get(), dstFmt, WICBitmapDitherTypeNone, nullptr, 0.0,
                                    WICBitmapPaletteTypeMedianCut))) {
            Log("[image] cannot convert %S to the target format", path.c_str());
            return out;
        }
        src = conv;
    }

    out.width = w;
    out.height = h;
    out.rowPitch = w * bpp;
    out.format = grey16 ? DXGI_FORMAT_R16_UNORM : DXGI_FORMAT_R8G8B8A8_UNORM;
    out.pixels.resize(static_cast<size_t>(out.rowPitch) * h);

    WICRect rc{0, 0, static_cast<INT>(w), static_cast<INT>(h)};
    hr = src->CopyPixels(&rc, out.rowPitch, static_cast<UINT>(out.pixels.size()),
                         out.pixels.data());
    if (FAILED(hr)) {
        Log("[image] CopyPixels failed for %S", path.c_str());
        out.pixels.clear();
        return out;
    }
    return out;
}

bool SavePng(const std::wstring& path, const uint8_t* rgba, uint32_t width, uint32_t height,
             uint32_t rowPitch, size_t byteCount) {
    // A readback buffer is RowPitch * (h - 1) + rowBytes, not RowPitch * h. Refuse rather than
    // hand WIC a length that walks off the end.
    const size_t needed = static_cast<size_t>(rowPitch) * (height - 1) + static_cast<size_t>(width) * 4;
    if (byteCount < needed) {
        Log("[image] refusing to save %S: buffer is %zu bytes, need %zu", path.c_str(), byteCount,
            needed);
        return false;
    }
    IWICImagingFactory* wic = WicFactory::Get();
    if (!wic) return false;

    Com<IWICStream> stream;
    if (FAILED(wic->CreateStream(&stream))) return false;
    if (FAILED(stream->InitializeFromFilename(path.c_str(), GENERIC_WRITE))) {
        Log("[image] cannot write %S", path.c_str());
        return false;
    }
    Com<IWICBitmapEncoder> enc;
    if (FAILED(wic->CreateEncoder(GUID_ContainerFormatPng, nullptr, &enc))) return false;
    if (FAILED(enc->Initialize(stream.Get(), WICBitmapEncoderNoCache))) return false;

    Com<IWICBitmapFrameEncode> frame;
    Com<IPropertyBag2> props;
    if (FAILED(enc->CreateNewFrame(&frame, &props))) return false;
    if (FAILED(frame->Initialize(props.Get()))) return false;
    if (FAILED(frame->SetSize(width, height))) return false;

    // ** THE TRAP (inherited knowledge from vqview). ** SetPixelFormat is [in, out]: WIC overwrites
    // it with the nearest format the encoder actually supports and returns S_OK. The PNG encoder
    // negotiates 32bppBGRA, so writing RGBA bytes straight through swaps red and blue -- which
    // looks like a renderer shading bug when in fact only the verification path is lying. Wrap the
    // pixels in an IWICBitmap explicitly tagged RGBA and let a converter reach whatever the
    // encoder negotiated.
    WICPixelFormatGUID want = GUID_WICPixelFormat32bppRGBA;
    if (FAILED(frame->SetPixelFormat(&want))) return false;

    Com<IWICBitmap> src;
    if (FAILED(wic->CreateBitmapFromMemory(width, height, GUID_WICPixelFormat32bppRGBA, rowPitch,
                                           static_cast<UINT>(byteCount),
                                           const_cast<uint8_t*>(rgba), &src))) {
        return false;
    }

    if (want == GUID_WICPixelFormat32bppRGBA) {
        if (FAILED(frame->WriteSource(src.Get(), nullptr))) return false;
    } else {
        Com<IWICFormatConverter> conv;
        if (FAILED(wic->CreateFormatConverter(&conv))) return false;
        if (FAILED(conv->Initialize(src.Get(), want, WICBitmapDitherTypeNone, nullptr, 0.0,
                                    WICBitmapPaletteTypeMedianCut))) {
            return false;
        }
        if (FAILED(frame->WriteSource(conv.Get(), nullptr))) return false;
    }

    if (FAILED(frame->Commit())) return false;
    if (FAILED(enc->Commit())) return false;
    return true;
}

}  // namespace ga
