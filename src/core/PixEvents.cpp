// M7k: programmatic GPU captures -- the hypervisor grows a GPU hand.
//
// The INSTALLED PIX ships WinPixGpuCapturer.dll; loading it before device creation and
// calling its PIXGpuCaptureNextFrames export writes a .wpix to disk with no PIX session
// attached and no NuGet dependency (the blog-documented programmatic-capture path, done
// by hand so the project stays dependency-free). Every pass is already named by its
// state-diagram node via PixScope, so a capture opens in PIX reading like the GA AST:
// sea/swe/churn/water.bank/globe, in order, with the banks inspectable per texel.
#include "PixEvents.h"

#include <windows.h>

#include <filesystem>
#include <string>

#include "Common.h"

namespace ga {

namespace {
HMODULE g_capturer = nullptr;
typedef HRESULT(WINAPI* PfnCaptureNextFrames)(PCWSTR, UINT32);
PfnCaptureNextFrames g_captureNextFrames = nullptr;
}  // namespace

bool PixLoadGpuCapturer() {
    if (g_capturer) return g_captureNextFrames != nullptr;
    // A PIX launch injects the capturer already; reuse it rather than double-loading.
    g_capturer = GetModuleHandleW(L"WinPixGpuCapturer.dll");
    if (g_capturer) {
        Log("[pix] capturer already resident (launched under PIX)");
    } else {
        namespace fs = std::filesystem;
        const wchar_t* pf = _wgetenv(L"ProgramW6432");
        const fs::path base = fs::path(pf ? pf : L"C:\\Program Files") / L"Microsoft PIX";
        std::wstring best;
        std::error_code ec;
        for (const auto& e : fs::directory_iterator(base, ec)) {
            if (!e.is_directory(ec)) continue;
            const std::wstring v = e.path().filename().wstring();
            if (best.empty() || v > best) best = v;   // release dirs (yymm.dd) sort fine
        }
        if (!best.empty()) {
            const fs::path dll = base / best / L"WinPixGpuCapturer.dll";
            g_capturer = LoadLibraryW(dll.c_str());
            if (g_capturer) Log("[pix] capturer loaded: %S", dll.c_str());
        }
        if (!g_capturer) {
            Log("[pix] WinPixGpuCapturer.dll not found -- install PIX (winget install "
                "microsoft.pix) or launch under PIX for GPU captures");
            return false;
        }
    }
    // Older capturers exported the pix3 name directly; current ones export the internal
    // "CaptureNextFrame" that pix3.h's PIXGpuCaptureNextFrames forwards to (same
    // (PCWSTR, UINT32) signature -- see microsoft/PixEvents).
    g_captureNextFrames = reinterpret_cast<PfnCaptureNextFrames>(
        GetProcAddress(g_capturer, "PIXGpuCaptureNextFrames"));
    if (!g_captureNextFrames) {
        g_captureNextFrames = reinterpret_cast<PfnCaptureNextFrames>(
            GetProcAddress(g_capturer, "CaptureNextFrame"));
    }
    if (!g_captureNextFrames) {
        Log("[pix] no capture export found (PIXGpuCaptureNextFrames/CaptureNextFrame)");
    }
    return g_captureNextFrames != nullptr;
}

bool PixGpuCaptureFrames(const wchar_t* wpixPath, uint32_t frameCount) {
    if (!g_captureNextFrames) return false;
    const HRESULT hr = g_captureNextFrames(wpixPath, frameCount);
    Log("[pix] GPU capture %s: %S (%u frame%s)", SUCCEEDED(hr) ? "armed" : "FAILED",
        wpixPath, frameCount, frameCount == 1 ? "" : "s");
    return SUCCEEDED(hr);
}

}  // namespace ga
