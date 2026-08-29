// PIX markers without a dependency on WinPixEventRuntime.dll.
//
// ID3D12GraphicsCommandList::BeginEvent/SetMarker with metadata 1 is the legacy ANSI-string
// encoding, which PIX (and RenderDoc) decode without any runtime DLL next to the exe. That is all
// M0/M1 need: named passes in a GPU capture. The upgrade path, when we want CPU-side timing events
// and PIXGpuCaptureNextFrames() programmatic captures, is the WinPixEventRuntime NuGet package --
// deliberately not pulled in yet to keep the project dependency-free.
#pragma once

#include <d3d12.h>
#include <cstring>

namespace ga {

inline void PixBegin(ID3D12GraphicsCommandList* cl, const char* name) {
    cl->BeginEvent(1u, name, static_cast<UINT>(std::strlen(name) + 1));
}
inline void PixEnd(ID3D12GraphicsCommandList* cl) { cl->EndEvent(); }
inline void PixMarker(ID3D12GraphicsCommandList* cl, const char* name) {
    cl->SetMarker(1u, name, static_cast<UINT>(std::strlen(name) + 1));
}

struct PixScope {
    ID3D12GraphicsCommandList* cl;
    PixScope(ID3D12GraphicsCommandList* c, const char* name) : cl(c) { PixBegin(c, name); }
    ~PixScope() { PixEnd(cl); }
    PixScope(const PixScope&) = delete;
    PixScope& operator=(const PixScope&) = delete;
};

}  // namespace ga
