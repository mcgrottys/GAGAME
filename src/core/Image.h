// PNG load/save through WIC, so there is no third-party image dependency.
//
// 16-bit greyscale loads as R16_UNORM (bathymetry-grade precision); everything else normalises to
// RGBA8. SavePng backs --dump, which is how the renderer is verified without a human at the glass.
#pragma once

#include "core/Common.h"

#include <dxgiformat.h>
#include <vector>

namespace ga {

struct ImageData {
    std::vector<uint8_t> pixels;
    uint32_t width = 0, height = 0;
    uint32_t rowPitch = 0;
    DXGI_FORMAT format = DXGI_FORMAT_UNKNOWN;
    bool Valid() const { return width && height && !pixels.empty(); }
};

// Loads a PNG. 16-bit grey becomes R16_UNORM; anything else becomes R8G8B8A8_UNORM.
ImageData LoadPng(const std::wstring& path);

// Writes 8-bit RGBA to a PNG.
//
// byteCount is REQUIRED and must be the actual length of the buffer. Do not let this function infer
// rowPitch * height: D3D12's GetCopyableFootprints returns RowPitch * (height - 1) + rowBytes, which
// is SMALLER whenever the width is not already 256-byte aligned. Inferring the larger value made
// WIC read past the end of the allocation in vqview -- a hard crash at 1100x800 that never
// reproduced at 1280x640, because 1280 * 4 is already a multiple of 256.
bool SavePng(const std::wstring& path, const uint8_t* rgba, uint32_t width, uint32_t height,
             uint32_t rowPitch, size_t byteCount);

}  // namespace ga
