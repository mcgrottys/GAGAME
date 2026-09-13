// ================================================================================================
//  FieldSet - the indirection between "a product wants to sample data at a world position" and
//  "there is a texture somewhere". Carried from vqview-inlet, whose header said, presciently:
//
//      "The moment this scales to the earth, or the moment field data lives in a tiled/reserved
//       resource, the mapping from world position to texel stops being a single affine transform."
//
//  That moment is this project. Every sample goes through a FieldDesc (worldToUv affine +
//  per-channel value scale/bias + SRV heap slot + algebraic layout), and the HLSL side samples
//  through SampleField(), so when the residency manager starts swapping tiles it rewrites
//  FieldDescs and the shaders do not change.
//
//  ALGEBRAIC LAYOUTS (the GA mapping, verbatim from vqview because it is exactly our plan):
//      G2 multivector   1 scalar + 2 vector + 1 bivector      = 4  -> ONE RGBA texel
//      G3 rotor         1 scalar + 3 bivector                 = 4  -> ONE RGBA texel (a quaternion)
//      G3 multivector   1 + 3 + 3 + 1                         = 8  -> two RGBA texels
//
//  ** TRAP. ** Hardware bilinear filtering interpolates CHANNELS, not geometric objects. A
//  componentwise lerp of two rotors is not a rotor. Rotor fields must be point-sampled and
//  renormalised (nlerp) or slerped by hand. sPointClamp exists for exactly this.
// ================================================================================================
#pragma once

#include "hal/Gpu.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace ga {

// Mirrored exactly in shaders/Common.hlsli. Keep the two in step.
enum class FieldLayout : uint32_t {
    Scalar = 0,        // R      : one quantity
    Vector2 = 1,       // RG     : (east, north)
    Rgba = 2,          // RGBA   : four independent quantities
    Multivector2 = 3,  // RGBA   : (scalar, e1, e2, e12) -- a full G2 multivector
    Rotor3 = 4,        // RGBA   : (scalar, e23, e31, e12) -- a G3 rotor, i.e. a quaternion
};

// 64 bytes, 16-byte aligned. Mirrored in HLSL as a StructuredBuffer element.
struct FieldDesc {
    float worldToUv[4] = {1, 1, 0, 0};   // uv = worldXZ * scale + bias
    float valueScale[4] = {1, 1, 1, 1};
    float valueBias[4] = {0, 0, 0, 0};
    uint32_t srvIndex = 0;
    uint32_t layout = static_cast<uint32_t>(FieldLayout::Scalar);
    uint32_t pad0 = 0, pad1 = 0;
};
static_assert(sizeof(FieldDesc) == 64, "FieldDesc must stay 16-byte aligned for HLSL");

// Loads named PNG fields from a directory, uploads them, and builds the FieldDesc table.
// Products look fields up by name and get back an index into that table.
class FieldSet {
public:
    void Init(Gpu& gpu, const std::wstring& dir, float patchWidthM, float patchHeightM);

    // M9aw: the PNG registration path (Add + LoadPng) is deleted -- it had no caller, and a
    // raster enters the engine as a TileTree leaf on the sparse addresses, never as a
    // committed texture from a PNG (AUDIT rows 4, 13).

    // Uploads the FieldDesc table to the GPU.
    //
    // Always creates at least a dummy row: root parameter 2 is a root SRV, and an UNBOUND root
    // descriptor is undefined behaviour, not a no-op -- vqview measured it as outright device
    // removal. M1 registers no fields yet, so the dummy row is what keeps the binding legal.
    void Finalize();

    uint32_t Find(const std::wstring& fileName) const;
    uint32_t Count() const { return static_cast<uint32_t>(m_descs.size()); }
    uint32_t TableSrv() const { return m_tableSrv; }
    // Root parameter 2 is a root SRV, which is bound by ADDRESS. This is that address.
    D3D12_GPU_VIRTUAL_ADDRESS TableGpuVa() const { return m_table.gpu; }
    bool Ready() const { return m_tableSrv != UINT32_MAX; }

    float PatchWidthM() const { return m_patchW; }
    float PatchHeightM() const { return m_patchH; }

private:
    Gpu* m_gpu = nullptr;
    std::wstring m_dir;
    float m_patchW = 1, m_patchH = 1;
    std::vector<FieldDesc> m_descs;
    std::vector<GpuTexture> m_textures;
    std::unordered_map<std::wstring, uint32_t> m_byName;
    GpuBuffer m_table;
    uint32_t m_tableSrv = UINT32_MAX;
};

}  // namespace ga
