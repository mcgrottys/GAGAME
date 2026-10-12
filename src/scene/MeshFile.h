// ================================================================================================
//  MeshFile - a glTF binary (.glb) read into the one vertex form the vessel layer draws.
//
//  The engine has no mesh art and no input-assembler path, by decision (shaders/Vessel.hlsl's
//  banner). A vehicle's reference model is the exception Mark asked for: the Baja's body over the
//  physics' own wheels. So the reader is deliberately small -- the 12-byte header, the JSON chunk
//  (core/Json.h), the BIN chunk, accessors and buffer views for POSITION, NORMAL and the indices,
//  and each primitive's base colour -- and it emits ONE form: eight uints a vertex (position xyz
//  and normal xyz as float bits, an RGBA8 colour, a pad) and uint indices, the layout
//  Vessel.hlsl's VsMesh reads through the bindless uint buffers (gBufU). Textures, animations,
//  skins, node transforms and the KHR extensions are not read; a file that needs them draws
//  without them and says so in the log.
//
//  Materials can be left out BY NAME PREFIX: the reference model's own tyres, rims and brake
//  discs are not drawn because the physics draws its wheels where the struts put them, and the
//  glass is not drawn because there is no transparency pass (an opaque window would hide the
//  interior the model has). What was skipped is counted and reported.
// ================================================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

struct MeshData {
    std::vector<uint32_t> verts;    // 8 uints a vertex: pos.xyz, n.xyz (float bits), rgba8, 0
    std::vector<uint32_t> idx;      // triangle list
    double boundsMin[3] = {0.0, 0.0, 0.0};
    double boundsMax[3] = {0.0, 0.0, 0.0};
    uint32_t primitives = 0;        // drawn
    uint32_t skipped = 0;           // left out by name, or by an alpha below the opaque floor
    size_t VertexCount() const { return verts.size() / 8; }
    size_t TriangleCount() const { return idx.size() / 3; }
};

// `scale` takes the file's units to metres; `yawDeg` turns the model about +y so its front is +z
// (the body frame's bow); `skipPrefixes` are material-name prefixes left out; materials whose base
// colour alpha is below `opaqueFloor` are left out too. False with `why` when the file cannot be
// read as a glb with triangles.
bool LoadGlbMesh(const std::string& path, double scale, double yawDeg,
                 const std::vector<std::string>& skipPrefixes, double opaqueFloor, MeshData& out,
                 std::string* why);

}  // namespace ga
