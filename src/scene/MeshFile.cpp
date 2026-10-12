// MeshFile - the glb reader (see the header).
#include "scene/MeshFile.h"

#include "core/Common.h"
#include "core/Json.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

namespace ga {

namespace {

struct Accessor {
    const uint8_t* data = nullptr;
    size_t count = 0;
    size_t stride = 0;
    int componentType = 0;   // 5120 b, 5121 B, 5122 h, 5123 H, 5125 I, 5126 f
    int components = 0;
};

int ComponentsOf(const std::string& type) {
    if (type == "SCALAR") return 1;
    if (type == "VEC2") return 2;
    if (type == "VEC3") return 3;
    if (type == "VEC4") return 4;
    return 0;
}

size_t BytesOf(int componentType) {
    switch (componentType) {
        case 5120: case 5121: return 1;
        case 5122: case 5123: return 2;
        case 5125: case 5126: return 4;
        default: return 0;
    }
}

bool ReadAccessor(const JsonValue& doc, const std::vector<uint8_t>& bin, int index, Accessor& a) {
    const JsonValue* accs = doc.Get("accessors");
    const JsonValue* views = doc.Get("bufferViews");
    if (!accs || !views || index < 0 || size_t(index) >= accs->arr.size()) return false;
    const JsonValue& acc = accs->arr[size_t(index)];
    const int view = int(acc.Num("bufferView", -1));
    if (view < 0 || size_t(view) >= views->arr.size()) return false;
    const JsonValue& bv = views->arr[size_t(view)];
    a.componentType = int(acc.Num("componentType", 0));
    a.components = ComponentsOf(acc.Str("type"));
    a.count = size_t(acc.Num("count", 0));
    const size_t elem = BytesOf(a.componentType) * size_t(a.components);
    if (elem == 0 || a.count == 0) return false;
    const size_t stride = size_t(bv.Num("byteStride", 0));
    a.stride = stride ? stride : elem;
    const size_t off = size_t(bv.Num("byteOffset", 0)) + size_t(acc.Num("byteOffset", 0));
    if (off + (a.count - 1) * a.stride + elem > bin.size()) return false;
    a.data = bin.data() + off;
    return true;
}

uint32_t ReadIndex(const Accessor& a, size_t i) {
    const uint8_t* p = a.data + i * a.stride;
    switch (a.componentType) {
        case 5121: return *p;
        case 5123: { uint16_t v; std::memcpy(&v, p, 2); return v; }
        default: { uint32_t v; std::memcpy(&v, p, 4); return v; }
    }
}

uint32_t PackRgba8(const double c[4]) {
    auto b = [](double v) { return uint32_t((std::max)(0.0, (std::min)(1.0, v)) * 255.0 + 0.5); };
    return b(c[0]) | (b(c[1]) << 8) | (b(c[2]) << 16) | (b(c[3]) << 24);
}

uint32_t Bits(float f) {
    uint32_t u;
    std::memcpy(&u, &f, 4);
    return u;
}

}  // namespace

bool LoadGlbMesh(const std::string& path, double scale, double yawDeg,
                 const std::vector<std::string>& skipPrefixes, double opaqueFloor, MeshData& out,
                 std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    std::ifstream f(path, std::ios::binary);
    if (!f) return fail("cannot open " + path);
    uint32_t head[3];
    f.read(reinterpret_cast<char*>(head), 12);
    if (!f || head[0] != 0x46546C67u) return fail("not a glb (magic)");   // "glTF"
    uint32_t chunk[2];
    f.read(reinterpret_cast<char*>(chunk), 8);
    if (!f || chunk[1] != 0x4E4F534Au) return fail("first chunk is not JSON");
    std::string json(chunk[0], '\0');
    f.read(json.data(), chunk[0]);
    std::vector<uint8_t> bin;
    if (f.read(reinterpret_cast<char*>(chunk), 8) && chunk[1] == 0x004E4942u) {
        bin.resize(chunk[0]);
        f.read(reinterpret_cast<char*>(bin.data()), chunk[0]);
    }
    if (bin.empty()) return fail("no BIN chunk");
    std::string err;
    const JsonValue doc = JsonParser::Parse(json, &err);
    if (!err.empty()) return fail("JSON: " + err);
    const JsonValue* meshes = doc.Get("meshes");
    const JsonValue* mats = doc.Get("materials");
    if (!meshes || meshes->arr.empty()) return fail("no meshes");

    const double yaw = yawDeg * 3.14159265358979323846 / 180.0;
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    out = MeshData{};
    bool first = true;
    for (const JsonValue& mesh : meshes->arr) {
        const JsonValue* prims = mesh.Get("primitives");
        if (!prims) continue;
        for (const JsonValue& prim : prims->arr) {
            const int mode = int(prim.Num("mode", 4));
            if (mode != 4) { ++out.skipped; continue; }   // triangles only
            // The material: its name (for the skips) and its base colour.
            double col[4] = {0.8, 0.8, 0.8, 1.0};
            std::string name;
            const int mi = int(prim.Num("material", -1));
            if (mats && mi >= 0 && size_t(mi) < mats->arr.size()) {
                const JsonValue& m = mats->arr[size_t(mi)];
                name = m.Str("name");
                if (const JsonValue* pbr = m.Get("pbrMetallicRoughness")) {
                    if (const JsonValue* bc = pbr->Get("baseColorFactor")) {
                        for (size_t k = 0; k < 4 && k < bc->arr.size(); ++k) col[k] = bc->arr[k].number;
                    }
                }
            }
            bool skip = col[3] < opaqueFloor;
            for (const std::string& p : skipPrefixes) {
                if (!p.empty() && name.compare(0, p.size(), p) == 0) skip = true;
            }
            if (skip) { ++out.skipped; continue; }
            const JsonValue* attrs = prim.Get("attributes");
            if (!attrs) { ++out.skipped; continue; }
            Accessor pos, nrm, ind;
            if (!ReadAccessor(doc, bin, int(attrs->Num("POSITION", -1)), pos) || pos.componentType != 5126 ||
                pos.components != 3) {
                ++out.skipped;
                continue;
            }
            const bool hasN = ReadAccessor(doc, bin, int(attrs->Num("NORMAL", -1)), nrm) &&
                              nrm.componentType == 5126 && nrm.components == 3;
            const bool hasI = ReadAccessor(doc, bin, int(prim.Num("indices", -1)), ind) && ind.components == 1;
            const uint32_t base = static_cast<uint32_t>(out.VertexCount());
            const uint32_t rgba = PackRgba8(col);
            out.verts.reserve(out.verts.size() + pos.count * 8);
            for (size_t i = 0; i < pos.count; ++i) {
                float p[3], n[3] = {0.0f, 1.0f, 0.0f};
                std::memcpy(p, pos.data + i * pos.stride, 12);
                if (hasN && i < nrm.count) std::memcpy(n, nrm.data + i * nrm.stride, 12);
                // Scale to metres and turn about +y so the model's front is +z (the bow).
                const double x = p[0] * scale, y = p[1] * scale, z = p[2] * scale;
                const double xr = cy * x + sy * z, zr = -sy * x + cy * z;
                const double nx = cy * n[0] + sy * n[2], nz = -sy * n[0] + cy * n[2];
                const double v[3] = {xr, y, zr};
                for (int k = 0; k < 3; ++k) {
                    if (first) { out.boundsMin[k] = out.boundsMax[k] = v[k]; }
                    out.boundsMin[k] = (std::min)(out.boundsMin[k], v[k]);
                    out.boundsMax[k] = (std::max)(out.boundsMax[k], v[k]);
                }
                first = false;
                out.verts.push_back(Bits(float(xr)));
                out.verts.push_back(Bits(float(y)));
                out.verts.push_back(Bits(float(zr)));
                out.verts.push_back(Bits(float(nx)));
                out.verts.push_back(Bits(float(n[1])));
                out.verts.push_back(Bits(float(nz)));
                out.verts.push_back(rgba);
                out.verts.push_back(0u);
            }
            if (hasI) {
                const size_t tris = ind.count / 3;
                out.idx.reserve(out.idx.size() + tris * 3);
                for (size_t i = 0; i < tris * 3; ++i) out.idx.push_back(base + ReadIndex(ind, i));
            } else {
                for (size_t i = 0; i + 2 < pos.count; i += 3) {
                    out.idx.push_back(base + uint32_t(i));
                    out.idx.push_back(base + uint32_t(i + 1));
                    out.idx.push_back(base + uint32_t(i + 2));
                }
            }
            ++out.primitives;
        }
    }
    if (out.idx.empty()) return fail("no triangles drawn (every primitive skipped or unreadable)");
    Log("[mesh] %s: %u primitives drawn, %u left out (named skips, glass under alpha %.2f, non-triangles); "
        "%zu vertices, %zu triangles; bounds x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f m",
        path.c_str(), out.primitives, out.skipped, opaqueFloor, out.VertexCount(), out.TriangleCount(),
        out.boundsMin[0], out.boundsMax[0], out.boundsMin[1], out.boundsMax[1], out.boundsMin[2],
        out.boundsMax[2]);
    return true;
}

}  // namespace ga
