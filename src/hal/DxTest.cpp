#include "hal/DxTest.h"

#include <d3d12shader.h>
#include <dxcapi.h>

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "core/Common.h"
#include "core/GaAst.h"
#include "hal/Shader.h"
#include "render/Renderer.h"   // M12 step 5b: SceneConstants and the fill the seam gates

namespace ga {
namespace {

std::string ReadFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    return std::string((std::istreambuf_iterator<char>(f)),
                       std::istreambuf_iterator<char>());
}

// ---- 1. the C++ side of CB parity: byte-count a CB struct from header text. Our CB style
// is rows: T name[4] / [8] / [16], plain scalars, and a nested ComposedSurfaceCb (none since
// M12 step 4g made the surface its own buffer; the case stays, with the prefix rule below).
// Anything this parser cannot read is a FAIL (the style is part of the contract).
// M9ax: a row of the C++ struct, by name and byte offset -- so the gate can hold the LAYOUT,
// not only the size. A same-size insertion in the middle of one side passed the size check
// and rotated every later row on the other (ChurnCbData vs ChurnCb, M9ar..M9ax: the churn
// read its current gain from the longitude; priors 22).
struct CbField {
    std::string name;
    int offset, size;
};

int StructBytes(const std::string& text, const std::string& name, bool& ok,
                const std::string& composedText, std::vector<CbField>* fields = nullptr,
                const std::string& prefix = "", int base = 0) {
    const std::string key = "struct " + name + " {";
    const size_t at = text.find(key);
    if (at == std::string::npos) {
        Log("[dxtest] FAIL cb parity: struct %s not found", name.c_str());
        ok = false;
        return 0;
    }
    int bytes = 0;
    std::istringstream ss(text.substr(at + key.size()));
    std::string line;
    while (std::getline(ss, line)) {
        const size_t cmt = line.find("//");
        if (cmt != std::string::npos) line = line.substr(0, cmt);
        if (line.find("};") != std::string::npos) return bytes;
        // strip leading whitespace
        size_t b = line.find_first_not_of(" \t");
        if (b == std::string::npos) continue;
        line = line.substr(b);
        if (line.empty()) continue;
        if (line.rfind("ComposedSurfaceCb", 0) == 0) {
            bool okN = true;
            // the member name ("cs") prefixes the nested rows: cs + u -> gCsU on the HLSL side
            std::string member = line.substr(17);
            const size_t semi = member.find(';');
            if (semi != std::string::npos) member = member.substr(0, semi);
            const size_t b0 = member.find_first_not_of(" \t");
            const size_t b1 = member.find_last_not_of(" \t");
            member = (b0 == std::string::npos) ? "" : member.substr(b0, b1 - b0 + 1);
            bytes += StructBytes(composedText, "ComposedSurfaceCb", okN, composedText, fields,
                                 prefix + member, base + bytes);
            if (!okN) ok = false;
            continue;
        }
        int unit = 0;
        std::string rest;
        if (line.rfind("float", 0) == 0 && !std::isalnum((unsigned char)line[5])) {
            unit = 4;
            rest = line.substr(5);
        } else if (line.rfind("uint32_t", 0) == 0) {
            unit = 4;
            rest = line.substr(8);
        } else if (line.rfind("int32_t", 0) == 0) {
            unit = 4;
            rest = line.substr(7);
        } else {
            continue;   // not a data row (blank, access specifier, etc.)
        }
        // one or more declarators: name, name[4], name[16], comma-separated
        size_t pos = 0;
        while (pos < rest.size()) {
            const size_t br = rest.find('[', pos);
            const size_t comma = rest.find(',', pos);
            const size_t semi = rest.find(';', pos);
            const size_t end = (comma != std::string::npos && comma < semi) ? comma : semi;
            if (end == std::string::npos) break;
            const int sz = (br != std::string::npos && br < end)
                               ? unit * std::atoi(rest.c_str() + br + 1)
                               : unit;
            if (fields) {
                const size_t nEnd = (br != std::string::npos && br < end) ? br : end;
                std::string fname = rest.substr(pos, nEnd - pos);
                const size_t f0 = fname.find_first_not_of(" \t");
                const size_t f1 = fname.find_last_not_of(" \t");
                fname = (f0 == std::string::npos) ? "" : fname.substr(f0, f1 - f0 + 1);
                fields->push_back({prefix + fname, base + bytes, sz});
            }
            bytes += sz;
            if (end == semi) break;
            pos = end + 1;
        }
    }
    Log("[dxtest] FAIL cb parity: struct %s has no closing brace", name.c_str());
    ok = false;
    return bytes;
}

struct CbContract {
    const char* cppFile;
    const char* cppStruct;
    const char* hlslFile;
    const char* cbuffer;
    const wchar_t* entry;
    const wchar_t* target;
    // M12 step 4g: what the HLSL rows carry between the 'g' and the C++ name -- the surface's
    // gCsU is ComposedSurfaceCb::u, so its contract says "cs"; every other cbuffer's rows are
    // the struct's own names.
    const char* rowPrefix = "";
};

// The registered contracts. ADD A ROW HERE when a new layer grows a CB -- and the gate
// will hold both sides of it forever after.
const CbContract kCbs[] = {
    {"src/scene/WaterBankLayer.h", "BankCbData", "shaders/WaterBank.hlsl", "BankCb",
     L"CsBankFill", L"cs_6_0"},
    {"src/scene/GlobeLayer.h", "GlobeCbData", "shaders/Globe.hlsl", "GlobeCb", L"PsMain",
     L"ps_6_0"},
    {"src/scene/SeaLayer.h", "SeaCbData", "shaders/Sea.hlsl", "SeaCb", L"PsMain",
     L"ps_6_0"},
    {"src/scene/SeaLayer.h", "ChurnCbData", "shaders/SeaChurn.hlsl", "ChurnCb",
     L"CsChurnUpdate", L"cs_6_0"},
    {"src/sim/SweSolver.h", "SweCbData", "shaders/Swe.hlsl", "SweCb", L"CsSweHeight",
     L"cs_6_0"},
    // M12 step 4g: THE SURFACE'S OWN BUFFER (b2), declared once in Common.hlsli and read by
    // every shader on the shared layout; reflected through the globe's pixel stage, which
    // samples it. Where the four layers' cbuffers embedded these rows, this row holds them.
    {"src/compose/Compositor.h", "ComposedSurfaceCb", "shaders/Globe.hlsl", "SurfaceCb",
     L"PsMain", L"ps_6_0", "cs"},
};

// ================================================================================================
//  M12 step 5b: THE VIEWS SEAM'S STRICT HALF -- the scene constants, frozen.
//
//  The seam moved the first sixty lines of Renderer::RenderFrame into a function of its inputs
//  (Renderer::FillSceneConstants) so that a view which is not the one being recorded can have
//  its rows built. A pure move is a claim, and this is the instrument that holds it: the body
//  BELOW is that span transcribed from e19bcac, the commit before the seam, and it is never
//  called by the engine. The gate builds both at the six poses the M12 stills are rendered from
//  and memcmps the two structs. The day an expression drifts -- a transpose, a degree that was
//  a radian, a row that moved -- one of the 208 bytes stops matching and says where.
// ================================================================================================
void FillSceneConstantsFrozen(const SceneFill& f, SceneConstants& sc) {
    sc = SceneConstants{};
    const Camera& cam = *f.cam;
    const DirectX::XMMATRIX view = cam.ViewRelative();
    const DirectX::XMMATRIX proj =
        cam.Projection(static_cast<float>(f.width) / static_cast<float>(f.height));
    DirectX::XMStoreFloat4x4(reinterpret_cast<DirectX::XMFLOAT4X4*>(sc.viewProj),
                             DirectX::XMMatrixMultiply(view, proj));
    if (f.sunPlaced) {
        sc.sunDir[0] = f.sunDirTangent[0];
        sc.sunDir[1] = f.sunDirTangent[1];
        sc.sunDir[2] = f.sunDirTangent[2];
    } else {
        const float az = DirectX::XMConvertToRadians(f.sunAzimuthDeg);
        const float el = DirectX::XMConvertToRadians(f.sunElevationDeg);
        sc.sunDir[0] = std::cos(el) * std::sin(az);
        sc.sunDir[1] = std::sin(el);
        sc.sunDir[2] = std::cos(el) * std::cos(az);
    }
    for (int c = 0; c < 3; ++c) {
        sc.sigmaW[c] = f.sigmaW[c];
        sc.bscat[c] = f.bscat[c];
    }
    sc.params0[0] = f.timeSec;
    sc.params0[1] = f.heightScale;
    sc.params0[2] = f.patchWidthM;
    sc.params0[3] = f.patchHeightM;
    sc.params1[0] = 1.0f;
    sc.params1[1] = static_cast<float>(f.width) / static_cast<float>(f.height);
    sc.params1[2] = cam.nearZ;
    sc.params1[3] = f.exposure;
    sc.eyeRelWorld[0] = static_cast<float>(cam.px);
    sc.eyeRelWorld[1] = static_cast<float>(cam.py);
    sc.eyeRelWorld[2] = static_cast<float>(cam.pz);
    DirectX::XMFLOAT3 fwd, rgt, upv;
    cam.ViewBasis(fwd, rgt, upv);
    const DirectX::XMVECTOR vf = DirectX::XMLoadFloat3(&fwd);
    const DirectX::XMVECTOR vr = DirectX::XMLoadFloat3(&rgt);
    const DirectX::XMVECTOR vu = DirectX::XMLoadFloat3(&upv);
    const float tanH = std::tan(cam.fovY * 0.5f);
    const float aspect = static_cast<float>(f.width) / static_cast<float>(f.height);
    DirectX::XMStoreFloat3(reinterpret_cast<DirectX::XMFLOAT3*>(sc.camFwd), vf);
    DirectX::XMStoreFloat3(reinterpret_cast<DirectX::XMFLOAT3*>(sc.camRight),
                           DirectX::XMVectorScale(vr, tanH * aspect));
    DirectX::XMStoreFloat3(reinterpret_cast<DirectX::XMFLOAT3*>(sc.camUp),
                           DirectX::XMVectorScale(vu, tanH));
    sc.viewport[0] = static_cast<float>(f.width);
    sc.viewport[1] = static_cast<float>(f.height);
    sc.viewport[2] = 1.0f / static_cast<float>(f.width);
    sc.viewport[3] = 1.0f / static_cast<float>(f.height);
    sc.misc[0] = f.waterLevel;
    {
        const float r = DirectX::XMConvertToRadians(f.sunAngRadiusDeg);
        sc.misc[1] = std::cos(r * 1.15f);
        sc.misc[2] = std::cos(r * 0.85f);
    }
    // M13: the sky's table (the frozen fill grows with the live one, or the gate compares a
    // row that exists against one that does not).
    sc.skyLut[0] = (f.skyMsSrv == 0xFFFFFFFFu) ? -1.0f : static_cast<float>(f.skyMsSrv);
    sc.skyLut[1] = f.planetRadiusM;
    sc.skyLut[2] = f.eyeRadiusM;
    sc.skyLut[3] = (f.skyTransSrv == 0xFFFFFFFFu) ? -1.0f : static_cast<float>(f.skyTransSrv);
}

// The six poses the M12 gate renders (tools/stills.sh, plus gate_stills.sh's droste), built the
// way FrameLoop::Session builds them from --campos/--cam: SetFromCompass(east, alt, north, az,
// pitch). key7km is the SEA DEFAULT pose, not a globe one, because --globe-cam without --globe
// never moved the start camera -- the 5a finding, and the reason that still is the jetty view.
// `sunPlaced` alternates so both branches of the sun's rows are compared, not just the placed one.
struct ViewPose {
    const char* name;
    double e, alt, n;
    float az, pitch;
    bool sunPlaced;
};
const ViewPose kViewPoses[] = {
    {"helm", 120.0, 7.0, -10.0, 92.5f, -1.5f, true},
    {"helm_ebb", 120.0, 7.0, -10.0, 92.5f, -1.5f, false},
    {"bird", 380.0, 1500.0, 10.0, 272.0f, -88.0f, true},
    {"key7km", 522.0, 7.0, 72.0, 246.0f, -4.0f, false},
    {"globe", 0.0, 200000.0, 0.0, 0.0f, 0.0f, true},
    {"droste", 632.0, 71.0, 30.0, 55.0f, 8.0f, false},
};

bool ReflectBlob(IDxcUtils* utils, const ShaderBlob& blob,
                 Com<ID3D12ShaderReflection>& out) {
    DxcBuffer buf{blob.Data(), blob.Size(), 0u};
    return SUCCEEDED(utils->CreateReflection(&buf, IID_PPV_ARGS(&out)));
}

}  // namespace

bool RunDxSelfTest() {
    bool ok = true;
    ShaderCompiler sc;
    sc.Init();
    Com<IDxcUtils> utils;
    if (FAILED(DxcCreateInstance(CLSID_DxcUtils, IID_PPV_ARGS(&utils)))) {
        Log("[dxtest] FAIL: no IDxcUtils");
        return false;
    }
    const std::string composed = ReadFile("src/compose/Compositor.h");

    // ---- 1. CB parity: C++ header bytes vs DXC-reflected cbuffer size.
    for (const CbContract& c : kCbs) {
        bool okC = true;
        const int cpp = StructBytes(ReadFile(c.cppFile), c.cppStruct, okC, composed);
        std::wstring wpath(c.hlslFile, c.hlslFile + std::strlen(c.hlslFile));
        const ShaderBlob blob = sc.Compile(wpath, c.entry, c.target);
        Com<ID3D12ShaderReflection> refl;
        if (!blob.Valid() || !ReflectBlob(utils.Get(), blob, refl)) {
            Log("[dxtest] FAIL cb parity: %s (%S) did not compile/reflect", c.hlslFile,
                c.entry);
            ok = false;
            continue;
        }
        ID3D12ShaderReflectionConstantBuffer* cb =
            refl->GetConstantBufferByName(c.cbuffer);
        D3D12_SHADER_BUFFER_DESC bd{};
        if (!cb || FAILED(cb->GetDesc(&bd))) {
            Log("[dxtest] FAIL cb parity: cbuffer %s not found in %s", c.cbuffer,
                c.hlslFile);
            ok = false;
            continue;
        }
        if (!okC || cpp != static_cast<int>(bd.Size)) {
            Log("[dxtest] FAIL cb parity: %s::%s = %d B but %s cbuffer %s = %u B "
                "(a row added on one side only?)",
                c.cppFile, c.cppStruct, cpp, c.hlslFile, c.cbuffer, bd.Size);
            ok = false;
            continue;
        }
        // M9ax: THE LAYOUT, ROW BY ROW. Every reflected variable must start where a C++ row
        // of the same size and the same name starts (names compared without the HLSL 'g'
        // prefix, case-insensitively, under the contract's rowPrefix: the surface's cs + u ==
        // gCsU, as a nested ComposedSurfaceCb's member once did). Equal sizes with rotated
        // rows is exactly the failure this exists for.
        std::vector<CbField> rows;
        bool okR = true;
        StructBytes(ReadFile(c.cppFile), c.cppStruct, okR, composed, &rows, c.rowPrefix);
        auto normHlsl = [](std::string n) {
            if (n.size() > 1 && n[0] == 'g' && std::isupper((unsigned char)n[1])) n = n.substr(1);
            std::string o;
            for (const char ch : n) if (std::isalnum((unsigned char)ch)) o += (char)std::tolower((unsigned char)ch);
            return o;
        };
        auto normCpp = [](const std::string& n) {
            std::string o;
            for (const char ch : n) if (std::isalnum((unsigned char)ch)) o += (char)std::tolower((unsigned char)ch);
            return o;
        };
        for (UINT vi = 0; vi < bd.Variables; ++vi) {
            ID3D12ShaderReflectionVariable* var = cb->GetVariableByIndex(vi);
            D3D12_SHADER_VARIABLE_DESC vd{};
            if (!var || FAILED(var->GetDesc(&vd))) continue;
            const CbField* row = nullptr;
            for (const CbField& r : rows) {
                if (r.offset == static_cast<int>(vd.StartOffset)) { row = &r; break; }
            }
            if (!row) {
                Log("[dxtest] FAIL cb layout: %s %s::%s at %u B starts inside a C++ row of "
                    "%s::%s -- rows rotated or split on one side",
                    c.hlslFile, c.cbuffer, vd.Name, vd.StartOffset, c.cppFile, c.cppStruct);
                ok = false;
                continue;
            }
            if (row->size != static_cast<int>(vd.Size) ||
                normHlsl(vd.Name) != normCpp(row->name)) {
                Log("[dxtest] FAIL cb layout: at %u B %s has %s (%u B) but %s has %s (%d B) "
                    "-- same bytes, different row: the size check cannot see this",
                    vd.StartOffset, c.cbuffer, vd.Name, vd.Size, c.cppStruct,
                    row->name.c_str(), row->size);
                ok = false;
            }
        }
    }

    // ---- 2. THE SAMPLER LAW: discover every compute entry, compile, reflect, and refuse
    // the proven trap (static sampler + unbounded bindless array in a compute stage).
    int kernels = 0, flagged = 0, meshEntries = 0;
    for (const auto& de : std::filesystem::directory_iterator("shaders")) {
        const std::string path = de.path().string();
        if (path.size() < 5 || path.substr(path.size() - 5) != ".hlsl") continue;
        const std::string text = ReadFile(path);
        // mesh shaders carry [numthreads] too -- their stage has its own rules (and
        // ComposedHeight sampling in the MESH stage empirically works; the proven trap
        // is compute), so the sampler scan skips entries marked [outputtopology]. (A hull
        // shader's [outputtopology] has no [numthreads], so it is neither kind.)
        size_t at = 0;
        while ((at = text.find("[numthreads(", at)) != std::string::npos) {
            const size_t back = text.rfind("[outputtopology", at);
            const bool mesh = back != std::string::npos && at - back < 200;
            const size_t v = text.find("void ", at);
            if (v == std::string::npos) break;
            size_t e = v + 5;
            while (e < text.size() &&
                   (std::isalnum((unsigned char)text[e]) || text[e] == '_')) {
                ++e;
            }
            const std::string entry = text.substr(v + 5, e - v - 5);
            at = e;
            std::wstring wentry(entry.begin(), entry.end());
            std::wstring wpath(path.begin(), path.end());
            if (mesh) {
                // ---- 2b. M10: THE MESH STAGE COMPILES. The entries the sampler scan
                // skips must compile and VALIDATE as ms_6_5. The CB-parity step above
                // compiles PsMain only, so a mesh shader that failed validation passed
                // this gate -- and at run time the globe's mesh pipeline silently fell back
                // (MEASURED, M10: a second SetMeshOutputCounts call site on an early-out
                // path; the root planet vanished from a rail while --selftest said PASS).
                ++meshEntries;
                if (!sc.Compile(wpath, wentry.c_str(), L"ms_6_5").Valid()) {
                    Log("[dxtest] FAIL mesh stage: %s::%s does not compile/validate as "
                        "ms_6_5 -- the globe's mesh pipeline would silently fall back",
                        path.c_str(), entry.c_str());
                    ok = false;
                }
                continue;
            }
            const ShaderBlob blob = sc.Compile(wpath, wentry.c_str(), L"cs_6_0");
            if (!blob.Valid()) {
                // entries needing defines or newer SM are outside this gate's scope
                continue;
            }
            Com<ID3D12ShaderReflection> refl;
            if (!ReflectBlob(utils.Get(), blob, refl)) continue;
            ++kernels;
            D3D12_SHADER_DESC sd{};
            refl->GetDesc(&sd);
            bool sampler = false, bindless = false;
            std::string bindlessName;
            for (UINT i = 0; i < sd.BoundResources; ++i) {
                D3D12_SHADER_INPUT_BIND_DESC rb{};
                refl->GetResourceBindingDesc(i, &rb);
                if (rb.Type == D3D_SIT_SAMPLER) sampler = true;
                if ((rb.Type == D3D_SIT_TEXTURE || rb.Type == D3D_SIT_UAV_RWTYPED) &&
                    rb.BindCount == 0) {   // unbounded = the bindless heap
                    bindless = true;
                    bindlessName = rb.Name ? rb.Name : "?";
                }
            }
            if (sampler && bindless) {
                Log("[dxtest] FAIL sampler law: compute entry %s::%s uses a sampler AND "
                    "the unbounded bindless array '%s' -- on this driver that sampling "
                    "silently returns ZERO outside the pixel stage (priors ledger #1); "
                    "use the manual-bilinear Load helpers",
                    path.c_str(), entry.c_str(), bindlessName.c_str());
                ok = false;
                ++flagged;
            }
        }
    }

    // ---- 3. AST anchors: every registered edge's code anchor names a file that exists.
    ast::RegisterKnownWaterEdges();
    std::set<std::string> names;
    for (const char* root : {"shaders", "src", "harvester", "tools", "proofs"}) {
        if (!std::filesystem::exists(root)) continue;
        for (const auto& de : std::filesystem::recursive_directory_iterator(root)) {
            if (de.is_regular_file()) names.insert(de.path().filename().string());
        }
    }
    int anchors = 0;
    for (const auto& e : ast::Edges()) {
        std::istringstream ss(e.code);
        std::string tok;
        while (ss >> tok) {
            const size_t dot = tok.rfind('.');
            if (dot == std::string::npos) continue;
            std::string ext = tok.substr(dot + 1);
            const size_t colon = ext.find(':');
            if (colon != std::string::npos) ext = ext.substr(0, colon);
            if (ext != "hlsl" && ext != "hlsli" && ext != "cpp" && ext != "h" &&
                ext != "py") {
                continue;
            }
            std::string fname = tok.substr(0, dot + 1) + ext;
            const size_t slash = fname.find_last_of("/\\");
            if (slash != std::string::npos) fname = fname.substr(slash + 1);
            ++anchors;
            if (!names.count(fname)) {
                Log("[dxtest] FAIL ast anchor: edge %s->%s '%s' cites missing file %s",
                    e.from, e.to, e.field, fname.c_str());
                ok = false;
            }
        }
    }

    // ---- 4. M12 step 5b: the views seam records the same bytes. The frozen fill above and
    // the renderer's live one, for the same camera at each of the six gate poses, memcmp EQUAL.
    {
        int equal = 0;
        for (size_t i = 0; i < std::size(kViewPoses); ++i) {
            const ViewPose& vp = kViewPoses[i];
            Camera cam;
            cam.SetFromCompass(vp.e, vp.alt, vp.n, vp.az, vp.pitch);
            cam.fovY = 55.0f * 3.14159265f / 180.0f;   // --fov's default, the session's own line
            cam.nearZ = 0.25f;
            // The anti-gravity FIELD the frame loop writes every frame: radial from the planet
            // centre at (0, -R, 0), which is world up to the last bit at the estuary and a real
            // direction at 200 km.
            const double R = 6371000.0;
            const double gy = cam.py + R;
            const double gl = std::sqrt(cam.px * cam.px + gy * gy + cam.pz * cam.pz);
            cam.upHint[0] = static_cast<float>(cam.px / gl);
            cam.upHint[1] = static_cast<float>(gy / gl);
            cam.upHint[2] = static_cast<float>(cam.pz / gl);

            SceneFill f;
            f.cam = &cam;
            f.timeSec = 3600.0f * static_cast<float>(i);
            f.width = 1600;
            f.height = 900;
            f.heightScale = 1.15f;
            f.patchWidthM = 1000.0f;
            f.patchHeightM = 1000.0f;
            f.exposure = 1.0f + 0.1f * static_cast<float>(i);
            f.sunPlaced = vp.sunPlaced;
            f.sunDirTangent[0] = 0.4f;
            f.sunDirTangent[1] = 0.8f;
            f.sunDirTangent[2] = -0.44721f;
            f.sunAzimuthDeg = 112.0f;
            f.sunElevationDeg = 26.0f;
            f.sunAngRadiusDeg = 0.26656f;
            f.sigmaW[0] = 0.330f; f.sigmaW[1] = 0.1238f; f.sigmaW[2] = 0.1463f;
            f.bscat[0] = 0.0173f; f.bscat[1] = 0.0233f; f.bscat[2] = 0.0248f;
            f.waterLevel = -0.37f + 0.2f * static_cast<float>(i);

            SceneConstants a{}, b{};
            FillSceneConstantsFrozen(f, a);
            Renderer::FillSceneConstants(f, b);
            if (std::memcmp(&a, &b, sizeof(SceneConstants)) == 0) {
                ++equal;
                continue;
            }
            const float* pa = reinterpret_cast<const float*>(&a);
            const float* pb = reinterpret_cast<const float*>(&b);
            int at = 0;
            while (at < static_cast<int>(sizeof(SceneConstants) / sizeof(float)) &&
                   std::memcmp(pa + at, pb + at, sizeof(float)) == 0) {
                ++at;
            }
            Log("[view] FAIL SceneConstants at %s: first difference at float %d (byte %d): "
                "frozen %.9g, live %.9g -- the fill is no longer a pure move",
                vp.name, at, at * 4, static_cast<double>(pa[at]), static_cast<double>(pb[at]));
            ok = false;
        }
        Log("[view] SceneConstants: EQUAL x%d (%zu B, the frozen e19bcac fill vs "
            "Renderer::FillSceneConstants, at helm / helm_ebb / bird / key7km / globe / droste)",
            equal, sizeof(SceneConstants));
    }

    if (ok) {
        Log("[dxtest] ---- PASS: cb parity x%zu (reflection vs header: size + row layout), sampler law "
            "(%d compute entries, %d flagged), %d mesh entries validate as ms_6_5, %d ast "
            "anchors resolve ----",
            std::size(kCbs), kernels, flagged, meshEntries, anchors);
    }
    return ok;
}

}  // namespace ga
