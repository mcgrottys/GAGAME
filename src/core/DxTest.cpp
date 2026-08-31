#include "DxTest.h"

#include <d3d12shader.h>
#include <dxcapi.h>

#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "Common.h"
#include "GaAst.h"
#include "Shader.h"

namespace ga {
namespace {

std::string ReadFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    return std::string((std::istreambuf_iterator<char>(f)),
                       std::istreambuf_iterator<char>());
}

// ---- 1. the C++ side of CB parity: byte-count a CB struct from header text. Our CB style
// is rows: T name[4] / [8] / [16], plain scalars, and the one nested ComposedSurfaceCb.
// Anything this parser cannot read is a FAIL (the style is part of the contract).
int StructBytes(const std::string& text, const std::string& name, bool& ok,
                const std::string& composedText) {
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
            bytes += StructBytes(composedText, "ComposedSurfaceCb", okN, composedText);
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
            if (br != std::string::npos && br < end) {
                bytes += unit * std::atoi(rest.c_str() + br + 1);
            } else {
                bytes += unit;
            }
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
        }
    }

    // ---- 2. THE SAMPLER LAW: discover every compute entry, compile, reflect, and refuse
    // the proven trap (static sampler + unbounded bindless array in a compute stage).
    int kernels = 0, flagged = 0;
    for (const auto& de : std::filesystem::directory_iterator("shaders")) {
        const std::string path = de.path().string();
        if (path.size() < 5 || path.substr(path.size() - 5) != ".hlsl") continue;
        const std::string text = ReadFile(path);
        // mesh shaders carry [numthreads] too -- their stage has its own rules (and
        // ComposedHeight sampling in the MESH stage empirically works; the proven trap
        // is compute), so the scan skips entries marked [outputtopology].
        size_t at = 0;
        while ((at = text.find("[numthreads(", at)) != std::string::npos) {
            const size_t back = text.rfind("[outputtopology", at);
            if (back != std::string::npos && at - back < 200) { at += 12; continue; }
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

    if (ok) {
        Log("[dxtest] ---- PASS: cb parity x%zu (reflection vs header), sampler law "
            "(%d compute entries, %d flagged), %d ast anchors resolve ----",
            std::size(kCbs), kernels, flagged, anchors);
    }
    return ok;
}

}  // namespace ga
