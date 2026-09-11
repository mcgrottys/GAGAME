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
            continue;
        }
        // M9ax: THE LAYOUT, ROW BY ROW. Every reflected variable must start where a C++ row
        // of the same size and the same name starts (names compared without the HLSL 'g'
        // prefix, case-insensitively; nested ComposedSurfaceCb rows carry their member as a
        // prefix, cs + u == gCsU). Equal sizes with rotated rows is exactly the failure this
        // exists for.
        std::vector<CbField> rows;
        bool okR = true;
        StructBytes(ReadFile(c.cppFile), c.cppStruct, okR, composed, &rows);
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

    if (ok) {
        Log("[dxtest] ---- PASS: cb parity x%zu (reflection vs header: size + row layout), sampler law "
            "(%d compute entries, %d flagged), %d mesh entries validate as ms_6_5, %d ast "
            "anchors resolve ----",
            std::size(kCbs), kernels, flagged, meshEntries, anchors);
    }
    return ok;
}

}  // namespace ga
