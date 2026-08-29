#include "core/Shader.h"

namespace ga {

void ShaderCompiler::Init() {
    GA_CHECK(DxcCreateInstance(CLSID_DxcUtils, IID_PPV_ARGS(&m_utils)));
    GA_CHECK(DxcCreateInstance(CLSID_DxcCompiler, IID_PPV_ARGS(&m_compiler)));
    GA_CHECK(m_utils->CreateDefaultIncludeHandler(&m_includes));
}

ShaderBlob ShaderCompiler::Compile(const std::wstring& path, const wchar_t* entry,
                                   const wchar_t* target,
                                   const std::vector<std::wstring>& defines) {
    ShaderBlob out;

    Com<IDxcBlobEncoding> source;
    HRESULT hr = m_utils->LoadFile(path.c_str(), nullptr, &source);
    if (FAILED(hr)) {
        Log("[shader] cannot read %S : %s", path.c_str(), HrString(hr).c_str());
        return out;
    }

    // Directory of the shader, so #include "Common.hlsli" resolves next to the file.
    std::wstring dir = path;
    const size_t slash = dir.find_last_of(L"/\\");
    dir = (slash == std::wstring::npos) ? L"." : dir.substr(0, slash);

    std::vector<std::wstring> argStore = {
        path, L"-E", entry, L"-T", target,
        L"-I", dir,
        L"-Zi", L"-Qembed_debug",            // keep debug info in the DXIL for PIX
        L"-WX",                              // warnings are errors: a shader warning is a bug
        L"-HV", L"2021",
    };
    for (const auto& d : defines) { argStore.push_back(L"-D"); argStore.push_back(d); }

    std::vector<const wchar_t*> args;
    args.reserve(argStore.size());
    for (const auto& a : argStore) args.push_back(a.c_str());

    DxcBuffer buf{};
    buf.Ptr = source->GetBufferPointer();
    buf.Size = source->GetBufferSize();
    BOOL known = FALSE;
    UINT32 cp = 0;
    source->GetEncoding(&known, &cp);
    buf.Encoding = known ? cp : DXC_CP_ACP;

    Com<IDxcResult> result;
    hr = m_compiler->Compile(&buf, args.data(), static_cast<UINT32>(args.size()), m_includes.Get(),
                             IID_PPV_ARGS(&result));
    if (FAILED(hr) || !result) {
        Log("[shader] DXC invocation failed for %S", path.c_str());
        return out;
    }

    Com<IDxcBlobUtf8> errors;
    result->GetOutput(DXC_OUT_ERRORS, IID_PPV_ARGS(&errors), nullptr);
    if (errors && errors->GetStringLength() > 0) {
        Log("[shader] %S %S:\n%s", path.c_str(), entry, errors->GetStringPointer());
    }

    HRESULT status = S_OK;
    result->GetStatus(&status);
    if (FAILED(status)) {
        Log("[shader] COMPILE FAILED: %S %S", path.c_str(), entry);
        return out;
    }

    result->GetOutput(DXC_OUT_OBJECT, IID_PPV_ARGS(&out.blob), nullptr);
    if (!out.Valid()) Log("[shader] no object produced for %S %S", path.c_str(), entry);
    return out;
}

}  // namespace ga
