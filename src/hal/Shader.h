// Runtime HLSL compilation through DXC.
//
// Compiling at runtime rather than baking .cso files at build time is a deliberate iteration
// choice: it is what makes hot reload possible (press R in the viewer). The cost is a dependency
// on dxcompiler.dll sitting next to the exe, which CMake copies out of the Windows SDK.
#pragma once

#include "core/Common.h"

#include <dxcapi.h>
#include <vector>
#include <string>

namespace ga {

struct ShaderBlob {
    Com<IDxcBlob> blob;
    const void* Data() const { return blob ? blob->GetBufferPointer() : nullptr; }
    size_t Size() const { return blob ? blob->GetBufferSize() : 0; }
    bool Valid() const { return blob && blob->GetBufferSize() > 0; }
};

class ShaderCompiler {
public:
    void Init();

    // Returns an invalid blob and logs the DXC diagnostics on failure, rather than throwing.
    // A failed hot reload must not take the viewer down; the caller keeps the previous PSO.
    ShaderBlob Compile(const std::wstring& path, const wchar_t* entry, const wchar_t* target,
                       const std::vector<std::wstring>& defines = {});

private:
    Com<IDxcUtils> m_utils;
    Com<IDxcCompiler3> m_compiler;
    Com<IDxcIncludeHandler> m_includes;
};

}  // namespace ga
