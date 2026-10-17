// ================================================================================================
//  FileText - the file and path helpers seven files each wrote for themselves (tools/codemap's
//  clone pass found the copies, token for token: four ReadFile, two ReadAll and DxTest's ReadFile;
//  three DirOf; two EndsWith). One spelling each, here.
// ================================================================================================
#pragma once

#include <cstring>
#include <fstream>
#include <iterator>
#include <string>

namespace ga {

// The whole file, byte for byte; "" when it cannot be opened.
inline std::string ReadFileText(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

// The folder a path names its file in: everything before the last '/' or '\', "." when none.
inline std::string DirOf(const std::string& path) {
    const size_t slash = path.find_last_of("/\\");
    return (slash == std::string::npos) ? std::string(".") : path.substr(0, slash);
}

inline bool EndsWith(const std::string& s, const char* tail) {
    const size_t n = std::strlen(tail);
    return s.size() >= n && s.compare(s.size() - n, n, tail) == 0;
}

}  // namespace ga
