// Shared basics: COM pointer, HRESULT checking, logging.
// Carried from vqview-inlet (namespace renamed); see GAMEPLAN.md section 2.
#pragma once

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <wrl/client.h>
#include <cstdint>
#include <cstdio>
#include <string>
#include <stdexcept>

namespace ga {

template <class T> using Com = Microsoft::WRL::ComPtr<T>;

inline void Log(const char* fmt, ...) {
    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    fputs(buf, stdout);
    fputc('\n', stdout);
    fflush(stdout);
    OutputDebugStringA(buf);
    OutputDebugStringA("\n");
}

inline std::string HrString(HRESULT hr) {
    char* msg = nullptr;
    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                       FORMAT_MESSAGE_IGNORE_INSERTS,
                   nullptr, static_cast<DWORD>(hr), 0, reinterpret_cast<char*>(&msg), 0, nullptr);
    std::string s = msg ? msg : "unknown";
    if (msg) LocalFree(msg);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    char buf[64];
    snprintf(buf, sizeof(buf), " (0x%08X)", static_cast<unsigned>(hr));
    return s + buf;
}

// Throws rather than returning a code. Every call site here is initialisation or a command-list
// op; there is no useful partial-failure recovery, and a thrown message with the call text beats
// a silent zeroed pointer that faults 200 lines later.
#define GA_CHECK(expr)                                                                  \
    do {                                                                                \
        HRESULT _hr = (expr);                                                           \
        if (FAILED(_hr)) {                                                              \
            throw std::runtime_error(std::string(#expr) + " failed: " + ga::HrString(_hr)); \
        }                                                                               \
    } while (0)

template <class T> constexpr T AlignUp(T v, T a) { return (v + a - 1) & ~(a - 1); }

// Half -> float, for CPU-side verification readbacks of RGBA16F/R16F targets.
inline float HalfToFloat(uint16_t h) {
    const uint32_t s = static_cast<uint32_t>(h & 0x8000) << 16;
    uint32_t e = (h >> 10) & 0x1F;
    uint32_t m = h & 0x3FF;
    uint32_t f;
    if (e == 0) {
        if (m == 0) {
            f = s;
        } else {
            e = 113;
            while (!(m & 0x400)) { m <<= 1; --e; }
            m &= 0x3FF;
            f = s | (e << 23) | (m << 13);
        }
    } else if (e == 31) {
        f = s | 0x7F800000u | (m << 13);
    } else {
        f = s | ((e + 112) << 23) | (m << 13);
    }
    float out;
    memcpy(&out, &f, 4);
    return out;
}

}  // namespace ga
