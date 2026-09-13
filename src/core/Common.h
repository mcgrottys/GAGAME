// Shared basics: COM pointer, HRESULT checking, logging.
// Carried from vqview-inlet (namespace renamed); see GAMEPLAN.md section 2.
#pragma once

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <wrl/client.h>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <mutex>
#include <string>
#include <stdexcept>

namespace ga {

template <class T> using Com = Microsoft::WRL::ComPtr<T>;

// FNV-1a over raw bytes: the fingerprint the M12 gates compare (constant-buffer rows, tables).
inline uint64_t Fnv1aBytes(const void* p, size_t n, uint64_t h = 14695981039346656037ull) {
    const uint8_t* b = static_cast<const uint8_t*>(p);
    for (size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 1099511628211ull;
    }
    return h;
}

// M12 step 4d: THE DISTANCE BETWEEN TWO DOUBLES IN ULPS -- what an instrument reports when a
// closed form and the form replacing it are compared and are not bitwise equal: the number of
// representable doubles between them (0 for two identical values, and for the two zeros;
// UINT64_MAX when either is a NaN). The gate's word EQUAL means the bit patterns agree; the
// ulp count is the size of the disagreement when they do not -- and an ulp count is unbounded
// where a value is a rounding of zero, so the callers say the absolute gap beside it.
inline uint64_t UlpDistance(double a, double b) {
    if (a != a || b != b) return UINT64_MAX;
    int64_t ia = 0, ib = 0;
    memcpy(&ia, &a, sizeof ia);
    memcpy(&ib, &b, sizeof ib);
    // Sign-magnitude onto one monotone integer line: a negative pattern is its magnitude, negated.
    const int64_t oa = ia < 0 ? INT64_MIN - ia : ia;
    const int64_t ob = ib < 0 ? INT64_MIN - ib : ib;
    return oa >= ob ? static_cast<uint64_t>(oa) - static_cast<uint64_t>(ob)
                    : static_cast<uint64_t>(ob) - static_cast<uint64_t>(oa);
}

// A tally over one instrumented site: how many values were compared, how many were EQUAL (bit
// for bit), how many differed only in the sign of a zero, and the farthest pair in ulps and in
// absolute terms. Verdict() is the line the gate reads.
struct UlpTally {
    uint64_t n = 0, equal = 0, zeroSign = 0, maxUlps = 0;
    double maxAbs = 0.0;
    // One pair; returns its ulp distance (0 = EQUAL, or the two zeros).
    uint64_t Add(double a, double b) {
        ++n;
        if (memcmp(&a, &b, sizeof a) == 0) {
            ++equal;
            return 0;
        }
        const uint64_t u = UlpDistance(a, b);
        if (u == 0) ++zeroSign;
        if (u > maxUlps) maxUlps = u;
        const double d = a > b ? a - b : b - a;
        if (d > maxAbs) maxAbs = d;
        return u;
    }
    bool AllEqual() const { return equal == n; }
    std::string Verdict() const {
        char b[192];
        if (n == 0) {
            snprintf(b, sizeof b, "unmeasured (0)");
        } else if (AllEqual()) {
            snprintf(b, sizeof b, "EQUAL (%llu of %llu)", static_cast<unsigned long long>(equal),
                     static_cast<unsigned long long>(n));
        } else {
            snprintf(b, sizeof b, "NOT EQUAL: max %llu ulps, |d| %.3g (%llu of %llu equal, %llu sign-of-zero)",
                     static_cast<unsigned long long>(maxUlps), maxAbs,
                     static_cast<unsigned long long>(equal), static_cast<unsigned long long>(n),
                     static_cast<unsigned long long>(zeroSign));
        }
        return b;
    }
};

// The word for one compared field of `count` doubles, tallied: "EQUAL" (bit for bit), "sign-of-
// zero", or the farthest component in ulps with the absolute gap beside it.
inline std::string UlpWord(const double* a, const double* b, int count, UlpTally& tally) {
    const uint64_t e0 = tally.equal;
    uint64_t worst = 0;
    double maxAbs = 0.0;
    for (int i = 0; i < count; ++i) {
        const uint64_t u = tally.Add(a[i], b[i]);
        if (u > worst) worst = u;
        const double d = a[i] > b[i] ? a[i] - b[i] : b[i] - a[i];
        if (d > maxAbs) maxAbs = d;
    }
    if (tally.equal - e0 == static_cast<uint64_t>(count)) return "EQUAL";
    char s[96];
    if (worst == 0) {
        snprintf(s, sizeof s, "sign-of-zero");
    } else {
        snprintf(s, sizeof s, "%llu ulps (|d| %.3g)", static_cast<unsigned long long>(worst), maxAbs);
    }
    return s;
}

inline void Log(const char* fmt, ...) {
    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    // Formatted first, printed under a lock. Every pool thread logs -- the tile-tree archive
    // lines come off loader threads -- and fputs of a whole line is not atomic, so lines could
    // interleave mid-line. Every gate in this project is read out of these logs.
    static std::mutex mx;
    std::lock_guard<std::mutex> lk(mx);
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
// M9j: float32 -> half, for composed pages. UploadTexture copies RAW BYTES, so handing a
// float32 buffer to an RGBA16F bank writes garbage that still reports as a successful upload.
//
// NAMED APART from Compositor.cpp's file-local FloatToHalf on purpose, and it is not a
// duplicate to be merged away: that one FLUSHES DENORMALS TO ZERO, which is fine for the
// heights it converts and fatal here. Divergence and vorticity run around 1e-5 /s, below
// half's smallest NORMAL (6.1e-5) -- flushed, the entire field would land on zero and the page
// would render as "irrotational everywhere", which is a lie with no symptom.
inline uint16_t F32ToHalf(float f) {
    uint32_t x;
    std::memcpy(&x, &f, 4);
    const uint32_t sign = (x >> 16) & 0x8000u;
    int32_t exp = int32_t((x >> 23) & 0xFF) - 127 + 15;
    uint32_t man = x & 0x7FFFFFu;
    if (((x >> 23) & 0xFF) == 0xFF) {          // inf / nan
        return uint16_t(sign | 0x7C00u | (man ? 0x200u : 0u));
    }
    if (exp >= 31) return uint16_t(sign | 0x7C00u);   // overflow -> inf
    if (exp <= 0) {                                   // subnormal or zero
        if (exp < -10) return uint16_t(sign);
        man |= 0x800000u;
        const uint32_t shift = uint32_t(14 - exp);
        uint32_t h = man >> shift;
        if ((man >> (shift - 1)) & 1u) ++h;           // round to nearest
        return uint16_t(sign | h);
    }
    uint16_t h = uint16_t(sign | uint32_t(exp << 10) | (man >> 13));
    if ((man & 0x1FFFu) > 0x1000u) ++h;               // round to nearest
    return h;
}

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
