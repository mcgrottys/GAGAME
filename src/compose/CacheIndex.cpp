#include "compose/CacheIndex.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace ga {

std::wstring CacheIndex::DefaultPath() {
    wchar_t* la = nullptr;
    size_t n = 0;
    if (_wdupenv_s(&la, &n, L"LOCALAPPDATA") != 0 || !la) return std::wstring();
    std::wstring p = std::wstring(la) + L"\\GAGAME\\storage-index.bin";
    free(la);
    return p;
}

namespace {

// StorageIndex.Save's stream (BinaryWriter, little-endian): fixed-width integers and doubles, a
// bool as one byte, a string as its UTF-8 bytes after a 7-bit varint length. The reader skips
// what the lens does not need and only ever moves forward; running off the end is a bad file.
struct Cursor {
    const uint8_t* p;
    const uint8_t* end;
    bool ok = true;
    bool Need(size_t n) {
        if (!ok || static_cast<size_t>(end - p) < n) ok = false;
        return ok;
    }
    template <class T> T Get() {
        T v{};
        if (Need(sizeof(T))) {
            memcpy(&v, p, sizeof(T));
            p += sizeof(T);
        }
        return v;
    }
    void Skip(size_t n) {
        if (Need(n)) p += n;
    }
    void SkipString() {
        uint32_t len = 0;
        for (int shift = 0; shift < 35; shift += 7) {
            const uint8_t b = Get<uint8_t>();
            len |= static_cast<uint32_t>(b & 0x7F) << shift;
            if (!(b & 0x80)) break;
        }
        Skip(len);
    }
};

}  // namespace

bool CacheIndex::Read(const std::wstring& path, std::string& why) {
    const auto t0 = std::chrono::steady_clock::now();
    rg.clear();
    folders = entries = texelsLit = 0;
    std::vector<uint8_t> file;
    {
        FILE* f = nullptr;
        if (path.empty() || _wfopen_s(&f, path.c_str(), L"rb") != 0 || !f) {
            why = "no file (run `harvest storage` to write it)";
            return false;
        }
        fseek(f, 0, SEEK_END);
        const long n = ftell(f);
        fseek(f, 0, SEEK_SET);
        file.resize(n > 0 ? static_cast<size_t>(n) : 0);
        const size_t got = file.empty() ? 0 : fread(file.data(), 1, file.size(), f);
        fclose(f);
        if (got != file.size()) {
            why = "short read";
            return false;
        }
    }
    Cursor c{file.data(), file.data() + file.size()};
    if (file.size() < 8 || memcmp(file.data(), "GASTOR04", 8) != 0) {
        why = "not a GASTOR04 storage index";
        return false;
    }
    c.Skip(8);
    c.SkipString();   // Root
    const int32_t ns = c.Get<int32_t>();
    for (int32_t i = 0; i < ns && c.ok; ++i) c.SkipString();

    // One grid a level, 0..kLevel, each of 6 4^L cells: the finest level stored over the cell
    // (-1 = none) and the bytes stored in it. An entry lands in the grid of its own level (a finer
    // one in its kLevel ancestor); the pass below hands each cell's level and bytes to its four
    // children, so every texel of the last grid ends holding every entry over it -- one law, a
    // walk down the quadtree, whatever the mix of levels.
    std::vector<std::vector<int8_t>> lvl(kLevel + 1);
    std::vector<std::vector<float>> bytes(kLevel + 1);
    for (uint32_t L = 0; L <= kLevel; ++L) {
        lvl[L].assign(static_cast<size_t>(6) << (2 * L), int8_t(-1));
        bytes[L].assign(static_cast<size_t>(6) << (2 * L), 0.0f);
    }
    const int32_t nf = c.Get<int32_t>();
    for (int32_t fi = 0; fi < nf && c.ok; ++fi) {
        c.Skip(4 + 8 + 4 + 4);   // path, stamp, note, voids
        const int32_t ne = c.Get<int32_t>();
        for (int32_t k = 0; k < ne && c.ok; ++k) {
            c.Skip(16);   // source, param, store, lattice
            const uint64_t cell = c.Get<uint64_t>();
            const int64_t b = c.Get<int64_t>();
            c.Skip(4 + 1);   // tiles, derived
            const uint32_t face = static_cast<uint32_t>(cell >> 61);
            const uint32_t L = static_cast<uint32_t>((cell >> 56) & 31u);
            const uint32_t y = static_cast<uint32_t>((cell >> 28) & 0x0FFFFFFFu);
            const uint32_t x = static_cast<uint32_t>(cell & 0x0FFFFFFFu);
            if (face > 5 || L > 28 || x >= (1u << L) || y >= (1u << L)) continue;   // off the lattice
            const uint32_t G = (std::min)(L, kLevel), sh = L - G, n = 1u << G;
            const size_t i = (static_cast<size_t>(face) * n + (y >> sh)) * n + (x >> sh);
            lvl[G][i] = static_cast<int8_t>((std::max)(static_cast<int>(lvl[G][i]), static_cast<int>(L)));
            bytes[G][i] += static_cast<float>(b > 0 ? b : 0);
            ++entries;
        }
        const int32_t nu = c.Get<int32_t>();
        for (int32_t k = 0; k < nu && c.ok; ++k) c.Skip(12 + 8 + 4 + 4);
        const int32_t nc = c.Get<int32_t>();
        for (int32_t k = 0; k < nc && c.ok; ++k) c.Skip(12 + 5 * 8 + 1 + 8);
        ++folders;
    }
    if (!c.ok) {
        why = "truncated or of another layout";
        entries = folders = 0;
        return false;
    }
    for (uint32_t L = 0; L < kLevel; ++L) {
        const uint32_t n = 1u << L;
        for (uint32_t f = 0; f < 6; ++f)
            for (uint32_t y = 0; y < n; ++y)
                for (uint32_t x = 0; x < n; ++x) {
                    const size_t i = (static_cast<size_t>(f) * n + y) * n + x;
                    if (lvl[L][i] < 0 && bytes[L][i] == 0.0f) continue;
                    for (uint32_t q = 0; q < 4; ++q) {
                        const size_t j = (static_cast<size_t>(f) * 2 * n + 2 * y + (q >> 1)) * 2 * n + 2 * x + (q & 1);
                        lvl[L + 1][j] = (std::max)(lvl[L + 1][j], lvl[L][i]);
                        bytes[L + 1][j] += 0.25f * bytes[L][i];
                    }
                }
    }
    // The last grid IS the image: face-major rows of kSide texels, so row = face kSide + y.
    const std::vector<int8_t>& lv = lvl[kLevel];
    const std::vector<float>& by = bytes[kLevel];
    rg.assign(lv.size() * 2, 0);
    for (size_t i = 0; i < lv.size(); ++i) {
        if (lv[i] < 0) continue;
        const double g = std::log2(1.0 + static_cast<double>(by[i])) / 40.0 * 255.0;
        rg[2 * i] = static_cast<uint8_t>((std::min)(static_cast<int>(lv[i]), 12) * 20);
        rg[2 * i + 1] = static_cast<uint8_t>(std::clamp(std::lround(g), 1L, 255L));   // > 0: something is there
        ++texelsLit;
    }
    ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return true;
}

}  // namespace ga
