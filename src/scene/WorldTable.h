// ================================================================================================
//  WorldTable - every world an eye reaches, by index (one an eye, one a frame).
//
//  A world is the same planet seen from another eye: a Droste level, or the place behind k windows
//  of the view's chain. What the passes need of it is a few rows -- its gauge, its sun and sky
//  (Globe.hlsl LoadLevel), and for the chain each window's box and the light seen through it --
//  and nothing about it is a depth limit: the table is as long as the chain the screen allows
//  (scene/Gateway.cpp WindowChain stops at a window under two pixels). The renderer uploads it per
//  view and says where in b2 (gCsEyeT.w); Common.hlsli's Wt* functions read it.
//
//  Packed as float4 rows: row 0 = (levels, windows, first level row, first box row) as uints,
//  row 1 = (first light row, -, -, -); then kLevelRows a level, kBoxRows a window (WindowBox::Pack),
//  kLightRows a window (the zenith there with the eye's radius in w; the sun seen from there).
// ================================================================================================
#pragma once

#include "scene/WindowBox.h"

#include <cstdint>
#include <cstring>
#include <vector>

namespace ga::scene {

struct WorldTable {
    static constexpr uint32_t kLevelRows = 6, kBoxRows = 4, kLightRows = 2;
    std::vector<float> levels;   // kLevelRows float4 a level (GlobeLayer: the level table)
    std::vector<float> boxes;    // kBoxRows float4 a window of the chain
    std::vector<float> light;    // kLightRows float4 a window

    // The chain: n windows, their boxes, the zenith there (xyz, eye radius in w) and the sun (xyz).
    void SetChain(const WindowBox* b, const float* up4, const float* sun3, int n) {
        boxes.assign(size_t(n > 0 ? n : 0) * kBoxRows * 4, 0.0f);
        light.assign(size_t(n > 0 ? n : 0) * kLightRows * 4, 0.0f);
        for (int k = 0; k < n; ++k) {
            b[k].Pack(&boxes[size_t(k) * kBoxRows * 4]);
            float* l = &light[size_t(k) * kLightRows * 4];
            for (int i = 0; i < 4; ++i) l[i] = up4 ? up4[k * 4 + i] : 0.0f;
            for (int i = 0; i < 3; ++i) l[4 + i] = sun3 ? sun3[k * 3 + i] : 0.0f;
        }
    }
    uint32_t Levels() const { return uint32_t(levels.size() / (kLevelRows * 4)); }
    uint32_t Windows() const { return uint32_t(boxes.size() / (kBoxRows * 4)); }

    void Pack(std::vector<float>& out) const {
        const uint32_t a = 2u, b = a + Levels() * kLevelRows, c = b + Windows() * kBoxRows;
        out.assign(size_t(c + Windows() * kLightRows) * 4, 0.0f);
        const uint32_t head[8] = {Levels(), Windows(), a, b, c, 0u, 0u, 0u};
        memcpy(out.data(), head, sizeof(head));
        if (!levels.empty()) memcpy(&out[size_t(a) * 4], levels.data(), levels.size() * sizeof(float));
        if (!boxes.empty()) memcpy(&out[size_t(b) * 4], boxes.data(), boxes.size() * sizeof(float));
        if (!light.empty()) memcpy(&out[size_t(c) * 4], light.data(), light.size() * sizeof(float));
    }
};

}  // namespace ga::scene
