// ================================================================================================
//  WindowBox - a gate's window as the passes test it (M13).
//
//  The globe, the sky and the hulls all keep a pixel by the SAME ordered slab test (Common.hlsli
//  GateSlabFrom, walked down the chain): how many windows of the view's chain the segment from the
//  eye to the pixel's point passes, in order, is the depth of the world that pixel belongs to. Each
//  window is its box in the TRUE camera frame -- rows camera -> box, half extents, and its centre
//  relative to the eye -- packed the one way all three constant buffers carry it: four float4 rows
//  a window, rows 0..2 the box's axes with the half extent in w, row 3 its centre.
// ================================================================================================
#pragma once

namespace ga {

// (How deep a view's chain of windows goes is the screen's to say -- scene/Gateway.cpp WindowChain
// stops at a window under two pixels -- and the scene's windowDepth, when it caps it.)

struct WindowBox {
    float rows[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float half[3] = {0.0f, 0.0f, 0.0f};
    float centre[3] = {0.0f, 0.0f, 0.0f};

    void Pack(float out[16]) const {
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) out[r * 4 + c] = rows[r * 3 + c];
            out[r * 4 + 3] = half[r];
        }
        for (int c = 0; c < 3; ++c) out[12 + c] = centre[c];
        out[15] = 0.0f;
    }
};

}  // namespace ga
