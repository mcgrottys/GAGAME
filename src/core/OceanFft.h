// ================================================================================================
//  OceanFft - M2: the stateless spectral ocean, three cascades of 256^2.
//
//  Per forecast tick: CsInitSpectrum builds h0(k) from the partition parameters (deterministic
//  Gaussians seeded by the forecast cycle -- two machines on the same cycle synthesise the same
//  ocean, the gameplan's multiplayer promise). Per frame: CsModulate advances every bin by the
//  rotor e^{iwt}, CsFft runs the 2D inverse transform (radix-2, groupshared, two packed complex
//  channels per texture), CsAssemble writes displacement + derivative textures that the SeaLayer
//  samples through the bindless heap.
//
//  Everything stays in UAV state through the compute chain; only the two output textures per
//  cascade transition to pixel-shader-readable for the draw and back again next frame.
// ================================================================================================
#pragma once

#include "core/Gpu.h"
#include "core/Shader.h"
#include "sim/SeaState.h"

#include <string>

namespace ga {

class OceanFft {
public:
    static constexpr uint32_t kN = 256;
    static constexpr uint32_t kCascades = 3;

    void Init(Gpu& gpu, ShaderCompiler& sc, const std::wstring& shaderDir);
    bool ReloadShaders(Gpu& gpu, ShaderCompiler& sc);

    // New partition set (forecast hour changed). seed keys the Gaussian draw; pass something
    // derived from the forecast cycle so the ocean is reproducible.
    void SetSeaState(const PartParam* parts, int count, uint32_t seed);

    // Records the whole compute chain for this frame into cl. tSec = seconds since the cycle.
    void Record(ID3D12GraphicsCommandList* cl, Gpu& gpu, float tSec);

    float PatchL(uint32_t c) const { return m_patchL[c]; }
    uint32_t DispSrv(uint32_t c) const { return m_dispSrv[c]; }
    uint32_t DerivSrv(uint32_t c) const { return m_derivSrv[c]; }
    ID3D12Resource* DerivRes(uint32_t c) const { return m_cascade[c].deriv.Get(); }
    bool Ready() const { return m_ready; }

    // Numeric gate: read the displacement textures back and measure the RENDERED significant
    // height (4 sqrt of the summed height-channel variance). A single realization scatters a few
    // percent around the target; a synthesis bug shows up as a factor.
    double MeasureHs(Gpu& gpu);

private:
    struct Cascade {
        Com<ID3D12Resource> h0, pingA, pongA, pingB, pongB, disp, deriv;
        uint32_t blockInit = 0, blockMod = 0, blockRows = 0, blockCols = 0, blockAsm = 0;
        bool outputsArePs = false;   // disp/deriv currently in pixel-shader-resource state
    };

    bool BuildPipelines(Gpu& gpu, ShaderCompiler& sc);
    void Dispatch(ID3D12GraphicsCommandList* cl, Gpu& gpu, ID3D12PipelineState* pso,
                  uint32_t block, uint32_t cascade, uint32_t dir, float tSec, uint32_t gx,
                  uint32_t gy);

    std::wstring m_shaderDir;
    Com<ID3D12RootSignature> m_rootSig;
    Com<ID3D12PipelineState> m_init, m_modulate, m_fft, m_assemble;

    Cascade m_cascade[kCascades];
    float m_patchL[kCascades] = {756.0f, 186.0f, 47.0f};
    float m_bandLo[kCascades] = {};
    float m_bandHi[kCascades] = {};
    uint32_t m_dispSrv[kCascades] = {};
    uint32_t m_derivSrv[kCascades] = {};

    struct PartsCbData {
        uint32_t count[4] = {};
        PartParam parts[4];
    };
    PartsCbData m_parts;
    uint32_t m_seed = 1;
    bool m_spectrumDirty = false;
    bool m_ready = false;
    float m_lambda = 1.1f;
};

}  // namespace ga
