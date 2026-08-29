// ================================================================================================
//  MarkerLayer - M6j: renders any Exchange channel whose elements are {PGA motor, scale+color}
//  as instanced pylons (Markers.hlsl applies the sandwich on the GPU). The layer knows the
//  CHANNEL NAME, not the producer -- today the tide-station plugin publishes it; tomorrow a
//  physics plugin can publish moving motors into the same socket and nothing here changes.
// ================================================================================================
#pragma once

#include "compose/Exchange.h"
#include "scene/Layer.h"

#include <string>

namespace ga {

class MarkerLayer : public Layer {
public:
    void Configure(const std::wstring& shaderDir, const Exchange* exchange,
                   const std::string& channel) {
        m_shaderDir = shaderDir;
        m_exchange = exchange;
        m_channel = channel;
    }

    const char* Name() const override { return "markers"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              ID3D12RootSignature* rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;

private:
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);

    std::wstring m_shaderDir;
    const Exchange* m_exchange = nullptr;
    std::string m_channel;
    ID3D12RootSignature* m_rootSig = nullptr;
    Com<ID3D12PipelineState> m_pso;
};

}  // namespace ga
