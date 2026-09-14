// ================================================================================================
//  LayerComponent - M12 step 5a: a Layer, UNCHANGED, behind the Component contract.
//
//  The adapter: Configure keeps the wiring and reports what a Layer::Init needs and did not
//  get (the device, the compiler, the field table, the root signature); Init is Layer::Init;
//  Apply reads the one property every layer has (`enabled`); Update is a no-op -- a layer's
//  per-frame setters are the owning component's business, called from ITS Update; Record
//  builds the FrameContext the layer has always drawn from and calls Render; ReloadShaders is
//  the layer's. Declared and compiled in 5a, instantiated by the Assembly from 5d.
// ================================================================================================
#pragma once

#include "hal/Gpu.h"
#include "scene/Component.h"
#include "scene/Layer.h"

namespace ga::scene {

struct LayerProps {
    bool enabled = true;
};

class LayerComponent final : public Component {
public:
    explicit LayerComponent(std::unique_ptr<Layer> layer) : m_layer(std::move(layer)) {}

    const char* Name() const override { return m_layer->Name(); }
    const Schema& Props() const override { return LayerSchema(); }
    Layer& Wrapped() { return *m_layer; }

    std::vector<std::string> Configure(const Wiring& w) override {
        m_w = w;
        std::vector<std::string> missing;
        if (!w.gpu) missing.push_back("gpu");
        if (!w.shaders) missing.push_back("shaders");
        if (!w.fields) missing.push_back("fields");
        if (!w.rootSignature) missing.push_back("rootSignature");
        return missing;
    }
    bool Init(Gpu& gpu) override {
        if (!m_w.shaders || !m_w.fields || !m_w.rootSignature) return false;
        m_layer->Init(gpu, *m_w.shaders, *m_w.fields,
                      static_cast<hal::RootSignature>(m_w.rootSignature));
        return true;
    }
    void Apply(const PropSet& props) override {
        LayerProps p;
        p.enabled = m_layer->enabled;
        std::string why;
        if (props.ApplyTo(&p, nullptr, &why)) {
            enabled = p.enabled;
            m_layer->enabled = p.enabled;
        }
    }
    void Update(const FrameInfo&) override {}
    void Record(const ViewContext& v) override {
        if (!m_layer->enabled) return;
        FrameContext ctx;
        ctx.gpu = m_w.gpu;
        ctx.cmd = v.cmd;
        ctx.camera = v.camera;
        ctx.sceneCb = v.sceneCb;
        ctx.timeSec = v.timeSec;
        ctx.width = v.width;
        ctx.height = v.height;
        ctx.prof = v.prof;
        m_layer->Render(ctx);
    }
    void ReloadShaders() override {
        if (m_w.gpu && m_w.shaders) m_layer->ReloadShaders(*m_w.gpu, *m_w.shaders);
    }

    // The one schema every layer shares (a Flyweight: built once over a prototype).
    static const Schema& LayerSchema() {
        static LayerProps proto;
        static const Schema* s = [] {
            Schema* sc = new Schema("layer", &proto);
            sc->Bind("enabled", proto.enabled, "draw this layer", Reload::Hot);
            return sc;
        }();
        return *s;
    }

private:
    std::unique_ptr<Layer> m_layer;
    Wiring m_w;
};

}  // namespace ga::scene
