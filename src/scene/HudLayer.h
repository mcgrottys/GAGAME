// ================================================================================================
//  HudLayer - THE WINDSHIELD: what is drawn on the glass, not in the world.
//
//  A layer the renderer records AFTER the tonemap, straight into the display-referred target
//  (Renderer::AddOverlay), so a readout is neither exposed with the scene nor lit by its sun:
//  white text is white at noon and at midnight. Everything on the glass is one primitive -- an
//  axis-aligned rectangle of the target, in pixels, with a kind that says how it fills:
//      Glyph     a 5x7 character (core/Font5x7.h), its five columns carried on the element
//      Fill      a solid rectangle
//      Frame     a rectangle's border, `edge` px thick
//      Ring      the disc inscribed in the rectangle, or its ring when `edge` > 0
//  One instanced draw of six vertices per element (shaders/Hud.hlsl), the list rebuilt by its
//  owner every frame between Begin() and the render: the glass holds no state of its own, so a
//  second eye's readouts are the same calls with its own rectangle.
//
//  Colours are display values (0..1 after the curve), blended over the picture by their alpha.
// ================================================================================================
#pragma once

#include "scene/Layer.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

class HudLayer : public Layer {
public:
    enum Kind : uint32_t { Glyph = 0, Fill = 1, Frame = 2, Ring = 3 };
    struct Rgba {
        float r = 1, g = 1, b = 1, a = 1;
    };

    void Configure(const std::wstring& shaderDir) { m_shaderDir = shaderDir; }

    const char* Name() const override { return "hud"; }
    void Init(Gpu& gpu, ShaderCompiler& sc, FieldSet& fields,
              hal::RootSignature rootSig) override;
    void ReloadShaders(Gpu& gpu, ShaderCompiler& sc) override;
    void Render(const FrameContext& ctx) override;

    // ---- the frame's glass. Begin clears it; every call after adds to it, in draw order.
    void Begin() { m_el.clear(); }
    void Rect(float x, float y, float w, float h, Kind kind, Rgba c, float edge = 0.0f);
    // A line of text with its top-left at (x, y), each font pixel `px` screen pixels square,
    // and a one-pixel drop shadow so it reads over sky and sea alike. Returns its width in px.
    float Text(float x, float y, float px, const std::string& s, Rgba c);
    static float TextWidth(const std::string& s, float px) { return s.size() * 6.0f * px; }

private:
    // Mirrored in shaders/Hud.hlsl (HudEl), 48 bytes.
    struct Element {
        float rect[4];    // x0, y0, x1, y1 in target pixels
        float color[4];   // display rgb, alpha
        uint32_t kind, glyphLo, glyphHi;   // the glyph's columns 0-3 / column 4, a byte each
        float edge;
    };
    static_assert(sizeof(Element) == 48, "HudEl is 48 bytes on both sides");
    bool BuildPso(Gpu& gpu, ShaderCompiler& sc);

    std::wstring m_shaderDir;
    hal::RootSignature m_rootSig = nullptr;
    hal::Pso m_pso;
    std::vector<Element> m_el;
};

}  // namespace ga
