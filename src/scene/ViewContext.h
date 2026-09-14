// ================================================================================================
//  ViewContext / ViewSet - M12 step 5b: THE SEAM. The renderer records a LIST of views.
//
//  Today the list has one element and the bytes it records are the bytes it recorded before this
//  file existed. That is the whole point: the second view is priced here, once, while the picture
//  cannot move -- rather than in the middle of a feature, where a split screen or a remote
//  player's eye would have had to invent a per-view everything at the same time as the thing it
//  was for.
//
//  A ViewContext is ONE view's recording:
//      index        which view of the set; 0 is the session's own, and FrameContext::viewIndex
//                   carries it to a layer that keys per-view state
//      view         the View that produced it (scene/View.h), or null while the session's
//                   Camera is still the state -- which it is until 5d wires the scene
//      constants    THE b0 rows for this view (SceneConstants), a pure function of the view and
//                   the scene-wide lighting: Renderer::FillSceneConstants, gated byte for byte
//                   against the frozen old fill in dxtest
//      cmd          the frame's recording. The RENDERER owns it and writes it here; a caller
//                   building a ViewSet leaves it null
//      target       the named target chain this view lands in. "main" is the renderer's own --
//                   HDR sceneColor -> tonemap -> ldrTarget/backbuffer. A second chain is a
//                   NAMED FOLLOW-ON, not this step: every view today records into "main"
//      viewport     the rectangle of that target, in pixels; a zero width or height means the
//                   whole target, which is what every recorded frame means. An OFFSET (x, y)
//                   needs one more overload on hal::CommandContext, which this step does not
//                   add -- the renderer REPORTS an offset it cannot honour rather than
//                   silently drawing full-screen
//      legacy       the FrameContext every Layer has always drawn from. The renderer fills the
//                   fields it owns (gpu, cmd, sceneCb, prof, viewIndex); the caller fills the
//                   view's own (camera, timeSec, width, height). It is called `legacy` because
//                   it goes when LayerComponent replaces Layer, and not before.
//
//  A ViewSet is those in FILE ORDER, and file order is walk order: the residency wants are
//  ADDITIVE within a frame, so N walks in view order are the union of what the N views want,
//  and the predicted-want hash folds call order -- which is why a second view always walks
//  AFTER the main one and never before it.
//
//  WHAT IS NOT HERE, named rather than half-built: one target chain per view; a second
//  camera-anchored water bank and its own walk per view (the M10 "set B" bank is the precedent,
//  and it stays hard-coded as set B this step -- no layer changes its keying here);
//  WeatherManager::Update over a span of viewers; and the network replication of a remote
//  player's view motor, which is a `view` node whose placement is replicated and nothing else.
//
//  Prior art, named: a list of views with per-view constants and viewports is every engine's
//  render-view/scene-view list (Unreal's FSceneView array per FSceneViewFamily, Unity's
//  per-camera culling and constant buffers); nothing here is new, and the only thing it buys is
//  that the day the engine grows a second eye, it grows one.
// ================================================================================================
#pragma once

#include "render/Renderer.h"   // SceneConstants: the b0 rows this view records
#include "scene/Layer.h"       // FrameContext: what a Layer has always drawn from

#include <cstdint>
#include <vector>

namespace ga::scene {

class View;

// The rectangle of the target a view records into, in pixels. Zero width or height = the whole
// target, which is what every view means today.
struct ViewRect {
    uint32_t x = 0, y = 0, w = 0, h = 0;
};

struct ViewContext {
    uint32_t index = 0;
    const View* view = nullptr;
    SceneConstants constants{};
    hal::CommandContext* cmd = nullptr;
    const char* target = "main";
    ViewRect viewport;
    FrameContext legacy;
};

struct ViewSet {
    std::vector<ViewContext> views;   // FILE ORDER = walk order
};

}  // namespace ga::scene
