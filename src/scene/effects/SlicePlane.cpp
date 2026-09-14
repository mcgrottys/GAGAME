// SlicePlane - the cutaway plane as an effect node (M12 step 5e). See SlicePlane.h.
#include "scene/effects/SlicePlane.h"

#include "core/Common.h"
#include "core/GaAst.h"
#include "scene/GlobeLayer.h"

namespace ga::scene {

std::vector<std::string> SlicePlane::Configure(const Wiring& w) {
    (void)w;   // no device object of its own: the globe draws, this declares
    return {};
}

bool SlicePlane::Init(Gpu& gpu) {
    (void)gpu;
    return true;
}

void SlicePlane::Apply(const PropSet& props) {
    std::string why;
    if (!props.ApplyTo(&m_props, nullptr, &why)) {
        Log("[effect] '%s' apply refused: %s", m_name.c_str(), why.c_str());
        return;
    }
    FanOut();
}

void SlicePlane::Update(const FrameInfo& f) { (void)f; }
void SlicePlane::Record(const ViewContext& v) { (void)v; }

std::vector<std::string> SlicePlane::Configure(const Observers& o) {
    m_o = o;
    std::vector<std::string> missing;
    if (!o.globe) missing.push_back("globe");
    if (!missing.empty()) Log("[effect] '%s' NOT wired: globe (the plane cuts nothing)", m_name.c_str());
    return missing;
}

void SlicePlane::Declare(const std::string& name, bool on, double d) {
    if (!name.empty()) m_name = name;
    enabled = on;
    m_props.d = d;
}

void SlicePlane::FanOut() {
    if (!m_o.globe) return;
    m_o.globe->sliceOn = enabled;
    m_o.globe->sliceD = static_cast<float>(enabled ? m_props.d : 0.0);
}

std::vector<FieldPort> SlicePlane::Inputs() const {
    // The user's plane: a grade-1 element of the PGA (a plane is a vector there), the signed
    // distance in world metres.
    return {{"user.plane", 0b10u, "signed distance m", "world.m"}};
}

std::vector<FieldPort> SlicePlane::Outputs() const {
    // The discard: a scalar test in the globe's pixel stage.
    return {{"slice", 0b1u, "keep s<=0", "globe.ps"}};
}

void SlicePlane::RegisterEdges() {
    // M7o: the demo node registers its edge like any other -- the AST is how
    // features arrive now. One blade, one inner product, one discard.
    ga::ast::Register({"user.plane", "globe.ps", "slice",
                       {"world.m", true, 0, 0, 0}, {"world.m", true, 0, 0, 0},
                       false, "signed distance m", "keep s<=0", 1.0,
                       "Globe.hlsl slice discard (gatest: sandwich negates s; "
                       "proofs/slice_plane.py)"});
}

}  // namespace ga::scene
