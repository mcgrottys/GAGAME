// Node - the Composite's bodies (M12 step 5a). See Node.h.
#include "scene/Node.h"

#include "scene/Component.h"

namespace ga::scene {

Node::Node(std::string name) : m_name(std::move(name)) {
    m_space.name = m_name;
}

Node::~Node() = default;

Node& Node::AddChild(std::unique_ptr<Node> child) {
    child->m_parent = this;
    child->m_space.parent = &m_space;
    m_children.push_back(std::move(child));
    return *m_children.back();
}

Component& Node::AddComponent(std::unique_ptr<Component> component) {
    m_components.push_back(std::move(component));
    return *m_components.back();
}

Node* Node::Find(const std::string& name) {
    if (m_name == name) return this;
    for (auto& c : m_children) {
        if (Node* n = c->Find(name)) return n;
    }
    return nullptr;
}

const Node* Node::Find(const std::string& name) const {
    if (m_name == name) return this;
    for (const auto& c : m_children) {
        if (const Node* n = c->Find(name)) return n;
    }
    return nullptr;
}

std::string Node::Path() const {
    return m_parent ? m_parent->Path() + "." + m_name : m_name;
}

ga::Placement ResolveAlong(const Node& node) {
    return node.Placement().ToRoot();
}

void Walk(Node& node, const std::function<bool(Node&, int)>& visit, int depth) {
    if (!visit(node, depth)) return;
    for (auto& c : node.Children()) Walk(*c, visit, depth + 1);
}

void Walk(const Node& node, const std::function<bool(const Node&, int)>& visit, int depth) {
    if (!visit(node, depth)) return;
    for (const auto& c : node.Children()) Walk(static_cast<const Node&>(*c), visit, depth + 1);
}

}  // namespace ga::scene
