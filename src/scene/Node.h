// ================================================================================================
//  Node - M12 step 5a: THE SCENE GRAPH'S ONE STRUCTURAL TYPE. A Composite whose placement is a
//  Space (core/Space.h), so "where is this node" is the frame calculus and nothing else.
//
//  A node has a name, a Space (its placement in the parent is that Space's link; its unit
//  length is declared there, priors 32), a parent, children and components -- and BOTH lists
//  keep FILE ORDER, because order is data: the layers draw in registration order (Renderer),
//  a rail's keys are a sequence, the AST's paint rows are an order. Nothing here sorts.
//
//  Beside the type, two pure functions and no third: ResolveAlong(node) is the fold root->node
//  through the Spaces -- Space::ToRoot(), the identity at the root -- computed where it is
//  read and never cached into a 4x4 (the one matrix in the engine is Placement::ToMatrix at
//  the rasterizer boundary); Walk(node, visitor) is the catamorphism every traversal is written
//  as -- the want walk, the record walk, the reload walk are visitors, not loops of their own.
//
//  Prior art, named: the Composite of Gamma et al.; USD's prim hierarchy (a prim's path is its
//  identity, composition resolves along it) and Godot's SceneTree (nodes own children, a scene
//  file instances a subtree). The Droste tower is NOT a child here: it is Space::Cycle, an
//  explicit reference the walk expands under a traversal context, as ARCHITECTURE.md states.
// ================================================================================================
#pragma once

#include "core/Space.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ga::scene {

class Component;

class Node {
public:
    explicit Node(std::string name);
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;
    ~Node();

    const std::string& Name() const { return m_name; }
    // Its Space: link = the placement in the parent, unitM the declared unit length.
    Space& Placement() { return m_space; }
    const Space& Placement() const { return m_space; }
    Node* Parent() const { return m_parent; }

    // Children and components are appended -- file order -- and never moved once added, so a
    // Space's parent pointer and a component's owner stay valid for the node's life.
    Node& AddChild(std::unique_ptr<Node> child);
    Component& AddComponent(std::unique_ptr<Component> component);
    const std::vector<std::unique_ptr<Node>>& Children() const { return m_children; }
    const std::vector<std::unique_ptr<Component>>& Components() const { return m_components; }

    // Depth-first in file order, this node included; null when no node has the name.
    Node* Find(const std::string& name);
    const Node* Find(const std::string& name) const;
    // "root.child.grandchild" -- what a refusal names.
    std::string Path() const;

    bool enabled = true;

private:
    std::string m_name;
    Space m_space;
    Node* m_parent = nullptr;
    std::vector<std::unique_ptr<Node>> m_children;
    std::vector<std::unique_ptr<Component>> m_components;
};

// THE FOLD: the node's placement in the root's frame, through the Spaces (Space::ToRoot).
ga::Placement ResolveAlong(const Node& node);

// THE CATAMORPHISM: visit(node, depth) over the subtree in file order; a visitor returning
// false prunes that node's children (the traversal context's budget, the enabled flag).
void Walk(Node& node, const std::function<bool(Node&, int)>& visit, int depth = 0);
void Walk(const Node& node, const std::function<bool(const Node&, int)>& visit, int depth = 0);

}  // namespace ga::scene
