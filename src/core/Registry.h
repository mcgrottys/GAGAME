// ================================================================================================
//  Registry - M12 step 5a: THE HOUSE FACTORY IDIOM, WRITTEN ONCE.
//
//  This engine already has two factories of exactly one shape -- FieldLoader.h's LoaderRegistry
//  (a file type -> a loader) and VesselSpec.h's VesselRegistry (a hull kind -> a spec) -- and
//  VesselSpec.h says out loud that it copied the first: "a descriptor that IS the identity, a
//  std::function builder registered under a name, composition over inheritance because what
//  varies between products is DATA and not behaviour." The scene needs three more of the same
//  shape (components by type, effects by type, tools by name), and a third hand-written copy is
//  where the idiom would start to drift. So the shape is a template, and the two originals are
//  NOT touched: scenetest (scene/SceneTest.cpp) proves this template answers exactly as they do
//  on the built-in kinds -- the same names in the same order, the same product for the same
//  inputs, the same empty answer for a miss -- so a later step can alias them to it with a
//  gate already in hand rather than a rewrite on faith.
//
//  THE ONE RULE A MISS OBEYS: it logs and returns the EMPTY product -- Product{} -- never a
//  substitute. VesselRegistry states why: "a silently-substituted default hull would be the
//  worst possible failure here (it would float, and it would be the wrong boat)". For a
//  std::unique_ptr product the empty answer is nullptr, for a spec it is the spec whose kind is
//  empty; the caller checks, as it always did.
//
//  Prior art, named: the Abstract Factory / Registry pairing of Gamma et al. (products by name,
//  the concrete factory a function), and the plugin registries of Godot (ClassDB) and USD
//  (the plugin registry keyed by type name) where a name is the whole identity a file needs to
//  carry. Nothing here is a class hierarchy; the registry is a map from a string to a function,
//  and Names() is what a scene editor lists.
// ================================================================================================
#pragma once

#include "core/Common.h"

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ga {

template <class Product, class... Args>
class Registry {
public:
    using Factory = std::function<Product(Args...)>;

    // `what` names the registry in the miss line ("[vessel] no such kind ..."), so a log reads
    // the same whether the map was hand-written or this template.
    explicit Registry(const char* what = "registry") : m_what(what) {}

    void Register(const std::string& name, Factory make) { m_byName[name] = std::move(make); }

    bool Knows(const std::string& name) const { return m_byName.count(name) != 0; }

    // The empty product on a miss, logged. Never a substitute (see the banner).
    Product Make(const std::string& name, Args... args) const {
        auto it = m_byName.find(name);
        if (it == m_byName.end()) {
            Log("[%s] no such kind '%s' -- %zu registered", m_what, name.c_str(),
                m_byName.size());
            return Product{};
        }
        return it->second(std::forward<Args>(args)...);
    }

    // Sorted: std::map's order, which is what the two originals return as well.
    std::vector<std::string> Names() const {
        std::vector<std::string> v;
        v.reserve(m_byName.size());
        for (const auto& kv : m_byName) v.push_back(kv.first);
        return v;
    }

    size_t Count() const { return m_byName.size(); }
    const char* What() const { return m_what; }

private:
    const char* m_what = "registry";
    std::map<std::string, Factory> m_byName;
};

}  // namespace ga
