#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include "core/State.h"

// Immutable snapshot of every entity's state at time t.
// Built once per step BEFORE any entity propagates, so all entities react to
// the same instant regardless of update order -- the interaction seam for
// multi-vehicle behaviors (guidance, formation, collision checks).
class WorldView {
public:
    struct Item {
        int         id = -1;
        std::string name;
        State       state;
        bool        alive = true;
    };

    void add(Item item) { items_.push_back(std::move(item)); }

    const State& stateOf(int id) const {
        for (const Item& it : items_)
            if (it.id == id) return it.state;
        throw std::out_of_range("WorldView: no entity with id " + std::to_string(id));
    }

    bool isAlive(int id) const {
        for (const Item& it : items_)
            if (it.id == id) return it.alive;
        return false;
    }

    const std::vector<Item>& items() const { return items_; }

private:
    std::vector<Item> items_;
};
