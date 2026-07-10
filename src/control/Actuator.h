#pragma once

#include "core/ControlInput.h"

// Strategy: actuator dynamics between COMMANDED and ACTUAL control.
// A null actuator on an entity means ideal (actual == commanded).
class Actuator {
public:
    virtual ~Actuator() = default;
    virtual ControlInput apply(const ControlInput& commanded, double dt) = 0;
};
