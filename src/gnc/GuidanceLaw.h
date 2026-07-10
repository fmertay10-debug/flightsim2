#pragma once

#include "gnc/CommandSet.h"
#include "core/State.h"
#include "sim/WorldView.h"

// Strategy: guidance law (the OUTER GNC loop). Turns relative geometry --
// own state plus the WorldView snapshot (to see the target) -- into a
// CommandSet that OVERLAYS the flight plan: any field the law sets wins over
// the scripted value; unset fields fall through to the plan.
//
// Laws are stateful (a PN law integrates its command), so update() is
// non-const. If the target is dead/absent a law should hold its last command.
class GuidanceLaw {
public:
    virtual ~GuidanceLaw() = default;

    virtual CommandSet update(const State& self, const WorldView& world,
                              double dt) = 0;
};
