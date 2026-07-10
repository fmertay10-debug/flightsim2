#pragma once

#include <vector>

#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"
#include "gnc/CommandSet.h"
#include "mass/MassModel.h"

// Everything a control law may need at one step, assembled by the Entity.
struct GncContext {
    const State&     state;
    const AirData&   air;
    const MassState& mass;   // mass / inertia (about CG) / CG station now
    double dt = 0.0;
};

// Strategy: control law -- tracks Commands by writing actuator channels.
// Direct-write laws (the PIDs, the gain schedule) put plant knowledge in
// their gains and write surface/gimbal channels themselves; allocation-based
// laws emit desired body moments internally and let an Allocator distribute
// them over whatever channels the vehicle declares (ADR-0002: both contracts
// are legal).
//
// Laws are stateful (integrators, filters) -- one instance per entity.
class ControlLaw {
public:
    virtual ~ControlLaw() = default;

    // Which command vocabularies this law can track. Attitude-level is the
    // default; laws that track acceleration commands override. Validated
    // against the guidance law's emitted level when guidance is attached.
    virtual bool accepts(CommandLevel level) const {
        return level == CommandLevel::Attitude;
    }

    // Resolve the channels this law writes against the vehicle's declared
    // table and keep the handles. Channels the law cannot function without go
    // through table.require() -- it throws with a listing of what IS declared,
    // so a law paired with a vehicle that lacks the matching consumer fails at
    // LOAD instead of silently flying open-loop. Secondary channels use
    // table.find() (an invalid handle drops the writes). Returns every handle
    // bound (invalid ones included); the loader warns about declared channels
    // no law drives. Called once at load.
    virtual std::vector<ChannelHandle> bindChannels(const ChannelTable& table) = 0;

    virtual void update(const GncContext& gc,
                        const CommandSet& cmd,
                        ChannelValues& out) = 0;
};
