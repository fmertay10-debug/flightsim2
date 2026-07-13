#pragma once

#include <memory>
#include <string>
#include <vector>

#include "gnc/ActuatorBank.h"
#include "gnc/control/ControlLaw.h"
#include "gnc/guidance/FlightPlan.h"
#include "core/Channel.h"
#include "core/State.h"
#include "core/Telemetry.h"
#include "dynamics/EquationsOfMotion.h"
#include "environment/Environment.h"
#include "gnc/guidance/GuidanceLaw.h"
#include "vehicle/Vehicle.h"

// An object in the simulated world: kinematic state + an integrator,
// optionally carrying a Vehicle (mass + force components) and a GNC stack
// (flight plan / guidance / controller / actuators). Any part can be null:
//   null vehicle    -> no loads at all (kinematic movers)
//   null controller -> zero control input
//   null actuators  -> ideal (actual == commanded)
//
// The ChannelTable is the vehicle's declared actuator channels; the loader
// builds it (components declare, the controller binds) BEFORE construction,
// so every component's handles are already resolved against it.
//
// Two-phase stepping: propagate() computes the next state WITHOUT writing it;
// the Simulation commits all entities afterwards so everyone reacts to the
// same world snapshot.
class Entity {
public:
    Entity(std::string name,
           std::unique_ptr<Vehicle>           vehicle,   // may be null for kinematic movers
           std::unique_ptr<ControlLaw>        controller,
           std::unique_ptr<ActuatorBank>      actuators,
           std::unique_ptr<EquationsOfMotion> eom,
           FlightPlan                         flightPlan,
           const State&                       initialState,
           ChannelTable                       channels = {});

    // Guidance overlays the flight plan: fields the law sets win over the
    // scripted values. Attached after construction (needs the target's id).
    // Throws if the guidance law emits a command level the control law does
    // not accept (vocabulary pairing is validated like the channel graph).
    void setGuidance(std::unique_ptr<GuidanceLaw> guidance);

    // Phase 1: next state from the shared snapshot. No mutation of state_.
    State propagate(const Environment& env, const WorldView& world, double dt);

    // Phase 2: adopt the state computed in phase 1 -- both the vehicle 6-DOF
    // state and the externalized component states (ADR-0003) staged by
    // propagate. Staged, not committed, so the step stays order-independent.
    void commit(const State& next) { state_ = next; compState_ = nextCompState_; }

    const State&      state() const     { return state_; }
    const Telemetry&  telemetry() const { return telem_; }
    const std::string& name() const     { return name_; }

    int  id() const        { return id_; }
    void setId(int id)     { id_ = id; telem_.id = id; }
    bool alive() const     { return alive_; }
    void kill()            { alive_ = false; }

private:
    std::string name_;
    int         id_ = -1;
    bool        alive_ = true;

    std::unique_ptr<Vehicle>           vehicle_;
    std::unique_ptr<ControlLaw>        controlLaw_;
    std::unique_ptr<ActuatorBank>      actuators_;
    std::unique_ptr<EquationsOfMotion> eom_;
    std::unique_ptr<GuidanceLaw>       guidance_;
    FlightPlan                         flightPlan_;

    ChannelTable  channels_;

    State     state_;
    Telemetry telem_;

    // Externalized component state (ADR-0003), integrated alongside state_ in
    // the augmented vector. compStateOffsets_[i] is component i's slice start;
    // total length = sum of numStates(). Empty while every component is
    // stateless (the current case), so all of this is a no-op then.
    std::vector<double> compState_;        // committed
    std::vector<double> nextCompState_;    // staged by propagate, adopted by commit
    std::vector<double> compRate_;         // xdot scratch (sized once)
    std::vector<int>    compStateOffsets_;
};
