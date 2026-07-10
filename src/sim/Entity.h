#pragma once

#include <memory>
#include <string>
#include <vector>

#include "aero/AeroModel.h"
#include "control/ActuatorBank.h"
#include "control/Controller.h"
#include "control/FlightPlan.h"
#include "core/Channel.h"
#include "core/State.h"
#include "core/Telemetry.h"
#include "dynamics/EquationsOfMotion.h"
#include "effector/Effector.h"
#include "environment/Environment.h"
#include "guidance/GuidanceLaw.h"
#include "vehicle/Vehicle.h"

// One vehicle's complete simulation stack: vehicle properties + aero +
// controller + actuators + EOM + flight plan, plus its kinematic state.
// Composition via Strategy interfaces -- any part can be swapped or null:
//   null aero       -> no aerodynamic loads (e.g. kinematic targets)
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
           std::unique_ptr<AeroModel>         aero,
           std::unique_ptr<Controller>        controller,
           std::unique_ptr<ActuatorBank>      actuators,
           std::unique_ptr<EquationsOfMotion> eom,
           FlightPlan                         flightPlan,
           const State&                       initialState,
           ChannelTable                       channels = {});

    // Guidance overlays the flight plan: fields the law sets win over the
    // scripted values. Attached after construction (needs the target's id).
    void setGuidance(std::unique_ptr<GuidanceLaw> guidance) {
        guidance_ = std::move(guidance);
    }

    // Control effectors (thrust application: axial or TVC, plus future RCS).
    // Empty is fine (an unpowered/kinematic entity). Set at construction time.
    void setEffectors(std::vector<std::unique_ptr<Effector>> effectors) {
        effectors_ = std::move(effectors);
    }

    // Phase 1: next state from the shared snapshot. No mutation of state_.
    State propagate(const Environment& env, const WorldView& world, double dt);

    // Phase 2: adopt the state computed in phase 1.
    void commit(const State& next) { state_ = next; }

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
    std::unique_ptr<AeroModel>         aero_;
    std::unique_ptr<Controller>        controller_;
    std::unique_ptr<ActuatorBank>      actuators_;
    std::unique_ptr<EquationsOfMotion> eom_;
    std::unique_ptr<GuidanceLaw>       guidance_;
    std::vector<std::unique_ptr<Effector>> effectors_;
    FlightPlan                         flightPlan_;

    ChannelTable  channels_;
    ChannelHandle throttle_;   // propulsion demand (invalid if not declared)

    State     state_;
    Telemetry telem_;
};
