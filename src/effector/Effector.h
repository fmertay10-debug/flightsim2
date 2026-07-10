#pragma once

#include "core/AirData.h"
#include "core/ControlInput.h"
#include "core/State.h"
#include "core/Wrench.h"

// Everything an effector may need at one step, assembled by the Entity.
struct EffectorContext {
    const State&   state;
    const AirData& air;
    double thrust = 0.0;   // available thrust magnitude this step [N]
    double xcg    = 0.0;   // CG station [m, nose datum, aft positive] for arms
};

// Strategy: a control EFFECTOR -- a discrete actuator that turns control
// commands + flight condition into a body-frame wrench (about the CG). This is
// how NON-aerodynamic control enters the sim; aerodynamic control surfaces act
// through the AeroModel instead (they change the airflow, not push directly).
//
// The Entity holds a LIST of effectors and sums their wrenches, so adding a
// new control method (TVC, RCS, ...) is: implement an Effector + register it in
// effector::Factory, and add a matching Controller method. Nothing else in the
// core changes.
//
// Concrete: ThrustEffector (axial thrust), TvcEffector (gimbaled thrust).
class Effector {
public:
    virtual ~Effector() = default;
    virtual Wrench compute(const EffectorContext& ctx, const ControlInput& u) const = 0;
};
