#pragma once

#include "effector/Effector.h"

// Thrust-vector control: the engine nozzle gimbals, deflecting the thrust
// vector off the body axis. The deflected thrust makes a side force at the
// nozzle (aft of the CG) and hence a control moment. Pitch and yaw only --
// a single nozzle produces no roll moment (roll needs fins or an RCS effector).
//
// The moment arm is (nozzle station - CG station), so it GROWS as the CG
// migrates forward during the burn -- the variable-mass model feeds straight
// in. Control authority is proportional to thrust, so TVC works only while the
// motor burns (zero authority after burnout, by construction).
//
// Sign convention (matches ControlInput): +tvcPitch -> nose-up (My>0),
// +tvcYaw -> nose-right (Mz>0).
class TvcEffector : public Effector {
public:
    TvcEffector(double nozzleStation, double maxGimbal)
        : nozzleStation_(nozzleStation), maxGimbal_(maxGimbal) {}

    Wrench compute(const EffectorContext& ctx, const ControlInput& u) const override;

private:
    double nozzleStation_;   // [m, nose datum, aft positive]
    double maxGimbal_;       // [rad]
};
