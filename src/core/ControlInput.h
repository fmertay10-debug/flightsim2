#pragma once

// The actuator-level command vector a controller produces each step. It is a
// UNION of every control channel the project supports; a given vehicle uses
// only the channels its effectors read (fin-controlled vehicles use the
// surfaces, a TVC vehicle uses the gimbal, a future RCS vehicle its own).
// Surface/gimbal deflections are in RADIANS; throttle is dimensionless [0..1].
//
// Sign conventions:
//   elevator > 0 : trailing edge down -> nose-DOWN pitching moment
//   aileron  > 0 : right-roll moment (right aileron up)
//   rudder   > 0 : trailing edge left -> nose-LEFT yawing moment
//   tvcPitch > 0 : nozzle gimballed to give a nose-UP moment
//   tvcYaw   > 0 : nozzle gimballed to give a nose-RIGHT moment
struct ControlInput {
    // Aerodynamic control surfaces (read by the AeroModel).
    double elevator = 0.0;   // pitch surface [rad]
    double aileron  = 0.0;   // roll surface  [rad]
    double rudder   = 0.0;   // yaw surface   [rad]
    // Propulsion demand (read by the propulsion model).
    double throttle = 0.0;   // [0..1]
    // Thrust-vector gimbal (read by a TVC effector).
    double tvcPitch = 0.0;   // pitch gimbal [rad]
    double tvcYaw   = 0.0;   // yaw gimbal   [rad]
};
