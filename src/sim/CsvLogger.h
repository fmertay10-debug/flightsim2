#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "core/Channel.h"
#include "sim/SimObserver.h"

// Observer that appends one CSV row per step for ONE entity (filtered by id).
// Creates the parent directory if needed. Columns:
//   time, x, y, z, vx, vy, vz, phi, theta, psi, p, q, r,
//   elevator, aileron, rudder, throttle,
//   elevator_cmd, aileron_cmd, rudder_cmd, throttle_cmd,
//   tvc_pitch, tvc_yaw,
//   [<extra channel>, <extra channel>_cmd, ...]
//   alpha, beta, mach, airspeed, altitude, mass, thrust,
//   pitch_sp, roll_sp, heading_sp, altitude_sp, speed_sp,
//   qbar,
//   [<component>_fx, _fy, _fz, _mx, _my, _mz, ...]   (per force component,
//                                                     CG-referenced wrench)
// The standard channel columns are looked up by NAME in the entity's channel
// table (0 when the vehicle doesn't declare them, matching the old fixed
// struct); channels beyond the standard six get their own appended columns.
// The header is written on the first row, when the table is known.
class CsvLogger : public SimObserver {
public:
    CsvLogger(int entityId, const std::string& path, int decimation = 1);

    void onStep(const Telemetry& t) override;
    void onFinish() override;

private:
    void writeHeader(const Telemetry* t);

    int           entityId_;
    int           decimation_;   // log every Nth step
    long long     count_ = 0;
    bool          headerWritten_ = false;
    ChannelHandle elevator_, aileron_, rudder_, throttle_, tvcPitch_, tvcYaw_;
    std::vector<int> extras_;    // table indices of non-standard channels
    int nComponents_ = 0;        // per-component wrench column sets
    std::ofstream out_;
};
