#pragma once

#include <fstream>
#include <string>

#include "sim/SimObserver.h"

// Observer that appends one CSV row per step for ONE entity (filtered by id).
// Creates the parent directory if needed. Columns:
//   time, x, y, z, vx, vy, vz, phi, theta, psi, p, q, r,
//   elevator, aileron, rudder, throttle,
//   elevator_cmd, aileron_cmd, rudder_cmd, throttle_cmd,
//   alpha, beta, mach, airspeed, altitude, mass, thrust
class CsvLogger : public SimObserver {
public:
    CsvLogger(int entityId, const std::string& path, int decimation = 1);

    void onStep(const Telemetry& t) override;
    void onFinish() override;

private:
    int           entityId_;
    int           decimation_;   // log every Nth step
    long long     count_ = 0;
    std::ofstream out_;
};
