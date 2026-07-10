#pragma once

#include "core/Telemetry.h"

// Observer: receives every entity's telemetry once per step.
// Loggers, plotters, and scoring hooks implement this.
class SimObserver {
public:
    virtual ~SimObserver() = default;
    virtual void onStep(const Telemetry& telemetry) = 0;
    virtual void onFinish() {}   // called once when the run ends
};
