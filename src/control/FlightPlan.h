#pragma once

#include <vector>

#include "control/CommandSet.h"
#include "io/Json.h"

// A time-ordered list of command segments -- the scenario's script for one
// vehicle. Segments MERGE cumulatively: a segment only overrides the fields
// it specifies, everything else carries over from earlier segments.
//
// JSON (angles in degrees, converted here):
//   "flight_plan": [
//     {"time_s": 0,  "altitude_m": 1000, "heading_deg": 0, "speed_ms": 60},
//     {"time_s": 30, "heading_deg": 90}
//   ]
class FlightPlan {
public:
    FlightPlan() = default;

    static FlightPlan fromJson(const json::Value& array);

    // Active command set at time t (all segments with time <= t, merged).
    CommandSet at(double time) const;

    bool empty() const { return segments_.empty(); }

private:
    struct Segment {
        double     time = 0.0;
        CommandSet cmd;
    };
    std::vector<Segment> segments_;   // sorted ascending by time
};
