#include "control/FlightPlan.h"

#include <algorithm>

#include "math/Units.h"

namespace {
    constexpr double DEG = units::PI / 180.0;
}

FlightPlan FlightPlan::fromJson(const json::Value& array) {
    FlightPlan plan;
    for (std::size_t i = 0; i < array.size(); ++i) {
        const json::Value& s = array[i];
        Segment seg;
        seg.time = s.num("time_s", 0.0);
        if (s.has("pitch_deg"))   seg.cmd.pitch    = s.num("pitch_deg") * DEG;
        if (s.has("roll_deg"))    seg.cmd.roll     = s.num("roll_deg") * DEG;
        if (s.has("heading_deg")) seg.cmd.heading  = s.num("heading_deg") * DEG;
        if (s.has("altitude_m"))  seg.cmd.altitude = s.num("altitude_m");
        if (s.has("speed_ms"))    seg.cmd.speed    = s.num("speed_ms");
        if (s.has("throttle"))    seg.cmd.throttle = s.num("throttle");
        plan.segments_.push_back(seg);
    }
    std::stable_sort(plan.segments_.begin(), plan.segments_.end(),
                     [](const Segment& a, const Segment& b) { return a.time < b.time; });
    return plan;
}

CommandSet FlightPlan::at(double time) const {
    CommandSet out;
    for (const Segment& seg : segments_) {
        if (seg.time > time) break;
        if (seg.cmd.pitch)    out.pitch    = seg.cmd.pitch;
        if (seg.cmd.roll)     out.roll     = seg.cmd.roll;
        if (seg.cmd.heading)  out.heading  = seg.cmd.heading;
        if (seg.cmd.altitude) out.altitude = seg.cmd.altitude;
        if (seg.cmd.speed)    out.speed    = seg.cmd.speed;
        if (seg.cmd.throttle) out.throttle = seg.cmd.throttle;
    }
    return out;
}
