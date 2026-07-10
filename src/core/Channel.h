#pragma once

#include <array>
#include <string>
#include <vector>

// Named actuator channels -- the contract between the control side (which
// WRITES channels by name) and the force-producing side (which DECLARES and
// READS them). Replaces the old fixed ControlInput union.
//
// Lifecycle: at LOAD time the scenario loader asks each force-producing
// component (aero model, effectors, propulsion) to declare the channels it
// consumes into the entity's ChannelTable, then asks the controller to bind
// the channels it writes. Unknown names fail loudly there -- a controller can
// no longer silently drive channels nothing reads. At RUN time everything is
// index-based through ChannelHandle; no string lookups in the sim loop.
//
// Units/signs are the channel owner's contract (deflections in radians,
// throttle dimensionless [0..1]); the owner documents them where it declares.

// Canonical names of the standard channels (a vehicle declares only the ones
// its components consume; new components may declare entirely new names).
// Signs/units are the declaring component's contract:
//   elevator > 0 : trailing edge down -> nose-DOWN pitching moment   [rad]
//   aileron  > 0 : right-roll moment (right aileron up)              [rad]
//   rudder   > 0 : trailing edge left -> nose-LEFT yawing moment     [rad]
//   throttle     : propulsion demand                                 [0..1]
//   tvc_pitch> 0 : nozzle gimballed to give a nose-UP moment         [rad]
//   tvc_yaw  > 0 : nozzle gimballed to give a nose-RIGHT moment      [rad]
namespace channels {
inline constexpr const char* kElevator = "elevator";
inline constexpr const char* kAileron  = "aileron";
inline constexpr const char* kRudder   = "rudder";
inline constexpr const char* kThrottle = "throttle";
inline constexpr const char* kTvcPitch = "tvc_pitch";
inline constexpr const char* kTvcYaw   = "tvc_yaw";
} // namespace channels

// Selects which parameter set of the vehicle's "actuator" config block
// applies to the channel (see ActuatorBank).
enum class ChannelKind {
    Surface,    // aerodynamic surface: tau_s / rate_dps / limit_deg
    Gimbal,     // thrust gimbal: tau_s / rate_dps / gimbal_limit_deg
    Throttle,   // propulsion demand: throttle_tau_s, clamped [0..1]
};

struct ChannelDef {
    std::string name;               // snake_case, e.g. "elevator", "tvc_pitch"
    ChannelKind kind = ChannelKind::Surface;
    double minValue = 0.0;          // hard position limits (min <= 0 <= max)
    double maxValue = 0.0;
};

// Index into a ChannelTable / ChannelValues. Default-constructed = invalid;
// reads through an invalid handle yield 0 and writes are dropped, so a
// channel a controller binds optionally (find) needs no per-step guards.
struct ChannelHandle {
    int index = -1;
    bool valid() const { return index >= 0; }
};

class ChannelValues;

// One entity's channel set, built once at load and immutable afterwards.
class ChannelTable {
public:
    static constexpr int kMaxChannels = 16;   // ChannelValues inline capacity

    // Declare a channel (components own their channels). Throws on a
    // duplicate name or when kMaxChannels is exceeded.
    ChannelHandle add(const ChannelDef& def);

    // Invalid handle if the name is not declared.
    ChannelHandle find(const std::string& name) const;

    // Like find(), but throws with a message listing every declared channel.
    // Controllers use this for the channels they cannot fly without.
    ChannelHandle require(const std::string& name) const;

    int size() const { return static_cast<int>(defs_.size()); }
    const ChannelDef& def(int index) const { return defs_[index]; }

private:
    std::vector<ChannelDef> defs_;
};

// The runtime command/position vector: one value per declared channel.
// Fixed inline storage -- no heap traffic in the sim loop.
class ChannelValues {
public:
    ChannelValues() = default;
    explicit ChannelValues(const ChannelTable& table)
        : count_(table.size()) {}

    double get(ChannelHandle h) const {
        return (h.valid() && h.index < count_) ? values_[h.index] : 0.0;
    }
    void set(ChannelHandle h, double value) {
        if (h.valid() && h.index < count_) values_[h.index] = value;
    }

    int size() const { return count_; }
    double at(int index) const { return values_[index]; }
    void setAt(int index, double value) { values_[index] = value; }

private:
    std::array<double, ChannelTable::kMaxChannels> values_{};
    int count_ = 0;
};
