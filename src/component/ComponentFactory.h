#pragma once

#include <functional>
#include <memory>
#include <string>

#include "component/ForceComponent.h"
#include "io/Json.h"

// The ONE registry for force components -- every entry in a vehicle's
// "components" array names its implementation here explicitly via "type"
// (no key-sniffing, no vehicle-type defaults). Built-in types:
//
//   aero:   "aircraft_aero"                        (point derivatives)
//           "f16_aero" | "rocket_table_aero"       (lookup tables)
//   motors: "turbojet" | "solid_motor" | "tabulated_thrust" | "f16_engine"
//           (each takes an optional "gimbal" block -> thrust-vectoring mount:
//            {"nozzle_station_m": 6.0, "max_gimbal_deg": 6})
//
// EXTENSION POINT: registerComponent("my_type", builder) -- the builder gets
// the component's config object and the vehicle definition's base directory
// for resolving relative data paths.
namespace component {

class Factory {
public:
    using Builder = std::function<std::unique_ptr<ForceComponent>(
        const json::Value&, const std::string&)>;

    static void registerComponent(const std::string& type, Builder builder);

    // Reads config["type"]; throws with a listing of registered types.
    static std::unique_ptr<ForceComponent> create(const json::Value& config,
                                                  const std::string& baseDir);
};

} // namespace component
