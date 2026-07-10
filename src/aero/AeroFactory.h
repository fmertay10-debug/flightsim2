#pragma once

#include <functional>
#include <memory>
#include <string>

#include "aero/AeroModel.h"
#include "io/Json.h"

// Registry-based Factory for aero models, keyed by vehicle type name.
// Built-in types ("aircraft", "rocket") are registered automatically.
//
// EXTENSION POINT: to add a vehicle type, implement an AeroModel subclass and
// register its builder once at startup:
//     aero::Factory::registerModel("quadcopter", QuadAero::fromJson);
namespace aero {

class Factory {
public:
    // baseDir: directory of the vehicle definition file, for resolving
    // relative data paths (e.g. DATCOM table CSVs).
    using Builder = std::function<std::unique_ptr<AeroModel>(const json::Value&,
                                                             const std::string& baseDir)>;

    static void registerModel(const std::string& type, Builder builder);

    // Builds the aero model for `type` from its "aero" config block.
    // Throws std::invalid_argument for unknown types.
    static std::unique_ptr<AeroModel> create(const std::string& type,
                                             const json::Value& config,
                                             const std::string& baseDir);
};

} // namespace aero
