#include "core/Channel.h"

#include <stdexcept>

ChannelHandle ChannelTable::add(const ChannelDef& def) {
    if (find(def.name).valid())
        throw std::invalid_argument("channel '" + def.name + "' declared twice");
    if (size() >= kMaxChannels)
        throw std::length_error("more than " + std::to_string(kMaxChannels) +
                                " channels declared (raise ChannelTable::kMaxChannels)");
    defs_.push_back(def);
    return ChannelHandle{size() - 1};
}

ChannelHandle ChannelTable::find(const std::string& name) const {
    for (int i = 0; i < size(); ++i)
        if (defs_[i].name == name) return ChannelHandle{i};
    return ChannelHandle{};
}

ChannelHandle ChannelTable::require(const std::string& name) const {
    const ChannelHandle h = find(name);
    if (h.valid()) return h;
    std::string declared;
    for (const auto& d : defs_) {
        if (!declared.empty()) declared += ", ";
        declared += d.name;
    }
    if (declared.empty()) declared = "<none>";
    throw std::invalid_argument(
        "controller writes channel '" + name +
        "' but no component on this vehicle declares it (declared: " +
        declared + ")");
}
