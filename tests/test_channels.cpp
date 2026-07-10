#include <stdexcept>
#include <string>

#include "core/Channel.h"
#include "test_util.h"

int main() {
    // Declaration and lookup.
    ChannelTable t;
    const ChannelHandle elev = t.add({"elevator", ChannelKind::Surface, -0.4, 0.4});
    const ChannelHandle thr  = t.add({"throttle", ChannelKind::Throttle, 0.0, 1.0});
    CHECK(elev.valid());
    CHECK(t.size() == 2);
    CHECK(t.find("elevator").index == elev.index);
    CHECK(!t.find("rudder").valid());
    CHECK(t.require("throttle").index == thr.index);
    CHECK(t.def(elev.index).kind == ChannelKind::Surface);

    // Duplicate declaration throws.
    bool threw = false;
    try {
        t.add({"elevator", ChannelKind::Surface, -1.0, 1.0});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    // require() on a missing channel throws and lists what IS declared.
    threw = false;
    try {
        t.require("tvc_pitch");
    } catch (const std::invalid_argument& e) {
        threw = true;
        const std::string msg = e.what();
        CHECK(msg.find("tvc_pitch") != std::string::npos);
        CHECK(msg.find("elevator") != std::string::npos);
        CHECK(msg.find("throttle") != std::string::npos);
    }
    CHECK(threw);

    // Values: reads/writes through handles; invalid handles are inert.
    ChannelValues v(t);
    CHECK(v.size() == 2);
    CHECK_NEAR(v.get(elev), 0.0, 0.0);
    v.set(elev, 0.25);
    CHECK_NEAR(v.get(elev), 0.25, 0.0);
    const ChannelHandle missing = t.find("rudder");
    v.set(missing, 9.9);                    // dropped
    CHECK_NEAR(v.get(missing), 0.0, 0.0);   // reads as 0

    // Capacity guard.
    ChannelTable full;
    for (int i = 0; i < ChannelTable::kMaxChannels; ++i)
        full.add({"ch" + std::to_string(i), ChannelKind::Surface, -1.0, 1.0});
    threw = false;
    try {
        full.add({"overflow", ChannelKind::Surface, -1.0, 1.0});
    } catch (const std::length_error&) {
        threw = true;
    }
    CHECK(threw);

    std::printf("test_channels: all checks passed\n");
    return 0;
}
