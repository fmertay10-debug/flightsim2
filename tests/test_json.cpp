#include "test_util.h"

#include <stdexcept>
#include <string>

#include "io/Json.h"

int main() {
    const std::string text = R"({
        // a comment
        "name": "demo",
        "count": 3,
        "scale": -1.5e2,
        "on": true,
        "off": false,
        "nothing": null,
        "vec": [1, 2, 3],
        "nested": { "a": { "b": [ {"c": 42} ] } },
        "escaped": "line\nbreak \"quoted\""
    })";

    const json::Value v = json::Value::parse(text);

    CHECK(v.isObject());
    CHECK(v.str("name") == "demo");
    CHECK_NEAR(v.num("count"), 3.0, 1e-12);
    CHECK_NEAR(v.num("scale"), -150.0, 1e-12);
    CHECK(v.boolean("on", false) == true);
    CHECK(v.boolean("off", true) == false);
    CHECK(v.at("nothing").isNull());
    CHECK(v.num("missing", 7.5) == 7.5);
    CHECK(v.str("missing", "dflt") == "dflt");

    const Vector3 vec = v.vec3("vec");
    CHECK_NEAR(vec.y, 2.0, 1e-12);

    CHECK_NEAR(v.at("nested").at("a").at("b")[0].num("c"), 42.0, 1e-12);
    CHECK(v.str("escaped") == "line\nbreak \"quoted\"");

    // Missing required key throws with the key name in the message.
    bool threw = false;
    try {
        v.num("does_not_exist");
    } catch (const std::runtime_error& e) {
        threw = std::string(e.what()).find("does_not_exist") != std::string::npos;
    }
    CHECK(threw);

    // Malformed input throws.
    threw = false;
    try {
        json::Value::parse("{\"a\": }");
    } catch (const std::exception&) {
        threw = true;
    }
    CHECK(threw);

    std::printf("test_json: all checks passed\n");
    return 0;
}
