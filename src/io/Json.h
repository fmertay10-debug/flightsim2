#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "math/Vector3.h"

// Minimal JSON parser (stdlib only) for scenario and vehicle config files.
// Supports the full JSON grammar plus // line comments. No trailing commas.
//
// Access style: strict accessors throw std::runtime_error with the offending
// key in the message, so a broken config fails loudly with a useful error.
namespace json {

class Value {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    Value() : type_(Type::Null) {}

    // --- Parsing (Factory entry points) ---
    static Value parse(const std::string& text);
    static Value parseFile(const std::string& path);   // throws if unreadable

    // --- Type queries ---
    Type type()     const { return type_; }
    bool isNull()   const { return type_ == Type::Null; }
    bool isBool()   const { return type_ == Type::Bool; }
    bool isNumber() const { return type_ == Type::Number; }
    bool isString() const { return type_ == Type::String; }
    bool isArray()  const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }

    // --- Scalar access (throws on type mismatch) ---
    bool               asBool()   const;
    double             asNumber() const;
    const std::string& asString() const;

    // --- Array access ---
    std::size_t  size() const;                    // array/object element count
    const Value& operator[](std::size_t i) const; // array element (throws OOB)

    // --- Object access ---
    bool         has(const std::string& key) const;
    const Value& at(const std::string& key)  const;  // required key (throws)
    const std::vector<std::pair<std::string, Value>>& members() const;

    // --- Convenience accessors for config code ---
    double      num(const std::string& key) const;                  // required
    double      num(const std::string& key, double def) const;      // optional
    std::string str(const std::string& key) const;                  // required
    std::string str(const std::string& key, const std::string& def) const;
    bool        boolean(const std::string& key, bool def) const;
    Vector3     vec3(const std::string& key) const;                 // required [x,y,z]
    Vector3     vec3(const std::string& key, const Vector3& def) const;

private:
    Type type_;
    bool                                       bool_ = false;
    double                                     number_ = 0.0;
    std::string                                string_;
    std::vector<Value>                         array_;
    // Insertion-ordered object members (small configs, linear scan is fine).
    std::vector<std::pair<std::string, Value>> object_;

    friend class Parser;
};

} // namespace json
