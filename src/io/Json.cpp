#include "io/Json.h"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace json {

namespace {

[[noreturn]] void fail(const std::string& msg) {
    throw std::runtime_error("json: " + msg);
}

} // namespace

// ---------------------------------------------------------------- Parser

class Parser {
public:
    explicit Parser(const std::string& text) : s_(text) {}

    Value parse() {
        Value v = parseValue();
        skipWhitespace();
        if (pos_ != s_.size()) fail(where() + ": trailing characters after JSON value");
        return v;
    }

private:
    const std::string& s_;
    std::size_t pos_ = 0;
    int line_ = 1;

    std::string where() const { return "line " + std::to_string(line_); }

    char peek() {
        if (pos_ >= s_.size()) fail(where() + ": unexpected end of input");
        return s_[pos_];
    }

    char next() {
        const char c = peek();
        ++pos_;
        if (c == '\n') ++line_;
        return c;
    }

    void skipWhitespace() {
        while (pos_ < s_.size()) {
            const char c = s_[pos_];
            if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
                next();
            } else if (c == '/' && pos_ + 1 < s_.size() && s_[pos_ + 1] == '/') {
                while (pos_ < s_.size() && s_[pos_] != '\n') next();
            } else {
                break;
            }
        }
    }

    void expect(char c) {
        if (next() != c) fail(where() + ": expected '" + std::string(1, c) + "'");
    }

    Value parseValue() {
        skipWhitespace();
        const char c = peek();
        switch (c) {
            case '{': return parseObject();
            case '[': return parseArray();
            case '"': { Value v; v.type_ = Value::Type::String; v.string_ = parseString(); return v; }
            case 't': case 'f': return parseBool();
            case 'n': return parseNull();
            default:  return parseNumber();
        }
    }

    Value parseObject() {
        Value v;
        v.type_ = Value::Type::Object;
        expect('{');
        skipWhitespace();
        if (peek() == '}') { next(); return v; }
        while (true) {
            skipWhitespace();
            std::string key = parseString();
            skipWhitespace();
            expect(':');
            v.object_.emplace_back(std::move(key), parseValue());
            skipWhitespace();
            const char c = next();
            if (c == '}') break;
            if (c != ',') fail(where() + ": expected ',' or '}' in object");
        }
        return v;
    }

    Value parseArray() {
        Value v;
        v.type_ = Value::Type::Array;
        expect('[');
        skipWhitespace();
        if (peek() == ']') { next(); return v; }
        while (true) {
            v.array_.push_back(parseValue());
            skipWhitespace();
            const char c = next();
            if (c == ']') break;
            if (c != ',') fail(where() + ": expected ',' or ']' in array");
        }
        return v;
    }

    std::string parseString() {
        expect('"');
        std::string out;
        while (true) {
            const char c = next();
            if (c == '"') break;
            if (c == '\\') {
                const char e = next();
                switch (e) {
                    case '"':  out += '"';  break;
                    case '\\': out += '\\'; break;
                    case '/':  out += '/';  break;
                    case 'b':  out += '\b'; break;
                    case 'f':  out += '\f'; break;
                    case 'n':  out += '\n'; break;
                    case 'r':  out += '\r'; break;
                    case 't':  out += '\t'; break;
                    case 'u': {
                        // \uXXXX: decode BMP code point to UTF-8 (no surrogate pairs).
                        unsigned cp = 0;
                        for (int i = 0; i < 4; ++i) {
                            const char h = next();
                            cp <<= 4;
                            if (h >= '0' && h <= '9') cp += h - '0';
                            else if (h >= 'a' && h <= 'f') cp += 10 + h - 'a';
                            else if (h >= 'A' && h <= 'F') cp += 10 + h - 'A';
                            else fail(where() + ": bad \\u escape");
                        }
                        if (cp < 0x80) {
                            out += static_cast<char>(cp);
                        } else if (cp < 0x800) {
                            out += static_cast<char>(0xC0 | (cp >> 6));
                            out += static_cast<char>(0x80 | (cp & 0x3F));
                        } else {
                            out += static_cast<char>(0xE0 | (cp >> 12));
                            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                            out += static_cast<char>(0x80 | (cp & 0x3F));
                        }
                        break;
                    }
                    default: fail(where() + ": bad escape character");
                }
            } else {
                out += c;
            }
        }
        return out;
    }

    Value parseNumber() {
        const std::size_t start = pos_;
        if (peek() == '-') next();
        while (pos_ < s_.size() && (std::isdigit(static_cast<unsigned char>(s_[pos_]))
               || s_[pos_] == '.' || s_[pos_] == 'e' || s_[pos_] == 'E'
               || s_[pos_] == '+' || s_[pos_] == '-')) {
            next();
        }
        const std::string tok = s_.substr(start, pos_ - start);
        char* end = nullptr;
        const double d = std::strtod(tok.c_str(), &end);
        if (end == tok.c_str() || *end != '\0')
            fail(where() + ": invalid number '" + tok + "'");
        Value v;
        v.type_ = Value::Type::Number;
        v.number_ = d;
        return v;
    }

    Value parseBool() {
        Value v;
        v.type_ = Value::Type::Bool;
        if (s_.compare(pos_, 4, "true") == 0)       { pos_ += 4; v.bool_ = true;  }
        else if (s_.compare(pos_, 5, "false") == 0) { pos_ += 5; v.bool_ = false; }
        else fail(where() + ": invalid literal");
        return v;
    }

    Value parseNull() {
        if (s_.compare(pos_, 4, "null") != 0) fail(where() + ": invalid literal");
        pos_ += 4;
        return Value();
    }
};

// ---------------------------------------------------------------- Value

Value Value::parse(const std::string& text) {
    return Parser(text).parse();
}

Value Value::parseFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) fail("cannot open file '" + path + "'");
    std::ostringstream ss;
    ss << in.rdbuf();
    try {
        return parse(ss.str());
    } catch (const std::exception& e) {
        fail(std::string(e.what()) + " (in file '" + path + "')");
    }
}

bool Value::asBool() const {
    if (type_ != Type::Bool) fail("value is not a bool");
    return bool_;
}

double Value::asNumber() const {
    if (type_ != Type::Number) fail("value is not a number");
    return number_;
}

const std::string& Value::asString() const {
    if (type_ != Type::String) fail("value is not a string");
    return string_;
}

std::size_t Value::size() const {
    if (type_ == Type::Array)  return array_.size();
    if (type_ == Type::Object) return object_.size();
    fail("size() on non-container value");
}

const Value& Value::operator[](std::size_t i) const {
    if (type_ != Type::Array) fail("operator[] on non-array value");
    if (i >= array_.size())   fail("array index out of range");
    return array_[i];
}

bool Value::has(const std::string& key) const {
    if (type_ != Type::Object) return false;
    for (const auto& kv : object_)
        if (kv.first == key) return true;
    return false;
}

const Value& Value::at(const std::string& key) const {
    if (type_ != Type::Object) fail("at('" + key + "') on non-object value");
    for (const auto& kv : object_)
        if (kv.first == key) return kv.second;
    fail("missing required key '" + key + "'");
}

const std::vector<std::pair<std::string, Value>>& Value::members() const {
    if (type_ != Type::Object) fail("members() on non-object value");
    return object_;
}

double Value::num(const std::string& key) const { return at(key).asNumber(); }
double Value::num(const std::string& key, double def) const {
    return has(key) ? at(key).asNumber() : def;
}

std::string Value::str(const std::string& key) const { return at(key).asString(); }
std::string Value::str(const std::string& key, const std::string& def) const {
    return has(key) ? at(key).asString() : def;
}

bool Value::boolean(const std::string& key, bool def) const {
    return has(key) ? at(key).asBool() : def;
}

Vector3 Value::vec3(const std::string& key) const {
    const Value& a = at(key);
    if (!a.isArray() || a.size() != 3)
        fail("key '" + key + "' must be an array of 3 numbers");
    return { a[0].asNumber(), a[1].asNumber(), a[2].asNumber() };
}

Vector3 Value::vec3(const std::string& key, const Vector3& def) const {
    return has(key) ? vec3(key) : def;
}

} // namespace json
