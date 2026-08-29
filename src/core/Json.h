// Minimal JSON reader for files THIS project's own tools emit (harvester output).
//
// vqview hand-rolled a depth-counted number scraper and its comment records the bug that approach
// produced (a naive first-']' scan silently returned 2 of 32 values). A tiny real parser is ~150
// lines, reusable, and cannot fail that way. It is tolerant of whitespace and \uXXXX escapes but
// is NOT a general-purpose validator -- garbage in, error string out, nothing clever.
#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <utility>

namespace ga {

struct JsonValue {
    enum class Type { Null, Bool, Number, String, Array, Object };
    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string str;
    std::vector<JsonValue> arr;
    std::vector<std::pair<std::string, JsonValue>> obj;

    const JsonValue* Get(const char* key) const {
        if (type != Type::Object) return nullptr;
        for (const auto& kv : obj) {
            if (kv.first == key) return &kv.second;
        }
        return nullptr;
    }
    double Num(const char* key, double def) const {
        const JsonValue* v = Get(key);
        return (v && v->type == Type::Number) ? v->number : def;
    }
    std::string Str(const char* key, const char* def = "") const {
        const JsonValue* v = Get(key);
        return (v && v->type == Type::String) ? v->str : std::string(def);
    }
};

class JsonParser {
public:
    static JsonValue Parse(const std::string& text, std::string* error) {
        JsonParser p(text);
        JsonValue v;
        if (!p.ParseValue(v) || (p.SkipWs(), p.m_pos != text.size())) {
            if (error) *error = p.m_error.empty() ? "trailing content" : p.m_error;
            return JsonValue{};
        }
        if (error) error->clear();
        return v;
    }

private:
    explicit JsonParser(const std::string& t) : m_text(t) {}

    const std::string& m_text;
    size_t m_pos = 0;
    std::string m_error;

    bool Fail(const char* what) {
        char buf[128];
        snprintf(buf, sizeof(buf), "%s at offset %zu", what, m_pos);
        if (m_error.empty()) m_error = buf;
        return false;
    }
    void SkipWs() {
        while (m_pos < m_text.size()) {
            const char c = m_text[m_pos];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') ++m_pos;
            else break;
        }
    }
    bool Consume(char c) {
        SkipWs();
        if (m_pos < m_text.size() && m_text[m_pos] == c) { ++m_pos; return true; }
        return false;
    }
    bool Literal(const char* lit) {
        const size_t n = strlen(lit);
        if (m_text.compare(m_pos, n, lit) == 0) { m_pos += n; return true; }
        return false;
    }

    bool ParseValue(JsonValue& out) {
        SkipWs();
        if (m_pos >= m_text.size()) return Fail("unexpected end");
        const char c = m_text[m_pos];
        if (c == '{') return ParseObject(out);
        if (c == '[') return ParseArray(out);
        if (c == '"') { out.type = JsonValue::Type::String; return ParseString(out.str); }
        if (c == 't') { if (!Literal("true")) return Fail("bad literal");
                        out.type = JsonValue::Type::Bool; out.boolean = true; return true; }
        if (c == 'f') { if (!Literal("false")) return Fail("bad literal");
                        out.type = JsonValue::Type::Bool; out.boolean = false; return true; }
        if (c == 'n') { if (!Literal("null")) return Fail("bad literal");
                        out.type = JsonValue::Type::Null; return true; }
        return ParseNumber(out);
    }

    bool ParseNumber(JsonValue& out) {
        const size_t start = m_pos;
        while (m_pos < m_text.size()) {
            const char c = m_text[m_pos];
            if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E')
                ++m_pos;
            else break;
        }
        if (m_pos == start) return Fail("expected a number");
        out.type = JsonValue::Type::Number;
        out.number = strtod(m_text.c_str() + start, nullptr);
        return true;
    }

    bool ParseString(std::string& out) {
        if (!Consume('"')) return Fail("expected '\"'");
        out.clear();
        while (m_pos < m_text.size()) {
            char c = m_text[m_pos++];
            if (c == '"') return true;
            if (c != '\\') { out.push_back(c); continue; }
            if (m_pos >= m_text.size()) return Fail("bad escape");
            const char e = m_text[m_pos++];
            switch (e) {
                case '"': out.push_back('"'); break;
                case '\\': out.push_back('\\'); break;
                case '/': out.push_back('/'); break;
                case 'b': out.push_back('\b'); break;
                case 'f': out.push_back('\f'); break;
                case 'n': out.push_back('\n'); break;
                case 'r': out.push_back('\r'); break;
                case 't': out.push_back('\t'); break;
                case 'u': {
                    if (m_pos + 4 > m_text.size()) return Fail("bad \\u escape");
                    unsigned cp = 0;
                    for (int i = 0; i < 4; ++i) {
                        const char h = m_text[m_pos++];
                        cp <<= 4;
                        if (h >= '0' && h <= '9') cp |= static_cast<unsigned>(h - '0');
                        else if (h >= 'a' && h <= 'f') cp |= static_cast<unsigned>(h - 'a' + 10);
                        else if (h >= 'A' && h <= 'F') cp |= static_cast<unsigned>(h - 'A' + 10);
                        else return Fail("bad \\u digit");
                    }
                    // UTF-8 encode the BMP code point. Surrogate pairs are not expected in our own
                    // emitted data; a lone surrogate encodes as-is, which is harmless for display.
                    if (cp < 0x80) out.push_back(static_cast<char>(cp));
                    else if (cp < 0x800) {
                        out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
                        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                    } else {
                        out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
                        out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
                        out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                    }
                    break;
                }
                default: return Fail("unknown escape");
            }
        }
        return Fail("unterminated string");
    }

    bool ParseArray(JsonValue& out) {
        if (!Consume('[')) return Fail("expected '['");
        out.type = JsonValue::Type::Array;
        SkipWs();
        if (Consume(']')) return true;
        for (;;) {
            JsonValue v;
            if (!ParseValue(v)) return false;
            out.arr.push_back(std::move(v));
            if (Consume(',')) continue;
            if (Consume(']')) return true;
            return Fail("expected ',' or ']'");
        }
    }

    bool ParseObject(JsonValue& out) {
        if (!Consume('{')) return Fail("expected '{'");
        out.type = JsonValue::Type::Object;
        SkipWs();
        if (Consume('}')) return true;
        for (;;) {
            SkipWs();
            std::string key;
            if (!ParseString(key)) return false;
            if (!Consume(':')) return Fail("expected ':'");
            JsonValue v;
            if (!ParseValue(v)) return false;
            out.obj.emplace_back(std::move(key), std::move(v));
            if (Consume(',')) continue;
            if (Consume('}')) return true;
            return Fail("expected ',' or '}'");
        }
    }
};

}  // namespace ga
