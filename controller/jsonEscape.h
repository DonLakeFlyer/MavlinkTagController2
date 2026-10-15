#pragma once

#include <cstdio>
#include <string>

// Escapes text for use inside a JSON string literal (RFC 8259): quote,
// backslash and control characters. Other bytes, including UTF-8, pass through.
inline std::string jsonEscape(const std::string& text)
{
    std::string escaped;
    escaped.reserve(text.size());
    for (const char c : text) {
        switch (c) {
        case '"':  escaped += "\\\""; break;
        case '\\': escaped += "\\\\"; break;
        case '\b': escaped += "\\b";  break;
        case '\f': escaped += "\\f";  break;
        case '\n': escaped += "\\n";  break;
        case '\r': escaped += "\\r";  break;
        case '\t': escaped += "\\t";  break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(c)));
                escaped += buf;
            } else {
                escaped += c;
            }
            break;
        }
    }
    return escaped;
}
