#pragma once

#include <cstddef>
#include <string>

// Reads a fixed-size char field from the wire. Stops at the first NUL or at the
// field end (the sender's terminator is not trusted) and replaces control
// characters, '"' and '\\' so the text is safe inside log lines and JSON.
template <size_t N>
std::string wireText(const char (&field)[N])
{
    std::string text;
    for (size_t i = 0; i < N && field[i] != '\0'; ++i) {
        const unsigned char c = static_cast<unsigned char>(field[i]);
        text += (c < 0x20 || c == 0x7f || c == '"' || c == '\\') ? '?' : static_cast<char>(c);
    }
    return text;
}
