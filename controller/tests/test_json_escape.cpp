#include "jsonEscape.h"
#include "test_check.h"

#include <cstdio>
#include <string>

int main()
{
    CHECK(jsonEscape("") == "");
    CHECK(jsonEscape("session start: GPS offset +0.012 s; no step needed") == "session start: GPS offset +0.012 s; no step needed");
    CHECK(jsonEscape("/home/pi/Logs/Logs-RawCapture-2026-10-15_20-34-27/airspy-hf.1.dat")
          == "/home/pi/Logs/Logs-RawCapture-2026-10-15_20-34-27/airspy-hf.1.dat");

    CHECK(jsonEscape("say \"hi\"") == "say \\\"hi\\\"");
    CHECK(jsonEscape("C:\\logs") == "C:\\\\logs");
    CHECK(jsonEscape("a\nb\rc\td\be\ff") == "a\\nb\\rc\\td\\be\\ff");
    CHECK(jsonEscape(std::string("x\x01y\x1f", 4)) == "x\\u0001y\\u001f");
    CHECK(jsonEscape(std::string("nul\0end", 7)) == "nul\\u0000end");
    // DEL and UTF-8 are valid unescaped in JSON.
    CHECK(jsonEscape("\x7f") == "\x7f");
    CHECK(jsonEscape("caf\xc3\xa9 \xc2\xb0") == "caf\xc3\xa9 \xc2\xb0");

    std::printf("test_json_escape: all tests passed\n");
    return 0;
}
