#pragma once

#include <cstdlib>
#include <cstring>

// The controller is deployed on the rPi under the "pi" user, so HOME is the
// cheapest reliable discriminator between the vehicle and a dev/SITL host.
inline bool isRunningOnRPi()
{
    const char* home = getenv("HOME");
    return home != nullptr && strcmp(home, "/home/pi") == 0;
}
