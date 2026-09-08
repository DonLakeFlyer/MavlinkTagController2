#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <pwd.h>
#include <unistd.h>

// $HOME, falling back to the passwd entry when the environment is stripped
// (e.g. a cron or systemd unit without HOME). Aborts rather than returning
// an empty path: every log/config location is rooted here.
inline const std::string& homeDir()
{
    static const std::string home = [] {
        if (const char* env = getenv("HOME"); env != nullptr && *env != '\0') {
            return std::string(env);
        }
        if (const passwd* pw = getpwuid(getuid()); pw != nullptr && pw->pw_dir != nullptr && *pw->pw_dir != '\0') {
            return std::string(pw->pw_dir);
        }
        std::fprintf(stderr, "Fatal: cannot determine home directory (HOME unset, no passwd entry)\n");
        std::abort();
    }();
    return home;
}

// The controller is deployed on the rPi under the "pi" user, so HOME is the
// cheapest reliable discriminator between the vehicle and a dev/SITL host.
inline bool isRunningOnRPi()
{
    return homeDir() == "/home/pi";
}
