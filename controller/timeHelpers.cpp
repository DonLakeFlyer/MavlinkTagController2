#include "timeHelpers.h"

#include <chrono>

uint64_t msecsSinceEpoch()
{
    return uint64_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
}

double secondsSinceEpoch()
{
    return (double)msecsSinceEpoch() / 1000.0;
}
