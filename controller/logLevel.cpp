#include "logLevel.h"

#include <atomic>

static std::atomic<bool> sVerboseLogging { false };

void setVerboseLogging(bool enabled) { sVerboseLogging.store(enabled); }
bool verboseLogging() { return sVerboseLogging.load(); }
