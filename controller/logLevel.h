#pragma once

// Runtime switch for logVerbose() lines. Set from --verbose at startup and
// from the GCS via SET_LOG_LEVEL; kept apart from log.cpp so the command
// dispatcher (and its tests) need no logging backend.
void setVerboseLogging(bool enabled);
bool verboseLogging();
