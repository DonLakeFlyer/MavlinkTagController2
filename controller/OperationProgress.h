#pragma once

#include "TunnelProtocol.h"

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

// Tracks the one long-running operation the controller allows at a time and
// publishes its OperationProgress_t frames. begin() doubles as the busy gate:
// it refuses while another operation is RUNNING, and TunnelCommandDispatcher
// NACKs the second command with busyMessage().
class OperationProgressReporter {
public:
    /// Both run with the reporter's mutex held: keep them short, never re-enter the reporter.
    using SendFn = std::function<void(const TunnelProtocol::OperationProgress_t&)>;
    using LogFn  = std::function<void(const std::string&)>;

    explicit OperationProgressReporter(SendFn send = {}, LogFn log = {});

    /// title is shown by the GCS and used in busyMessage(). stepCount 0 = indeterminate.
    /// Returns false, sending nothing, if another operation is running.
    bool begin(uint32_t command, uint32_t requestId, const std::string& title, uint32_t stepCount = 0);
    /// Sends only if step or message changed. Empty message keeps the current one.
    void update(uint32_t step, const std::string& message = "");
    /// As above, also setting step_count for operations that learn it after begin().
    void update(uint32_t step, uint32_t stepCount, const std::string& message);
    void finish(bool success, const std::string& message = "");
    /// Re-sends the RUNNING frame; called from the 1 Hz heartbeat so a lost
    /// frame does not strand the GCS.
    void resendIfRunning();

    bool        busy() const;
    /// "Busy: <title> in progress", or "" when idle.
    std::string busyMessage() const;

private:
    void        _send();
    void        _updateLocked(uint32_t step, uint32_t stepCount, const std::string& message);
    std::string _busyMessageLocked() const;

    mutable std::mutex                  _mutex;
    SendFn                              _sendFn;
    LogFn                               _logFn;
    bool                                _running = false;
    std::string                         _title;
    TunnelProtocol::OperationProgress_t _current {};
};
