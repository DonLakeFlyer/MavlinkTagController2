#include "OperationProgress.h"
#include "formatString.h"

#include <algorithm>
#include <cstring>
#include <string_view>

using namespace TunnelProtocol;

namespace {

void setMessage(OperationProgress_t& frame, const std::string& message)
{
    memset(frame.message, 0, sizeof(frame.message));
    strncpy(frame.message, message.c_str(), sizeof(frame.message) - 1);
}

} // namespace

OperationProgressReporter::OperationProgressReporter(SendFn send, LogFn log)
    : _sendFn(std::move(send))
    , _logFn(std::move(log))
{
}

bool OperationProgressReporter::begin(uint32_t command, uint32_t requestId, const std::string& title, uint32_t stepCount)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_running) {
        if (_logFn) {
            _logFn(formatString("operation_progress begin command=%u refused: %s", command, _busyMessageLocked().c_str()));
        }
        return false;
    }
    _running = true;
    _title   = title;
    memset(&_current, 0, sizeof(_current));
    _current.header.command = COMMAND_ID_OPERATION_PROGRESS;
    _current.command        = command;
    _current.request_id     = requestId;
    _current.state          = OPERATION_STATE_RUNNING;
    _current.step           = 0;
    _current.step_count     = stepCount;
    setMessage(_current, title);
    _send();
    return true;
}

void OperationProgressReporter::update(uint32_t step, const std::string& message)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_running) {
        return;
    }
    _updateLocked(step, _current.step_count, message);
}

void OperationProgressReporter::update(uint32_t step, uint32_t stepCount, const std::string& message)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_running) {
        return;
    }
    _updateLocked(step, stepCount, message);
}

void OperationProgressReporter::_updateLocked(uint32_t step, uint32_t stepCount, const std::string& message)
{
    // Compare what would be stored, so a repeated over-length message is not a change.
    const std::string_view stored(message.data(), std::min(message.size(), sizeof(_current.message) - 1));
    const bool messageChanged = !message.empty() && stored != _current.message;
    if (step == _current.step && stepCount == _current.step_count && !messageChanged) {
        return;
    }
    _current.step_count = stepCount;
    _current.step       = stepCount ? std::min(step, stepCount) : step;
    if (messageChanged) {
        setMessage(_current, message);
    }
    _send();
}

void OperationProgressReporter::finish(bool success, const std::string& message)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_running) {
        return;
    }
    _current.state = success ? OPERATION_STATE_COMPLETE : OPERATION_STATE_FAILED;
    if (success && _current.step_count) {
        _current.step = _current.step_count;
    }
    if (!message.empty()) {
        setMessage(_current, message);
    }
    _send();
    _running = false;
    _title.clear();
}

void OperationProgressReporter::resendIfRunning()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_running && _sendFn) {
        _sendFn(_current);
    }
}

bool OperationProgressReporter::busy() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _running;
}

std::string OperationProgressReporter::busyMessage() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _busyMessageLocked();
}

std::string OperationProgressReporter::_busyMessageLocked() const
{
    if (!_running) {
        return "";
    }
    return formatString("Busy: %s in progress", _title.c_str());
}

void OperationProgressReporter::_send()
{
    if (_logFn) {
        _logFn(formatString("operation_progress command=%u request_id=%u state=%u step=%u/%u msg=%s",
                            _current.command, _current.request_id, _current.state,
                            _current.step, _current.step_count, _current.message));
    }
    if (_sendFn) {
        _sendFn(_current);
    }
}
