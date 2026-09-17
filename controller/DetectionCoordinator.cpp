#include "DetectionCoordinator.h"
#include "TunnelProtocol.h"

DetectionCoordinator::DetectionCoordinator(HeartbeatSink onHeartbeatStatus)
    : _onHeartbeatStatus(std::move(onHeartbeatStatus))
{
}

DetectionCoordinator::Result DetectionCoordinator::tagsUploaded(bool hasTags)
{
    std::unique_lock<std::mutex> lock(_mutex);
    if (_state != State::Idle && _state != State::HasTags) {
        return Result::Busy;
    }
    _transition(hasTags ? State::HasTags : State::Idle, lock);
    return Result::Accepted;
}

DetectionCoordinator::Result DetectionCoordinator::requestStart()
{
    std::unique_lock<std::mutex> lock(_mutex);
    switch (_state) {
    case State::Idle:       return Result::NoTags;
    case State::Starting:   return Result::AlreadyStarting;
    case State::Detecting:  return Result::AlreadyDetecting;
    case State::Stopping:   return Result::AlreadyDetecting;
    case State::HasTags:    break;
    }
    _transition(State::Starting, lock);
    return Result::Accepted;
}

void DetectionCoordinator::startFinished(bool ok)
{
    std::unique_lock<std::mutex> lock(_mutex);
    if (_state != State::Starting) {
        return;
    }
    _transition(ok ? State::Detecting : State::HasTags, lock);
}

DetectionCoordinator::Result DetectionCoordinator::requestStop()
{
    std::unique_lock<std::mutex> lock(_mutex);
    switch (_state) {
    case State::Idle:
    case State::HasTags:    return Result::NotDetecting;
    case State::Starting:   return Result::StartInProgress;
    case State::Stopping:   return Result::AlreadyStopping;
    case State::Detecting:  break;
    }
    _transition(State::Stopping, lock);
    return Result::Accepted;
}

void DetectionCoordinator::stopFinished()
{
    std::unique_lock<std::mutex> lock(_mutex);
    if (_state != State::Stopping) {
        return;
    }
    _transition(State::HasTags, lock);
}

bool DetectionCoordinator::waitWhile(State state, std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(_mutex);
    return _changed.wait_for(lock, timeout, [this, state] { return _state != state; });
}

DetectionCoordinator::State DetectionCoordinator::state() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _state;
}

bool DetectionCoordinator::idle() const
{
    const State s = state();
    return s == State::Idle || s == State::HasTags;
}

uint16_t DetectionCoordinator::heartbeatStatus() const
{
    return heartbeatStatusFor(state());
}

uint16_t DetectionCoordinator::heartbeatStatusFor(State state)
{
    switch (state) {
    case State::Idle:       return HEARTBEAT_STATUS_IDLE;
    case State::HasTags:    return HEARTBEAT_STATUS_HAS_TAGS;
    // Heartbeat flips to DETECTING only once the pipeline is up, and stays
    // there until teardown completes, so the GCS never sees a half state.
    case State::Starting:   return HEARTBEAT_STATUS_HAS_TAGS;
    case State::Detecting:  return HEARTBEAT_STATUS_DETECTING;
    case State::Stopping:   return HEARTBEAT_STATUS_DETECTING;
    }
    return HEARTBEAT_STATUS_IDLE;
}

void DetectionCoordinator::_transition(State next, std::unique_lock<std::mutex>& lock)
{
    const bool heartbeatChanged = heartbeatStatusFor(next) != heartbeatStatusFor(_state);
    _state = next;
    _changed.notify_all();
    if (heartbeatChanged && _onHeartbeatStatus) {
        const uint16_t status = heartbeatStatusFor(next);
        lock.unlock();
        _onHeartbeatStatus(status);
    }
}
