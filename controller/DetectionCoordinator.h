#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>

// Tag-list / detection lifecycle state machine. Commands arrive on the
// MAVLink thread; the start and stop pipelines finish on worker threads, so
// every transition is serialised here. No logging; the caller owns
// diagnostics, process management and heartbeat publication (via the
// callback, invoked outside the lock after every state change).
class DetectionCoordinator {
public:
    enum class State {
        Idle,       // no tag list
        HasTags,    // list uploaded, nothing running
        Starting,   // start pipeline launching processes
        Detecting,  // pipeline up
        Stopping,   // stop pipeline tearing down
    };

    enum class Result {
        Accepted,
        NoTags,             // requestStart while Idle
        AlreadyStarting,    // requestStart while Starting: intent satisfied, ACK success
        AlreadyDetecting,   // requestStart while Detecting
        NotDetecting,       // requestStop while Idle/HasTags
        StartInProgress,    // requestStop while Starting: caller must wait
        AlreadyStopping,    // requestStop while Stopping: intent satisfied, ACK success
        Busy,               // tagsUploaded while Starting/Detecting/Stopping
    };

    using HeartbeatSink = std::function<void(uint16_t)>;

    explicit DetectionCoordinator(HeartbeatSink onHeartbeatStatus = {});

    Result tagsUploaded(bool hasTags);
    Result requestStart();
    void   startFinished(bool ok);
    Result requestStop();
    void   stopFinished();

    /// Blocks while the state equals `state`, up to `timeout`. Returns true if it left.
    bool waitWhile(State state, std::chrono::milliseconds timeout);

    State    state() const;
    bool     idle() const;             // Idle or HasTags: tag uploads allowed
    uint16_t heartbeatStatus() const;  // HEARTBEAT_STATUS_* for the current state

    static uint16_t heartbeatStatusFor(State state);

private:
    void _transition(State next, std::unique_lock<std::mutex>& lock);

    mutable std::mutex      _mutex;
    std::condition_variable _changed;
    State                   _state { State::Idle };
    HeartbeatSink           _onHeartbeatStatus;
};
