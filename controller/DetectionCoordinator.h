#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>

// Tag-list / detection lifecycle state machine. Commands arrive on the
// MAVLink thread; the start, stop and capture pipelines finish on worker
// threads, so every transition is serialised here. Raw capture is a state
// here too because it holds the SDR, which is what START_DETECTION contends
// for. No logging; the caller owns diagnostics, process management and
// heartbeat publication (via the callback, invoked outside the lock after
// every state change).
class DetectionCoordinator {
public:
    enum class State {
        Idle,       // no tag list
        HasTags,    // list uploaded, nothing running
        Starting,   // start pipeline launching processes
        Detecting,  // pipeline up
        Stopping,   // stop pipeline tearing down
        Capturing,  // raw capture process holds the SDR
    };

    enum class Result {
        Accepted,
        NoTags,             // requestStart / requestCapture while Idle
        AlreadyStarting,    // requestStart while Starting: intent satisfied, ACK success
        AlreadyDetecting,   // requestStart while Detecting
        NotDetecting,       // requestStop while Idle/HasTags/Capturing
        StartInProgress,    // requestStop while Starting: caller must wait
        AlreadyStopping,    // requestStop while Stopping: intent satisfied, ACK success
        Busy,               // tagsUploaded / requestCapture while Starting/Detecting/Stopping
        Capturing,          // requestStart / tagsUploaded while Capturing
        AlreadyCapturing,   // requestCapture while Capturing
    };

    using HeartbeatSink = std::function<void(uint16_t)>;

    explicit DetectionCoordinator(HeartbeatSink onHeartbeatStatus = {});

    Result tagsUploaded(bool hasTags);
    Result requestStart();
    void   startFinished(bool ok);
    Result requestStop();
    void   stopFinished();
    Result requestCapture();
    void   captureFinished();

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
