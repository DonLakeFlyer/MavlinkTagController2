#include "MavlinkOutgoingMessageQueue.h"
#include "log.h"
#include "MavlinkSystem.h"

MavlinkOutgoingMessageQueue::MavlinkOutgoingMessageQueue(MavlinkSystem* mavlink)
    : _mavlink  (mavlink)
    , _thread   (&MavlinkOutgoingMessageQueue::_run, this)
{
    _thread.detach();
}

void MavlinkOutgoingMessageQueue::addMessage(const mavlink_message_t& message)
{
    {
        std::unique_lock<decltype(_threadWaitMutex)> uLock(_threadWaitMutex);

        _messages.push_back(message);
    }
    _threadWaitCondition.notify_all();
}

void MavlinkOutgoingMessageQueue::_run(void)
{
    while (true) {
        // Wait until we have messages to send
        std::unique_lock<decltype(_threadWaitMutex)> l(_threadWaitMutex);
        _threadWaitCondition.wait(l, [this]{ return _messages.size(); });

        if (_messages.size() > 0) {
            mavlink_message_t message = _messages[0];
            _messages.erase(_messages.begin());
            _mavlink->_sendMessageOnConnection(message);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
