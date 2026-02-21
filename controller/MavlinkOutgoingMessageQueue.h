#pragma once

#include <mavlink.h>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

class MavlinkSystem;

class MavlinkOutgoingMessageQueue
{
public:
    MavlinkOutgoingMessageQueue(MavlinkSystem* mavlink);

    MavlinkSystem*  mavlinkSystem   () const { return _mavlink; }
    void            addMessage      (const mavlink_message_t& message);

private:
	void _run(void);

private:
    MavlinkSystem*                  _mavlink;
    std::vector<mavlink_message_t>  _messages;
    std::thread				        _thread;
    std::mutex                      _threadWaitMutex;
    std::condition_variable         _threadWaitCondition;
};