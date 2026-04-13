#pragma once

#include <thread>
#include <string>
#include <memory>
#include <thread>
#include <atomic>

class MavlinkSystem;
class TelemetryCache;
class CommandHandler;

class UDPPulseReceiver
{
public:
	UDPPulseReceiver(std::string localIp, int localPort, CommandHandler* commandHandler);
	~UDPPulseReceiver();

	void start	(void);
	void run 	(void);
	void stop 	(void);

private:
	bool _setupPort (void);
	void _receive 	(void);

	std::thread*	_thread 	{ nullptr };
    std::string 	_localIp;
    int 			_localPort;
    int 			_fdSocket	{-1};
	CommandHandler*	_commandHandler;
};
