#pragma once

#include <thread>
#include <string>
#include <memory>
#include <thread>
#include <atomic>

class MavlinkSystem;
class TelemetryCache;

// This class is to handle the reception of a pulse in a generic way. This way both
// pulse received over udp and simulated can call this to process the pulse.
class PulseHandler
{
public:
	PulseHandler(MavlinkSystem* mavlink, TelemetryCache* telemetryCache);
	~PulseHandler();

	// Enough for MTU 1500 bytes.
	typedef struct {
		double tag_id;
		double frequency_hz;
		double start_time_seconds;
		double predict_next_start_seconds;
		double snr;
		double stft_score;
		double group_seq_counter;
		double group_ind;
		double group_snr;
		double detection_status;
		double confirmed_status;
		double noise_psd;
	} UDPPulseInfo_T;

	void handlePulse(const UDPPulseInfo_T& udpPulseInfo);

private:
    MavlinkSystem*	_mavlink 		= nullptr;
	TelemetryCache*	_telemetryCache	= nullptr;
};