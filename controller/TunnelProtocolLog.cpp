#include "TunnelProtocolLog.h"
#include "TunnelProtocol.h"
#include "formatString.h"

#include <cstring>

using namespace TunnelProtocol;

namespace TunnelProtocolLog {

std::string commandName(uint32_t command)
{
    switch (command) {
    case COMMAND_ID_ACK:                    return "ACK";
    case COMMAND_ID_START_TAGS:             return "START_TAGS";
    case COMMAND_ID_END_TAGS:               return "END_TAGS";
    case COMMAND_ID_TAG:                    return "TAG";
    case COMMAND_ID_START_DETECTION:        return "START_DETECTION";
    case COMMAND_ID_STOP_DETECTION:         return "STOP_DETECTION";
    case COMMAND_ID_PULSE:                  return "PULSE";
    case COMMAND_ID_RAW_CAPTURE:            return "RAW_CAPTURE";
    case COMMAND_ID_HEARTBEAT:              return "HEARTBEAT";
    case COMMAND_ID_START_ROTATION:         return "START_ROTATION";
    case COMMAND_ID_STOP_ROTATION:          return "STOP_ROTATION";
    case COMMAND_ID_SAVE_LOGS:              return "SAVE_LOGS";
    case COMMAND_ID_CLEAN_LOGS:             return "CLEAN_LOGS";
    case COMMAND_ID_AIRSPY_STATUS:          return "AIRSPY_STATUS";
    case COMMAND_ID_START_COLLECTION:       return "START_COLLECTION";
    case COMMAND_ID_START_COLLECTION_SLICE: return "START_COLLECTION_SLICE";
    case COMMAND_ID_FINISH_COLLECTION:      return "FINISH_COLLECTION";
    case COMMAND_ID_BEARING_RESULT:         return "BEARING_RESULT";
    case COMMAND_ID_COLLECTION_STATUS:      return "COLLECTION_STATUS";
    case COMMAND_ID_PYTHON_PULSE:           return "PYTHON_PULSE";
    case COMMAND_ID_OPERATION_PROGRESS:     return "OPERATION_PROGRESS";
    case COMMAND_ID_SET_LOG_LEVEL:          return "SET_LOG_LEVEL";
    case COMMAND_ID_DETECTOR_HEARTBEAT:     return "DETECTOR_HEARTBEAT";
    }
    return formatString("UNKNOWN(%u)", command);
}

std::string commandResultName(uint32_t result)
{
    switch (result) {
    case COMMAND_RESULT_SUCCESS: return "SUCCESS";
    case COMMAND_RESULT_FAILURE: return "FAILURE";
    }
    return formatString("UNKNOWN(%u)", result);
}

std::string collectionStatusName(uint32_t status)
{
    switch (status) {
    case COLLECTION_STATUS_SLICE_ARMED:       return "SLICE_ARMED";
    case COLLECTION_STATUS_SLICE_COMPLETE:    return "SLICE_COMPLETE";
    case COLLECTION_STATUS_FAILED:            return "FAILED";
    case COLLECTION_STATUS_STOPPED:           return "STOPPED";
    case COLLECTION_STATUS_REVISIT_REQUESTED: return "REVISIT_REQUESTED";
    }
    return formatString("UNKNOWN(%u)", status);
}

std::string collectionFinishName(uint32_t disposition)
{
    switch (disposition) {
    case COLLECTION_FINISH_FINALIZE: return "FINALIZE";
    case COLLECTION_FINISH_CANCEL:   return "CANCEL";
    }
    return formatString("UNKNOWN(%u)", disposition);
}

std::string operationStateName(uint32_t state)
{
    switch (state) {
    case OPERATION_STATE_RUNNING:  return "RUNNING";
    case OPERATION_STATE_COMPLETE: return "COMPLETE";
    case OPERATION_STATE_FAILED:   return "FAILED";
    }
    return formatString("UNKNOWN(%u)", state);
}

std::string heartbeatStatusName(uint32_t status)
{
    switch (status) {
    case HEARTBEAT_STATUS_IDLE:           return "IDLE";
    case HEARTBEAT_STATUS_RECEIVING_TAGS: return "RECEIVING_TAGS";
    case HEARTBEAT_STATUS_HAS_TAGS:       return "HAS_TAGS";
    case HEARTBEAT_STATUS_DETECTING:      return "DETECTING";
    case HEARTBEAT_STATUS_CAPTURE:        return "CAPTURE";
    }
    return formatString("UNKNOWN(%u)", status);
}

std::string detectionModeName(uint32_t mode)
{
    switch (mode) {
    case DETECTION_MODE_UAVRT:  return "UAVRT";
    case DETECTION_MODE_PYTHON: return "PYTHON";
    }
    return formatString("UNKNOWN(%u)", mode);
}

std::string logLevelName(uint32_t level)
{
    switch (level) {
    case LOG_LEVEL_DEBUG:   return "DEBUG";
    case LOG_LEVEL_VERBOSE: return "VERBOSE";
    }
    return formatString("UNKNOWN(%u)", level);
}

namespace {

template <typename T>
bool copyPayload(const void* payload, size_t length, T* out)
{
    if (length != sizeof(T)) {
        return false;
    }
    memcpy(out, payload, sizeof(T));
    return true;
}

std::string fields(const void* payload, size_t length, uint32_t command)
{
    switch (command) {
    case COMMAND_ID_ACK: {
        AckInfo_t m;
        if (!copyPayload(payload, length, &m)) break;
        m.message[sizeof(m.message) - 1] = '\0';
        return formatString(" command:%s request_id:%u result:%s%s%s",
                            commandName(m.command).c_str(), m.request_id, commandResultName(m.result).c_str(),
                            m.message[0] ? " message:" : "", m.message);
    }
    case COMMAND_ID_OPERATION_PROGRESS: {
        OperationProgress_t m;
        if (!copyPayload(payload, length, &m)) break;
        m.message[sizeof(m.message) - 1] = '\0';
        return formatString(" command:%s request_id:%u state:%s step:%u/%u%s%s",
                            commandName(m.command).c_str(), m.request_id, operationStateName(m.state).c_str(),
                            m.step, m.step_count, m.message[0] ? " message:" : "", m.message);
    }
    case COMMAND_ID_START_TAGS: {
        StartTagsInfo_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" upload_id:%u tag_count:%u", m.upload_id, m.tag_count);
    }
    case COMMAND_ID_END_TAGS: {
        EndTagsInfo_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" upload_id:%u tag_count:%u", m.upload_id, m.tag_count);
    }
    case COMMAND_ID_TAG: {
        TagInfo_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" upload_id:%u tag_index:%u id:%u frequency_hz:%u pulse_width_msecs:%u"
                            " intra_pulse1_msecs:%u intra_pulse2_msecs:%u intra_pulse_uncertainty_msecs:%u"
                            " intra_pulse_jitter_msecs:%u k:%u false_alarm_probability:%g"
                            " channelizer_channel_number:%u channelizer_channel_center_frequency_hz:%u",
                            m.upload_id, m.tag_index, m.id, m.frequency_hz, m.pulse_width_msecs,
                            m.intra_pulse1_msecs, m.intra_pulse2_msecs, m.intra_pulse_uncertainty_msecs,
                            m.intra_pulse_jitter_msecs, m.k, m.false_alarm_probability,
                            m.channelizer_channel_number, m.channelizer_channel_center_frequency_hz);
    }
    case COMMAND_ID_START_DETECTION: {
        StartDetectionInfo_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" radio_center_frequency_hz:%u gain:%u detection_mode:%s detection_margin:%g"
                            " confidence_ratio:%g debug_detector:%u dump_spectrogram:%u",
                            m.radio_center_frequency_hz, m.gain, detectionModeName(m.detection_mode).c_str(),
                            m.detection_margin, m.confidence_ratio, m.debug_detector, m.dump_spectrogram);
    }
    case COMMAND_ID_RAW_CAPTURE: {
        RawCaptureInfo_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" gain:%u frequency_hz:%u", m.gain, m.frequency_hz);
    }
    case COMMAND_ID_HEARTBEAT: {
        Heartbeat_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" protocol_version:%u system_id:%u status:%s cpu_temp_c:%.1f",
                            m.protocol_version, m.system_id, heartbeatStatusName(m.status).c_str(), m.cpu_temp_c);
    }
    case COMMAND_ID_START_COLLECTION: {
        StartCollection_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" collection_id:%u radio_center_frequency_hz:%u n_slices:%u detection_margin:%g"
                            " confidence_ratio:%g debug_detector:%u dump_spectrogram:%u antenna_id:%u",
                            m.collection_id, m.radio_center_frequency_hz, m.n_slices, m.detection_margin,
                            m.confidence_ratio, m.debug_detector, m.dump_spectrogram, m.antenna_id);
    }
    case COMMAND_ID_START_COLLECTION_SLICE: {
        StartCollectionSlice_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" collection_id:%u slice_id:%u heading_deg:%.1f",
                            m.collection_id, m.slice_id, m.heading_deg);
    }
    case COMMAND_ID_FINISH_COLLECTION: {
        FinishCollection_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" collection_id:%u disposition:%s",
                            m.collection_id, collectionFinishName(m.disposition).c_str());
    }
    case COMMAND_ID_SET_LOG_LEVEL: {
        SetLogLevel_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" level:%s", logLevelName(m.level).c_str());
    }
    case COMMAND_ID_COLLECTION_STATUS: {
        CollectionStatus_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" collection_id:%u slice_id:%u status:%s expected_detectors:%u completed_detectors:%u"
                            " error_code:%u revisit_heading_deg:%.1f",
                            m.collection_id, m.slice_id, collectionStatusName(m.status).c_str(),
                            m.expected_detectors, m.completed_detectors, m.error_code, m.revisit_heading_deg);
    }
    case COMMAND_ID_BEARING_RESULT: {
        BearingResult_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" collection_id:%u tag_id:%u bearing_deg:%.1f r_squared:%.3f n_valid_slices:%u"
                            " best_snr:%.1f confirmed:%u",
                            m.collection_id, m.tag_id, m.bearing_deg, m.r_squared, m.n_valid_slices,
                            m.best_snr, m.confirmed);
    }
    case COMMAND_ID_PULSE: {
        PulseInfo_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" tag_id:%u frequency_hz:%u detection_status:%u confirmed_status:%u snr:%.1f"
                            " group_seq_counter:%u group_ind:%u noise_psd:%g",
                            m.tag_id, m.frequency_hz, m.detection_status, m.confirmed_status, m.snr,
                            m.group_seq_counter, m.group_ind, m.noise_psd);
    }
    case COMMAND_ID_PYTHON_PULSE: {
        PythonPulseInfo_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" collection_id:%u slice_id:%u tag_id:%u frequency_hz:%u cycle_counter:%u"
                            " detection_status:%u confirmed_status:%u rate_state:%u candidate_id:%u"
                            " snr:%.1f score_ratio:%.3f signal_psd:%g noise_psd:%g",
                            m.collection_id, m.slice_id, m.tag_id, m.frequency_hz, m.cycle_counter,
                            m.detection_status, m.confirmed_status, m.rate_state, m.candidate_id,
                            m.snr, m.score_ratio, m.signal_psd, m.noise_psd);
    }
    case COMMAND_ID_DETECTOR_HEARTBEAT: {
        DetectorHeartbeat_t m;
        if (!copyPayload(payload, length, &m)) break;
        return formatString(" tag_id:%u detection_mode:%s", m.tag_id, detectionModeName(m.detection_mode).c_str());
    }
    case COMMAND_ID_STOP_DETECTION:
    case COMMAND_ID_SAVE_LOGS:
    case COMMAND_ID_CLEAN_LOGS:
    case COMMAND_ID_AIRSPY_STATUS:
        if (length == sizeof(HeaderInfo_t)) {
            return "";
        }
        break;
    default:
        break;
    }
    return formatString(" payload_length:%zu", length);
}

} // namespace

std::string describe(const char* verb, const void* payload, size_t length)
{
    HeaderInfo_t header {};
    if (length < sizeof(header)) {
        return formatString("UNKNOWN %s: payload_length:%zu (too small for header)", verb, length);
    }
    memcpy(&header, payload, sizeof(header));
    return formatString("%s %s: request_id:%u%s", commandName(header.command).c_str(), verb, header.request_id,
                        fields(payload, length, header.command).c_str());
}

} // namespace TunnelProtocolLog
