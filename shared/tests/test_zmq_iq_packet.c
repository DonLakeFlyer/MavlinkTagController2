#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "tagtracker_wireformat/zmq_iq_packet.h"

static int test_valid_roundtrip(void) {
    ttwf_zmq_iq_packet_header_t in = {
        .magic = TTWF_ZMQ_IQ_MAGIC,
        .version = TTWF_ZMQ_IQ_VERSION,
        .header_size = TTWF_ZMQ_IQ_HEADER_SIZE,
        .sequence = 101,
        .timestamp_us = 123456789ULL,
        .sample_rate = 768000,
        .sample_count = 2,
        .payload_bytes = 16,
        .flags = 0,
    };

    uint8_t frame[TTWF_ZMQ_IQ_HEADER_SIZE + 16];
    memset(frame, 0, sizeof(frame));

    if (ttwf_encode_zmq_iq_header(frame, sizeof(frame), &in) != TTWF_ZMQ_OK) {
        return 1;
    }

    ttwf_zmq_iq_packet_header_t out;
    if (ttwf_validate_zmq_iq_frame(frame, sizeof(frame), &out) != TTWF_ZMQ_OK) {
        return 1;
    }

    if (out.sequence != in.sequence || out.timestamp_us != in.timestamp_us ||
        out.sample_rate != in.sample_rate || out.sample_count != in.sample_count ||
        out.payload_bytes != in.payload_bytes || out.header_size != in.header_size) {
        return 1;
    }

    return 0;
}

static int test_bad_magic(void) {
    ttwf_zmq_iq_packet_header_t header = {
        .magic = 0xdeadbeefu,
        .version = TTWF_ZMQ_IQ_VERSION,
        .header_size = TTWF_ZMQ_IQ_HEADER_SIZE,
        .sequence = 1,
        .timestamp_us = 1,
        .sample_rate = 768000,
        .sample_count = 1,
        .payload_bytes = 8,
        .flags = 0,
    };

    uint8_t frame[TTWF_ZMQ_IQ_HEADER_SIZE + 8];
    if (ttwf_encode_zmq_iq_header(frame, sizeof(frame), &header) != TTWF_ZMQ_OK) {
        return 1;
    }

    return ttwf_validate_zmq_iq_frame(frame, sizeof(frame), NULL) == TTWF_ZMQ_ERR_BAD_MAGIC ? 0 : 1;
}

int main(void) {
    if (test_valid_roundtrip() != 0) {
        fprintf(stderr, "test_valid_roundtrip failed\n");
        return 1;
    }
    if (test_bad_magic() != 0) {
        fprintf(stderr, "test_bad_magic failed\n");
        return 1;
    }

    printf("TagTrackerWireFormat tests passed\n");
    return 0;
}
