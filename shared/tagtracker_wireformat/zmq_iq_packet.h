#ifndef TAGTRACKER_WIREFORMAT_ZMQ_IQ_PACKET_H
#define TAGTRACKER_WIREFORMAT_ZMQ_IQ_PACKET_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TTWF_ZMQ_IQ_MAGIC 0x5a514941u
#define TTWF_ZMQ_IQ_VERSION 1u
#define TTWF_ZMQ_IQ_HEADER_SIZE 40u
#define TTWF_ZMQ_IQ_BYTES_PER_COMPLEX_SAMPLE 8u
#define TTWF_ZMQ_IQ_FLAG_FINAL_CHUNK 0x1u

#define TTWF_ZMQ_OK 0
#define TTWF_ZMQ_ERR_ARG -1
#define TTWF_ZMQ_ERR_SHORT_FRAME -2
#define TTWF_ZMQ_ERR_BAD_MAGIC -3
#define TTWF_ZMQ_ERR_BAD_VERSION -4
#define TTWF_ZMQ_ERR_BAD_HEADER_SIZE -5
#define TTWF_ZMQ_ERR_BAD_PAYLOAD_SIZE -6
#define TTWF_ZMQ_ERR_BAD_SAMPLE_RATE -7
#define TTWF_ZMQ_ERR_BAD_SAMPLE_COUNT -8

#if defined(_MSC_VER)
#pragma pack(push, 1)
#define TTWF_PACKED
#else
#define TTWF_PACKED __attribute__((packed))
#endif

typedef struct TTWF_PACKED ttwf_zmq_iq_packet_header {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint64_t sequence;
    uint64_t timestamp_us;
    uint32_t sample_rate;
    uint32_t sample_count;
    uint32_t payload_bytes;
    uint32_t flags;
} ttwf_zmq_iq_packet_header_t;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(ttwf_zmq_iq_packet_header_t) == TTWF_ZMQ_IQ_HEADER_SIZE,
               "TTWF ZeroMQ IQ header must be exactly 40 bytes");
#endif

static inline uint16_t ttwf_read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t ttwf_read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline uint64_t ttwf_read_u64_le(const uint8_t *p) {
    return (uint64_t)p[0] |
           ((uint64_t)p[1] << 8) |
           ((uint64_t)p[2] << 16) |
           ((uint64_t)p[3] << 24) |
           ((uint64_t)p[4] << 32) |
           ((uint64_t)p[5] << 40) |
           ((uint64_t)p[6] << 48) |
           ((uint64_t)p[7] << 56);
}

static inline void ttwf_write_u16_le(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static inline void ttwf_write_u32_le(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
    p[2] = (uint8_t)((value >> 16) & 0xFFu);
    p[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static inline void ttwf_write_u64_le(uint8_t *p, uint64_t value) {
    p[0] = (uint8_t)(value & 0xFFull);
    p[1] = (uint8_t)((value >> 8) & 0xFFull);
    p[2] = (uint8_t)((value >> 16) & 0xFFull);
    p[3] = (uint8_t)((value >> 24) & 0xFFull);
    p[4] = (uint8_t)((value >> 32) & 0xFFull);
    p[5] = (uint8_t)((value >> 40) & 0xFFull);
    p[6] = (uint8_t)((value >> 48) & 0xFFull);
    p[7] = (uint8_t)((value >> 56) & 0xFFull);
}

static inline int ttwf_decode_zmq_iq_header(const uint8_t *data,
                                            size_t data_size,
                                            ttwf_zmq_iq_packet_header_t *out) {
    if (data == NULL || out == NULL) {
        return TTWF_ZMQ_ERR_ARG;
    }
    if (data_size < TTWF_ZMQ_IQ_HEADER_SIZE) {
        return TTWF_ZMQ_ERR_SHORT_FRAME;
    }

    out->magic = ttwf_read_u32_le(data + 0u);
    out->version = ttwf_read_u16_le(data + 4u);
    out->header_size = ttwf_read_u16_le(data + 6u);
    out->sequence = ttwf_read_u64_le(data + 8u);
    out->timestamp_us = ttwf_read_u64_le(data + 16u);
    out->sample_rate = ttwf_read_u32_le(data + 24u);
    out->sample_count = ttwf_read_u32_le(data + 28u);
    out->payload_bytes = ttwf_read_u32_le(data + 32u);
    out->flags = ttwf_read_u32_le(data + 36u);
    return TTWF_ZMQ_OK;
}

static inline int ttwf_encode_zmq_iq_header(uint8_t *out,
                                            size_t out_size,
                                            const ttwf_zmq_iq_packet_header_t *header) {
    if (out == NULL || header == NULL) {
        return TTWF_ZMQ_ERR_ARG;
    }
    if (out_size < TTWF_ZMQ_IQ_HEADER_SIZE) {
        return TTWF_ZMQ_ERR_SHORT_FRAME;
    }

    ttwf_write_u32_le(out + 0u, header->magic);
    ttwf_write_u16_le(out + 4u, header->version);
    ttwf_write_u16_le(out + 6u, header->header_size);
    ttwf_write_u64_le(out + 8u, header->sequence);
    ttwf_write_u64_le(out + 16u, header->timestamp_us);
    ttwf_write_u32_le(out + 24u, header->sample_rate);
    ttwf_write_u32_le(out + 28u, header->sample_count);
    ttwf_write_u32_le(out + 32u, header->payload_bytes);
    ttwf_write_u32_le(out + 36u, header->flags);

    return TTWF_ZMQ_OK;
}

static inline int ttwf_validate_zmq_iq_frame(const uint8_t *frame,
                                             size_t frame_size,
                                             ttwf_zmq_iq_packet_header_t *out_header) {
    ttwf_zmq_iq_packet_header_t local_header;
    ttwf_zmq_iq_packet_header_t *header_ptr =
        (out_header != NULL) ? out_header : &local_header;

    int rc = ttwf_decode_zmq_iq_header(frame, frame_size, header_ptr);
    if (rc != TTWF_ZMQ_OK) {
        return rc;
    }

    if (header_ptr->magic != TTWF_ZMQ_IQ_MAGIC) {
        return TTWF_ZMQ_ERR_BAD_MAGIC;
    }
    if (header_ptr->version != TTWF_ZMQ_IQ_VERSION) {
        return TTWF_ZMQ_ERR_BAD_VERSION;
    }
    if (header_ptr->header_size < TTWF_ZMQ_IQ_HEADER_SIZE ||
        (size_t)header_ptr->header_size > frame_size) {
        return TTWF_ZMQ_ERR_BAD_HEADER_SIZE;
    }

    const size_t payload_size = frame_size - (size_t)header_ptr->header_size;
    if ((size_t)header_ptr->payload_bytes != payload_size) {
        return TTWF_ZMQ_ERR_BAD_PAYLOAD_SIZE;
    }
    if (header_ptr->sample_rate == 0u) {
        return TTWF_ZMQ_ERR_BAD_SAMPLE_RATE;
    }
    if (header_ptr->sample_count == 0u) {
        return TTWF_ZMQ_ERR_BAD_SAMPLE_COUNT;
    }

    const uint64_t expected_payload =
        (uint64_t)header_ptr->sample_count *
        (uint64_t)TTWF_ZMQ_IQ_BYTES_PER_COMPLEX_SAMPLE;
    if (expected_payload != (uint64_t)header_ptr->payload_bytes) {
        return TTWF_ZMQ_ERR_BAD_PAYLOAD_SIZE;
    }

    return TTWF_ZMQ_OK;
}

#ifdef __cplusplus
}
#endif

#endif
