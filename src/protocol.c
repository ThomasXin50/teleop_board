#include "protocol.h"

#include <string.h>

#include "crc32.h"

#define PROTO_HEADER_SIZE 7u
#define PROTO_CRC_SIZE    4u

static void discard_prefix(proto_rx_t *rx, size_t count) {
    if (count >= rx->length) {
        rx->length = 0;
        return;
    }
    memmove(rx->data, rx->data + count, rx->length - count);
    rx->length -= count;
}

void proto_rx_init(proto_rx_t *rx) {
    memset(rx, 0, sizeof(*rx));
}

bool proto_rx_push(proto_rx_t *rx, uint8_t byte, proto_packet_t *packet) {
    if (rx->length == sizeof(rx->data)) {
        discard_prefix(rx, 1u);
        rx->bad_frames++;
    }
    rx->data[rx->length++] = byte;

    while (rx->length >= 2u) {
        if (rx->data[0] != PROTO_MAGIC_0 || rx->data[1] != PROTO_MAGIC_1) {
            discard_prefix(rx, 1u);
            continue;
        }
        if (rx->length < PROTO_HEADER_SIZE) return false;

        const uint8_t payload_length = rx->data[4];
        if (rx->data[2] != PROTO_VERSION || payload_length > PROTO_MAX_PAYLOAD) {
            rx->bad_frames++;
            discard_prefix(rx, 1u);
            continue;
        }

        const size_t frame_length = PROTO_HEADER_SIZE + payload_length + PROTO_CRC_SIZE;
        if (rx->length < frame_length) return false;

        const size_t crc_offset = PROTO_HEADER_SIZE + payload_length;
        const uint32_t received_crc = (uint32_t)rx->data[crc_offset]
                                    | ((uint32_t)rx->data[crc_offset + 1u] << 8)
                                    | ((uint32_t)rx->data[crc_offset + 2u] << 16)
                                    | ((uint32_t)rx->data[crc_offset + 3u] << 24);
        const uint32_t expected_crc = crc32_ieee(&rx->data[2], 5u + payload_length);
        if (received_crc != expected_crc) {
            rx->bad_frames++;
            discard_prefix(rx, 1u);
            continue;
        }

        packet->type = rx->data[3];
        packet->length = payload_length;
        packet->sequence = (uint16_t)rx->data[5]
                         | ((uint16_t)rx->data[6] << 8);
        memcpy(packet->payload, &rx->data[PROTO_HEADER_SIZE], payload_length);
        rx->good_frames++;
        discard_prefix(rx, frame_length);
        return true;
    }
    return false;
}

size_t proto_encode(uint8_t type, uint16_t sequence,
                    const uint8_t *payload, uint8_t payload_length,
                    uint8_t *output, size_t output_capacity) {
    const size_t frame_length = PROTO_HEADER_SIZE + payload_length + PROTO_CRC_SIZE;
    if (payload_length > PROTO_MAX_PAYLOAD || output_capacity < frame_length) return 0;
    if (payload_length != 0u && payload == NULL) return 0;

    output[0] = PROTO_MAGIC_0;
    output[1] = PROTO_MAGIC_1;
    output[2] = PROTO_VERSION;
    output[3] = type;
    output[4] = payload_length;
    output[5] = (uint8_t)(sequence & 0xFFu);
    output[6] = (uint8_t)(sequence >> 8);
    if (payload_length != 0u) memcpy(&output[PROTO_HEADER_SIZE], payload, payload_length);

    const uint32_t crc = crc32_ieee(&output[2], 5u + payload_length);
    const size_t crc_offset = PROTO_HEADER_SIZE + payload_length;
    output[crc_offset] = (uint8_t)(crc & 0xFFu);
    output[crc_offset + 1u] = (uint8_t)((crc >> 8) & 0xFFu);
    output[crc_offset + 2u] = (uint8_t)((crc >> 16) & 0xFFu);
    output[crc_offset + 3u] = (uint8_t)(crc >> 24);
    return frame_length;
}