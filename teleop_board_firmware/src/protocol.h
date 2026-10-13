#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config.h"

#define PROTO_MAGIC_0       0xA5u
#define PROTO_MAGIC_1       0x5Au
#define PROTO_VERSION       3u
#define PROTO_TYPE_COMMAND  1u
#define PROTO_TYPE_TELEMETRY 2u
#define PROTO_TYPE_FAILSAFE  3u  // host asks for failsafe braking (payload: reason)
#define PROTO_RX_CAPACITY   (PROTO_MAX_FRAME * 2u)

typedef struct {
    uint8_t type;
    uint8_t length;
    uint16_t sequence;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_packet_t;

typedef struct {
    uint8_t data[PROTO_RX_CAPACITY];
    size_t length;
    uint32_t good_frames;
    uint32_t bad_frames;
} proto_rx_t;

void proto_rx_init(proto_rx_t *rx);
bool proto_rx_push(proto_rx_t *rx, uint8_t byte, proto_packet_t *packet);
size_t proto_encode(uint8_t type, uint16_t sequence,
                    const uint8_t *payload, uint8_t payload_length,
                    uint8_t *output, size_t output_capacity);

#endif