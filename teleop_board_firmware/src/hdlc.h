#ifndef HDLC_H
#define HDLC_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define HDLC_FLAG        0x7Eu

#define HDLC_FCS_BYTES   4u      // CRC-32
#define HDLC_MAX_PAYLOAD  12u   
#define HDLC_RX_WINDOW    64u

uint32_t hdlc_encode(const uint8_t *payload, uint32_t payload_len,
                     uint8_t *out, uint32_t out_cap);

// BEcause we are using UART instead of SPI, the idea is to accumulate the incoming bytes in
// a window and scan said window for a full frame. 
typedef struct {
    uint8_t  win[HDLC_RX_WINDOW];
    uint32_t len;                 
    uint32_t expect_payload;      \
    uint32_t n_good;
    uint32_t n_bad;
} hdlc_rx_t;
//Append one byte to the window
void hdlc_rx_init(hdlc_rx_t *r, uint32_t expect_payload);

//Logic for actually extracting said frame. Returns len of payload and write it to out
//Also consumes the frame and returns 0 otherwise
void hdlc_rx_byte(hdlc_rx_t *r, uint8_t b);



#endif