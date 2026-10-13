#include "config.h"
#include "hdlc.h"
#include "crc32.h"
#include <string.h>
//STUFF I got from Thomas' framed_spi.c

static inline bool push_bit(uint8_t *out,
                            uint32_t out_cap,
                            uint32_t *bitpos,
                            uint8_t bit) {
    uint32_t byte_i = (*bitpos) >> 3;
    uint32_t bit_i  = (*bitpos) & 7u;

    if (byte_i >= out_cap) return false;
    if (bit) out[byte_i] |= (uint8_t)(1u << (7u - bit_i));
    (*bitpos)++;
    return true;
}

static bool bit_stuff(const uint8_t *in,
                      uint32_t in_len,
                      uint8_t *out,
                      uint32_t out_cap,
                      uint32_t *bitpos) {
    int ones = 0;

    for (size_t i = 0; i < in_len; i++) {
        for (int b = 7; b >= 0; b--) {
            uint8_t bit = (in[i] >> b) & 1u;
            if (!push_bit(out, out_cap, bitpos, bit)) return false;

            if (bit) ones++;
            else ones = 0;

            if (ones == 5) {
                if (!push_bit(out, out_cap, bitpos, 0)) return false;
                ones = 0;
            }
        }
    }
    return true;
}
//Will return 0 if payload is greater than HDL_MAX_PAYLOAD
//Will return 0 if paylaod empty
static uint32_t build_frame(const uint8_t *payload,
                            uint32_t payload_len,
                            uint8_t *out,
                            uint32_t out_cap) {
                                //CHanged from Thomas's code to not be variable length and
                                //depend on global vars
    uint8_t tmp[HDLC_MAX_PAYLOAD+HDLC_FCS_BYTES];

    if(payload> HDLC_MAX_PAYLOAD){
        return 0;
    }
    if(payload == 0u){
        return 0;
    }

    memcpy(tmp, payload, payload_len);
    uint32_t crc = crc32_ieee(payload, payload_len);
    framed_spi_pack_u32_le(&tmp[payload_len], crc);

    memset(out, 0, out_cap);
    out[0] = HDLC_FLAG;

    uint32_t bitpos = 8u;
    if (!bit_stuff(tmp, payload_len + 4u, out, out_cap, &bitpos)) return 0;

    uint32_t bytes = (bitpos + 7u) >> 3;
    if (bytes + 1u > out_cap) return 0;

    out[bytes] = HDLC_FLAG;
    return bytes + 1u;
}

//Unchanged from Thomas
void pack_u32_le(uint8_t *dst, uint32_t x) {
    dst[0] = (uint8_t)(x & 0xFFu);
    dst[1] = (uint8_t)((x >> 8) & 0xFFu);
    dst[2] = (uint8_t)((x >> 16) & 0xFFu);
    dst[3] = (uint8_t)((x >> 24) & 0xFFu);
}


//USed THomas' CPP logic and chnaged it to C because this 2040 needs to both decode and encode HDLC

static inline uint8_t get_bit_msb(const uint8_t *data, uint32_t len, int32_t bit_index) {
    const int32_t byte_i = bit_index >> 3;
    const int32_t bit_i  = bit_index & 7;
    if (byte_i < 0 || (uint32_t)byte_i >= len) return 0;
    return (data[(uint32_t)byte_i] >> (7 - bit_i)) & 1u;
}

static bool find_two_flags(const uint8_t *rx, uint32_t rx_len,
                           int32_t *first_flag_end_bit,
                           int32_t *second_flag_end_bit) {
    *first_flag_end_bit = -1;
    *second_flag_end_bit = -1;

    uint8_t sh = 0;
    const int32_t nbits = (int32_t)rx_len * 8;

    for (int32_t bi = 0; bi < nbits; bi++) {
        sh = (uint8_t)((sh << 1) | get_bit_msb(rx,rx_len, bi));
        if (sh == HDLC_FLAG) {
            if (*first_flag_end_bit < 0) {
                *first_flag_end_bit = bi; // last bit of first flag
                sh = 0;
            } else {
                *second_flag_end_bit = bi; // last bit of second flag
                return true;
            }
        }
    }
    return false;
}


static bool bit_unstuff_range(const uint8_t *in,
                              uint32_t in_len,
                              int32_t start_bit,
                              int32_t end_bit_exclusive,
                              uint8_t* out,
                              uint32_t out_cap_bytes,
                              uint32_t *out_bitpos) {
    int32_t ones = 0;

    for (int bi = start_bit; bi < end_bit_exclusive && *out_bitpos < out_cap_bytes * 8; bi++) {
        const uint8_t bit = get_bit_msb(in,in_len, bi);

        // If we've already seen five 1s, this bit MUST be a stuffed 0 and must be skipped.
        if (ones == 5) {
            if (bit != 0) return false; // invalid stuffing
            ones = 0;
            continue;
        }

        const uint32_t byte_i = (*out_bitpos) >> 3;
        const uint32_t bit_i  = (*out_bitpos) & 7;
        if (byte_i >= out_cap_bytes) return false;

        if (bit) out[byte_i] |= (uint8_t)(1u << (7 - bit_i));
        (*out_bitpos)++;

        if (bit) ones++;
        else     ones = 0;
    }

    return true;
}

static bool decode_frame(const uint8_t *rx,
                        uint32_t rx_len,
                        uint8_t *out, uint32_t out_cap,
                         uint32_t payload_len,
                         int32_t *consumed_to_bit) {
    if (payload_len == 0u) return false;
    //Amount of requires bytes 
    const uint32_t want_bytes = payload_len+HDLC_FCS_BYTES;
    if(out_cap<want_bytes)
                            return false; 
    // Find flags and compute where the data bits are.
    int32_t first_flag_end_bit; 
    int32_t second_flag_end_bit;
    if (!find_two_flags(rx,rx_len, &first_flag_end_bit, &second_flag_end_bit)) return false;
    consumed_to_bit = second_flag_end_bit;
    const int32_t data_start_bit = first_flag_end_bit + 1;
    const int32_t data_end_bit_exclusive = second_flag_end_bit - 7;

    if (data_end_bit_exclusive <= data_start_bit) return false;
    //Needs to be zeroed because of the way that setting specific bits works (it uses or)
    memset(out, 0, want_bytes);


    
    // Unstuff bits in the data range into out[], which should now contain payload || crc_le
    size_t out_bitpos = 0;
    if (!bit_unstuff_range(rx,rx_len, data_start_bit, data_end_bit_exclusive,
                           out, want_bytes, &out_bitpos)) {
        return false;
    }

    if (out_bitpos < want_bytes * 8u) return false;

    // Check CRC PS: this is lowk Claude
    const uint32_t crc_rx = (uint32_t)out[payload_len]
                      | ((uint32_t)out[payload_len + 1] << 8)
                      | ((uint32_t)out[payload_len + 2] << 16)
                      | ((uint32_t)out[payload_len + 3] << 24);

    return crc_rx == crc32_ieee(out, payload_len);
}




//New Code to be able to use functions with UART which is byte by byte
void hdlc_rx_init(hdlc_rx_t *r, uint32_t expect_payload) {
    r->len            = 0;
    r->expect_payload = expect_payload;
    r->n_good         = 0;
    r->n_bad          = 0;
}

void hdlc_rx_byte(hdlc_rx_t *r, uint8_t b) {
    if (r->len < HDLC_RX_WINDOW) {
        r->win[r->len++] = b;
    } else {
        // If full it slides all the bytes and dleetes oldest one
        //IDK if this is too expensive to run but not sure what to do here instead
        memmove(r->win, r->win + 1, HDLC_RX_WINDOW - 1);
        r->win[HDLC_RX_WINDOW - 1] = b;
    }
}

uint32_t hdlc_rx_try_decode(hdlc_rx_t *r, uint8_t *out, uint32_t out_cap) {
    //if -1 then no 2 flags if 0 then two flags but somehow corrupted frame
    int32_t consumed_to_bit = -1;

    if (decode_frame(r->win, r->len, r->expect_payload,
                     out, out_cap, &consumed_to_bit)) {
        r->n_good++;

        //Drops the read bytes from the window. Round up in case not byte boundary 
        uint32_t consumed_bytes = (uint32_t)((consumed_to_bit + 1 + 7) / 8);
        if (consumed_bytes >= r->len) {
            r->len = 0;
        } else {
            //Once again, is this too expensive??
            memmove(r->win, r->win + consumed_bytes, r->len - consumed_bytes);
            r->len -= consumed_bytes;
        }
        //IG this isn't useful per say, but made it return paylaod length to be consistent with code I saw in Thomas' code
        //And lowk, Claude recommended that I do return it
        return r->expect_payload;
    }

//Deletes the corrupted branch
    if (consumed_to_bit >= 0) {
        r->n_bad++;
        uint32_t consumed_bytes = (uint32_t)((consumed_to_bit + 1 + 7) / 8);
        if (consumed_bytes >= r->len) {
            r->len = 0;
        } else {
            memmove(r->win, r->win + consumed_bytes, r->len - consumed_bytes);
            r->len -= consumed_bytes;
        }
    }
    //No flags no problem-just don't do anything
    return 0;
}