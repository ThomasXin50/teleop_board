#include "crc32.h"

uint32_t crc32_ieee(const uint8_t *data, size_t n) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint32_t)data[i];
        for (int k = 0; k < 8; k++) {
            uint32_t mask = -(int32_t)(crc & 1u); 
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}