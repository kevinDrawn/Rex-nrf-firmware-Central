#include <stdint.h>
static inline int32_t bytes_to_i24_be(const uint8_t *data)
{
    uint32_t value = ((uint32_t)data[0] << 16) |
                     ((uint32_t)data[1] << 8) |
                     (uint32_t)data[2];

    /* Sign-extend the ADS1294 two's-complement 24-bit sample. */
    return (int32_t)(value ^ 0x800000U) - 0x800000;
}
int32_t check_i24(const uint8_t *data) { return bytes_to_i24_be(data); }
