#include <stdint.h>
#include <stddef.h>
#define LOG_BATCH_EMG 20
#define EMG_WIRE_SAMPLE_BYTES 3
static void put_emg_i24_be(uint8_t *dst, const int32_t *samples)
{
    for (size_t i = 0; i < LOG_BATCH_EMG; ++i) {
        uint32_t value = (uint32_t)samples[i];
        dst[0] = (uint8_t)(value >> 16);
        dst[1] = (uint8_t)(value >> 8);
        dst[2] = (uint8_t)value;
        dst += EMG_WIRE_SAMPLE_BYTES;
    }
}
void check_pack(uint8_t *dst, const int32_t *samples) { put_emg_i24_be(dst,samples); }
