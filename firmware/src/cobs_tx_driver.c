/**
 * cobs_tx_driver.c — COBS encoder (no dynamic allocation)
 *
 * Frame layout: 0x00 | <overhead byte> | <payload bytes, 0x00 replaced> | 0x00
 * The overhead byte at each code position holds the distance to the next 0x00
 * (or end-of-group). Values 0x01-0xFE encode 0-253 non-zero bytes between
 * stuffed zeros. 0xFF means 254 non-zero bytes follow with no implicit zero.
 */

#include "cobs_tx_driver.h"
#include <string.h>

size_t cobs_encode(const uint8_t *src, size_t src_len,
                   uint8_t *out,       size_t out_cap) {
    if (!src || !out) return 0;
    if (src_len == 0 || src_len > COBS_MAX_PAYLOAD) return 0;
    /* Worst case: every byte is 0x00 → one overhead byte per payload byte + 2 delimiters */
    if (out_cap < src_len + 2 + (src_len / 254) + 2) return 0;

    size_t write    = 0;
    size_t code_pos = 0;     /* index of current overhead byte in out[] */
    uint8_t code    = 1;

    out[write++] = 0x00;     /* leading delimiter */
    code_pos     = write++;  /* reserve overhead byte slot */

    for (size_t i = 0; i < src_len; ++i) {
        if (src[i] != 0x00) {
            out[write++] = src[i];
            ++code;
            if (code == 0xFF) {
                /* Flush: 254 non-zero bytes accumulated, no implicit zero inserted */
                out[code_pos] = code;
                code_pos      = write++;
                code          = 1;
            }
        } else {
            /* Zero byte: terminate current group */
            out[code_pos] = code;
            code_pos      = write++;
            code          = 1;
        }
    }
    out[code_pos] = code;    /* finalize last overhead byte */
    out[write++]  = 0x00;    /* trailing delimiter */
    return write;
}
