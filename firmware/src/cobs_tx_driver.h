#pragma once
/**
 * cobs_tx_driver.h — COBS encoder for Vigia Phase 2 USB CDC framing
 *
 * Consistent Overhead Byte Stuffing (COBS) encodes a binary payload
 * into a frame with 0x00 delimiters:
 *   [0x00][encoded bytes][0x00]
 *
 * No dynamic allocation; static internal buffer; NOT reentrant.
 * Maximum payload: COBS_MAX_PAYLOAD bytes.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COBS_MAX_PAYLOAD  200
#define COBS_FRAME_MAX    (COBS_MAX_PAYLOAD + 4)  /* +2 delimiters + overhead */

/**
 * Encode `src` (src_len bytes) using COBS into `out`.
 *
 * `out` must be at least COBS_FRAME_MAX bytes.
 * Returns total frame length (including both 0x00 delimiters) on success.
 * Returns 0 if src_len > COBS_MAX_PAYLOAD or out_cap is insufficient.
 */
size_t cobs_encode(const uint8_t *src, size_t src_len,
                   uint8_t *out,       size_t out_cap);

#ifdef __cplusplus
}
#endif
