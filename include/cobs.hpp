#pragma once

#include <cstddef>
#include <cstdint>

namespace vigia {

/** Decode COBS-framed bytes (without leading/trailing 0x00 delimiters). */
std::size_t cobsDecode(const std::uint8_t* src, std::size_t src_len,
                       std::uint8_t* dst, std::size_t dst_cap);

/** Encode payload using COBS (includes leading/trailing 0x00 delimiters). */
std::size_t cobsEncode(const std::uint8_t* src, std::size_t src_len,
                       std::uint8_t* out, std::size_t out_cap);

} // namespace vigia
