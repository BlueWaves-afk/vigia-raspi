#include "cobs.hpp"

namespace vigia {

std::size_t cobsDecode(const std::uint8_t* src, std::size_t src_len,
                       std::uint8_t* dst, std::size_t dst_cap)
{
    if (!src || !dst || src_len < 2)
        return 0;

    std::size_t write = 0;
    std::size_t read = 0;

    while (read < src_len) {
        const std::uint8_t code = src[read++];
        if (code == 0)
            break;

        for (std::uint8_t i = 1; i < code; ++i) {
            if (read >= src_len || write >= dst_cap)
                return 0;
            dst[write++] = src[read++];
        }

        if (code < 0xFF) {
            if (write >= dst_cap)
                return 0;
            dst[write++] = 0x00;
        }
    }

    if (write > 0 && dst[write - 1] == 0x00)
        --write;

    return write;
}

std::size_t cobsEncode(const std::uint8_t* src, std::size_t src_len,
                       std::uint8_t* out, std::size_t out_cap)
{
    if (!src || !out || src_len == 0)
        return 0;

    std::size_t write = 0;
    std::size_t code_pos = 0;
    std::uint8_t code = 1;

    // Worst-case COBS output: src_len data bytes + ceil(src_len/254) overhead
    // bytes + 1 leading delimiter + 1 trailing delimiter.
    const std::size_t worst_case = src_len + (src_len / 254) + 3;
    if (out_cap < worst_case)
        return 0;

    out[write++] = 0x00;
    code_pos = write++;

    for (std::size_t i = 0; i < src_len; ++i) {
        if (src[i] != 0x00) {
            out[write++] = src[i];
            ++code;
            if (code == 0xFF) {
                out[code_pos] = code;
                code_pos = write++;
                code = 1;
            }
        } else {
            out[code_pos] = code;
            code_pos = write++;
            code = 1;
        }
    }

    out[code_pos] = code;
    out[write++] = 0x00;
    return write;
}

} // namespace vigia
