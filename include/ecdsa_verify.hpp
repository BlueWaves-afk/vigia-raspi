#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace vigia {

class EcdsaVerifier {
public:
    struct Config {
        std::string pubkey_file;  // 64-byte raw secp256r1 pubkey (hex) or PEM
        bool allow_stub_sig{false};
    };

    explicit EcdsaVerifier();
    explicit EcdsaVerifier(Config cfg);

    bool loadPublicKey();
    bool hasKey() const { return pubkey_.size() == 64; }

    /** Verify ECDSA-SHA256 over et_hash (32 bytes) with R||S signature (64 bytes). */
    bool verify(const std::uint8_t hash[32],
                const std::uint8_t sig[64]) const;

    bool isStubSignature(const std::uint8_t sig[64]) const;

private:
    Config cfg_;
    std::array<std::uint8_t, 64> pubkey_{};
};

/** Load hex pubkey file (128 hex chars = 64 bytes X||Y). */
std::optional<std::array<std::uint8_t, 64>> loadPubkeyHexFile(
    const std::string& path);

} // namespace vigia
