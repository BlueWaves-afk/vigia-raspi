#include "ecdsa_verify.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

#if defined(VIGIA_HAVE_OPENSSL)
#include <openssl/bn.h>
#include <openssl/ec.h>
#include <openssl/ecdsa.h>
#include <openssl/evp.h>
#include <openssl/obj_mac.h>
#endif

namespace vigia {

namespace {

std::string trim(const std::string& s)
{
    std::size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start])))
        ++start;
    std::size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1])))
        --end;
    return s.substr(start, end - start);
}

} // namespace

std::optional<std::array<std::uint8_t, 64>> loadPubkeyHexFile(
    const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open())
        return std::nullopt;

    std::ostringstream oss;
    oss << file.rdbuf();
    std::string hex = trim(oss.str());

    for (char& c : hex) {
        if (c == ':' || c == ' ' || c == '\n' || c == '\r' || c == '\t')
            continue;
    }

    std::string cleaned;
    cleaned.reserve(hex.size());
    for (char c : hex) {
        if (std::isxdigit(static_cast<unsigned char>(c)))
            cleaned.push_back(static_cast<char>(std::tolower(c)));
    }

    if (cleaned.size() != 128)
        return std::nullopt;

    std::array<std::uint8_t, 64> out{};
    for (std::size_t i = 0; i < 64; ++i) {
        const std::string byte = cleaned.substr(i * 2, 2);
        out[i] = static_cast<std::uint8_t>(std::stoul(byte, nullptr, 16));
    }
    return out;
}

EcdsaVerifier::EcdsaVerifier()
    : EcdsaVerifier(Config{}) {}

EcdsaVerifier::EcdsaVerifier(Config cfg)
    : cfg_(std::move(cfg))
{}

bool EcdsaVerifier::loadPublicKey()
{
    pubkey_.fill(0);
    if (cfg_.pubkey_file.empty())
        return false;

    const auto loaded = loadPubkeyHexFile(cfg_.pubkey_file);
    if (!loaded)
        return false;

    pubkey_ = *loaded;
    return true;
}

bool EcdsaVerifier::isStubSignature(const std::uint8_t sig[64]) const
{
    if (!sig)
        return false;
    return std::all_of(sig, sig + 64,
                       [](std::uint8_t b) { return b == 0x00; });
}

bool EcdsaVerifier::verify(const std::uint8_t hash[32],
                           const std::uint8_t sig[64]) const
{
    if (!hash || !sig)
        return false;

    if (isStubSignature(sig))
        return cfg_.allow_stub_sig;

#if defined(VIGIA_HAVE_OPENSSL)
    if (!hasKey())
        return false;

    EC_KEY* ec_key = EC_KEY_new_by_curve_name(NID_X9_62_prime256v1);
    if (!ec_key)
        return false;

    BIGNUM* bx = BN_bin2bn(pubkey_.data(), 32, nullptr);
    BIGNUM* by = BN_bin2bn(pubkey_.data() + 32, 32, nullptr);
    if (!bx || !by) {
        BN_free(bx);
        BN_free(by);
        EC_KEY_free(ec_key);
        return false;
    }

    const int set_ok = EC_KEY_set_public_key_affine_coordinates(ec_key, bx, by);
    BN_free(bx);
    BN_free(by);
    if (set_ok != 1) {
        EC_KEY_free(ec_key);
        return false;
    }

    ECDSA_SIG* ecdsa_sig = ECDSA_SIG_new();
    if (!ecdsa_sig) {
        EC_KEY_free(ec_key);
        return false;
    }

    BIGNUM* r = BN_bin2bn(sig, 32, nullptr);
    BIGNUM* s = BN_bin2bn(sig + 32, 32, nullptr);
    if (!r || !s) {
        BN_free(r);
        BN_free(s);
        ECDSA_SIG_free(ecdsa_sig);
        EC_KEY_free(ec_key);
        return false;
    }

#if OPENSSL_VERSION_NUMBER >= 0x10100000L
    ECDSA_SIG_set0(ecdsa_sig, r, s);
#else
    ecdsa_sig->r = r;
    ecdsa_sig->s = s;
#endif

    const int ok = ECDSA_do_verify(hash, 32, ecdsa_sig, ec_key);
    ECDSA_SIG_free(ecdsa_sig);
    EC_KEY_free(ec_key);
    return ok == 1;
#else
    (void)hash;
    (void)sig;
    return false;
#endif
}

} // namespace vigia
