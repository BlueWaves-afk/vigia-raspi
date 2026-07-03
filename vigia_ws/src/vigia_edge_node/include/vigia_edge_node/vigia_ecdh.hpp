#pragma once
#ifdef VIGIA_HAVE_MBEDTLS
/**
 * vigia_ecdh.hpp — Pi-side P-256 identity key + ECDH/ECDSA/HKDF/HMAC helpers.
 *
 * Design spec §4.2a: Pi uses a static P-256 identity key stored at
 * /etc/vigia/pi_p256.der (PKCS#8 DER, unencrypted).  Generated once by
 * tools/vigia-pair-qr.py --provision.
 *
 * Thread-safety: VigiaIdentityKey is read-only after construction.
 * The ctr_drbg context is guarded by an internal mutex for sign/ecdh calls.
 */

#include <array>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

// Allow access to MBEDTLS_PRIVATE() members (mbedTLS 3.x opaque struct policy).
#define MBEDTLS_ALLOW_PRIVATE_ACCESS
#include <sys/stat.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/ecdh.h>
#include <mbedtls/ecdsa.h>
#include <mbedtls/entropy.h>
#include <mbedtls/hkdf.h>
#include <mbedtls/md.h>
#include <mbedtls/pk.h>
#include <mbedtls/sha256.h>

namespace vigia {

// ── Free functions ────────────────────────────────────────────────────────────

/** HKDF-SHA256 (RFC 5869). */
inline std::vector<uint8_t> hkdf_sha256(
    const uint8_t* ikm,  size_t ikm_len,
    const uint8_t* salt, size_t salt_len,
    const uint8_t* info, size_t info_len,
    size_t out_len = 32)
{
    std::vector<uint8_t> out(out_len);
    const mbedtls_md_info_t* md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    int rc = mbedtls_hkdf(md, salt, salt_len, ikm, ikm_len, info, info_len,
                          out.data(), out_len);
    if (rc != 0)
        throw std::runtime_error("vigia_hkdf_sha256 failed: " + std::to_string(rc));
    return out;
}

/** HMAC-SHA256 one-shot. */
inline std::vector<uint8_t> hmac_sha256(
    const uint8_t* key, size_t key_len,
    const uint8_t* data, size_t data_len)
{
    std::vector<uint8_t> out(32);
    const mbedtls_md_info_t* md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (mbedtls_md_hmac(md, key, key_len, data, data_len, out.data()) != 0)
        throw std::runtime_error("vigia_hmac_sha256 failed");
    return out;
}

/** SHA-256 one-shot. */
inline std::array<uint8_t, 32> sha256(const uint8_t* data, size_t len) {
    std::array<uint8_t, 32> out{};
    mbedtls_sha256(data, len, out.data(), /*is224=*/0);
    return out;
}

// ── VigiaIdentityKey ──────────────────────────────────────────────────────────

/**
 * Pi's static P-256 identity key.  Loads from [key_path] (PKCS#8 DER); throws
 * std::runtime_error if the file is missing or corrupt.
 *
 * Use VigiaIdentityKey::generate(path) once at provisioning time to create the key.
 */
class VigiaIdentityKey {
public:
    static constexpr const char* kDefaultPath = "/etc/vigia/pi_p256.der";

    explicit VigiaIdentityKey(const std::string& key_path = kDefaultPath) {
        mbedtls_pk_init(&pk_);
        mbedtls_entropy_init(&entropy_);
        mbedtls_ctr_drbg_init(&ctr_drbg_);

        const char* pers = "vigia_ble";
        if (mbedtls_ctr_drbg_seed(&ctr_drbg_, mbedtls_entropy_func, &entropy_,
                                   reinterpret_cast<const uint8_t*>(pers), strlen(pers)) != 0)
            throw std::runtime_error("VigiaIdentityKey: ctr_drbg_seed failed");

        if (mbedtls_pk_parse_keyfile(&pk_, key_path.c_str(), nullptr,
                                      mbedtls_ctr_drbg_random, &ctr_drbg_) != 0)
            throw std::runtime_error("VigiaIdentityKey: failed to load key from " + key_path);

        if (mbedtls_pk_get_type(&pk_) != MBEDTLS_PK_ECKEY)
            throw std::runtime_error("VigiaIdentityKey: key is not EC type");
    }

    ~VigiaIdentityKey() {
        mbedtls_pk_free(&pk_);
        mbedtls_ctr_drbg_free(&ctr_drbg_);
        mbedtls_entropy_free(&entropy_);
    }

    VigiaIdentityKey(const VigiaIdentityKey&) = delete;
    VigiaIdentityKey& operator=(const VigiaIdentityKey&) = delete;

    /** Generate a new P-256 keypair and write it to [key_path] (PKCS#8 DER). */
    static void generate(const std::string& key_path = kDefaultPath) {
        mbedtls_pk_context pk;
        mbedtls_entropy_context entropy;
        mbedtls_ctr_drbg_context ctr_drbg;

        mbedtls_pk_init(&pk);
        mbedtls_entropy_init(&entropy);
        mbedtls_ctr_drbg_init(&ctr_drbg);

        const char* pers = "vigia_keygen";
        mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                               reinterpret_cast<const uint8_t*>(pers), strlen(pers));
        mbedtls_pk_setup(&pk, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
        mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1,
                            mbedtls_pk_ec(pk),
                            mbedtls_ctr_drbg_random, &ctr_drbg);

        uint8_t der_buf[256];
        int der_len = mbedtls_pk_write_key_der(&pk, der_buf, sizeof(der_buf));
        if (der_len <= 0)
            throw std::runtime_error("VigiaIdentityKey::generate: pk_write_key_der failed");

        // pk_write_key_der writes to the END of the buffer.
        const uint8_t* der_start = der_buf + sizeof(der_buf) - der_len;

        FILE* f = fopen(key_path.c_str(), "wb");
        if (!f) throw std::runtime_error("VigiaIdentityKey::generate: cannot write to " + key_path);
        fwrite(der_start, 1, static_cast<size_t>(der_len), f);
        fclose(f);
        chmod(key_path.c_str(), 0600);

        mbedtls_pk_free(&pk);
        mbedtls_ctr_drbg_free(&ctr_drbg);
        mbedtls_entropy_free(&entropy);
    }

    /** Returns the public key as an uncompressed P-256 point (65 bytes: 0x04 || X || Y). */
    std::vector<uint8_t> public_key_uncompressed() const {
        std::lock_guard<std::mutex> lk(mu_);
        mbedtls_ecp_keypair* kp = mbedtls_pk_ec(pk_);
        uint8_t buf[65];
        size_t olen = 0;
        if (mbedtls_ecp_point_write_binary(&kp->grp, &kp->Q,
                MBEDTLS_ECP_PF_UNCOMPRESSED, &olen, buf, sizeof(buf)) != 0 || olen != 65)
            throw std::runtime_error("VigiaIdentityKey: failed to export public key");
        return {buf, buf + 65};
    }

    /**
     * ECDSA-SHA256 sign.  Returns a DER-encoded signature (~70 bytes).
     * Signs the SHA-256 hash of [data, len].
     */
    std::vector<uint8_t> sign(const uint8_t* data, size_t len) const {
        std::lock_guard<std::mutex> lk(mu_);
        auto hash = vigia::sha256(data, len);

        uint8_t sig_buf[MBEDTLS_ECDSA_MAX_LEN];
        size_t sig_len = 0;

        mbedtls_ecdsa_context ecdsa;
        mbedtls_ecdsa_init(&ecdsa);
        mbedtls_ecdsa_from_keypair(&ecdsa, mbedtls_pk_ec(pk_));

        int rc = mbedtls_ecdsa_write_signature(
            &ecdsa, MBEDTLS_MD_SHA256,
            hash.data(), hash.size(),
            sig_buf, sizeof(sig_buf), &sig_len,
            mbedtls_ctr_drbg_random,
            const_cast<mbedtls_ctr_drbg_context*>(&ctr_drbg_));

        mbedtls_ecdsa_free(&ecdsa);
        if (rc != 0)
            throw std::runtime_error("VigiaIdentityKey::sign failed: " + std::to_string(rc));

        return {sig_buf, sig_buf + sig_len};
    }

    /**
     * ECDH: compute 32-byte raw shared secret with [peer_pub_65] (uncompressed P-256, 65 bytes).
     * Callers must apply HKDF before use as a key.
     */
    std::vector<uint8_t> ecdh_shared_secret(const uint8_t* peer_pub_65) const {
        std::lock_guard<std::mutex> lk(mu_);
        mbedtls_ecp_keypair* kp = mbedtls_pk_ec(pk_);

        mbedtls_ecp_point Q_peer;
        mbedtls_mpi shared;
        mbedtls_ecp_point_init(&Q_peer);
        mbedtls_mpi_init(&shared);

        int rc = mbedtls_ecp_point_read_binary(&kp->grp, &Q_peer, peer_pub_65, 65);
        if (rc != 0) {
            mbedtls_ecp_point_free(&Q_peer); mbedtls_mpi_free(&shared);
            throw std::runtime_error("ecdh_shared_secret: invalid peer public key");
        }

        rc = mbedtls_ecdh_compute_shared(
            &kp->grp, &shared, &Q_peer, &kp->d,
            mbedtls_ctr_drbg_random,
            const_cast<mbedtls_ctr_drbg_context*>(&ctr_drbg_));

        if (rc != 0) {
            mbedtls_ecp_point_free(&Q_peer); mbedtls_mpi_free(&shared);
            throw std::runtime_error("ecdh_shared_secret: compute_shared failed: " + std::to_string(rc));
        }

        std::vector<uint8_t> out(32, 0);
        mbedtls_mpi_write_binary(&shared, out.data(), 32);
        mbedtls_ecp_point_free(&Q_peer);
        mbedtls_mpi_free(&shared);
        return out;
    }

    /**
     * Verify an ECDSA-SHA256 signature from a peer's uncompressed P-256 public key.
     *
     * When data_len == 32, `data` is treated as a precomputed SHA-256 digest
     * (ATECC608A / SignedEt path).  Otherwise `data` is hashed first (BLE handshake).
     * Signature must be 64-byte IEEE P1363 raw R||S (ATECC608A output format).
     * Returns true iff the signature is valid.  Never throws.
     */
    static bool verify_peer(
        const uint8_t* peer_pub_65,
        const uint8_t* data, size_t data_len,
        const uint8_t* sig,  size_t sig_len)
    {
        if (!peer_pub_65 || !data || !sig || sig_len != 64)
            return false;

        try {
            std::array<uint8_t, 32> digest_buf{};
            const uint8_t* digest = data;
            size_t digest_len = data_len;
            if (data_len != 32) {
                digest_buf = sha256(data, data_len);
                digest = digest_buf.data();
                digest_len = digest_buf.size();
            }

            mbedtls_ecdsa_context ecdsa;
            mbedtls_ecdsa_init(&ecdsa);
            mbedtls_ecp_group_load(&ecdsa.grp, MBEDTLS_ECP_DP_SECP256R1);

            if (mbedtls_ecp_point_read_binary(&ecdsa.grp, &ecdsa.Q, peer_pub_65, 65) != 0) {
                mbedtls_ecdsa_free(&ecdsa);
                return false;
            }

            mbedtls_mpi r, s;
            mbedtls_mpi_init(&r);
            mbedtls_mpi_init(&s);
            int rc = -1;
            if (mbedtls_mpi_read_binary(&r, sig, 32) == 0 &&
                mbedtls_mpi_read_binary(&s, sig + 32, 32) == 0) {
                rc = mbedtls_ecdsa_verify(
                    &ecdsa.grp, digest, digest_len, &ecdsa.Q, &r, &s);
            }
            mbedtls_mpi_free(&r);
            mbedtls_mpi_free(&s);
            mbedtls_ecdsa_free(&ecdsa);
            return rc == 0;
        } catch (...) { return false; }
    }

private:
    mutable mbedtls_pk_context       pk_;
    mutable mbedtls_entropy_context  entropy_;
    mutable mbedtls_ctr_drbg_context ctr_drbg_;
    mutable std::mutex               mu_;
};

} // namespace vigia
#endif // VIGIA_HAVE_MBEDTLS
