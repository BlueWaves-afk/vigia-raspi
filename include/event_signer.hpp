#pragma once

#include "hazard_event.hpp"

#include <cstddef>
#include <string>

namespace vigia {

class EventSigner {
public:
    struct Config {
        std::string hmac_key_file;
    };

    explicit EventSigner(Config cfg = Config{});

    bool loadKey();
    bool hasKey() const { return !hmac_key_.empty(); }

    /* Build signed ingest envelope JSON from a hazard observation. */
    std::string signEnvelope(const HazardObservation& obs) const;

private:
    std::string canonicalPayload(const HazardObservation& obs,
                                const std::string& eventId,
                                const std::string& observedAt) const;
    std::string sha256Hex(const std::string& data) const;
    std::string hmacSha256Base64(const std::string& data) const;

    Config cfg_;
    std::string hmac_key_;
};

} // namespace vigia
