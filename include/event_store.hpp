#pragma once

#include "event_promoter.hpp"
#include "event_signer.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

namespace vigia {

class SyncClient {
public:
    struct Config {
        std::string endpoint;
        bool use_curl{false};

        Config() : endpoint("http://127.0.0.1:8080/v1/events") {}
    };

    explicit SyncClient(Config cfg);
    SyncClient();

    bool postBatch(const std::string& jsonBody);

private:
    bool postWithCurl(const std::string& jsonBody) const;
    bool postToStdout(const std::string& jsonBody) const;

    Config cfg_;
};

class EventStore {
public:
    struct Config {
        EventPromoter::Config promoter;
        EventSigner::Config signer;
        SyncClient::Config sync;
        std::size_t batch_size{50};
        float sync_interval_s{5.0f};
        bool dev_stdout_fallback{true};
    };

    explicit EventStore(Config cfg);
    EventStore();
    ~EventStore();

    EventStore(const EventStore&) = delete;
    EventStore& operator=(const EventStore&) = delete;

    void start();
    void stop();

    EventPromoter& promoter() { return promoter_; }
    const EventPromoter& promoter() const { return promoter_; }

private:
    void syncLoop();
    std::string buildBatchJson(const std::vector<HazardObservation>& batch) const;
    void flushBatch(std::vector<HazardObservation>& batch);

    Config cfg_;
    EventPromoter promoter_;
    EventSigner signer_;
    SyncClient sync_client_;

    std::thread sync_thread_;
    std::atomic<bool> running_{false};
};

} // namespace vigia
