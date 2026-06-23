#include "event_store.hpp"

#include "hazard_event.hpp"

#include <iostream>
#include <sstream>

#if defined(VIGIA_HAVE_CURL)
#include <curl/curl.h>
#endif

namespace vigia {

namespace {

#if defined(VIGIA_HAVE_CURL)
size_t curlDiscardWrite(char* /*ptr*/, size_t size, size_t nmemb, void* /*userdata*/)
{
    return size * nmemb;
}
#endif

} // namespace

SyncClient::SyncClient()
    : SyncClient(Config{})
{}

SyncClient::SyncClient(Config cfg)
    : cfg_(std::move(cfg))
{}

bool SyncClient::postBatch(const std::string& jsonBody)
{
    if (cfg_.use_curl)
        return postWithCurl(jsonBody);
    return postToStdout(jsonBody);
}

bool SyncClient::postWithCurl(const std::string& jsonBody) const
{
#if defined(VIGIA_HAVE_CURL)
    CURL* curl = curl_easy_init();
    if (!curl)
        return false;

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, cfg_.endpoint.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonBody.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(jsonBody.size()));
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlDiscardWrite);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);

    const CURLcode res = curl_easy_perform(curl);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    return res == CURLE_OK;
#else
    (void)jsonBody;
    return false;
#endif
}

bool SyncClient::postToStdout(const std::string& jsonBody) const
{
    std::cout << "[vigia-sync] POST " << cfg_.endpoint << '\n'
              << jsonBody << '\n';
    return true;
}

EventStore::EventStore()
    : EventStore(Config{})
{}

EventStore::EventStore(Config cfg)
    : cfg_(std::move(cfg)),
      promoter_(cfg_.promoter),
      signer_(cfg_.signer),
      sync_client_(cfg_.sync)
{
    signer_.loadKey();
}

EventStore::~EventStore()
{
    stop();
}

void EventStore::start()
{
    if (running_.exchange(true))
        return;

#if defined(VIGIA_HAVE_CURL)
    curl_global_init(CURL_GLOBAL_DEFAULT);
#endif

    sync_thread_ = std::thread(&EventStore::syncLoop, this);
}

void EventStore::stop()
{
    if (!running_.exchange(false))
        return;

    if (sync_thread_.joinable())
        sync_thread_.join();

#if defined(VIGIA_HAVE_CURL)
    curl_global_cleanup();
#endif
}

void EventStore::syncLoop()
{
    std::vector<HazardObservation> batch;
    batch.reserve(cfg_.batch_size);

    const auto interval = std::chrono::milliseconds(
        static_cast<int>(cfg_.sync_interval_s * 1000.0f));

    auto last_flush = std::chrono::steady_clock::now();
    HazardObservation obs{};

    while (running_.load()) {
        if (promoter_.waitDequeue(obs, std::chrono::milliseconds(200))) {
            batch.push_back(obs);

            if (batch.size() >= cfg_.batch_size)
                flushBatch(batch);
        }

        const auto now = std::chrono::steady_clock::now();
        if (!batch.empty() && now - last_flush >= interval) {
            flushBatch(batch);
            last_flush = now;
        }
    }

    while (promoter_.tryDequeue(obs))
        batch.push_back(obs);

    if (!batch.empty())
        flushBatch(batch);
}

std::string EventStore::buildBatchJson(
    const std::vector<HazardObservation>& batch) const
{
    std::ostringstream oss;
    oss << "{\"events\":[";

    for (std::size_t i = 0; i < batch.size(); ++i) {
        if (i > 0)
            oss << ',';
        if (signer_.hasKey())
            oss << signer_.signEnvelope(batch[i]);
        else
            oss << hazardObservationToJson(batch[i]);
    }

    oss << "]}";
    return oss.str();
}

void EventStore::flushBatch(std::vector<HazardObservation>& batch)
{
    if (batch.empty())
        return;

    const std::string body = buildBatchJson(batch);
    if (!sync_client_.postBatch(body)) {
        std::cerr << "[EventStore] sync failed (" << batch.size() << " events)\n";
    }

    batch.clear();
}

} // namespace vigia
