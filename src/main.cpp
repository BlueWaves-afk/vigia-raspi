#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <utility>

#include <opencv2/core.hpp>

#include "analytical.hpp"
#include "coordinator.hpp"
#include "fusion.hpp"
#include "perception.hpp"
#include "safe_queue.hpp"

namespace {

std::atomic<bool> g_running{true};

void signalHandler(int)
{
    g_running.store(false);
}

#ifdef __linux__
#include <pthread.h>
#include <sched.h>

void pinThreadToCore(std::thread& thread, int coreId)
{
    cpu_set_t cpuSet;
    CPU_ZERO(&cpuSet);
    CPU_SET(coreId, &cpuSet);
    int result = pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set_t), &cpuSet);
    if (result != 0) {
        std::cerr << "Unable to pin thread to core " << coreId << ": " << result << '\n';
    }
}
#else
void pinThreadToCore(std::thread&, int)
{
    // Thread pinning is not available on this platform.
}
#endif

template <typename Fn>
std::thread spawnPinnedThread(Fn&& fn, int coreId)
{
    std::thread worker(std::forward<Fn>(fn));
    pinThreadToCore(worker, coreId);
    return worker;
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string yoloXml = (argc > 1) ? argv[1] : "models/yolo11n.xml";
    const std::string yoloBin = (argc > 2) ? argv[2] : "models/yolo11n.bin";
    const std::string midasXml = (argc > 3) ? argv[3] : "models/midas_v21_small.xml";
    const std::string midasBin = (argc > 4) ? argv[4] : "models/midas_v21_small.bin";

    SafeQueue<vigia::FramePacket> perceptionQueue;
    SafeQueue<vigia::AnalyticalRequest> analyticalQueue;
    SafeQueue<vigia::PerceptionResult> perceptionResults;
    SafeQueue<vigia::AnalyticalResult> analyticalResults;
    SafeQueue<vigia::FusionState> fusionQueue;

    vigia::CoordinatorConfig coordinatorConfig;
    coordinatorConfig.frameBufferSize = 16;
    coordinatorConfig.initialFrameSkip = 1;
    coordinatorConfig.thermalCheckInterval = std::chrono::milliseconds{750};
    coordinatorConfig.thermalThresholdC = 75.0F;

    vigia::Coordinator coordinator(coordinatorConfig,
                                   perceptionQueue,
                                   analyticalQueue,
                                   perceptionResults,
                                   analyticalResults,
                                   fusionQueue);

    vigia::PerceptionAgent perception(yoloXml, yoloBin, "CPU");

    // Camera intrinsics tuned for 256x256 downsampled frames.
    vigia::CameraIntrinsics intrinsics{425.0F, 425.0F, 128.0F, 128.0F};
    vigia::RansacParameters ransacParams{300, 0.04F, 0.55F};
    vigia::AnalyticalAgent analytical(intrinsics, ransacParams);

    vigia::FusionWeights weights{0.5F, 0.35F, 0.15F};
    vigia::FusionEngine fusion(weights);

    std::thread coordinatorThread = spawnPinnedThread([&]() { coordinator.run(g_running); }, 0);
    std::thread perceptionThread = spawnPinnedThread([
                                                        &]() {
        perception.run(perceptionQueue, perceptionResults, g_running);
    }, 1);
    std::thread analyticalThread = spawnPinnedThread([
                                                       &]() {
        analytical.run(analyticalQueue, analyticalResults, g_running);
    }, 2);
    std::thread fusionThread = spawnPinnedThread([
                                                    &]() {
        fusion.run(analyticalResults, fusionQueue, g_running);
    }, 3);

    while (g_running.load()) {
        if (auto fused = fusionQueue.try_pop()) {
            std::cout << "[RRI] frame=" << fused->frameId << " score=" << fused->rri
                      << " residual=" << fused->geometricResidual
                      << " persistence=" << fused->persistence << '\n';
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
        }
    }

    perceptionQueue.push({});
    analyticalQueue.push({});

    if (coordinatorThread.joinable()) {
        coordinatorThread.join();
    }
    if (perceptionThread.joinable()) {
        perceptionThread.join();
    }
    if (analyticalThread.joinable()) {
        analyticalThread.join();
    }
    if (fusionThread.joinable()) {
        fusionThread.join();
    }

    return 0;
}