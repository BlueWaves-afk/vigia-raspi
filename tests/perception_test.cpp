#include "perception.hpp"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <thread>
#include <atomic>

using namespace vigia;

int main() {
    PerceptionAgent agent(
        "models/yolo26/yolo26_model.xml",
        "CPU"
    );

    cv::Mat img = cv::imread("pothole_image.jpeg");
    std::cerr << "Loaded test image\n";
    if (img.empty()) {
        std::cerr << "Failed to load test image\n";
        return 1;
    }

    SafeQueue<FramePacket> inQ;
    SafeQueue<PerceptionResult> outQ;
    std::atomic<bool> running{true};

    FramePacket pkt;
    pkt.frameId = 1;
    pkt.frame = img;
    pkt.timestamp = std::chrono::steady_clock::now();

    inQ.push(pkt);

    std::thread t([&] {
        agent.run(inQ, outQ, running);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    running.store(false);
    t.join();

    auto res = outQ.try_pop();
    if (res) {
        std::cout << "Detections: " << res->detections.size() << "\n";
        std::cout << "Max confidence: " << res->greatestConfidence << "\n";
    }
}
