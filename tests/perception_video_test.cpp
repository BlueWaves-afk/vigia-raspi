#include "perception.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>
#include <iomanip>

using namespace vigia;

/*
BUILD COMMAND:

clang++ -std=c++17 \
 tests/perception_video_test.cpp \
 src/perception.cpp \
 -Iinclude \
 -I/opt/homebrew/include/opencv4 \
 -I/opt/homebrew/opt/openvino/include \
 -L/opt/homebrew/lib \
 -L/opt/homebrew/opt/openvino/lib \
 -Wl,-rpath,/opt/homebrew/opt/openvino/lib \
 -lopenvino \
 -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui \
 -pthread -O3 \
 -o perception_video_test
*/

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: perception_video_test <video.mp4>\n";
        return 1;
    }

    const std::string videoPath = argv[1];

    PerceptionAgent agent(
        "models/yolo26/yolo26_model.xml",
        "CPU"
    );

    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open video: " << videoPath << "\n";
        return 1;
    }

    cv::namedWindow("YOLO Detections", cv::WINDOW_NORMAL);

    std::size_t frameIdx = 0;

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty())
            break;

        frameIdx++;

        auto detections = agent.runInference(frame);

        cv::Mat vis = frame.clone();

        std::cout << "[FRAME " << frameIdx << "] detections=" << detections.size();

        float maxConf = 0.0f;
        for (const auto& d : detections)
            maxConf = std::max(maxConf, d.confidence);

        std::cout << " maxConf=" << std::fixed << std::setprecision(2)
                  << maxConf << "\n";

        for (const auto& det : detections) {
            const cv::Rect& box = det.boundingBox;

            cv::rectangle(vis, box, {0, 255, 255}, 2);

            std::ostringstream label;
            label << "ID=" << det.classId
                  << " C=" << std::fixed << std::setprecision(2)
                  << det.confidence;

            cv::putText(
                vis,
                label.str(),
                {box.x, std::max(12, box.y - 6)},
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                {0, 255, 255},
                1,
                cv::LINE_AA
            );
        }

        cv::imshow("YOLO Detections", vis);

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
            break;
    }

    cv::destroyAllWindows();
    std::cout << "[TEST] Perception video test completed\n";
    return 0;
}
