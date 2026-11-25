// Camera setup helper (no robots): shows the overhead camera live and prints
// which tag36h11 tags it sees, so the camera and corner tags can be placed.
//
// Build:
//   export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
//   clang++ -std=c++17 test_camera_view.cpp $(pkg-config --cflags --libs opencv5) -o test_camera_view
// Run:   ./test_camera_view          (camera 0)
//        ./test_camera_view 1        (camera 1, e.g. iPhone Continuity Camera)
// Quit:  Q in the window

#include "CPFA_OverheadCamera.h"

#include <opencv2/highgui.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
    const int cameraIndex = argc > 1 ? std::atoi(argv[1]) : 0;
    const double halfX = argc > 3 ? std::atof(argv[2]) : 1.06;
    const double halfY = argc > 3 ? std::atof(argv[3]) : 0.685;

    cv::VideoCapture cap(cameraIndex);
    if (!cap.isOpened()) {
        std::cerr << "Camera " << cameraIndex << " not available\n";
        return 1;
    }

    cv::aruco::ArucoDetector detector(
        cv::aruco::getPredefinedDictionary(CPFAOverheadCamera::kTagDictionary));
    CPFAOverheadCamera arena(halfX, halfY);

    std::cout << "Showing camera " << cameraIndex << ". Need corner tags 10 11 12 13. Q to quit.\n";
    auto lastPrint = std::chrono::steady_clock::now();
    cv::Mat frame;

    while (cap.read(frame)) {
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        detector.detectMarkers(frame, corners, ids);
        arena.processFrame(frame);

        cv::Mat view = frame.clone();
        if (!ids.empty()) cv::aruco::drawDetectedMarkers(view, corners, ids);
        cv::putText(view, arena.isCalibrated() ? "CALIBRATED" : "NOT CALIBRATED (need 10 11 12 13)",
                    {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    arena.isCalibrated() ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 255), 2);
        cv::imshow("Camera check", view);
        if ((cv::waitKey(1) & 0xFF) == 'q') break;

        const auto now = std::chrono::steady_clock::now();
        if (now - lastPrint > std::chrono::seconds(1)) {
            lastPrint = now;
            std::printf("%dx%d  tags seen:", frame.cols, frame.rows);
            for (int id : ids) std::printf(" %d", id);
            if (ids.empty()) std::printf(" none");
            std::printf("  |  %s", arena.isCalibrated() ? "CALIBRATED" : "not calibrated");
            if (arena.isCalibrated()) {
                for (int r = 0; r < CPFAOverheadCamera::kRobotCount; ++r) {
                    const auto p = arena.getPose(r);
                    if (p.seen) std::printf("  R%d(%.2f,%.2f)", r, p.x, p.y);
                }
                std::printf("  food:%zu", arena.foodSeen().size());
            }
            std::printf("\n");
        }
    }
    return 0;
}
