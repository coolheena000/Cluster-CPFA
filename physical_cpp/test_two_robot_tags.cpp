// Camera-only test for two robots (no Wi-Fi, robots do not move):
// tag 1 = robot 05367, tag 2 = robot 05333, corner tags 10 11 12 13.
// Shows each robot's x, y (metres) and heading, and whether it is in the nest.
//
// Build:
//   export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
//   clang++ -std=c++17 test_two_robot_tags.cpp $(pkg-config --cflags --libs opencv5) -o test_two_robot_tags
// Run:   ./test_two_robot_tags        (camera 0 = USB webcam)
//        ./test_two_robot_tags 1      (camera 1 = MacBook camera)
// Quit:  Q in the window. Positions are also saved to two_robot_tags.csv

#include "CPFA_OverheadCamera.h"

#include <opencv2/highgui.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>

int main(int argc, char** argv) {
    const int cameraIndex = argc > 1 ? std::atoi(argv[1]) : 0;
    const double halfX = 1.06;   // half the distance between corner tag centres
    const double halfY = 0.685;
    const double nestRadius = 0.25;
    const char* names[2] = {"05367", "05333"};  // robot 0 = tag 1, robot 1 = tag 2

    cv::VideoCapture cap(cameraIndex);
    if (!cap.isOpened()) {
        std::cerr << "Camera " << cameraIndex << " not available\n";
        return 1;
    }

    cv::aruco::ArucoDetector detector(
        cv::aruco::getPredefinedDictionary(CPFAOverheadCamera::kTagDictionary));
    CPFAOverheadCamera arena(halfX, halfY);
    std::ofstream log("two_robot_tags.csv");
    log << "time_s,robot,x_m,y_m,heading_deg,in_nest\n";

    std::cout << "Camera " << cameraIndex << ". Tag 1 = 05367, tag 2 = 05333, corners 10 11 12 13. Q to quit.\n";
    const auto start = std::chrono::steady_clock::now();
    auto lastPrint = start;
    cv::Mat frame;

    while (cap.read(frame)) {
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        detector.detectMarkers(frame, corners, ids);
        arena.processFrame(frame);

        std::map<int, cv::Point2f> tagPx;
        for (std::size_t i = 0; i < ids.size(); ++i)
            tagPx[ids[i]] = (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) * 0.25f;

        cv::Mat view = frame.clone();
        if (!ids.empty()) cv::aruco::drawDetectedMarkers(view, corners, ids);
        cv::putText(view, arena.isCalibrated() ? "CALIBRATED" : "NOT CALIBRATED (need 10 11 12 13)",
                    {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    arena.isCalibrated() ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 255), 2);

        const auto now = std::chrono::steady_clock::now();
        const bool print = now - lastPrint > std::chrono::seconds(1);
        const double t = std::chrono::duration<double>(now - start).count();
        if (print) {
            lastPrint = now;
            std::printf("[%5.1fs] tags:", t);
            for (int id : ids) std::printf(" %d", id);
            if (ids.empty()) std::printf(" none");
            std::printf("  |");
        }

        for (int r = 0; r < 2; ++r) {
            const int tag = CPFAOverheadCamera::kFirstRobotTag + r;
            const auto p = arena.getPose(r);
            if (p.seen) {
                const double deg = p.heading * 180.0 / 3.141592653589793;
                const bool inNest = std::hypot(p.x, p.y) < nestRadius;
                char label[96];
                std::snprintf(label, sizeof(label), "%s (%.2f, %.2f) %.0f deg%s",
                              names[r], p.x, p.y, deg, inNest ? " NEST" : "");
                cv::putText(view, label, tagPx[tag] + cv::Point2f(30, -30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);
                if (print) {
                    std::printf("  %s x=%+.2f y=%+.2f heading=%+4.0f%s",
                                names[r], p.x, p.y, deg, inNest ? " IN NEST" : "");
                    log << t << ',' << names[r] << ',' << p.x << ',' << p.y << ','
                        << deg << ',' << inNest << '\n';
                }
            } else if (print) {
                std::printf("  %s %s", names[r],
                            tagPx.count(tag) ? "seen (not calibrated)" : "NOT SEEN");
            }
        }
        if (print) std::printf("\n");

        cv::imshow("Two robot tags", view);
        if ((cv::waitKey(1) & 0xFF) == 'q') break;
    }
    return 0;
}
