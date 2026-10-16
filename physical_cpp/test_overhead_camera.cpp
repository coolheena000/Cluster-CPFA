// Offline test: synthetic arena image with known marker poses.
#include "CPFA_OverheadCamera.h"

#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

static void putMarker(cv::Mat& img, int id, cv::Point centre, int rotationClockwise) {
    cv::Mat m;
    cv::aruco::generateImageMarker(
        cv::aruco::getPredefinedDictionary(CPFAOverheadCamera::kTagDictionary), id, 64, m);
    if (rotationClockwise == 90) cv::rotate(m, m, cv::ROTATE_90_CLOCKWISE);
    if (rotationClockwise == 180) cv::rotate(m, m, cv::ROTATE_180);
    if (rotationClockwise == 270) cv::rotate(m, m, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::copyMakeBorder(m, m, 10, 10, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(255));
    m.copyTo(img(cv::Rect(centre.x - m.cols / 2, centre.y - m.rows / 2, m.cols, m.rows)));
}

int main() {
    const double halfX = 1.27, halfY = 0.84;
    const double scale = 400.0;  // pixels per metre
    cv::Mat img(1000, 1200, CV_8UC1, cv::Scalar(255));
    auto px = [&](double x, double y) {
        return cv::Point(static_cast<int>(600 + x * scale), static_cast<int>(500 - y * scale));
    };

    putMarker(img, 10, px(-halfX, halfY), 0);
    putMarker(img, 11, px(halfX, halfY), 0);
    putMarker(img, 12, px(halfX, -halfY), 0);
    putMarker(img, 13, px(-halfX, -halfY), 0);

    // Robot index r carries tag r + 1.
    struct Case { int robot; double x, y; int rot; double expectedDeg; };
    const Case cases[] = {
        {0, 0.50, 0.30, 0, 90}, {1, -0.60, 0.10, 90, 0},
        {2, 0.00, -0.40, 180, -90}, {3, -0.30, -0.30, 270, 180}
    };
    for (const auto& c : cases)
        putMarker(img, CPFAOverheadCamera::kFirstRobotTag + c.robot, px(c.x, c.y), c.rot);

    // Two foam-cube food items, both tag 0.
    putMarker(img, 0, px(0.80, -0.40), 0);
    putMarker(img, 0, px(-0.90, 0.50), 0);

    cv::Mat bgr;
    cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);

    CPFAOverheadCamera camera(halfX, halfY);
    camera.processFrame(bgr);
    assert(camera.isCalibrated());

    for (const auto& c : cases) {
        auto p = camera.getPose(c.robot);
        double deg = p.heading * 180.0 / 3.141592653589793;
        double err = std::fabs(std::remainder(deg - c.expectedDeg, 360.0));
        std::printf("R%d expected (%+.2f,%+.2f,%+4.0f) got (%+.3f,%+.3f,%+6.1f)\n",
                    c.robot, c.x, c.y, c.expectedDeg, p.x, p.y, deg);
        assert(p.seen);
        assert(std::fabs(p.x - c.x) < 0.01 && std::fabs(p.y - c.y) < 0.01);
        assert(err < 3.0);
    }

    const auto& food = camera.foodSeen();
    std::printf("food seen: %zu\n", food.size());
    assert(food.size() == 2);
    for (const auto& f : food) {
        std::printf("  food (%+.3f,%+.3f)\n", f.x, f.y);
        const bool a = std::fabs(f.x - 0.80) < 0.01 && std::fabs(f.y + 0.40) < 0.01;
        const bool b = std::fabs(f.x + 0.90) < 0.01 && std::fabs(f.y - 0.50) < 0.01;
        assert(a || b);
    }

    std::cout << "OVERHEAD CAMERA TEST PASS\n";
    return 0;
}
