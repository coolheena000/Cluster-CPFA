// Camera check with a top-down map (no robots needed, no Wi-Fi):
//   window 1 "Camera": live image, detected tags, which corner tags are missing
//   window 2 "Arena map": the arena seen from above (camera image warped with the
//            corner tags), 25 cm grid, nest circle, robot tags 1-4 with heading,
//            food tags as dots
// If the map looks like a flat, square arena with the nest in the middle and the
// robots where they really are, the camera covers the arena and is calibrated.
//
// Build:
//   export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
//   clang++ -std=c++17 test_camera_map.cpp $(pkg-config --cflags --libs opencv5) -o test_camera_map
// Run:   ./test_camera_map                  (camera 0, 59 x 59 in arena)
//        ./test_camera_map 0 1.06 0.685     (camera, half sizes in metres)
// Keys:  Q quit, S save both windows as camera_view.png and arena_map.png

#include "CPFA_OverheadCamera.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

static cv::Point2f centre(const std::vector<cv::Point2f>& c) {
    return (c[0] + c[1] + c[2] + c[3]) * 0.25f;
}

int main(int argc, char** argv) {
    const int cameraIndex = argc > 1 ? std::atoi(argv[1]) : 0;
    const double hx = argc > 3 ? std::atof(argv[2]) : 0.749;
    const double hy = argc > 3 ? std::atof(argv[3]) : 0.749;
    const double nestRadius = 0.25;
    const double margin = 0.10;   // map border outside the corner tags, metres
    const double ppm = 400.0;     // map pixels per metre

    cv::VideoCapture cap(cameraIndex);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "Camera %d not available\n", cameraIndex);
        return 1;
    }
    cv::aruco::ArucoDetector detector(
        cv::aruco::getPredefinedDictionary(CPFAOverheadCamera::kTagDictionary));

    // Same corner layout as CPFA_OverheadCamera.h: 10 top-left, 11 top-right,
    // 12 bottom-right, 13 bottom-left; x to the right, y up, metres.
    const std::map<int, cv::Point2f> cornerArena = {
        {10, {static_cast<float>(-hx), static_cast<float>(hy)}},
        {11, {static_cast<float>(hx), static_cast<float>(hy)}},
        {12, {static_cast<float>(hx), static_cast<float>(-hy)}},
        {13, {static_cast<float>(-hx), static_cast<float>(-hy)}}};

    // Arena metres -> map pixels.
    const cv::Size mapSize(static_cast<int>(2 * (hx + margin) * ppm),
                           static_cast<int>(2 * (hy + margin) * ppm));
    auto toMap = [&](cv::Point2f p) {
        return cv::Point(static_cast<int>((p.x + hx + margin) * ppm),
                         static_cast<int>((hy + margin - p.y) * ppm));
    };
    const cv::Matx33d arenaToMap(ppm, 0, (hx + margin) * ppm,
                                 0, -ppm, (hy + margin) * ppm,
                                 0, 0, 1);

    std::printf("Camera %d, arena %.2f x %.2f m between the corner tags. Q quit, S save.\n",
                cameraIndex, 2 * hx, 2 * hy);

    cv::Mat frame, view, map;
    cv::Mat homography;  // camera pixels -> arena metres, kept from the last full calibration
    double lastPrint = -1.0;
    const auto start = cv::getTickCount();

    while (cap.read(frame)) {
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        detector.detectMarkers(frame, corners, ids);

        std::map<int, std::vector<cv::Point2f>> tags;
        for (std::size_t i = 0; i < ids.size(); ++i) tags[ids[i]] = corners[i];

        std::string missing;
        for (const auto& c : cornerArena)
            if (!tags.count(c.first)) missing += " " + std::to_string(c.first);
        if (missing.empty()) {
            std::vector<cv::Point2f> src, dst;
            for (const auto& c : cornerArena) {
                src.push_back(centre(tags[c.first]));
                dst.push_back(c.second);
            }
            homography = cv::getPerspectiveTransform(src, dst);
        }

        // ---- window 1: camera ----
        view = frame.clone();
        if (!ids.empty()) cv::aruco::drawDetectedMarkers(view, corners, ids);
        const std::string status = missing.empty()
            ? "ALL 4 CORNER TAGS SEEN"
            : "MISSING CORNER TAG:" + missing;
        cv::putText(view, status, {20, 50}, cv::FONT_HERSHEY_SIMPLEX, 1.4,
                    missing.empty() ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 255), 3);
        cv::Mat small;
        cv::resize(view, small, cv::Size(), 0.6, 0.6);
        cv::imshow("Camera", small);

        // ---- window 2: top-down map ----
        if (!homography.empty()) {
            cv::warpPerspective(frame, map, cv::Mat(arenaToMap) * homography, mapSize);
            for (double g = -std::floor(hx / 0.25) * 0.25; g <= hx + 1e-6; g += 0.25)
                cv::line(map, toMap({float(g), float(-hy)}), toMap({float(g), float(hy)}),
                         cv::Scalar(200, 200, 200), 1);
            for (double g = -std::floor(hy / 0.25) * 0.25; g <= hy + 1e-6; g += 0.25)
                cv::line(map, toMap({float(-hx), float(g)}), toMap({float(hx), float(g)}),
                         cv::Scalar(200, 200, 200), 1);
            cv::rectangle(map, toMap({float(-hx), float(hy)}), toMap({float(hx), float(-hy)}),
                          cv::Scalar(255, 120, 0), 2);
            cv::circle(map, toMap({0, 0}), static_cast<int>(nestRadius * ppm),
                       cv::Scalar(0, 160, 0), 2);
            cv::putText(map, "NEST", toMap({-0.07f, 0.0f}), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 160, 0), 2);

            std::string robotsText;
            for (const auto& t : tags) {
                const int id = t.first;
                std::vector<cv::Point2f> a;
                cv::perspectiveTransform(t.second, a, homography);
                const cv::Point2f mid = (a[0] + a[1] + a[2] + a[3]) * 0.25f;
                const bool robot = id >= CPFAOverheadCamera::kFirstRobotTag &&
                                   id < CPFAOverheadCamera::kFirstRobotTag + CPFAOverheadCamera::kRobotCount;
                const bool corner = cornerArena.count(id) > 0;
                if (robot) {
                    const cv::Point2f front = (a[0] + a[1]) * 0.5f;  // top edge of the tag
                    const double heading = std::atan2(front.y - mid.y, front.x - mid.x);
                    const cv::Point2f tip(mid.x + 0.12f * static_cast<float>(std::cos(heading)),
                                          mid.y + 0.12f * static_cast<float>(std::sin(heading)));
                    cv::circle(map, toMap(mid), static_cast<int>(0.037 * ppm), cv::Scalar(0, 0, 255), 3);
                    cv::arrowedLine(map, toMap(mid), toMap(tip), cv::Scalar(0, 0, 255), 3);
                    cv::putText(map, "R" + std::to_string(id), toMap(mid) + cv::Point(18, -18),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                    char buf[96];
                    std::snprintf(buf, sizeof buf, "  tag %d at (%.2f, %.2f) m, %.0f deg%s", id,
                                  mid.x, mid.y, heading * 180.0 / 3.141592653589793,
                                  std::hypot(mid.x, mid.y) < nestRadius ? " IN NEST" : "");
                    robotsText += buf;
                } else if (!corner) {
                    cv::circle(map, toMap(mid), 8, cv::Scalar(0, 140, 255), -1);
                }
            }
            if (!missing.empty())
                cv::putText(map, "OLD CALIBRATION - corner tag missing", {10, 30},
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            cv::imshow("Arena map", map);

            const double now = (cv::getTickCount() - start) / cv::getTickFrequency();
            if (now - lastPrint > 1.0) {
                lastPrint = now;
                std::printf("%s |%s\n", status.c_str(), robotsText.empty() ? "  no robot tag" : robotsText.c_str());
            }
        } else {
            const double now = (cv::getTickCount() - start) / cv::getTickFrequency();
            if (now - lastPrint > 1.0) {
                lastPrint = now;
                std::printf("%s (no map until all four corner tags are seen once)\n", status.c_str());
            }
        }

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 'Q') break;
        if (key == 's' || key == 'S') {
            cv::imwrite("camera_view.png", view);
            if (!map.empty()) cv::imwrite("arena_map.png", map);
            std::printf("Saved camera_view.png%s\n", map.empty() ? "" : " and arena_map.png");
        }
    }
    return 0;
}
