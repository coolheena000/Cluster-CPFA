#ifndef CPFA_OVERHEAD_CAMERA_H
#define CPFA_OVERHEAD_CAMERA_H

// Overhead camera localization (replaces Carlos's ARGoS positioning sensor).
//
// AprilTag tag36h11 markers (the lab's tags):
//   ID 1..4   on top of robot 0..3 (robot i = tag i+1),
//             tag TOP edge pointing to the robot FRONT
//   ID 10..13 flat on the floor, tag centre exactly on the arena corner:
//             10 = top-left     (-halfX, +halfY)
//             11 = top-right    (+halfX, +halfY)
//             12 = bottom-right (+halfX, -halfY)
//             13 = bottom-left  (-halfX, -halfY)
//             so 10-11 and 13-12 are the long sides (x)
//             ("top" = top of the camera image)
//   any other ID (e.g. the foam cubes, ID 0) = food item
//
// Pose: metres, x right, y up, heading radians counter-clockwise from +x.

#include <opencv2/core.hpp>
#include <opencv2/geometry/2d.hpp>  // getPerspectiveTransform (OpenCV 5)
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/videoio.hpp>

#include <array>
#include <cmath>
#include <map>
#include <vector>

class CPFAOverheadCamera {
public:
    struct Pose {
        double x = 0.0;
        double y = 0.0;
        double heading = 0.0;
        bool seen = false;
    };

    struct Point {
        double x = 0.0;
        double y = 0.0;
        int id = -1;  // tag ID (food tags only)
        float px = 0.0f, py = 0.0f;  // tag centre in the camera image, pixels
    };

    static constexpr int kRobotCount = 4;
    static constexpr int kFirstRobotTag = 1;
    static constexpr int kTagDictionary = cv::aruco::DICT_APRILTAG_36h11;

    CPFAOverheadCamera(double halfX, double halfY)
        : detector_(cv::aruco::getPredefinedDictionary(kTagDictionary)) {
        cornerIds_ = {10, 11, 12, 13};
        cornerArena_ = {
            cv::Point2f(static_cast<float>(-halfX), static_cast<float>(halfY)),   // 10
            cv::Point2f(static_cast<float>(halfX), static_cast<float>(halfY)),    // 11
            cv::Point2f(static_cast<float>(halfX), static_cast<float>(-halfY)),   // 12
            cv::Point2f(static_cast<float>(-halfX), static_cast<float>(-halfY))   // 13
        };
    }

    bool open(int cameraIndex) {
        return capture_.open(cameraIndex);
    }

    // Grab one frame from the camera and update all poses.
    bool update() {
        if (!capture_.read(frame_)) return false;
        processFrame(frame_);
        return true;
    }

    // Detect tags in an image (also used by the offline test).
    void processFrame(const cv::Mat& image) {
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        detector_.detectMarkers(image, corners, ids);

        std::map<int, std::vector<cv::Point2f>> found;
        std::vector<std::pair<cv::Point2f, int>> foodPx;  // several cubes may share one ID
        for (std::size_t i = 0; i < ids.size(); ++i) {
            const bool robot = ids[i] >= kFirstRobotTag && ids[i] < kFirstRobotTag + kRobotCount;
            const bool corner = ids[i] >= cornerIds_.front() && ids[i] <= cornerIds_.back();
            if (robot || corner) found[ids[i]] = corners[i];
            else foodPx.push_back({centre(corners[i]), ids[i]});
        }

        // Calibrate (or re-calibrate) when all four corner tags are visible.
        bool allCorners = true;
        for (int id : cornerIds_)
            if (!found.count(id)) allCorners = false;

        if (allCorners) {
            std::vector<cv::Point2f> src;
            for (int id : cornerIds_) src.push_back(centre(found[id]));
            homography_ = cv::getPerspectiveTransform(src, cornerArena_);
            calibrated_ = true;
        }

        for (int r = 0; r < kRobotCount; ++r) {
            const int tag = kFirstRobotTag + r;
            poses_[r].seen = false;
            if (!calibrated_ || !found.count(tag)) continue;

            const auto& c = found[tag];  // top-left, top-right, bottom-right, bottom-left
            const cv::Point2f mid = centre(c);
            const cv::Point2f front = (c[0] + c[1]) * 0.5f;  // middle of top edge

            const cv::Point2f p = toArena(mid);
            const cv::Point2f f = toArena(front);

            poses_[r].x = p.x;
            poses_[r].y = p.y;
            poses_[r].heading = std::atan2(f.y - p.y, f.x - p.x);
            poses_[r].seen = true;
        }

        food_.clear();
        if (calibrated_) {
            for (const auto& px : foodPx) {
                const cv::Point2f p = toArena(px.first);
                food_.push_back({p.x, p.y, px.second, px.first.x, px.first.y});
            }
        }

        if (!ids.empty() && !image.empty()) {
            image.copyTo(debugFrame_);
            cv::aruco::drawDetectedMarkers(debugFrame_, corners, ids);
        }
    }

    bool isCalibrated() const { return calibrated_; }
    Pose getPose(int robotIndex) const { return poses_.at(robotIndex); }
    const std::vector<Point>& foodSeen() const { return food_; }
    const cv::Mat& debugFrame() const { return debugFrame_.empty() ? frame_ : debugFrame_; }

private:
    static cv::Point2f centre(const std::vector<cv::Point2f>& c) {
        return (c[0] + c[1] + c[2] + c[3]) * 0.25f;
    }

    cv::Point2f toArena(const cv::Point2f& px) const {
        std::vector<cv::Point2f> in{px}, out;
        cv::perspectiveTransform(in, out, homography_);
        return out[0];
    }

    cv::aruco::ArucoDetector detector_;
    cv::VideoCapture capture_;
    cv::Mat frame_;
    cv::Mat debugFrame_;
    cv::Mat homography_;
    bool calibrated_ = false;
    std::array<int, 4> cornerIds_{};
    std::vector<cv::Point2f> cornerArena_;
    std::array<Pose, kRobotCount> poses_{};
    std::vector<Point> food_;
};

#endif
