#ifndef CPFA_COLLISION_MANEUVER_H
#define CPFA_COLLISION_MANEUVER_H

// Carlos BaseController::CollisionDetection():
//   on a front collision (angle within +-37.5 deg): Stop, clear the movement
//   stack, turn LEFT (37.5 - angle) or RIGHT (37.5 + angle) degrees, then move
//   FORWARD SearchStepSize. Collision detection runs every tick, so a new
//   collision restarts the maneuver. ARGoS times each movement with
//   TicksToWaitWhileMoving; here the same angle/distance is converted to time
//   from the e-puck2 wheel geometry.

#include "CPFA_MotorAdapter.h"
#include "CPFA_PhysicalCollision.h"

class CPFACollisionManeuver {
public:
    // e-puck2: 1000 steps = one wheel turn = 0.1288 m; wheel axle 0.053 m.
    static constexpr double kMetresPerStep = 0.1288 / 1000.0;
    static constexpr double kAxle = 0.053;
    static constexpr double kPi = 3.141592653589793;

    CPFACollisionManeuver(int16_t forwardSpeed, int16_t turnSpeed, double searchStepSize = 0.08)
        : forwardSpeed_(forwardSpeed), turnSpeed_(turnSpeed), stepSize_(searchStepSize) {}

    // Call once per control tick. Returns true while the maneuver owns the
    // wheels; `movement` and `speed` are then the command to send.
    bool update(const CPFAPhysicalCollision::Decision& collision, double dt,
                CPFAMovement& movement, int16_t& speed) {
        if (collision.obstacleDetected) {
            const double angle = collision.collisionAngle;  // degrees
            turn_ = angle <= 0.0 ? CPFAMovement::LEFT : CPFAMovement::RIGHT;
            const double degrees = angle <= 0.0 ? 37.5 - angle : 37.5 + angle;
            turnLeft_ = turnSeconds(degrees);
            forwardLeft_ = forwardSeconds(stepSize_);
            ++count_;
        }

        if (turnLeft_ > 0.0) {
            turnLeft_ -= dt;
            movement = turn_;
            speed = turnSpeed_;
            return true;
        }
        if (forwardLeft_ > 0.0) {
            forwardLeft_ -= dt;
            movement = CPFAMovement::FORWARD;
            speed = forwardSpeed_;
            return true;
        }
        return false;
    }

    bool active() const { return turnLeft_ > 0.0 || forwardLeft_ > 0.0; }
    int count() const { return count_; }

    double turnSeconds(double degrees) const {
        const double wheel = turnSpeed_ * kMetresPerStep;          // m/s
        const double omega = 2.0 * wheel / kAxle;                  // rad/s
        return (degrees * kPi / 180.0) / omega;
    }

    double forwardSeconds(double metres) const {
        return metres / (forwardSpeed_ * kMetresPerStep);
    }

private:
    int16_t forwardSpeed_;
    int16_t turnSpeed_;
    double stepSize_;
    CPFAMovement turn_ = CPFAMovement::STOP;
    double turnLeft_ = 0.0;
    double forwardLeft_ = 0.0;
    int count_ = 0;
};

#endif
