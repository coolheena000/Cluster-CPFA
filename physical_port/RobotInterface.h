#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "Vector2.h"
#include <cstddef>

// What the CPFA needs from a robot. The real e-puck2 and the offline test
// each provide their own version.
class RobotInterface {
	public:
		virtual ~RobotInterface() {}

		virtual Vector2 GetPosition() = 0;  // metres
		virtual double  GetHeading() = 0;   // radians, (-pi, pi]

		// Replaces the food list in Carlos's SetHoldingFood() and SetLocalResourceDensity().
		virtual bool   FoodDetected() = 0;
		virtual size_t LocalResourceDensity() = 0;  // includes the food just found

		virtual void Stop() = 0;
		virtual void SetRightTurn(double degrees) = 0;
};

class RandomInterface {
	public:
		virtual ~RandomInterface() {}
		virtual double Uniform(double min, double max) = 0;
		virtual double Gaussian(double stdDev) = 0;
};

#endif
