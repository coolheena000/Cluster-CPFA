
#ifndef CPFA_EPUCK_SENSOR_GEOMETRY_H
#define CPFA_EPUCK_SENSOR_GEOMETRY_H

#include <array>

// IR0 to IR7, radians.
// Based on e-puck2 firmware IR communication direction table.
// Physical proximity calibration is still pending.
constexpr std::array<double, 8> kEpuckSensorAngles = {
    -0.296706, -0.8726646, -1.570796, -2.617994,
     2.617994,  1.570796,   0.8726646,  0.296706
};

#endif
