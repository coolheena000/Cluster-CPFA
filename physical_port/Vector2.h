#ifndef VECTOR2_H
#define VECTOR2_H

#include <cmath>

// 2D point in metres (takes the place of argos::CVector2).
struct Vector2 {
	double x = 0.0;
	double y = 0.0;

	double SquareLength() const { return x * x + y * y; }
	double Length() const { return std::sqrt(SquareLength()); }
};

inline Vector2 operator+(Vector2 a, Vector2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vector2 operator-(Vector2 a, Vector2 b) { return {a.x - b.x, a.y - b.y}; }

#endif
