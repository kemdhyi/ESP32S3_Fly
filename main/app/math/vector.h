// 轻量级矢量库
// Lightweight vector library
//
// ESP-IDF 原生版本：移除 Printable 依赖，算法与 Arduino 版逐行一致。

#pragma once

#include <math.h>
#include <stddef.h>

#include "app/math/util.h"

class Vector {
public:
	float x, y, z;

	Vector() : x(0), y(0), z(0) {}
	Vector(float x, float y, float z) : x(x), y(y), z(z) {}

	bool zero() const { return x == 0 && y == 0 && z == 0; }
	bool finite() const { return isfinite(x) && isfinite(y) && isfinite(z); }
	bool valid() const { return finite(); }
	bool invalid() const { return !valid(); }

	void invalidate() {
		x = NAN;
		y = NAN;
		z = NAN;
	}

	float norm() const { return sqrtf(x * x + y * y + z * z); }

	void normalize() {
		float n = norm();
		x /= n;
		y /= n;
		z /= n;
	}

	Vector operator + (const float b) const { return Vector(x + b, y + b, z + b); }
	Vector operator * (const float b) const { return Vector(x * b, y * b, z * b); }
	Vector operator / (const float b) const { return Vector(x / b, y / b, z / b); }

	Vector operator + (const Vector& b) const { return Vector(x + b.x, y + b.y, z + b.z); }
	Vector operator - (const Vector& b) const { return Vector(x - b.x, y - b.y, z - b.z); }

	Vector& operator += (const Vector& b) { return *this = *this + b; }
	Vector& operator -= (const Vector& b) { return *this = *this - b; }

	// 逐元素乘除
	Vector operator * (const Vector& b) const { return Vector(x * b.x, y * b.y, z * b.z); }
	Vector operator / (const Vector& b) const { return Vector(x / b.x, y / b.y, z / b.z); }

	bool operator == (const Vector& b) const { return x == b.x && y == b.y && z == b.z; }
	bool operator != (const Vector& b) const { return !(*this == b); }

	static float dot(const Vector& a, const Vector& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	static Vector cross(const Vector& a, const Vector& b) {
		return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}

	static float angleBetween(const Vector& a, const Vector& b) {
		return acosf(constrain(dot(a, b) / (a.norm() * b.norm()), -1.0f, 1.0f));
	}

	static Vector rotationVectorBetween(const Vector& a, const Vector& b) {
		float an = a.norm();
		float bn = b.norm();
		if (an < 1e-6f || bn < 1e-6f) {
			return Vector(0, 0, 0);
		}
		Vector direction = cross(a, b);
		if (direction.norm() < 1e-6f) { // vectors are parallel
			if (dot(a, b) > 0) { // same direction
				return Vector(0, 0, 0);
			}
			// opposite direction
			Vector perp = cross(a, Vector(1, 0, 0));
			if (perp.norm() < 1e-6f) {
				perp = cross(a, Vector(0, 1, 0));
			}
			perp.normalize();
			return perp * (float)M_PI;
		}
		direction.normalize();
		float angle = angleBetween(a, b);
		return direction * angle;
	}
};

static inline Vector operator * (const float a, const Vector& b) { return b * a; }
static inline Vector operator + (const float a, const Vector& b) { return b + a; }
