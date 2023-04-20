#pragma once

#include <cmath>

struct Vector3 {
    float x, y, z;

    inline Vector3 operator+(const Vector3& rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    inline Vector3 operator-(const Vector3& rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    inline Vector3 operator*(const float& rhs) const {
        return {x * rhs, y * rhs, z * rhs};
    }

    inline Vector3 operator/(const float& rhs) const {
        return {x / rhs, y / rhs, z / rhs};
    }

    inline Vector3& operator+=(const Vector3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    inline Vector3& operator-=(const Vector3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    inline Vector3& operator*=(const float& rhs) {
        x *= rhs;
        y *= rhs;
        z *= rhs;
        return *this;
    }

    inline Vector3& operator/=(const float& rhs) {
        x /= rhs;
        y /= rhs;
        z /= rhs;
        return *this;
    }

    inline float dot(const Vector3& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    inline Vector3 cross(const Vector3& rhs) const {
        return {y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x};
    }

    inline float length() const {
        return sqrt(x * x + y * y + z * z);
    }

    inline Vector3 normalize() const {
        return *this / length();
    }

    inline bool operator==(const Vector3& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
};

struct Quaternion {
    float w, x, y, z;

    inline float length() const {
        return sqrt(w * w + x * x + y * y + z * z);
    }

    inline Quaternion operator*(const Quaternion& rhs) const {
        return {w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
                w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
                w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
                w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w};
    }

    inline Quaternion operator*(const float& rhs) const {
        return {w * rhs, x * rhs, y * rhs, z * rhs};
    }

    inline Quaternion operator/(const float& rhs) const {
        return {w / rhs, x / rhs, y / rhs, z / rhs};
    }

    inline Quaternion norm() const {
        return *this / length();
    }


};


