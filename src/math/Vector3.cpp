#include "math/Vector3.h"

#include <cmath>
#include <iomanip>

Vector3::Vector3() : x(0.0), y(0.0), z(0.0) {}
Vector3::Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

double&       Vector3::operator[](int i)       { return (&x)[i]; }
const double& Vector3::operator[](int i) const { return (&x)[i]; }

Vector3 Vector3::operator+(const Vector3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
Vector3 Vector3::operator-(const Vector3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
Vector3 Vector3::operator*(double s)           const { return {x * s, y * s, z * s}; }
Vector3 Vector3::operator/(double s)           const { return {x / s, y / s, z / s}; }

Vector3& Vector3::operator+=(const Vector3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
Vector3& Vector3::operator-=(const Vector3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }

double Vector3::dot(const Vector3& rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }

Vector3 Vector3::cross(const Vector3& rhs) const {
    return { y * rhs.z - z * rhs.y,
             z * rhs.x - x * rhs.z,
             x * rhs.y - y * rhs.x };
}

double Vector3::norm() const { return std::sqrt(x * x + y * y + z * z); }

Vector3 Vector3::normalized() const {
    const double n = norm();
    return { x / n, y / n, z / n };
}

Vector3 operator*(double scalar, const Vector3& v) { return v * scalar; }

std::ostream& operator<<(std::ostream& os, const Vector3& v) {
    os << std::fixed << std::setprecision(4)
       << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}
