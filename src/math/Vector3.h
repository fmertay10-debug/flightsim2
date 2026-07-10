#pragma once

#include <ostream>

struct Vector3 {
    double x, y, z;

    Vector3();                              // (0, 0, 0)
    Vector3(double x, double y, double z);

    double&       operator[](int i);
    const double& operator[](int i) const;

    Vector3 operator+(const Vector3& rhs) const;
    Vector3 operator-(const Vector3& rhs) const;
    Vector3 operator*(double scalar)      const;
    Vector3 operator/(double scalar)      const;

    Vector3& operator+=(const Vector3& rhs);
    Vector3& operator-=(const Vector3& rhs);

    double  dot(const Vector3& rhs)   const;
    Vector3 cross(const Vector3& rhs) const;
    double  norm()                    const;
    Vector3 normalized()              const;

    friend Vector3 operator*(double scalar, const Vector3& v);
    friend std::ostream& operator<<(std::ostream& os, const Vector3& v);
};
