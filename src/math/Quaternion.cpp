#include "math/Quaternion.h"

#include <cmath>
#include <iomanip>

Quaternion::Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
Quaternion::Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::fromEuler(double phi, double theta, double psi) {
    const double cphi = std::cos(phi * 0.5),   sphi = std::sin(phi * 0.5);
    const double cth  = std::cos(theta * 0.5), sth  = std::sin(theta * 0.5);
    const double cpsi = std::cos(psi * 0.5),   spsi = std::sin(psi * 0.5);

    return Quaternion(
        cphi * cth * cpsi + sphi * sth * spsi,
        sphi * cth * cpsi - cphi * sth * spsi,
        cphi * sth * cpsi + sphi * cth * spsi,
        cphi * cth * spsi - sphi * sth * cpsi
    );
}

Quaternion Quaternion::operator*(const Quaternion& r) const {
    return Quaternion(
        w * r.w - x * r.x - y * r.y - z * r.z,
        w * r.x + x * r.w + y * r.z - z * r.y,
        w * r.y - x * r.z + y * r.w + z * r.x,
        w * r.z + x * r.y - y * r.x + z * r.w
    );
}

Quaternion Quaternion::conjugate() const { return Quaternion(w, -x, -y, -z); }

double Quaternion::norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

Quaternion& Quaternion::normalize() {
    const double n = norm();
    w /= n; x /= n; y /= n; z /= n;
    return *this;
}

Vector3 Quaternion::rotate(const Vector3& v) const {
    const Quaternion p(0.0, v.x, v.y, v.z);
    const Quaternion r = conjugate() * p * (*this);
    return { r.x, r.y, r.z };
}

Vector3 Quaternion::rotateBack(const Vector3& v) const {
    const Quaternion p(0.0, v.x, v.y, v.z);
    const Quaternion r = (*this) * p * conjugate();
    return { r.x, r.y, r.z };
}

Matrix3x3 Quaternion::toDcm() const {
    return Matrix3x3(
        1 - 2 * (y * y + z * z),  2 * (x * y + w * z),      2 * (x * z - w * y),
        2 * (x * y - w * z),      1 - 2 * (x * x + z * z),  2 * (y * z + w * x),
        2 * (x * z + w * y),      2 * (y * z - w * x),      1 - 2 * (x * x + y * y)
    );
}

Vector3 Quaternion::toEuler() const {
    const Matrix3x3 C = toDcm();

    // C(0,2) = -sin(theta)
    const double theta = std::asin(-C(0, 2));
    const double cth   = std::cos(theta);

    double phi, psi;
    if (std::abs(cth) > 1e-9) {
        phi = std::atan2(C(1, 2), C(2, 2));
        psi = std::atan2(C(0, 1), C(0, 0));
    } else {
        // Gimbal lock: theta = +/-90 deg. Convention: psi = 0, solve phi.
        psi = 0.0;
        phi = std::atan2(C(1, 0), C(1, 1));
    }
    return { phi, theta, psi };
}

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    os << std::fixed << std::setprecision(4)
       << "[w=" << q.w << ", x=" << q.x << ", y=" << q.y << ", z=" << q.z << "]";
    return os;
}
