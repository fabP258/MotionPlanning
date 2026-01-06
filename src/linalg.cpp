#include "linalg.h"
#include <cmath>

namespace Common {
namespace Linalg {

float determinant(const Matrix2x2f &m) {
    return m[0][0] * m[1][1] - m[0][1] * m[1][0];
}

float determinant(const Matrix3x3f &m) {
    float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]);
    det -= m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]);
    det += m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    return det;
}

std::optional<Matrix2x2f> inverse(const Matrix2x2f &m) {
    float det = determinant(m);
    if (std::abs(det) < 1e-6) {
        return std::nullopt;
    }

    float invDet = 1.0f / det;

    Matrix2x2f result;
    result[0][0] = m[1][1] * invDet;
    result[0][1] = -m[0][1] * invDet;
    result[1][0] = -m[1][0] * invDet;
    result[1][1] = m[0][0] * invDet;

    return result;
}

std::optional<Matrix3x3f> inverse(const Matrix3x3f &m) {
    float det = determinant(m);
    if (std::abs(det) < 1e-6) {
        return std::nullopt;
    }

    float invDet = 1.0f / det;
    Matrix3x3f result;

    // Row 0
    result[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    result[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    result[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;

    // Row 1
    result[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
    result[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    result[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invDet;

    // Row 2
    result[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    result[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invDet;
    result[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invDet;

    return result;
}

Vector2f multiply(const Matrix2x2f &mat, const Vector2f &vec) {
    Vector2f result;
    result[0] = mat[0][0] * vec[0] + mat[0][1] * vec[1];
    result[1] = mat[1][0] * vec[0] + mat[1][1] * vec[1];
    return result;
}

Vector3f multiply(const Matrix3x3f &mat, const Vector3f &vec) {
    Vector3f result = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i] += mat[i][j] * vec[j];
        }
    }

    return result;
}

} // namespace Linalg
} // namespace Common