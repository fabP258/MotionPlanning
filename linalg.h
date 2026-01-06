#ifndef LINALG_H_INCLUDED
#define LINALG_H_INCLUDED

#include <array>
#include <optional>

namespace Common {
namespace Linalg {

using Matrix2x2f = std::array<std::array<float, 2>, 2>;
using Matrix3x3f = std::array<std::array<float, 3>, 3>;
using Matrix4x4f = std::array<std::array<float, 4>, 4>;

using Vector2f = std::array<float, 2>;
using Vector3f = std::array<float, 3>;

float determinant(const Matrix2x2f &mat);
float determinant(const Matrix3x3f &mat);

std::optional<Matrix2x2f> inverse(const Matrix2x2f &mat);
std::optional<Matrix3x3f> inverse(const Matrix3x3f &mat);

Vector2f multiply(const Matrix2x2f &mat, const Vector2f &vec);
Vector3f multiply(const Matrix3x3f &mat, const Vector3f &vec);

} // namespace Linalg
} // namespace Common

#endif // LINALG_H_INCLUDED