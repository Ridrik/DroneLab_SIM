#ifndef DRONELAB_SIMULATION_UTILS_EIGEN_ADAPTER_HPP
#define DRONELAB_SIMULATION_UTILS_EIGEN_ADAPTER_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dronelab/api/core/utils/vec.hpp"

namespace dronelab::sim::utils {

template <std::size_t N, typename T>
[[nodiscard]] auto toEigen(const Vec<T, N>& vec) {
    return Eigen::Map<const Eigen::Vector<T, N>>(vec.data());
}

[[nodiscard]] inline auto fromEigen(const Eigen::Quaterniond& q) noexcept {
    return Vec4d{q.x(), q.y(), q.z(), q.w()};
}

[[nodiscard]] inline auto toEigenQuat(const Vec4d& vec) {
    return Eigen::Quaterniond{vec[3], vec[0], vec[1], vec[2]};
}

template <typename Derived>
    requires(static_cast<bool>(Derived::IsVectorAtCompileTime) &&
             Derived::SizeAtCompileTime != Eigen::Dynamic)
[[nodiscard]] auto fromEigen(const Eigen::MatrixBase<Derived>& vec) noexcept {
    using T = typename Derived::Scalar;
    constexpr auto kN = static_cast<std::size_t>(Derived::SizeAtCompileTime);
    Vec<T, kN> out{};
    for (std::size_t i{0}; i < kN; ++i) {
        out[i] = vec(i);
    }
    return out;
}

template <typename Derived>
    requires((Derived::RowsAtCompileTime == 1 || Derived::ColsAtCompileTime == 1) &&
             Derived::SizeAtCompileTime != Eigen::Dynamic)
[[nodiscard]] auto fromEigen(const Eigen::ArrayBase<Derived>& arr) noexcept {
    return fromEigen(arr.matrix());
}

template <std::size_t N, typename Derived>
[[nodiscard]] auto fromEigen(const Eigen::MatrixBase<Derived>& vec) noexcept {
    using T = typename Derived::Scalar;
    Vec<T, N> out{};
    for (std::size_t i{0}; i < N; ++i) {
        out[i] = vec(i);
    }
    return out;
}

}  // namespace dronelab::sim::utils

#endif  // DRONELAB_SIMULATION_UTILS_EIGEN_ADAPTER_HPP