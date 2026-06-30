#ifndef DRONELAB_SIMULATION_UTILS_REPRESENTATIONS_HPP
#define DRONELAB_SIMULATION_UTILS_REPRESENTATIONS_HPP

#include <cmath>
#include <concepts>
#include <span>

namespace dronelab::sim::utils {

// Yaw from quaternion (NED)
template <std::floating_point T, bool xyzwOrder = true>
[[nodiscard]] double getYawFromQuaternion(std::span<T> quat) {
    if constexpr (xyzwOrder) {
        const double qW = quat[3];
        const double qX = quat[0];
        const double qY = quat[1];
        const double qZ = quat[2];
        const double yaw = std::atan2(2.0 * (qW * qZ + qX * qY), 1.0 - 2.0 * (qY * qY + qZ * qZ));
        return yaw;
    } else {
        const double qW = quat[0];
        const double qX = quat[1];
        const double qY = quat[2];
        const double qZ = quat[3];
        const double yaw = std::atan2(2.0 * (qW * qZ + qX * qY), 1.0 - 2.0 * (qY * qY + qZ * qZ));
        return yaw;
    }
}

}  // namespace dronelab::sim::utils

#endif  // DRONELAB_SIMULATION_UTILS_REPRESENTATIONS_HPP