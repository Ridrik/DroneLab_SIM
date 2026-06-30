#ifndef DRONELAB_SIMULATION_DATA_SENSORS_IMU_HPP
#define DRONELAB_SIMULATION_DATA_SENSORS_IMU_HPP

#include "dronelab/api/simulation/sensors/imu.hpp"

namespace dronelab::sim::msg {

constexpr void addImuRawData(msg::VirtualImuData& virtualData, const msg::ImuData& rawData,
                             double dt) {
    virtualData.deltaTheta += rawData.gyro * dt;
    virtualData.deltaV += rawData.accel * dt;
    virtualData.totalDt += dt;
}

constexpr void addVirtualData(VirtualImuData& dst, const VirtualImuData& src) {
    dst.deltaTheta += src.deltaTheta;
    dst.deltaV += src.deltaV;
}

void averageVirtualImu(msg::VirtualImuData& virtualData, std::size_t count) {
    virtualData.deltaTheta /= static_cast<double>(count);
    virtualData.deltaV /= static_cast<double>(count);
}

void resetVirtualImu(msg::VirtualImuData& virtualData) {
    virtualData.deltaV = {};
    virtualData.deltaTheta = {};
    virtualData.totalDt = 0;
}

}  // namespace dronelab::sim::msg

#endif  // DRONELAB_SIMULATION_DATA_SENSORS_IMU_HPP