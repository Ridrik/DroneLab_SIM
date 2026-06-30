#include "dronelab/simulation/sensors/imu.hpp"

#include <cmath>

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/data/constants.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/simulation/utils/eigen_adapter.hpp"

namespace dronelab::sim {

IMUSensor::IMUSensor(ContextView context, std::size_t id)
    : Node(context, NodeType::kIMU),
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberDroneState_{context, *this, TopicKey::kDroneState},
      subscriberSimCmd_{context, *this, TopicKey::kSimCmd},
      publisherImuData_{context, *this, TopicKey::kImuData},
      noiseParams_{context.getConfig().getImuParams()},
      updateRateSeconds_{1.0 / static_cast<double>(context.getConfig().getImuRate())},
      updateRateUs_{static_cast<std::int32_t>(1000000.0 * updateRateSeconds_)},
      updateRateSecondsSqrt_{std::sqrt(updateRateSeconds_)},
      gyroNoiseGenerator_{
          1.0, context.getConfig().getSeed() + (static_cast<std::size_t>(kSensorId) + id) * 10},
      accNoiseGenerator_{1.0, context.getConfig().getSeed() +
                                  (static_cast<std::size_t>(kSensorId) + id) * 10 + 1U},
      gyroBiasNoiseGenerator_{1.0, context.getConfig().getSeed() +
                                       (static_cast<std::size_t>(kSensorId) + id) * 10 + 2U},
      accelBiasNoiseGenerator_{1.0, context.getConfig().getSeed() +
                                        (static_cast<std::size_t>(kSensorId) + id) * 10 + 3U},
      id_{id} {
    subscriberDroneState_.subscribe([this](const auto& msg) { receivedDroneState(*msg); });

    subscriberSimTime_
        .filter([this](const auto& msg) {
            return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
        })
        .subscribe([this](const auto& msg) {
            internalStepUs_ = *msg;
            lastStepUs_.store(*msg, std::memory_order_relaxed);
            publishImuData();
        });
    imuData_.id = id_;
}

void IMUSensor::receivedDroneState(const msg::DroneState& state) {
    SPDLOG_TRACE("[{}] On Received Drone State, on lock acquisition.", name_);
    std::lock_guard lock{mutex_};
    SPDLOG_TRACE("[{}] On Received Drone State, lock acquired.", name_);

    currentState_ = state;
    if (!initialized_) {
        SPDLOG_DEBUG("[{}] Drone state registered, initialized for IMU data processing.", name_);
        initialized_ = true;
    }
}

void IMUSensor::publishImuData() {
    std::lock_guard lock{mutex_};
    SPDLOG_TRACE("[{}] On IMU Data processing", name_);

    if (!initialized_) {
        SPDLOG_TRACE("[{}] Exiting IMU Data processing, not yet initialized with any drone state.",
                     name_);
        return;
    }

    // Convert orientation to rotation matrix (world -> body)
    const Eigen::Matrix3d R =
        utils::toEigenQuat(currentState_.orientation).toRotationMatrix().transpose();
    static const auto gravity = Eigen::Vector3d{0, 0, constants::kGravity};

    auto dt = msg::microsToSeconds<double>(internalStepUs_ - imuData_.simStamp);
    if (dt <= 0 || dt > 1.0) [[unlikely]] {
        SPDLOG_DEBUG("[{}] Abnormal dt = {}, setting to 1e-3", name_, dt);
        dt = 1e-3;
    }
    const auto dtSqrt = std::sqrt(dt);

    const Vec3d gyroNoise =
        gyroNoiseGenerator_.sampleVec() * noiseParams_.gyroMeasurementNoiseStddev / dtSqrt;
    const Vec3d accelNoise =
        accNoiseGenerator_.sampleVec() * noiseParams_.accelMeasurementNoiseStddev / dtSqrt;

    imuData_.gyro = currentState_.angularVelocity + currentGyroBias_ + gyroNoise;
    imuData_.accel =
        Vec{utils::fromEigen(R * (utils::toEigen(currentState_.acceleration) - gravity))} +
        currentAccBias_ + accelNoise;
    imuData_.bG = currentGyroBias_;
    imuData_.bA = currentAccBias_;

    // Random-walk bias update
    currentGyroBias_ +=
        gyroBiasNoiseGenerator_.sampleVec() * noiseParams_.gyroBiasRandomWalkStddev * dtSqrt;
    currentAccBias_ +=
        accelBiasNoiseGenerator_.sampleVec() * noiseParams_.accelBiasRandomWalkStddev * dtSqrt;

    imuData_.simStamp = internalStepUs_;
    publisherImuData_.publish(imuData_);
}

}  // namespace dronelab::sim