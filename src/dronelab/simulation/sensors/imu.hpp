#ifndef DRONELAB_SIMULATION_SENSORS_IMU_HPP
#define DRONELAB_SIMULATION_SENSORS_IMU_HPP

#include <mutex>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/core/utils/vec.hpp"
#include "dronelab/api/simulation/common/commands.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/sensors/imu.hpp"
#include "dronelab/simulation/utils/noise_generator.hpp"

namespace dronelab::sim {

class IMUSensor : public Node {
   public:
    explicit IMUSensor(ContextView context, std::size_t id);

   private:
    void receivedDroneState(const msg::DroneState& state);
    void publishImuData();

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<msg::DroneState> subscriberDroneState_;
    Subscriber<msg::SimulationCommand> subscriberSimCmd_;
    Publisher<msg::ImuData> publisherImuData_;

    bool initialized_{false};
    msg::DroneState currentState_;
    msg::TimePoint lastTrueStateTime_;

    msg::ImuData imuData_;
    msg::IMUNoiseParams noiseParams_;
    Vec3d currentGyroBias_;
    Vec3d currentAccBias_;
    Eigen::Quaterniond imuToBody_{Eigen::Quaterniond::Identity()};

    const double updateRateSeconds_;
    const std::int32_t updateRateUs_;
    const double updateRateSecondsSqrt_;

    ::dronelab::utils::NoiseGenerator gyroNoiseGenerator_;
    ::dronelab::utils::NoiseGenerator accNoiseGenerator_;
    ::dronelab::utils::NoiseGenerator gyroBiasNoiseGenerator_;
    ::dronelab::utils::NoiseGenerator accelBiasNoiseGenerator_;

    msg::Stamp internalStepUs_{-msg::secondsToStamp(1.0)};
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};

    static constexpr msg::SensorID kSensorId{msg::SensorID::kIMU};

    std::size_t id_;
    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_SENSORS_IMU_HPP