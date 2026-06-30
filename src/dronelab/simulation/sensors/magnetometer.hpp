#ifndef DRONELAB_SIMULATION_SENSORS_MAGNETOMETER_HPP
#define DRONELAB_SIMULATION_SENSORS_MAGNETOMETER_HPP

#include <mutex>

#include <Eigen/Dense>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/commands.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/sensors/magnetometer.hpp"
#include "dronelab/simulation/utils/noise_generator.hpp"

namespace dronelab::sim {

class Magnetometer : public Node {
   public:
    explicit Magnetometer(ContextView context);

   private:
    void receivedDroneState(const msg::DroneState& state);
    void processSimCmd(const msg::SensorUpdateCmd& cmd);
    void updateData();
    void activate();
    void deactivate();

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<sim::msg::DroneState> subscriberDroneState_;
    Subscriber<sim::msg::SimulationCommand> subscriberSimCmd_;
    Publisher<sim::msg::MagnetometerData> publisherMagnetometerData_;

    msg::DroneState lastState_;
    msg::MagnetometerParams magParams_;
    ::dronelab::utils::Enh3NoiseGenerator noiseGenerator_;
    Eigen::Vector3d earthMagFieldEigen_;
    bool initialized_{false};
    bool activated_;
    const std::int32_t updateRateUs_{};
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};

    static constexpr msg::SensorID kSensorId{msg::SensorID::kMagnetometer};

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_SENSORS_MAGNETOMETER_HPP