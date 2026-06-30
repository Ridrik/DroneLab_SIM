#ifndef DRONELAB_SIMULATION_SENSORS_BAROMETER_HPP
#define DRONELAB_SIMULATION_SENSORS_BAROMETER_HPP

#include <atomic>
#include <cstdint>
#include <mutex>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/commands.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/sensors/barometer.hpp"
#include "dronelab/simulation/utils/noise_generator.hpp"

namespace dronelab::sim {

class Barometer : public Node {
   public:
    explicit Barometer(ContextView context);

   private:
    void receivedDroneState(const msg::DroneState& state);
    void processSimCmd(const msg::SensorUpdateCmd& cmd);
    void updateData();
    void activate();
    void deactivate();

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<sim::msg::DroneState> subscriberDroneState_;
    Subscriber<sim::msg::SimulationCommand> subscriberSimCmd_;
    Publisher<sim::msg::BarometerData> publisherBarometerData_;

    msg::DroneState lastState_;
    msg::BarometerData barData_;
    msg::BarometerConfig barParams_;
    ::dronelab::utils::NoiseGenerator noiseGenerator_;
    ::dronelab::utils::NoiseGenerator biasNoiseGenerator_;
    double currentBias_{0.0};
    double filteredMeasurement_{0.0};
    bool initialized_{false};
    bool initFiltering_{true};
    bool activated_;
    const std::int32_t updateRateUs_{};
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};
    msg::Stamp internalCurStepUs_{-msg::secondsToStamp(1.0)};

    static constexpr msg::SensorID kSensorId{msg::SensorID::kBarometer};

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_SENSORS_BAROMETER_HPP