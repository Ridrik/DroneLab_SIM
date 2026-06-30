#ifndef DRONELAB_SIMULATION_SENSORS_GPS_SENSOR_HPP
#define DRONELAB_SIMULATION_SENSORS_GPS_SENSOR_HPP

#include <mutex>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/simulation/utils/noise_generator.hpp"

namespace dronelab::msg {
struct GpsData;
struct SimulationCommand;
}  // namespace dronelab::msg

namespace dronelab::sim {

class GpsSensor : public Node {
   public:
    explicit GpsSensor(ContextView context);

   private:
    void receivedDroneState(const msg::DroneState& state);
    void processSimCmd(const msg::SensorUpdateCmd& cmd);
    void updateGpsData();
    void activate();
    void deactivate();

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<sim::msg::DroneState> subscriberDroneState_;
    Subscriber<sim::msg::SimulationCommand> subscriberSimCmd_;
    Publisher<sim::msg::GpsData> publisherGpsData_;

    msg::DroneState lastState_;
    msg::GpsNoiseParams gpsNoiseParams_;
    ::dronelab::utils::EnhNoiseGenerator posNoiseGenerator_;
    ::dronelab::utils::NoiseGenerator velNoiseGenerator_;
    bool initialized_{false};
    bool activated_;

    const std::int32_t updateRateUs_;
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};

    static constexpr msg::SensorID kSensorId{msg::SensorID::kGPS};

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_SENSORS_GPS_SENSOR_HPP