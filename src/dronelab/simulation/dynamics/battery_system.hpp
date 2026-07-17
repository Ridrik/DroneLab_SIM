#ifndef DRONELAB_SIMULATION_DYNAMICS_BATTERY_SYSTEM_HPP
#define DRONELAB_SIMULATION_DYNAMICS_BATTERY_SYSTEM_HPP

#include <atomic>
#include <mutex>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/dynamics/actuator_data.hpp"
#include "dronelab/api/simulation/dynamics/battery.hpp"

namespace dronelab::sim {

class BatterySystem : public Node {
   public:
    explicit BatterySystem(ContextView context);

   private:
    void receivedActuatorFeedback(const msg::ActuatorFeedback& feedback);
    void stepUpdate();
    void receivedSimReport() override;

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<msg::ActuatorFeedback> subscriberActuatorFeedback_;
    Publisher<msg::BatteryData> publisherBatteryData_;

    msg::BatteryConfig batteryConfig_;
    msg::BatteryData batteryData_;
    double invCapacitymAh_;
    double remainingmAh_;
    double totalEnergyConsumed_{0.0};
    double minVoltage_{0.0};
    double maxVoltage_{0.0};
    msg::ActuatorFeedback latestActuatorFeedback_;

    const double updateRateSeconds_{};
    const std::int32_t updateRateUs_{};
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_DYNAMICS_BATTERY_SYSTEM_HPP