#ifndef DRONELAB_SIMULATION_NAVIGATION_LOITER_BEHAVIOUR_HPP
#define DRONELAB_SIMULATION_NAVIGATION_LOITER_BEHAVIOUR_HPP

#include <mutex>

#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/management/flight.hpp"
#include "dronelab/api/simulation/navigation/trajectory.hpp"
#include "dronelab/api/simulation/sensors/estimate.hpp"

namespace dronelab::sim {

class LoiterBehaviour : public Node {
   public:
    explicit LoiterBehaviour(ContextView context);

   private:
    void update();
    void makeNewLoiterTarget();
    void receivedFlightStatus(const msg::FlightStatus& status);
    void receivedDroneState(const msg::DroneState& state);
    void receivedStateEstimate(const msg::StateEstimate& state);

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<sim::msg::FlightStatus> subscriberFlightStatus_;
    Subscriber<sim::msg::StateEstimate> subscriberStateEstimate_;
    Subscriber<sim::msg::DroneState> subscriberDroneState_;
    Publisher<sim::msg::Setpoint> publisherSetpoint_;
    Publisher<sim::msg::MissionEvent> publisherMissionEvent_;

    msg::FlightStatus latestFlightStatus_;
    msg::StateEstimate lastState_;
    msg::Setpoint lastSetpoint_;
    bool createLoiterTarget_{true};

    const double updateRateSeconds_{};
    const std::int32_t updateRateUs_{};

    msg::Stamp internalLastStepUs_{-msg::secondsToStamp(1.0)};
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};

    bool useRealState_{false};

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_NAVIGATION_LOITER_BEHAVIOUR_HPP