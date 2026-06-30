#ifndef DRONELAB_SIMULATION_NAVIGATION_TRAJECTORY_GENERATOR_HPP
#define DRONELAB_SIMULATION_NAVIGATION_TRAJECTORY_GENERATOR_HPP

#include <atomic>
#include <cstdint>
#include <mutex>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/navigation/trajectory.hpp"
#include "dronelab/api/simulation/sensors/estimate.hpp"

namespace dronelab::msg {
struct DroneState;
struct FlightStatus;
}  // namespace dronelab::msg

namespace dronelab::sim {

class TrajectoryGenerator : public Node {
   public:
    explicit TrajectoryGenerator(ContextView context);

   private:
    void receivedStateEstimate(const msg::StateEstimate& state);
    void receivedDroneState(const msg::DroneState& state);
    void receivedTrajectoryTarget(const msg::TrajectoryTarget& trajWayPoint);
    void receivedFlightStatus(const msg::FlightStatus& status);
    void calculateTrajectory();
    void canCalculateSetPoint();
    void calculateSetPoint();
    void calculateSetPointPolynomial();
    void calculateSetPointPlaceHolder();
    void evaluateTrajectory(double t, Vec3d& pos, Vec3d& vel, Vec3d& acc, double& yaw) const;

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<msg::StateEstimate> subscriberStateEstimate_;
    Subscriber<msg::DroneState> subscriberDroneState_;
    Subscriber<msg::TrajectoryTarget> subscriberTrajTarget_;
    Subscriber<msg::FlightStatus> subscriberFlightStatus_;
    Publisher<msg::Setpoint> publisherSetPoint_;

    msg::FlightStatus latestFlightStatus_;
    msg::StateEstimate lastEstimate_;
    msg::TrajectoryTarget lastTrajTarget_;
    bool waypointReceived_{false};
    bool initialized_{false};
    msg::TrajectoryStrategy strategy_;
    msg::Trajectory trajectory_;
    msg::DroneParameters droneParameters_;
    std::int64_t timePoint_{};
    double t_{0.0};  // Duration
    bool useRealState_{};

    const double updateRateSeconds_;
    const std::int32_t updateRateUs_;
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};

    mutable std::mutex mutex_;
};
}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_NAVIGATION_TRAJECTORY_GENERATOR_HPP