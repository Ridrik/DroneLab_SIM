#ifndef DRONELAB_SIMULATION_NAVIGATION_WAYPOINT_BEHAVIOUR_HPP
#define DRONELAB_SIMULATION_NAVIGATION_WAYPOINT_BEHAVIOUR_HPP

#include <mutex>

#include <Eigen/Dense>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/management/mission.hpp"
#include "dronelab/api/simulation/navigation/trajectory.hpp"
#include "dronelab/api/simulation/navigation/waypoint.hpp"
#include "dronelab/api/simulation/sensors/estimate.hpp"

namespace dronelab::msg {
struct DroneState;
struct Waypoint;
}  // namespace dronelab::msg

namespace dronelab::sim {

class WaypointBehaviour : public Node {
   public:
    explicit WaypointBehaviour(ContextView context);

   private:
    void receivedMissionProgress(const msg::MissionProgress& mission);
    void receivedStateEstimate(const msg::StateEstimate& state);
    void receivedDroneState(const msg::DroneState& state);
    [[nodiscard]] double computeConvergenceTime(const Eigen::Vector3d& relWpPos, double yawCur,
                                                double nextYaw,
                                                double nominalSpeed,  // m/s
                                                double minTime,       // s
                                                double maxTime        // s
    ) const;
    [[nodiscard]] static bool isWithinTargetTolerance(double posErrNorm, double yawErr,
                                                      const msg::Waypoint& wp);
    [[nodiscard]] static bool isOutsideTargetTolerance(double posErrNorm, double yawErr,
                                                       const msg::WaypointProgress& wpProg);
    void update();
    void triggerNewTrajectoryTarget(const msg::WaypointProgress& referenceWaypoint);
    void receivedFlightStatus(const msg::FlightStatus& status);

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<sim::msg::MissionProgress> subscriberMissionProgress_;
    Subscriber<sim::msg::StateEstimate> subscriberStateEstimate_;
    Subscriber<sim::msg::DroneState> subscriberDroneState_;
    Subscriber<sim::msg::FlightStatus> subscriberFlightStatus_;
    Publisher<sim::msg::TrajectoryTarget> publisherTrajTarget_;
    Publisher<sim::msg::WaypointProgress> publisherWaypointProgress_;

    msg::StateEstimate lastState_;

    bool initialized_{false};
    bool useRealState_{false};

    struct WaypointTrack {
        void reset() {
            waypointProg = std::nullopt;
            reached = false;
            timeout = false;
            insideTolerace = false;
            elapsedTime = 0.0;
            holdTimer = 0.0;
            maxTimeBeforeTimeout = 0.0;
        }
        std::optional<std::reference_wrapper<sim::msg::WaypointProgress>> waypointProg{
            std::nullopt};
        msg::TrajectoryTarget trajTarget;
        bool reached{false};
        bool timeout{false};
        bool insideTolerace{false};
        double elapsedTime{0.0};
        double holdTimer{0.0};
        double maxTimeBeforeTimeout{0.0};
    };

    msg::MissionProgress missionProgress_;
    msg::FlightStatus latestFlightStatus_{};
    WaypointTrack currentTrackableWaypoint_;

    const double updateRateSeconds_{};
    const std::int32_t updateRateUs_{};
    msg::Stamp internalSimTimeUs_{-msg::secondsToStamp(1.0)};
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_NAVIGATION_WAYPOINT_BEHAVIOUR_HPP