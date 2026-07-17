#ifndef DRONELAB_SIMULATION_NAVIGATION_RTL_BEHAVIOUR_HPP
#define DRONELAB_SIMULATION_NAVIGATION_RTL_BEHAVIOUR_HPP

#include <atomic>
#include <cstdint>
#include <mutex>

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/management/flight.hpp"
#include "dronelab/api/simulation/navigation/rtl.hpp"
#include "dronelab/api/simulation/navigation/trajectory.hpp"
#include "dronelab/api/simulation/sensors/estimate.hpp"

namespace dronelab::sim {

class RtlBehaviour : public Node {
   public:
    explicit RtlBehaviour(ContextView context);

   private:
    void receivedFlightStatus(const msg::FlightStatus& status);
    void receivedStateEstimate(const msg::StateEstimate& state);
    void receivedDroneState(const msg::DroneState& state);
    [[nodiscard]] double computeConvergenceTime(const Eigen::Vector3d& relWpPos, double yawCur,
                                                double nextYaw,
                                                double nominalSpeed,  // m/s
                                                double minTime,       // s
                                                double maxTime        // s
    ) const;
    [[nodiscard]] static bool isWithinTargetTolerance(double posErrNorm, double yawErr,
                                                      const msg::RtlBehaviourConfig& config);
    [[nodiscard]] bool isOutsideTargetTolerance(double posErrNorm, double yawErr) const;
    void sendHomeTarget(const Vec3d& homePosition);
    void update();

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<sim::msg::FlightStatus> subscriberFlightStatus_;
    Subscriber<sim::msg::StateEstimate> subscriberStateEstimate_;
    Subscriber<sim::msg::DroneState> subscriberDroneState_;
    Publisher<sim::msg::TrajectoryTarget> publisherTrajTarget_;
    Publisher<sim::msg::MissionEvent> publisherMissionEvent_;

    struct RtlTrack {
        void reset() {
            targetSet = false;
            reached = false;
            timeout = false;
            insideTolerace = false;
            elapsedTime = 0.0;
            holdTimer = 0.0;
            maxTimeBeforeTimeout = 0.0;
        }
        msg::TrajectoryTarget trajTarget;
        bool targetSet{false};
        bool reached{false};
        bool timeout{false};
        bool insideTolerace{false};
        double radiusOut{0.0};
        double yawOut{0.0};
        double elapsedTime{0.0};
        double holdTimer{0.0};
        double maxTimeBeforeTimeout{0.0};
    };

    RtlTrack homeTrack_;
    msg::FlightStatus latestFlightStatus_;
    msg::StateEstimate lastState_;
    msg::RtlBehaviourConfig rtlBehaviourConfig_;

    const double updateRateSeconds_{};
    const std::int32_t updateRateUs_{};

    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};
    msg::Stamp internalLastStepUs_{-msg::secondsToStamp(1.0)};

    bool useRealState_{false};

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_NAVIGATION_RTL_BEHAVIOUR_HPP