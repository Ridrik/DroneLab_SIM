#ifndef DRONELAB_SERVICES_PID_CONTROLLER_HPP
#define DRONELAB_SERVICES_PID_CONTROLLER_HPP

#include <mutex>

#include <Eigen/Geometry>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/commands.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/control/control_commands.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/sensors/estimate.hpp"

namespace dronelab::sim {

class PIDController : public Node {
   public:
    explicit PIDController(ContextView context);

   private:
    void receivedStateEstimate(const msg::StateEstimate& stateEstimate);
    void receivedDroneState(const msg::DroneState& state);
    void receivedSetPoint(const msg::Setpoint& setPoint);
    static void processSimCmd(const msg::PIDUpdateCmd& cmd);
    void updatePos();
    void updateVel();
    void updateAtt();
    void updateCommands();

    Subscriber<msg::Stamp> subscriberSimTimeAtt_;
    Subscriber<msg::Stamp> subscriberSimTimePos_;
    Subscriber<sim::msg::StateEstimate> subscriberStateEstimate_;
    Subscriber<sim::msg::DroneState> subscriberDroneState_;
    Subscriber<sim::msg::Setpoint> subscriberSetpoint_;
    Subscriber<sim::msg::SimulationCommand> subscriberSimCmd_;
    Publisher<sim::msg::ControlCommands> publisherControlCommand_;
    Publisher<sim::msg::PIDLog> publisherPidLog_;

    msg::StateEstimate currentState_;
    msg::Setpoint currentSetpoint_;
    msg::ControlCommands lastControlCommand_;
    msg::PIDCascadeGains controlGains_;
    const msg::DroneParameters droneParams_;
    Eigen::Matrix3d inertia_;

    Vec3d totalDesiredVel_;
    Vec3d totalDesiredAcc_;
    Vec3d desiredAngVelocity_;
    double thrustMagnitude_{};
    Eigen::Quaterniond qDesiredWb_{Eigen::Quaterniond::Identity()};

    bool stateInitialized_{false};
    bool setpointInitialized_{false};
    const bool useRealState_{};

    const std::int32_t posRateUs_{};
    const double posRateSeconds_{};
    std::atomic<bool> hasPos_{false};
    const std::int32_t attRateUs_{};
    const double attRateSeconds_{};
    std::atomic<msg::Stamp> posLastStepUs_{-msg::secondsToStamp(1.0)};
    std::atomic<msg::Stamp> attLastStepUs_{-msg::secondsToStamp(1.0)};

    std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SERVICES_PID_CONTROLLER_HPP