#ifndef DRONELAB_SIMULATION_CONTROL_LQR_CONTROLLER_HPP
#define DRONELAB_SIMULATION_CONTROL_LQR_CONTROLLER_HPP

#include <mutex>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/commands.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/control/control_commands.hpp"
#include "dronelab/api/simulation/control/lqr.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/sensors/estimate.hpp"
#include "dronelab/simulation/data/control/lqr.hpp"

namespace dronelab::sim {

class LQRController : public Node {
   public:
    explicit LQRController(ContextView context);

   private:
    void receivedStateEstimate(const msg::StateEstimate& stateEstimate);
    void receivedDroneState(const msg::DroneState& state);
    void receivedSetPoint(const msg::Setpoint& setPoint);
    static void processSimCmd(const msg::PIDUpdateCmd& cmd);
    void update();
    void updateCommands();

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<sim::msg::StateEstimate> subscriberStateEstimate_;
    Subscriber<sim::msg::DroneState> subscriberDroneState_;
    Subscriber<sim::msg::Setpoint> subscriberSetpoint_;
    Subscriber<sim::msg::SimulationCommand> subscriberSimCmd_;
    Publisher<sim::msg::ControlCommands> publisherControlCommand_;

    msg::StateEstimate currentState_;
    msg::Setpoint currentSetpoint_;
    msg::ControlCommands lastControlCommand_;
    static constexpr std::size_t kStateDim{12};
    static constexpr std::size_t kControlDim{4};
    msg::LQRSolver<kStateDim, kControlDim> controlGains_;
    Eigen::Vector<double, kControlDim> uHover_;
    const msg::DroneParameters droneParams_;

    bool stateInitialized_{false};
    bool setpointInitialized_{false};
    const bool useRealState_{};

    const std::int32_t intervalRateUs_{};
    const double intervalRateSeconds_{};
    std::atomic<bool> hasPos_{false};
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_CONTROL_LQR_CONTROLLER_HPP