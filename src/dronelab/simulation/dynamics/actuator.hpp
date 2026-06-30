#ifndef DRONELAB_SIMULATION_DYNAMICS_ACTUATOR_HPP
#define DRONELAB_SIMULATION_DYNAMICS_ACTUATOR_HPP

#include <mutex>
#include <random>

#include <Eigen/Dense>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/control/actuator_commands.hpp"
#include "dronelab/api/simulation/dynamics/actuator_data.hpp"
#include "dronelab/api/simulation/dynamics/battery.hpp"
#include "dronelab/api/simulation/dynamics/drone_params.hpp"
#include "dronelab/simulation/data/dynamics/actuator_data.hpp"

namespace dronelab::sim {

class Actuator : public Node {
   public:
    explicit Actuator(ContextView context);

    [[nodiscard]] msg::MotorUpdateInfo calcSpeedDerivative(
        const msg::ActuatorState& currentFeedback) const;

    void calcAndSendMotorFeedback(const msg::ActuatorState& state, msg::Stamp curTimeUs);

    [[nodiscard]] const msg::ActuatorState& getActuatorState() const {
        return currentFeedback_.state;
    }

   private:
    void receivedActuatorCommand(const msg::ActuatorCommands& cmd);
    void receivedBatteryData(const msg::BatteryData& data);
    void updateActuator();

    [[nodiscard]] msg::MotorUpdateInfo calcSpeedDerivativeActuatorBase(
        const msg::ActuatorCommands& cmd, double batteryVoltage,
        const msg::ActuatorState& currentFeedback) const;
    [[nodiscard]] msg::MotorUpdateInfo calcSpeedDerivativeAdvancedClosedLoop(
        double batteryVoltage, const msg::ActuatorState& currentFeedback) const;
    [[nodiscard]] msg::ActuatorPhysics calcActuatorPhysics(const msg::ActuatorState& state) const;
    [[nodiscard]] msg::ActuatorPhysics calcActuatorPhysicsBase(
        const msg::ActuatorCommands& cmd, double batteryVoltage,
        const msg::ActuatorState& state) const;
    [[nodiscard]] msg::ActuatorPhysics calcActuatorPhysicsAdvanced(
        double batteryVoltage, const msg::ActuatorState& state) const;
    [[nodiscard]] double getMotorTemperatureDerivative(double currentTemp, double motorCurrent,
                                                       double motorSpeed) const;
    [[nodiscard]] bool isEscShutdown(double escVoltage, double motorTemp) const;
    [[nodiscard]] double addSensorNoise(double mean, double noisePercent) const;

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<msg::ActuatorCommands> subscriberActuatorCommands_;
    Subscriber<msg::BatteryData> subscriberBatteryData_;
    Publisher<msg::ActuatorFeedback> publisherActuatorFeedback_;

    msg::DroneParameters droneParameters_;
    double referenceMaxThrustPerMotor_;
    double invReferenceMaxThrustPerMotor_;
    double referenceMaxMotorSpeed_;
    double invThrustCoefficient_;
    double invMotorResistance_;
    double invMotorInertia_;
    std::size_t motorNumber_{4};
    const msg::BatteryConfig batteryParameters_;
    msg::ActuatorCommands latestActuatorCommands_;
    msg::BatteryData latestBatteryData_;
    msg::ActuatorFeedback currentFeedback_;
    Vecd<msg::kActuatorMaxNum> desiredSpeedPreGain_;
    double voltageCompFactor_;
    Eigen::Vector<double, msg::kActuatorMaxNum> previousMotorCurrents_{
        Eigen::Vector<double, msg::kActuatorMaxNum>::Zero()};
    Vecd<msg::kActuatorMaxNum> latestVoltageCommandsEscAdvanced_;

    const double updateRateSeconds_;
    const std::int32_t updateRateUs_;
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};

    mutable std::mt19937 rngDevice_{std::random_device{}()};

    bool initialized_{false};

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_DYNAMICS_ACTUATOR_HPP