#include "dronelab/simulation/dynamics/actuator.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <random>

#include <Eigen/Core>
#include <Eigen/src/Core/GlobalFunctions.h>
#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/core/utils/common.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/common/vehicle.hpp"
#include "dronelab/api/simulation/control/actuator_commands.hpp"
#include "dronelab/api/simulation/dynamics/actuator_data.hpp"
#include "dronelab/api/simulation/dynamics/battery.hpp"
#include "dronelab/api/simulation/dynamics/drone_params.hpp"
#include "dronelab/simulation/utils/eigen_adapter.hpp"

namespace dronelab::sim {

Actuator::Actuator(ContextView context)
    : Node{context, NodeType::kActuator},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberActuatorCommands_{context, *this, TopicKey::kActuatorCommands},
      subscriberBatteryData_{context, *this, TopicKey::kBatteryStatus},
      publisherActuatorFeedback_{context, *this, TopicKey::kActuatorFeedback},
      droneParameters_{context.getConfig().getDroneParameters()},
      referenceMaxThrustPerMotor_{droneParameters_.calcMaxMotorThrust(
          context.getConfig().getBatteryConfig().nominalVoltage)},
      invReferenceMaxThrustPerMotor_{1.0 / referenceMaxThrustPerMotor_},
      referenceMaxMotorSpeed_{droneParameters_.calcMaxMotorSpeed(
          context.getConfig().getBatteryConfig().nominalVoltage)},
      invThrustCoefficient_{1.0 / droneParameters_.thrustCoefficient},
      invMotorResistance_{1.0 / droneParameters_.actuatorConfig.motorResistance},
      invMotorInertia_{1.0 / droneParameters_.actuatorConfig.motorInertia},
      motorNumber_{msg::getNumMotors(droneParameters_.vehicletype)},
      updateRateSeconds_{1.0 / static_cast<double>(context.getConfig().getActuatorRate())},
      updateRateUs_{static_cast<std::int32_t>(updateRateSeconds_ * 1e6)} {
    SPDLOG_DEBUG("[{}] Max motor speed & thrust: {} [rad/s], {} [N]", name_,
                 referenceMaxMotorSpeed_, referenceMaxThrustPerMotor_);
    SPDLOG_DEBUG("[{}] Actuator model: {}", name_,
                 droneParameters_.actuatorConfig.escMode ==
                         msg::DroneParameters::ActuatorConfig::EscMode::kBaseOpenLoop
                     ? "Basic Open loop ESC"
                     : "Advanced closed loop ESC");
    subscriberActuatorCommands_.subscribe(
        [this](const auto& msg) { receivedActuatorCommand(*msg); });

    subscriberBatteryData_.subscribe([this](const auto& msg) { receivedBatteryData(*msg); });

    using EscMode = msg::DroneParameters::ActuatorConfig::EscMode;
    if (droneParameters_.actuatorConfig.escMode == EscMode::kAdvancedClosedLoop) {
        subscriberSimTime_
            .filter([this](const auto& msg) -> bool {
                return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
            })
            .subscribe([this](const auto& msg) {
                lastStepUs_.store(*msg, std::memory_order_relaxed);
                updateActuator();
            });
    }

    std::ranges::for_each(currentFeedback_.state.motorTemperatures, [this](auto& temp) {
        temp = droneParameters_.actuatorConfig.ambientTemperatureC;
    });

    const auto& kp = droneParameters_.actuatorConfig.escGain.kp;
    const auto& ki = droneParameters_.actuatorConfig.escGain.ki;
    const auto& actuatorConfig = droneParameters_.actuatorConfig;
    constexpr double kKiLowestExpectedValue{1e-8};
    if (ki.isZero(kKiLowestExpectedValue) && !kp.isZero()) {
        desiredSpeedPreGain_ =
            (kp + Vecd<msg::kActuatorMaxNum>{droneParameters_.actuatorConfig.motorKe})
                .cwiseQuotient(kp);
    }
    voltageCompFactor_ = actuatorConfig.motorResistance * droneParameters_.torqueCoefficient /
                         actuatorConfig.motorKt;
}

void Actuator::receivedActuatorCommand(const msg::ActuatorCommands& cmd) {
    std::lock_guard lock{mutex_};
    FAULT_ASSERT(cmd.motorNum == motorNumber_,
                 "Actuator received commands motor num {} not expected size {}!", cmd.motorNum,
                 motorNumber_);
    SPDLOG_TRACE("[{}] Processing actuator command with simstamp {}", name_, cmd.simStamp);
    latestActuatorCommands_ = cmd;

    initialized_ = true;
}

void Actuator::receivedBatteryData(const msg::BatteryData& data) {
    std::lock_guard lock{mutex_};
    latestBatteryData_ = data;
}

void Actuator::updateActuator() {
    const msg::ActuatorCommands& currentCommands = latestActuatorCommands_;
    Vecd<msg::kActuatorMaxNum> targetSpeeds;
    switch (currentCommands.mode) {
        case msg::ActuatorMode::kThrottle: {
            Eigen::Vector<double, msg::kActuatorMaxNum> motorThrustCmd =
                utils::toEigen(currentCommands.motorThrottleCmd.cwiseProduct(
                    currentCommands.motorThrottleCmd)) *
                referenceMaxThrustPerMotor_;
            targetSpeeds =
                utils::fromEigen((motorThrustCmd * invThrustCoefficient_).array().sqrt());
            break;
        }
        case msg::ActuatorMode::kMotorSpeed: {
            targetSpeeds = currentCommands.motorSpeeds;
            break;
        }
        case msg::ActuatorMode::kMotorThrust: {
            targetSpeeds = utils::fromEigen(
                utils::toEigen(currentCommands.motorThrusts * invThrustCoefficient_)
                    .array()
                    .sqrt());
            break;
        }
    }

    // Motor dynamics
    const auto& currentFeedback = currentFeedback_;
    auto& actuatorConfig = droneParameters_.actuatorConfig;
    auto& escGain = actuatorConfig.escGain;
    const auto preTargetSpeed = desiredSpeedPreGain_.cwiseProduct(targetSpeeds);
    const auto speedError = preTargetSpeed - currentFeedback.state.motorSpeeds;
    escGain.updateIntegralError(speedError, updateRateSeconds_);
    latestVoltageCommandsEscAdvanced_ =
        escGain.calculateGain(speedError, updateRateSeconds_) +
        currentFeedback.state.motorSpeeds.cwiseProduct(currentFeedback.state.motorSpeeds) *
            voltageCompFactor_;
}

msg::MotorUpdateInfo Actuator::calcSpeedDerivative(
    const msg::ActuatorState& currentFeedback) const {
    using EscMode = msg::DroneParameters::ActuatorConfig::EscMode;
    msg::ActuatorCommands cmds;
    double batteryVoltage{};
    {
        std::lock_guard lock{mutex_};
        cmds = latestActuatorCommands_;
        batteryVoltage = latestBatteryData_.voltage;
    }
    switch (droneParameters_.actuatorConfig.escMode) {
        case EscMode::kBaseOpenLoop:
            return calcSpeedDerivativeActuatorBase(cmds, batteryVoltage, currentFeedback);
        case EscMode::kAdvancedClosedLoop:
            return calcSpeedDerivativeAdvancedClosedLoop(batteryVoltage, currentFeedback);
    }
    std::unreachable();
}

void Actuator::calcAndSendMotorFeedback(const msg::ActuatorState& state, msg::Stamp curTimeUs) {
    currentFeedback_.state = state;
    currentFeedback_.physics = calcActuatorPhysics(state);
    currentFeedback_.simStamp = curTimeUs;
    currentFeedback_.motorNum = motorNumber_;
    publisherActuatorFeedback_.publish(currentFeedback_);
}

msg::MotorUpdateInfo Actuator::calcSpeedDerivativeActuatorBase(
    const msg::ActuatorCommands& cmd, double batteryVoltage,
    const msg::ActuatorState& currentFeedback) const {
    const msg::ActuatorCommands& currentCommands = cmd;
    const auto& actuatorConfig = droneParameters_.actuatorConfig;
    Vecd<msg::kActuatorMaxNum> speedDerivative{};
    for (std::size_t i{0}; i < motorNumber_; ++i) {
        // --- Apply simple ESC: throttle [0..1] -> voltage ---
        const double throttle = currentCommands.motorThrottleCmd[i];  // 0..1
        const double vApplied = throttle * batteryVoltage;

        // --- Electrical dynamics ---
        double omega = currentFeedback.motorSpeeds[i];
        const double backEmf = actuatorConfig.motorKe * omega;
        double motorCurrent = (vApplied - backEmf) * invMotorResistance_;
        motorCurrent = std::clamp(motorCurrent, actuatorConfig.minMotorCurrent,
                                  actuatorConfig.maxMotorCurrent);

        // --- Mechanical dynamics ---
        const double motorTorque = actuatorConfig.motorKt * motorCurrent;
        const double loadTorque = droneParameters_.torqueCoefficient * omega * std::abs(omega);
        const double motorAccel = (motorTorque - loadTorque) * invMotorInertia_;
        speedDerivative[i] = motorAccel;
    }

    return msg::MotorUpdateInfo{.accel = speedDerivative, .tempDerivative = {}};
}

msg::MotorUpdateInfo Actuator::calcSpeedDerivativeAdvancedClosedLoop(
    double batteryVoltage, const msg::ActuatorState& currentFeedback) const {
    const auto& vCmds = latestVoltageCommandsEscAdvanced_;
    const auto& actuatorConfig = droneParameters_.actuatorConfig;
    msg::MotorUpdateInfo motorUpdateInfo;
    for (std::size_t i{0}; i < motorNumber_; ++i) {
        const double& omega = currentFeedback.motorSpeeds[i];
        const double escVoltage = batteryVoltage;
        // calculateEscVoltage(batteryVoltage, currentFeedback.motorCurrents(i));
        const double vApplied = std::clamp(vCmds[i], 0.0, escVoltage);
        const double backEmf = actuatorConfig.motorKe * omega;
        double motorCurrent = (vApplied - backEmf) * invMotorResistance_;
        motorCurrent = std::clamp(motorCurrent, droneParameters_.actuatorConfig.minMotorCurrent,
                                  droneParameters_.actuatorConfig.maxMotorCurrent);

        const auto& curTemp = currentFeedback.motorTemperatures[i];
        const bool escShutdown = isEscShutdown(escVoltage, curTemp);

        if (escShutdown) {
            motorCurrent = 0.0;
        }

        motorUpdateInfo.tempDerivative[i] =
            getMotorTemperatureDerivative(curTemp, motorCurrent, omega);

        // --- Mechanical model ---
        const double loadTorque = droneParameters_.torqueCoefficient * omega * std::abs(omega);
        const double motorTorque = actuatorConfig.motorKt * motorCurrent;
        const double motorAccel = (motorTorque - loadTorque) * invMotorInertia_;
        motorUpdateInfo.accel[i] = motorAccel;
    }

    return motorUpdateInfo;
}

msg::ActuatorPhysics Actuator::calcActuatorPhysics(const msg::ActuatorState& state) const {
    msg::ActuatorCommands cmds;
    double batteryVoltage{};
    {
        std::lock_guard lock{mutex_};
        cmds = latestActuatorCommands_;
        batteryVoltage = latestBatteryData_.voltage;
    }

    using EscMode = msg::DroneParameters::ActuatorConfig::EscMode;
    switch (droneParameters_.actuatorConfig.escMode) {
        case EscMode::kBaseOpenLoop:
            return calcActuatorPhysicsBase(cmds, batteryVoltage, state);
        case EscMode::kAdvancedClosedLoop:
            return calcActuatorPhysicsAdvanced(batteryVoltage, state);
    }
    std::unreachable();
}

msg::ActuatorPhysics Actuator::calcActuatorPhysicsBase(const msg::ActuatorCommands& cmd,
                                                       double batteryVoltage,
                                                       const msg::ActuatorState& state) const {
    msg::ActuatorPhysics physics;
    for (std::size_t i{0}; i < motorNumber_; ++i) {
        // --- Apply simple ESC: throttle [0..1] -> voltage ---
        const double throttle = cmd.motorThrottleCmd[i];  // 0..1
        const double vApplied = throttle * batteryVoltage;
        physics.escVoltages[i] = batteryVoltage;
        physics.vCmdFactor[i] = vApplied / batteryVoltage;

        // --- Electrical dynamics ---
        double omega = state.motorSpeeds[i];
        const double backEmf = droneParameters_.actuatorConfig.motorKe * omega;
        physics.motorCurrents[i] = (vApplied - backEmf) * invMotorResistance_;
        physics.motorCurrents[i] =
            std::clamp(physics.motorCurrents[i], droneParameters_.actuatorConfig.minMotorCurrent,
                       droneParameters_.actuatorConfig.maxMotorCurrent);
    }
    return physics;
}

msg::ActuatorPhysics Actuator::calcActuatorPhysicsAdvanced(double batteryVoltage,
                                                           const msg::ActuatorState& state) const {
    const auto& actuatorConfig = droneParameters_.actuatorConfig;
    const auto& vCmds = latestVoltageCommandsEscAdvanced_;

    msg::ActuatorPhysics physics;
    for (std::size_t i{0}; i < motorNumber_; ++i) {
        const double& omega = state.motorSpeeds[i];
        const double escVoltage = batteryVoltage;
        const double vApplied = std::clamp(vCmds[i], 0.0, escVoltage);
        physics.escVoltages[i] = escVoltage;
        physics.vCmdFactor[i] = vApplied / escVoltage;
        const double backEmf = actuatorConfig.motorKe * omega;
        physics.motorCurrents[i] = (vApplied - backEmf) * invMotorResistance_;
        physics.motorCurrents[i] =
            std::clamp(physics.motorCurrents[i], droneParameters_.actuatorConfig.minMotorCurrent,
                       droneParameters_.actuatorConfig.maxMotorCurrent);

        const auto& curTemp = state.motorTemperatures[i];
        const bool escShutdown = isEscShutdown(escVoltage, curTemp);

        if (escShutdown) {
            physics.motorCurrents[i] = 0.0;
            physics.escShutdown[i] = true;
        }
    }

    return physics;
}

double Actuator::getMotorTemperatureDerivative(double currentTemp, double motorCurrent,
                                               double motorSpeed) const {
    // Calculate power dissipation (electrical + mechanical losses)
    const double electricalPower =
        motorCurrent * motorCurrent * droneParameters_.actuatorConfig.motorResistance;
    const double mechanicalPower = motorSpeed * droneParameters_.actuatorConfig.mechanicalLossCoeff;
    const double totalPowerDissipation = electricalPower + mechanicalPower;

    // Target temperature based on steady-state heat balance
    const double targetTemp =
        droneParameters_.actuatorConfig.ambientTemperatureC +
        totalPowerDissipation * droneParameters_.actuatorConfig.motorThermalResistance;

    // First-order thermal response: τ(dT/dt) = T_target - T_current
    const double tempDerivative =
        (targetTemp - currentTemp) / droneParameters_.actuatorConfig.motorThermalTimeConstant;

    return tempDerivative;
}

bool Actuator::isEscShutdown(double escVoltage, double motorTemp) const {
    return (escVoltage < droneParameters_.actuatorConfig.escLowVoltageCutoff) ||
           (motorTemp > droneParameters_.actuatorConfig.escMaxTemp);
}

double Actuator::addSensorNoise(double mean, double noisePercent) const {
    if (noisePercent <= 0.0 - ::dronelab::utils::kEpsilon) {
        return 1.0;
    }
    std::normal_distribution<double> noise(mean, noisePercent);
    return noise(rngDevice_);
}

}  // namespace dronelab::sim