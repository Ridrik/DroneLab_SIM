#include "dronelab/simulation/control/mixer.hpp"

#include <cmath>

#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/core/utils/common.hpp"
#include "dronelab/api/simulation/control/actuator_commands.hpp"
#include "dronelab/api/simulation/control/control_commands.hpp"
#include "dronelab/api/simulation/control/mixer.hpp"
#include "dronelab/api/simulation/dynamics/drone_params.hpp"

namespace dronelab::sim {

Mixer::Mixer(ContextView context)
    : Node{context, NodeType::kMixer},
      subscriberControlCommands_{context, *this, TopicKey::kControlCommands},
      publisherActuatorCommands_{context, *this, TopicKey::kActuatorCommands},
      droneParameters_{context.getConfig().getDroneParameters()},
      referenceMaxThrustPerMotor_{droneParameters_.calcMaxMotorThrust(
          context.getConfig().getBatteryConfig().nominalVoltage)},
      invReferenceMaxThrustPerMotor_{1.0 / referenceMaxThrustPerMotor_},
      invThrustCoefficient_{1.0 / droneParameters_.thrustCoefficient} {
    subscriberControlCommands_.subscribe(
        [this](const auto& msg) -> void { receivedControlCommands(*msg); });
}

void Mixer::receivedControlCommands(const msg::ControlCommands& command) {
    {
        std::lock_guard lock{mutex_};
        lastControlCommands_ = command;
    }
    update(command);
}

QuadMixer::QuadMixer(ContextView context) : Mixer{context} {
    // Drone physical constants
    const double l = droneParameters_.armLength;
    const double kF = droneParameters_.thrustCoefficient;
    const double kM = droneParameters_.torqueCoefficient;
    const double lM = std::numbers::sqrt2 * l * 0.5;
    // --- Mixer Matrix (NED + X configuration) ---
    const auto& quadXAlloc = droneParameters_.quadXAlloc;
    Eigen::Matrix4d mixer{Eigen::Matrix4d::Zero()};
    const double kMf = kM / kF;
    const std::unordered_map<sim::msg::QuadXMap, Eigen::Vector4d> motorToMixerMap = {
        {msg::QuadXMap::kFrontRight, {1.0, -lM, +lM, kMf}},
        {msg::QuadXMap::kRearLeft, {1.0, +lM, -lM, kMf}},
        {msg::QuadXMap::kFrontLeft, {1.0, +lM, +lM, kMf}},
        {msg::QuadXMap::kRearRight, {1.0, -lM, -lM, kMf}}};
    for (std::uint8_t i{0}; i < 4; ++i) {
        mixer.col(i) = motorToMixerMap.at(quadXAlloc.indexAlloc[i]);
        if (!quadXAlloc.isCCWorCW[i]) {
            mixer.col(i)[3] = -mixer.col(i)[3];
        }
    }

    mixerInv_ = mixer.inverse();
}

HexaMixer::HexaMixer(ContextView context) : Mixer{context} {
    // Drone physical constants
    const double l = droneParameters_.armLength;
    const double kF = droneParameters_.thrustCoefficient;
    const double kM = droneParameters_.torqueCoefficient;
    const double lCos30 = std::cos(utils::degToRad(30.0)) * l;
    const double lSin30 = std::sin(utils::degToRad(30.0)) * l;
    // --- Mixer Matrix (NED) ---
    const auto& hexAlloc = droneParameters_.hexAlloc;
    Eigen::Matrix<double, 4, 6> mixer{Eigen::Matrix<double, 4, 6>::Zero()};
    const double kMf = kM / kF;
    const std::unordered_map<sim::msg::HexMap, Eigen::Vector4d> motorToMixerMap = {
        {msg::HexMap::kFront30Right, {1.0, -lSin30, +lCos30, kMf}},
        {msg::HexMap::kFront30Left, {1.0, +lSin30, +lCos30, kMf}},
        {msg::HexMap::kLeft, {1.0, +l, 0.0, kMf}},
        {msg::HexMap::kRear30Left, {1.0, +lSin30, -lCos30, kMf}},
        {msg::HexMap::kRear30Right, {1.0, -lSin30, -lCos30, kMf}},
        {msg::HexMap::kRight, {1.0, -l, 0.0, kMf}}};
    for (std::uint8_t i{0}; i < 6; ++i) {
        mixer.col(i) = motorToMixerMap.at(hexAlloc.indexAlloc[i]);
        if (!hexAlloc.isCCWorCW[i]) {
            mixer.col(i)[3] = -mixer.col(i)[3];
        }
    }

    // const auto mixerT = mixer.transpose();
    // mixerPseudoInv_ = (mixerT * mixer).inverse() * mixerT;

    const auto mixerT = mixer.transpose();
    mixerPseudoInv_ = mixerT * (mixer * mixerT).inverse();
    /*Eigen::JacobiSVD<Eigen::Matrix<double, 4, 6>> svd(mixer,
                                                      Eigen::ComputeThinU | Eigen::ComputeThinV);

    mixerPseudoInv_ =
        svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();*/
}

OctoMixer::OctoMixer(ContextView context) : Mixer{context} {
    // Drone physical constants
    const double l = droneParameters_.armLength;
    const double kF = droneParameters_.thrustCoefficient;
    const double kM = droneParameters_.torqueCoefficient;
    const double lCos225 = std::cos(utils::degToRad(22.5)) * l;
    const double lSin225 = std::sin(utils::degToRad(22.5)) * l;
    // --- Mixer Matrix (NED) ---
    const auto& octoAlloc = droneParameters_.octoAlloc;
    Eigen::Matrix<double, 4, 8> mixer{Eigen::Matrix<double, 4, 8>::Zero()};
    const double kMf = kM / kF;
    const std::unordered_map<sim::msg::OctoMap, Eigen::Vector4d> motorToMixerMap = {
        {msg::OctoMap::kFront22Right, {1.0, -lSin225, +lCos225, kMf}},
        {msg::OctoMap::kFront22Left, {1.0, +lSin225, +lCos225, kMf}},
        {msg::OctoMap::kFront67Left, {1.0, +lCos225, +lSin225, kMf}},
        {msg::OctoMap::kRear67Left, {1.0, +lCos225, -lSin225, kMf}},
        {msg::OctoMap::kRear22Left, {1.0, +lSin225, -lCos225, kMf}},
        {msg::OctoMap::kRear22Right, {1.0, -lSin225, -lCos225, kMf}},
        {msg::OctoMap::kRear67Right, {1.0, -lCos225, -lSin225, kMf}},
        {msg::OctoMap::kFront67Right, {1.0, -lCos225, +lSin225, kMf}}};
    for (std::uint8_t i{0}; i < 8; ++i) {
        mixer.col(i) = motorToMixerMap.at(octoAlloc.indexAlloc[i]);
        if (!octoAlloc.isCCWorCW[i]) {
            mixer.col(i)[3] = -mixer.col(i)[3];
        }
    }

    const auto mixerT = mixer.transpose();
    mixerPseudoInv_ = mixerT * (mixer * mixerT).inverse();
}

void QuadMixer::update(const msg::ControlCommands& command) const {
    const double T = command.thrust;
    const auto& tau = command.torque;  // [roll, pitch, yaw]
    const auto controlVector = Eigen::Vector<double, 4>{T, tau.x(), tau.y(), tau.z()};

    // Per-motor thrusts [N]
    static msg::ActuatorCommands cmd;
    const auto motorThrusts = (mixerInv_ * controlVector).eval();

    for (std::uint8_t i{0}; i < 4; ++i) {
        cmd.motorThrusts[i] = std::clamp(motorThrusts[i], 0.0, referenceMaxThrustPerMotor_);
        cmd.motorThrottleCmd[i] = std::sqrt(cmd.motorThrusts[i] * invReferenceMaxThrustPerMotor_);
        cmd.motorSpeeds[i] = std::sqrt(cmd.motorThrusts[i] * invThrustCoefficient_);
    }
    cmd.simStamp = lastControlCommands_.simStamp;
    cmd.mode = msg::ActuatorMode::kThrottle;
    cmd.motorNum = 4;

    publisherActuatorCommands_.publish(cmd);
}

void HexaMixer::update(const msg::ControlCommands& command) const {
    const double T = command.thrust;
    const auto& tau = command.torque;  // [roll, pitch, yaw]
    const auto controlVector = Eigen::Vector<double, 4>{T, tau.x(), tau.y(), tau.z()};

    // Per-motor thrusts [N]
    static msg::ActuatorCommands cmd;
    const auto motorThrusts = (mixerPseudoInv_ * controlVector).eval();
    for (std::uint8_t i{0}; i < 6; ++i) {
        cmd.motorThrusts[i] = std::clamp(motorThrusts[i], 0.0, referenceMaxThrustPerMotor_);
        cmd.motorThrottleCmd[i] = std::sqrt(cmd.motorThrusts[i] * invReferenceMaxThrustPerMotor_);
        cmd.motorSpeeds[i] = std::sqrt(cmd.motorThrusts[i] * invThrustCoefficient_);
    }
    cmd.simStamp = lastControlCommands_.simStamp;
    cmd.mode = msg::ActuatorMode::kThrottle;
    cmd.motorNum = 6;

    publisherActuatorCommands_.publish(cmd);
}

void OctoMixer::update(const msg::ControlCommands& command) const {
    const double T = command.thrust;
    const auto& tau = command.torque;  // [roll, pitch, yaw]
    const auto controlVector = Eigen::Vector<double, 4>{T, tau.x(), tau.y(), tau.z()};

    // Per-motor thrusts [N]
    static msg::ActuatorCommands cmd;
    const auto motorThrusts = (mixerPseudoInv_ * controlVector).eval();

    for (std::uint8_t i{0}; i < 8; ++i) {
        cmd.motorThrusts[i] = std::clamp(motorThrusts[i], 0.0, referenceMaxThrustPerMotor_);
        cmd.motorThrottleCmd[i] = std::sqrt(cmd.motorThrusts[i] * invReferenceMaxThrustPerMotor_);
        cmd.motorSpeeds[i] = std::sqrt(cmd.motorThrusts[i] * invThrustCoefficient_);
    }

    cmd.simStamp = lastControlCommands_.simStamp;
    cmd.mode = msg::ActuatorMode::kThrottle;
    cmd.motorNum = 8;

    publisherActuatorCommands_.publish(cmd);
}

}  // namespace dronelab::sim