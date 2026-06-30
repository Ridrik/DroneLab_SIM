#include "dronelab/simulation/control/lqr_controller.hpp"

#include <atomic>
#include <cstdint>

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/data/constants.hpp"
#include "dronelab/api/core/exceptions.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/simulation/control/plant.hpp"
#include "dronelab/simulation/data/control/lqr.hpp"
#include "dronelab/simulation/utils/eigen_adapter.hpp"

namespace dronelab::sim {

LQRController::LQRController(ContextView context)
    : Node{context, NodeType::kController},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberStateEstimate_{context, *this, TopicKey::kStateEstimate},
      subscriberDroneState_{context, *this, TopicKey::kDroneState},
      subscriberSetpoint_{context, *this, TopicKey::kSetPoint},
      subscriberSimCmd_{context, *this, TopicKey::kSimCmd},
      publisherControlCommand_{context, *this, TopicKey::kControlCommands},
      droneParams_{context.getConfig().getDroneParameters()},
      intervalRateUs_{static_cast<std::int32_t>(
          1000000.0 / static_cast<double>(context.getConfig().getAttControllerRate()))},
      intervalRateSeconds_{1.0 / static_cast<double>(context.getConfig().getAttControllerRate())} {
    if (!useRealState_) {
        subscriberStateEstimate_.subscribe(
            [this](const auto& msg) -> void { receivedStateEstimate(*msg); });
    } else {
        SPDLOG_WARN("[{}] Using drone's true state instead of estimated state.", name_);
        subscriberDroneState_.subscribe(
            [this](const auto& msg) -> void { receivedDroneState(*msg); });
    }

    const auto& dt = intervalRateSeconds_;
    Eigen::Matrix<double, kStateDim, kStateDim> A{
        Eigen::Matrix<double, kStateDim, kStateDim>::Identity()};
    A.block<3, 3>(+msg::StateIdx::kPos, +msg::StateIdx::kVel) = dt * Eigen::Matrix3d::Identity();
    A.block<3, 3>(+msg::StateIdx::kAngle, +msg::StateIdx::kRate) = dt * Eigen::Matrix3d::Identity();
    const auto& g = constants::kGravity;
    A(+msg::StateIdx::kVel + 0, +msg::StateIdx::kAngle + 1) = -g * dt;
    A(+msg::StateIdx::kVel + 1, +msg::StateIdx::kAngle + 0) = g * dt;
    Eigen::Matrix<double, kStateDim, kControlDim> B{
        Eigen::Matrix<double, kStateDim, kControlDim>::Zero()};
    B(+msg::StateIdx::kVel + 2, 0) = -dt / droneParams_.mass;
    B(+msg::StateIdx::kRate, 1) = dt / droneParams_.inertiaDiag[0];
    B(+msg::StateIdx::kRate + 1, 2) = dt / droneParams_.inertiaDiag[1];
    B(+msg::StateIdx::kRate + 2, 3) = dt / droneParams_.inertiaDiag[2];
    Eigen::Matrix<double, kStateDim, kStateDim> Q{
        Eigen::Matrix<double, kStateDim, kStateDim>::Zero()};
    Q.diagonal() << 20.0, 20.0, 30.0,  // Position weights
        4.0, 4.0, 6.0,                 // Velocity weights
        25.0, 25.0, 40.0,              // Angle weights
        3.0, 3.0, 2.0;                 // Rate weights
    Eigen::Matrix<double, kControlDim, kControlDim> R{
        Eigen::Matrix<double, kControlDim, kControlDim>::Zero()};
    R.diagonal() << 0.5, 0.05, 0.05, 0.3;
    if (auto expectGains = controlGains_.computeStaticGain(A, B, Q, R); !expectGains.has_value()) {
        throw exceptions::InitializationException(
            std::format("[{}] LQR Solver did not converge: {}", name_,
                        msg::dareErrorToString(expectGains.error())));
    }
    uHover_ << droneParams_.mass * g, 0.0, 0.0, 0.0;

    subscriberSetpoint_.subscribe([this](const auto& msg) { receivedSetPoint(*msg); });

    subscriberSimTime_
        .filter([this](const auto& msg) -> bool {
            return (*msg) - lastStepUs_.load(std::memory_order_relaxed) >= intervalRateUs_;
        })
        .subscribe([this](const auto& msg) -> void {
            lastStepUs_.store(*msg, std::memory_order_relaxed);
            update();
        });

    subscriberSimCmd_
        .filter([](const auto& msg) {
            return std::holds_alternative<sim::msg::PIDUpdateCmd>(msg->cmd);
        })
        .subscribe([](const auto& msg) {
            // Temp
            LQRController::processSimCmd(std::get<sim::msg::PIDUpdateCmd>(msg->cmd));
        });
}

void LQRController::processSimCmd(const msg::PIDUpdateCmd& /*cmd*/) {}

void LQRController::receivedStateEstimate(const msg::StateEstimate& stateEstimate) {
    std::lock_guard lock{mutex_};
    SPDLOG_TRACE("[{}] Processing state estimate. Pos [m]: [{}, {}, {}]", name_,
                 stateEstimate.position.x(), stateEstimate.position.y(),
                 stateEstimate.position.z());
    currentState_ = stateEstimate;

    if (!stateInitialized_) {
        stateInitialized_ = true;
        SPDLOG_DEBUG("[{}] Received first state estimate.", name_);
    }
}

void LQRController::receivedDroneState(const msg::DroneState& state) {
    std::lock_guard lock{mutex_};
    SPDLOG_TRACE("[{}] Processing drone state. Pos [m]: [{}, {}, {}]", name_, state.position.x(),
                 state.position.y(), state.position.z());
    currentState_.position = state.position;
    currentState_.velocity = state.velocity;
    currentState_.orientation = state.orientation;
    currentState_.angularVelocity = state.angularVelocity;
    currentState_.gyroBias = {};
    currentState_.accelBias = {};
    currentState_.simStamp = state.simStamp;

    if (!stateInitialized_) {
        stateInitialized_ = true;
        SPDLOG_DEBUG("[{}] Received first state estimate.", name_);
    }
}

void LQRController::receivedSetPoint(const msg::Setpoint& setPoint) {
    std::lock_guard lock{mutex_};
    SPDLOG_TRACE("[{}] Received Set Point. Pos [m]: {}, {}, {}", name_, setPoint.position.x(),
                 setPoint.position.y(), setPoint.position.z());

    currentSetpoint_ = setPoint;
    if (!setpointInitialized_) {
        setpointInitialized_ = true;
        SPDLOG_DEBUG("[{}] Received first set point.", name_);
    }
}

void LQRController::update() {
    std::lock_guard lock{mutex_};
    if (!setpointInitialized_ || !stateInitialized_) {
        return;
    }
    Eigen::Vector<double, kStateDim> xRef{Eigen::Vector<double, kStateDim>::Zero()};
    xRef.block<3, 1>(+msg::StateIdx::kPos, 0) = utils::toEigen(currentSetpoint_.position);
    xRef.block<3, 1>(+msg::StateIdx::kVel, 0) = utils::toEigen(currentSetpoint_.velocity);
    xRef(+msg::StateIdx::kAngle + 2, 0) = currentSetpoint_.yaw;
    Eigen::Vector<double, kStateDim> xCur{Eigen::Vector<double, kStateDim>::Zero()};
    xCur.block<3, 1>(+msg::StateIdx::kPos, 0) = utils::toEigen(currentState_.position);
    xCur.block<3, 1>(+msg::StateIdx::kVel, 0) = utils::toEigen(currentState_.velocity);
    const auto eulerAngleCur = utils::toEigenQuat(currentState_.orientation)
                                   .toRotationMatrix()
                                   .canonicalEulerAngles(2, 1, 0);
    xCur.block<3, 1>(+msg::StateIdx::kAngle, 0) =
        Eigen::Vector3d{eulerAngleCur(2, 0), eulerAngleCur(1, 0), eulerAngleCur(0, 0)};
    xCur.block<3, 1>(+msg::StateIdx::kRate, 0) = utils::toEigen(currentState_.angularVelocity);

    Eigen::Vector<double, kStateDim> xError = xCur - xRef;
    xError(+msg::StateIdx::kAngle + 2, 0) =
        ::dronelab::utils::wrapAtPi(xError(+msg::StateIdx::kAngle + 2, 0));
    const Eigen::Vector4d out = controlGains_.computeCommand(xError) + uHover_;
    lastControlCommand_.thrust = out(0);
    lastControlCommand_.torque = utils::fromEigen(out.block<3, 1>(1, 0));
    lastControlCommand_.simStamp = lastStepUs_;
    publisherControlCommand_.publish(lastControlCommand_);
}

}  // namespace dronelab::sim