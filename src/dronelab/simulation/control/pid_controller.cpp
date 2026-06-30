#include "dronelab/simulation/control/pid_controller.hpp"

#include <atomic>
#include <cmath>

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/data/constants.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/simulation/utils/eigen_adapter.hpp"

namespace dronelab::sim {

PIDController::PIDController(ContextView context)
    : Node{context, NodeType::kController},
      subscriberSimTimeAtt_{context, *this, TopicKey::kSimTime},
      subscriberSimTimePos_{context, *this, TopicKey::kSimTime},
      subscriberStateEstimate_{context, *this, TopicKey::kStateEstimate},
      subscriberDroneState_{context, *this, TopicKey::kDroneState},
      subscriberSetpoint_{context, *this, TopicKey::kSetPoint},
      subscriberSimCmd_{context, *this, TopicKey::kSimCmd},
      publisherControlCommand_{context, *this, TopicKey::kControlCommands},
      publisherPidLog_{context, *this, TopicKey::kPIDLog},
      controlGains_{context.getConfig().getPIDCascadeGains()},
      droneParams_{context.getConfig().getDroneParameters()},
      inertia_{Eigen::Map<const Eigen::Vector3d>(
                   context.getConfig().getDroneParameters().inertiaDiag.data())
                   .asDiagonal()},
      posRateUs_{static_cast<int32_t>(
          1000000.0 / static_cast<double>(context.getConfig().getPosControllerRate()))},
      posRateSeconds_{1.0 / static_cast<double>(context.getConfig().getPosControllerRate())},
      attRateUs_{static_cast<int32_t>(
          1000000.0 / static_cast<double>(context.getConfig().getAttControllerRate()))},
      attRateSeconds_{1.0 / static_cast<double>(context.getConfig().getAttControllerRate())} {
    if (!useRealState_) {
        subscriberStateEstimate_.subscribe(
            [this](const auto& msg) -> void { receivedStateEstimate(*msg); });
    } else {
        SPDLOG_WARN("[{}] Using drone's true state instead of estimated state.", name_);
        subscriberDroneState_.subscribe(
            [this](const auto& msg) -> void { receivedDroneState(*msg); });
    }

    subscriberSetpoint_.subscribe([this](const auto& msg) { receivedSetPoint(*msg); });

    subscriberSimTimePos_
        .filter([this](const auto& msg) -> bool {
            return (*msg) - posLastStepUs_.load(std::memory_order_relaxed) >= posRateUs_;
        })
        .subscribe([this](const auto& msg) -> void {
            posLastStepUs_.store(*msg, std::memory_order_relaxed);
            std::lock_guard lock{mutex_};
            updatePos();
            updateVel();
        });
    subscriberSimTimeAtt_
        .filter([this](const auto& msg) -> bool {
            return (*msg) - attLastStepUs_.load(std::memory_order_relaxed) >= attRateUs_ &&
                   hasPos_.load(std::memory_order_acquire);
        })
        .subscribe([this](const auto& msg) -> void {
            attLastStepUs_.store(*msg, std::memory_order_relaxed);
            std::lock_guard lock{mutex_};
            updateAtt();
            updateCommands();
        });

    subscriberSimCmd_
        .filter([](const auto& msg) {
            return std::holds_alternative<sim::msg::PIDUpdateCmd>(msg->cmd);
        })
        .subscribe([](const auto& msg) {
            // Temp
            PIDController::processSimCmd(std::get<sim::msg::PIDUpdateCmd>(msg->cmd));
        });
}

void PIDController::processSimCmd(const msg::PIDUpdateCmd& /*cmd*/) {}

void PIDController::receivedStateEstimate(const msg::StateEstimate& stateEstimate) {
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

void PIDController::receivedDroneState(const msg::DroneState& state) {
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

void PIDController::receivedSetPoint(const msg::Setpoint& setPoint) {
    std::lock_guard lock{mutex_};
    SPDLOG_TRACE("[{}] Received Set Point. Pos [m]: {}, {}, {}", name_, setPoint.position.x(),
                 setPoint.position.y(), setPoint.position.z());

    currentSetpoint_ = setPoint;
    if (!setpointInitialized_) {
        setpointInitialized_ = true;
        SPDLOG_DEBUG("[{}] Received first set point.", name_);
    }
}

void PIDController::updatePos() {
    if (!setpointInitialized_ || !stateInitialized_) {
        return;
    }
    SPDLOG_TRACE("[{}] On Position update.", name_);

    // === 1. POSITION CONTROL (World Frame) ===
    const auto positionError = currentSetpoint_.position - currentState_.position;
    // Update integral with windup protection
    controlGains_.positionGains.updateIntegralError(positionError, posRateSeconds_);

    auto posLog = msg::PIDLog{.loop = msg::PIDLoop::kPos,
                              .error = positionError,
                              .simStamp = posLastStepUs_.load(std::memory_order_relaxed)};
    const auto desiredVelFromPos = controlGains_.positionGains.calculateGain(
        positionError, posRateSeconds_, posLog.p, posLog.i, posLog.d);
    posLog.output = desiredVelFromPos;
    publisherPidLog_.publish(posLog);

    // Combine with feedforward desired velocity
    totalDesiredVel_ = currentSetpoint_.velocity + desiredVelFromPos;
}

void PIDController::updateVel() {
    if (!setpointInitialized_ || !stateInitialized_) {
        return;
    }
    SPDLOG_TRACE("[{}] On Command update.", name_);

    // VELOCITY CONTROL (World Frame)
    const auto velocityError = totalDesiredVel_ - currentState_.velocity;

    controlGains_.velocityGains.updateIntegralError(velocityError, posRateSeconds_);

    auto velLog = msg::PIDLog{.loop = msg::PIDLoop::kVel,
                              .error = velocityError,
                              .simStamp = posLastStepUs_.load(std::memory_order_relaxed)};
    const auto desiredAccelFromVel = controlGains_.velocityGains.calculateGain(
        velocityError, posRateSeconds_, velLog.p, velLog.i, velLog.d);
    velLog.output = desiredAccelFromVel;
    publisherPidLog_.publish(velLog);
    // Total desired acceleration (world frame)
    totalDesiredAcc_ = currentSetpoint_.acceleration + desiredAccelFromVel;

    // Derive desired rotation

    SPDLOG_TRACE("[{}] On Command update.", name_);

    // COMPUTE DESIRED THRUST VECTOR (World Frame)
    static constexpr auto kGravityWorld = Vec3d{0, 0, constants::kGravity};  // NED: gravity in +Z
    const Eigen::Vector3d fDesWorld =
        droneParams_.mass * utils::toEigen(totalDesiredAcc_ - kGravityWorld);

    thrustMagnitude_ = fDesWorld.norm();

    // Desired thrust direction (unit vector) (World components)
    Eigen::Vector3d zBDes = -fDesWorld.normalized();
    static const auto zBodyHover = Eigen::Vector3d{0, 0, 1};
    const double cosAngle = std::clamp(zBDes.dot(zBodyHover), -1.0, 1.0);
    const double angle = std::acos(cosAngle);
    if (angle > droneParams_.maxTiltAngle) {
        // Axis to rotate around to get from z_hover to zBDes
        Eigen::Vector3d rotAxis = zBodyHover.cross(zBDes);
        if (rotAxis.norm() < 1e-6) {
            rotAxis = Eigen::Vector3d(1, 0, 0);
        } else {
            rotAxis.normalize();
        }

        // Create quaternion representing rotation of maxTiltAngle about rot axis
        Eigen::AngleAxisd limitedRotation(droneParams_.maxTiltAngle, rotAxis);

        // Rotate z_hover by limited_rotation to get limited zBDes
        SPDLOG_TRACE("[{}] Requested Zbody direction [{} {} {}] exceeds maximum tilt angle ({})",
                     name_, zBDes.x(), zBDes.y(), zBDes.z(), droneParams_.maxTiltAngle);
        zBDes = limitedRotation * zBodyHover;
        zBDes.normalize();
        SPDLOG_TRACE("[{}] Capping Zbody direction to [{} {} {}]", name_, zBDes.x(), zBDes.y(),
                     zBDes.z());

        thrustMagnitude_ = std::max(fDesWorld.dot(-zBDes), 0.0);
    }

    const auto yawDes = currentSetpoint_.yaw;
    auto xC =
        Eigen::Vector3d{std::cos(yawDes), std::sin(yawDes), 0.0};  // yaw direction in world frame

    Eigen::Vector3d yBDes = zBDes.cross(xC);
    if (yBDes.norm() < 1e-6) {
        // Handle singularity case - use alternative x direction
        auto xAlt = Eigen::Vector3d{1.0, 0.0, 0.0};
        if (std::abs(zBDes.dot(xAlt)) > 0.9) {
            xAlt = {0.0, 1.0, 0.0};
        }
        yBDes = zBDes.cross(xAlt);
    }
    yBDes.normalize();
    const Eigen::Vector3d xBDes = yBDes.cross(zBDes).normalized();

    Eigen::Matrix3d rDesWb;
    rDesWb.col(0) = xBDes;
    rDesWb.col(1) = yBDes;
    rDesWb.col(2) = zBDes;

    SPDLOG_TRACE("[{}] Desired X body axis components (world frame): {}, {}, {}", name_, xBDes.x(),
                 xBDes.y(), xBDes.z());
    SPDLOG_TRACE("[{}] Desired Y body axis components (world frame): {}, {}, {}", name_, yBDes.x(),
                 yBDes.y(), yBDes.z());
    SPDLOG_TRACE("[{}] Desired Z body axis components (world frame): {}, {}, {}", name_, zBDes.x(),
                 zBDes.y(), zBDes.z());

    qDesiredWb_ = Eigen::Quaterniond{rDesWb};

    hasPos_.store(true, std::memory_order_release);
}

void PIDController::updateAtt() {
    SPDLOG_TRACE("[{}] On Command update.", name_);

    // ATTITUDE CONTROL
    // Current and desired are Body-To-World.
    const auto qCurrentWb = utils::toEigenQuat(currentState_.orientation);
    Eigen::Quaterniond qError = (qCurrentWb.inverse() * qDesiredWb_).normalized();
    if (qError.w() < 0) {
        qError.coeffs() *= -1;
    }

    const auto angleAxisError = Eigen::AngleAxisd{qError};
    const double angleErr = angleAxisError.angle();
    const Eigen::Vector3d& axis = angleAxisError.axis();

    Vec3d attitudeErrorBody;
    if (angleErr > 1e-6) {
        attitudeErrorBody = utils::fromEigen(angleErr * axis);
    }
    SPDLOG_TRACE("[{}] Attitude error (world to body frame) [rad]: {}, {}, {}", name_,
                 attitudeErrorBody.x(), attitudeErrorBody.y(), attitudeErrorBody.z());

    controlGains_.attitudeGains.updateIntegralError(attitudeErrorBody, attRateSeconds_);
    auto attLog = msg::PIDLog{.loop = msg::PIDLoop::kAtt,
                              .error = attitudeErrorBody,
                              .simStamp = attLastStepUs_.load(std::memory_order_relaxed)};
    desiredAngVelocity_ = controlGains_.attitudeGains.calculateGain(
        attitudeErrorBody, attRateSeconds_, attLog.p, attLog.i, attLog.d);
    attLog.output = desiredAngVelocity_;
    publisherPidLog_.publish(attLog);
}

void PIDController::updateCommands() {
    // ANGULAR RATE CONTROL
    const auto angularError = desiredAngVelocity_ - currentState_.angularVelocity;

    auto rateLog = msg::PIDLog{.loop = msg::PIDLoop::kRate,
                               .error = angularError,
                               .simStamp = attLastStepUs_.load(std::memory_order_relaxed)};
    // PID control law
    const auto tauControl = controlGains_.angularRateGains.calculateGain(
        angularError, attRateSeconds_, rateLog.p, rateLog.i, rateLog.d);
    rateLog.output = tauControl;
    publisherPidLog_.publish(rateLog);

    // Add dynamics compensation (feedforward term)
    // tau = J * omega_dot + omega x (J * omega)
    const auto omegaEigen = utils::toEigen(currentState_.angularVelocity);
    const auto gyroscopicTerm = utils::fromEigen(omegaEigen.cross(inertia_ * omegaEigen));

    // Total torque command
    double thrustBodyZUpward = std::max(thrustMagnitude_, 0.0);  // body z-axis in world frame

    // Fill and publish
    lastControlCommand_.thrust = thrustBodyZUpward;
    lastControlCommand_.torque = tauControl + gyroscopicTerm;
    lastControlCommand_.simStamp = attLastStepUs_.load(std::memory_order_relaxed);

    publisherControlCommand_.publish(lastControlCommand_);
}

}  // namespace dronelab::sim