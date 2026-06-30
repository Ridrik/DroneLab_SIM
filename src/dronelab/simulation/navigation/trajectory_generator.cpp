#include "dronelab/simulation/navigation/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/utils/common.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/navigation/trajectory.hpp"
#include "dronelab/simulation/utils/eigen_adapter.hpp"

namespace dronelab::sim {

TrajectoryGenerator::TrajectoryGenerator(ContextView context)
    : Node{context, NodeType::kTrajectoryGenerator},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberStateEstimate_{context, *this, TopicKey::kStateEstimate},
      subscriberDroneState_{context, *this, TopicKey::kDroneState},
      subscriberTrajTarget_{context, *this, TopicKey::kTrajectoryTarget},
      subscriberFlightStatus_{context, *this, TopicKey::kFlightStatus},
      publisherSetPoint_{context, *this, TopicKey::kSetPoint},
      strategy_{context.getConfig().getTrajectoryStrategy()},
      droneParameters_{context.getConfig().getDroneParameters()},
      useRealState_{context.getConfig().useRealState()},
      updateRateSeconds_{1.0 / static_cast<double>(context.getConfig().getTrajectoryRate())},
      updateRateUs_{static_cast<int32_t>(updateRateSeconds_ * 1e6)} {
    if (!useRealState_) {
        subscriberStateEstimate_.subscribe(
            [this](const auto& msg) -> void { receivedStateEstimate(*msg); });
    } else {
        SPDLOG_WARN("[{}] Using drone's true state instead of estimated state.", name_);
        subscriberDroneState_.subscribe(
            [this](const auto& msg) -> void { receivedDroneState(*msg); });
    }
    subscriberTrajTarget_.subscribe(
        [this](const auto& msg) -> void { receivedTrajectoryTarget(*msg); });
    subscriberFlightStatus_.subscribe(
        [this](const auto& msg) -> void { receivedFlightStatus(*msg); });
    SPDLOG_DEBUG("[{}] Trajectory strategy mode is {}, with interval {}", name_,
                 msg::trajStratToString(strategy_), t_);

    subscriberSimTime_
        .filter([this](const auto& msg) -> bool {
            return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
        })
        .subscribe([this](const auto& msg) -> void {
            lastStepUs_.store(*msg, std::memory_order_relaxed);
            calculateSetPoint();
        });
}

void TrajectoryGenerator::receivedStateEstimate(const msg::StateEstimate& state) {
    std::lock_guard lock{mutex_};
    SPDLOG_TRACE("[{}] Received state estimate. Pos [m]: [{}, {}, {}]", name_, state.position.x(),
                 state.position.y(), state.position.z());
    lastEstimate_ = state;
    initialized_ = true;
}

void TrajectoryGenerator::receivedDroneState(const msg::DroneState& state) {
    std::lock_guard lock{mutex_};
    SPDLOG_TRACE("[{}] Received state estimate. Pos [m]: [{}, {}, {}]", name_, state.position.x(),
                 state.position.y(), state.position.z());
    lastEstimate_.position = state.position;
    lastEstimate_.velocity = state.velocity;
    lastEstimate_.orientation = state.orientation;
    lastEstimate_.angularVelocity = state.angularVelocity;
    lastEstimate_.gyroBias = {};
    lastEstimate_.accelBias = {};
    lastEstimate_.simStamp = lastStepUs_.load(std::memory_order_relaxed);
    initialized_ = true;
}

void TrajectoryGenerator::receivedTrajectoryTarget(const msg::TrajectoryTarget& trajWayPoint) {
    std::lock_guard lock{mutex_};
    SPDLOG_DEBUG("[{}] Received trajectory target. Pos [m]: [{}, {}, {}]", name_,
                 trajWayPoint.position.x(), trajWayPoint.position.y(), trajWayPoint.position.z());
    lastTrajTarget_ = trajWayPoint;
    waypointReceived_ = true;
}

void TrajectoryGenerator::calculateTrajectory() {
    SPDLOG_DEBUG("[{}] Calculating new trajectory.", name_);
    Eigen::MatrixXd A(6, 6);  // NOLINT
    double t2 = t_ * t_;
    double t3 = t2 * t_;
    double t4 = t3 * t_;
    double t5 = t4 * t_;

    A << 1, 0, 0, 0, 0, 0,                     // pos(0)
        0, 1, 0, 0, 0, 0,                      // vel(0)
        0, 0, 2, 0, 0, 0,                      // acc(0)
        1, t_, t2, t3, t4, t5,                 // pos(T)
        0, 1, 2 * t_, 3 * t2, 4 * t3, 5 * t4,  // vel(T)
        0, 0, 2, 6 * t_, 12 * t2, 20 * t3;     // acc(T)

    Eigen::VectorXd b(6);

    // Solve for X
    b << lastEstimate_.position.x(), lastEstimate_.velocity.x(), 0, lastTrajTarget_.position.x(), 0,
        0;
    trajectory_.coeffsX = utils::fromEigen<6>(A.colPivHouseholderQr().solve(b).eval());

    // Solve for Y
    b << lastEstimate_.position.y(), lastEstimate_.velocity.y(), 0, lastTrajTarget_.position.y(), 0,
        0;
    trajectory_.coeffsY = utils::fromEigen<6>(A.colPivHouseholderQr().solve(b).eval());

    // Solve for Z
    b << lastEstimate_.position.z(), lastEstimate_.velocity.z(), 0, lastTrajTarget_.position.z(), 0,
        0;

    trajectory_.coeffsZ = utils::fromEigen<6>(A.colPivHouseholderQr().solve(b).eval());

    const auto& qCurrentBw = lastEstimate_.orientation;
    const double qW = qCurrentBw[3];
    const double qX = qCurrentBw[0];
    const double qY = qCurrentBw[1];
    const double qZ = qCurrentBw[2];

    // yaw extraction (NED)
    double yawStart = std::atan2(2.0 * (qW * qZ + qX * qY), 1.0 - 2.0 * (qY * qY + qZ * qZ));
    double yawEnd = lastTrajTarget_.yaw;

    double yawError = yawEnd - yawStart;
    yawError = ::dronelab::utils::wrapAtPi(yawError);

    // Now the smooth version:
    const double yawEndUnwrapped = yawStart + yawError;

    Eigen::VectorXd bYaw(6);
    bYaw << yawStart, 0.0, 0.0, yawEndUnwrapped, 0.0, 0.0;

    trajectory_.coeffsYaw = utils::fromEigen<6>(A.colPivHouseholderQr().solve(bYaw));
    trajectory_.valid = true;

    timePoint_ = 0;
}

void TrajectoryGenerator::calculateSetPoint() {
    std::lock_guard lock{mutex_};
    switch (strategy_) {
        case msg::TrajectoryStrategy::kPolynomial: {
            if (initialized_ && waypointReceived_) {
                waypointReceived_ = false;
                // Update t_ and calculate the trajectory
                t_ = lastTrajTarget_.convergenceTime;
                calculateTrajectory();
            }
            if (trajectory_.valid && latestFlightStatus_.mode == lastTrajTarget_.flightMode) {
                calculateSetPointPolynomial();
            } else {
                SPDLOG_TRACE(
                    "[{}] Invalid trajectory or flight mode not on trajectory mode. Skipping "
                    "setpoing",
                    name_);
            }
            break;
        }
        default: {
            if (waypointReceived_) {
                calculateSetPointPlaceHolder();
                waypointReceived_ = false;
            }
            break;
        }
    }
}

void TrajectoryGenerator::calculateSetPointPlaceHolder() {
    SPDLOG_TRACE("[{}] Calculating set point directly from waypoint.", name_);
    msg::Setpoint setPoint;
    setPoint.position = lastTrajTarget_.position;
    Eigen::AngleAxisd yawRot(lastTrajTarget_.yaw, Eigen::Vector3d::UnitZ());
    setPoint.yaw = lastTrajTarget_.yaw;
    setPoint.simStamp = lastStepUs_.load(std::memory_order_relaxed);
    publisherSetPoint_.publish(setPoint);
}

void TrajectoryGenerator::calculateSetPointPolynomial() {
    SPDLOG_TRACE("[{}] Calculating set point from trajectory.", name_);
    msg::Setpoint setPoint;

    timePoint_ += updateRateUs_;
    const auto t = std::min(msg::microsToSeconds<double>(timePoint_), t_);

    evaluateTrajectory(t, setPoint.position, setPoint.velocity, setPoint.acceleration,
                       setPoint.yaw);

    // Set timestamp
    setPoint.simStamp = lastStepUs_.load(std::memory_order_relaxed);

    publisherSetPoint_.publish(setPoint);
}

void TrajectoryGenerator::evaluateTrajectory(double t, Vec3d& pos, Vec3d& vel, Vec3d& acc,
                                             double& yaw) const {
    const auto tvec = Vecd<6>{1, t, t * t, t * t * t, t * t * t * t, t * t * t * t * t};
    const auto dTvec = Vecd<6>{0, 1, 2 * t, 3 * t * t, 4 * t * t * t, 5 * t * t * t * t};
    const auto ddTvec = Vecd<6>{0, 0, 2, 6 * t, 12 * t * t, 20 * t * t * t};

    pos.x() = trajectory_.coeffsX.dot(tvec);
    pos.y() = trajectory_.coeffsY.dot(tvec);
    pos.z() = trajectory_.coeffsZ.dot(tvec);

    vel.x() = trajectory_.coeffsX.dot(dTvec);
    vel.y() = trajectory_.coeffsY.dot(dTvec);
    vel.z() = trajectory_.coeffsZ.dot(dTvec);

    acc.x() = trajectory_.coeffsX.dot(ddTvec);
    acc.y() = trajectory_.coeffsY.dot(ddTvec);
    acc.z() = trajectory_.coeffsZ.dot(ddTvec);

    yaw = ::dronelab::utils::wrapAtPi(trajectory_.coeffsYaw.dot(tvec));
}

void TrajectoryGenerator::receivedFlightStatus(const msg::FlightStatus& status) {
    latestFlightStatus_ = status;
}

}  // namespace dronelab::sim