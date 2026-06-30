#include "dronelab/simulation/navigation/rtl_behaviour.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <spdlog/spdlog.h>

#include "dronelab/api/core/config.hpp"
#include "dronelab/api/simulation/management/flight.hpp"
#include "dronelab/api/simulation/management/mission.hpp"
#include "dronelab/api/simulation/navigation/trajectory.hpp"
#include "dronelab/simulation/utils/eigen_adapter.hpp"
#include "dronelab/simulation/utils/representations.hpp"

namespace dronelab::sim {

RtlBehaviour::RtlBehaviour(ContextView context)
    : Node{context, NodeType::kRtlBehaviour},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberFlightStatus_{context, *this, TopicKey::kFlightStatus},
      subscriberStateEstimate_{context, *this, TopicKey::kStateEstimate},
      subscriberDroneState_{context, *this, TopicKey::kDroneState},
      publisherTrajTarget_{context, *this, TopicKey::kTrajectoryTarget},
      publisherMissionEvent_{context, *this, TopicKey::kMissionEvent},
      rtlBehaviourConfig_{context.getConfig().getMission().rtlBehaviourConfig},
      updateRateSeconds_{1.0 / static_cast<double>(context.getConfig().getRtlBehaviourRate())},
      updateRateUs_{static_cast<std::int32_t>(updateRateSeconds_ * 1e6)},
      useRealState_{context.getConfig().useRealState()} {
    subscriberFlightStatus_.subscribe(
        [this](const auto& msg) -> void { receivedFlightStatus(*msg); });
    if (useRealState_) {
        subscriberDroneState_.subscribe(
            [this](const auto& msg) -> void { receivedDroneState(*msg); });
    } else {
        subscriberStateEstimate_.subscribe(
            [this](const auto& msg) -> void { receivedStateEstimate(*msg); });
    }

    subscriberSimTime_
        .filter([this](const auto& msg) -> bool {
            return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
        })
        .subscribe([this](const auto& msg) -> void {
            internalLastStepUs_ = *msg;
            lastStepUs_.store(*msg, std::memory_order_relaxed);
            update();
        });
}

void RtlBehaviour::receivedStateEstimate(const msg::StateEstimate& state) {
    std::lock_guard lock{mutex_};
    lastState_ = state;
}

void RtlBehaviour::receivedDroneState(const msg::DroneState& state) {
    std::lock_guard lock{mutex_};
    lastState_.position = state.position;
    lastState_.velocity = state.velocity;
    lastState_.orientation = state.orientation;
    lastState_.angularVelocity = state.angularVelocity;
    lastState_.gyroBias = {};
    lastState_.accelBias = {};
    lastState_.simStamp = state.simStamp;
}

void RtlBehaviour::receivedFlightStatus(const msg::FlightStatus& status) {
    std::lock_guard lock{mutex_};
    latestFlightStatus_ = status;
}

double RtlBehaviour::computeConvergenceTime(const Eigen::Vector3d& relWpPos, double yawCur,
                                            double nextYaw,
                                            double nominalSpeed,  // m/s
                                            double minTime,       // s
                                            double maxTime        // s
) const {
    const double distance = relWpPos.norm();
    const double yawDistance = std::abs(nextYaw - yawCur);
    const double time = distance / nominalSpeed + yawDistance * 180.0 / (45 * std::numbers::pi);
    SPDLOG_DEBUG("[{}] Convergence time: distance calculated is {}. Time before clamping: {}",
                 name_, distance, time);
    return std::clamp(time, minTime, maxTime);
}

void RtlBehaviour::sendHomeTarget(const Vec3d& homePosition) {
    const double yaw = utils::getYawFromQuaternion(std::span{lastState_.orientation.data(), 4});
    msg::TrajectoryTarget trajTarget;
    trajTarget.position = homePosition + rtlBehaviourConfig_.safeTargetFromHome;
    trajTarget.yaw = 0.0;
    trajTarget.flightMode = msg::FlightMode::kReturnToLaunch;
    trajTarget.convergenceTime = computeConvergenceTime(
        utils::toEigen(homePosition - lastState_.position), yaw, 0.0,
        rtlBehaviourConfig_.nominalSpeed, rtlBehaviourConfig_.minimumTargetTime,
        rtlBehaviourConfig_.maximumTargetTime);
    homeTrack_.maxTimeBeforeTimeout =
        trajTarget.convergenceTime * (1.0 + rtlBehaviourConfig_.clearanceTimeThreshold);
    homeTrack_.radiusOut =
        rtlBehaviourConfig_.acceptanceRadius * msg::RtlBehaviourConfig::kHysteresis;
    homeTrack_.yawOut = rtlBehaviourConfig_.acceptanceYaw * msg::RtlBehaviourConfig::kHysteresis;
    publisherTrajTarget_.publish(trajTarget);
    homeTrack_.trajTarget = trajTarget;
    SPDLOG_INFO("[{}] Home target set, with safe target at ({}, {}, {}) [m].", name_,
                trajTarget.position.x(), trajTarget.position.y(), trajTarget.position.z());
}

bool RtlBehaviour::isWithinTargetTolerance(double posErrNorm, double yawErr,
                                           const msg::RtlBehaviourConfig& config) {
    return posErrNorm < config.acceptanceRadius && std::abs(yawErr) < config.acceptanceYaw;
}

bool RtlBehaviour::isOutsideTargetTolerance(double posErrNorm, double yawErr) const {
    return posErrNorm > homeTrack_.radiusOut && std::abs(yawErr) > homeTrack_.yawOut;
}

void RtlBehaviour::update() {
    std::lock_guard lock{mutex_};
    if (latestFlightStatus_.mode != msg::FlightMode::kReturnToLaunch) {
        // Consider reapplying trajectory target
        homeTrack_.targetSet = false;
        return;
    }
    if (homeTrack_.targetSet && homeTrack_.reached) {
        return;
    }
    if (!homeTrack_.targetSet && rtlBehaviourConfig_.homePosition.has_value()) {
        homeTrack_.reset();
        sendHomeTarget(*rtlBehaviourConfig_.homePosition);
        homeTrack_.targetSet = true;
    } else if (!homeTrack_.targetSet) {
        const msg::MissionEvent event{.type = msg::MissionEventType::kHomeReached,
                                      .stamp = internalLastStepUs_};
        publisherMissionEvent_.publish(event);
        return;
    }
    const auto& referenceTarget = homeTrack_.trajTarget;
    const Eigen::Vector3d posError = utils::toEigen(referenceTarget.position - lastState_.position);
    const auto errorNorm = posError.norm();
    const auto yawCur = utils::getYawFromQuaternion(lastState_.orientation.view());
    const auto yawError = ::dronelab::utils::wrapAtPi(referenceTarget.yaw - yawCur);
    homeTrack_.elapsedTime += updateRateSeconds_;
    if (!homeTrack_.insideTolerace &&
        RtlBehaviour::isWithinTargetTolerance(errorNorm, yawError, rtlBehaviourConfig_)) {
        homeTrack_.insideTolerace = true;
        homeTrack_.holdTimer = 0.0;
    } else if (homeTrack_.insideTolerace && isOutsideTargetTolerance(errorNorm, yawError)) {
        homeTrack_.insideTolerace = false;
        homeTrack_.holdTimer = 0.0;
    }

    if (homeTrack_.insideTolerace) {
        homeTrack_.holdTimer += updateRateSeconds_;
        if (homeTrack_.holdTimer >= rtlBehaviourConfig_.holdTime) {
            SPDLOG_INFO(
                "[{}] Drone held position around home position for {} seconds. Sending update.",
                name_, homeTrack_.holdTimer);
            homeTrack_.reached = true;
            const msg::MissionEvent event{.type = msg::MissionEventType::kHomeReached,
                                          .stamp = internalLastStepUs_};
            publisherMissionEvent_.publish(event);
            return;
        }
    } else if (!homeTrack_.timeout && homeTrack_.elapsedTime > homeTrack_.maxTimeBeforeTimeout) {
        SPDLOG_INFO("[{}] Timeout for rtl ({}s elapsed). Sending notification", name_,
                    homeTrack_.elapsedTime);
        homeTrack_.timeout = true;
        const msg::MissionEvent event{.type = msg::MissionEventType::kHomeTimeout,
                                      .stamp = internalLastStepUs_};
        publisherMissionEvent_.publish(event);
    }
}

}  // namespace dronelab::sim