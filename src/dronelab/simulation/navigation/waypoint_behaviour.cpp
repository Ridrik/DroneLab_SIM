#include "dronelab/simulation/navigation/waypoint_behaviour.hpp"

#include <atomic>
#include <cmath>
#include <mutex>

#include <Eigen/src/Geometry/Quaternion.h>
#include <spdlog/spdlog.h>

#include "dronelab/api/core/config.hpp"
#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/core/utils/common.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/management/flight.hpp"
#include "dronelab/api/simulation/management/mission.hpp"
#include "dronelab/api/simulation/navigation/trajectory.hpp"
#include "dronelab/api/simulation/navigation/waypoint.hpp"
#include "dronelab/api/simulation/sensors/estimate.hpp"
#include "dronelab/simulation/utils/eigen_adapter.hpp"
#include "dronelab/simulation/utils/representations.hpp"

namespace dronelab::sim {

WaypointBehaviour::WaypointBehaviour(ContextView context)
    : Node{context, NodeType::kWaypointBehaviour},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberMissionProgress_{context, *this, TopicKey::kMissionProgress},
      subscriberStateEstimate_{context, *this, TopicKey::kStateEstimate},
      subscriberDroneState_{context, *this, TopicKey::kDroneState},
      subscriberFlightStatus_{context, *this, TopicKey::kFlightStatus},
      publisherTrajTarget_{context, *this, TopicKey::kTrajectoryTarget},
      publisherWaypointProgress_{context, *this, TopicKey::kWaypointProgress},
      useRealState_{context.getConfig().useRealState()},
      updateRateSeconds_{1.0 / static_cast<double>(context.getConfig().getWaypointBehaviourRate())},
      updateRateUs_{static_cast<std::int32_t>(updateRateSeconds_ * 1000000.0)} {
    subscriberMissionProgress_.subscribe(
        [this](const auto& msg) -> void { receivedMissionProgress(*msg); });

    if (!useRealState_) {
        subscriberStateEstimate_.subscribe(
            [this](const auto& msg) -> void { receivedStateEstimate(*msg); });
    } else {
        SPDLOG_WARN("[{}] Using drone's true state instead of estimated state.", name_);
        subscriberDroneState_.subscribe(
            [this](const auto& msg) -> void { receivedDroneState(*msg); });
    }

    subscriberFlightStatus_.subscribe(
        [this](const auto& msg) -> void { receivedFlightStatus(*msg); });

    subscriberSimTime_
        .filter([this](const auto& msg) -> bool {
            return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
        })
        .subscribe([this](const auto& msg) -> void {
            internalSimTimeUs_ = *msg;
            lastStepUs_.store(*msg, std::memory_order_relaxed);
            update();
        });
}

void WaypointBehaviour::receivedFlightStatus(const msg::FlightStatus& status) {
    std::lock_guard lock{mutex_};
    latestFlightStatus_ = status;
}

void WaypointBehaviour::receivedMissionProgress(const msg::MissionProgress& mission) {
    std::lock_guard lock{mutex_};
    SPDLOG_DEBUG("[{}] Received new mission progress update", name_);
    FAULT_ASSERT(mission.valid(), "Given invalid mission to behaviours");
    const bool isCompleted = mission.allWaypointsCleared();
    if (isCompleted) {
        missionProgress_ = mission;
        return;
    }
    const auto& updatedWaypoint = mission.getCurrentWaypoint();
    const bool isUpdatedWaypointNew = !currentTrackableWaypoint_.waypointProg.has_value() ||
                                      missionProgress_.allWaypointsCleared() ||
                                      updatedWaypoint != missionProgress_.getCurrentWaypoint();
    missionProgress_ = mission;
    if (!isUpdatedWaypointNew) {
        currentTrackableWaypoint_.waypointProg = std::ref(missionProgress_.getCurrentWaypoint());
        return;
    }
    currentTrackableWaypoint_.reset();
    if (latestFlightStatus_.mode != msg::FlightMode::kWaypointFollow) {
        // waypoint track will be referenced once allowed to execute on update
        return;
    }
    SPDLOG_DEBUG("[{}] Received new waypoint target, publishing it", name_);
    triggerNewTrajectoryTarget(missionProgress_.getCurrentWaypoint());
    currentTrackableWaypoint_.waypointProg = std::ref(missionProgress_.getCurrentWaypoint());
}

void WaypointBehaviour::receivedStateEstimate(const msg::StateEstimate& state) {
    std::lock_guard lock{mutex_};
    lastState_ = state;
    if (!initialized_) {
        initialized_ = true;
        SPDLOG_DEBUG("[{}] Initializing way point deployment", name_);
    }
}

void WaypointBehaviour::receivedDroneState(const msg::DroneState& state) {
    std::lock_guard lock{mutex_};
    SPDLOG_TRACE("[{}] Received state estimate. Pos [m]: [{}, {}, {}]", name_, state.position.x(),
                 state.position.y(), state.position.z());
    lastState_.position = state.position;
    lastState_.velocity = state.velocity;
    lastState_.orientation = state.orientation;
    lastState_.angularVelocity = state.angularVelocity;
    lastState_.gyroBias = {};
    lastState_.accelBias = {};
    lastState_.simStamp = state.simStamp;
    if (!initialized_) {
        initialized_ = true;
        SPDLOG_DEBUG("[{}] Initializing way point deployment", name_);
    }
}

double WaypointBehaviour::computeConvergenceTime(const Eigen::Vector3d& relWpPos, double yawCur,
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

bool WaypointBehaviour::isWithinTargetTolerance(double posErrNorm, double yawErr,
                                                const msg::Waypoint& wp) {
    return posErrNorm < wp.acceptanceRadius && std::abs(yawErr) < wp.acceptanceYaw;
}

bool WaypointBehaviour::isOutsideTargetTolerance(double posErrNorm, double yawErr,
                                                 const msg::WaypointProgress& wpProg) {
    return posErrNorm > wpProg.radiusOut || std::abs(yawErr) > wpProg.yawOut;
}

void WaypointBehaviour::triggerNewTrajectoryTarget(const msg::WaypointProgress& referenceWaypoint) {
    SPDLOG_INFO("[{}] Publishing trajectory target from waypoint id {}", name_,
                referenceWaypoint.waypoint.id);
    msg::TrajectoryTarget trajTarget;
    const double yaw = utils::getYawFromQuaternion(lastState_.orientation.view());
    trajTarget.convergenceTime = computeConvergenceTime(
        utils::toEigen(referenceWaypoint.waypoint.position - lastState_.position), yaw,
        referenceWaypoint.waypoint.yaw, missionProgress_.params.nominalSpeed,
        missionProgress_.params.minimumTargetTime, missionProgress_.params.maximumTargetTime);
    trajTarget.position = referenceWaypoint.waypoint.position;
    trajTarget.yaw = referenceWaypoint.waypoint.yaw;
    trajTarget.flightMode = msg::FlightMode::kWaypointFollow;
    trajTarget.simStamp = internalSimTimeUs_;
    currentTrackableWaypoint_.trajTarget = trajTarget;
    currentTrackableWaypoint_.maxTimeBeforeTimeout =
        trajTarget.convergenceTime * (1.0 + missionProgress_.params.clearanceTimeThreshold);
    publisherTrajTarget_.publish(trajTarget);
}

void WaypointBehaviour::update() {
    std::lock_guard lock{mutex_};
    const bool canOperate = initialized_ &&
                            latestFlightStatus_.mode == msg::FlightMode::kWaypointFollow &&
                            missionProgress_.canContinue();
    if (!canOperate) {
        SPDLOG_TRACE("[{}] Cannot operate mission. Exiting update.", name_);
        return;
    }
    if (currentTrackableWaypoint_.reached) {
        SPDLOG_TRACE("[{}] Target reached. Exiting update.", name_);
        return;
    }
    const bool isWaypointBeingTracked = currentTrackableWaypoint_.waypointProg.has_value();
    if (!isWaypointBeingTracked) {
        currentTrackableWaypoint_.reset();
        currentTrackableWaypoint_.waypointProg = std::ref(missionProgress_.getCurrentWaypoint());
        triggerNewTrajectoryTarget(currentTrackableWaypoint_.waypointProg.value());
        return;
    }
    auto& referenceWaypointProgress = currentTrackableWaypoint_.waypointProg->get();
    const auto& referenceWaypoint = referenceWaypointProgress.waypoint;
    const Eigen::Vector3d posError =
        utils::toEigen(referenceWaypoint.position - lastState_.position);
    const auto errorNorm = posError.norm();
    const auto yawCur = utils::getYawFromQuaternion(lastState_.orientation.view());
    const auto yawError = ::dronelab::utils::wrapAtPi(referenceWaypoint.yaw - yawCur);
    currentTrackableWaypoint_.elapsedTime += updateRateSeconds_;

    if (!currentTrackableWaypoint_.insideTolerace &&
        WaypointBehaviour::isWithinTargetTolerance(errorNorm, yawError, referenceWaypoint)) {
        currentTrackableWaypoint_.insideTolerace = true;
        ++referenceWaypointProgress.tryCount;
        if (!referenceWaypointProgress.timeFirstIn.has_value()) {
            referenceWaypointProgress.timeFirstIn = msg::stampToSeconds<double>(internalSimTimeUs_);
            referenceWaypointProgress.arrivalSpeed = utils::toEigen(lastState_.velocity).norm();
        }
        currentTrackableWaypoint_.holdTimer = 0.0;
    } else if (currentTrackableWaypoint_.insideTolerace &&
               WaypointBehaviour::isOutsideTargetTolerance(errorNorm, yawError,
                                                           referenceWaypointProgress)) {
        currentTrackableWaypoint_.insideTolerace = false;
        currentTrackableWaypoint_.holdTimer = 0.0;
    }
    if (referenceWaypointProgress.timeFirstIn.has_value()) {
        const double overShoot = std::max(0.0, errorNorm - referenceWaypoint.acceptanceRadius);
        if (!referenceWaypointProgress.overShoot ||
            overShoot > referenceWaypointProgress.overShoot.value()) {
            referenceWaypointProgress.overShoot = overShoot;
        }
    }

    if (currentTrackableWaypoint_.insideTolerace) {
        currentTrackableWaypoint_.holdTimer += updateRateSeconds_;
        const bool holdTimeNotApplicable = !referenceWaypoint.holdTime.has_value();
        if (holdTimeNotApplicable ||
            currentTrackableWaypoint_.holdTimer >= referenceWaypoint.holdTime) {
            SPDLOG_INFO("[{}] Drone held position around waypoint for {} seconds. Sending update.",
                        name_, currentTrackableWaypoint_.holdTimer);
            currentTrackableWaypoint_.reached = true;
            referenceWaypointProgress.status = msg::WaypointStatus::kReached;
            referenceWaypointProgress.timeReached =
                msg::microsToSeconds<double>(internalSimTimeUs_);
            referenceWaypointProgress.deltaTime = referenceWaypointProgress.timeReached.value() -
                                                  referenceWaypointProgress.timeAssigned.value();
            publisherWaypointProgress_.publish(referenceWaypointProgress);
            return;
        }
    } else if (!currentTrackableWaypoint_.timeout &&
               currentTrackableWaypoint_.elapsedTime >
                   currentTrackableWaypoint_.maxTimeBeforeTimeout) {
        // Let timeout be considered when it is outside the acceptable range (to account for
        // variable hold times)
        SPDLOG_INFO("[{}] Timeout for waypoint id {}. Sending notification", name_,
                    referenceWaypoint.id);
        currentTrackableWaypoint_.timeout = true;
        referenceWaypointProgress.status = msg::WaypointStatus::kTimeout;
        publisherWaypointProgress_.publish(referenceWaypointProgress);
    }
}

}  // namespace dronelab::sim