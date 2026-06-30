#include "dronelab/simulation/navigation/loiter_behaviour.hpp"

#include <spdlog/spdlog.h>

#include "dronelab/api/core/node.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/management/flight.hpp"
#include "dronelab/api/simulation/navigation/trajectory.hpp"
#include "dronelab/api/simulation/sensors/estimate.hpp"
#include "dronelab/simulation/utils/representations.hpp"

namespace dronelab::sim {

LoiterBehaviour::LoiterBehaviour(ContextView context)
    : Node{context, NodeType::kLoiterBehaviour},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberFlightStatus_{context, *this, TopicKey::kFlightStatus},
      subscriberStateEstimate_{context, *this, TopicKey::kStateEstimate},
      subscriberDroneState_{context, *this, TopicKey::kDroneState},
      publisherSetpoint_{context, *this, TopicKey::kSetPoint},
      publisherMissionEvent_{context, *this, TopicKey::kMissionEvent},
      updateRateSeconds_{1.0 / static_cast<double>(context.getConfig().getWaypointBehaviourRate())},
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

void LoiterBehaviour::receivedStateEstimate(const msg::StateEstimate& state) {
    std::lock_guard lock{mutex_};
    lastState_ = state;
}

void LoiterBehaviour::receivedDroneState(const msg::DroneState& state) {
    std::lock_guard lock{mutex_};
    lastState_.position = state.position;
    lastState_.velocity = state.velocity;
    lastState_.orientation = state.orientation;
    lastState_.angularVelocity = state.angularVelocity;
    lastState_.gyroBias = {};
    lastState_.accelBias = {};
    lastState_.simStamp = state.simStamp;
}

void LoiterBehaviour::receivedFlightStatus(const msg::FlightStatus& status) {
    std::lock_guard lock{mutex_};
    latestFlightStatus_ = status;
}

void LoiterBehaviour::makeNewLoiterTarget() {
    lastSetpoint_.position = lastState_.position;
    lastSetpoint_.velocity = {};
    lastSetpoint_.acceleration = {};
    lastSetpoint_.yaw = utils::getYawFromQuaternion(std::span{lastState_.orientation.data(), 4});
    SPDLOG_INFO("[{}] Generating new loiter target: Pos ({}, {}, {}) [m], yaw {} [deg]", name_,
                lastSetpoint_.position.x(), lastSetpoint_.position.y(), lastSetpoint_.position.z(),
                ::dronelab::utils::radToDeg(lastSetpoint_.yaw));
}

void LoiterBehaviour::update() {
    {
        std::lock_guard lock{mutex_};
        if (latestFlightStatus_.mode != msg::FlightMode::kLoiter) {
            createLoiterTarget_ = true;
            return;
        }
        if (createLoiterTarget_) {
            makeNewLoiterTarget();
            createLoiterTarget_ = false;
        }
    }
    lastSetpoint_.simStamp = internalLastStepUs_;
    SPDLOG_TRACE("[{}] Publishing setpoint, stamp {}", name_, lastSetpoint_.simStamp);
    publisherSetpoint_.publish(lastSetpoint_);
}

}  // namespace dronelab::sim