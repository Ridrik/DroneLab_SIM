#include "dronelab/simulation/management/flight_manager.hpp"

#include <algorithm>
#include <atomic>
#include <ranges>

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/sim_report.hpp"
#include "dronelab/api/simulation/common/simulation.hpp"
#include "dronelab/api/simulation/management/flight.hpp"
#include "dronelab/api/simulation/management/mission.hpp"
#include "dronelab/api/simulation/navigation/waypoint.hpp"
#include "dronelab/api/simulation/system/monitor.hpp"
#include "dronelab/api/simulation/system/status.hpp"

namespace dronelab::sim {

FlightManager::FlightManager(ContextView context)
    : Node{context, NodeType::kFlightManager},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberMonitorStatus_{context, *this, TopicKey::kMonitorStatus},
      subscriberDroneState_{context, *this, TopicKey::kDroneState},
      subscriberMissionEvents_{context, *this, TopicKey::kMissionEvent},
      subscriberWaypointProgress_{context, *this, TopicKey::kWaypointProgress},
      subscriberWaypoint_{context, *this, TopicKey::kWayPoint},
      publisherFlightStatus_{context, *this, TopicKey::kFlightStatus},
      publisherMissionProgress_{context, *this, TopicKey::kMissionProgress},
      publisherMissionStatus_{context, *this, TopicKey::kMissionStatus},
      publisherNodeRequest_{context, *this, TopicKey::kNodeRequest},
      updateRateSeconds_{1.0 / static_cast<double>(context.getConfig().getFlightManagerRate())},
      updateRateUs_{static_cast<std::int32_t>(1000000.0 * updateRateSeconds_)},
      config_{context.getConfig().getFlightManagerConfig()},
      hasRtl_{context.getConfig().getMission().rtlBehaviourConfig.homePosition.has_value()},
      mission_{context.getConfig().getMission()},
      missionProgress_{mission_} {
    if (!mission_.valid()) {
        missionProgress_.status = msg::MissionStatus::kInvalid;
        publisherMissionProgress_.publish(missionProgress_);
        publisherMissionStatus_.publish(missionProgress_.status);
        publisherNodeRequest_.publish(msg::NodeRequest::kStop);
        return;
    }

    subscriberWaypoint_.subscribe([this](const auto& msg) -> void { receivedWaypoint(*msg); });
    subscriberMonitorStatus_.subscribe(
        [this](const auto& msg) -> void { processMonitorStatus(*msg); });
    subscriberMissionEvents_.subscribe([this](const auto& msg) -> void {
        std::lock_guard lock{mutex_};
        optMissionEvent_ = *msg;
    });
    subscriberWaypointProgress_.subscribe([this](const auto& msg) -> void {
        std::lock_guard lock{mutex_};
        optWaypointUpdate_ = *msg;
    });
    subscriberSimTime_
        .filter([this](const auto& msg) -> bool {
            return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
        })
        .subscribe([this](const auto& msg) -> void {
            internalLatestStepUs_ = *msg;
            lastStepUs_.store(*msg, std::memory_order_relaxed);
            update();
        });

    std::ranges::sort(config_.eventRules,
                      [](const msg::EventRule& a, const msg::EventRule& b) -> bool {
                          return a.priority > b.priority;
                      });
    std::ranges::sort(config_.rules,
                      [](const msg::PolicyRule& a, const msg::PolicyRule& b) -> bool {
                          return a.priority > b.priority;
                      });
}

void FlightManager::receivedWaypoint(const msg::Waypoint& waypoint) {
    std::lock_guard lock{mutex_};
    SPDLOG_TRACE("[{}] Received new waypoint. Updating mission", name_);
    auto waypointCopy = waypoint;
    if (!mission_.valid()) {
        waypointCopy.id = 0;
    } else {
        const auto lastWp = mission_.waypoints.back();
        waypointCopy.id = lastWp.id + 1;
    }
    mission_.addWaypoint(waypointCopy);
    missionProgress_.addWaypoint(waypointCopy);
    publisherMissionProgress_.publish(missionProgress_);
}

void FlightManager::processMonitorStatus(const msg::MonitorStatus& status) {
    std::lock_guard lock{mutex_};
    lastMonitorStatus_ = status;
}

void FlightManager::checkAndUpdateMission() {
    auto& currentWaypointProgress = missionProgress_.getCurrentWaypoint();
    missionProgress_.simStamp = internalLatestStepUs_;
    publisherMissionProgress_.publish(missionProgress_);
    auto newId = currentWaypointProgress.waypoint.id + 1;
    ++missionProgress_.currentIndex;
    const bool allWaypointsCleared = missionProgress_.allWaypointsCleared();
    if (allWaypointsCleared) {
        missionProgress_.status = msg::MissionStatus::kSuccess;
        publisherMissionStatus_.publish(missionProgress_.status);
        missionProgress_.milestones.push_back(msg::FlightMilestone::kWaypointsCompleted);
        publisherMissionProgress_.publish(missionProgress_);
        return;
    }

    auto& targetWaypointProgress = missionProgress_.getCurrentWaypoint();
    targetWaypointProgress.waypoint.id = newId;
    targetWaypointProgress.status = msg::WaypointStatus::kActive;
    targetWaypointProgress.timeAssigned = msg::microsToSeconds<double>(internalLatestStepUs_);
    missionProgress_.simStamp = internalLatestStepUs_;
    publisherMissionProgress_.publish(missionProgress_);
}

void FlightManager::processWaypointProgress(const msg::WaypointProgress& wpProg) {
    FAULT_ASSERT(wpProg.status == msg::WaypointStatus::kReached ||
                     wpProg.status == msg::WaypointStatus::kTimeout,
                 "[{}] Received unexpected waypoint progress status {}", name_,
                 msg::waypointStatusToString(wpProg.status));
    auto& currentWpProg = missionProgress_.getCurrentWaypoint();
    FAULT_ASSERT(currentWpProg.waypoint.id == wpProg.waypoint.id,
                 "[{}] Expecting current waypoint id {}, received {}", name_,
                 currentWpProg.waypoint.id, wpProg.waypoint.id);
    currentWpProg = wpProg;
    if (wpProg.status == msg::WaypointStatus::kReached) {
        SPDLOG_INFO("[{}] Waypoint id {} reached", name_, wpProg.waypoint.id);
        ++missionProgress_.waypointsReached;
    } else {
        missionProgress_.milestones.push_back(msg::FlightMilestone::kWaypointTimeout);
        SPDLOG_INFO("[{}] Skipping to next waypoint due to timeout.", name_);
        ++missionProgress_.waypointsTimeoutOrSkipped;
    }
    checkAndUpdateMission();
}

void FlightManager::processMissionEvent(const msg::MissionEvent& event) {
    SPDLOG_TRACE("[{}] processing new mission event", name_);
    switch (event.type) {
        case msg::MissionEventType::kHomeReached: {
            missionProgress_.milestones.push_back(msg::FlightMilestone::kHomeReached);
            break;
        }
        case msg::MissionEventType::kHomeTimeout: {
            missionProgress_.milestones.push_back(msg::FlightMilestone::kHomeTimeout);
            break;
        }
        case msg::MissionEventType::kLanded:
            missionProgress_.milestones.push_back(msg::FlightMilestone::kLandReached);
            break;
    }
    publisherMissionProgress_.publish(missionProgress_);
}

void FlightManager::initPublish() {
    flightStatus_.mode = msg::FlightMode::kLoiter;
    flightStatus_.status = msg::SystemStatus::kInitializing;
    flightStatus_.flightTime = 0.0;
    flightStatus_.simStamp = 0;

    // Publish initial FlightStatus
    publisherFlightStatus_.publish(flightStatus_);
}

void FlightManager::startMission() {
    auto& wpProg = missionProgress_.getCurrentWaypoint();
    wpProg.waypoint.id = 0;
    wpProg.timeAssigned = 0.0;
    missionProgress_.status = msg::MissionStatus::kInProgress;
    publisherMissionProgress_.publish(missionProgress_);
    publisherMissionStatus_.publish(msg::MissionStatus::kInProgress);
}

void FlightManager::determineFinalMissionStatus(const msg::EventRule& rule,
                                                const msg::MonitorStatus& /*monitorStatus*/) {
    // TODO: Review and Complete
    if (rule.trigger == msg::FlightMilestone::kWaypointsCompleted) {
        missionProgress_.status = msg::MissionStatus::kSuccess;
        missionProgress_.statusMsg = "Waypoints Completed";
    } else if (rule.trigger == msg::FlightMilestone::kHomeReached ||
               rule.trigger == msg::FlightMilestone::kLandReached) {
        missionProgress_.status = msg::MissionStatus::kSuccess;
        missionProgress_.statusMsg = "Mission completed";
    } else if (rule.trigger == msg::FlightMilestone::kHomeTimeout ||
               rule.trigger == msg::FlightMilestone::kWaypointTimeout) {
        missionProgress_.status = msg::MissionStatus::kFailed;
        missionProgress_.statusMsg = "Timeout occured";
    }
}

void FlightManager::update() {
    std::lock_guard lock{mutex_};
    if (!init_) {
        initPublish();
        init_ = true;
    }
    if (optMissionEvent_.has_value()) {
        processMissionEvent(optMissionEvent_.value());
        optMissionEvent_ = std::nullopt;
    }
    if (optWaypointUpdate_.has_value()) {
        processWaypointProgress(optWaypointUpdate_.value());
        optWaypointUpdate_ = std::nullopt;
    }
    if (flightStatus_.status == msg::SystemStatus::kInitializing) {
        if (!canInitialize()) {
            flightStatus_.simStamp = internalLatestStepUs_;
            flightStatus_.flightTime = msg::microsToSeconds<double>(internalLatestStepUs_);
            publisherFlightStatus_.publish(flightStatus_);
            return;
        }
        missionProgress_.milestones.push_back(msg::FlightMilestone::kSystemInitialized);
        publisherMissionProgress_.publish(missionProgress_);
        flightStatus_.status = msg::SystemStatus::kActive;
        flightStatus_.mode = msg::FlightMode::kWaypointFollow;
    }
    const auto [newSysStatus, newMode] = evaluateRulesAndStatus(
        missionProgress_.milestones, lastMonitorStatus_, flightStatus_.status, flightStatus_.mode);
    if (newMode != flightStatus_.mode) {
        SPDLOG_INFO("[{}] Flight mode change: {} -> {}", name_,
                    msg::flightModeToString(flightStatus_.mode), msg::flightModeToString(newMode));
        flightStatus_.mode = newMode;
    }
    if (newSysStatus != flightStatus_.status) {
        SPDLOG_INFO("[{}] Flight system status change: {} -> {}", name_,
                    msg::systemStatusToString(flightStatus_.status),
                    msg::systemStatusToString(newSysStatus));
        flightStatus_.status = newSysStatus;
    }
    if (newMode == msg::FlightMode::kWaypointFollow &&
        missionProgress_.status == msg::MissionStatus::kUninit) {
        startMission();
    }
    if (newMode == msg::FlightMode::kReturnToLaunch && !missionProgress_.onRtl) {
        missionProgress_.onRtl = true;
        missionProgress_.simStamp = internalLatestStepUs_;
        publisherMissionProgress_.publish(missionProgress_);
    }

    flightStatus_.simStamp = internalLatestStepUs_;
    flightStatus_.flightTime = msg::microsToSeconds<double>(internalLatestStepUs_);
    publisherFlightStatus_.publish(flightStatus_);

    if (sendSimulationTerminationRequest_) {
        publisherMissionProgress_.publish(missionProgress_);
        publisherNodeRequest_.publish(msg::NodeRequest::kStop);
    }
}

bool FlightManager::canInitialize() const {
    bool allRequiredHealthy{true};

    allRequiredHealthy &=
        lastMonitorStatus_.subsystems[+msg::Subsystem::kImu].health == msg::Health::kOk;
    allRequiredHealthy &=
        !config_.gpsRequired ||
        lastMonitorStatus_.subsystems[+msg::Subsystem::kGps].health == msg::Health::kOk;
    allRequiredHealthy &=
        !config_.magRequired ||
        lastMonitorStatus_.subsystems[+msg::Subsystem::kMag].health == msg::Health::kOk;
    for (std::uint8_t i{0}; i < 4; ++i) {
        allRequiredHealthy &=
            lastMonitorStatus_.subsystems[+msg::Subsystem::kActuator1 + i].health ==
            msg::Health::kOk;
    }
    return allRequiredHealthy;
}

std::pair<sim::msg::SystemStatus, msg::FlightMode> FlightManager::evaluateRulesAndStatus(
    const std::vector<sim::msg::FlightMilestone>& milestones,
    const msg::MonitorStatus& monitorStatus, msg::SystemStatus refStatus, msg::FlightMode refMode) {
    auto evaluateStatus = refStatus;
    auto evaluateMode = refMode;

    FAULT_ASSERT(
        std::ranges::is_sorted(
            config_.rules, [](const auto& a, const auto& b) { return a.priority > b.priority; }),
        "Subsystem rules found not to be sorted by priority!");

    FAULT_ASSERT(std::ranges::is_sorted(
                     config_.eventRules,
                     [](const auto& a, const auto& b) { return a.priority > b.priority; }),
                 "Event rules found not to be sorted by priority!");

    // Health rules and Event rules are pre-sorted by priority (highest first).
    // First applicable rule wins. Missing fields mean "leave unchanged".

    // Health safety comes first
    if (auto it = std::ranges::find_if(config_.rules,
                                       [&](const msg::PolicyRule& rule) {
                                           const auto& report =
                                               monitorStatus.subsystems[+rule.subsystem];
                                           return report.health == rule.trigger;
                                       });
        it != config_.rules.end()) {
        if (it->systemStatusChange) {
            evaluateStatus = *it->systemStatusChange;
        }
        if (it->flightModeChange) {
            evaluateMode = *it->flightModeChange;
        }
        return {evaluateStatus, evaluateMode};
    }

    for (auto milestone : milestones | std::views::reverse) {
        for (const auto& eventRule : config_.eventRules) {
            if (eventRule.trigger != milestone) {
                continue;
            }
            if (eventRule.terminateSimulation) {
                sendSimulationTerminationRequest_ = true;
                missionProgress_.terminationEvent = eventRule;
                determineFinalMissionStatus(eventRule, monitorStatus);
                return {evaluateStatus, evaluateMode};
            }
            if (eventRule.flightModeChange) {
                if (*eventRule.flightModeChange == msg::FlightMode::kReturnToLaunch && !hasRtl_) {
                    continue;
                }
                evaluateMode = *eventRule.flightModeChange;
            }
            if (eventRule.systemStatusChange) {
                evaluateStatus = *eventRule.systemStatusChange;
            }
            return {evaluateStatus, evaluateMode};
        }
    }
    return {evaluateStatus, evaluateMode};
}

namespace {

void writeOptionalToJson(SimReportable& report, std::string_view field,
                         const std::optional<double> optValue) {
    if (!optValue.has_value()) {
        return;
    }
    report.value(field, optValue.value());
}

}  // namespace

void FlightManager::receivedSimReport() {
    SimReportable report;
    report.value("module", name_);
    report.value("status", msg::missionStatusToString(missionProgress_.status));
    report.value("total_waypoints", missionProgress_.waypoints.size());
    report.value("waypoints_reached", missionProgress_.waypointsReached);
    report.value("waypoints_timeout", missionProgress_.waypointsTimeoutOrSkipped);
    SimReportable wpAllReport;
    for (const auto& wpProg : missionProgress_.waypoints) {
        SimReportable wpReport;
        const auto& wp = wpProg.waypoint;
        wpReport.value("id", wp.id);
        wpReport.value("position", {wp.position.x(), wp.position.y(), wp.position.z()});
        wpReport.value("yaw", wp.yaw);
        wpReport.value("status", msg::waypointStatusToString(wpProg.status));
        wpReport.value("tolerance", wp.acceptanceRadius);
        wpReport.value("yaw_tolerance", wp.acceptanceYaw);
        wpReport.value("radius_out", wpProg.radiusOut);
        wpReport.value("yaw_out", wpProg.yawOut);
        writeOptionalToJson(wpReport, "hold_time", wp.holdTime);
        writeOptionalToJson(wpReport, "time_assigned", wpProg.timeAssigned);
        writeOptionalToJson(wpReport, "time_reached", wpProg.timeReached);
        writeOptionalToJson(wpReport, "delta_time", wpProg.deltaTime);
        writeOptionalToJson(wpReport, "1st_time_in", wpProg.timeFirstIn);
        writeOptionalToJson(wpReport, "arrival_speed", wpProg.arrivalSpeed);
        writeOptionalToJson(wpReport, "overshoot", wpProg.overShoot);
        wpReport.value("try_count", wpProg.tryCount);
        wpAllReport.pushBack(wpReport);
    }
    report.add("waypoints", wpAllReport);

    SimReportCmd cmd{
        .type = SimReportCmd::Type::kReport, .node = type_, .report = std::move(report)};
    publisherSimReport_->publish(std::move(cmd));
}

}  // namespace dronelab::sim