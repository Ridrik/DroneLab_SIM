#ifndef DRONELAB_SIMULATION_MANAGEMENT_FLIGHT_MANAGER_HPP
#define DRONELAB_SIMULATION_MANAGEMENT_FLIGHT_MANAGER_HPP

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/simulation.hpp"
#include "dronelab/api/simulation/management/flight.hpp"
#include "dronelab/api/simulation/management/mission.hpp"
#include "dronelab/api/simulation/navigation/waypoint.hpp"
#include "dronelab/api/simulation/system/monitor.hpp"
#include "dronelab/api/simulation/system/status.hpp"

namespace dronelab::sim {

class FlightManager : public Node {
   public:
    explicit FlightManager(ContextView context);

   private:
    void update();

    [[nodiscard]] bool canInitialize() const;
    [[nodiscard]] std::pair<msg::SystemStatus, msg::FlightMode> evaluateRulesAndStatus(
        const std::vector<msg::FlightMilestone>& milestones,
        const msg::MonitorStatus& monitorStatus, msg::SystemStatus refStatus,
        msg::FlightMode refMode);
    void processMonitorStatus(const msg::MonitorStatus& status);
    void processMissionEvent(const msg::MissionEvent& event);
    void processWaypointProgress(const msg::WaypointProgress& wpProg);
    void checkAndUpdateMission();
    void receivedWaypoint(const msg::Waypoint& waypoint);
    void receivedSimReport() override;
    void initPublish();
    void startMission();
    void determineFinalMissionStatus(const msg::EventRule& rule,
                                     const msg::MonitorStatus& monitorStatus);

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<msg::MonitorStatus> subscriberMonitorStatus_;
    Subscriber<msg::DroneState> subscriberDroneState_;
    Subscriber<msg::MissionEvent> subscriberMissionEvents_;
    Subscriber<msg::WaypointProgress> subscriberWaypointProgress_;
    Subscriber<msg::Waypoint> subscriberWaypoint_;
    Publisher<msg::FlightStatus> publisherFlightStatus_;
    Publisher<msg::MissionProgress> publisherMissionProgress_;
    Publisher<msg::MissionStatus> publisherMissionStatus_;
    Publisher<msg::NodeRequest> publisherNodeRequest_;

    const double updateRateSeconds_;
    const std::int32_t updateRateUs_;
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};
    msg::Stamp internalLatestStepUs_{-msg::secondsToStamp(1.0)};

    msg::FlightManagerConfig config_;
    bool hasRtl_;
    bool init_{false};
    bool sendSimulationTerminationRequest_{false};
    msg::FlightStatus flightStatus_;
    std::optional<sim::msg::WaypointProgress> optWaypointUpdate_{std::nullopt};
    std::optional<sim::msg::MissionEvent> optMissionEvent_{std::nullopt};
    msg::MonitorStatus lastMonitorStatus_;
    msg::DroneState droneState_;
    msg::Mission mission_;
    msg::MissionProgress missionProgress_;

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_MANAGEMENT_FLIGHT_MANAGER_HPP