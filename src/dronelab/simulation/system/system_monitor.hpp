#ifndef DRONELAB_SIMULATION_SYSTEM_SYSTEM_MONITOR_HPP
#define DRONELAB_SIMULATION_SYSTEM_SYSTEM_MONITOR_HPP

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/actuator_data.hpp"
#include "dronelab/api/simulation/dynamics/battery.hpp"
#include "dronelab/api/simulation/management/mission.hpp"
#include "dronelab/api/simulation/sensors/barometer.hpp"
#include "dronelab/api/simulation/sensors/gps.hpp"
#include "dronelab/api/simulation/sensors/imu.hpp"
#include "dronelab/api/simulation/sensors/magnetometer.hpp"
#include "dronelab/api/simulation/system/monitor.hpp"

namespace dronelab::sim {

class SystemMonitor : public Node {
   public:
    explicit SystemMonitor(ContextView context);

   private:
    void checkBatteryData();
    void checkGpsData();
    void checkImuData();
    void checkMagnetometerData();
    void checkBarometerData();
    void checkActuatorData();

    void receivedBatteryData(const msg::BatteryData& data);
    void receivedGpsData(const msg::GpsData& data);
    void receivedImuData(const msg::ImuData& data);
    void receivedMagData(const msg::MagnetometerData& data);
    void receivedBarometerData(const msg::BarometerData& data);
    void receivedActuatorData(const msg::ActuatorFeedback& data);

    void update();
    void receivedSimReport() override;

    struct MonitorProgress {
        MonitorProgress()
            : gpsDegraded{ComponentActive{.time = 0.0, .occurences = 0}},
              imuDegraded{ComponentActive{.time = 0.0, .occurences = 0}},
              magDegraded{ComponentActive{.time = 0.0, .occurences = 0}} {
            std::ranges::fill(actuatorsDegraded, ComponentActive{.time = 0.0, .occurences = 0});
        }
        struct ComponentActive {
            double time{0.0};
            std::size_t occurences{1};
        };
        ComponentActive gpsDegraded;
        ComponentActive imuDegraded;
        ComponentActive barometerDegraded;
        ComponentActive magDegraded;
        ComponentActive batteryDegraded;
        std::array<ComponentActive, 4> actuatorsDegraded;
    };

    msg::BatteryData latestBatteryData_;
    msg::GpsData latestGpsData_;
    msg::ImuData latestImuData_;
    msg::MagnetometerData latestMagData_;
    msg::BarometerData latestBarData_;
    msg::ActuatorFeedback latestActuatorData_;
    msg::MissionProgress missionProgress_;
    msg::ServiceRates serviceRates_;
    msg::MonitorConfig monitorConfig_;
    msg::MonitorStatus monitorStatus_;

    MonitorProgress monitorProgress_;

    const double updateRateSeconds_{};
    const std::int32_t updateRateUs_{};
    std::atomic<msg::Stamp> lastStepUs_{msg::secondsToStamp(-1.0)};
    msg::Stamp internalLatestStepUs_{msg::secondsToStamp(-1.0)};

    Subscriber<msg::Stamp> subscriberSimTime_;
    Subscriber<sim::msg::BatteryData> subscriberBatteryData_;
    Subscriber<sim::msg::GpsData> subscriberGpsData_;
    Subscriber<sim::msg::ImuData> subscriberImuData_;
    Subscriber<sim::msg::MagnetometerData> subscriberMagnetometerData_;
    Subscriber<sim::msg::BarometerData> subscriberBarometerData_;
    Subscriber<sim::msg::ActuatorFeedback> subscriberActuatorData_;
    Subscriber<sim::msg::MissionProgress> subscriberMissionProgress_;
    Publisher<sim::msg::MonitorStatus> publisherMonitorStatus_;

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_SYSTEM_SYSTEM_MONITOR_HPP