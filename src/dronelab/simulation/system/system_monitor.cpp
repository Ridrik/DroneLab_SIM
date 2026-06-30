#include "dronelab/simulation/system/system_monitor.hpp"

#include <cstdint>
#include <mutex>

#include "dronelab/api/core/sim_report.hpp"
#include "dronelab/api/simulation/system/monitor.hpp"

namespace dronelab::sim {

SystemMonitor::SystemMonitor(ContextView context)
    : Node{context, NodeType::kSystemMonitor},
      missionProgress_{context.getConfig().getMission()},
      serviceRates_{context.getConfig().getRates()},
      monitorConfig_{context.getConfig().getMonitorConfig()},
      updateRateSeconds_{1.0 / static_cast<double>(context.getConfig().getSystemMonitorRate())},
      updateRateUs_{static_cast<std::int32_t>(1000000.0 * updateRateSeconds_)},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberBatteryData_{context, *this, TopicKey::kBatteryStatus},
      subscriberGpsData_{context, *this, TopicKey::kGpsData},
      subscriberImuData_{context, *this, TopicKey::kImuData},
      subscriberMagnetometerData_{context, *this, TopicKey::kMagnetometerData},
      subscriberBarometerData_{context, *this, TopicKey::kBarometerData},
      subscriberActuatorData_{context, *this, TopicKey::kActuatorFeedback},
      subscriberMissionProgress_{context, *this, TopicKey::kMissionProgress},
      publisherMonitorStatus_{context, *this, TopicKey::kMonitorStatus} {
    subscriberSimTime_
        .filter([this](const auto& msg) -> bool {
            return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
        })
        .subscribe([this](const auto& msg) -> void {
            internalLatestStepUs_ = *msg;
            lastStepUs_.store(*msg, std::memory_order_relaxed);
            update();
        });
    subscriberBatteryData_.subscribe(
        [this](const auto& msg) -> void { receivedBatteryData(*msg); });
    subscriberGpsData_.subscribe([this](const auto& msg) -> void { receivedGpsData(*msg); });
    subscriberImuData_.subscribe([this](const auto& msg) -> void { receivedImuData(*msg); });
    subscriberMagnetometerData_.subscribe(
        [this](const auto& msg) -> void { receivedMagData(*msg); });
    subscriberBarometerData_.subscribe(
        [this](const auto& msg) -> void { receivedBarometerData(*msg); });
    subscriberActuatorData_.subscribe(
        [this](const auto& msg) -> void { receivedActuatorData(*msg); });
    subscriberMissionProgress_.subscribe([this](const auto& msg) -> void {
        std::lock_guard lock{mutex_};
        missionProgress_ = *msg;
    });
}

void SystemMonitor::receivedBatteryData(const msg::BatteryData& data) {
    std::lock_guard lock{mutex_};
    latestBatteryData_ = data;
    monitorStatus_.subsystems[+msg::Subsystem::kBattery].lastUpdate = latestBatteryData_.simStamp;
}

void SystemMonitor::receivedGpsData(const msg::GpsData& data) {
    std::lock_guard lock{mutex_};
    latestGpsData_ = data;
    monitorStatus_.subsystems[+msg::Subsystem::kGps].lastUpdate = latestGpsData_.simStamp;
}

void SystemMonitor::receivedImuData(const msg::ImuData& data) {
    std::lock_guard lock{mutex_};
    latestImuData_ = data;
    monitorStatus_.subsystems[+msg::Subsystem::kImu].lastUpdate = latestImuData_.simStamp;
}

void SystemMonitor::receivedMagData(const msg::MagnetometerData& data) {
    std::lock_guard lock{mutex_};
    latestMagData_ = data;
    monitorStatus_.subsystems[+msg::Subsystem::kMag].lastUpdate = latestMagData_.simStamp;
}

void SystemMonitor::receivedBarometerData(const msg::BarometerData& data) {
    std::lock_guard lock{mutex_};
    latestBarData_ = data;
    monitorStatus_.subsystems[+msg::Subsystem::kBarometer].lastUpdate = latestBarData_.simStamp;
}

void SystemMonitor::receivedActuatorData(const msg::ActuatorFeedback& data) {
    std::lock_guard lock{mutex_};
    latestActuatorData_ = data;
    for (std::uint8_t i{0}; i < 4; ++i) {
        monitorStatus_.subsystems[+msg::Subsystem::kActuator1 + i].lastUpdate =
            latestActuatorData_.simStamp;
    }
}

void SystemMonitor::checkBatteryData() {
    monitorStatus_.batterySoc = latestBatteryData_.remainingCapacity;

    auto& report = monitorStatus_.subsystems[+msg::Subsystem::kBattery];
    if (report.lastUpdate < 0) {
        report.health = msg::Health::kUnknown;
        return;
    }
    msg::Health newHealth = msg::Health::kOk;

    if (monitorStatus_.batterySoc < monitorConfig_.batteryEmergencySoc) {
        newHealth = msg::Health::kFailed;
    } else if (monitorStatus_.batterySoc < monitorConfig_.batteryFailsafeSoc) {
        newHealth = msg::Health::kDegraded;
    }

    if (report.health == msg::Health::kOk && newHealth != msg::Health::kOk) {
        ++monitorProgress_.batteryDegraded.occurences;
        monitorProgress_.batteryDegraded.time = 0;
    } else if (newHealth != msg::Health::kOk) {
        monitorProgress_.batteryDegraded.time += updateRateSeconds_;
    }

    report.health = newHealth;
    report.lastUpdate = internalLatestStepUs_;
}

void SystemMonitor::checkGpsData() {
    auto& report = monitorStatus_.subsystems[+msg::Subsystem::kGps];
    if (report.lastUpdate < 0) {
        report.health = msg::Health::kUnknown;
        return;
    }
    const bool isHealthy = (internalLatestStepUs_ - report.lastUpdate) < monitorConfig_.maxGpsAge;

    if (report.health == msg::Health::kOk && !isHealthy) {
        ++monitorProgress_.gpsDegraded.occurences;
    }
    if (!isHealthy) {
        monitorProgress_.gpsDegraded.time += updateRateSeconds_;
    }

    report.health = isHealthy ? msg::Health::kOk : msg::Health::kFailed;
    // report.lastUpdate = internalLatestStepUs_;
}

void SystemMonitor::checkImuData() {
    auto& report = monitorStatus_.subsystems[+msg::Subsystem::kImu];
    if (report.lastUpdate < 0) {
        report.health = msg::Health::kUnknown;
        return;
    }
    const bool isHealthy = (internalLatestStepUs_ - report.lastUpdate) < monitorConfig_.maxImuAge;
    if (report.health == msg::Health::kOk && !isHealthy) {
        ++monitorProgress_.imuDegraded.occurences;
    }
    if (!isHealthy) {
        monitorProgress_.imuDegraded.time += updateRateSeconds_;
    }

    report.health = isHealthy ? msg::Health::kOk : msg::Health::kFailed;
}

void SystemMonitor::checkMagnetometerData() {
    auto& report = monitorStatus_.subsystems[+msg::Subsystem::kMag];
    if (report.lastUpdate < 0) {
        report.health = msg::Health::kUnknown;
        return;
    }
    const bool isHealthy = (internalLatestStepUs_ - report.lastUpdate) < monitorConfig_.maxMagAge;
    if (report.health == msg::Health::kOk && !isHealthy) {
        ++monitorProgress_.imuDegraded.occurences;
    }
    if (!isHealthy) {
        monitorProgress_.imuDegraded.time += updateRateSeconds_;
    }

    report.health = isHealthy ? msg::Health::kOk : msg::Health::kFailed;
}

void SystemMonitor::checkBarometerData() {
    auto& report = monitorStatus_.subsystems[+msg::Subsystem::kBarometer];
    if (report.lastUpdate < 0) {
        report.health = msg::Health::kUnknown;
        return;
    }
    const bool isHealthy =
        (internalLatestStepUs_ - report.lastUpdate) < monitorConfig_.maxBarometerAge;
    if (report.health == msg::Health::kOk && !isHealthy) {
        ++monitorProgress_.barometerDegraded.occurences;
    }
    if (!isHealthy) {
        monitorProgress_.barometerDegraded.time += updateRateSeconds_;
    }

    report.health = isHealthy ? msg::Health::kOk : msg::Health::kFailed;
}

void SystemMonitor::checkActuatorData() {
    for (std::uint8_t i{0}; i < 4; ++i) {
        auto& report = monitorStatus_.subsystems[+msg::Subsystem::kActuator1 + i];
        if (report.lastUpdate < 0) {
            report.health = msg::Health::kUnknown;
            continue;
        }

        const bool isHealthy =
            !latestActuatorData_.physics.escShutdown[i] &&
            (internalLatestStepUs_ - report.lastUpdate) < monitorConfig_.maxActuatorAge;

        if (report.health == msg::Health::kOk && !isHealthy) {
            ++monitorProgress_.actuatorsDegraded[i].occurences;
            monitorProgress_.actuatorsDegraded[i].time = 0;
        } else if (!isHealthy) {
            monitorProgress_.actuatorsDegraded[i].time += updateRateSeconds_;
        }

        report.health = isHealthy ? msg::Health::kOk : msg::Health::kFailed;
    }
}

void SystemMonitor::update() {
    std::lock_guard lock{mutex_};
    checkBatteryData();
    checkGpsData();
    checkImuData();
    checkMagnetometerData();
    checkBarometerData();
    checkActuatorData();

    monitorStatus_.simStamp = internalLatestStepUs_;
    publisherMonitorStatus_.publish(monitorStatus_);
}

void SystemMonitor::receivedSimReport() {
    SimReportable report;
    report.value("module", name_);
    struct ComponentIdentifier {
        std::string_view label;
        const MonitorProgress::ComponentActive* componentPtr;
    };
    const std::array<ComponentIdentifier, 8> componentLabels = {
        {{.label = "gps", .componentPtr = &monitorProgress_.gpsDegraded},
         {.label = "imu", .componentPtr = &monitorProgress_.imuDegraded},
         {.label = "magnetometer", .componentPtr = &monitorProgress_.magDegraded},
         {.label = "barometer", .componentPtr = &monitorProgress_.barometerDegraded},
         {.label = "actuator1", .componentPtr = monitorProgress_.actuatorsDegraded.data()},
         {.label = "actuator2", .componentPtr = &monitorProgress_.actuatorsDegraded[1]},
         {.label = "actuator3", .componentPtr = &monitorProgress_.actuatorsDegraded[2]},
         {.label = "actuator4", .componentPtr = &monitorProgress_.actuatorsDegraded[3]}}};

    for (const auto& component : componentLabels) {
        SimReportable compReport;
        compReport.value("total_time", component.componentPtr->time);
        compReport.value("number_of_switches", component.componentPtr->occurences);
        if (component.componentPtr->occurences == 0) {
            compReport.value("average_time_per_zone", std::string_view{"N/A"});
        } else {
            compReport.value("average_time_per_zone",
                             component.componentPtr->time /
                                 static_cast<double>(component.componentPtr->occurences));
        }
        report.add(component.label, compReport);
    }
    publisherSimReport_->publish(SimReportCmd{
        .type = SimReportCmd::Type::kReport, .node = type_, .report = std::move(report)});
}

}  // namespace dronelab::sim