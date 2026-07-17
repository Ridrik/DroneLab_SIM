#include "dronelab/simulation/dynamics/battery_system.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <ranges>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/core/sim_report.hpp"
#include "dronelab/api/core/utils/common.hpp"
#include "dronelab/api/simulation/dynamics/battery.hpp"

namespace dronelab::sim {

BatterySystem::BatterySystem(ContextView context)
    : Node{context, NodeType::kBatteryManager},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberActuatorFeedback_{context, *this, TopicKey::kActuatorFeedback},
      publisherBatteryData_{context, *this, TopicKey::kBatteryStatus},
      batteryConfig_{context.getConfig().getBatteryConfig()},
      invCapacitymAh_{1.0 / batteryConfig_.capacitymAh},
      remainingmAh_{batteryConfig_.capacitymAh},
      updateRateSeconds_{1.0 / static_cast<double>(context.getConfig().getBatteryRate())},
      updateRateUs_{static_cast<std::int32_t>(updateRateSeconds_ * 1000000.0)} {
    subscriberActuatorFeedback_.subscribe(
        [this](const auto& data) { receivedActuatorFeedback(*data); });

    subscriberSimTime_
        .filter([this](const auto& msg) -> bool {
            return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
        })
        .subscribe([this](const auto& msg) {
            lastStepUs_.store(*msg, std::memory_order_relaxed);
            stepUpdate();
        });
}

void BatterySystem::receivedActuatorFeedback(const msg::ActuatorFeedback& feedback) {
    std::lock_guard lock{mutex_};
    latestActuatorFeedback_ = feedback;
}

void BatterySystem::stepUpdate() {
    std::lock_guard lock{mutex_};
    auto dt = msg::microsToSeconds<double>(lastStepUs_.load(std::memory_order_relaxed) -
                                           batteryData_.simStamp);
    if (dt <= 0 || dt > 1.0) [[unlikely]] {
        dt = 1e-3;
    }
    // Simple base drain, considering only actuators
    const double totalCurrentA = std::ranges::fold_left(
        latestActuatorFeedback_.physics.motorCurrents, 0.0,
        std::plus{});  // latestActuatorFeedback_.physics.motorCurrents.sum();
    static constexpr double kGain{1.0 / 3.6};
    const double consumedmAh = totalCurrentA * dt * kGain;
    remainingmAh_ -= consumedmAh;
    remainingmAh_ = std::max(remainingmAh_, 0.0);

    const double socFraction = remainingmAh_ * invCapacitymAh_;
    const double dischargePercent = 1.0 - socFraction;
    const double dischargeFactor = 1.0 - 0.1 * dischargePercent;
    const double voltageSag = totalCurrentA * batteryConfig_.internalResistance;
    const double openCircuitVoltage = batteryConfig_.nominalVoltage * dischargeFactor;

    batteryData_.voltage = std::max(openCircuitVoltage - voltageSag, 0.0);
    batteryData_.current = totalCurrentA;
    constexpr double kToPercentage{100.0};
    batteryData_.remainingCapacity = socFraction * kToPercentage;          // %
    batteryData_.powerConsumption = batteryData_.voltage * totalCurrentA;  // W
    totalEnergyConsumed_ += dt * batteryData_.powerConsumption / 3600.0;   // Wh
    minVoltage_ = std::min(minVoltage_, batteryData_.voltage);
    maxVoltage_ = std::max(maxVoltage_, batteryData_.voltage);
    batteryData_.simStamp = lastStepUs_.load(std::memory_order_relaxed);

    publisherBatteryData_.publish(batteryData_);
}

void BatterySystem::receivedSimReport() {
    SimReportable report;
    report.value("module", name_);
    report.value("initial_battery_mAh", batteryConfig_.capacitymAh);
    report.value("remaining_battery_mAh", remainingmAh_);
    report.value("remaining_soc", batteryData_.remainingCapacity);
    report.value("soc_used", 100.0 - batteryData_.remainingCapacity);
    report.value("total_energy_Wh", totalEnergyConsumed_);
    report.value("voltage_min_V", minVoltage_);
    report.value("voltage_max_V", maxVoltage_);
    SimReportCmd cmd{
        .type = SimReportCmd::Type::kReport, .node = type_, .report = std::move(report)};
    publisherSimReport_->publish(std::move(cmd));
}

}  // namespace dronelab::sim