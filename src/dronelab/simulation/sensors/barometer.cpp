#include "dronelab/simulation/sensors/barometer.hpp"

#include <mutex>
#include <type_traits>
#include <variant>

#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/simulation/common/commands.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/simulation/utils/noise_generator.hpp"

namespace dronelab::sim {

Barometer::Barometer(ContextView context)
    : Node{context, NodeType::kBarometer},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberDroneState_{context, *this, TopicKey::kDroneState},
      subscriberSimCmd_{context, *this, TopicKey::kSimCmd},
      publisherBarometerData_{context, *this, TopicKey::kBarometerData},
      barParams_{context.getConfig().getBarometerParams()},
      noiseGenerator_{barParams_.noiseStddev,
                      context.getConfig().getSeed() + static_cast<std::size_t>(kSensorId) * 10},
      biasNoiseGenerator_{
          barParams_.biasDriftStddev,
          context.getConfig().getSeed() + static_cast<std::size_t>(kSensorId) * 10 + 1},
      activated_{context.getConfig().getSensorConfig().barometerEnabled},
      updateRateUs_{static_cast<std::int32_t>(
          1000000.0 / static_cast<double>(context.getConfig().getBarometerRate()))} {
    subscriberSimCmd_
        .filter([](const auto& msg) -> bool {
            return std::holds_alternative<sim::msg::SensorUpdateCmd>(msg->cmd);
        })
        .subscribe(
            [](const auto& msg, auto* self) -> void {
                self->processSimCmd(std::get<sim::msg::SensorUpdateCmd>(msg->cmd));
            },
            this);

    bool activated = activated_;
    activate();
    if (!activated) {
        deactivate();
        SPDLOG_INFO("[{}] Sensor initialized status -> deactivated.", name_);
    }
}

void Barometer::activate() {
    SPDLOG_INFO("[{}] Activating Sensor", name_);
    subscriberDroneState_.subscribe([this](const auto& msg) -> void { receivedDroneState(*msg); });

    subscriberSimTime_
        .filter([this](const auto& msg) -> bool {
            return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
        })
        .subscribe([this](const auto& msg) -> void {
            lastStepUs_.store(*msg, std::memory_order_relaxed);
            internalCurStepUs_ = *msg;
            updateData();
        });

    activated_ = true;
}

void Barometer::deactivate() {
    SPDLOG_INFO("[{}] Deactivating Sensor", name_);
    initFiltering_ = true;
    subscriberDroneState_.unsubscribe();
    subscriberSimTime_.unsubscribe();

    activated_ = false;
}

void Barometer::processSimCmd(const msg::SensorUpdateCmd& cmd) {
    std::lock_guard lock{mutex_};
    std::visit(
        [&](auto&& subcmd) {
            using T = std::decay_t<decltype(subcmd)>;
            if constexpr (std::is_same_v<T, msg::EnableOrDisableSensor>) {
                if (subcmd.target == msg::SensorID::kBarometer) {
                    if (subcmd.value && !activated_) {
                        activate();
                    } else if (!subcmd.value && activated_) {
                        deactivate();
                    }
                }
            }
        },
        cmd.value);
}

void Barometer::receivedDroneState(const msg::DroneState& state) {
    std::lock_guard lock{mutex_};
    lastState_ = state;
    initialized_ = true;
}

void Barometer::updateData() {
    std::lock_guard lock{mutex_};
    if (!initialized_) {
        return;
    }

    auto dt = msg::microsToSeconds<double>(internalCurStepUs_ - barData_.simStamp);
    if (dt <= 0 || dt > 1.0) [[unlikely]] {
        SPDLOG_DEBUG("[{}] Abnormal dt = {}, setting to 1e-3", name_, dt);
        initFiltering_ = true;
    }

    static constexpr auto kQuantize = [](double value, double resolutionPa) {
        return std::round(value / resolutionPa) * resolutionPa;
    };

    const auto dtSqrt = std::sqrt(dt);

    const double referenceHeight = -lastState_.position.z();
    const double referencePressure =
        barParams_.seaLevelPresure * std::pow(1.0 - (referenceHeight / 44330.0), 5.255);
    const double sampleNoise = noiseGenerator_.sample();

    double measured = referencePressure + currentBias_ + sampleNoise;
    if (barParams_.enableFiltering) {
        const double alpha = dt / (barParams_.filterTimeConstant + dt);
        if (initFiltering_) {
            filteredMeasurement_ = measured;
            initFiltering_ = false;
        }
        filteredMeasurement_ += alpha * (measured - filteredMeasurement_);
        measured = filteredMeasurement_;
    }
    barData_.pressureMeasure = kQuantize(measured, barParams_.resolution);
    barData_.simStamp = internalCurStepUs_;

    currentBias_ += dtSqrt * biasNoiseGenerator_.sample();

    publisherBarometerData_.publish(barData_);
}

}  // namespace dronelab::sim