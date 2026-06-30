#include "dronelab/simulation/sensors/magnetometer.hpp"

#include <mutex>
#include <type_traits>
#include <variant>

#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/simulation/common/commands.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/simulation/utils/eigen_adapter.hpp"
#include "dronelab/simulation/utils/noise_generator.hpp"

namespace dronelab::sim {

Magnetometer::Magnetometer(ContextView context)
    : Node{context, NodeType::kMagnetometer},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberDroneState_{context, *this, TopicKey::kDroneState},
      subscriberSimCmd_{context, *this, TopicKey::kSimCmd},
      publisherMagnetometerData_{context, *this, TopicKey::kMagnetometerData},
      magParams_{context.getConfig().getMagParams()},
      noiseGenerator_{magParams_.noiseStddev,
                      context.getConfig().getSeed() + static_cast<std::size_t>(kSensorId) * 10},
      earthMagFieldEigen_{utils::toEigen(magParams_.earthMagneticField)},
      activated_{context.getConfig().getSensorConfig().magnetometerEnabled},
      updateRateUs_{static_cast<std::int32_t>(
          1000000.0 / static_cast<double>(context.getConfig().getMagRate()))} {
    subscriberSimCmd_
        .filter([](const auto& msg) -> bool {
            return std::holds_alternative<sim::msg::SensorUpdateCmd>(msg->cmd);
        })
        .subscribe([this](const auto& msg) -> void {
            processSimCmd(std::get<sim::msg::SensorUpdateCmd>(msg->cmd));
        });
    bool activated = activated_;
    activate();
    if (!activated) {
        deactivate();
        SPDLOG_INFO("[{}] Sensor initialized status -> deactivated.", name_);
    }
}

void Magnetometer::activate() {
    SPDLOG_INFO("[{}] Activating Sensor", name_);
    subscriberDroneState_.subscribe([this](const auto& msg) -> void { receivedDroneState(*msg); });

    subscriberSimTime_
        .filter([this](const auto& msg) -> bool {
            return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
        })
        .subscribe([this](const auto& msg) -> void {
            lastStepUs_.store(*msg, std::memory_order_relaxed);
            updateData();
        });

    activated_ = true;
}

void Magnetometer::deactivate() {
    SPDLOG_INFO("[{}] Deactivating Sensor", name_);
    subscriberDroneState_.unsubscribe();
    subscriberSimTime_.unsubscribe();

    activated_ = false;
}

void Magnetometer::processSimCmd(const msg::SensorUpdateCmd& cmd) {
    std::lock_guard lock{mutex_};
    std::visit(
        [&](auto&& subcmd) {
            using T = std::decay_t<decltype(subcmd)>;
            if constexpr (std::is_same_v<T, msg::EnableOrDisableSensor>) {
                if (subcmd.target == msg::SensorID::kMagnetometer) {
                    if (subcmd.value && !activated_) {
                        activate();
                    } else if (!subcmd.value && activated_) {
                        deactivate();
                    }
                }
            } else if constexpr (std::is_same_v<T, msg::MagNoiseCmd>) {
                const auto idx = static_cast<std::uint8_t>(subcmd.coord);
                switch (subcmd.target) {
                    case msg::MagNoiseCmd::Target::kBias: {
                        magParams_.bias[idx] = subcmd.value;
                        break;
                    }
                    case msg::MagNoiseCmd::Target::kNoise: {
                        noiseGenerator_.updateCoord(idx, subcmd.value);
                        break;
                    }
                }
            }
        },
        cmd.value);
}

void Magnetometer::receivedDroneState(const msg::DroneState& state) {
    std::lock_guard lock{mutex_};
    lastState_ = state;
    initialized_ = true;
}

void Magnetometer::updateData() {
    std::lock_guard lock{mutex_};
    if (!initialized_) {
        return;
    }
    const Vec3d magBodySample =
        utils::fromEigen(utils::toEigenQuat(lastState_.orientation).inverse() *
                         earthMagFieldEigen_) +
        magParams_.bias + noiseGenerator_.sample();
    const msg::MagnetometerData data{.magneticField = magBodySample,
                                     .simStamp = lastStepUs_.load(std::memory_order_relaxed)};
    publisherMagnetometerData_.publish(data);
}

}  // namespace dronelab::sim