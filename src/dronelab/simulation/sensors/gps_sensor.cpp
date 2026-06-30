#include "dronelab/simulation/sensors/gps_sensor.hpp"

#include <mutex>
#include <variant>

#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/simulation/common/commands.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/simulation/utils/noise_generator.hpp"

namespace dronelab::sim {

GpsSensor::GpsSensor(ContextView context)
    : Node{context, NodeType::kGpsSensor},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      subscriberDroneState_{context, *this, TopicKey::kDroneState},
      subscriberSimCmd_{context, *this, TopicKey::kSimCmd},
      publisherGpsData_{context, *this, TopicKey::kGpsData},
      posNoiseGenerator_{context.getConfig().getGpsParams().positionNoiseStddevXy,
                         context.getConfig().getGpsParams().positionNoiseStddevZ,
                         context.getConfig().getSeed() + static_cast<std::size_t>(kSensorId) * 10},
      velNoiseGenerator_{
          context.getConfig().getGpsParams().velocityNoiseStddev,
          context.getConfig().getSeed() + static_cast<std::size_t>(kSensorId) * 10 + 1},
      activated_{context.getConfig().getSensorConfig().gpsEnabled},
      updateRateUs_{static_cast<std::int32_t>(
          1000000.0 / static_cast<double>(context.getConfig().getGpsRate()))} {
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

void GpsSensor::activate() {
    SPDLOG_INFO("[{}] Activating Sensor", name_);
    subscriberDroneState_.subscribe([this](const auto& msg) -> void { receivedDroneState(*msg); });
    subscriberSimTime_
        .filter([this](const auto& msg) -> bool {
            return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
        })
        .subscribe([this](const auto& msg) -> void {
            lastStepUs_.store(*msg, std::memory_order_relaxed);
            updateGpsData();
        });

    activated_ = true;
}

void GpsSensor::deactivate() {
    SPDLOG_INFO("[{}] Deactivating Sensor", name_);
    subscriberDroneState_.unsubscribe();
    subscriberSimTime_.unsubscribe();

    activated_ = false;
}

void GpsSensor::receivedDroneState(const msg::DroneState& state) {
    std::lock_guard lock{mutex_};

    lastState_ = state;
    initialized_ = true;
}

void GpsSensor::processSimCmd(const msg::SensorUpdateCmd& cmd) {
    std::lock_guard lock{mutex_};
    std::visit(
        [&](auto&& subcmd) {
            using T = std::decay_t<decltype(subcmd)>;
            if constexpr (std::is_same_v<T, msg::EnableOrDisableSensor>) {
                if (subcmd.target == msg::SensorID::kGPS) {
                    if (subcmd.value && !activated_) {
                        activate();
                    } else if (!subcmd.value && activated_) {
                        deactivate();
                    }
                }
            } else if constexpr (std::is_same_v<T, msg::GPSNoiseCmd>) {
                switch (subcmd.target) {
                    case msg::GPSNoiseCmd::Target::kXY: {
                        posNoiseGenerator_.updateXY(subcmd.value);
                        break;
                    }
                    case msg::GPSNoiseCmd::Target::kZ: {
                        posNoiseGenerator_.updateZ(subcmd.value);
                        break;
                    }
                    case msg::GPSNoiseCmd::Target::kVel: {
                        break;
                    }
                }
            }
        },
        cmd.value);
}

void GpsSensor::updateGpsData() {
    std::lock_guard lock{mutex_};
    if (!initialized_) {
        return;
    }
    auto gpsData = msg::GpsData{lastState_.position + posNoiseGenerator_.sample(),
                                lastState_.velocity + velNoiseGenerator_.sampleVec(),
                                lastStepUs_.load(std::memory_order_relaxed)};
    publisherGpsData_.publish(gpsData);
}

}  // namespace dronelab::sim
