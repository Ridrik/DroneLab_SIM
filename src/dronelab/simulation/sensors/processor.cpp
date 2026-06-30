#include "dronelab/simulation/sensors/processor.hpp"

#include <atomic>
#include <cmath>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/sensors/imu.hpp"
#include "dronelab/simulation/data/sensors/imu.hpp"

namespace dronelab::sim {

Processor::Processor(ContextView context)
    : Node{context, NodeType::kImuProcessing},
      subMasterClock_{context, *this, TopicKey::kSimTime},
      subImuData_{context, *this, TopicKey::kImuData},
      pubVirtualImuData_{context, *this, TopicKey::kVirtualImuData},
      targetEkfTimeUs_{static_cast<msg::Stamp>(
          1000000.0 / static_cast<double>(context.getConfig().getRates().ekf))},
      targetEkfTimeSec_{msg::microsToSeconds<double>(targetEkfTimeUs_)},
      masterId_{0},
      mainTickUs_{static_cast<msg::Stamp>(
          1000000.0 / static_cast<double>(context.getConfig().getRates().imu))} {
    subMasterClock_
        .filter([this](const auto& msg) -> bool {
            return *msg - lastUpdateUs_.load(std::memory_order_relaxed) >= targetEkfTimeUs_;
        })
        .subscribe([this](const auto& msg) {
            lastUpdateUs_.fetch_add(targetEkfTimeUs_, std::memory_order_relaxed);
            update();
        });
    subImuData_.subscribe([this](const auto& msg) { receivedImuData(*msg); });
    constexpr std::size_t kSafetyBufferSize{2};
    const auto bufSize = std::clamp<std::size_t>(
        static_cast<std::size_t>(
            std::ceil(static_cast<double>(context.getConfig().getRates().imu) /
                      static_cast<double>(context.getConfig().getRates().ekf))) *
            kSafetyBufferSize,
        1U, 32U);
    const auto imuCount = context.getConfig().getImuParams().imuToBody.size();
    imuBuf_.reserve(imuCount);
    for (std::size_t i{0}; i < imuCount; ++i) {
        imuBuf_.emplace_back(bufSize);
    }
    accumPerImu_.resize(bufSize);
}

void Processor::receivedImuData(const msg::ImuData& data) {
    imuBuf_[data.id].enqueue(data);
}

void Processor::update() {
    const auto curTick = lastUpdateUs_.load(std::memory_order_relaxed);
    const auto prevTick = curTick - targetEkfTimeUs_;
    std::size_t imuCount{0};
    for (std::size_t i{0}; i < imuBuf_.size(); ++i) {
        auto& buf = imuBuf_[i];
        auto& accum = accumPerImu_[i];
        bool anyRecord{false};
        msg::ImuData* prevRawData{nullptr};
        auto remSize{buf.getSize()};
        while (remSize > 0) {
            auto* optRawImu = buf.getFront();
            FAULT_ASSERT(optRawImu != nullptr, "Got null front IMU when queue is not empty");
            auto& rawImu = *optRawImu;
            if (!rawImu.valid || rawImu.simStamp < prevTick) {
                buf.dequeue();
                --remSize;
                continue;
            }
            if (prevRawData != nullptr) {
                FAULT_ASSERT(rawImu.simStamp > prevRawData->simStamp,
                             "Unexpected consecutive raw imu stamps ({} -> {})",
                             prevRawData->simStamp, rawImu.simStamp);
                // accum.add(*prevRawData,
                //           msg::microsToSeconds<double>(rawImu.simStamp - prevRawData->simStamp));
                msg::addImuRawData(
                    accum, *prevRawData,
                    msg::stampToSeconds<double>(rawImu.simStamp - prevRawData->simStamp));
                anyRecord = true;
            }
            prevRawData = &rawImu;
            if (remSize == 1) {  // We keep one raw IMU for next round (unless it's too old or
                                 // invalid, as per condition above)
                break;
            }
            buf.dequeue();
            --remSize;
        }
        if (prevRawData != nullptr && prevRawData->simStamp < curTick) {
            // accum.add(*prevRawData, msg::microsToSeconds<double>(curTick -
            // prevRawData->simStamp));
            msg::addImuRawData(accum, *prevRawData,
                               msg::stampToSeconds<double>(curTick - prevRawData->simStamp));
            anyRecord = true;
            prevRawData->simStamp = curTick;  // We shift it for next update
        }
        if (anyRecord) {
            // virtualImuPacket_.add(accum);
            msg::addVirtualData(virtualImuPacket_, accum);
            msg::resetVirtualImu(accum);
            // accum.reset();
            ++imuCount;
        }
    }
    if (imuCount > 0) {
        msg::averageVirtualImu(virtualImuPacket_, imuCount);
        virtualImuPacket_.totalDt = targetEkfTimeSec_;
        pubVirtualImuData_.publish(virtualImuPacket_);
        msg::resetVirtualImu(virtualImuPacket_);
    }
}

}  // namespace dronelab::sim