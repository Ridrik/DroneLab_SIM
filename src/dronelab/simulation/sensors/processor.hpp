#ifndef DRONELAB_SIMULATION_SENSORS_PROCESSOR_HPP
#define DRONELAB_SIMULATION_SENSORS_PROCESSOR_HPP

#include <cstddef>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/sensors/imu.hpp"
#include "dronelab/simulation/utils/lock_free_queue.hpp"

namespace dronelab::sim {

class Processor : public Node {
   public:
    Processor(ContextView context);

   private:
    void update();
    void receivedImuData(const msg::ImuData& data);

    Subscriber<msg::Stamp> subMasterClock_;
    Subscriber<msg::ImuData> subImuData_;
    Publisher<msg::VirtualImuData> pubVirtualImuData_;

    std::vector<concurrency::LockFreeQueue<msg::ImuData, concurrency::Mode::kSPSC>> imuBuf_;
    std::vector<msg::VirtualImuData> accumPerImu_;
    msg::VirtualImuData virtualImuPacket_;
    std::uint16_t curTickUs_{};

    msg::Stamp targetEkfTimeUs_;
    double targetEkfTimeSec_;
    std::atomic<msg::Stamp> lastUpdateUs_{0};
    std::size_t masterId_;
    msg::Stamp mainTickUs_;
    msg::Stamp lastMainTickUs_{};
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_SENSORS_PROCESSOR_HPP