#ifndef DRONELAB_SIMULATION_DRONE_SIMULATOR_HPP
#define DRONELAB_SIMULATION_DRONE_SIMULATOR_HPP

#include <condition_variable>
#include <cstdint>
#include <vector>

#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/core/sim_report.hpp"
#include "dronelab/api/core/timer_view.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/common/simulation.hpp"
#include "dronelab/api/simulation/environment/environment.hpp"
#include "dronelab/api/simulation/simulator.hpp"

namespace dronelab::sim {

class DroneSimulator : public DroneSimulatorBase {
   public:
    explicit DroneSimulator(ContextView context);

    void startSimulation(msg::SimMode mode) override;
    void requestPause() override;
    void requestResume() override;
    void requestStop(bool abruptStop = false) override;

    [[nodiscard]] SimReportable moveReport() && override {
        return std::move(report_);
    }

    [[nodiscard]] auto requestAdjustSpeed(float speed) -> bool override;
    [[nodiscard]] auto getSimState() const -> msg::SimState override {
        return static_cast<sim::msg::SimState>(simState_.load(std::memory_order_relaxed));
    }
    [[nodiscard]] auto getSpeed() const -> float override {
        return speed_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] auto getStamp() const -> msg::Stamp override {
        return simTimeUs_;
    }
    [[nodiscard]] auto getEndStatus() const -> msg::SimulationEndStatus override {
        return static_cast<sim::msg::SimulationEndStatus>(endStatus_);
    }
    void setEnvironment(Environment* environment) override {
        environment_ = environment;
    }

   private:
    enum class SimAction : std::uint8_t { kNone, kPause, kStop };
    void run();
    void initNodes(msg::SimMode mode);
    void initSensors();
    void initDynamics();
    void initFlightController();
    void runIoContext();
    void runRealTime();
    void runLimitless();
    void ioTimeCheck(msg::TimePoint nextTick, msg::Stamp simTimeUs);
    void stop();
    [[nodiscard]] SimAction pause();
    void postProcessing();
    [[nodiscard]] SimAction processNodeRequest();
    [[nodiscard]] SimAction processExtRequest();

    std::vector<std::unique_ptr<Node>> nodes_;
    const Environment* environment_{nullptr};
    std::atomic<sim::msg::SimState> simState_{msg::SimState::kUninit};
    msg::Stamp simTimeUs_{};
    std::atomic<sim::msg::ExternalAsyncRequest> externalRequest_{msg::ExternalAsyncRequest::kNone};
    std::atomic<sim::msg::NodeRequest> nodeRequest_{msg::NodeRequest::kNone};
    msg::SimulationEndStatus endStatus_{msg::SimulationEndStatus::kUnknown};
    SimReportable report_;
    int usDt_{0};
    std::atomic<float> speed_{1.0F};

    std::chrono::microseconds tickDuration_{};
    Subscriber<sim::msg::NodeRequest> subscriberNodeRequest_;
    Subscriber<SimReportCmd> subscriberSimMainReport_;
    Publisher<SimReportCmd> publisherSimMainReport_;
    Publisher<sim::msg::Stamp> publisherSimTime_;
    msg::RunStrategy runStrategy_{msg::RunStrategy::kLoopRealTime};
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    TimerView timerView_;
    static constexpr std::chrono::milliseconds kIoCheckUpdateRate{std::chrono::milliseconds(100)};
};
}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_DRONE_SIMULATOR_HPP