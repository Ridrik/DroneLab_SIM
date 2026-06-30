#include "dronelab/simulation/drone_simulator.hpp"

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/exceptions.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/sim_report.hpp"
#include "dronelab/api/core/utils/common.hpp"
#include "dronelab/api/simulation/common/simulation.hpp"
#include "dronelab/api/simulation/common/vehicle.hpp"
#include "dronelab/api/simulation/control/config.hpp"
#include "dronelab/api/simulation/simulator.hpp"
#include "dronelab/dronelab_sim_export.h"
#include "dronelab/simulation/control/lqr_controller.hpp"
#include "dronelab/simulation/control/mixer.hpp"
#include "dronelab/simulation/control/pid_controller.hpp"
#include "dronelab/simulation/dynamics/battery_system.hpp"
#include "dronelab/simulation/dynamics/dynamic_simulator.hpp"
#include "dronelab/simulation/management/flight_manager.hpp"
#include "dronelab/simulation/navigation/loiter_behaviour.hpp"
#include "dronelab/simulation/navigation/rtl_behaviour.hpp"
#include "dronelab/simulation/navigation/trajectory_generator.hpp"
#include "dronelab/simulation/navigation/waypoint_behaviour.hpp"
#include "dronelab/simulation/sensors/barometer.hpp"
#include "dronelab/simulation/sensors/ekf.hpp"
#include "dronelab/simulation/sensors/gps_sensor.hpp"
#include "dronelab/simulation/sensors/imu.hpp"
#include "dronelab/simulation/sensors/magnetometer.hpp"
#include "dronelab/simulation/sensors/processor.hpp"
#include "dronelab/simulation/system/system_monitor.hpp"

#include "fault/core.hpp"

namespace {

#ifdef _WIN32
constexpr int kSleepThresholdUs = 2000;
constexpr int kSleepOffsetThresholdUs = 200;
#else
constexpr int kSleepThresholdUs = 200;
constexpr int kSleepOffsetThresholdUs = 20;
#endif

[[nodiscard]] constexpr std::size_t lcmAll(std::size_t a, std::size_t b) {
    return std::lcm(a, b);
}

template <typename... Rest>
[[nodiscard]] constexpr std::size_t lcmAll(std::size_t a, std::size_t b, Rest... rest) {
    return lcmAll(std::lcm(a, b), rest...);
}

[[nodiscard]] std::size_t tickUs(const dronelab::sim::msg::ServiceRates& rates) {
    const auto lcm = lcmAll(rates.dynamics, rates.imu, rates.gps, rates.attController,
                            rates.posController, rates.traj, rates.waypointBehaviour);
    const double tickSec = 1.0 / static_cast<double>(lcm);
    return static_cast<std::size_t>(std::round(tickSec * 1e6));
}

}  // namespace

namespace dronelab::sim {

DroneSimulator::DroneSimulator(ContextView context)
    : DroneSimulatorBase{context, NodeType::kMain},
      subscriberNodeRequest_{context, *this, TopicKey::kNodeRequest},
      subscriberSimMainReport_{context, *this, TopicKey::kSimReport},
      publisherSimMainReport_{context, *this, TopicKey::kSimReport},
      publisherSimTime_{context, *this, TopicKey::kSimTime},
      timerView_{context} {
    subscriberNodeRequest_.subscribe([this](const auto& msg) {
        SPDLOG_DEBUG("[{}] Received node request", name_);
        nodeRequest_ = *msg;
    });
}

DroneSimulator::SimAction DroneSimulator::processNodeRequest() {
    auto request = nodeRequest_.exchange(msg::NodeRequest::kNone, std::memory_order_relaxed);
    switch (request) {
        case msg::NodeRequest::kNone:
        case msg::NodeRequest::kPause:
        case msg::NodeRequest::kResume:
            return SimAction::kNone;
        case msg::NodeRequest::kStop: {
            SPDLOG_INFO(
                "[{}] Detected Node Request to stop. Action granted, stopping "
                "simulation.",
                name_);
            endStatus_ = msg::SimulationEndStatus::kInternalTermination;
            return SimAction::kStop;
        }
    }
    return SimAction::kNone;
}

void DroneSimulator::postProcessing() {
    SPDLOG_INFO("[{}] Sending report request to Nodes", name_);
    subscriberSimMainReport_
        .filter([](const auto& cmd) { return cmd->type == SimReportCmd::Type::kReport; })
        .subscribe([this](const auto& cmd) -> void {
            const auto nodeStr = nodeTypeToString(cmd->node);
            report_.add(nodeStr, cmd->report);
        });
    publisherSimMainReport_.publish(
        SimReportCmd{.type = SimReportCmd::Type::kRequest, .node = NodeType::kMain, .report = {}});
    // TODO: Maybe add blocking situation here, but point is for these operations to be synchronous
}

void DroneSimulator::startSimulation(msg::SimMode mode) {
    FAULT_ASSERT(simState_ == msg::SimState::kUninit,
                 "(From {}) Call to start simulation when simulation indicates to not be reset",
                 name_);
    FAULT_ASSERT(nodes_.empty(),
                 "(From {}) Requested to start simulation, found it in a dirty state.", name_);
    fault::try_catch(
        [this, mode] {
            nodeRequest_ = msg::NodeRequest::kNone;
            externalRequest_ = msg::ExternalAsyncRequest::kNone;
            initNodes(mode);
            run();
        },
        fault::CatchPolicy::kSaveExceptionWithShutdownRequest);
}

void DroneSimulator::initNodes(msg::SimMode mode) {
    initDynamics();
    initSensors();
    if (mode == msg::SimMode::kSelfContained) {
        initFlightController();
    }
}

void DroneSimulator::initSensors() {
    const auto config = context_.getConfig();
    const auto& imuParams = config.getImuParams();
    for (std::size_t i{0}; i < imuParams.imuToBody.size(); ++i) {
        nodes_.push_back(std::make_unique<IMUSensor>(context_, i));
    }
    nodes_.push_back(std::make_unique<GpsSensor>(context_));
    nodes_.push_back(std::make_unique<Magnetometer>(context_));
    nodes_.push_back(std::make_unique<Barometer>(context_));
    nodes_.push_back(std::make_unique<BatterySystem>(context_));

    // Vehicle Monitor
    nodes_.push_back(std::make_unique<SystemMonitor>(context_));
}

void DroneSimulator::initDynamics() {
    nodes_.push_back(std::make_unique<DynamicSimulator>(context_, environment_));
}

void DroneSimulator::initFlightController() {
    nodes_.push_back(std::make_unique<Processor>(context_));
    nodes_.push_back(std::make_unique<Ekf>(context_));
    switch (context_.getConfig().getControlConfig().mode) {
        case msg::ControlStrategy::kPIDCascade: {
            nodes_.push_back(std::make_unique<PIDController>(context_));
            break;
        }
        case msg::ControlStrategy::kLQRHover: {
            nodes_.push_back(std::make_unique<LQRController>(context_));
            break;
        }
    }
    switch (context_.getConfig().getDroneParameters().vehicletype) {
        case msg::VehicleType::kQuadCopter: {
            nodes_.push_back(std::make_unique<QuadMixer>(context_));
            break;
        }
        case msg::VehicleType::kHexaCopter: {
            nodes_.push_back(std::make_unique<HexaMixer>(context_));
            break;
        }
        case msg::VehicleType::kOctoCopter: {
            nodes_.push_back(std::make_unique<OctoMixer>(context_));
            break;
        }
    }

    nodes_.push_back(std::make_unique<WaypointBehaviour>(context_));
    nodes_.push_back(std::make_unique<LoiterBehaviour>(context_));
    nodes_.push_back(std::make_unique<RtlBehaviour>(context_));
    nodes_.push_back(std::make_unique<TrajectoryGenerator>(context_));
    nodes_.push_back(std::make_unique<FlightManager>(context_));
}

void DroneSimulator::requestPause() {
    FAULT_ASSERT(simState_ != msg::SimState::kPaused,
                 "Pause request, but simulation indicates to already be paused");
    externalRequest_ = msg::ExternalAsyncRequest::kPause;
    SPDLOG_INFO("[{}] Call to pause has been scheduled for the simulator.", name_);
}

void DroneSimulator::requestResume() {
    FAULT_ASSERT(simState_ == msg::SimState::kPaused,
                 "Resume request, but simulation indicates not to be paused");
    SPDLOG_INFO("[{}]  Call to resume has been scheduled for the simulator.", name_);
    FAULT_ASSERT(!nodes_.empty(), "Call to resume simulation, found it empty");
    externalRequest_ = msg::ExternalAsyncRequest::kResume;
    cv_.notify_all();
}

void DroneSimulator::ioTimeCheck(msg::TimePoint nextTick, msg::Stamp simTimeUs) {
    const auto now = std::chrono::steady_clock::now();
    if (now > nextTick) {
        const auto lagUs =
            std::chrono::duration_cast<std::chrono::microseconds>(now - nextTick).count();
        SPDLOG_TRACE(
            "[DroneSimulation] CPU Bottleneck: simulation delta step {} cannot keep up "
            "with real time: lagging by {} microseconds.",
            usDt_, lagUs);
    }
    timerView_.expiresAt(nextTick);
    timerView_.asyncWait([this, &nextTick, &simTimeUs](const auto& ec) {
        if (ec) {
            SPDLOG_ERROR("[{}] I/O Timer failure. Stopping simulation.", name_);
            simTimeUs_ = simTimeUs;
            postProcessing();
            stop();
            return;
        }

        publisherSimTime_.publish(simTimeUs);
        auto action = processExtRequest();
        auto action2 = processNodeRequest();
        if (action == SimAction::kNone && action2 == SimAction::kNone) {
            ioTimeCheck(nextTick + tickDuration_, simTimeUs + usDt_);
            return;
        }
        if (action == SimAction::kPause || action2 == SimAction::kPause) {
            simState_ = msg::SimState::kPaused;
            auto afterPauseAction = pause();  // Blocking call
            switch (afterPauseAction) {
                case SimAction::kPause: {
                    throw exceptions::LogicError(
                        "(From DroneSimulator) Sim just got out of a pause, but still with "
                        "action "
                        "to pause.");
                    break;
                }
                case SimAction::kStop: {
                    simTimeUs_ = simTimeUs;
                    postProcessing();
                    stop();
                    break;
                }
                case SimAction::kNone: {
                    simState_ = msg::SimState::kActive;
                    ioTimeCheck(nextTick + tickDuration_, simTimeUs + usDt_);
                    return;
                }
            }
        } else if (action == SimAction::kStop || action2 == SimAction::kStop) {
            simTimeUs_ = simTimeUs;
            postProcessing();
            stop();
        }
    }  // namespace dronelab::sim
    );
}

void DroneSimulator::run() {
    switch (runStrategy_) {
        case msg::RunStrategy::kIoContext: {
            runIoContext();
            break;
        }
        case msg::RunStrategy::kLoopRealTime: {
            runRealTime();
            break;
        }
        case msg::RunStrategy::kLoopLimitless: {
            runLimitless();
            break;
        }
    }
}

void DroneSimulator::runIoContext() {
    context_.ioReset();

    if (runStrategy_ == msg::RunStrategy::kIoContext) {
        context_.post([this] { simState_ = msg::SimState::kActive; });
    }

    auto serviceRates = context_.getConfig().getRates();
    usDt_ = static_cast<int>(tickUs(serviceRates));
    tickDuration_ = std::chrono::microseconds(usDt_);
    SPDLOG_DEBUG("[{}] Selected simulation time step is {} microseconds.", name_, usDt_);
    msg::TimePoint wallStart = std::chrono::steady_clock::now();
    msg::Stamp simTimeUs{0};

    ioTimeCheck(wallStart, simTimeUs);

    context_.ioRun();
}

void DroneSimulator::runRealTime() {
    const auto serviceRates = context_.getConfig().getRates();
    const auto usDt = static_cast<int>(tickUs(serviceRates));
    SPDLOG_DEBUG("[{}] Selected simulation time step is {} microseconds.", name_, usDt);

    const auto tickDuration = std::chrono::microseconds(usDt);
    const auto spinThreshold = std::chrono::microseconds(kSleepThresholdUs);
    const auto spinOffsetThreshold = std::chrono::microseconds(kSleepOffsetThresholdUs);

    std::int64_t simTimeUs{0};
    const msg::TimePoint wallStart = std::chrono::steady_clock::now();
    auto nextTick = wallStart;
    simState_ = msg::SimState::kActive;
    while (true) {
        publisherSimTime_.publish(simTimeUs);
        auto action = processExtRequest();
        auto action2 = processNodeRequest();
        if (action == SimAction::kPause || action2 == SimAction::kPause) {
            simState_ = msg::SimState::kPaused;
            bool exit{false};
            auto afterPauseAction = pause();  // Blocking call
            switch (afterPauseAction) {
                case SimAction::kPause: {
                    throw exceptions::LogicError(
                        "(From DroneSimulator) Sim just got out of a pause, but still with "
                        "action "
                        "to pause.");
                }
                case SimAction::kStop: {
                    exit = true;
                    break;
                }
                case SimAction::kNone: {
                    nextTick = std::chrono::steady_clock::now();
                    simState_ = msg::SimState::kActive;
                    break;
                }
            }
            if (exit) {
                break;
            }
        } else if (action == SimAction::kStop || action2 == SimAction::kStop) {
            break;
        }
        simTimeUs += usDt;
        const float speed = speed_.load(std::memory_order_relaxed);
        const auto tickScaled = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::duration<float>(tickDuration) / speed);
        nextTick += tickScaled;

        const auto now = std::chrono::steady_clock::now();
        if (now < nextTick - spinThreshold) {
            std::this_thread::sleep_until(nextTick - spinThreshold);
            while (std::chrono::steady_clock::now() < nextTick - spinOffsetThreshold) {
                std::this_thread::yield();
            }
        } else if (now > nextTick) {
            auto lagUs =
                std::chrono::duration_cast<std::chrono::microseconds>(now - nextTick).count();
            SPDLOG_TRACE(
                "[{}] CPU Bottleneck: simulation delta step {} cannot keep up "
                "with real time: lagging by {} microseconds.",
                name_, usDt, lagUs);
            nextTick = now + spinOffsetThreshold;
        }
    }
    simTimeUs_ = simTimeUs;
    postProcessing();
    stop();
}

void DroneSimulator::runLimitless() {
    const auto serviceRates = context_.getConfig().getRates();
    const auto usDt = static_cast<int>(tickUs(serviceRates));
    SPDLOG_DEBUG("[{}] Selected simulation time step is {} microseconds.", name_, usDt);
    std::int64_t simTimeUs{0};
    simState_ = msg::SimState::kActive;
    while (true) {
        simTimeUs += usDt;
        publisherSimTime_.publish(simTimeUs);
        const auto action = processExtRequest();
        const auto action2 = processNodeRequest();
        if (action == SimAction::kPause || action2 == SimAction::kPause) {
            simState_ = msg::SimState::kPaused;
            bool exit{false};
            const auto afterPauseAction = pause();  // Blocking call
            switch (afterPauseAction) {
                case SimAction::kPause: {
                    throw exceptions::LogicError(
                        "(From DroneSimulator) Sim just got out of a pause, but still with "
                        "action "
                        "to pause.");
                }
                case SimAction::kStop: {
                    exit = true;
                    break;
                }
                case SimAction::kNone: {
                    simState_ = msg::SimState::kActive;
                    break;
                }
            }
            if (exit) {
                break;
            }
        } else if (action == SimAction::kStop || action2 == SimAction::kStop) {
            break;
        }
    }
    simTimeUs_ = simTimeUs;
    postProcessing();
    stop();
}

DroneSimulator::SimAction DroneSimulator::processExtRequest() {
    const auto request =
        externalRequest_.exchange(msg::ExternalAsyncRequest::kNone, std::memory_order_relaxed);
    switch (request) {
        case msg::ExternalAsyncRequest::kNone:
            return SimAction::kNone;
        case msg::ExternalAsyncRequest::kPause: {
            SPDLOG_INFO(
                "[{}] Detected external request to Pause. Action granted, pausing "
                "simulation",
                name_);
            return SimAction::kPause;
        }
        case msg::ExternalAsyncRequest::kResume:
            return SimAction::kNone;
        case msg::ExternalAsyncRequest::kStop: {
            SPDLOG_INFO(
                "[{}] Detected external request to Stop. Action granted, stopping "
                "simulation",
                name_);
            return SimAction::kStop;
        }
    }
    return SimAction::kNone;
}

DroneSimulator::SimAction DroneSimulator::pause() {
    std::unique_lock lock(mutex_);
    SimAction action{SimAction::kNone};
    cv_.wait(lock, [this, &action] {
        auto checkRequest =
            externalRequest_.exchange(msg::ExternalAsyncRequest::kNone, std::memory_order_relaxed);
        if (checkRequest == msg::ExternalAsyncRequest::kStop) {
            action = SimAction::kStop;
            return true;
        }
        return checkRequest == msg::ExternalAsyncRequest::kResume;
    });
    SPDLOG_DEBUG("[{}] Simulation is no longer paused (sleeping)", name_);
    return action;
}

void DroneSimulator::stop() {
    simState_ = msg::SimState::kStopped;
    if (runStrategy_ == msg::RunStrategy::kIoContext && !context_.ioStop()) {
        SPDLOG_ERROR("[{}] On Simulation Stop, found that simulation was already stopped", name_);
    }
    context_.joinThreadPool();
    nodes_.clear();
}

void DroneSimulator::requestStop(bool abruptStop) {
    FAULT_ASSERT(simState_ != msg::SimState::kStopped && simState_ != msg::SimState::kUninit,
                 "(From {}) Request to stop, but simulation indicates to already be "
                 "stopped",
                 name_);
    externalRequest_ = msg::ExternalAsyncRequest::kStop;
    endStatus_ = abruptStop ? msg::SimulationEndStatus::kExternalAborted
                            : msg::SimulationEndStatus::kUserTermination;
    cv_.notify_all();
    SPDLOG_DEBUG("[{}] Stop notification sent.", name_);
}

bool DroneSimulator::requestAdjustSpeed(float speed) {
    if (speed < 0.1 - ::dronelab::utils::kEpsilon || speed > 2 + ::dronelab::utils::kEpsilon) {
        return false;
    }
    speed_.store(speed, std::memory_order_relaxed);
    return true;
}

}  // namespace dronelab::sim

extern "C" DRONELAB_SIM_EXPORT dronelab::sim::DroneSimulatorBase* createDroneSimulator(
    dronelab::ContextView context) {
    return new dronelab::sim::DroneSimulator{context};  // NOLINT
}

extern "C" DRONELAB_SIM_EXPORT void deleteDroneSimulator(dronelab::sim::DroneSimulatorBase* base) {
    delete base;  // NOLINT
}