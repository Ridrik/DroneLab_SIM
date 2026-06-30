#include "dronelab/simulation/dynamics/dynamic_simulator.hpp"

#include <algorithm>
#include <mutex>

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/data/constants.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/core/utils/common.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/common/vehicle.hpp"
#include "dronelab/api/simulation/dynamics/actuator_data.hpp"
#include "dronelab/api/simulation/dynamics/drone_params.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/environment/wind_field.hpp"
#include "dronelab/simulation/utils/eigen_adapter.hpp"

namespace {

Eigen::Quaterniond quaternionDerivative(const Eigen::Quaterniond& q,
                                        const Eigen::Vector3d& omegaBody) {
    const auto& w = omegaBody;
    Eigen::Matrix4d omega;
    omega << 0, -w.x(), -w.y(), -w.z(), w.x(), 0, w.z(), -w.y(), w.y(), -w.z(), 0, w.x(), w.z(),
        w.y(), -w.x(), 0;

    const Eigen::Vector4d qVec(q.w(), q.x(), q.y(), q.z());

    const Eigen::Vector4d qDot = 0.5 * omega * qVec;
    return Eigen::Quaterniond{qDot(0), qDot(1), qDot(2), qDot(3)};
}

}  // namespace

namespace dronelab::sim {

DynamicSimulator::DynamicSimulator(ContextView context, const Environment* environment)
    : Node{context, NodeType::kDynamicSimulator},
      windField_{environment != nullptr ? environment->getWindField() : nullptr},
      subscriberSimTime_{context, *this, TopicKey::kSimTime},
      // subscriberActuatorFeedback_{context, *this, TopicKey::kActuatorFeedback},
      publisherDroneState_{context, *this, TopicKey::kDroneState},
      actuator_{context},
      latestDroneState_{context.getConfig().getDroneState()},
      droneParameters_{context.getConfig().getDroneParameters()},
      inertia_{Eigen::Map<const Eigen::Vector3d>(droneParameters_.inertiaDiag.data()).asDiagonal()},
      iInv_{inertia_.inverse()},
      invMass_{1.0 / droneParameters_.mass},
      motorNumber_{msg::getNumMotors(droneParameters_.vehicletype)},
      optHomePosition_{context.getConfig().getMission().rtlBehaviourConfig.homePosition},
      updateRateSeconds_{1.0 / static_cast<double>(context.getConfig().getDynamicsRate())},
      updateRateUs_{static_cast<std::int32_t>(updateRateSeconds_ * 1000000.0)} {
    if (windField_ == nullptr) {
        SPDLOG_WARN("[{}] No Wind field provided", name_);
    }
    // subscriberActuatorFeedback_.subscribe(
    //    [this](const auto& data) { receivedActuatorFeedback(*data); });

    subscriberSimTime_
        .filter([this](const auto& msg) -> bool {
            return *msg - lastStepUs_.load(std::memory_order_relaxed) >= updateRateUs_;
        })
        .subscribe([this](const auto& msg) -> void {
            curStepUs_ = *msg;
            lastStepUs_.store(curStepUs_, std::memory_order_relaxed);
            updateDroneState();
        });

    const double l = droneParameters_.armLength;
    const double kF = droneParameters_.thrustCoefficient;
    const double kM = droneParameters_.torqueCoefficient;
    const double lM = std::numbers::sqrt2 * l * 0.5;
    // --- Mixer Matrix (NED + X configuration) ---
    const auto& quadXAlloc = droneParameters_.quadXAlloc;
    const double kMf = kM / kF;
    const std::unordered_map<sim::msg::QuadXMap, Eigen::Vector4d> motorToMixerQuadMap = {
        {msg::QuadXMap::kFrontRight, {1.0, -lM, +lM, kMf}},
        {msg::QuadXMap::kRearLeft, {1.0, +lM, -lM, kMf}},
        {msg::QuadXMap::kFrontLeft, {1.0, +lM, +lM, kMf}},
        {msg::QuadXMap::kRearRight, {1.0, -lM, -lM, kMf}}};
    for (std::uint8_t i{0}; i < 4; ++i) {
        mixerQuad_.col(i) = motorToMixerQuadMap.at(quadXAlloc.indexAlloc[i]);
        if (!quadXAlloc.isCCWorCW[i]) {
            mixerQuad_.col(i)[3] = -mixerQuad_.col(i)[3];
        }
    }

    using namespace ::dronelab::utils;

    const double lCos30 = std::cos(degToRad(30.0)) * l;
    const double lSin30 = std::sin(degToRad(30.0)) * l;
    // --- Mixer Matrix (NED) ---
    const auto& hexAlloc = droneParameters_.hexAlloc;
    const std::unordered_map<sim::msg::HexMap, Eigen::Vector4d> motorToMixerHexaMap = {
        {msg::HexMap::kFront30Right, {1.0, -lSin30, +lCos30, kMf}},
        {msg::HexMap::kFront30Left, {1.0, +lSin30, +lCos30, kMf}},
        {msg::HexMap::kLeft, {1.0, +l, 0.0, kMf}},
        {msg::HexMap::kRear30Left, {1.0, +lSin30, -lCos30, kMf}},
        {msg::HexMap::kRear30Right, {1.0, -lSin30, -lCos30, kMf}},
        {msg::HexMap::kRight, {1.0, -l, 0.0, kMf}}};
    for (std::uint8_t i{0}; i < 6; ++i) {
        mixerHexa_.col(i) = motorToMixerHexaMap.at(hexAlloc.indexAlloc[i]);
        if (!hexAlloc.isCCWorCW[i]) {
            mixerHexa_.col(i)[3] = -mixerHexa_.col(i)[3];
        }
    }

    const double lCos225 = std::cos(degToRad(22.5)) * l;
    const double lSin225 = std::sin(degToRad(22.5)) * l;
    // --- Mixer Matrix (NED) ---
    const auto& octoAlloc = droneParameters_.octoAlloc;
    const std::unordered_map<sim::msg::OctoMap, Eigen::Vector4d> motorToMixerMap = {
        {msg::OctoMap::kFront22Right, {1.0, -lSin225, +lCos225, kMf}},
        {msg::OctoMap::kFront22Left, {1.0, +lSin225, +lCos225, kMf}},
        {msg::OctoMap::kFront67Left, {1.0, +lCos225, +lSin225, kMf}},
        {msg::OctoMap::kRear67Left, {1.0, +lCos225, -lSin225, kMf}},
        {msg::OctoMap::kRear22Left, {1.0, +lSin225, -lCos225, kMf}},
        {msg::OctoMap::kRear22Right, {1.0, -lSin225, -lCos225, kMf}},
        {msg::OctoMap::kRear67Right, {1.0, -lCos225, -lSin225, kMf}},
        {msg::OctoMap::kFront67Right, {1.0, -lCos225, +lSin225, kMf}}};
    for (std::uint8_t i{0}; i < 8; ++i) {
        mixerOcto_.col(i) = motorToMixerMap.at(octoAlloc.indexAlloc[i]);
        if (!octoAlloc.isCCWorCW[i]) {
            mixerOcto_.col(i)[3] = -mixerOcto_.col(i)[3];
        }
    }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> DynamicSimulator::computeForcesAndTorques(
    const DroneStateVector& state) {
    const auto& motorSpeeds = state.actuatorState.motorSpeeds;

    // Compute thrust rotor vector
    const Eigen::Vector<double, msg::kActuatorMaxNum> thrusts =
        droneParameters_.thrustCoefficient * utils::toEigen(motorSpeeds).array().square().matrix();

    // Total thrust (body frame, z direction)
    const Eigen::Vector3d totalForceBody(0, 0, -thrusts.sum());
    Eigen::Vector3d torque{Eigen::Vector3d::Zero()};
    switch (droneParameters_.vehicletype) {
        case msg::VehicleType::kQuadCopter: {
            torque = (mixerQuad_ * thrusts.head<4>()).block<3, 1>(1, 0);
            break;
        }
        case msg::VehicleType::kHexaCopter: {
            torque = (mixerHexa_ * thrusts.head<6>()).block<3, 1>(1, 0);
            break;
        }
        case msg::VehicleType::kOctoCopter: {
            torque = (mixerOcto_ * thrusts.head<8>()).block<3, 1>(1, 0);
            break;
        }
    }

    const Eigen::Vector3d thrustWorld = state.orientation * totalForceBody;

    static const Eigen::Vector3d gravity(0, 0, constants::kGravity);

    // Wind
    Eigen::Vector3d wind = Eigen::Vector3d::Zero();
    if (windField_ != nullptr) {
        wind = utils::toEigen(windField_->getWindAt(
            utils::fromEigen(state.position),
            msg::microsToSeconds<double>(lastStepUs_.load(std::memory_order_relaxed))));
    }

    // Aerodynamic drag
    Eigen::Vector3d dragForce = Eigen::Vector3d::Zero();
    if (droneParameters_.useQuadraticDrag) {
        const Eigen::Vector3d airspeed = state.velocity - wind;
        const double vMag = airspeed.norm();
        if (vMag > 1e-3) [[likely]] {
            dragForce = -0.5 * droneParameters_.airDensity * droneParameters_.dragCoefficient *
                        droneParameters_.frontalArea * vMag * airspeed;
        }
    }

    // Total force in world frame
    Eigen::Vector3d totalForceWorld = thrustWorld + droneParameters_.mass * gravity + dragForce;

    return {std::move(totalForceWorld), std::move(torque)};
}

// Dynamics function for RK4 integration
DroneStateDerivative DynamicSimulator::computeStateDerivative(const DroneStateVector& state,
                                                              double /*t*/) {
    DroneStateDerivative derivative;

    const auto [totalForce, torque] = computeForcesAndTorques(state);

    // Linear dynamics
    derivative.positionDot = state.velocity;
    derivative.velocityDot = totalForce * invMass_;

    // Angular dynamics
    const Eigen::Matrix3d& I = inertia_;
    derivative.angularVelocityDot =
        iInv_ * (torque - state.angularVelocity.cross(I * state.angularVelocity));

    derivative.orientationDot = quaternionDerivative(state.orientation, state.angularVelocity);
    derivative.motorUpdateInfo = actuator_.calcSpeedDerivative(state.actuatorState);

    return derivative;
}

DroneStateVector DynamicSimulator::rK4Step(const DroneStateVector& state, double dt) {
    const DroneStateDerivative k1 = computeStateDerivative(state, 0.0);

    const double halfDt = 0.5 * dt;
    DroneStateVector stateK2 = state + k1 * halfDt;
    stateK2.orientation.normalize();
    const DroneStateDerivative k2 = computeStateDerivative(stateK2, halfDt);

    DroneStateVector stateK3 = state + k2 * halfDt;
    stateK3.orientation.normalize();
    const DroneStateDerivative k3 = computeStateDerivative(stateK3, halfDt);

    DroneStateVector stateK4 = state + k3 * dt;
    stateK4.orientation.normalize();
    const DroneStateDerivative k4 = computeStateDerivative(stateK4, dt);

    // RK4 formula
    const DroneStateDerivative combined = k1 + ((k2 + k3) * 2.0) + k4;
    constexpr double kOneSixth{1.0 / 6.0};
    DroneStateVector newState = state + (combined * (dt * kOneSixth));

    newState.orientation.normalize();

    return newState;
}

void DynamicSimulator::checkAndApplyConstraints(DroneStateVector& state, double minHeight,
                                                Eigen::Vector3d& thrust) {
    // Basic altitude check - placeholder until proper boundary checks are implemented
    constexpr double kRepulsionGain{50.0};  // N/m
    constexpr double kDamping{10.0};        // N/(m/s)

    if (state.position.z() > minHeight) {
        const double penetration = state.position.z() - minHeight;
        const Eigen::Vector3d correctionForce(
            0, 0, kRepulsionGain * penetration - kDamping * state.velocity.z());
        thrust.z() += correctionForce.z();

        state.position.z() = minHeight;
        state.velocity.z() = std::min(state.velocity.z(), 0.0);
    }

    /*if (state.position.z() > minHeight) {
        state.position.z() = minHeight;
        state.velocity.z() = std::min(state.velocity.z(), 0.0);
        thrust.z() = std::min(thrust.z(), 0.0);
    }*/
}

void DynamicSimulator::updateDroneState() {
    std::lock_guard lock{mutex_};

    auto currentState = DroneStateVector{latestDroneState_, actuator_.getActuatorState()};
    DroneStateVector newState = rK4Step(currentState, updateRateSeconds_);
    // This is meant as a temporary height check until proper collision is implemented, noting
    // that only floor and launch pad exist
    double maxHeightConstraintInNED = -0.5 * droneParameters_.bodyDimensions.z();
    if (optHomePosition_.has_value() &&
        std::abs(newState.position.x()) < constants::kLaunchPadHalfSize &&
        std::abs(newState.position.y()) < constants::kLaunchPadHalfSize) {
        maxHeightConstraintInNED -= constants::kLaunchPadFullHeight;
    }
    auto [totalForce, torque] = computeForcesAndTorques(newState);
    DynamicSimulator::checkAndApplyConstraints(newState, maxHeightConstraintInNED, totalForce);
    latestDroneState_.position = utils::fromEigen(newState.position);
    latestDroneState_.velocity = utils::fromEigen(newState.velocity);
    latestDroneState_.orientation = utils::fromEigen(newState.orientation);
    latestDroneState_.angularVelocity = utils::fromEigen(newState.angularVelocity);
    latestDroneState_.acceleration = utils::fromEigen(totalForce * invMass_);

    latestDroneState_.simStamp = curStepUs_;
    publisherDroneState_.publish(latestDroneState_);

    actuator_.calcAndSendMotorFeedback(newState.actuatorState, curStepUs_);
}

/*void DynamicSimulator::receivedActuatorFeedback(const msg::ActuatorFeedback& msg) {
    std::lock_guard lock{mutex_};
    // latestFeedback_ = msg;
}*/

}  // namespace dronelab::sim