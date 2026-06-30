#ifndef DRONELAB_SIMULATION_SERVICES_DYNAMIC_SIMULATOR_HPP
#define DRONELAB_SIMULATION_SERVICES_DYNAMIC_SIMULATOR_HPP

#include <mutex>
#include <utility>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/actuator_data.hpp"
#include "dronelab/api/simulation/dynamics/drone_params.hpp"
#include "dronelab/api/simulation/dynamics/state.hpp"
#include "dronelab/api/simulation/environment/environment.hpp"
#include "dronelab/api/simulation/environment/wind_field.hpp"
#include "dronelab/simulation/dynamics/actuator.hpp"
#include "dronelab/simulation/utils/eigen_adapter.hpp"

namespace dronelab::sim {

// State derivative structure
struct DroneStateDerivative {
    Eigen::Vector3d positionDot;         // velocity
    Eigen::Vector3d velocityDot;         // acceleration
    Eigen::Quaterniond orientationDot;   // quaternion derivative
    Eigen::Vector3d angularVelocityDot;  // angular acceleration
    msg::MotorUpdateInfo motorUpdateInfo;

    DroneStateDerivative() : orientationDot{Eigen::Quaterniond::Identity()} {};

    DroneStateDerivative operator+(const DroneStateDerivative& other) const {
        DroneStateDerivative result;
        result.positionDot = positionDot + other.positionDot;
        result.velocityDot = velocityDot + other.velocityDot;
        result.orientationDot = Eigen::Quaterniond(orientationDot.w() + other.orientationDot.w(),
                                                   orientationDot.x() + other.orientationDot.x(),
                                                   orientationDot.y() + other.orientationDot.y(),
                                                   orientationDot.z() + other.orientationDot.z());
        result.angularVelocityDot = angularVelocityDot + other.angularVelocityDot;
        result.motorUpdateInfo = motorUpdateInfo + other.motorUpdateInfo;
        return result;
    }

    DroneStateDerivative operator*(double scalar) const {
        DroneStateDerivative result;
        result.positionDot = positionDot * scalar;
        result.velocityDot = velocityDot * scalar;
        result.orientationDot =
            Eigen::Quaterniond(orientationDot.w() * scalar, orientationDot.x() * scalar,
                               orientationDot.y() * scalar, orientationDot.z() * scalar);
        result.angularVelocityDot = angularVelocityDot * scalar;
        result.motorUpdateInfo = motorUpdateInfo * scalar;
        return result;
    }
};

struct DroneStateVector {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angularVelocity;
    msg::ActuatorState actuatorState;

    DroneStateVector() : orientation{Eigen::Quaterniond::Identity()} {}

    DroneStateVector(const msg::DroneState& state, msg::ActuatorState actuatorState)
        : position{utils::toEigen(state.position)},
          velocity{utils::toEigen(state.velocity)},
          orientation{utils::toEigenQuat(state.orientation)},
          angularVelocity{utils::toEigen(state.angularVelocity)},
          actuatorState{actuatorState} {}

    // Operator overloads for RK4 calculations
    DroneStateVector operator+(const DroneStateDerivative& derivative) const {
        DroneStateVector result;
        result.position = position + derivative.positionDot;
        result.velocity = velocity + derivative.velocityDot;

        result.orientation.coeffs() = orientation.coeffs() + derivative.orientationDot.coeffs();

        result.angularVelocity = angularVelocity + derivative.angularVelocityDot;
        result.actuatorState.motorSpeeds =
            actuatorState.motorSpeeds + derivative.motorUpdateInfo.accel;
        result.actuatorState.motorTemperatures =
            actuatorState.motorTemperatures + derivative.motorUpdateInfo.tempDerivative;
        return result;
    }
};

class DynamicSimulator : public Node {
   public:
    DynamicSimulator(ContextView context, const Environment* environment);

   private:
    DroneStateDerivative computeStateDerivative(const DroneStateVector& state, double t);
    DroneStateVector rK4Step(const DroneStateVector& state, double dt);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> computeForcesAndTorques(
        const DroneStateVector& state);

    void updateDroneState();
    static void checkAndApplyConstraints(DroneStateVector& state, double minHeight,
                                         Eigen::Vector3d& thrust);

    const WindFieldBase* windField_;
    Subscriber<msg::Stamp> subscriberSimTime_;
    Publisher<sim::msg::DroneState> publisherDroneState_;

    Actuator actuator_;

    msg::DroneState latestDroneState_;
    const msg::DroneParameters droneParameters_;
    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d iInv_;
    double invMass_;
    Eigen::Matrix4d mixerQuad_{Eigen::Matrix4d::Zero()};
    Eigen::Matrix<double, 4, 6> mixerHexa_{Eigen::Matrix<double, 4, 6>::Zero()};
    Eigen::Matrix<double, 4, 8> mixerOcto_{Eigen::Matrix<double, 4, 8>::Zero()};
    std::size_t motorNumber_{4};
    std::optional<Vec3d> optHomePosition_;

    const double updateRateSeconds_{};
    const std::int32_t updateRateUs_{};
    msg::Stamp curStepUs_{-msg::secondsToStamp(1.0)};
    std::atomic<msg::Stamp> lastStepUs_{-msg::secondsToStamp(1.0)};

    mutable std::mutex mutex_;
};
}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_SERVICES_DYNAMIC_SIMULATOR_HPP