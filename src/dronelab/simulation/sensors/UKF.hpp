#ifndef DRONELAB_SIMULATION_SENSORS_UKF_HPP
#define DRONELAB_SIMULATION_SENSORS_UKF_HPP

#include "dronelab/api/core/context.hpp"
#include "dronelab/simulation/sensors/estimator.hpp"

namespace dronelab::sim {

class UKF : public Estimator {
   public:
    explicit UKF(ContextView context);

   private:
    void propagate(const msg::ImuData& data) override;
    void update(const msg::GpsData& data) override;
    void update(const msg::BarometerData& data) override;
    void update(const msg::MagnetometerData& data) override;

    void updateCovariance(double dt, const Eigen::Vector3d& correctedGyro,
                          const Eigen::Vector3d& correctedAccel);

    msg::Stamp lastImuSimStamp_{-msg::secondsToStamp(100.0)};

    const Eigen::Vector3d gravity_{0, 0, constants::kGravity};  // Gravity constant in m/s^2
    msg::DroneParameters droneParams_;  // Drone parameters (mass, inertia, etc.)
    msg::EKFConfig ekfConfig_;

    Eigen::Matrix3d gpsPositionNoiseCov_;
    Eigen::Matrix3d gpsVelocityNoiseCov_;
    Eigen::Matrix<double, 12, 12> inputNoiseCov_;
    Eigen::Matrix3d magNoiseCov_;
    double barNoiseCov_;
    Eigen::Matrix3d inertiaInv_;

    mutable std::mutex mutex_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_SENSORS_UKF_HPP