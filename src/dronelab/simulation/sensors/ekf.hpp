#ifndef DRONELAB_SIMULATION_SENSORS_EKF_HPP
#define DRONELAB_SIMULATION_SENSORS_EKF_HPP

#include <mutex>

#include <Eigen/Dense>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/data/constants.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/simulation/dynamics/drone_params.hpp"
#include "dronelab/api/simulation/sensors/barometer.hpp"
#include "dronelab/api/simulation/sensors/estimate.hpp"
#include "dronelab/api/simulation/sensors/gps.hpp"
#include "dronelab/api/simulation/sensors/imu.hpp"
#include "dronelab/api/simulation/sensors/magnetometer.hpp"
#include "dronelab/simulation/sensors/estimator.hpp"

namespace dronelab::sim {

class Ekf : public Estimator {
   public:
    explicit Ekf(ContextView context);

   private:
    void processVirtualImu(const msg::VirtualImuData& data);
    void processGps(const msg::GpsData& data) final;
    void processMag(const msg::MagnetometerData& data) final;
    void processBaro(const msg::BarometerData& data) final;

    void propagate(const msg::VirtualImuData& data) final;
    UpdateStatus update(const msg::GpsData& data) final;
    UpdateStatus update(const msg::BarometerData& data) final;
    UpdateStatus update(const msg::MagnetometerData& data) final;

    void updateCovariance(double dt, const Eigen::Vector3d& correctedDeltaTheta,
                          const Eigen::Vector3d& correctedDeltaV);

    Subscriber<sim::msg::VirtualImuData> subVirtualImuData_;

    msg::Stamp lastImuSimStamp_{-msg::secondsToStamp(100.0)};

    const Eigen::Vector3d gravity_{0, 0, constants::kGravity};  // m/s^2
    msg::DroneParameters droneParams_;
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

#endif  // DRONELAB_SIMULATION_SENSORS_EKF_HPP