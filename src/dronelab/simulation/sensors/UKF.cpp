#include "dronelab/simulation/sensors/UKF.hpp"

namespace dronelab::sim {

UKF::UKF(ContextView context)
    : Estimator{context, NodeType::kEKF},
      droneParams_{context.getConfig().getDroneParameters()},
      ekfConfig_{context.getConfig().getEkfConfig()},
      barNoiseCov_{std::pow(ekfConfig_.barParams.noiseStddev, 2)} {
    inertiaInv_ = droneParams_.inertiaDiag.inverse();
    stateEstimate_.position = ekfConfig_.initialPositionEstimate;

    const auto& gpsNoiseParams = ekfConfig_.gpsParams;
    gpsPositionNoiseCov_.setZero();
    gpsPositionNoiseCov_(0, 0) = std::pow(gpsNoiseParams.positionNoiseStddevXy, 2);
    gpsPositionNoiseCov_(1, 1) = std::pow(gpsNoiseParams.positionNoiseStddevXy, 2);
    gpsPositionNoiseCov_(2, 2) = std::pow(gpsNoiseParams.positionNoiseStddevZ, 2);

    gpsVelocityNoiseCov_ =
        Eigen::Matrix3d::Identity() * std::pow(gpsNoiseParams.velocityNoiseStddev, 2);

    const auto& magParams = ekfConfig_.magParams;
    magNoiseCov_.setZero();
    magNoiseCov_(0, 0) = std::pow(magParams.noiseStddev[0], 2);
    magNoiseCov_(1, 1) = std::pow(magParams.noiseStddev[1], 2);
    magNoiseCov_(2, 2) = std::pow(magParams.noiseStddev[2], 2);

    // === Process noise ===
    const auto& np = ekfConfig_.imuParams;  // Alias for brevity
    inputNoiseCov_ = Eigen::Matrix<double, 12, 12>::Zero();

    // IMU measurement noise (6x6 block)
    inputNoiseCov_.block<3, 3>(ImuIdx::kGyroCov, ImuIdx::kGyroCov) =
        std::pow(np.gyroMeasurementNoiseStddev, 2) * Eigen::Matrix3d::Identity();
    inputNoiseCov_.block<3, 3>(ImuIdx::kAccelCov, ImuIdx::kAccelCov) =
        std::pow(np.accelMeasurementNoiseStddev, 2) * Eigen::Matrix3d::Identity();

    // Bias random walk noise (6x6 block)
    inputNoiseCov_.block<3, 3>(ImuIdx::kGyroWalkCov, ImuIdx::kGyroWalkCov) =
        std::pow(np.gyroBiasRandomWalkStddev, 2) * Eigen::Matrix3d::Identity();
    inputNoiseCov_.block<3, 3>(ImuIdx::kAccelWalkCov, ImuIdx::kAccelWalkCov) =
        std::pow(np.accelBiasRandomWalkStddev, 2) * Eigen::Matrix3d::Identity();

    const auto& initialStdDeviationDiagonal = ekfConfig_.initialStateUncertainty;
    Eigen::Map<const Eigen::Matrix<double, 15, 1>> vec(initialStdDeviationDiagonal.data());
    stateEstimate_.covariance = (vec.array().square()).matrix().asDiagonal();
}

}  // namespace dronelab::sim