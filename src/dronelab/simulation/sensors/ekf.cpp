
#include "dronelab/simulation/sensors/ekf.hpp"

#include <cmath>
#include <mutex>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/sensors/barometer.hpp"
#include "dronelab/api/simulation/sensors/estimate.hpp"
#include "dronelab/api/simulation/sensors/gps.hpp"
#include "dronelab/api/simulation/sensors/imu.hpp"
#include "dronelab/api/simulation/sensors/magnetometer.hpp"
#include "dronelab/simulation/sensors/estimator.hpp"
#include "dronelab/simulation/utils/eigen_adapter.hpp"

namespace {

inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d skew;
    skew << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return skew;
}

}  // namespace

namespace dronelab::sim {

Ekf::Ekf(ContextView context)
    : Estimator{context, NodeType::kEKF},
      subVirtualImuData_{context, *this, TopicKey::kVirtualImuData},
      droneParams_{context.getConfig().getDroneParameters()},
      ekfConfig_{context.getConfig().getEkfConfig()},
      barNoiseCov_{std::pow(ekfConfig_.barParams.noiseStddev, 2)} {
    inertiaInv_ =
        Eigen::Map<const Eigen::Vector3d>(droneParams_.inertiaDiag.data()).asDiagonal().inverse();
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
    Eigen::Map<const Eigen::Matrix<double, msg::kEstimatorStateSize, 1>> vec(
        initialStdDeviationDiagonal.data());
    Eigen::Map<Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize>>(
        stateEstimate_.covariance.data()) = (vec.array().square()).matrix().asDiagonal();

    subVirtualImuData_.subscribe([this](const auto& msg) { processVirtualImu(*msg); });
}

void Ekf::processVirtualImu(const msg::VirtualImuData& data) {
    propagate(data);
    publishStateEstimate();
}

/*void Ekf::processImu(const msg::ImuData& data) {
    const auto dt = msg::microsToSeconds<double>(data.simStamp - lastImuSimStamp_);
    const auto virtualImu = msg::VirtualImuData{.stamp = data.simStamp,
                                                .totalDt = dt,
                                                .deltaV = data.accel * dt,
                                                .deltaTheta = data.gyro * dt};
    propagate(virtualImu);
    publishStateEstimate();
}*/

void Ekf::processGps(const msg::GpsData& data) {
    update(data);
    publishStateEstimate();
}

void Ekf::processMag(const msg::MagnetometerData& data) {
    update(data);
    publishStateEstimate();
}

void Ekf::processBaro(const msg::BarometerData& data) {
    update(data);
    publishStateEstimate();
}

void Ekf::propagate(const msg::VirtualImuData& data) {
    SPDLOG_TRACE("[{}] On IMU Propagation, on lock acquisition.", name_);
    std::lock_guard lock{mutex_};

    const auto& dt = data.totalDt;
    SPDLOG_TRACE(
        "[{}] On IMU Propagation, lock acquired. Delta time between IMU measurements is {}", name_,
        dt);

    lastImuSimStamp_ = data.stamp;
    if (dt <= 1e-5 || dt > 1) [[unlikely]] {
        SPDLOG_DEBUG("[{}] Ignoring IMU data with dt = {:.6f} seconds", name_, dt);
        return;  // Ignore zero/negative/large time jumps
    }

    const Eigen::Vector3d& position = utils::toEigen(stateEstimate_.position);
    const Eigen::Vector3d& velocity = utils::toEigen(stateEstimate_.velocity);
    const Eigen::Quaterniond& attitude = utils::toEigenQuat(stateEstimate_.orientation);

    const Eigen::Vector3d& gyroBias = utils::toEigen(stateEstimate_.gyroBias);
    const Eigen::Vector3d& accelBias = utils::toEigen(stateEstimate_.accelBias);

    // Attitude integration
    Eigen::Vector3d deltaAngle = utils::toEigen(data.deltaTheta) - gyroBias * dt;
    Eigen::Quaterniond deltaQ;
    const double angleSq = deltaAngle.squaredNorm();
    if (angleSq > 1e-12) {
        const auto angle = std::sqrt(angleSq);
        Eigen::Vector3d axis = deltaAngle / angle;
        deltaQ = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    } else {
        // First-order small-angle approximation
        deltaQ = Eigen::Quaterniond(1.0, 0.5 * deltaAngle.x(), 0.5 * deltaAngle.y(),
                                    0.5 * deltaAngle.z());
    }
    Eigen::Quaterniond newAttitude =
        (attitude * deltaQ).normalized();  // Apply incremental rotation

    const Eigen::Vector3d correctedImuDeltaV = utils::toEigen(data.deltaV) - accelBias * dt;
    const Eigen::Vector3d worldDeltaV =
        attitude.toRotationMatrix() * (correctedImuDeltaV) + gravity_ * dt;

    SPDLOG_TRACE("[{}] World DeltaV: {} {} {}", name_, worldDeltaV.x(), worldDeltaV.y(),
                 worldDeltaV.z());

    // Linear motion integration
    Eigen::Vector3d newVelocity = velocity + worldDeltaV;  // euler forward integration
    Eigen::Vector3d newPosition =
        position + 0.5 * (velocity + newVelocity) * dt;  // trapezoidal rule

    const double minHeight = -0.5 * droneParams_.bodyDimensions.z();
    if (newPosition.z() > minHeight) {
        newPosition.z() = minHeight;
        newVelocity.z() = std::min(newVelocity.z(), 0.0);
    }

    updateCovariance(dt, deltaAngle, correctedImuDeltaV);

    stateEstimate_.position = utils::fromEigen(newPosition);
    stateEstimate_.velocity = utils::fromEigen(newVelocity);
    stateEstimate_.orientation = utils::fromEigen(newAttitude);
    stateEstimate_.angularVelocity =
        utils::fromEigen(static_cast<Eigen::Vector3d>(deltaAngle / dt));
    stateEstimate_.simStamp = data.stamp;
    stateEstimate_.sensorID = msg::SensorID::kIMU;
}

void Ekf::updateCovariance(double dt, const Eigen::Vector3d& correctedDeltaTheta,
                           const Eigen::Vector3d& correctedDeltaV) {
    // Get current orientation (quaternion)
    const auto q = utils::toEigenQuat(stateEstimate_.orientation);
    const Eigen::Matrix3d R = q.toRotationMatrix();

    // Construct discrete Jacobian F (1st order discrete from continuous jacobian) (15x15)
    Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize> F =
        Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize>::Zero();

    F.block<3, 3>(+EkfIdx::kPos, +EkfIdx::kPos) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(+EkfIdx::kPos, +EkfIdx::kVel) = Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(+EkfIdx::kVel, +EkfIdx::kVel) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(+EkfIdx::kVel, +EkfIdx::kAngle) = -R * skewSymmetric(correctedDeltaV);
    F.block<3, 3>(+EkfIdx::kVel, +EkfIdx::kAccelBias) = -R * dt;
    // -R * dt;  // Alternative: -skewSimmetric(corrected_gyro) * dt;
    F.block<3, 3>(+EkfIdx::kAngle, +EkfIdx::kAngle) =
        Eigen::Matrix3d::Identity() - skewSymmetric(correctedDeltaTheta);
    F.block<3, 3>(+EkfIdx::kAngle, +EkfIdx::kGyroBias) = -Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(+EkfIdx::kGyroBias, +EkfIdx::kGyroBias) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(+EkfIdx::kAccelBias, +EkfIdx::kAccelBias) = Eigen::Matrix3d::Identity();

    // Process noise
    Eigen::Matrix<double, msg::kEstimatorStateSize, 12> G =
        Eigen::Matrix<double, msg::kEstimatorStateSize, 12>::Zero();
    G.block<3, 3>(+EkfIdx::kVel, +ImuIdx::kAccelCov) = -R;  // accel_noise -> velocity
    // Attitude error: affected by gyroscope noise
    G.block<3, 3>(+EkfIdx::kAngle, +ImuIdx::kGyroCov) =
        -Eigen::Matrix3d::Identity();  // gyro_noise ->
                                       // attitude_error
    // Gyro bias: affected by gyro bias random walk
    G.block<3, 3>(+EkfIdx::kGyroBias, +ImuIdx::kGyroWalkCov) =
        Eigen::Matrix3d::Identity();  // gyro_bias_drift -> gyro_bias
    // Accel bias: affected by accel bias random walk
    G.block<3, 3>(+EkfIdx::kAccelBias, +ImuIdx::kAccelWalkCov) =
        Eigen::Matrix3d::Identity();  // accel_bias_drift -> accel_bias

    // Review
    const Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize> Q =
        G * inputNoiseCov_ * G.transpose() * dt;

    // Propagate covariance
    Eigen::Map<Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize>>(
        stateEstimate_.covariance.data()) =
        F *
            Eigen::Map<Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize>>(
                stateEstimate_.covariance.data()) *
            F.transpose() +
        Q;
}

Estimator::UpdateStatus Ekf::update(const msg::GpsData& data) {
    std::lock_guard lock{mutex_};
    stateEstimate_.simStamp = data.simStamp;
    stateEstimate_.sensorID = msg::SensorID::kGPS;
    SPDLOG_TRACE("[{}] On GPS Update, lock acquired.", name_);

    // Residual
    Eigen::VectorXd residual;  // size depends on measurement
    residual.resize(data.hasVelocity ? 6 : 3);

    residual.segment<3>(0) = utils::toEigen(data.position - stateEstimate_.position);

    if (data.hasVelocity) {
        residual.segment<3>(3) = utils::toEigen(data.velocity - stateEstimate_.velocity);
    }
    // H matrix
    Eigen::MatrixXd H;
    if (data.hasVelocity) {
        H = Eigen::Matrix<double, 6, 15>::Zero();
        H.block<3, 3>(0, EkfIdx::kPos) = Eigen::Matrix3d::Identity();  // pos w.r.t. position
        H.block<3, 3>(3, EkfIdx::kVel) = Eigen::Matrix3d::Identity();  // vel w.r.t. velocity
    } else {
        H = Eigen::Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, EkfIdx::kPos) = Eigen::Matrix3d::Identity();  // pos only
    }
    // Measurement noise
    Eigen::MatrixXd R;
    if (data.hasVelocity) {
        R = Eigen::Matrix<double, 6, 6>::Zero();
        R.block<3, 3>(0, 0) = gpsPositionNoiseCov_;
        R.block<3, 3>(3, 3) = gpsVelocityNoiseCov_;
    } else {
        R = gpsPositionNoiseCov_;
    }

    // Kalman gain
    auto P = Eigen::Map<Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize>>(
        stateEstimate_.covariance.data());
    const Eigen::MatrixXd S = H * P * H.transpose() + R;
    const double mahalanobis = residual.transpose() * S.ldlt().solve(residual);
    constexpr double kGateThreshold3D{11.345};  // 99% confidence for 3D (chi-squared)
    constexpr double kGateThreshold6D{16.812};
    const auto& gateThreshold = data.hasVelocity ? kGateThreshold6D : kGateThreshold3D;
    /*if (mahalanobis > gateThreshold) {
        SPDLOG_WARN("[{}] GPS update rejected: innovation too large", name_);
        return Estimator::UpdateStatus::kMahalanobisGate;
    }*/

    // Correction K = P*H.T*S^-1 -> K.T = S^-T *H*P.T. S,P == Symmetric -> S*K.T = H*P -> K.T =
    // S.ldlt().solve(H*P)
    const Eigen::MatrixXd K = S.ldlt().solve(H * P).transpose();
    const Eigen::Matrix<double, 15, 1> deltaX = K * residual;
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE
    std::ostringstream oss;
    oss << deltaX.transpose();
    SPDLOG_TRACE("[{}] GPS correction deltaX: {}", name_, oss.str());
#endif

    // Inject
    stateEstimate_.position += utils::fromEigen(deltaX.segment<3>(+EkfIdx::kPos));
    stateEstimate_.velocity += utils::fromEigen(deltaX.segment<3>(+EkfIdx::kVel));
    Eigen::Vector3d deltaTheta = deltaX.segment<3>(+EkfIdx::kAngle);
    Eigen::Quaterniond deltaQ;
    const double angle = deltaTheta.norm();
    if (angle > 1e-12) {
        Eigen::Vector3d axis = deltaTheta / angle;
        deltaQ = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    } else {
        deltaQ = Eigen::Quaterniond(1.0, 0.5 * deltaTheta.x(), 0.5 * deltaTheta.y(),
                                    0.5 * deltaTheta.z());
    }
    const Eigen::Quaterniond newAttitude = (utils::toEigenQuat(stateEstimate_.orientation) * deltaQ)
                                               .normalized();  // Apply incremental rotation
    stateEstimate_.orientation = utils::fromEigen(newAttitude);
    stateEstimate_.gyroBias += deltaX.segment<3>(+EkfIdx::kGyroBias);
    stateEstimate_.accelBias += deltaX.segment<3>(+EkfIdx::kAccelBias);

    // Covariance update (Joseph form)
    static const Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    Eigen::Matrix<double, 15, 15> ikh = I - K * H;
    P = ikh * P * ikh.transpose() + K * R * K.transpose();
    P = 0.5 * (P + P.transpose());
    Eigen::Map<Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize>>(
        stateEstimate_.covariance.data()) = P;
    return Estimator::UpdateStatus::kValid;
}

Estimator::UpdateStatus Ekf::update(const msg::BarometerData& data) {
    std::lock_guard lock{mutex_};
    stateEstimate_.simStamp = data.simStamp;
    stateEstimate_.sensorID = msg::SensorID::kBarometer;
    SPDLOG_TRACE("[{}] updating with Barometer measurement", name_);
    const auto& measuredPressure = data.pressureMeasure;
    const double estimatedHeight = -stateEstimate_.position.z();
    constexpr double kExp{5.255};
    constexpr double kScale{44330.0};
    const double inner = 1.0 - (estimatedHeight / kScale);
    const double predictedPressure = ekfConfig_.barParams.seaLevelPresure * std::pow(inner, kExp);
    const double residual = measuredPressure - predictedPressure;
    Eigen::RowVector<double, 15> H = Eigen::RowVector<double, 15>::Zero();
    H[+EkfIdx::kPos + 2] =
        (kExp / kScale) * ekfConfig_.barParams.seaLevelPresure * std::pow(inner, kExp - 1.0);

    const auto& R = barNoiseCov_;
    auto P = Eigen::Map<Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize>>(
        stateEstimate_.covariance.data());
    const double hVal = H[+EkfIdx::kPos + 2];
    const Eigen::Matrix<double, msg::kEstimatorStateSize, 1> pHt = P.col(+EkfIdx::kPos + 2) * hVal;
    const double S = hVal * P(+EkfIdx::kPos + 2, +EkfIdx::kPos + 2) * hVal + R;
    const double invS = 1.0 / S;

    const double mahalanobis = residual * residual * invS;
    constexpr double kGateThreshold = 6.635;

    /*if (mahalanobis > kGateThreshold) {
        SPDLOG_WARN("[{}] Barometer update rejected: innovation too large", name_);
        return Estimator::UpdateStatus::kMahalanobisGate;
    }*/
    const Eigen::MatrixXd K = pHt * invS;

    // Correction
    const Eigen::Matrix<double, 15, 1> deltaX = K * residual;
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE
    std::ostringstream oss;
    oss << deltaX.transpose();
    SPDLOG_TRACE("[{}] Barometer correction deltaX: {}", name_, oss.str());
#endif

    // Inject
    const Eigen::Vector3d deltaTheta = deltaX.segment<3>(+EkfIdx::kAngle);
    Eigen::Quaterniond deltaQ;
    const double angleSq = deltaTheta.squaredNorm();
    if (angleSq > 1e-12) {
        const double angle = std::sqrt(angleSq);
        Eigen::Vector3d axis = deltaTheta / angle;
        deltaQ = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    } else {
        // First-order approximation for numerical stability
        deltaQ = Eigen::Quaterniond(1.0, 0.5 * deltaTheta.x(), 0.5 * deltaTheta.y(),
                                    0.5 * deltaTheta.z());
    }
    const Eigen::Quaterniond newAttitude = (utils::toEigenQuat(stateEstimate_.orientation) * deltaQ)
                                               .normalized();  // Apply incremental rotation
    stateEstimate_.orientation = utils::fromEigen(newAttitude);
    stateEstimate_.position += deltaX.segment<3>(+EkfIdx::kPos);
    stateEstimate_.velocity += deltaX.segment<3>(+EkfIdx::kVel);
    stateEstimate_.gyroBias += deltaX.segment<3>(+EkfIdx::kGyroBias);
    stateEstimate_.accelBias += deltaX.segment<3>(+EkfIdx::kAccelBias);

    // Covariance update (Joseph form)
    static const Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    Eigen::Matrix<double, 15, 15> ikh = I - K * H;
    P = ikh * P * ikh.transpose() + K * R * K.transpose();
    Eigen::Map<Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize>>(
        stateEstimate_.covariance.data()) = 0.5 * (P + P.transpose());
    return Estimator::UpdateStatus::kValid;
}

Estimator::UpdateStatus Ekf::update(const msg::MagnetometerData& data) {
    std::lock_guard lock{mutex_};
    stateEstimate_.simStamp = data.simStamp;
    stateEstimate_.sensorID = msg::SensorID::kMagnetometer;
    SPDLOG_TRACE("[{}] updating with Magnetometer measurement", name_);

    const auto oriEigen = utils::toEigenQuat(stateEstimate_.orientation);
    const Eigen::Matrix3d rWorldToBodyEst = oriEigen.toRotationMatrix().transpose();
    const Eigen::Vector3d mPred =
        rWorldToBodyEst * utils::toEigen(ekfConfig_.magParams.earthMagneticField);
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, +EkfIdx::kAngle) = skewSymmetric(mPred);
    const Eigen::Vector3d residual = utils::toEigen(data.magneticField) - mPred;
    const auto& R = magNoiseCov_;
    // Kalman gain
    auto P = Eigen::Map<Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize>>(
        stateEstimate_.covariance.data());
    const Eigen::MatrixXd S = H * P * H.transpose() + R;
    const double mahalanobis = residual.transpose() * S.ldlt().solve(residual);
    constexpr double kGateThreshold = 11.345;  // 99% confidence for 3D (chi-squared)

    /*if (mahalanobis > kGateThreshold) {
        SPDLOG_WARN("[{}] Magnetometer update rejected: innovation too large", name_);
        return Estimator::UpdateStatus::kMahalanobisGate;
    }*/

    const Eigen::MatrixXd K = S.ldlt().solve(H * P).transpose();

    // Correction
    const Eigen::Matrix<double, 15, 1> deltaX = K * residual;
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE
    std::ostringstream oss;
    oss << deltaX.transpose();
    SPDLOG_TRACE("[{}] Magnetometer correction deltaX: {}", name_, oss.str());
#endif

    // Inject
    const Eigen::Vector3d deltaTheta = deltaX.segment<3>(+EkfIdx::kAngle);
    Eigen::Quaterniond deltaQ;
    const double angle = deltaTheta.norm();
    if (angle > 1e-12) {
        Eigen::Vector3d axis = deltaTheta / angle;
        deltaQ = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    } else {
        // First-order approximation for numerical stability
        deltaQ = Eigen::Quaterniond(1.0, 0.5 * deltaTheta.x(), 0.5 * deltaTheta.y(),
                                    0.5 * deltaTheta.z());
    }
    Eigen::Quaterniond newAttitude =
        (oriEigen * deltaQ).normalized();  // Apply incremental rotation
    stateEstimate_.orientation = utils::fromEigen(newAttitude);
    stateEstimate_.gyroBias += deltaX.segment<3>(+EkfIdx::kGyroBias);
    stateEstimate_.accelBias += deltaX.segment<3>(+EkfIdx::kAccelBias);

    // Covariance update (Joseph form)
    static const Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    Eigen::Matrix<double, 15, 15> ikh = I - K * H;
    P = ikh * P * ikh.transpose() + K * R * K.transpose();
    Eigen::Map<Eigen::Matrix<double, msg::kEstimatorStateSize, msg::kEstimatorStateSize>>(
        stateEstimate_.covariance.data()) = 0.5 * (P + P.transpose());
    return Estimator::UpdateStatus::kValid;
}

}  // namespace dronelab::sim