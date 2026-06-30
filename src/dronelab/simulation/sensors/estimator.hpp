#ifndef DRONELAB_SIMULATION_SENSORS_ESTIMATOR_ESTIMATOR_HPP
#define DRONELAB_SIMULATION_SENSORS_ESTIMATOR_ESTIMATOR_HPP

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/sensors/barometer.hpp"
#include "dronelab/api/simulation/sensors/estimate.hpp"
#include "dronelab/api/simulation/sensors/gps.hpp"
#include "dronelab/api/simulation/sensors/imu.hpp"
#include "dronelab/api/simulation/sensors/magnetometer.hpp"

namespace dronelab::sim {

// For a 2-sigma (approx 95%) ellipsoid
constexpr double kSigma = 2.447;

enum EkfIdx : std::uint8_t { kPos = 0, kVel = 3, kAngle = 6, kGyroBias = 9, kAccelBias = 12 };
enum ImuIdx : std::uint8_t { kGyroCov = 0, kAccelCov = 3, kGyroWalkCov = 6, kAccelWalkCov = 9 };

class Estimator : public Node {
   public:
    Estimator(ContextView context, NodeType type);

    [[nodiscard]] msg::StateEstimate getStateEstimate() const;

   protected:
    virtual void processGps(const msg::GpsData& data) = 0;
    virtual void processMag(const msg::MagnetometerData& data) = 0;
    virtual void processBaro(const msg::BarometerData& data) = 0;

    virtual void propagate(const msg::VirtualImuData& data) = 0;
    enum class UpdateStatus : std::uint8_t { kValid, kMahalanobisGate };
    virtual UpdateStatus update(const msg::GpsData& data) = 0;
    virtual UpdateStatus update(const msg::BarometerData& data) = 0;
    virtual UpdateStatus update(const msg::MagnetometerData& data) = 0;

    void publishStateEstimate();

    Subscriber<sim::msg::GpsData> subscriberGps_;
    Subscriber<sim::msg::BarometerData> subscriberBarometer_;
    Subscriber<sim::msg::MagnetometerData> subscriberMagnetometer_;
    Publisher<sim::msg::StateEstimate> publisherStateEstimate_;

    msg::StateEstimate stateEstimate_;
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_SENSORS_ESTIMATOR_ESTIMATOR_HPP