#ifndef DRONELAB_SIMULATION_DATA_DYNAMICS_ACTUATOR_DATA_HPP
#define DRONELAB_SIMULATION_DATA_DYNAMICS_ACTUATOR_DATA_HPP

#include "dronelab/api/core/utils/vec.hpp"
#include "dronelab/api/simulation/common/config.hpp"
#include "dronelab/api/simulation/dynamics/actuator_data.hpp"

namespace dronelab::sim::msg {

struct MotorUpdateInfo {
    Vecd<kActuatorMaxNum> accel;
    Vecd<kActuatorMaxNum> tempDerivative;
    MotorUpdateInfo operator+(const MotorUpdateInfo& other) const {
        return MotorUpdateInfo{.accel = accel + other.accel,
                               .tempDerivative = tempDerivative + other.tempDerivative};
    }

    MotorUpdateInfo operator*(double scalar) const {
        return MotorUpdateInfo{.accel = accel * scalar, .tempDerivative = tempDerivative * scalar};
    }
};

constexpr void add(ActuatorState& src, const MotorUpdateInfo& derivative) noexcept {
    static_assert(src.motorSpeeds.size() == src.motorTemperatures.size());
    for (std::size_t i{0}; i < src.motorSpeeds.size(); ++i) {
        src.motorSpeeds[i] += derivative.accel[i];
        src.motorTemperatures[i] += derivative.tempDerivative[i];
    }
}

}  // namespace dronelab::sim::msg

#endif  // DRONELAB_SIMULATION_DATA_DYNAMICS_ACTUATOR_DATA_HPP