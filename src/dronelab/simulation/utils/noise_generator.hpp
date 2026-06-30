#ifndef DRONELAB_CORE_UTILS_NOISE_GENERATOR_HPP
#define DRONELAB_CORE_UTILS_NOISE_GENERATOR_HPP

#include <random>

#include "dronelab/api/core/exceptions.hpp"
#include "dronelab/api/core/utils/vec.hpp"

namespace dronelab::utils {

class NoiseGenerator {
   public:
    NoiseGenerator(double stddev, std::size_t seed)
        : dist_{0.0, stddev},
          gen_{std::mt19937{static_cast<std::random_device::result_type>(seed)}} {}

    auto sample() {
        return dist_(gen_);
    }
    auto sampleVec() {
        return Vec3d{dist_(gen_), dist_(gen_), dist_(gen_)};
    }

    void updateDist(double newStddev) {
        std::normal_distribution<double>::param_type newDist(0, newStddev);
        dist_.param(newDist);
    }

   private:
    std::normal_distribution<double> dist_;
    std::mt19937 gen_;
};

class EnhNoiseGenerator {
   public:
    EnhNoiseGenerator(double stddevXy, double stddevZ, std::size_t seed)
        : distXy_{0.0, stddevXy},
          distZ_{0.0, stddevZ},
          gen_{std::mt19937{static_cast<std::random_device::result_type>(seed)}} {}

    EnhNoiseGenerator(const EnhNoiseGenerator&) = delete;
    EnhNoiseGenerator& operator=(const EnhNoiseGenerator&) = delete;

    EnhNoiseGenerator(EnhNoiseGenerator&&) = default;
    EnhNoiseGenerator& operator=(EnhNoiseGenerator&&) = default;

    ~EnhNoiseGenerator() = default;

    auto sample() {
        return Vec3d{distXy_(gen_), distXy_(gen_), distZ_(gen_)};
    }

    void updateXY(double newStddevXy) {
        std::normal_distribution<double>::param_type newDistXy(0, newStddevXy);
        distXy_.param(newDistXy);
    }
    void updateZ(double newStddevZ) {
        std::normal_distribution<double>::param_type newDistZ(0, newStddevZ);
        distZ_.param(newDistZ);
    }

   private:
    std::normal_distribution<double> distXy_;
    std::normal_distribution<double> distZ_;
    std::mt19937 gen_;
};

class Enh3NoiseGenerator {
   public:
    Enh3NoiseGenerator(double stddevX, double stddevY, double stddevZ, std::size_t seed)
        : distX_{0.0, stddevX},
          distY_{0.0, stddevY},
          distZ_{0.0, stddevZ},
          gen_{std::mt19937{static_cast<std::random_device::result_type>(seed)}} {}

    Enh3NoiseGenerator(const Vec3d& stddev, std::size_t seed)
        : distX_{0.0, stddev.x()},
          distY_{0.0, stddev.y()},
          distZ_{0.0, stddev.z()},
          gen_{std::mt19937{static_cast<std::random_device::result_type>(seed)}} {}

    auto sample() {
        return Vec3d{distX_(gen_), distY_(gen_), distZ_(gen_)};
    }

    void updateCoord(std::uint8_t coord, double newStddev) {
        if (coord > 2) {
            throw exceptions::LogicError(
                std::string("[Enh3NoiseGenerator] Attempted to update invalid index ") +
                std::to_string(coord));
        }
        std::normal_distribution<double>::param_type newDist(0, newStddev);
        distRef_[coord]->param(newDist);
    }
    void updateX(double newStddevX) {
        std::normal_distribution<double>::param_type newDistX(0, newStddevX);
        distX_.param(newDistX);
    }
    void updateY(double newStddevY) {
        std::normal_distribution<double>::param_type newDistY(0, newStddevY);
        distY_.param(newDistY);
    }
    void updateZ(double newStddevZ) {
        std::normal_distribution<double>::param_type newDistZ(0, newStddevZ);
        distZ_.param(newDistZ);
    }

   private:
    std::normal_distribution<double> distX_;
    std::normal_distribution<double> distY_;
    std::normal_distribution<double> distZ_;
    std::array<std::normal_distribution<double>*, 3> distRef_{&distX_, &distY_, &distZ_};
    std::mt19937 gen_;
};

}  // namespace dronelab::utils

#endif  // DRONELAB_CORE_UTILS_NOISE_GENERATOR_HPP