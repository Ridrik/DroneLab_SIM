#ifndef DRONELAB_SIMULATION_ENVIRONMENT_WIND_FIELD_HPP
#define DRONELAB_SIMULATION_ENVIRONMENT_WIND_FIELD_HPP

#include <Eigen/Dense>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/utils/vec.hpp"
#include "dronelab/api/simulation/environment/wind.hpp"
#include "dronelab/api/simulation/environment/wind_field.hpp"

namespace dronelab::sim {

class WindField final : public WindFieldBase {
   public:
    explicit WindField(ContextView context);

    WindField(const WindField&) = delete;
    WindField& operator=(const WindField&) = delete;
    WindField(WindField&&) = delete;
    WindField& operator=(WindField&&) = delete;

    ~WindField() = default;

    [[nodiscard]] Vec3d getWindAt(const Vec3d& position, double time) const override;

   private:
    msg::WindEnvConfig env_;
    // Wind computation for each type
    [[nodiscard]] static Eigen::Vector3d constantWind(const msg::WindRegion& region);
    [[nodiscard]] static Eigen::Vector3d gradientWind(const msg::WindRegion& region,
                                                      const Vec3d& pos);
    [[nodiscard]] static Eigen::Vector3d gustyWind(const msg::WindRegion& region, double time);
    [[nodiscard]] static Eigen::Vector3d swirlWind(const msg::WindRegion& region, const Vec3d& pos);
    [[nodiscard]] static Eigen::Vector3d turbulentWind(const msg::WindRegion& region,
                                                       const Vec3d& pos, double time);
    [[nodiscard]] static Eigen::Vector3d comboWind(const msg::WindRegion& region, const Vec3d& pos,
                                                   double time);
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SIMULATION_ENVIRONMENT_WIND_FIELD_HPP