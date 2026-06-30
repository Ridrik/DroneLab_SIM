
#include "dronelab/simulation/environment/wind_field.hpp"

#include <numbers>

#include <Eigen/Dense>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/simulation/environment/wind_field.hpp"
#include "dronelab/dronelab_sim_export.h"
#include "dronelab/simulation/utils/eigen_adapter.hpp"

namespace dronelab::sim {

namespace {

[[nodiscard]] bool contains(const msg::WindRegion& region, const Vec3d& point) {
    switch (region.shape) {
        case msg::RegionShape::kBox: {
            const auto minCorner = utils::toEigen(region.position - region.size * 0.5);
            const auto maxCorner = utils::toEigen(region.position + region.size * 0.5);
            const auto pEigen = utils::toEigen(point);
            return (pEigen.array() >= minCorner.array()).all() &&
                   (pEigen.array() <= maxCorner.array()).all();
        }
        case msg::RegionShape::kSphere: {
            return utils::toEigen(point - region.position).norm() <= (region.size.x() * 0.5);
        }
        case msg::RegionShape::kCylinder: {
            const double dxz =
                Eigen::Vector2d(point.x() - region.position.x(), point.z() - region.position.z())
                    .norm();
            const bool withinRadius = dxz <= (region.size.x() * 0.5);
            const bool withinHeight =
                std::abs(point.y() - region.position.y()) <= (region.size.y() * 0.5);
            return withinRadius && withinHeight;
        }
    }
    return false;  // fallback
}

}  // namespace

WindField::WindField(ContextView context) : env_{context.getConfig().getWindEnvConfig()} {}

Vec3d WindField::getWindAt(const Vec3d& position, double time) const {
    Eigen::Vector3d totalWind = Eigen::Vector3d::Zero();

    for (const auto& region : env_.regions) {
        if (!contains(region, position)) {
            continue;
        }

        Eigen::Vector3d windVec{Eigen::Vector3d::Zero()};
        switch (region.type) {
            case msg::WindFieldType::kConstant:
                windVec = constantWind(region);
                break;
            case msg::WindFieldType::kGradient:
                windVec = gradientWind(region, position);
                break;
            case msg::WindFieldType::kGusty:
                windVec = gustyWind(region, time);
                break;
            case msg::WindFieldType::kSwirl:
                windVec = swirlWind(region, position);
                break;
            case msg::WindFieldType::kTurbulent:
                windVec = turbulentWind(region, position, time);
                break;
            case msg::WindFieldType::kCombo:
                windVec = comboWind(region, position, time);
                break;
        }

        totalWind += windVec;
    }

    return utils::fromEigen(totalWind);
}

Eigen::Vector3d WindField::constantWind(const msg::WindRegion& region) {
    return utils::toEigen(region.params.constantWind);
}

Eigen::Vector3d WindField::gradientWind(const msg::WindRegion& region, const Vec3d& pos) {
    const double speed = region.params.gradientBaseSpeed + region.params.gradientSlope * pos.z();
    return speed * Eigen::Vector3d(1.0, 0.0, 0.0);
}

Eigen::Vector3d WindField::gustyWind(const msg::WindRegion& region, double time) {
    const double gust = region.params.gustyAmplitude *
                        std::sin(2.0 * std::numbers::pi * region.params.gustyFrequency * time);
    return (region.params.gustyBaseSpeed + gust) *
           utils::toEigen(region.params.gustyDirection).normalized();
}

Eigen::Vector3d WindField::swirlWind(const msg::WindRegion& region, const Vec3d& pos) {
    const Eigen::Vector2d pXy(pos.x() - region.position.x(), pos.y() - region.position.y());
    const double r = pXy.norm();
    Eigen::Vector2d tangential(-pXy.y(), pXy.x());
    tangential.normalize();

    const double swirlSpeed = region.params.swirlStrength / (1.0 + region.params.swirlDecay * r);
    return swirlSpeed * Eigen::Vector3d(tangential.x(), tangential.y(), 0.0);
}

Eigen::Vector3d WindField::turbulentWind(const msg::WindRegion& region, const Vec3d& pos,
                                         double time) {
    const double u = std::sin(region.params.turbulentScaleX * pos.x() +
                              time * region.params.turbulentTimeScaleX);
    const double v = std::sin(region.params.turbulentScaleY * pos.y() +
                              time * region.params.turbulentTimeScaleY);
    const double w = std::sin(region.params.turbulentScaleZ * pos.z() +
                              time * region.params.turbulentTimeScaleZ);
    return {u, v, w};
}

Eigen::Vector3d WindField::comboWind(const msg::WindRegion& region, const Vec3d& pos, double time) {
    return constantWind(region) +
           region.params.comboTurbulenceFactor * turbulentWind(region, pos, time) +
           region.params.comboSwirlFactor * swirlWind(region, pos);
}

}  // namespace dronelab::sim

extern "C" DRONELAB_SIM_EXPORT dronelab::sim::WindFieldBase* createWindField(
    dronelab::ContextView context) {
    return new dronelab::sim::WindField{context};  // NOLINT
}

extern "C" DRONELAB_SIM_EXPORT void deleteWindField(dronelab::sim::WindFieldBase* base) {
    delete base;  // NOLINT
}