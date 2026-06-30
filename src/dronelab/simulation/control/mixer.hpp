#ifndef DRONELAB_SERVICES_MIXER_HPP_
#define DRONELAB_SERVICES_MIXER_HPP_

#include <mutex>

#include <Eigen/Dense>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"
#include "dronelab/api/simulation/control/actuator_commands.hpp"
#include "dronelab/api/simulation/control/control_commands.hpp"
#include "dronelab/api/simulation/dynamics/drone_params.hpp"

namespace dronelab::sim {

class Mixer : public Node {
   public:
    explicit Mixer(ContextView context);

   protected:
    void receivedControlCommands(const msg::ControlCommands& command);

    virtual void update(const msg::ControlCommands& command) const = 0;

    Subscriber<sim::msg::ControlCommands> subscriberControlCommands_;
    Publisher<sim::msg::ActuatorCommands> publisherActuatorCommands_;

    msg::ControlCommands lastControlCommands_;

    const msg::DroneParameters droneParameters_;
    double referenceMaxThrustPerMotor_;
    double invReferenceMaxThrustPerMotor_;
    double invThrustCoefficient_;

    bool initialized_{false};

    std::mutex mutex_;
};

class QuadMixer final : public Mixer {
   public:
    explicit QuadMixer(ContextView context);

   private:
    void update(const msg::ControlCommands& command) const override;

    Eigen::Matrix4d mixerInv_{};
};

class HexaMixer final : public Mixer {
   public:
    explicit HexaMixer(ContextView context);

   private:
    void update(const msg::ControlCommands& command) const override;

    Eigen::Matrix<double, 6, 4> mixerPseudoInv_{};
};

class OctoMixer final : public Mixer {
   public:
    explicit OctoMixer(ContextView context);

   private:
    void update(const msg::ControlCommands& command) const override;

    Eigen::Matrix<double, 8, 4> mixerPseudoInv_{};
};

}  // namespace dronelab::sim

#endif  // DRONELAB_SERVICES_MIXER_HPP_