#include "dronelab/simulation/sensors/estimator.hpp"

#include <spdlog/spdlog.h>

#include "dronelab/api/core/context.hpp"
#include "dronelab/api/core/node.hpp"
#include "dronelab/api/core/pub_sub.hpp"

namespace dronelab::sim {

Estimator::Estimator(ContextView context, NodeType type)
    : Node{context, type},
      subscriberGps_{context, *this, TopicKey::kGpsData},
      subscriberBarometer_{context, *this, TopicKey::kBarometerData},
      subscriberMagnetometer_{context, *this, TopicKey::kMagnetometerData},
      publisherStateEstimate_{context, *this, TopicKey::kStateEstimate} {
    subscriberGps_.subscribe([this](const auto& data) -> void { processGps(*data); });
    subscriberBarometer_.subscribe([this](const auto& data) -> void { processBaro(*data); });
    subscriberMagnetometer_.subscribe([this](const auto& data) -> void { processMag(*data); });
}

void Estimator::publishStateEstimate() {
    publisherStateEstimate_.publish(stateEstimate_);
}

msg::StateEstimate Estimator::getStateEstimate() const {
    return stateEstimate_;
}

}  // namespace dronelab::sim