#include "dronelab/simulation/version.h"

#include "dronelab/api/simulation/symbols.hpp"
#include "dronelab/dronelab_sim_export.h"

extern "C" DRONELAB_SIM_EXPORT dronelab::sim::Version GET_SIMULATOR_VERSION() {
    return dronelab::sim::Version{.major = dronelab::sim::version::kMajor,
                                  .minor = dronelab::sim::version::kMinor,
                                  .patch = dronelab::sim::version::kPatch,
                                  .str = dronelab::sim::version::kString.data()};
}