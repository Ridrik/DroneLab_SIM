#ifndef DRONELAB_DRONELAB_SIM_EXPORT_H
#define DRONELAB_DRONELAB_SIM_EXPORT_H

#if defined(_WIN32)
#define DRONELAB_SIM_EXPORT __declspec(dllexport)
#else
#define DRONELAB_SIM_EXPORT __attribute__((visibility("default")))
#endif

#endif  // DRONELAB_DRONELAB_SIM_EXPORT_H