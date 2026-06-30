# DroneLab Sim

**Platform:** Windows 10/11 (64-bit), Linux (tested on x86 Ubuntu)  
**Requirements** C++23 capable compiler

**Developer:** Rodrigo Rosa  

## Overview

**DroneLab_SIM** consists in the simulation module for **DroneLab**, a 3D drone visualization and simulation environment.
The simulation module is responsible for providing a simulation environment when requested by its interface. Among its features, it is implicitly responsible for providing modules such as:
- Sensors (e.g IMU, GNSS)
- Actuators (depending on vehicle type, currently only multicopter types)
- Mission planning (Navigation behaviour, setpoints)
- Flight controller (estimation, control)
- Monitoring, rules, mission end triggers
- Mission end report

## Dependencies

Ensure that you have spdlog and eigen >5.0 installed system wide. (They may be fetched as part of the project soon)

- Eigen reference: https://libeigen.gitlab.io/releases/5.0/  
- Spdlog reference: https://github.com/gabime/spdlog

## Quick Start

1. Clone the repository:

```bash
git clone https://github.com/Ridrik/DroneLab_SIM.git
cd DroneLab_SIM
```

2. Fetch DroneLab latest pre-release.

On Windows:

```bash

.\scripts\setup.ps1

```

On Linux:

```bash

./scripts/setup.sh

```

3. Configure the project.
```bash

cmake -B build -DCMAKE_BUILD_TYPE=Debug

```

(You may set up your CMakePresets.json instead)

4. Build

```bash
cmake --build build --target all
```

5. Run

```bash
cmake --build build --target run_dronelab
```

## Documentation & Preview
For further documentation, please see:

https://github.com/Ridrik/DroneLab-Demo

Preview/showcase available at:

https://ridrik.github.io/DroneLabShowcase/

## 🧩 Third-Party Components and Licenses

**DroneLab_SIM** makes use of the following open-source libraries. No source modifications have been made unless otherwise noted.

**Note** These dependancies are not currently fetched by this repo, and the user should install them system wide.

| Component | Purpose | License |
| ---------- | -------- | -------- |
| [**Eigen**](https://gitlab.com/libeigen/eigen) | Linear algebra (matrix and vector math) | MPL 2.0 |
| [**spdlog**](https://github.com/gabime/spdlog) | Fast C++ logging library | MIT |


## Licensing

This repository contains two components:

* `src/` — Open-source simulation module licensed under MIT.
* `dist/` — Proprietary host application. Source code is not provided. Redistribution and use are subject to the accompanying proprietary license terms.

The host application dynamically loads the simulation module at runtime.
