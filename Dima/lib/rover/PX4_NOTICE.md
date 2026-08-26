# PX4 Rover control attribution

This directory contains independent, project-native adaptations of Rover
control algorithms and module boundaries from PX4-Autopilot.

- Upstream: PX4/PX4-Autopilot
- Release: v1.17.0
- Commit: d6f12ad1c4f70ad3230afd7d86e971421e02fef4
- Referenced areas: `src/lib/rover_control`, `src/lib/pid`,
  `src/lib/slew_rate`, and `src/modules/rover_differential`
- License: BSD 3-Clause

Copyright (c) PX4 Development Team contributors.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the conditions of the BSD 3-Clause
license are met. The upstream copyright notice, conditions, and disclaimer
must be retained with redistributed source or binary documentation.

No uORB, Parameter Core, PX4 events, PX4 work queue, PX4 platform HAL, or dynamic
allocation code is copied into this project-native implementation.
