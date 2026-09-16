# uw-communication

This package implements the underwater acoustic communication layer used by the UWMSN research system. It models intermittent connectivity, packet loss, communication delays, and message exchange between AUVs in the simulation environment.

## Purpose

`uw-communication` acts as the networking middleware between the simulator and the motion-planning layer. It recreates the constraints of underwater acoustic communication so the optimization algorithms can be tested under realistic communication failures and latency.

## Main components

- `src/acoustic_modem.py`: communication model for the single-target configuration.
- `src/acoustic_modem_MTT.py`: communication model for the multi-target configuration.
- `launch/acoustic_modem_init.launch`: standard launch configuration.
- `launch/acoustic_modem_init_MTT.launch`: multi-target launch configuration.
- `include/uw-communication/acoustic_modem_h.py`: helper definitions and simulation parameters.

## Role in the project workflow

1. The simulator publishes AUV positions and target states.
2. The acoustic modem simulates packet delivery and communication timing.
3. The motion-optimization package consumes the received measurements and controller intent updates.

## Citation

If you use this package in your work, please cite the publication associated with the project and include the final journal/conference details once they are available.

> Tiranti, A., et al. "Motion optimization strategy for passive acoustic monitoring with a team of AUVs considering intermittent communication." Please cite the published paper appropriately in any derived work.

A placeholder for a second paper may be added here once the final bibliographic metadata is known.

## Notes

- This package is designed for ROS 1 and expects to run as part of a catkin workspace.
- It is intended to be used together with `uwmsn-sim` and `uwmsn-motion_opt`.
- The code is suitable for research use and is currently being prepared for public release.
