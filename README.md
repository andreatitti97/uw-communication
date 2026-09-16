# uw-communication

This package implements the underwater acoustic communication layer used by the UWMSN research system. It models intermittent connectivity, packet loss, communication delays, and message exchange between AUVs in the simulation environment.

## Purpose

`uw-communication` acts as the networking middleware between the simulator and the motion-planning layer. It recreates the constraints of underwater acoustic communication so the optimization algorithms can be evaluated under realistic communication failures and latency.

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

## Requirements

This package is intended for a ROS 1 catkin workspace and requires:

- ROS 1 (Noetic is the expected target)
- `rospy`
- `std_msgs`
- `uwmsn_msgs` or an equivalent custom message package
- `numpy`

## Installation

Place the package in the `src` folder of a catkin workspace, then build the workspace:

```bash
cd ~/ros1_ws
catkin_make
source devel/setup.bash
```

## Usage

Start the modem for a given AUV count:

```bash
roslaunch uw-communication acoustic_modem_init.launch auvID:=1 auvNum:=4
```

For the multi-target configuration:

```bash
roslaunch uw-communication acoustic_modem_init_MTT.launch auvID:=1 auvNum:=4
```

## Reproducibility

Use the same launch configuration for all runs in a given experiment set. Keep the AUV count, target count, and packet-loss configuration fixed when comparing communication quality metrics or optimization outcomes.

## Citation

If you use this package in research or teaching, please cite the project publication describing the motion optimization strategy for passive acoustic monitoring with intermittent communication.

> Tiranti, A., et al. "Motion optimization strategy for passive acoustic monitoring with a team of AUVs considering intermittent communication." Please cite the published paper appropriately in any derived work.

## Notes

- This package is designed for ROS 1 and expects to run with the rest of the project.
- It is intended to be used together with `uwmsn-sim` and `uwmsn-motion_opt`.
- The code is research-focused and is suitable for reproducible experiments in an academic environment.
