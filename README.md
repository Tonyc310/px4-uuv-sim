# px4-uuv-sim

A physics-based simulation of an unmanned underwater vehicle, the BlueROV2, in Gazebo, flown by the PX4 autopilot and commanded from ROS 2. The vehicle's 6-DOF dynamics follow Fossen's marine craft model, with hydrodynamic coefficients from published BlueROV2 identification papers, and scripted scenarios check the vehicle's behavior pass or fail.

It uses public vehicles and public data only.

## Status

The skeleton: a ROS 2 workspace that builds and tests in the development container, and does nothing yet. The plan, in order:

1. **Toolchain:** Gazebo's underwater example and PX4's software-in-the-loop simulation running, with PX4's topics in ROS 2.
2. **Vehicle model:** a Gazebo system plugin for the BlueROV2's 6-DOF dynamics (rigid body, added mass, damping, buoyancy, thrusters), checked against Gazebo's own hydrodynamics and Fossen's Python simulator.
3. **Sensors:** IMU, pressure, magnetometer and DVL, with configurable noise, bias, drift and dropouts.
4. **PX4 and ROS 2:** PX4 holding depth and heading and flying a waypoint mission, commanded over ROS 2.
5. **Scenarios:** currents, starting states and sensor faults, recorded and scored pass or fail by one command.
6. **Hardware in the loop:** the same scenarios with PX4 on a real flight controller.

## Build and test

Everything runs in a Docker container with ROS 2 Jazzy on Ubuntu 24.04, the platform PX4 supports, so the host needs only Docker. One command builds the container and the workspace and runs the tests; CI runs the same:

```bash
tools/dev.sh test
```

```
Starting >>> uuv_bringup
Finished <<< uuv_bringup [0.68s]

Summary: 1 package finished [0.75s]
Summary: 0 tests, 0 errors, 0 failures, 0 skipped
```

`tools/dev.sh build` only builds, and `tools/dev.sh` alone opens a shell in the container, with the repository as the colcon workspace at `/ws`.

## Layout

```
docker/   the development container
src/      the ROS 2 packages: uuv_bringup, the launch files
tools/    dev.sh, which runs any command in the container
```

## License

MIT: see [LICENSE](LICENSE).
