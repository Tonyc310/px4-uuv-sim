# px4-uuv-sim

A physics-based simulation of an unmanned underwater vehicle, the BlueROV2, in Gazebo, flown by the PX4 autopilot and commanded from ROS 2. The vehicle's 6-DOF dynamics follow Fossen's marine craft model, with hydrodynamic coefficients from published BlueROV2 identification papers, and scripted scenarios check the vehicle's behavior pass or fail.

It uses public vehicles and public data only.

## Status

Milestone 1, the toolchain, is under way: Gazebo Harmonic and PX4 v1.17.0's software-in-the-loop simulation run in the development container, each tested. The plan, in order:

1. **Toolchain:** Gazebo's underwater example and PX4's software-in-the-loop simulation running, with PX4's topics in ROS 2.
2. **Vehicle model:** a Gazebo system plugin for the BlueROV2's 6-DOF dynamics (rigid body, added mass, damping, buoyancy, thrusters), checked against Gazebo's own hydrodynamics and Fossen's Python simulator.
3. **Sensors:** IMU, pressure, magnetometer and DVL, with configurable noise, bias, drift and dropouts.
4. **PX4 and ROS 2:** PX4 holding depth and heading and flying a waypoint mission, commanded over ROS 2.
5. **Scenarios:** currents, starting states and sensor faults, recorded and scored pass or fail by one command.
6. **Hardware in the loop:** the same scenarios with PX4 on a real flight controller.

## Build and test

Everything runs in a Docker container with ROS 2 Jazzy and Gazebo Harmonic on Ubuntu 24.04, the platform PX4 supports, so the host needs only Docker. The container builds PX4 from source at a release tag, installs what each package's `package.xml` declares, with rosdep, and fetches the models Gazebo's examples need, so the tests run offline. The image is about 9 GB, and its first build takes about 15 minutes. One command builds the container and the workspace and runs the tests; CI runs the same:

```bash
tools/dev.sh test
```

```
Starting >>> uuv_bringup
Finished <<< uuv_bringup [21.2s]

Summary: 1 package finished [21.2s]
Summary: 13 tests, 0 errors, 0 failures, 0 skipped
```

The tests run Gazebo headless. Gazebo's underwater example, MBARI's Tethys LRAUV with buoyancy, hydrodynamics and a thruster, runs 5 simulated seconds under its propeller's thrust and must move more than 1 m. PX4's quadcopter, `gz_x500`, boots in Gazebo and must report itself ready for takeoff within a minute; the test first turns off the airframe's demand for a ground station's link, which a test doesn't have. ROS 2's linters check the Python, CMake and package files.

`tools/dev.sh build` only builds, and `tools/dev.sh` alone opens a shell in the container, with the repository as the colcon workspace at `/ws`. `tools/dev.sh gui` does the same with the desktop's display and GPU, for Gazebo's window:

```bash
tools/dev.sh gui gz sim auv_controls.sdf
tools/dev.sh gui src/uuv_bringup/scripts/px4_sitl gz_x500
```

The second runs PX4 with its quadcopter, with PX4's shell (`pxh>`) in the terminal. With no ground station connected, `param set NAV_DLL_ACT 0` lets it arm; `commander takeoff` then flies it up to 2.5 m.

## Layout

```
docker/   the development container
src/      the ROS 2 packages: uuv_bringup, which starts PX4's simulation (and later the rest)
tools/    dev.sh, which runs any command in the container
```

## License

MIT: see [LICENSE](LICENSE).
