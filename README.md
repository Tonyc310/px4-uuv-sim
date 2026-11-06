# px4-uuv-sim

A physics-based simulation of an unmanned underwater vehicle, the BlueROV2, in Gazebo, flown by the PX4 autopilot and commanded from ROS 2. The vehicle's 6-DOF dynamics follow Fossen's marine craft model, with hydrodynamic coefficients from published BlueROV2 identification papers, and scripted scenarios check the vehicle's behavior pass or fail.

It uses public vehicles and public data only.

## Status

Milestone 1, the toolchain, is done: Gazebo Harmonic and PX4 v1.17.0's software-in-the-loop simulation run in the development container, and PX4's topics reach ROS 2 through the Micro XRCE-DDS Agent, each tested. Milestone 2, the vehicle model for the BlueROV2 Heavy, is under way: `uuv_dynamics` holds the vehicle's equations of motion in Fossen's form and converts between Gazebo's frames and Fossen's. Its parameters come from Wu (2018), who estimated them from published data on the earlier BlueROV because the tank tests he planned weren't run, so the model is a plausible BlueROV2 Heavy, not a measured one. Each value cites its table and page in `src/uuv_dynamics/src/vehicle.cpp`. The plan, in order:

1. **Toolchain:** Gazebo's underwater example and PX4's software-in-the-loop simulation running, with PX4's topics in ROS 2.
2. **Vehicle model:** a Gazebo system plugin for the BlueROV2's 6-DOF dynamics (rigid body, added mass, damping, buoyancy, thrusters), checked against Gazebo's own hydrodynamics and Fossen's Python simulator.
3. **Sensors:** IMU, pressure, magnetometer and DVL, with configurable noise, bias, drift and dropouts.
4. **PX4 and ROS 2:** PX4 holding depth and heading and flying a waypoint mission, commanded over ROS 2.
5. **Scenarios:** currents, starting states and sensor faults, recorded and scored pass or fail by one command.
6. **Hardware in the loop:** the same scenarios with PX4 on a real flight controller.

## Build and test

Everything runs in a Docker container with ROS 2 Jazzy and Gazebo Harmonic on Ubuntu 24.04, the platform PX4 supports, so the host needs only Docker. The container builds PX4 from source at a release tag, with the Micro XRCE-DDS Agent and PX4's ROS 2 messages (`px4_msgs`) at the versions that match it. It installs what each package's `package.xml` declares, with rosdep, and fetches the models Gazebo's examples need, so the tests run offline. The image is about 9 GB, and its first build takes about 20 minutes. One command builds the container and the workspace and runs the tests; CI runs the same:

```bash
tools/dev.sh test
```

```
Starting >>> uuv_bringup
Starting >>> uuv_dynamics
Finished <<< uuv_dynamics [0.94s]
Finished <<< uuv_bringup [39.0s]

Summary: 2 packages finished [39.1s]
Summary: 56 tests, 0 errors, 0 failures, 0 skipped
```

The tests run Gazebo headless. Gazebo's underwater example, MBARI's Tethys LRAUV with buoyancy, hydrodynamics and a thruster, runs 5 simulated seconds under its propeller's thrust and must move more than 1 m. PX4's quadcopter, `gz_x500`, boots in Gazebo and must report itself ready for takeoff within a minute; the test first turns off the airframe's demand for a ground station's link, which a test doesn't have. With PX4 and the agent running, a ROS 2 node must receive PX4's vehicle status within 30 s. `uuv_dynamics`'s unit tests check the conversions between the frames Gazebo and ROS use (east-north-up, forward-left-up) and those Fossen's equations and PX4 use (north-east-down, forward-right-down), against known vectors and attitudes, and that each conversion undoes the other. They check that the equations of motion keep the guarantees the physics gives: the mass matrix is symmetric and positive definite, Coriolis forces do no work, drag always takes energy out, and a level, neutrally buoyant vehicle feels no restoring force. They also check behavior with known answers: net buoyancy accelerates the vehicle in heave by exactly what its mass and added mass allow, a tilted vehicle is turned back to level, surge settles at the speed where thrust equals drag, surge thrust pitches the nose down because it pushes above the centre of gravity, and a turning vehicle skids outward as the Coriolis terms predict. ROS 2's linters check the C++, Python, CMake and package files.

`tools/dev.sh build` only builds, and `tools/dev.sh` alone opens a shell in the container, with the repository as the colcon workspace at `/ws`. `tools/dev.sh gui` does the same with the desktop's display and GPU, for Gazebo's window:

```bash
tools/dev.sh gui gz sim auv_controls.sdf
tools/dev.sh gui src/uuv_bringup/scripts/px4_sitl gz_x500
```

The second runs PX4 with its quadcopter, with PX4's shell (`pxh>`) in the terminal. With no ground station connected, `param set NAV_DLL_ACT 0` lets it arm; `commander takeoff` then flies it up to 2.5 m.

## Layout

```
docker/   the development container and its entrypoint
src/      the ROS 2 packages: uuv_bringup, which starts PX4's simulation (and later the rest),
          and uuv_dynamics, the vehicle's equations of motion, with no Gazebo in it
tools/    dev.sh, which runs any command in the container
```

## License

MIT: see [LICENSE](LICENSE).
