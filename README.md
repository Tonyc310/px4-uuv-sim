# px4-uuv-sim

A physics-based simulation of an unmanned underwater vehicle, the BlueROV2, in Gazebo, flown by the PX4 autopilot and commanded from ROS 2. The vehicle's 6-DOF dynamics follow Fossen's marine craft model, with hydrodynamic coefficients from published BlueROV2 identification papers, and scripted scenarios check the vehicle's behavior pass or fail.

It uses public vehicles and public data only.

## Status

Milestone 1, the toolchain, is done: Gazebo Harmonic and PX4 v1.17.0's software-in-the-loop simulation run in the development container, and PX4's topics reach ROS 2 through the Micro XRCE-DDS Agent, each tested. Milestone 2, the vehicle model for the BlueROV2 Heavy, is under way: `uuv_dynamics` holds the vehicle's equations of motion in Fossen's form and the thrust of its eight T200 thrusters, integrates them over time, works out the wrench a Gazebo plugin must add to Gazebo's rigid-body physics, and converts between Gazebo's frames and Fossen's. `uuv_gazebo`'s plugin applies that wrench, thrust included, to a BlueROV2 Heavy model in Gazebo, and takes thruster commands the way PX4 sends them. Its parameters come from Wu (2018), who estimated them from published data on the earlier BlueROV because the tank tests he planned weren't run, so the model is a plausible BlueROV2 Heavy, not a measured one. Each value cites its table and page in `src/uuv_dynamics/src/vehicle.cpp`. The plan, in order:

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
Finished <<< uuv_dynamics [1.32s]
Starting >>> uuv_gazebo
Finished <<< uuv_gazebo [18.3s]
Finished <<< uuv_bringup [39.5s]

Summary: 3 packages finished [39.6s]
Summary: 89 tests, 0 errors, 0 failures, 0 skipped
```

The tests run Gazebo headless. Gazebo's underwater example, MBARI's Tethys LRAUV with buoyancy, hydrodynamics and a thruster, runs 5 simulated seconds under its propeller's thrust and must move more than 1 m. PX4's quadcopter, `gz_x500`, boots in Gazebo and must report itself ready for takeoff within a minute; the test first turns off the airframe's demand for a ground station's link, which a test doesn't have. With PX4 and the agent running, a ROS 2 node must receive PX4's vehicle status within 30 s. `uuv_dynamics`'s unit tests check the conversions between the frames Gazebo and ROS use (east-north-up, forward-left-up) and those Fossen's equations and PX4 use (north-east-down, forward-right-down), against known vectors and attitudes, and that each conversion undoes the other. They check that the equations of motion keep the guarantees the physics gives: the mass matrix is symmetric and positive definite, Coriolis forces do no work, drag always takes energy out, and a level, neutrally buoyant vehicle feels no restoring force. They also check behavior with known answers: net buoyancy accelerates the vehicle in heave by exactly what its mass and added mass allow, a tilted vehicle is turned back to level, surge settles at the speed where thrust equals drag, surge thrust pitches the nose down because it pushes above the centre of gravity, and a turning vehicle skids outward as the Coriolis terms predict. The wrench for Gazebo, added to what a rigid-body simulator computes itself, must reproduce Fossen's equations exactly. The integrator must match the exponential decay of a linearly damped surge, and move and turn the vehicle in the world as its body velocity says. A T200's thrust must follow Blue Robotics' measurements at 16 V, including the dead band around neutral, and the eight thrusters' thrusts must combine into the force and moment their positions and directions give. No pulse at all, which PX4 sends while disarmed, stops a thruster, and a command that isn't one finite pulse width per thruster is refused. In Gazebo, with the plugin, the BlueROV2 Heavy must follow the library's own integration for 2 s from six starts (rising from rest, righting from 20° nose up, surge slowing from 1 m/s while heading 60° off east, yaw slowing from 0.5 rad/s while rolled 20°, surging under thruster commands published on PX4's motor topic, and the same with a malformed command after it, which must change nothing), within 1 cm, 0.5° and 1% of its speed. ROS 2's linters check the C++, Python, CMake and package files.

`tools/dev.sh build` only builds, and `tools/dev.sh` alone opens a shell in the container, with the repository as the colcon workspace at `/ws`. `tools/dev.sh gui` does the same with the desktop's display and GPU, for Gazebo's window:

```bash
tools/dev.sh gui gz sim auv_controls.sdf
tools/dev.sh gui src/uuv_bringup/scripts/px4_sitl gz_x500
```

The second runs PX4 with its quadcopter, with PX4's shell (`pxh>`) in the terminal. With no ground station connected, `param set NAV_DLL_ACT 0` lets it arm; `commander takeoff` then flies it up to 2.5 m.

The BlueROV2 Heavy in open water, with the workspace built and its setup sourced so Gazebo finds the plugin and the model:

```bash
tools/dev.sh gui bash -c '. install/setup.sh && gz sim src/uuv_gazebo/worlds/water.sdf'
```

## Layout

```
docker/   the development container and its entrypoint
src/      the ROS 2 packages: uuv_bringup, which starts PX4's simulation (and later the rest),
          uuv_dynamics, the vehicle's equations of motion, with no Gazebo in it,
          and uuv_gazebo, the Gazebo plugin, the BlueROV2 Heavy model and an open-water world
tools/    dev.sh, which runs any command in the container
```

## Sources

- Mass, inertia, added mass and damping: C.-J. Wu, *6-DoF Modelling and Control of a Remotely Operated Vehicle*, Master's thesis, Flinders University, 2018 (Tables 5.1 to 5.3), estimates from published data on the earlier BlueROV, not measurements of the Heavy. Thruster positions: the same thesis, Table 4.2, from Blue Robotics' drawings.
- Thrust: Blue Robotics, [T200 Public Performance Data 10-20V, September 2019](https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx), the 16 V sheet, in `src/uuv_dynamics/src/thrusters.cpp`.
- Equations: T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed., Wiley, 2021.
- The Heavy's mesh, thruster order and thrust directions: PX4's BlueROV2 Heavy model and airframe (BSD 3-Clause), used from the PX4 build in the container.

## License

MIT: see [LICENSE](LICENSE).
