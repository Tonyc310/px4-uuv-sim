# px4-uuv-sim

A physics-based simulation of the BlueROV2 Heavy, an unmanned underwater vehicle with eight thrusters, in Gazebo. The vehicle's 6-DOF dynamics follow Fossen's marine craft model and run as a Gazebo plugin, which takes thruster commands on the topic and in the form that the PX4 autopilot's Gazebo bridge publishes. The model is checked against an independent implementation of the same equations, Fossen's Python Vehicle Simulator, and in surge against Gazebo's own Hydrodynamics system.

That much works today, along with PX4's software-in-the-loop simulation and its ROS 2 bridge in the same container, so far with PX4's own quadcopter. Still to come, in the order of the milestones below: sensor models, PX4 holding the vehicle's depth and heading and flying a mission commanded from ROS 2, scripted scenarios that check the vehicle's behavior pass or fail, and last the same scenarios with PX4 on a real flight controller.

It uses public vehicles and public data only.

## Status

The first two of six milestones are done:

1. **Toolchain (done):** Gazebo Harmonic and PX4 v1.17.0's software-in-the-loop simulation run in the development container, and PX4's topics reach ROS 2 through the Micro XRCE-DDS Agent, each tested.
2. **Vehicle model (done):** a Gazebo system plugin for the BlueROV2 Heavy's 6-DOF dynamics (rigid body, added mass, damping, buoyancy, thrusters), checked against Fossen's Python simulator and, in surge, against Gazebo's own Hydrodynamics system.
3. **Sensors:** IMU, pressure, magnetometer and DVL, with configurable noise, bias, drift and dropouts.
4. **PX4 and ROS 2:** PX4 holding depth and heading and flying a waypoint mission, commanded over ROS 2.
5. **Scenarios:** currents, starting states and sensor faults, recorded and scored pass or fail by one command.
6. **Hardware in the loop:** the same scenarios with PX4 on a real flight controller.

## The vehicle model

Three packages hold it:

- `uuv_dynamics` holds the vehicle's equations of motion in Fossen's form and the thrust of its eight T200 thrusters, integrates them over time, works out the wrench a Gazebo plugin must add to Gazebo's rigid-body physics, and converts between Gazebo's frames and Fossen's. It doesn't depend on Gazebo.
- `uuv_gazebo`'s plugin applies that wrench, thrust included, to a BlueROV2 Heavy model in Gazebo, and takes thruster commands the way PX4 sends them.
- `uuv_validation` checks the model against an independent implementation of the same equations, Fossen's Python Vehicle Simulator: free decay in each axis, the response to each thruster alone and to forward thrust, and the surge, heave and yaw step responses. It also checks surge against Gazebo's own Hydrodynamics system.

The model is a plausible BlueROV2 Heavy, not a measured one. Its parameters come from Wu (2018), who estimated them from published data on the earlier BlueROV because the tank tests he planned weren't run. Each value cites its table and page in `src/uuv_dynamics/src/vehicle.cpp`.

One consequence of those values shows when there is no autopilot: forward thrust pitches the nose up, because the horizontal thrusters push 85 mm below the body origin and the centre of gravity sits only 20 mm below it. Up to about 3 N the vehicle settles at a steady speed. At 10 N its speed keeps oscillating by 15%. At 45 N, about a third of full thrust, the nose-up moment (3.8 N m) exceeds the most the weight can right (2.3 N m), and the vehicle loops. Holding pitch is the autopilot's job, in milestone 4.

## Build and test

Everything runs in a Docker container with ROS 2 Jazzy and Gazebo Harmonic on Ubuntu 24.04, the platform PX4 supports, so the host needs only Docker. The container builds PX4 from source at a release tag, with the Micro XRCE-DDS Agent and PX4's ROS 2 messages (`px4_msgs`) at the versions that match it. It installs what each package's `package.xml` declares, with rosdep, and fetches the models Gazebo's examples need, so the tests run offline. The image is about 9 GB, and its first build takes about 20 minutes. One command builds the container and the workspace and runs the tests; CI runs the same:

```bash
tools/dev.sh test
```

```
Starting >>> uuv_dynamics
Starting >>> uuv_bringup
Finished <<< uuv_dynamics [1.23s]
Starting >>> uuv_gazebo
Starting >>> uuv_validation
Finished <<< uuv_gazebo [22.3s]
Finished <<< uuv_bringup [39.2s]
Finished <<< uuv_validation [1min 52s]

Summary: 4 packages finished [1min 53s]
Summary: 138 tests, 0 errors, 0 failures, 0 skipped
```

The tests run Gazebo headless. ROS 2's linters check the C++, Python, CMake and package files; the rest:

| What is tested | What must hold |
|---|---|
| **The toolchain** (`uuv_bringup`) | |
| Gazebo's underwater example | MBARI's Tethys LRAUV, with buoyancy, hydrodynamics and a thruster, moves more than 1 m in 5 simulated seconds under its propeller's thrust. |
| PX4 in Gazebo | PX4's quadcopter, `gz_x500`, boots within a minute and reports itself ready for takeoff within another. The test first turns off the airframe's demand for a ground station's link, which a test doesn't have. |
| PX4 to ROS 2 | With PX4 and the agent running, a ROS 2 node receives PX4's vehicle status within 30 s. |
| **The equations** (`uuv_dynamics`) | |
| Frame conversions | Between the frames Gazebo and ROS use (east-north-up, forward-left-up) and those Fossen's equations and PX4 use (north-east-down, forward-right-down), known vectors and attitudes convert correctly, and each conversion undoes the other. |
| What the physics guarantees | The mass matrix is symmetric and positive definite, Coriolis forces do no work, drag always takes energy out, and a level, neutrally buoyant vehicle feels no restoring force. |
| Behavior with known answers | Net buoyancy accelerates the vehicle in heave by exactly what its mass and added mass allow. A tilted vehicle is turned back to level. Under a force through the body origin, surge settles at the speed where the force equals drag and the nose pitches down, because the origin is above the centre of gravity. A turning vehicle skids outward as the Coriolis terms predict. |
| The wrench for Gazebo | Added to what a rigid-body simulator computes itself, it reproduces Fossen's equations exactly. |
| The integrator | It matches the exponential decay of a linearly damped surge, and moves and turns the vehicle in the world as its body velocity says. |
| The thrusters | A T200's thrust follows Blue Robotics' measurements at 16 V, including the dead band around neutral. Each thruster gets the thrust of its own pulse width, and the eight thrusts combine into the force and moment their positions and directions give. No pulse at all, which PX4 sends while disarmed, stops a thruster, and a command that isn't one finite pulse width per thruster is refused. |
| **The plugin in Gazebo** (`uuv_gazebo`) | |
| Gazebo against the library | The BlueROV2 Heavy follows the library's own integration for 2 s, within 1 cm, 0.5° and 1% of its speed, from six starts: rising from rest, righting from 20° nose up, surge slowing from 1 m/s while heading 60° off east, yaw slowing from 0.5 rad/s while rolled 20°, surging under thruster commands published on PX4's motor topic, and the same with a malformed command after it, which must change nothing. |
| **The trajectory printer** (`uuv_validation`) | |
| The `simulate` program | It prints the model's trajectory for the comparisons with Fossen's simulator. It refuses arguments it can't use, and reports a trajectory it couldn't write. |

`uuv_validation` also compares the model with two independent implementations. Each comparison states a tolerance, and next to it what was measured:

| Compared with | Motion | Must agree within | Measured |
|---|---|---|---|
| Fossen's simulator | Free decay from 0.5 m/s or 0.5 rad/s in each of the six axes, for 10 s | 0.25% of the top speed, over the whole velocity trajectory | under 0.1% |
| Fossen's simulator | From rest, each thruster alone at 1700 µs and the four horizontals pushing forward, for 5 s | the same 0.25% | 0.13% |
| Fossen's simulator | Step responses: pushed gently forward (2 N, where surge settles), down, and into a turn | 0.1% of the steady speed, and 2 ms of the rise time from 10% to 90% of it | better than a millionth, and under 1 ms |
| Gazebo's Hydrodynamics system | Surge step response to a 10 N push through the centre of gravity, with the same coefficients | 0.1% of the steady speed, and 2 ms of the rise time | a few billionths, and 0.3 ms |

Fossen's simulator shares nothing with the C++: the reference is in Python, with Fossen's own Coriolis and restoring functions, Euler angles where the C++ uses a quaternion, forward Euler where the C++ uses Runge-Kutta, and Wu's values typed again. Its simpler integrator is the lag in the rise times. The 0.25% is tight enough to catch a coefficient a few percent out: with sway's linear damping 3% out, sway's decay moves 0.5% and the test fails.

The comparison with Gazebo's system is in surge only. That system takes added mass from the previous step's acceleration, which diverges when the water carried along outweighs the vehicle, as it does for the Heavy in heave and sway. Pushed in heave the same way, at the same 1 ms step, it blows up within 50 ms. That is why this project's plugin solves the full equations for the acceleration each step. Gazebo's physics engine can also carry added mass itself (`fluid_added_mass` in a link's inertial), which stays stable in the same push.

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
          uuv_gazebo, the Gazebo plugin, the BlueROV2 Heavy model and an open-water world,
          and uuv_validation, the comparisons with independent implementations
tools/    dev.sh, which runs any command in the container
```

## Sources

- Mass, inertia, added mass and damping: C.-J. Wu, *6-DoF Modelling and Control of a Remotely Operated Vehicle*, Master's thesis, Flinders University, 2018 (Tables 5.1 to 5.3), estimates from published data on the earlier BlueROV, not measurements of the Heavy. Thruster positions: the same thesis, Table 4.2, from Blue Robotics' drawings.
- Thrust: Blue Robotics, [T200 Public Performance Data 10-20V, September 2019](https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx), the 16 V sheet, in `src/uuv_dynamics/src/thrusters.cpp`.
- Equations: T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed., Wiley, 2021.
- The reference the model is validated against: T. I. Fossen's [Python Vehicle Simulator](https://github.com/cybergalactic/PythonVehicleSimulator) (MIT), fetched into the container at a fixed commit.
- The Heavy's mesh, thruster order and thrust directions: PX4's BlueROV2 Heavy model and airframe (BSD 3-Clause), used from the PX4 build in the container.

## License

MIT: see [LICENSE](LICENSE).
