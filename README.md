# Robot Kinematics (C++)

![tests](https://github.com/marc22w/robot-kinematics-cpp/actions/workflows/tests.yml/badge.svg)

**Live 3D demo:** https://marc22w.github.io/robot-kinematics-cpp/viz/

A C++17 library for the kinematics and motion planning of 6-axis industrial robots,
demonstrated on the Universal Robots UR5 (official Denavit–Hartenberg parameters).

![PTP vs LIN tool path](docs/ptp_vs_lin.png)

## Results

| | Value |
|---|---|
| Inverse kinematics precision | position error below 1e-7 mm |
| Inverse kinematics speed | converges in ~4 iterations from a nearby start |
| Robustness | 100% of random reachable poses solved (with random restarts) |
| Forward kinematics | 132 ns per pose |
| Analytical IK (UR5) | all solutions (up to 8, avg. 7.1) in 3.3 µs |
| Analytical vs numerical IK from a cold start | 8.8× faster (3.3 µs vs 29.1 µs), all solutions instead of one |
| Numerical IK optimization | 4.8 → 2.9 µs per solve from a nearby start (−40%), same iterations, by reusing link frames between iterations and precomputing DH constants |
| Minimum-time PTP move (120° on axis 1) | 1.74 s, at the 4 rad/s² acceleration limit |
| Collision check on the LIN move | obstacle hit detected at t = 1.45 s (wrist 2 vs. pole) |
| LIN move, 301 points | 0.29 ms with analytical IK, 0.43 ms numerical |
| LIN through the wrist singularity | refused at t = 0.99 s (axis 4 would jump 20.3° in 10 ms) instead of an impossible motion |
| Velocity-limited LIN (0.5 m/s, 700 mm past the base axis) | without limiting, axis 6 would reach 285.6°/s (limit 180°/s); the tool slows to 0.315 m/s near the base, all joints stay ≤ 180°/s, duration 1.733 → 1.845 s |
| Obstacle avoidance (RRT-Connect) | direct PTP hits the pole; planned path clears it, every point ≥ 10 mm from obstacles |
| PTP vs LIN, same end pose | PTP tool path deviates up to 41.5 mm from the straight line |

## Features

- **Forward kinematics** from standard DH parameters (`Eigen::Isometry3d` poses).
- **Geometric Jacobian**, verified against finite differences.
- **Inverse kinematics**: Levenberg–Marquardt (damped least squares) with adaptive
  damping, step limiting near singularities and random restarts; reports unreachable targets.
- **Analytical inverse kinematics for the UR5**: closed-form solution returning all 8
  configurations (shoulder left/right, elbow up/down, wrist flip), closest-solution
  selection within joint limits, benchmarked against the numerical solver.
- **Joint limits and singularity detection**: IK stays within limits; wrist, elbow and
  shoulder singularities identified from the factored Jacobian determinant.
- **Collision checking**: robot links modelled as capsules; self-collision between
  non-adjacent links, planes (table, walls) and capsule obstacles; first collision along a
  trajectory with the two closest bodies reported.
- **Motion planning (RRT-Connect)**: collision-free joint-space paths around obstacles with a
  10 mm safety margin, path shortcutting, and conversion to a minimum-time trajectory.
- **3D web viewer** (no dependencies, own canvas renderer): animated robot, tool path,
  planner waypoints, live tool-speed chart and collision status; data exported from C++ to JSON.
- **PTP motion**: quintic (minimum-jerk) profile, zero velocity and acceleration at both ends.
- **Minimum duration** for a PTP move from per-joint velocity and acceleration limits.
- **LIN motion**: straight-line tool path with slerp-interpolated orientation, solved at every
  step with the numerical or the analytical (UR5) IK; a continuity check stops the move at
  unreachable points and at singularities, reporting the reason.
- **Velocity-limited LIN**: trapezoidal tool-speed profile with look-ahead (forward/backward
  pass, as on CNC controllers); the tool slows down automatically wherever a joint would
  exceed its velocity limit, e.g. near singularities.
- **3D web viewer** (no dependencies, own canvas renderer): animated robot, tool path,
  obstacles and live collision status; data exported from the C++ library to JSON.
- **CSV export** of joint trajectories and tool paths, plotted with a small Python script.
- **54 unit tests** (GoogleTest), run on every push with **GitHub Actions**.

![PTP motion profile](docs/ptp_profil.png)

## Build and run

Requirements: a C++17 compiler, CMake ≥ 3.20, Eigen 3 (`brew install cmake eigen`).
GoogleTest is downloaded automatically.

```bash
git clone https://github.com/marc22w/robot-kinematics-cpp.git
cd robot-kinematics-cpp
cmake -S . -B build && cmake --build build
./build/demo                                   # IK, PTP and LIN examples, writes CSV files
./build/singularitati                          # singularity analysis
./build/benchmark                              # analytical vs numerical IK
./build/coliziuni                              # collision checks
./build/planificare                            # obstacle avoidance with RRT-Connect
./build/export_viz                             # data for the 3D web viewer (docs/viz)
ctest --test-dir build --output-on-failure     # runs the test suite
```

Plots (optional): `pip install matplotlib numpy`, then `python scripts/grafice.py`.

## Usage

```cpp
#include "kinematics/robot.hpp"
#include "kinematics/trajectory.hpp"

const kin::Robot robot = kin::Robot::ur5();

Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
target.translation() << -0.400, -0.200, 0.300;           // m
const kin::IKResult ik = robot.inverse(target, kin::Joints::Zero());

const kin::CartesianResult lin =
    kin::linearCartesian(robot, ik.q, other_target, /*duration s*/ 3.0, /*dt s*/ 0.01);
```

## Project structure

```
include/kinematics/
  robot.hpp        # DH parameters, forward/inverse kinematics, Jacobian
  trajectory.hpp   # PTP and LIN trajectories, CSV export
src/               # implementation
tests/             # GoogleTest suite
examples/main.cpp  # demo program
scripts/grafice.py # plots for this README
```

## Roadmap

- [x] Forward kinematics (DH)
- [x] Geometric Jacobian and inverse kinematics
- [x] PTP and LIN trajectories with joint limits
- [x] Joint position limits and singularity detection (wrist, elbow, shoulder)
- [x] Simple collision checks (robot links as capsules)
- [x] Analytical inverse kinematics for the UR5 (all 8 solutions) and benchmark vs numerical

## Tech stack

C++17 · Eigen · CMake · GoogleTest · GitHub Actions · Python (matplotlib) for plots

## Author

Marc Vintila, Mechanical Engineering student at Technical University of Cluj-Napoca
