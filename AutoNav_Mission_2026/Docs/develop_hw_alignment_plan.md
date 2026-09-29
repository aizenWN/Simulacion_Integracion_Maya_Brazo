# Develop Hardware-Faithful Simulation Plan

Date: 2026-04-26
Status: draft for review — do not commit until reviewed.

## Goal

Keep `develop` and `humble-jetson` as separate branches with separate jobs:

- `humble-jetson` remains the hardware / field branch.
- `develop` remains the simulation / beta-testing branch.

The goal is **not** to turn `develop` into a dual-mode Jetson branch, and not to merge `humble-jetson` wholesale into it.

The goal is to make `develop` a better simulation lab for the real Maya rover:

> `humble-jetson` tells us what Maya really is. `develop` should become the wind tunnel where we stress-test that reality safely.

Before, simulation helped create the software carcass that hardware could inhabit by turning Gazebo off. Now hardware has produced a validated motion baseline, so simulation should be realigned around that validated baseline while preserving the useful SIM work that already exists.

## Core System foundation

AutoNav is not the whole rover stack. It sits on top of `~/Documents/Core_System_2026`, especially the firmware subtree:

- `kukulcan/firmware/upio-ros2/`

That subtree is the main MCU firmware for the Kukulcan controller PCB. It provides the micro-ROS boundary between the physical rover and AutoNav.

Current firmware architecture:

```text
RoboClaw + BNO055 + BME280 + LEDs + motor outputs
  -> HAL modules (`lib/hal_*`)
  -> app FreeRTOS tasks (`lib/app`)
  -> micro-ROS RTE (`lib/rte`)
  -> ROS 2 topics consumed by AutoNav
```

Important Core System facts:

- Firmware target: ESP32-S3R8 with PlatformIO + Arduino + FreeRTOS + micro-ROS.
- micro-ROS serial agent command documented by firmware:
  - `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600`
- Firmware consumes:
  - `cmd_vel` (`geometry_msgs/Twist`) for motor commands.
- Firmware publishes:
  - `sensors/bno055/imu/data`
  - `sensors/roboclaw/encoders/left_m1/ticks`
  - `sensors/roboclaw/encoders/right_m2/ticks`
  - `sensors/roboclaw/encoders/left_m1/qpps`
  - `sensors/roboclaw/encoders/right_m2/qpps`
- IMU and encoder publish cadence is intended around 50 Hz (`20 ms`).
- Motor task has a 250 ms `cmd_vel` timeout stop.
- Motor control is currently open-loop for `cmd_vel`; closed-loop velocity behavior remains a known system-level limitation.

Pinned integration note, not a current blocker:

- AutoNav docs/commands have sometimes referred to the right encoder topic as `right_m1`; firmware currently publishes `right_m2`.
- Because the validated square test worked, this is not critical for the immediate SIM-alignment step.
- Keep it pinned for later Core/AutoNav contract cleanup, but do not let it block reproducing the physical square-test milestone in simulation.

## Validated hardware truth to respect

From `humble-jetson` / 2026-04-26 validation:

- Wheel encoder odom + BNO055 IMU + EKF achieved controlled motion.
- The rover drove an almost perfect square pattern while correcting yaw orientation online.
- Validated encoder ticks per revolution: `7400`.
- EKF owns `odom -> base_footprint`.
- Nav2 should consume `/odometry/filtered`, not raw wheel odom or raw IMU directly.
- Wheel odom + BNO055 is the baseline local motion source.
- VSLAM can run over the limited Wi-Fi link in the tested lightweight ZED setup, but it remains optional.
- VSLAM must not replace wheel odom or publish competing TF into the Nav2 chain.

## Current `develop` SIM pipeline as-is

Current `develop` launch is SIM-first and simple:

```text
Gazebo random_world.sdf
  -> ros_gz_bridge
  -> /odom, /imu, /scan
  -> scan_frame_relay.py
  -> sim_covariance_relay.py
  -> robot_localization EKF
  -> /odometry/filtered + odom->base_footprint
  -> slam_toolbox + Nav2
```

Important current SIM details:

- `maya.launch.xml` is a Gazebo-first launch file, not a hardware launch file.
- Default world: `random_world.sdf`.
- Default render engine: `ogre2`.
- Default spawn: `x=4.5`, `y=2.0`, `z=0.25`, `yaw=0.0`.
- Depth scan pipeline is optional and disabled by default.
- RTAB-Map odom is optional and disabled by default.
- `sim_covariance_relay.py` rewrites Gazebo `/odom` and `/imu` covariance into `/odom_with_covariance` and `/imu_with_covariance`.
- EKF currently consumes relayed covariance topics via launch overrides:
  - `odom0=/odom_with_covariance`
  - `imu0=/imu_with_covariance`
- Nav2 consumes `/odometry/filtered`.
- SLAM currently uses `/scan`, not `/scan_fixed` or `/scan_merged`.

## Useful `develop` work to preserve

These are not junk. Do not overwrite them blindly from `humble-jetson`.

### Gazebo / physics

- Corrected DiffDrive side mapping in `mobile_base_gazebo.xacro`:
  - left joints are left wheels
  - right joints are right wheels
- Cylinder wheel collision geometry was introduced because mesh collisions made skid-steer contact unstable.
- Explicit wheel contact parameters were added for heightmap turning:
  - `mu1/mu2 = 1.2`
  - `slip1/slip2 = 0.02`
- `ogre2` render engine is part of the current working SIM baseline.

### Sensors

- SIM IMU publishes with `gz_frame_id=imu_link`; this fixed a real frame mismatch.
- SIM IMU was deliberately tuned toward a BNO055-like regime:
  - `update_rate=50`
  - gyro noise around `0.003 rad/s`
  - accel noise around `0.015 m/s^2`
- SIM lidar is currently:
  - `update_rate=10`
  - `samples=450`
  - range `0.03` to `12.0`
  - `gz_frame_id=lidar_link`
- RGB camera exists in SIM and should not be removed casually if later visual tests depend on it.

### EKF / Nav2 / SLAM

- Current SIM EKF fuses:
  - odom x/y + linear x + yaw rate
  - IMU yaw orientation + yaw rate
- `imu0_relative: true` is part of the current SIM baseline and should not be changed without a test.
- Current SLAM/Nav2 config contains SIM-specific tuning from previous drift and Nav2 debugging.
- Nav2 Spin diagnostics showed remaining yaw-behavior mismatch; do not assume yaw is solved in SIM just because hardware square test improved.

## Mismatch list: SIM assumptions vs hardware truth

### 1. Odom source

Current SIM:

- Gazebo DiffDrive publishes `/model/maya/odometry`, bridged to `/odom`.
- This is not generated from encoder ticks.

Hardware truth:

- Core System firmware publishes raw RoboClaw encoder tick topics over micro-ROS.
- AutoNav `encoder_odom.py` converts those ticks into `/odom`.
- Encoder odom + BNO055 + EKF is the validated baseline.
- Validated ticks per rev: `7400`.
- Firmware topic contract currently appears to use `right_m2`, while some AutoNav notes/commands mention `right_m1`; keep this pinned for later cleanup, but do not treat it as a blocker for the immediate SIM square-test goal.

Interpretation:

- This is the biggest conceptual mismatch, but it does **not** mean the next step is to port `encoder_odom.py` into SIM.
- First, treat Gazebo `/odom` as the SIM stand-in for wheel odom and make sure its geometry/noise/turn behavior resembles the hardware baseline closely enough.
- Simulated encoder ticks may be valuable later, but it is not the minimal next step.

### 2. Wheel geometry

Current SIM:

- `drive_wheel_radius = 0.1636`, matching hardware assumption.
- `drive_wheel_separation = 1.09449` in `develop`.

Hardware validation command:

- `encoder_wheel_separation:=1.0`
- `encoder_wheel_radius:=0.1636`

Interpretation:

- Radius already matches.
- Wheel separation differs and should be investigated carefully.
- Do **not** simply change SIM wheel separation to `1.0` without checking why `1.09449` exists and how it affects turn drift.
- Minimal next work: document/measure the effect of wheel separation in SIM turns against the hardware square-test behavior.

### 3. IMU behavior

Current SIM:

- IMU is BNO055-ish and frame-corrected.
- EKF currently trusts IMU yaw orientation and yaw rate.

Hardware truth:

- Hardware path currently treats BNO055 carefully.
- The validated HW EKF baseline from `humble-jetson` uses IMU yaw rate only, not direct orientation, because direct orientation was not yet trustworthy enough.

Interpretation:

- SIM may currently be too idealistic if it uses direct IMU orientation as a clean yaw source.
- But changing this immediately could regress known SIM behavior.
- Minimal next work: run/compare SIM square or spin behavior with current EKF and a hardware-like yaw-rate-only EKF variant, without making yaw-rate-only the default until it wins.

### 4. Lidar behavior

Current SIM:

- Gazebo lidar is 10 Hz, 450 samples, 12 m max range.

Hardware likely baseline:

- LD19 is the real 2D scan source.

Interpretation:

- SIM lidar should eventually approximate LD19 scan rate/range/noise/FOV enough for Nav2 and SLAM tests to transfer.
- This is important, but less urgent than odom/IMU because the current autonomy milestone was motion/heading, not lidar navigation.
- Minimal next work: record actual LD19 topic rate/range/frame contract from hardware logs or next run, then compare to SIM.

### 5. Nav2 / SLAM layer

Current SIM:

- Nav2 already consumes `/odometry/filtered`, which matches the desired contract.
- SLAM uses `/scan`.
- Depth scan and visual odom are optional and disabled by default.

Hardware truth:

- Next target is Nav2 re-enable over the validated encoder + BNO055 + EKF baseline.
- VSLAM is optional, not baseline.

Interpretation:

- `develop` is already conceptually right here: Nav2 sits above EKF odom.
- The first SIM realignment should not start by retuning Nav2.
- It should start by making the motion estimate feeding Nav2 more hardware-faithful.

### 6. VSLAM / ZED / perception

Current SIM:

- RTAB-Map odom scaffold exists but is off by default.
- RGB/depth camera exists.

Hardware truth:

- ZED VSLAM can run over Wi-Fi in the tested profile.
- It remains optional and should not be baseline.

Interpretation:

- Do not integrate VSLAM into the default `develop` baseline now.
- Keep visual odom/perception as later optional stress-test layers.
- First prove the minimal stack.

## Revised minimal plan

No config splitting yet. No branch merge. No launch refactor yet.

### Step 1 — Keep this document as a review draft

This file should be reviewed before commit.

Purpose:

- Align on the real goal.
- Avoid turning `develop` into a fake Jetson branch.
- Avoid accidental regression of working SIM physics/Nav2 behavior.

### Step 2 — Create a focused SIM-vs-HW mismatch checklist

Use the mismatch list above as the checklist. Include Core System as the upstream source of hardware truth, not just `humble-jetson`.

Classify each item:

- preserve as-is
- measure first
- test variant
- change only after evidence

Initial classification:

| Area | Current status | Proposed action |
| --- | --- | --- |
| Nav2 consumes `/odometry/filtered` | already aligned | preserve |
| EKF owns `odom -> base_footprint` | already aligned | preserve |
| VSLAM optional | aligned in principle | preserve optional/off baseline |
| Wheel radius `0.1636` | aligned | preserve |
| Wheel separation `1.09449` vs HW `1.0` | mismatch | measure/test before changing |
| SIM IMU direct yaw fusion vs HW yaw-rate-only caution | mismatch | A/B test variant before changing default |
| Gazebo odom instead of encoder-derived odom | conceptual mismatch | keep as stand-in first; simulate encoder ticks later only if needed |
| AutoNav right encoder topic vs firmware `right_m2` topic | possible cross-repo mismatch | pinned for later cleanup; not blocking SIM square-test reproduction |
| SIM lidar vs LD19 | likely mismatch/unknown | measure real LD19 contract before changing |
| Depth scan / visual odom | optional | keep off baseline |

### Step 3 — First code experiment should be tiny and reversible

Preferred first experiment:

- Add a documented, optional SIM test mode or launch argument for hardware-like EKF behavior.
- Do not change the default SIM baseline.
- Compare current SIM EKF vs hardware-like EKF behavior under the same motion test.

Candidate tests:

1. manual spin / Nav2 Spin yaw test
2. square-test-like command sequence in SIM
3. short Nav2 forward/return run

Success criteria:

- `/odometry/filtered` remains smooth.
- `odom -> base_footprint` has no jumps.
- final yaw after turns is closer to commanded motion.
- Nav2 behavior does not regress.

### Step 4 — Only after EKF evidence, consider wheel geometry

If yaw-rate-only EKF does not explain the SIM/HW gap, test wheel separation:

- current: `1.09449`
- hardware command: `1.0`

Do this as a controlled A/B, not a blind edit.

### Step 5 — Only later consider simulated encoder ticks

Porting `encoder_odom.py` into SIM is not the first move.

It becomes useful if:

- Gazebo `/odom` is too idealized or structurally unlike hardware encoder odom.
- We need to test tick quantization, sign, rate, or dropout behavior.
- We want regression tests that exercise the same odom code used on hardware.

Until then, Gazebo odom is acceptable as the wheel-odom stand-in.

## Things explicitly not planned right now

- Do not split `ekf.yaml` yet.
- Do not split Nav2 params yet.
- Do not port all hardware scripts into `develop` yet.
- Do not merge `humble-jetson` into `develop`.
- Do not make `develop` launch hardware by default or pretend to be Jetson.
- Do not default VSLAM into the SIM baseline.
- Do not change SIM wheel separation from `1.09449` to `1.0` without a measurement.
- Do not change `imu0_relative: true` without an A/B test.

## Minimal next action after review

After this draft is reviewed, the smallest useful technical target is:

> Reproduce the physical `humble-jetson` square-test milestone in simulation before integrating more autonomy layers.

That means creating a controlled SIM test equivalent to the validated hardware run:

```text
SIM wheel-like odom + SIM BNO055-like IMU
  -> EKF
  -> square-test command pattern
  -> verify near-square path and online yaw correction
```

Initial comparison should avoid changing the default SIM baseline. Prefer an A/B test:

1. current `develop` EKF behavior
2. hardware-like yaw-rate-only IMU fusion behavior

Success criteria:

- square path is close to the physical validated result
- yaw correction works online during straight segments
- `/odometry/filtered` remains smooth
- `odom -> base_footprint` has no jumps
- no Nav2, SLAM, LD19, VSLAM, depth, ArUco, or YOLO required yet

Once this square-test milestone is reproduced in SIM, integrate layers in this order:

1. LD19-like lidar / validated scan contract
2. SLAM
3. Nav2
4. optional VSLAM

This keeps simulation aligned with the proven physical progression instead of jumping straight into full-stack autonomy.
