---

## 8. Current HW Baseline Command (2026-02-27)

Use this command as the minimal Jetson hardware baseline for reliability testing with `tools/nav2_reliability_trials.sh`:

```bash
ros2 launch maya_bringup maya.launch.xml \
  sim:=false rviz:=false \
  use_zed:=true zed_camera_model:=zed2i zed_enable_ipc:=false \
  odom_topic:=/zed/zed_node/odom \
  imu_topic:=/imu/data \
  use_ld19:=true lidar_port:=/dev/ttyUSB1 \
  use_depth_scan:=false \
  use_aruco:=false \
  use_yolo:=false
```

Intent:
- keep ArUco/YOLO disabled during core localization/navigation reliability runs,
- isolate LD19 + ZED odom/IMU + EKF + SLAM/Nav2 behavior.

## 8. Postmortem – Failed SIM Attempt (2026-02-18)

This section records a failed tuning cycle so future work does not repeat it.

### 8.1 What failed repeatedly

- Validation showed `NavigateToPose` occasionally succeeded while rover did not meaningfully move.
- Global map stayed extremely small (`/map_metadata` often `5x5` or `6x6` at `0.03 m` resolution).
- Global costmap remained tiny and mostly occupied/unknown; planner frequently logged:
  - `Robot is out of bounds of the costmap`
  - `Sensor origin ... out of map bounds`
- Local costmap published, but often did not populate obstacles as expected.
- Startup ordering issue observed: local costmap activation retried while `odom` frame was not yet available.
- Intermittent actuation checks occurred during bringup windows, causing false negatives.

### 8.2 Key observed signals that matter

- `/cmd_vel` bridging to Gazebo existed and sometimes moved the rover; drive chain was not fully dead.
- Scan quality looked deceptively “good” in finite-ratio terms, but raw values were pathological:
  - ranges clustered at lidar minimum range (`~0.08 m`) with near-hit ratio near `1.0`.
- This pattern indicates self-occlusion / self-hit or invalid sensor geometry context, not a Nav2 planner bug.
- `voxel_grid` limitation hit when `z_voxels` was set above supported value (`16` max in this setup).

### 8.3 Changes that created additional risk (do not repeat blindly)

- Mixing large SLAM/Nav2 parameter sweeps before confirming raw sensor realism.
- Switching multiple subsystems at once (SLAM mode + costmaps + validator + sensor placement), making attribution hard.
- Tuning around symptoms (`xy_goal_tolerance`, medium-goal step, costmap thresholds) while map source remained invalid.
- Over-relying on merged scan for SLAM before validating that primary lidar scan itself is sane.

### 8.4 Root-cause hypothesis to prioritize first

- Primary blocker is sensor/environment geometry for lidar in simulation (self-hit / immediate clipping), causing tiny map growth.
- Secondary effects (costmap bounds warnings, goal “success without movement”, tiny global map) cascade from that.

### 8.5 Required workflow for next iteration

1. Validate raw sensor truth first (before Nav2 tuning):
   - `/scan` must show a realistic spread of ranges, not fixed min-range values.
2. Only after sensor truth is confirmed:
   - validate SLAM map growth (`/map_metadata` spans increase with teleop),
   - then validate Nav2 action behavior and costmaps.
3. Change one subsystem at a time:
   - sensor geometry/collision,
   - SLAM params,
   - costmap params,
   - controller params.
4. Keep deterministic checkpoints after each change:
   - TF chain exists (`map->odom`, `odom->base_footprint`),
   - map span > minimum target,
   - global costmap dimensions track map growth,
   - action trials require measurable movement.

### 8.6 Guardrails for future tuning

- Do not treat `Goal SUCCEEDED` as evidence of valid navigation without displacement checks.
- Do not increase `z_voxels` beyond supported implementation limits in this stack.
- Do not optimize planner/controller until SLAM map span is operationally large.
- Prefer lidar-first SLAM debugging path; introduce depth-based layers only after baseline is stable.

### 8.7 Resolved milestone (2026-02-20)

- Root cause of "lidar sees only a circle at min range" was rendering backend mismatch with GPU lidar in simulation.
- Effective fix: run Gazebo with `ogre2` render engine in launch defaults.
  - `src/maya_bringup/launch/maya.launch.xml` now defaults both `render_engine` and `render_engine_gui` to `ogre2`.
- After this fix:
  - `/scan` publishes realistic data (not fixed at range_min).
  - 2D SLAM with lidar-only input works and map quality is operationally valid in SIM.
- Temporary fallback path (`/scan_depth`) remains a useful contingency, but primary mapping path should now use `/scan`.

### 8.8 Nav2 Planner Timeout Note (error_code 207)

- `NavigateToPose` failure with `ABORTED` + `error_code: 207` maps to `ComputePathToPose TIMEOUT` in Nav2.
- This indicates a planner / global-costmap feasibility issue (or transform availability at planning time), not a raw sensor timestamp failure.
- This issue has occurred intermittently during startup and was previously resolved in at least one run, but the exact successful parameter set was not persisted.
- Required practice:
  - when a run resolves `207`, immediately save the exact `nav2_params.yaml` diff and launch args in this file (or a dated note in `Docs/`),
  - do not continue tuning without checkpointing the known-good values.

### 8.9 Health-check script expectations

- `tools/nav2_health_check.sh` is intended for strict SIM diagnostics.
- If `map -> base_footprint` is temporarily unavailable during startup, the script may fall back to `odom -> base_footprint` for goal generation to keep diagnostics running.
- A fallback frame goal should be treated as a startup-transient indicator, not a final autonomy acceptance result.

### 8.10 Confirmed Nav2 split diagnosis (2026-02-21)

- `ComputePathToPose` can succeed (`error_code: 0`, status `SUCCEEDED`) while `NavigateToPose` still fails.
- Observed failure mode:
  - `NavigateToPose` status `ABORTED` with `error_code: 107` (controller/follow-path timeout behavior).
  - Feedback shows pose nearly constant and `distance_remaining` not decreasing, indicating command execution blockage or no effective motion.
- Startup transient to account for:
  - `/map_metadata` may initially report `width: 0`, `height: 0`.
  - After short manual motion, SLAM map expands (non-zero dimensions) and planner checks become meaningful.
- Operational testing rule:
  1. Warm up SLAM map (short manual drive) until map dimensions are non-zero.
  2. Run `/compute_path_to_pose` first to validate planner.
  3. Run `/navigate_to_pose` second to validate controller execution.
  4. Record both action statuses and `error_code` values in diagnostics.

### 8.11 Nav2 BT Documentation Implication (NavigateToPose)

- Official Nav2 `NavigateToPose` BT node documentation confirms it is a Behavior Tree action wrapper over the `bt_navigator` action server.
- The BT node exposes `error_code_id` and `error_msg` outputs and supports custom behavior tree selection (`behavior_tree` input).
- Practical implication for current debugging:
  - custom BT integration may improve observability and recovery logic,
  - but it will not fix the current controller execution failure by itself.
- Current blocker is downstream of planning (controller / velocity pipeline / motion suppression), not the BT wrapper interface.

### 8.12 Current SIM Autonomy Status (2026-02-21)

- Confirmed working:
  - LiDAR `/scan` in SIM is valid after `ogre2` render engine fix.
  - SLAM (`slam_toolbox`) produces a valid map after short warmup motion (`/map_metadata` transitions from `0x0` to non-zero).
  - `ComputePathToPose` succeeds (`error_code: 0`) once map is initialized.
- Confirmed failing:
  - `NavigateToPose` aborts with `error_code: 107` after planner succeeds.
  - Feedback shows `current_pose` nearly constant and `distance_remaining` not decreasing.
- Interpretation:
  - Minimal baseline autonomous stack is close, but not yet complete.
  - Remaining blocker is controller execution path (FollowPath / velocity pipeline), not LiDAR, SLAM map generation, or global planning.

### 8.13 Next Minimal Logical Step (must do before wider integrations)

1. Validate Nav2 command chain during active `NavigateToPose`:
   - `/cmd_vel_nav`
   - `/cmd_vel_smoothed`
   - `/cmd_vel`
2. Check for motion suppression / safety gating:
   - `/collision_monitor_state`
3. Confirm bridge subscription and command delivery to Gazebo during nav:
   - `/cmd_vel` publisher/subscriber endpoints
4. Run one controlled unblock test with minimal config changes:
   - reduce progress checker strictness (`required_movement_radius`, `movement_time_allowance`)
   - optionally disable/bypass collision monitor temporarily for diagnosis only
5. Re-test `NavigateToPose` and record:
   - final status
   - `error_code`
   - whether pose changes and `distance_remaining` decreases

### 8.14 Integration Priority Rule (current phase)

- Defer non-essential integrations (custom BT trees, new sensors, GNSS, 3D mapping additions) until `NavigateToPose` can physically move the rover in SIM under Nav2.
- Acceptable integrations now are only those that directly improve:
  - controller-path observability,
  - diagnostics automation,
  - motion gating isolation,
  - EKF/odom validation.

### 8.15 Condensed SIM Nav2 Checkpoint (2026-02-23)

- Diagnostic conclusions (from command-chain + A/B tests):
  - `ComputePathToPose` and the Nav2 velocity pipeline are healthy (`/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel` confirmed).
  - Frequent `NavigateToPose` `error_code: 107` was primarily an effective-progress issue (not planner failure, not dead `/cmd_vel`).
  - Collision monitor was not a simple hard-stop, but it reduced progress in tighter maps/space-constrained scenarios.
  - Disabling collision monitor improved progress but was unsafe (rover could continue into collisions).
- Current validated configuration (SIM):
  - `slam_toolbox` uses lidar only: `scan_topic: /scan`
  - Nav2 costmaps set to lidar-only observations (`observation_sources: scan` in local/global costmaps; depth pointcloud blocks left defined but unused)
  - relaxed progress checker: `required_movement_radius: 0.05`, `movement_time_allowance: 30.0`
  - increased forward speed caps: `FollowPath.vx_max: 1.0`, `velocity_smoother.max_velocity[0]: 1.0`
  - collision monitor enabled: `collision_monitor.FootprintApproach.enabled: True`
  - random world heightmap scaled to `40m x 40m`
- Reliability milestone:
  - `tools/nav2_reliability_trials.sh` added for headless multi-trial validation (warmup forward drive + return-to-reference test).
  - On current 40x40 random world, automated headless protocol achieved:
    - `5 / 5` trials `SUCCEEDED`
    - all `error_code: 0`
    - mean end-goal error (map frame) ≈ `0.073 m`
    - median end-goal error (map frame) ≈ `0.030 m`
    - mean trial duration (sim clock) ≈ `104.8 s`
- Interpretation and remaining work:
  - End-to-end Nav2 in SIM is now repeatably functional under the current parameter set.
  - Next phase is tuning robustness and localization drift (EKF/IMU/SLAM), not basic command-chain viability.
  - Compare `map` vs Gazebo displacement by magnitude only (different frames); timestamp age metrics from separate CLI samples may show skew.

### 8.16 Frozen SIM Baseline for 25-Trial Regression (2026-02-23)

- Freeze this configuration before any further tuning or GNSS integration work:
  - Gazebo render engine defaults: `ogre2` (`render_engine`, `render_engine_gui`) in `maya.launch.xml`
  - SIM world: `random_world.sdf` with `random_world/model.sdf` heightmap scale `40m x 40m`
  - SIM spawn default in `maya.launch.xml`: `spawn_x=4.5` (current baseline pose remains in larger free space)
  - SLAM mapping path: `slam_toolbox` lidar-only with `scan_topic: /scan`
  - Nav2 costmaps: lidar-only active observations (`observation_sources: scan`), depth pointcloud blocks defined but inactive
  - Controller progress checker (relaxed):
    - `required_movement_radius: 0.05`
    - `movement_time_allowance: 30.0`
  - Forward speed caps (increased):
    - `controller_server.FollowPath.vx_max: 1.0`
    - `velocity_smoother.max_velocity[0]: 1.0`
  - Collision monitor enabled:
    - `collision_monitor.FootprintApproach.enabled: True`
- Frozen regression reference result (headless reliability script, current baseline):
  - `tools/nav2_reliability_trials.sh` return-to-reference protocol after warmup forward drive (~10m)
  - `5 / 5` trials `SUCCEEDED`
  - all `error_code: 0`
  - mean end-goal error (map frame) ≈ `0.073 m`
  - median end-goal error (map frame) ≈ `0.030 m`
  - mean trial duration (sim clock) ≈ `104.8 s`
- Rule for next tuning cycle:
  - Run larger-sample regression first (e.g., 25 trials) with this exact baseline before changing Nav2/SLAM/EKF parameters.
  - If a run improves outcomes, immediately checkpoint exact param diffs and launch args before additional changes.

### 8.17 SIM Reliability Regression Follow-up (20 trials across 4x5 batches, 2026-02-24)

- Baseline repeated over 20 total headless trials (multiple 5-trial runs due operator interruptions/startup retries):
  - `18 / 20` `SUCCEEDED` (`90%`)
  - `2 / 20` `TIMEOUT`
  - successful trials continue to report `error_code: 0`
- Aggregate behavior (approx across the 20-trial sample):
  - mean trial duration ≈ `109 s` (sim clock)
  - mean end-goal error (map frame) ≈ `0.11 m`
  - mean `|Gazebo-map displacement|` magnitude difference ≈ `0.55 m`
- Diagnosis update:
  - End-to-end Nav2 remains broadly functional, but completion reliability is not yet fully locked.
  - Timeouts often occur near the current goal acceptance boundary (several successful runs also finish near `xy_goal_tolerance`), so near-goal convergence / acceptance behavior is a likely contributor.
  - Localization consistency (SLAM / odom / EKF) remains a separate tuning target due to persistent map-vs-Gazebo displacement magnitude gap/variance.
- Next tuning priority (one variable at a time):
  1. SLAM localization consistency first (active `slam_toolbox` params in `nav2_params.yaml`, confirm runtime values).
  2. EKF / IMU yaw weighting only if SLAM-side changes do not materially improve consistency.
  3. Costmap inflation shaping for path centering / smooth potentials after localization behavior is characterized.

### 8.18 EKF IMU Yaw A/B (Yaw-Rate-Only) Under 180-Deg Heading Stress (2026-02-24)

- Test setup change (diagnostic stress protocol):
  - `tools/nav2_reliability_trials.sh` configured with post-warmup in-place turn:
    - `POST_WARMUP_TURN_DEG=180`
  - Goal pose remains the original spawn-reference pose/orientation, so return leg requires a large heading correction near goal.
- EKF A/B change under test:
  - `src/maya_bringup/config/ekf.yaml`
  - IMU absolute yaw fusion disabled; IMU yaw-rate fusion kept enabled (`imu0_config` yaw=false, vyaw=true).
- Result (10 trials):
  - `2 / 10` `SUCCEEDED` (`20%`)
  - `8 / 10` `ABORTED` (mostly `error_code: 208`)
  - mean end-goal error became very large (multi-meter) due frequent early aborts / non-convergence
  - several failed trials show low displacement and large remaining goal error after the post-warmup turn
- Diagnosis:
  - In this SIM stress scenario, yaw-rate-only IMU fusion is a major regression versus the prior baseline.
  - Removing absolute IMU yaw significantly degrades heading convergence / consistency after the forced heading reversal.
- Action:
  - Revert this EKF A/B change before continuing SLAM/inflation tuning.
  - If further EKF testing is needed, test `imu0_relative` or covariance changes separately while keeping absolute IMU yaw fused.

### 8.19 IMU Frame-ID Root Cause Fix and EKF Behavior Update (2026-02-24)

- Confirmed IMU integration bug in SIM:
  - `/imu.header.frame_id` was published as a scoped Gazebo sensor name (`maya/base_footprint/imu_sensor`) that did **not** exist in the ROS TF tree.
  - TF tree itself was structurally correct (`base_footprint -> base_link`, `base_link -> imu_link`, `base_link -> lidar_link`).
- Effective fix:
  - set Gazebo IMU sensor `gz_frame_id` explicitly to `imu_link` in `src/maya_description/urdf/sensors_gazebo.xacro`.
- After the fix:
  - `/imu.header.frame_id` now correctly reports `imu_link`.
  - Manual driving/turning showed noticeably improved pose/orientation consistency in RViz and better return-to-near-start behavior.
- EKF A/B result recorded:
  - `imu0_relative: true -> false` (with IMU frame fix in place) made heading behavior worse.
  - Keep `imu0_relative: true` in current SIM baseline.
- Remaining EKF/IMU concern (next tuning target):
  - `/imu.orientation_covariance` is still all zeros in SIM, which can cause EKF yaw over-trust.
  - `/odom` covariances from Gazebo DiffDrive are also zeros; EKF source weighting remains unrealistic.

### 8.20 Clean SIM Baseline Defaults (Lidar SLAM + EKF odom, no depth/VIO) (2026-02-24)

- Launch defaults were updated to restore a cleaner isolation baseline for localization tuning:
  - `use_depth_scan_pipeline:=false` by default in `src/maya_bringup/launch/maya.launch.xml`
  - `use_vio_odom:=false` (default)
  - `use_rtabmap_odom:=false` (default)
- Current default SIM runtime intent:
  - EKF local odom = `/odom` + `/imu`
  - `slam_toolbox` = lidar scan (`/scan`) + EKF odom prior
  - Nav2 unchanged
  - No depth-to-scan conversion / merged scan path by default
  - No RTAB-Map odometry by default
- Terminology note:
  - This is a **loose-coupled EKF odom + lidar SLAM** baseline, not tight LIO.

### 8.21 Optional VIO/RTAB-Map Integration Scaffold (Not Baseline) (2026-02-24)

- Prototype optional EKF visual odometry overlay was drafted locally (for `odom1=/visual_odom` planar fusion), but it is **not part of this develop checkpoint**.
  - Rationale: `/visual_odom` was not yet valid in SIM (`lost`, invalid quaternion / `9999` covariance), so the overlay file is intentionally left out until the source is proven.
- Added launch toggles in `src/maya_bringup/launch/maya.launch.xml`:
  - `use_vio_odom` (loads EKF overlay when true)
  - `use_rtabmap_odom` (launches optional `rtabmap_odom/rgbd_odometry` when true)
- RTAB-Map odom node is currently configured to:
  - remap odometry output to `/visual_odom`
  - `publish_tf:=false` (avoid TF ownership conflict with EKF/SLAM)
  - use RGB + depth camera topics from current Gazebo sensors
- Current observed RTAB-Map state in SIM (with optional path enabled):
  - `/visual_odom` publishes with compatible frame IDs (`odom` -> `base_footprint`)
  - but odometry was `lost` (`/odom_info.lost: true`), with `inliers: 0`, invalid quaternion (`w=0`) and `9999` covariance (not suitable for EKF fusion yet)
  - `rgbd_odometry` in the installed version did not subscribe to `/imu` under the attempted `subscribe_imu` parameter (parameter mismatch/version difference)
- Rule:
  - Do not enable `use_vio_odom:=true` in regression runs until `/visual_odom` is demonstrably valid (non-zero quaternion, non-9999 covariance, stable tracking).
  - If the EKF overlay file is not present in the current branch checkpoint, keep `use_vio_odom:=false`.

### 8.22 Reliability Script Enhancement – Dual-Phase Turn Stress Mode (2026-02-24)

- `tools/nav2_reliability_trials.sh` now supports an optional two-phase per-trial mode to compare easy vs turn-stress behavior under the same startup/map conditions.
- New env vars:
  - `DUAL_PHASE_TURN_STRESS_TEST` (default `false`)
  - `PHASE_A_POST_WARMUP_TURN_DEG` (default `0`)
  - `PHASE_B_POST_WARMUP_TURN_DEG` (default `90`)
- Behavior when enabled:
  1. `phase_a`:
     - warmup forward drive (~10m)
     - return-to-reference NavigateToPose
  2. `phase_b`:
     - second warmup forward drive (~10m)
     - post-warmup in-place turn (default `90°`)
     - return-to-reference NavigateToPose
- Summary improvements:
  - per-phase metrics are recorded under `trial -> phases`
  - aggregate metrics are split by phase (`aggregate.by_phase`)
  - per-trial stress-minus-easy deltas are reported in `phase_comparison`
- This mode is intended for quantitative turning diagnostics (TF drift / map-vs-Gazebo displacement / goal error deltas), while keeping legacy single-phase behavior available by default.

### 8.23 Future-Proofing Rules (Carry Forward) (2026-02-24)

- Treat **message `frame_id` values as first-class integration contracts**, not just TF tree existence.
  - Validate `/imu`, `/scan`, `/odom`, and any future `/visual_odom` headers against TF frames before tuning algorithms.
- For any new odometry source (VIO, GNSS fusion, encoder odom):
  1. verify topic exists,
  2. verify `header.frame_id` and `child_frame_id`,
  3. verify covariance sanity (non-zero, realistic),
  4. only then fuse into EKF/Nav2.
- Keep optional integrations disabled by default until they produce valid data:
  - RTAB-Map/VIO, depth scan pipeline, future GNSS overlays.
- Prefer paired/within-trial diagnostics (easy vs stress) when analyzing turning regressions to reduce startup/transient confounds.

### 8.24 Manual SIM Turning Checkpoint (2026-03-08)

- Standard SIM + autonomy entrypoint remains:
  - `ros2 launch maya_bringup maya.launch.xml`
- Manual operator validation after rebuilding and launching this baseline showed:
  - simulation real-time factor improved materially (operator observed roughly `30% -> 70%`),
  - manual driving and turning are noticeably cleaner than the prior baseline,
  - but in-place / tighter turns still cause SLAM drift and partial map overlap.
- Current interpretation:
  - the recent `slam_toolbox` parameter expansion improved scan-matching behavior,
  - however the remaining failure is **not** solved by adding more generic odometry sources alone,
  - the likely remaining issue is local yaw / turn prior quality in `wheel odom + IMU + EKF`, especially covariance realism and relative weighting during rotation.
- Current architecture reminder:
  - with the present 2D lidar baseline (`/scan`), the active stack is still `wheel-like odom + IMU -> EKF -> slam_toolbox/Nav2`,
  - this is **not** true LIO,
  - for the current rover baseline, prioritize a robust 2D stack before adding any optional LIO path.
- Required next debugging order:
  1. verify `/odom`, `/imu`, and `/odometry/filtered` covariance fields are non-zero and realistic,
  2. validate turn behavior of `odom -> base_footprint` independently of SLAM,
  3. only after odom/yaw quality is characterized, continue additional SLAM tuning.

### 8.25 Turn Drift Diagnostic Result (2026-03-08)

- Added focused turn diagnostic tool:
  - `bash tools/turn_drift_diagnostic.sh`
- Current scripted 180-degree turn result established a stronger root-cause signal than the reliability harness alone:
  - Gazebo truth rotation reached only about `57 deg` before timeout,
  - `/odom` final relative yaw was about `-10.8 deg`,
  - `/odometry/filtered` final relative yaw was about `36.7 deg`,
  - `/imu` final relative yaw was about `19.6 deg`,
  - mean absolute yaw error vs Gazebo remained very large:
    - `/odom` about `95 deg`
    - `/odometry/filtered` about `51 deg`
    - `/imu` about `49 deg`
- Covariance diagnosis from the same run:
  - `/odom.pose.covariance[yaw] = 0`
  - `/imu.orientation_covariance[yaw] = 0`
  - `/odometry/filtered.pose.covariance[yaw]` remained unrealistically tiny (`~5e-10`)
- Interpretation:
  - remaining turn drift is confirmed to be primarily a **local odom / IMU / EKF yaw quality problem**, not just a `slam_toolbox` tuning problem,
  - zero or near-zero source covariance means EKF is overconfident during rotation,
  - if Gazebo truth turn itself is limited while odom yaw diverges badly, check simulated drive kinematics before further SLAM tuning.
- Immediate corrective action taken:
  - corrected Gazebo DiffDrive left/right joint grouping in `src/maya_description/urdf/mobile_base_gazebo.xacro`
  - previous plugin wiring had left/right sides swapped, which is a plausible source of corrupted turning odometry in SIM
- Updated debugging priority after this checkpoint:
  1. re-run `tools/turn_drift_diagnostic.sh` after the DiffDrive fix,
  2. if yaw error drops materially, continue with covariance realism fixes,
  3. only after local yaw behavior is sane should further `slam_toolbox` tuning resume.

### 8.26 Turn Drift Baseline Isolation Outcome (2026-03-12)

- The turn-drift investigation added two new SIM-side baseline tools:
  - `src/maya_bringup/scripts/sim_covariance_relay.py`
  - `tools/turn_drift_diagnostic.sh`
- The current SIM launch path now routes EKF through relayed topics with non-zero covariances:
  - `/odom_with_covariance`
  - `/imu_with_covariance`
- EKF local fusion was adjusted so raw wheel odom no longer contributes absolute yaw pose directly; IMU still contributes heading and yaw rate.
- Sim model changes retained in the current baseline:
  - DiffDrive left/right joint grouping corrected in `src/maya_description/urdf/mobile_base_gazebo.xacro`
  - wheel contact collision changed from STL meshes to cylinders in `src/maya_description/urdf/mobile_base.xacro`
  - explicit wheel friction/slip parameters added in `src/maya_description/urdf/mobile_base_gazebo.xacro`
  - wheel-axis normalization experiment was rejected and reverted; right-wheel mirrored axis remains required in the current model
- Diagnostic conclusion:
  - on `empty.sdf`, fused `/odometry_filtered` became materially better and can track Gazebo truth closely enough to treat the flat-world local odom baseline as improved
  - on `random_world.sdf`, pure in-place turning remains unstable and terrain/contact dominated; rough-terrain turn drift is **not** solved by the current baseline
- Operational interpretation:
  - the current baseline is good enough to checkpoint improvements in diagnostics, covariance realism, and flat-world turn estimation
  - but random heightmap turning should still be treated as a stress case, not as a resolved capability
- Recommended validation split going forward:
  1. use `empty.sdf` for local odom / EKF turn-isolation tests
  2. use `random_world.sdf` for realistic turn-stress and return-to-reference tests
  3. use `tools/nav2_reliability_trials.sh` for acceptance metrics, not pure-turn-only diagnostics on rough terrain

### 8.27 Manual Nav2 Spin Checkpoint (2026-04-14)

- Manual testing confirmed that Nav2 `Spin` is the correct behavior to isolate yaw-turn execution.
  - Do **not** use `NavigateToPose` as the primary test for a pure in-place 180-degree turn.
  - `NavigateToPose` adds planner / BT / goal-feasibility failure paths that obscure yaw-specific diagnosis.
- Current validated manual test command:

```bash
ros2 action send_goal /spin nav2_msgs/action/Spin "{
  target_yaw: 3.1415927,
  time_allowance: {sec: 60, nanosec: 0}
}" --feedback
```

- What the manual `Spin` tests showed consistently:
  - action status often returns `SUCCEEDED` with `error_code: 0`,
  - `angular_distance_traveled` feedback reaches about `-3.16 rad` (roughly `181 deg`),
  - but the final settled heading from:
    - `/odometry/filtered`,
    - `tf odom -> base_footprint`,
    - `tf map -> base_footprint`,
    - `/imu_with_covariance`
    all agree on a final yaw around `163 deg` to `165 deg` for a commanded `180 deg` spin.
- Important interpretation:
  - the localization stack is internally consistent after the maneuver,
  - there is **not** currently evidence of a post-spin settling drift between EKF / TF / IMU,
  - the remaining mismatch is between Nav2 `Spin`'s reported traveled angle and the final estimated heading.
- Operational workaround established from manual tests:
  - commanding about `165 deg` (`180 deg - 15 deg`) produced the closest observed final heading to a true/visual `180 deg` turn.
  - Treat this as an **empirical SIM workaround**, not as confirmed ground truth.
- What this does **not** prove yet:
  - it does **not** yet prove whether the rover physically under-rotates or whether the final yaw estimate is biased during the maneuver,
  - because full Gazebo truth orientation for the same post-spin sample has not yet been captured and compared.
- Required next diagnostic to close the loop:
  1. run one clean `Spin` test,
  2. immediately capture:
     - `/odometry/filtered`,
     - `/imu_with_covariance`,
     - `tf odom -> base_footprint`,
     - full Gazebo dynamic pose block for `maya` including orientation quaternion,
  3. compare final estimated yaw against Gazebo truth yaw.
- Practical rule going forward:
  - for Nav2 yaw-behavior testing, use `Spin`,
  - for low-level drivetrain / odom isolation, raw `/cmd_vel` turn tests remain useful,
  - do not mix those two conclusions in one metric.
- Script status note:
  - `tools/nav2_reliability_trials.sh` was partially adapted to use `Spin` in the post-warmup turn stage,
  - but this should be treated as **experimental** until the manual `Spin` behavior is fully understood and the final-heading vs feedback mismatch is explained.

# AGENTS – Autonomous Navigation Mission (Maya Rover)

These instructions apply to the entire `AutoNav_Mission_2026` repository.

You are working on the Autonomous Navigation stack for the rover **Maya** for the University Rover Challenge 2026. The project is built around **ROS 2 (Jazzy)**, **Gazebo Sim (Harmonic / gz-sim 8)**, and a simulation-first workflow; code and launch files here are meant to transfer to Jetson hardware later.

Your job is to:
- Make small, correct changes.
- Respect existing structure and intent.
- Prefer clarity and robustness over “clever” one-liners.

If these instructions ever conflict with explicit task instructions, the task instructions win.

---

## 0. Current status (2026-02-03)

- Default sim launch now targets `random_world.sdf` and keeps Gazebo GUI enabled by default (no `-s` in `gz_args`).
- A new random map was generated via `tools/map_gen/build/map_generator`; the updated heightmap PNG is in `src/maya_bringup/world/random_world/media/materials/texture/random_world.png`.
- `island.sdf` remains available and working as a fallback world.
- Open issues:
  - Random heightmap world alignment and robot spawn placement are still not correct.
  - `random_world/model.sdf` likely needs to be adapted to the newly generated map dimensions and scale.
  - The generated `random_map.pgm` has not been moved into a runtime maps folder yet.

## 0.1 Next steps / plan

- Update `src/maya_bringup/world/random_world/model.sdf` to match the generated heightmap’s pixel dimensions and intended real‑world scale.
- Confirm / adjust heightmap `size` (X, Y, Z) and `pos` so the terrain sits at Z=0 and the robot spawns above ground.
- Decide and document a stable spawn pose for `maya` that avoids obstacles.
- Move / register generated map artifacts (`random_map.pgm` and any YAML) into `src/maya_bringup/maps/` and ensure Nav2 can load them.

## 0.2 Autonomy status (2026-02-15)

Current focus is hardware bringup on Jetson (ROS 2 Humble) with ZED + IMU + Nav2.

Latest validation:
- `zed-ros2-wrapper` builds successfully in this workspace with ZED SDK `5.1.0`.
- ZED standalone launch works (`zed_wrapper zed_camera.launch.py camera_model:=zed2i enable_ipc:=false`).
- NITROS transport is available during `zed_components` build.

Current validated HW bringup command (Jetson, Humble):

```bash
ros2 launch maya_bringup maya.launch.xml \
  sim:=false rviz:=false \
  use_zed:=true \
  zed_camera_model:=zed2i \
  zed_enable_ipc:=false \
  odom_topic:=/zed/zed_node/odom \
  imu_topic:=/zed/zed_node/imu/data \
  pointcloud_topic:=/zed/zed_node/point_cloud/cloud_registered
```

**Roadmap to autonomy completeness (rough %)**  
Percentages reflect readiness for field use, not just compile/run.

- **TF + Robot Description**: 70%
  - URDF publishes base + sensors.
  - Wheel meshes and joint states still need validation on HW.
- **Odometry (wheel encoders)**: 20%
  - Plan: PCB computes `/odom` and publishes TF `odom -> base_footprint`.
  - Raw encoder ticks also published for cross-check on Jetson.
- **IMU integration (primary + backup)**: 40%
  - Primary: PCB IMU.
  - Backup: ZED IMU (optional secondary input).
- **LiDAR + scan pipeline**: 50%
  - `/scan` and `/scan_merged` must be verified on HW.
- **2D SLAM (slam_toolbox)**: 60%
  - Mapping runs on HW bringup command above.
  - Drift still present and requires IMU/odom refinement.
- **Nav2 navigation**: 50%
  - Costmaps, planners, and controllers run in HW mode.
  - Needs more stable odom + IMU alignment before long autonomous runs.
- **3D mapping (RTAB-Map / equivalent)**: 10%
  - Planned after LiDAR + encoder odom are stable.
- **GNSS navigation**: 0%
  - Planned after wheel encoders + joint states + robot_state_publisher + IMU2 + 3D mapping are validated.

**Gate to start GNSS work**

1. Wheel encoder odom published from PCB (`/odom` + `odom -> base_footprint` TF).
2. Wheel joint states published (`/joint_states`) for RViz + debugging.
3. robot_state_publisher confirmed on Jetson (Humble) and remote RViz.
4. Secondary IMU (IMU2) optionally fused or at least logged.
5. LiDAR + 3D mapping pipeline producing a consistent local map.

**Immediate next logical steps (execution order)**

1. Implement encoder + odom publishing in Core System (PCB repo) with stable timestamps and diagnostics.
2. Publish wheel raw telemetry (`/wheel/left_ticks`, `/wheel/right_ticks`) and `/odom` from PCB on Jetson network.
3. Configure EKF to fuse PCB odom + primary IMU first; add ZED odom as secondary source after PCB odom is stable.
4. Add `/joint_states` publishing path (PCB or Jetson converter) to restore wheel visualization/debug fidelity.
5. Run short HW validation loops and tune EKF/SLAM covariances based on logged drift and TF consistency.

---

## 1. Tech stack and key tools

- **ROS 2**:
  - Jazzy on development PC (`develop` branch).
  - Humble on Jetson (`humble-jetson` branch).
- **Simulation**: Gazebo Sim (Ignition / gz-sim 8).
- **Languages**:
  - ROS 2 packages: C++, Python, XML launch, Xacro, YAML.
  - Some C / embedded work elsewhere (follow MISRA-C style when touching C).
- **Build system**: `colcon` with `--symlink-install`.
- **Repository layout (current)**:
  - `Docs/` – documentation, diagrams, README.
  - `src/maya_description/` – robot description (URDF/Xacro), meshes, RViz config.
  - `src/maya_bringup/` – simulation bringup (launch, worlds, bridges, joystick, etc.).
  - More Nav2 / perception / MCU bridge packages will appear later; don’t invent them unless explicitly requested.

---

## 2. General behavior

When editing code or configs:

1. **Read before you write.**  
   Before changing any file, scan it completely to understand style, naming, and patterns already in use.

2. **Minimize diffs.**  
   Prefer the smallest change that solves the problem. Do not refactor large areas unless explicitly asked.

3. **Keep things ROS-idiomatic.**
   - Use standard ROS 2 naming where possible (`*_bringup`, `*_description`, etc.).
   - Prefer parameters and launch arguments over hard-coded constants when it clearly improves flexibility.

4. **No “mystery behavior”.**  
   If you add logic, add at least a short comment when the intent would not be obvious to someone new to the project.

5. **Don’t break existing workflows.**  
   Anything that currently builds and launches (e.g., `maya.launch.xml` using `island.sdf`) must keep working unless the instructions explicitly say to replace it.

---

## 3. Build and run commands

When asked to verify changes, prefer these commands:

### Build

From repo root:

```bash
colcon build --symlink-install
source install/setup.zsh   # or setup.bash as appropriate
````

If you only touched one package (e.g. `maya_bringup`):

```bash
colcon build --symlink-install --packages-select maya_bringup
source install/setup.zsh
```

### Run main simulation (current baseline)

From repo root, after sourcing:

```bash
ros2 launch maya_bringup maya.launch.xml
```

This is expected to:

* Start Gazebo Sim with a world (`world/island.sdf` or a random world SDF).
* Spawn the `maya` robot from `maya_description` via `robot_state_publisher` + `ros_gz_sim create`.
* Start `ros_gz_bridge` using `config/gazebo_bridge.yaml`.
* Start RViz with `maya_description.rviz`.

When you modify launch files or worlds that affect this flow, ensure this command still works or is clearly updated in documentation/comments.

The explicit SIM entrypoint is:

```bash
ros2 launch maya_bringup maya_sim.launch.xml
```

---

## 4. Directory-specific guidance

### 4.1 `Docs/`

* Markdown and diagrams only; do not introduce build logic here.
* If you update diagrams or high-level docs, keep terminology consistent with the existing README (e.g. “Maya”, “Autonomous Navigation Mission”, “Jetson”, “MCU bridge”).
* Avoid adding very large assets into `Docs/Images`; prefer vector or small PNGs.

### 4.2 `src/maya_description/`

Purpose: robot model, meshes, and visualization.

* Keep Xacro modular:

  * `common_properties.xacro` – shared materials/constants.
  * `mobile_base.xacro`, `mobile_base_gazebo.xacro` – base structure & simulation tags.
  * `zed_mount.xacro` – camera mount / sensor additions.
  * `maya_description.urdf.xacro` – top-level robot description that composes the pieces.
* When editing, **do not** change link/joint names arbitrarily; they are used by Gazebo and Nav2.
* Inertia, mass and collision geometry may be simplified, but keep them physically reasonable and consistent.
* If you introduce new visual meshes or sensors, keep filenames and case consistent and document them briefly in comments.

### 4.3 `src/maya_bringup/`

Purpose: simulation bringup and (later) hardware bringup.

Current contents:

* `launch/maya_core.launch.xml`
  Core autonomy stack for SIM and HW:

  * Launch `robot_state_publisher`.
  * Run scan pipeline (`scan_frame_relay.py`, `pointcloud_to_laserscan`, `scan_merge.py`).
  * Launch `robot_localization` EKF.
  * Launch Nav2 bringup.
  * Start RViz with `maya_description.rviz`.

* `launch/maya_sensors_sim.launch.xml`
  SIM-only sensor layer:

  * Launch `ros_gz_sim` (`gz_sim.launch.py`) with `gz_args`.
  * Call `ros_gz_sim create` using `/robot_description`.
  * Start `ros_gz_bridge` with `config/gazebo_bridge.yaml`.

* `launch/maya_sim.launch.xml`
  SIM entrypoint: includes `maya_sensors_sim.launch.xml` + `maya_core.launch.xml` with `use_sim_time=true`.

* `launch/maya_hw.launch.xml`
  HW entrypoint: includes `maya_core.launch.xml` with `use_sim_time=false` and optional Jetson sensors.

* `launch/maya.launch.xml`
  Legacy alias for `maya_sim.launch.xml` (must keep working).

* `config/gazebo_bridge.yaml`
  Defines ROS↔GZ bridge topics (e.g., `/cmd_vel`, `/joint_states`; `/tf` only when not using `robot_localization`).

* `config/joy.yaml`
  Used with `teleop_twist_joy` / `joy_linux` for joystick teleop.

* `world/`

  * `island.sdf` – existing world that is known to work.
  * `random_world.sdf` – top-level world using a heightmap model.
  * `random_world/` – a Gazebo model:

    * `model.config`
    * `model.sdf` (heightmap model using `media/materials/texture/random_world.png`)
    * `media/materials/texture/random_world.png` – generated by the map generator.

When editing `maya_bringup`:

* Keep launch files small and readable; prefer `<let>` for paths and a minimal number of arguments.
* Avoid hard-coding machine-specific paths; always derive from `$(find-pkg-share ...)` or `ros2 pkg prefix`.
* If adding new bridge topics, make sure they match actual topics on both ROS and Gazebo sides.

---

---

## 4.4 HITL / Hardware-in-the-loop bringup (SIM vs HW modes)

Purpose: keep a single operator experience (RViz2 + Nav2 + SLAM + EKF + costmaps) while swapping sensor/odom sources.

### Design goals
- The autonomy “core stack” must run without Gazebo present.
- Only the sensor-provider layer changes between simulation and real hardware.
- Minimal duplication: prefer launch arguments + small YAML overlays over duplicating entire configs.

### Layered launch structure (maya_bringup)
- `launch/maya_core.launch.xml`
  - Runs in both SIM and HW.
  - Contains: `robot_state_publisher`, scan pipeline (`scan_frame_relay.py`, `pointcloud_to_laserscan`, `scan_merge.py`), `robot_localization` EKF, Nav2 bringup, RViz.
  - Accepts `use_sim_time` argument and applies it consistently.
- `launch/maya_sensors_sim.launch.xml`
  - SIM-only: Gazebo Sim Harmonic + robot spawn + `ros_gz_bridge` using `config/gazebo_bridge.yaml`.
- `launch/maya_sim.launch.xml`
  - SIM entrypoint: includes sensors_sim + core with `use_sim_time=true`.
- `launch/maya_hw.launch.xml`
  - HW entrypoint: includes core with `use_sim_time=false` and optionally includes a guarded Jetson sensor bringup include.
  - Must not launch Gazebo and must not crash if Jetson-specific packages are absent.

`launch/maya.launch.xml` must continue to work as the primary developer entrypoint (it may wrap/alias `maya_sim.launch.xml`).

### Stable Topic / TF contract
Keep these stable across SIM and HW modes so Nav2/RViz config remains unchanged:

Raw inputs (provided by Gazebo in SIM; by Jetson in HW):
- `/scan` (LaserScan)
- `/depth_camera/points` (PointCloud2)
- `/imu` (sensor_msgs/Imu)
- `/odom` (nav_msgs/Odometry)

Derived/core outputs:
- `/scan_fixed` (relay output, frame_id=lidar_link)
- `/scan_depth` (from depth pointcloud)
- `/scan_merged` (Nav2/SLAM scan source)
- `/odometry/filtered` (EKF output)
- `/tf`, `/tf_static`

TF ownership rules:
- Odometry side publishes `odom -> base_footprint` (typically EKF or wheel odom)
- Localization (slam_toolbox / AMCL) publishes `map -> odom`
- URDF publishes static transforms from `base_* -> sensor frames`

### Time rules (critical)
- SIM mode: `/clock` exists; core nodes use `use_sim_time=true`.
- HW mode: no `/clock` required; core nodes use wall time (`use_sim_time=false`).
Do not mix sim-time and wall-time within the same autonomy graph.

### Debug commands (TF + time)
- `ros2 run tf2_ros tf2_echo map odom`
- `ros2 run tf2_ros tf2_echo odom base_footprint`
- `ros2 topic echo --once /clock` (SIM only)
- `ros2 topic hz /scan_merged`
- `ros2 topic hz /depth_camera/points`

## 5. Style and safety notes

### 5.1 C / embedded (if touched here later)

* Follow **MISRA-C-like** principles:

  * No hidden side effects in macros.
  * Avoid dynamic allocation where possible.
  * Prefer explicit types (`uint32_t`, `int32_t`, etc.).
  * Avoid implicit casts; use explicit, checked casts.
* Always check error returns and handle them; don’t leave TODOs for critical paths.

### 5.2 C++ / ROS 2

* Use RAII and smart pointers (`std::shared_ptr` / `std::unique_ptr`).
* No bare `new`/`delete` unless absolutely required.
* Follow ROS 2 naming conventions: node names lower_snake_case, parameters lower_snake_case.
* Prefer `rclcpp::Logger` for logging.

### 5.3 Launch / YAML / SDF

* Keep XML/YAML formatted and indented consistently (2 spaces).
* Comments should explain *why* something is there (e.g., workarounds, known limitations), not just *what* it does.
* For SDF/world files:

  * Use relative URIs that resolve via Gazebo resource paths:

    * Model URIs: `model://random_world`
    * Textures: `file://media/materials/texture/random_world.png` or similar **only if** they resolve correctly under `GZ_SIM_RESOURCE_PATH`.
  * Avoid overly complex shaders or plugins unless necessary.

---

## 6. Checks to run (when possible)

When a task involves code or configuration changes, prefer to run:

1. Build:

   ```bash
   colcon build --symlink-install
   ```
2. Source:

   ```bash
   source install/setup.zsh
   ```
3. Basic sim sanity (if relevant to the change):

   ```bash
   ros2 launch maya_bringup maya.launch.xml
   ```

   or, for world-only debugging:

   ```bash
   gz sim "$(ros2 pkg prefix --share maya_bringup)/world/random_world.sdf"
   ```

If a task only touches documentation (Markdown), you do not need to run code, but you should still ensure that file paths and command examples are correct and consistent.

---

## 7. Current task brief – Nav2 bringup, SLAM, mapping/localization

**Context**

* Focus is now on Nav2 bringup with SLAM to ensure mapping and localization are reliable.
* The IMU config has been edited and performance improved, but the map still drifts.

**What we are trying to achieve right now**

* A stable Nav2 bringup path with SLAM that reduces or eliminates map drift.
* Clear, minimal configuration changes that improve localization consistency.
* A repeatable refinement workflow that balances SIM-first tuning with short HW validation cycles.
* Tight integration with Core System firmware for encoder-derived odometry as the next foundation milestone.

**Refinement strategy (SIM vs HW)**

1. Tune algorithmic and config behavior in simulation first (Nav2, SLAM, EKF params).
2. Validate only high-value changes on Jetson hardware in short, controlled runs.
3. Treat HW runs as acceptance tests for TF integrity, topic rates, and drift, not first-pass tuning.
4. Keep branch diffs small and isolate changes by subsystem (IMU, odom, SLAM, costmaps).

**Cross-repo collaboration rule (AutoNav + Core System)**

1. Firmware (Core System repo) owns encoder acquisition, low-level odom integration, and diagnostic flags.
2. AutoNav repo owns EKF/Nav2/SLAM topic contracts, fusion strategy, and launch wiring.
3. Any topic/frame contract change must be documented in both repos before testing.

**How to treat this in changes**

When a task mentions Nav2, SLAM, or IMU tuning:

1. Prefer configuration-level fixes first (SLAM params, Nav2 params, IMU integration).
2. Keep diffs minimal and document why changes are needed if not obvious.
3. Preserve existing launch entry points unless explicitly asked to restructure.

---
