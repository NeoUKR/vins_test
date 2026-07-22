Changes in 4.0.0:
  - ROS2 is the default runtime
  - add --ros1 to use ROS1
  - add standalone -test mode compatible with v2.2.0
  - add -fixpos stable start locking in standalone mode
  - use ros2 launch / ros2 bag by default
  - retain automatic parameter tuning from v3.8.0

Changes in 3.8.0:
  - each parameter value can be tested multiple times: test.repeats in test.json
  - aggregate score is selected by test.aggregation: median or mean
  - CSV/report include valid_repeats and score stddev

Changes in 3.7.0:
  - adds FIXPOS start locking for parameter tuning
  - fixpos settings are loaded from test.json: enabled/time/eps/max_wait
  - Start position is the averaged stable position, not the first odometry sample
  - fixes verdict printing to compare objective score, not DeltaXY only

Changes in 3.6.0:
  - active parameter is selected explicitly by CLI: -param td / -param acc_n
  - test.json can contain a "parameters" dictionary with per-parameter search settings
  - flat old-style test.json is still supported for backward compatibility

Changes in 3.5.0:
  - objective can be selected in JSON: delta_xy, delta_xyz, composite
  - composite score can penalize both XY closing error and 3D/Z drift
  - records accumulated path Distance XY/XYZ in CSV and reports
  - safer probe-both-sides search per step to avoid early wrong direction lock

Changes in 3.4.0:
  - quieter roslaunch output: position spam is filtered by default
  - more informative trial report with search step, parameter, direction, and verdict
  - clearer baseline/best comparison after every trial
  - adds --roslaunch-log-mode quiet|warn|all and --stop-file

Changes in 3.3.0:
  - search settings are loaded from JSON, default: test.json
  - parameter name can be loaded from JSON field "param"
  - uses directional local search instead of full range scan
  - limits search by max_fails, max_trials, max_step_reductions

Changes in 3.2.2:
  - rosbag stdout is hidden by default to avoid [rosbag] spam during trials
  - adds separate --show-rosbag-output / --hide-rosbag-output and --show-roslaunch-output / --hide-roslaunch-output flags

Changes in 3.2.1:
  - all ROS shell commands are executed after sourcing ROS setup.bash
  - adds --ros-setup argument, default: ~/ros1_noetic_ws/install_isolated/setup.bash
  - prints resolved ROS setup path and checks roslaunch/rosbag availability

Changes in 3.2.0:
  - diagnostic output for roslaunch and rosbag start: command, PID, process output
  - rosbag is started as: rosbag play --clock <bag> when --clock is enabled
  - primary topic activity check for camera and IMU topics before waiting for odometry
  - clear FAIL reason when camera/IMU/odometry data is missing
  - keeps all 3.1.0 behavior