# VINS Test

[Українська](README.md) | **English**

`VINS Test` is a command-line utility for validating VINS operation and automatically tuning numeric parameters in a YAML configuration. It supports ROS2 (the default) and ROS1, launches VINS and rosbag, collects odometry, evaluates return error, and saves test results to CSV.

Check the current application version and selected ROS variant with:

```bash
python3 vins_test.py --version
```

## Features

- standalone monitoring mode for an already running VINS instance;
- automatic local search for the best YAML parameter value;
- ROS2 and ROS1 support;
- camera, IMU, and odometry message monitoring;
- total CPU and GPU utilization monitoring during trials;
- RAM utilization and CPU temperature monitoring;
- stable starting-position lock (`FIXPOS`);
- repeated trials with median or mean aggregation;
- `Delta XY`, `Delta XYZ`, or composite objective scoring;
- configuration backup and automatic restoration;
- CSV output for all trial results.

On Raspberry Pi, the utility reads accumulated V3D queue runtime from `gpu_stats`, calculates utilization from consecutive samples, and uses the busiest queue as overall GPU load. Other DRM drivers can use `gpu_busy_percent`/`gt_busy_percent`, with `nvidia-smi` as the NVIDIA fallback. If the driver exposes no supported source, the trial continues; GPU metrics are reported as `unavailable` and the corresponding CSV fields remain empty.

## Requirements

- Linux and Python 3;
- ROS2 or ROS1 with its Python modules available;
- an installed and built VINS workspace;
- a rosbag containing camera and IMU topics;
- a VINS YAML configuration containing the numeric parameter to tune.

The default paths match the current project workspace layout. If your setup differs, provide custom paths with `--ros-setup`, `--vins-setup`, `--config`, and the other command-line options.

View all available arguments with:

```bash
python3 vins_test.py --help
```

## Quick start: validating VINS

Start VINS and the camera and IMU data sources first. Then open another terminal and run:

```bash
python3 vins_test.py --test
```

At startup, standalone mode displays the current application version, selected ROS variant, and the odometry and IMU topics. Its session report also includes minimum, maximum, and average CPU/GPU utilization.

ROS2 is used by default. Add `--ros1` to use ROS1:

```bash
python3 vins_test.py --ros1 --test
```

The default topics are:

- odometry — `/vins_estimator/odometry`;
- camera — `/cam0/image_raw`;
- IMU — `/imu0`.

Override them when necessary:

```bash
python3 vins_test.py --test \
  --odom-topic /vins_estimator/odometry \
  --cam-topic /camera/image_raw \
  --imu-topic /imu/data
```

During a session, the program displays the current position, message count, and traveled distance. Stopping VINS, losing IMU data, or pressing `Ctrl+C` ends the active session and prints the resulting `Delta XY`, `Delta XYZ`, and path length.

To begin measurement only after the position becomes stable, use:

```bash
python3 vins_test.py --test --fixpos \
  --fixpos-time 5 \
  --fixpos-eps 0.02 \
  --fixpos-max-wait 60
```

## Automatic parameter tuning

For every candidate value, the program:

1. changes the selected parameter in the YAML file;
2. launches VINS;
3. plays the rosbag;
4. collects odometry until playback finishes;
5. calculates the objective score and writes the result to CSV;
6. probes values on both sides of the current best value and gradually reduces the step.

ROS2 example:

```bash
python3 vins_test.py -param td \
  --test-json test.json \
  --config ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml \
  --bag ~/bags/test/data \
  --ros-setup ~/ros2_jazzy/install/setup.bash \
  --vins-setup ~/vins_ws/install/setup.bash
```

ROS1 example:

```bash
python3 vins_test.py --ros1 -param td \
  --test-json test.json \
  --config ~/vinsmono_ws/src/VINS-Mono/config/euroc/euroc_config.yaml \
  --bag ~/bags/test/data.bag \
  --ros-setup ~/ros1_noetic_ws/install_isolated/setup.bash \
  --vins-setup ~/vinsmono_ws/devel/setup.bash
```

Default launch commands:

- ROS2: `ros2 launch vins_estimator euroc.launch.py`;
- ROS1: `roslaunch vins_estimator euroc.launch`.

Use `--launch-cmd` to replace the launch command completely.

## Configuring `test.json`

The active parameter is always selected with `-param`. One JSON file can contain settings for multiple parameters:

```json
{
  "objective": "composite",
  "delta_xy_weight": 1.0,
  "delta_xyz_weight": 1.0,
  "distance_xy_weight": 0.0,
  "expected_distance_xy": null,
  "defaults": {
    "max_fails": 2,
    "max_trials": 25,
    "max_step_reductions": 4,
    "step_reduce_factor": 2.0
  },
  "parameters": {
    "td": {
      "step": 0.003,
      "min_step": 0.0001
    },
    "acc_n": {
      "step": 0.02,
      "min_step": 0.001,
      "max_trials": 20
    }
  },
  "fixpos": {
    "enabled": true,
    "time": 5.0,
    "eps": 0.02,
    "max_wait": 30.0
  },
  "test": {
    "repeats": 3,
    "aggregation": "median"
  }
}
```

After merging with `defaults`, each parameter must provide:

- `step` — initial search step;
- `min_step` — minimum search step;
- `max_fails` — allowed failed probes at the current step;
- `max_trials` — maximum number of trials;
- `max_step_reductions` — maximum number of step reductions.

The `objective` field accepts:

- `delta_xy` — minimize horizontal error between the start and end positions;
- `delta_xyz` — minimize three-dimensional error;
- `composite` — weighted sum of `Delta XY`, `Delta XYZ`, and optionally the `Distance XY` error when an expected path length is configured.

Supported `test.aggregation` values are `median` and `mean`.

## Results and configuration safety

Before tuning begins, a backup is created next to the YAML file:

```text
euroc_config.yaml.backup_vins_tune_YYYYMMDD_HHMMSS
```

By default, the original configuration is restored when the program finishes or is stopped. Add `--keep-best` to leave the best value found in the YAML file.

Results are saved to `vins_param_tune_results.csv`. Use `--csv` to select another path. Both the CSV file and console report include minimum, maximum, and average CPU/GPU load, RAM load and used memory, and CPU temperature for every result.

## Controlling execution

- `Ctrl+C` — stop the program and active VINS/rosbag processes;
- `--stop-file PATH` — select a safe-stop file;
- `--dry-run` — print candidate values without running trials;
- `--keep-best` — retain the best value found;
- `--show-rosbag-output` — display rosbag output;
- `--roslaunch-log-mode quiet|warn|all` — choose VINS launch output verbosity;
- `--no-clock` — do not pass the simulated-clock option to rosbag.

The default stop file is `~/scripts/STOP`. Create it from another terminal with:

```bash
touch ~/scripts/STOP
```

After detecting the file, the program removes it, finishes the current work, stops child processes, and restores the original configuration. If automatic removal fails because of permissions, delete `STOP` before the next run.

## Changelog

See [docs/CHANGELOG.md](docs/CHANGELOG.md) for the version history.
