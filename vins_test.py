#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vins_test.py

VINS test monitor and automatic parameter tuner.

Example:
  python3 vins_test.py -param td \
    --test-json test.json \
    --config ~/vinsmono_ws/src/VINS-Mono/config/euroc/euroc_config.yaml \
    --bag ~/bag/NeoIndra03/test/indoor/5/data.bag

  python3 vins_test.py -param acc_n \
    --test-json test.json \
    --bag ~/bag/NeoIndra03/test/indoor/5/data.bag

Default objective:
  minimize final Delta XY from /vins_estimator/odometry.

What it does per cycle:
  1. edits one YAML parameter in euroc_config.yaml
  2. starts: roslaunch vins_estimator euroc.launch
  3. starts: rosbag play data.bag
  4. waits until rosbag exits
  5. stops VINS
  6. calculates Delta XY and records the result

Notes:
  - The original config is backed up before the first change.
  - By default the original config is restored at the end.
  - Use --keep-best to leave the best found value in the YAML.
"""

import argparse
import atexit
import csv
import json
import math
import os
import re
import signal
import shlex
import statistics
import subprocess
import threading
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple
from collections import deque

APP_NAME = "VINS Test"
VERSION = "4.0.1"

ROS_VERSION = 1 if "--ros1" in sys.argv else 2
ROS_NODE = None
ROS_SPIN_THREAD = None
rospy = None
rclpy = None
Odometry = Image = Imu = None

def ros_is_shutdown():
    if ROS_VERSION == 1:
        return bool(rospy and rospy.is_shutdown())
    return not (rclpy and rclpy.ok())

def init_ros_node(args, name):
    global ROS_NODE, ROS_SPIN_THREAD, rospy, rclpy, Odometry, Image, Imu
    if ROS_VERSION == 1:
        ROS_WS = "/home/rpi/ros1_noetic_ws/install_isolated"
        sys.path.insert(0, ROS_WS + "/lib/python3/dist-packages")
        try:
            import rospy as _rospy
            from nav_msgs.msg import Odometry as _Odometry
            from sensor_msgs.msg import Image as _Image, Imu as _Imu
        except Exception as e:
            raise RuntimeError(f"cannot import ROS1 Python modules: {e}")
        rospy, Odometry, Image, Imu = _rospy, _Odometry, _Image, _Imu
        rospy.init_node(name, anonymous=True)
        rospy.Subscriber(args.odom_topic, Odometry, odom_cb, queue_size=100)
        rospy.Subscriber(args.cam_topic, Image, cam_cb, queue_size=100)
        rospy.Subscriber(args.imu_topic, Imu, imu_cb, queue_size=1000)
    else:
        try:
            import rclpy as _rclpy
            from rclpy.executors import SingleThreadedExecutor
            from nav_msgs.msg import Odometry as _Odometry
            from sensor_msgs.msg import Image as _Image, Imu as _Imu
        except Exception as e:
            raise RuntimeError(f"cannot import ROS2 Python modules; source ROS2 setup.bash first: {e}")
        rclpy, Odometry, Image, Imu = _rclpy, _Odometry, _Image, _Imu
        rclpy.init(args=None)
        ROS_NODE = rclpy.create_node(name)
        ROS_NODE.create_subscription(Odometry, args.odom_topic, odom_cb, 100)
        ROS_NODE.create_subscription(Image, args.cam_topic, cam_cb, 100)
        ROS_NODE.create_subscription(Imu, args.imu_topic, imu_cb, 1000)
        executor = SingleThreadedExecutor()
        executor.add_node(ROS_NODE)
        ROS_SPIN_THREAD = threading.Thread(target=executor.spin, daemon=True)
        ROS_SPIN_THREAD.start()

def shutdown_ros_node():
    global ROS_NODE
    if ROS_VERSION == 2 and rclpy is not None and rclpy.ok():
        if ROS_NODE is not None:
            ROS_NODE.destroy_node()
        rclpy.shutdown()

DEFAULT_CONFIG = "~/vinsmono_ws/src/VINS-Mono/config/euroc/euroc_config.yaml"
DEFAULT_LAUNCH_CMD = None
DEFAULT_BAG = "data.bag"
DEFAULT_ODOM_TOPIC = "/vins_estimator/odometry"

last_odom = None
last_odom_rx = 0.0
odom_seq = 0

cam_seq = 0
imu_seq = 0
last_cam_rx = 0.0
last_imu_rx = 0.0

active_processes = []
stop_requested = False


def request_stop(reason: str = ""):
    global stop_requested
    stop_requested = True
    if reason:
        print(f"\n[STOP] {reason}")


def signal_handler(sig, frame):
    request_stop("Ctrl+C received")
    stop_all_processes()


def check_stop_file(args) -> bool:
    if not getattr(args, "stop_file", None):
        return False
    path = expand_path(args.stop_file)
    if os.path.exists(path):
        try:
            os.remove(path)
        except Exception:
            pass
        request_stop(f"stop file detected: {path}")
        return True
    return False


def odom_cb(msg):
    global last_odom, last_odom_rx, odom_seq
    last_odom = msg
    last_odom_rx = time.time()
    odom_seq += 1


def cam_cb(msg):
    global cam_seq, last_cam_rx
    cam_seq += 1
    last_cam_rx = time.time()


def imu_cb(msg):
    global imu_seq, last_imu_rx
    imu_seq += 1
    last_imu_rx = time.time()


def get_pos(msg) -> Tuple[float, float, float]:
    p = msg.pose.pose.position
    return (float(p.x), float(p.y), float(p.z))


def dist_xy(a, b) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def dist_xyz(a, b) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def format_pos(pos) -> str:
    if pos is None:
        return "x= ---       y= ---       z= ---"
    return f"x={pos[0]: .6f}  y={pos[1]: .6f}  z={pos[2]: .6f}"


def clear_current_line():
    sys.stdout.write("\r\033[2K")
    sys.stdout.flush()


def print_live_position(trial_index: int, value: float, msg_count: int, start_pos, end_pos, end_time, start_time):
    now = time.time()
    stamp = time.strftime("%H:%M:%S")
    age = now - last_odom_rx if last_odom_rx > 0.0 else 0.0
    delta = dist_xy(start_pos, end_pos) if start_pos is not None and end_pos is not None else 0.0
    text = (
        f"[{stamp}] TRIAL #{trial_index} {value:.9f} | "
        f"POS {format_pos(end_pos)} | "
        f"Delta XY={delta:.6f} m | "
        f"count={msg_count} | age={age:.2f}s"
    )
    clear_current_line()
    sys.stdout.write(text)
    sys.stdout.flush()


def finish_live_line():
    sys.stdout.write("\n")
    sys.stdout.flush()


@dataclass
class TrialResult:
    index: int
    value: float
    score_delta_xy: Optional[float]
    delta_xyz: Optional[float]
    duration: float
    odom_messages: int
    start_pos: Optional[Tuple[float, float, float]]
    end_pos: Optional[Tuple[float, float, float]]
    ok: bool
    reason: str
    distance_xy: Optional[float] = None
    distance_xyz: Optional[float] = None
    objective_score: Optional[float] = None
    repeat_count: int = 1
    valid_repeats: int = 1
    score_stddev: Optional[float] = None
    delta_xy_stddev: Optional[float] = None
    delta_xyz_stddev: Optional[float] = None


@dataclass
class SearchSettings:
    param: str
    step: float
    min_step: float
    max_fails: int
    max_trials: int
    max_step_reductions: int
    step_reduce_factor: float = 2.0
    improvement_epsilon: float = 1e-9
    objective: str = "composite"
    delta_xy_weight: float = 1.0
    delta_xyz_weight: float = 1.0
    distance_xy_weight: float = 0.0
    expected_distance_xy: Optional[float] = None


@dataclass
class FixposSettings:
    enabled: bool = True
    time: float = 5.0
    eps: float = 0.02
    max_wait: float = 30.0


@dataclass
class TestSettings:
    repeats: int = 1
    aggregation: str = "median"


def expand_path(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path))


def load_search_settings(json_path: str, cli_param: Optional[str]) -> SearchSettings:
    """
    Load search settings from test.json.

    Preferred 3.6.0 format:
      {
        "objective": "composite",
        "parameters": {
          "td": {"step": 0.003, ...},
          "acc_n": {"step": 0.02, ...}
        }
      }

    The active parameter MUST be selected from CLI:
      -param td
      -param acc_n

    Old flat format is still supported:
      {"param": "td", "step": 0.003, ...}
    In old format, -param overrides JSON "param".
    """
    path = expand_path(json_path)
    if not os.path.exists(path):
        raise RuntimeError(
            f"Search JSON not found: {path}\n"
            f"Create test.json. Example:\n"
            f"{{\n"
            f"  \"objective\": \"composite\",\n"
            f"  \"parameters\": {{\n"
            f"    \"td\": {{\"step\": 0.003, \"min_step\": 0.0001, \"max_fails\": 2, \"max_trials\": 25, \"max_step_reductions\": 4}},\n"
            f"    \"acc_n\": {{\"step\": 0.02, \"min_step\": 0.001, \"max_fails\": 2, \"max_trials\": 20, \"max_step_reductions\": 4}}\n"
            f"  }}\n"
            f"}}"
        )
    with open(path, "r", encoding="utf-8") as f:
        root = json.load(f)

    required = ["step", "min_step", "max_fails", "max_trials", "max_step_reductions"]

    if isinstance(root.get("parameters"), dict):
        if not cli_param:
            available = ", ".join(sorted(root["parameters"].keys()))
            raise RuntimeError(
                "Active parameter is not specified. Use -param <name>.\n"
                f"Available parameters in JSON: {available}"
            )
        param = str(cli_param)
        if param not in root["parameters"]:
            available = ", ".join(sorted(root["parameters"].keys()))
            raise RuntimeError(
                f"Parameter '{param}' is not defined in JSON parameters.\n"
                f"Available parameters: {available}"
            )
        data = dict(root.get("defaults", {}))
        data.update(root["parameters"][param])
        # Global objective defaults can be defined once at root level.
        for k in [
            "objective", "delta_xy_weight", "delta_xyz_weight", "distance_xy_weight",
            "expected_distance_xy", "step_reduce_factor", "improvement_epsilon",
        ]:
            if k in root and k not in data:
                data[k] = root[k]
    else:
        # Backward compatibility with old flat test.json.
        param = str(cli_param or root.get("param") or "")
        if not param:
            raise RuntimeError("Parameter name is not specified. Use -param td or JSON field: param")
        data = root

    missing = [k for k in required if k not in data]
    if missing:
        raise RuntimeError(f"Missing required JSON fields for parameter '{param}': {', '.join(missing)}")

    settings = SearchSettings(
        param=str(param),
        step=float(data["step"]),
        min_step=float(data["min_step"]),
        max_fails=int(data["max_fails"]),
        max_trials=int(data["max_trials"]),
        max_step_reductions=int(data["max_step_reductions"]),
        step_reduce_factor=float(data.get("step_reduce_factor", 2.0)),
        improvement_epsilon=float(data.get("improvement_epsilon", 1e-9)),
        objective=str(data.get("objective", "composite")),
        delta_xy_weight=float(data.get("delta_xy_weight", 1.0)),
        delta_xyz_weight=float(data.get("delta_xyz_weight", 1.0)),
        distance_xy_weight=float(data.get("distance_xy_weight", 0.0)),
        expected_distance_xy=(None if data.get("expected_distance_xy", None) is None else float(data.get("expected_distance_xy"))),
    )
    if settings.step <= 0:
        raise RuntimeError(f"JSON field step for '{param}' must be > 0")
    if settings.min_step <= 0:
        raise RuntimeError(f"JSON field min_step for '{param}' must be > 0")
    if settings.step < settings.min_step:
        raise RuntimeError(f"JSON field step for '{param}' must be >= min_step")
    if settings.max_trials < 1:
        raise RuntimeError(f"JSON field max_trials for '{param}' must be >= 1")
    if settings.max_fails < 0:
        raise RuntimeError(f"JSON field max_fails for '{param}' must be >= 0")
    if settings.max_step_reductions < 0:
        raise RuntimeError(f"JSON field max_step_reductions for '{param}' must be >= 0")
    if settings.step_reduce_factor <= 1.0:
        raise RuntimeError(f"JSON field step_reduce_factor for '{param}' must be > 1.0")
    if settings.objective not in ("delta_xy", "delta_xyz", "composite"):
        raise RuntimeError("JSON field objective must be one of: delta_xy, delta_xyz, composite")
    if settings.delta_xy_weight < 0 or settings.delta_xyz_weight < 0 or settings.distance_xy_weight < 0:
        raise RuntimeError("JSON objective weights must be >= 0")
    return settings


def load_test_settings(json_path: str) -> TestSettings:
    path = expand_path(json_path)
    if not os.path.exists(path):
        return TestSettings()
    try:
        with open(path, "r", encoding="utf-8") as f:
            root = json.load(f)
    except Exception:
        return TestSettings()

    data = root.get("test", {})
    if not isinstance(data, dict):
        data = {}

    ts = TestSettings(
        repeats=int(data.get("repeats", 1)),
        aggregation=str(data.get("aggregation", "median")).lower(),
    )
    if ts.repeats < 1:
        raise RuntimeError("JSON test.repeats must be >= 1")
    if ts.aggregation not in ("median", "mean"):
        raise RuntimeError("JSON test.aggregation must be one of: median, mean")
    return ts


def load_fixpos_settings(json_path: str) -> FixposSettings:
    path = expand_path(json_path)
    if not os.path.exists(path):
        return FixposSettings()
    try:
        with open(path, "r", encoding="utf-8") as f:
            root = json.load(f)
    except Exception:
        return FixposSettings()

    data = root.get("fixpos", {})
    if not isinstance(data, dict):
        data = {}

    fs = FixposSettings(
        enabled=bool(data.get("enabled", True)),
        time=float(data.get("time", 5.0)),
        eps=float(data.get("eps", 0.02)),
        max_wait=float(data.get("max_wait", 30.0)),
    )
    if fs.time <= 0:
        raise RuntimeError("JSON fixpos.time must be > 0")
    if fs.eps <= 0:
        raise RuntimeError("JSON fixpos.eps must be > 0")
    if fs.max_wait < fs.time:
        raise RuntimeError("JSON fixpos.max_wait must be >= fixpos.time")
    return fs

def ros_shell_cmd(args, cmd: str) -> str:
    """Run a ROS command after sourcing the ROS underlay and optional VINS workspace."""
    setups = [expand_path(args.ros_setup)]
    if getattr(args, "vins_setup", None):
        setups.append(expand_path(args.vins_setup))
    prefix = " && ".join(f"source {shlex.quote(x)}" for x in setups)
    return f"{prefix} && {cmd}"


def read_file(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def write_file(path: str, text: str):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(text)
    os.replace(tmp, path)


def find_yaml_param(text: str, param: str) -> float:
    # Matches lines like: td: -0.026823    # comment
    pat = re.compile(r"^(?P<indent>\s*)" + re.escape(param) + r"\s*:\s*(?P<value>[-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)", re.MULTILINE)
    m = pat.search(text)
    if not m:
        raise RuntimeError(f"Parameter '{param}' was not found in YAML.")
    return float(m.group("value"))




def find_yaml_string_param(text: str, names: List[str], default: Optional[str] = None) -> Optional[str]:
    for name in names:
        pat = re.compile(
            r"^(?P<indent>\s*)" + re.escape(name) +
            r"\s*:\s*[\"']?(?P<value>[^\"'\s#]+)[\"']?",
            re.MULTILINE,
        )
        m = pat.search(text)
        if m:
            return m.group("value")
    return default


def run_command_capture(cmd: str, timeout: float = 20.0) -> Tuple[int, str]:
    try:
        proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            out, _ = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()
            return 124, out
        return proc.returncode, out or ""
    except Exception as e:
        return 1, str(e)


def print_ros_command_check(args):
    print()
    print("========== ROS COMMAND CHECK ==========")
    setup = expand_path(args.ros_setup)
    print(f"[DBG] ROS setup: {setup}")
    if getattr(args, "vins_setup", None):
        print(f"[DBG] VINS setup: {expand_path(args.vins_setup)}")
    if not os.path.exists(setup):
        print(f"[WARN] ROS setup file not found: {setup}")
        print("[WARN] Use --ros-setup /path/to/setup.bash")
        return
    for cmd_name in (("roslaunch", "rosbag", "rostopic") if ROS_VERSION == 1 else ("ros2",)):
        cmd = ros_shell_cmd(args, f"command -v {cmd_name}")
        code, out = run_command_capture(cmd, timeout=5.0)
        if code == 0 and out.strip():
            print(f"[DBG] {cmd_name}: {out.strip()}")
        else:
            print(f"[WARN] {cmd_name} not found after sourcing ROS setup")


def print_bag_info(args):
    print()
    print("========== BAG CHECK ==========")
    cmd_raw = (f"rosbag info {shlex.quote(args.bag)}" if ROS_VERSION == 1 else f"ros2 bag info {shlex.quote(args.bag)}")
    cmd = ros_shell_cmd(args, cmd_raw)
    print(f"[DBG] CMD: {cmd_raw}")
    print(f"[DBG] ENV: source {expand_path(args.ros_setup)}")
    code, out = run_command_capture(cmd, timeout=args.bag_info_timeout)
    if code != 0:
        print(f"[WARN] rosbag info failed, code={code}")
        if out.strip():
            print(out.strip())
        return

    lines = out.splitlines()
    topic_lines = [ln for ln in lines if args.cam_topic in ln or args.imu_topic in ln or args.odom_topic in ln]
    if topic_lines:
        print("[DBG] Relevant topics in bag/info:")
        for ln in topic_lines:
            print("[DBG] " + ln)
    else:
        print("[WARN] cam/imu topic names were not found in rosbag info output.")
        print(f"[WARN] Expected cam={args.cam_topic}, imu={args.imu_topic}")


def wait_for_topic_activity(args, start_cam: int, start_imu: int, timeout: float) -> Tuple[bool, str]:
    print()
    print("[DBG] Checking primary input topics from rosbag...")
    print(f"[DBG] Camera topic: {args.cam_topic}")
    print(f"[DBG] IMU topic:    {args.imu_topic}")
    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < timeout and not ros_is_shutdown() and not stop_requested:
        if check_stop_file(args):
            return False, "stopped by user"
        cam_count = cam_seq - start_cam
        imu_count = imu_seq - start_imu
        if time.time() - last_print >= 1.0:
            print(f"[DBG] topic check {time.time() - t0:.1f}/{timeout:.1f}s | cam={cam_count} imu={imu_count}")
            last_print = time.time()
        if cam_count > 0 and imu_count > 0:
            print(f"[DBG] Input topics OK: cam={cam_count}, imu={imu_count}")
            return True, "OK"
        time.sleep(0.05)

    cam_count = cam_seq - start_cam
    imu_count = imu_seq - start_imu
    missing = []
    if cam_count <= 0:
        missing.append(f"camera topic {args.cam_topic}")
    if imu_count <= 0:
        missing.append(f"IMU topic {args.imu_topic}")
    return False, "no data on " + " and ".join(missing) + f" within {timeout:.1f}s"


def set_yaml_param(text: str, param: str, value: float) -> str:
    pat = re.compile(
        r"^(?P<indent>\s*)"
        + re.escape(param)
        + r"(?P<sp1>\s*:\s*)"
        + r"(?P<value>[-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)"
        + r"(?P<tail>\s*(?:#.*)?$)",
        re.MULTILINE,
    )

    def repl(m):
        return f"{m.group('indent')}{param}{m.group('sp1')}{value:.9f}{m.group('tail')}"

    new_text, n = pat.subn(repl, text, count=1)
    if n != 1:
        raise RuntimeError(f"Parameter '{param}' was not replaced in YAML.")
    return new_text


def should_print_process_line(args, name: str, line: str) -> bool:
    """Filter noisy ROS output. By default keep only useful warnings/errors.

    VINS-Mono often prints lines like `position: x, y, z` at high rate through
    roslaunch. Those lines are intentionally hidden here because the tuner
    already prints its own live position summary.
    """
    if not line:
        return False

    if name == "roslaunch":
        mode = getattr(args, "roslaunch_log_mode", "warn")
        if mode == "all":
            return True
        if mode == "quiet":
            return False

        low = line.lower()
        # Hide VINS high-rate position spam even if the process terminates and
        # the partial line contains words such as "terminate".
        if low.lstrip().startswith("position:") or "[roslaunch] position:" in low:
            return False
        important_markers = (
            "[warn]", "[warning]", "[error]", "[fatal]",
            " warn", " error", " fatal", "exception", "traceback",
            "failed", "cannot", "not found", "segmentation", "core dumped",
        )
        return any(m in low for m in important_markers)

    if name == "rosbag":
        # rosbag output is usually hidden by default; if enabled, still suppress
        # common progress spam unless user chooses --roslaunch-log-mode all.
        low = line.lower()
        return any(m in low for m in ("error", "fatal", "failed", "exception", "traceback"))

    return True


def _process_output_reader(proc: subprocess.Popen, name: str, args=None):
    if proc.stdout is None:
        return
    try:
        for raw in iter(proc.stdout.readline, ""):
            if not raw:
                break
            line = raw.rstrip("\n")
            if args is None or should_print_process_line(args, name, line):
                print(f"[{name}] {line}")
    except Exception as e:
        print(f"[DBG] output reader for {name} stopped: {e}")


def start_process(args, cmd: str, name: str, cwd: Optional[str] = None, show_output: bool = True) -> subprocess.Popen:
    print()
    print(f"[DBG] Starting {name}...")
    print(f"[DBG] CMD: {cmd}")
    print(f"[DBG] ENV: source {expand_path(args.ros_setup)}")
    if cwd:
        print(f"[DBG] CWD: {cwd}")

    full_cmd = ros_shell_cmd(args, cmd)
    proc = subprocess.Popen(
        ["bash", "-lc", full_cmd],
        cwd=cwd,
        stdout=subprocess.PIPE if show_output else subprocess.DEVNULL,
        stderr=subprocess.STDOUT if show_output else subprocess.DEVNULL,
        text=True,
        bufsize=1,
        preexec_fn=os.setsid,
    )
    print(f"[DBG] {name} PID: {proc.pid}")
    active_processes.append((proc, name))

    if show_output:
        th = threading.Thread(target=_process_output_reader, args=(proc, name, args), daemon=True)
        th.start()

    return proc


def stop_process(proc: Optional[subprocess.Popen], name: str, timeout: float = 5.0):
    if proc is None:
        return
    try:
        active_processes[:] = [(p, n) for (p, n) in active_processes if p.pid != proc.pid]
    except Exception:
        pass
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if proc.poll() is not None:
                return
            time.sleep(0.1)
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        time.sleep(0.5)
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
    except Exception as e:
        print(f"WARN: failed to stop {name}: {e}")


def stop_all_processes():
    for proc, name in list(active_processes):
        stop_process(proc, name, timeout=2.0)


def stable_info(samples, need_time: float, eps: float):
    if len(samples) < 2:
        return False, 0.0, None

    t_first = samples[0][0]
    t_last = samples[-1][0]
    span = t_last - t_first

    xs = [p[0] for _, p in samples]
    ys = [p[1] for _, p in samples]
    zs = [p[2] for _, p in samples]

    rx = max(xs) - min(xs)
    ry = max(ys) - min(ys)
    rz = max(zs) - min(zs)

    avg_pos = (sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))
    stable = span >= need_time and rx <= eps and ry <= eps and rz <= eps
    return stable, span, (avg_pos, rx, ry, rz, len(samples))


def wait_for_fixpos(args, trial_index: int, value: float, start_seq: int):
    if not getattr(args, "fixpos_enabled", True):
        if last_odom is None:
            return False, None, odom_seq, "no odometry for start position"
        return True, get_pos(last_odom), odom_seq, "FIXPOS disabled; using first odometry position"

    from collections import deque
    samples = deque()
    last_seen = start_seq
    t0 = time.time()
    last_print = 0.0
    print()
    print(f"[FIXPOS] Waiting stable start: time={args.fixpos_time:.1f}s eps={args.fixpos_eps:.4f}m max_wait={args.fixpos_max_wait:.1f}s")

    while time.time() - t0 <= args.fixpos_max_wait and not ros_is_shutdown() and not stop_requested:
        if check_stop_file(args):
            return False, None, odom_seq, "stopped by user"

        now = time.time()
        if odom_seq != last_seen and last_odom is not None:
            samples.append((now, get_pos(last_odom)))
            last_seen = odom_seq

        while samples and (now - samples[0][0]) > (args.fixpos_time + 0.25):
            samples.popleft()

        stable, span, info = stable_info(samples, args.fixpos_time, args.fixpos_eps)

        if now - last_print >= 1.0:
            if info is None:
                print(f"[FIXPOS] trial #{trial_index} {args.param}={value:.9f} | collecting... {span:.1f}/{args.fixpos_time:.1f}s samples={len(samples)}")
            else:
                avg_pos, rx, ry, rz, n = info
                print(
                    f"[FIXPOS] trial #{trial_index} {args.param}={value:.9f} | "
                    f"time={span:.1f}/{args.fixpos_time:.1f}s samples={n} | "
                    f"range dx={rx:.4f} dy={ry:.4f} dz={rz:.4f} eps={args.fixpos_eps:.4f} | "
                    f"avg {format_pos(avg_pos)}"
                )
            last_print = now

        if stable:
            avg_pos, rx, ry, rz, n = info
            print(
                f"[FIXPOS] LOCKED: {format_pos(avg_pos)} | "
                f"samples={n} range dx={rx:.4f} dy={ry:.4f} dz={rz:.4f}"
            )
            return True, avg_pos, odom_seq, "FIXPOS locked"

        time.sleep(0.05)

    return False, None, odom_seq, f"FIXPOS not stable within {args.fixpos_max_wait:.1f}s"


def wait_for_first_odom(timeout: float) -> bool:
    t0 = time.time()
    start_seq = odom_seq
    while time.time() - t0 < timeout and not ros_is_shutdown() and not stop_requested:
        if odom_seq > start_seq and last_odom is not None:
            return True
        time.sleep(0.05)
    return False


def run_trial(args, trial_index: int, value: float, original_text: str, repeat_index: int = 1, repeats: int = 1) -> TrialResult:
    global odom_seq, cam_seq, imu_seq

    config_text = set_yaml_param(original_text, args.param, value)
    write_file(args.config, config_text)

    launch_proc = None
    bag_proc = None

    start_seq = odom_seq
    start_cam_seq = cam_seq
    start_imu_seq = imu_seq
    start_pos = None
    end_pos = None
    start_time = time.time()
    end_time = start_time

    print()
    title = f"========== TRIAL #{trial_index}" + (f" RUN {repeat_index}/{repeats}" if repeats > 1 else "") + " =========="
    print(title)
    print(f"{args.param}: {value:.9f}")

    try:
        launch_proc = start_process(args, args.launch_cmd, "roslaunch", show_output=args.show_roslaunch_output)
        time.sleep(args.launch_wait)
        if launch_proc.poll() is not None:
            reason = f"roslaunch exited before rosbag start, code={launch_proc.returncode}"
            stop_process(launch_proc, "vins")
            return TrialResult(trial_index, value, None, None, time.time() - start_time, 0, None, None, False, reason)

        quoted_bag = shlex.quote(args.bag)
        if args.clock:
            bag_cmd = (f"rosbag play --clock {quoted_bag}" if ROS_VERSION == 1 else f"ros2 bag play {quoted_bag} --clock")
        else:
            bag_cmd = (f"rosbag play {quoted_bag}" if ROS_VERSION == 1 else f"ros2 bag play {quoted_bag}")
        if args.bag_rate != 1.0:
            bag_cmd += f" -r {args.bag_rate}"

        bag_proc = start_process(args, bag_cmd, "rosbag", cwd=args.bag_cwd, show_output=args.show_rosbag_output)
        time.sleep(0.5)
        if bag_proc.poll() is not None:
            reason = f"rosbag exited immediately, code={bag_proc.returncode}"
            stop_process(launch_proc, "vins")
            return TrialResult(trial_index, value, None, None, time.time() - start_time, 0, None, None, False, reason)

        topics_ok, topics_reason = wait_for_topic_activity(args, start_cam_seq, start_imu_seq, args.topic_wait)
        if not topics_ok:
            reason = topics_reason
            stop_process(bag_proc, "rosbag")
            stop_process(launch_proc, "vins")
            return TrialResult(trial_index, value, None, None, time.time() - start_time, 0, None, None, False, reason)

        print()
        print(f"[DBG] Waiting for odometry topic: {args.odom_topic}")
        if not wait_for_first_odom(args.odom_wait):
            reason = f"no odometry on {args.odom_topic} within {args.odom_wait:.1f}s"
            stop_process(bag_proc, "rosbag")
            stop_process(launch_proc, "vins")
            return TrialResult(trial_index, value, None, None, time.time() - start_time, 0, None, None, False, reason)

        # Lock a stable start position before the real scoring starts.
        # With FIXPOS enabled, start_pos is an average over a stable odometry window.
        # Without FIXPOS, it falls back to the first odometry sample.
        fixed_ok, fixed_start_pos, first_seq, fixed_reason = wait_for_fixpos(args, trial_index, value, odom_seq)
        if not fixed_ok:
            reason = fixed_reason
            stop_process(bag_proc, "rosbag")
            stop_process(launch_proc, "vins")
            return TrialResult(trial_index, value, None, None, time.time() - start_time, 0, None, None, False, reason)

        start_pos = fixed_start_pos
        end_pos = start_pos
        # Start duration and path accumulation only after FIXPOS lock.
        start_time = time.time()
        end_time = start_time
        last_path_pos = start_pos
        path_distance_xy = 0.0
        path_distance_xyz = 0.0
        last_seen_seq = first_seq
        last_live_print = 0.0
        live_line_used = False

        while not ros_is_shutdown() and not stop_requested:
            if check_stop_file(args):
                reason = "stopped by user"
                stop_process(bag_proc, "rosbag")
                stop_process(launch_proc, "vins")
                if live_line_used:
                    finish_live_line()
                return TrialResult(trial_index, value, None, None, time.time() - start_time, max(0, odom_seq - first_seq + 1), start_pos, end_pos, False, reason, path_distance_xy, path_distance_xyz)
            if bag_proc.poll() is not None:
                break
            if time.time() - start_time > args.max_trial_time:
                reason = f"max trial time exceeded: {args.max_trial_time:.1f}s"
                stop_process(bag_proc, "rosbag")
                stop_process(launch_proc, "vins")
                if last_odom is not None:
                    end_pos = get_pos(last_odom)
                    end_time = time.time()
                if 'live_line_used' in locals() and live_line_used:
                    finish_live_line()
                msg_count = max(0, odom_seq - first_seq + 1)
                score = dist_xy(start_pos, end_pos) if start_pos and end_pos else None
                dz = dist_xyz(start_pos, end_pos) if start_pos and end_pos else None
                return TrialResult(trial_index, value, score, dz, end_time - start_time, msg_count, start_pos, end_pos, False, reason, path_distance_xy, path_distance_xyz)

            if odom_seq != last_seen_seq and last_odom is not None:
                new_pos = get_pos(last_odom)
                if last_path_pos is not None:
                    path_distance_xy += dist_xy(last_path_pos, new_pos)
                    path_distance_xyz += dist_xyz(last_path_pos, new_pos)
                last_path_pos = new_pos
                end_pos = new_pos
                end_time = time.time()
                last_seen_seq = odom_seq

            if time.time() - last_live_print >= args.pos_print_period:
                msg_count_live = max(0, odom_seq - first_seq + 1)
                print_live_position(trial_index, value, msg_count_live, start_pos, end_pos, end_time, start_time)
                live_line_used = True
                last_live_print = time.time()

            time.sleep(0.05)

        if live_line_used:
            finish_live_line()

        # Give VINS a small window to publish last messages after rosbag EOF.
        time.sleep(args.after_bag_wait)
        if last_odom is not None:
            end_pos = get_pos(last_odom)
            end_time = time.time()

        msg_count = max(0, odom_seq - first_seq + 1)
        if start_pos is None or end_pos is None or msg_count <= 0:
            reason = "no complete odometry session was recorded"
            score = None
            dz = None
            ok = False
        else:
            score = dist_xy(start_pos, end_pos)
            dz = dist_xyz(start_pos, end_pos)
            reason = "OK"
            ok = True

        return TrialResult(trial_index, value, score, dz, end_time - start_time, msg_count, start_pos, end_pos, ok, reason, path_distance_xy, path_distance_xyz)

    finally:
        stop_process(bag_proc, "rosbag")
        stop_process(launch_proc, "vins")
        time.sleep(args.between_trials)


def build_candidates(center: float, radius: float, step: float) -> List[float]:
    if step <= 0:
        raise ValueError("step must be > 0")
    count = int(round((2.0 * radius) / step))
    values = []
    for i in range(count + 1):
        v = center - radius + i * step
        values.append(round(v, 9))
    if round(center, 9) not in values:
        values.append(round(center, 9))
    return sorted(set(values))


def order_candidates_start_from_center(candidates: List[float], center: float) -> List[float]:
    center_key = round(center, 9)
    unique = sorted(set(round(v, 9) for v in candidates))
    if center_key not in unique:
        unique.insert(0, center_key)

    # First exactly the current YAML value, then nearest values around it.
    rest = [v for v in unique if v != center_key]
    rest.sort(key=lambda v: (abs(v - center_key), v))
    return [center_key] + rest


def calculate_objective_score(r: TrialResult, settings: SearchSettings) -> Optional[float]:
    if not r.ok or r.score_delta_xy is None or r.delta_xyz is None:
        return None
    if settings.objective == "delta_xy":
        return r.score_delta_xy
    if settings.objective == "delta_xyz":
        return r.delta_xyz

    # Composite score: lower is better.
    # Default: DeltaXY + DeltaXYZ, so a trial with small XY error but huge Z/3D drift
    # no longer wins automatically. Optionally add a path-length penalty if you know
    # the expected path length for this bag.
    score = settings.delta_xy_weight * r.score_delta_xy + settings.delta_xyz_weight * r.delta_xyz
    if (settings.expected_distance_xy is not None and r.distance_xy is not None and
            settings.distance_xy_weight > 0.0):
        score += settings.distance_xy_weight * abs(r.distance_xy - settings.expected_distance_xy)
    return score


def result_score(r: Optional[TrialResult]) -> Optional[float]:
    if r is None:
        return None
    return r.objective_score if r.objective_score is not None else r.score_delta_xy


def describe_improvement(score: float, baseline_score: Optional[float], best_score_before: Optional[float]) -> str:
    parts = []
    if baseline_score is not None:
        diff = baseline_score - score
        pct = (diff / baseline_score * 100.0) if baseline_score > 0 else 0.0
        word = "improved" if diff > 0 else "worse" if diff < 0 else "same"
        parts.append(f"vs first: {word} {abs(diff):.6f} m ({abs(pct):.2f}%)")
    if best_score_before is not None:
        diff = best_score_before - score
        pct = (diff / best_score_before * 100.0) if best_score_before > 0 else 0.0
        word = "improved" if diff > 0 else "worse" if diff < 0 else "same"
        parts.append(f"vs best: {word} {abs(diff):.6f} m ({abs(pct):.2f}%)")
    return " | ".join(parts)


def verdict_text(score: Optional[float], baseline_score: Optional[float], best_score_before: Optional[float], eps: float) -> str:
    if score is None:
        return "FAIL"
    if baseline_score is None:
        return "BASELINE"
    if best_score_before is not None and score < best_score_before - eps:
        return "NEW BEST"
    if score < baseline_score - eps:
        return "BETTER THAN FIRST"
    if abs(score - baseline_score) <= eps:
        return "SAME AS FIRST"
    return "WORSE"


def _aggregate(values: List[float], method: str) -> float:
    if method == "mean":
        return sum(values) / len(values)
    return float(statistics.median(values))


def _stddev(values: List[float]) -> Optional[float]:
    if len(values) < 2:
        return 0.0 if len(values) == 1 else None
    return float(statistics.stdev(values))


def aggregate_trial_results(index: int, value: float, runs: List[TrialResult], method: str) -> TrialResult:
    valid = [r for r in runs if r.ok and r.objective_score is not None]
    if not valid:
        reason = "all repeats failed: " + " | ".join(r.reason for r in runs)
        return TrialResult(
            index=index,
            value=value,
            score_delta_xy=None,
            delta_xyz=None,
            duration=sum(r.duration for r in runs),
            odom_messages=sum(r.odom_messages for r in runs),
            start_pos=None,
            end_pos=None,
            ok=False,
            reason=reason,
            repeat_count=len(runs),
            valid_repeats=0,
        )

    obj_values = [float(r.objective_score) for r in valid]
    xy_values = [float(r.score_delta_xy) for r in valid if r.score_delta_xy is not None]
    xyz_values = [float(r.delta_xyz) for r in valid if r.delta_xyz is not None]
    dist_xy_values = [float(r.distance_xy) for r in valid if r.distance_xy is not None]
    dist_xyz_values = [float(r.distance_xyz) for r in valid if r.distance_xyz is not None]

    # Pick representative start/end from the run nearest to the aggregated objective.
    agg_obj = _aggregate(obj_values, method)
    representative = min(valid, key=lambda r: abs(float(r.objective_score) - agg_obj))

    agg = TrialResult(
        index=index,
        value=value,
        score_delta_xy=_aggregate(xy_values, method) if xy_values else None,
        delta_xyz=_aggregate(xyz_values, method) if xyz_values else None,
        duration=_aggregate([r.duration for r in valid], method),
        odom_messages=int(round(_aggregate([float(r.odom_messages) for r in valid], method))),
        start_pos=representative.start_pos,
        end_pos=representative.end_pos,
        ok=True,
        reason=f"OK aggregated {len(valid)}/{len(runs)} repeats by {method}",
        distance_xy=_aggregate(dist_xy_values, method) if dist_xy_values else None,
        distance_xyz=_aggregate(dist_xyz_values, method) if dist_xyz_values else None,
        objective_score=agg_obj,
        repeat_count=len(runs),
        valid_repeats=len(valid),
        score_stddev=_stddev(obj_values),
        delta_xy_stddev=_stddev(xy_values),
        delta_xyz_stddev=_stddev(xyz_values),
    )
    return agg


def format_change_line(score: float, baseline_score: Optional[float], best_score_before: Optional[float]) -> str:
    info = describe_improvement(score, baseline_score, best_score_before)
    return info if info else "first valid result, used as baseline"


def print_trial_result(
    r: TrialResult,
    args,
    label: str,
    step: float,
    direction: Optional[int],
    trial_limit: int,
    step_reductions: int,
    max_step_reductions: int,
    fails_at_current_step: int,
    max_fails: int,
    baseline_score: Optional[float],
    best_score_before: Optional[float],
    eps: float,
):
    dir_txt = "none" if direction is None else ("+" if direction > 0 else "-")
    print()
    print("----- TRIAL RESULT -----")
    print(f"Trial:      #{r.index}/{trial_limit}")
    print(f"Search step:{label}")
    print(f"Parameter:  {args.param} = {r.value:.9f}")
    if getattr(r, "repeat_count", 1) > 1:
        print(f"Repeats:    valid {r.valid_repeats}/{r.repeat_count}, aggregation={getattr(args, 'aggregation', 'median')}")
    print(f"Step size:  {step:.9f}")
    print(f"Direction:  {dir_txt}")
    print(f"Limits:     fails {fails_at_current_step}/{max_fails}, step reductions {step_reductions}/{max_step_reductions}")

    if not r.ok:
        print(f"Verdict:    FAIL")
        print(f"Reason:     {r.reason}")
        print("------------------------")
        return

    verdict = verdict_text(r.objective_score, baseline_score, best_score_before, eps)
    print(f"Verdict:    {verdict}")
    print(f"Objective:  {r.objective_score:.6f}  ({getattr(args, 'objective_name', 'score')}, lower is better)")
    if r.score_stddev is not None and getattr(r, "repeat_count", 1) > 1:
        print(f"StdDev:     score={r.score_stddev:.6f}  DeltaXY={r.delta_xy_stddev:.6f}  DeltaXYZ={r.delta_xyz_stddev:.6f}")
    print(f"Delta XY:   {r.score_delta_xy:.6f} m")
    print(f"Delta XYZ:  {r.delta_xyz:.6f} m")
    if r.distance_xy is not None and r.distance_xyz is not None:
        print(f"Distance:   XY={r.distance_xy:.6f} m  XYZ={r.distance_xyz:.6f} m")
    print(f"Messages:   {r.odom_messages}")
    print(f"Duration:   {r.duration:.3f} s")
    print(f"Start:      x={r.start_pos[0]: .6f} y={r.start_pos[1]: .6f} z={r.start_pos[2]: .6f}")
    print(f"End:        x={r.end_pos[0]: .6f} y={r.end_pos[1]: .6f} z={r.end_pos[2]: .6f}")
    print(f"Change:     {format_change_line(r.objective_score, baseline_score, best_score_before)}")
    print("------------------------")


def save_csv(path: str, results: List[TrialResult]):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "trial", "value", "ok", "objective_score", "score_stddev", "valid_repeats", "repeat_count",
            "delta_xy_m", "delta_xy_stddev", "delta_xyz_m", "delta_xyz_stddev", "distance_xy_m", "distance_xyz_m",
            "duration_s", "odom_messages", "reason",
            "start_x", "start_y", "start_z", "end_x", "end_y", "end_z",
        ])
        for r in results:
            sx, sy, sz = r.start_pos if r.start_pos else ("", "", "")
            ex, ey, ez = r.end_pos if r.end_pos else ("", "", "")
            w.writerow([
                r.index, f"{r.value:.9f}", int(r.ok),
                "" if r.objective_score is None else f"{r.objective_score:.9f}",
                "" if r.score_stddev is None else f"{r.score_stddev:.9f}",
                r.valid_repeats, r.repeat_count,
                "" if r.score_delta_xy is None else f"{r.score_delta_xy:.9f}",
                "" if r.delta_xy_stddev is None else f"{r.delta_xy_stddev:.9f}",
                "" if r.delta_xyz is None else f"{r.delta_xyz:.9f}",
                "" if r.delta_xyz_stddev is None else f"{r.delta_xyz_stddev:.9f}",
                "" if r.distance_xy is None else f"{r.distance_xy:.9f}",
                "" if r.distance_xyz is None else f"{r.distance_xyz:.9f}",
                f"{r.duration:.3f}", r.odom_messages, r.reason,
                sx, sy, sz, ex, ey, ez,
            ])



def print_standalone_report(test_id, reason, start_pos, end_pos, start_time, end_time, count, path_xy, path_xyz):
    print()
    print(f"========== VINS TEST #{test_id} ==========")
    print(f"Finish reason: {reason}")
    if start_pos is None or end_pos is None:
        print("No complete VINS odometry session was recorded.")
        print("====================================")
        return
    dx, dy, dz = end_pos[0]-start_pos[0], end_pos[1]-start_pos[1], end_pos[2]-start_pos[2]
    duration = max(0.0, (end_time or time.time()) - (start_time or time.time()))
    print(f"Odometry messages: {count}")
    print(f"Duration: {duration:.3f} s")
    print(f"Start position:  {format_pos(start_pos)}")
    print(f"End position:    {format_pos(end_pos)}")
    print(f"Delta position:  dx={dx: .6f}  dy={dy: .6f}  dz={dz: .6f}")
    print(f"Delta XY:        {math.hypot(dx, dy):.6f} m")
    print(f"Delta XYZ:       {math.sqrt(dx*dx+dy*dy+dz*dz):.6f} m")
    print(f"Distance XY:     {path_xy:.6f} m")
    print(f"Distance XYZ:    {path_xyz:.6f} m")
    print("====================================")


def run_standalone_test(args):
    global stop_requested
    print(f"VINS standalone test mode | ROS{ROS_VERSION}")
    print(f"Odometry: {args.odom_topic}")
    print(f"IMU:      {args.imu_topic}")
    if args.fixpos_enabled:
        print(f"FIXPOS:   {args.fixpos_time:.1f}s, eps={args.fixpos_eps:.4f}m")
    print("Stop VINS, IMU, or press Ctrl+C to finish the active session.")

    state = "WAITING"
    test_id = 0
    start_pos = end_pos = None
    start_time = end_time = None
    count = 0
    last_seq = odom_seq
    path_xy = path_xyz = 0.0
    last_path = None
    samples = deque()
    last_fix_seq = -1

    while not ros_is_shutdown() and not stop_requested:
        now = time.time()
        imu_alive = last_imu_rx > 0 and now-last_imu_rx <= args.imu_timeout
        odom_alive = last_odom is not None and now-last_odom_rx <= args.odom_timeout
        pos = get_pos(last_odom) if last_odom is not None else None

        if not imu_alive or not odom_alive:
            reason = (f"IMU data stopped: no fresh data on {args.imu_topic}" if not imu_alive
                      else f"VINS odometry stopped: no fresh data on {args.odom_topic}")
            clear_current_line()
            print(f"[{time.strftime('%H:%M:%S')}] [WARN] {reason} | last {format_pos(pos)}", end="", flush=True)
            if state in ("FIXING", "RECORDING"):
                print()
                test_id += 1
                print_standalone_report(test_id, reason, start_pos, end_pos, start_time, end_time, count, path_xy, path_xyz)
                state = "WAITING"; start_pos = end_pos = None; start_time = end_time = None
                count = 0; last_seq = odom_seq; path_xy = path_xyz = 0.0; last_path = None
                samples.clear(); last_fix_seq = -1
            time.sleep(0.2)
            continue

        if state == "WAITING":
            if args.fixpos_enabled:
                state = "FIXING"; samples.clear(); last_fix_seq = -1
            else:
                state = "RECORDING"; start_pos = end_pos = pos; start_time = end_time = now
                count = 1; last_seq = odom_seq; last_path = pos

        if state == "FIXING":
            if odom_seq != last_fix_seq and pos is not None:
                samples.append((now, pos)); last_fix_seq = odom_seq
            while samples and now-samples[0][0] > args.fixpos_time + 0.25:
                samples.popleft()
            stable, span, info = stable_info(samples, args.fixpos_time, args.fixpos_eps)
            clear_current_line()
            if info:
                avg, rx, ry, rz, n = info
                print(f"[{time.strftime('%H:%M:%S')}] FIXPOS SEARCH {span:.1f}/{args.fixpos_time:.1f}s range={rx:.4f},{ry:.4f},{rz:.4f} n={n}", end="", flush=True)
            else:
                print(f"[{time.strftime('%H:%M:%S')}] FIXPOS SEARCH {span:.1f}/{args.fixpos_time:.1f}s", end="", flush=True)
            if stable:
                avg, rx, ry, rz, n = info
                print()
                print(f"FIXPOS locked: {format_pos(avg)} | range dx={rx:.4f} dy={ry:.4f} dz={rz:.4f}")
                start_pos = end_pos = avg; start_time = end_time = now; count = 0
                last_seq = odom_seq; last_path = avg; state = "RECORDING"
            time.sleep(0.05)
            continue

        if state == "RECORDING":
            if odom_seq != last_seq and pos is not None:
                path_xy += dist_xy(last_path, pos); path_xyz += dist_xyz(last_path, pos)
                last_path = end_pos = pos; end_time = now
                count += max(1, odom_seq-last_seq); last_seq = odom_seq
            clear_current_line()
            print(f"[{time.strftime('%H:%M:%S')}] VINS POS {format_pos(pos)} | count={count} | DistanceXY={path_xy:.3f}m", end="", flush=True)
        time.sleep(0.2)
    print()


def get_branch_name():
    try:
        branch = subprocess.run(
            ["git", "branch", "--show-current"],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            capture_output=True,
            text=True,
            timeout=2,
            check=False,
        ).stdout.strip()
        if branch:
            return branch
    except (OSError, subprocess.SubprocessError):
        pass
    return os.environ.get("GIT_BRANCH", "unknown")


def version_text():
    return (
        f"{APP_NAME}\n"
        f"Version: {VERSION}\n"
        f"Branch : {get_branch_name()}\n"
        f"ROS    : ROS{ROS_VERSION}"
    )


class VersionAction(argparse.Action):
    def __init__(self, option_strings, dest=argparse.SUPPRESS, default=argparse.SUPPRESS, **kwargs):
        super().__init__(option_strings, dest, nargs=0, default=default, **kwargs)

    def __call__(self, parser, namespace, values, option_string=None):
        parser.exit(message=version_text() + "\n")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--version", action=VersionAction, help="show version information and exit")
    p.add_argument("-param", required=False, help="YAML parameter to tune now, for example: -param td or -param acc_n. Search settings are loaded from test.json")
    p.add_argument("-test", "--test", action="store_true", help="standalone VINS session test mode; does not launch VINS or play a bag")
    p.add_argument("--ros1", action="store_true", help="use ROS1; ROS2 is the default")
    p.add_argument("--ros2", action="store_true", help="explicitly use ROS2 (default)")
    p.add_argument("--odom-timeout", type=float, default=1.5, help="standalone test: odometry freshness timeout")
    p.add_argument("--imu-timeout", type=float, default=1.5, help="standalone test: IMU freshness timeout")
    p.add_argument("--test-json", default="test.json", help="JSON file with parameter name and search settings")
    p.add_argument("--config", default=DEFAULT_CONFIG, help="VINS YAML config path")
    p.add_argument("--bag", default=DEFAULT_BAG, help="rosbag path, default: data.bag in current dir")
    p.add_argument("--bag-cwd", default=None, help="working directory for rosbag play")
    p.add_argument("--launch-cmd", default=DEFAULT_LAUNCH_CMD, help="VINS launch command")
    p.add_argument("--ros-setup", default=None, help="setup.bash sourced for launch/bag commands; defaults depend on ROS version")
    p.add_argument("--vins-setup", default=None, help="optional VINS workspace setup.bash sourced after --ros-setup")
    p.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    p.add_argument("--cam-topic", default=None, help="camera topic to check; default is image_topic/image0_topic from YAML")
    p.add_argument("--imu-topic", default=None, help="IMU topic to check; default is imu_topic from YAML")

    # Search parameters are loaded from JSON (--test-json):
    # step, min_step, max_fails, max_trials, max_step_reductions.
    # See test.json example generated with this version.

    p.add_argument("--launch-wait", type=float, default=5.0, help="seconds to wait after roslaunch before rosbag")
    p.add_argument("--odom-wait", type=float, default=20.0, help="seconds to wait for first odometry")
    p.add_argument("--topic-wait", type=float, default=10.0, help="seconds to wait for cam/IMU input data after rosbag starts")
    p.add_argument("-fixpos", "--fixpos", dest="fixpos_enabled", action="store_true", default=None, help="enable stable start position lock; overrides test.json")
    p.add_argument("--no-fixpos", dest="fixpos_enabled", action="store_false", help="disable stable start position lock; overrides test.json")
    p.add_argument("--fixpos-time", type=float, default=None, help="stable window duration in seconds; overrides test.json")
    p.add_argument("--fixpos-eps", type=float, default=None, help="max allowed X/Y/Z range in stable window, meters; overrides test.json")
    p.add_argument("--fixpos-max-wait", type=float, default=None, help="max seconds to wait for stable start; overrides test.json")
    p.add_argument("--bag-info-timeout", type=float, default=20.0, help="timeout for rosbag info precheck")
    p.add_argument("--after-bag-wait", type=float, default=2.0, help="wait after rosbag EOF for last odometry")
    p.add_argument("--pos-print-period", type=float, default=1.0, help="seconds between live position prints during a trial")
    p.add_argument("--between-trials", type=float, default=3.0, help="pause after stopping VINS")
    p.add_argument("--max-trial-time", type=float, default=600.0, help="safety timeout per trial")
    p.add_argument("--bag-rate", type=float, default=1.0)
    p.add_argument("--clock", action="store_true", default=True, help="add --clock to rosbag play; enabled by default")
    p.add_argument("--no-clock", dest="clock", action="store_false", help="do not add --clock to rosbag play")
    p.add_argument("--show-roslaunch-output", action="store_true", default=True, help="print filtered roslaunch output; enabled by default")
    p.add_argument("--hide-roslaunch-output", dest="show_roslaunch_output", action="store_false", help="hide roslaunch output")
    p.add_argument("--show-rosbag-output", action="store_true", default=False, help="print filtered rosbag output; disabled by default to avoid [rosbag] spam")
    p.add_argument("--hide-rosbag-output", dest="show_rosbag_output", action="store_false", help="hide rosbag output")
    p.add_argument("--show-process-output", action="store_true", default=None, help="legacy: print both roslaunch and rosbag output")
    p.add_argument("--hide-process-output", dest="show_process_output", action="store_false", help="legacy: hide both roslaunch and rosbag output")
    p.add_argument("--roslaunch-log-mode", choices=["quiet", "warn", "all"], default="warn", help="roslaunch output filter: quiet=none, warn=warnings/errors only, all=everything")
    p.add_argument("--stop-file", default="~/scripts/STOP", help="create this file to stop the tuner safely from another terminal")

    p.add_argument("--csv", default="vins_param_tune_results.csv")
    p.add_argument("--keep-best", action="store_true", help="leave best value in YAML at the end")
    p.add_argument("--dry-run", action="store_true", help="only print candidates, do not launch")
    return p.parse_args()


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    args = parse_args()
    if args.ros1 and args.ros2:
        raise RuntimeError("Choose only one: --ros1 or --ros2")
    if args.ros1 != (ROS_VERSION == 1):
        raise RuntimeError("ROS mode must be selected before imports; use --ros1 for ROS1, omit it for ROS2")
    if args.launch_cmd is None:
        args.launch_cmd = "roslaunch vins_estimator euroc.launch" if ROS_VERSION == 1 else "ros2 launch vins_estimator euroc.launch.py"
    if args.ros_setup is None:
        args.ros_setup = "~/ros1_noetic_ws/install_isolated/setup.bash" if ROS_VERSION == 1 else "~/ros2_jazzy/install/setup.bash"
    if args.vins_setup is None:
        args.vins_setup = "~/vinsmono_ws/devel/setup.bash" if ROS_VERSION == 1 else "~/vins_ws/install/setup.bash"
    args.cam_topic = args.cam_topic or "/cam0/image_raw"
    args.imu_topic = args.imu_topic or "/imu0"
    if args.fixpos_enabled is None:
        args.fixpos_enabled = False
    if args.fixpos_time is None:
        args.fixpos_time = 5.0
    if args.fixpos_eps is None:
        args.fixpos_eps = 0.02
    if args.fixpos_max_wait is None:
        args.fixpos_max_wait = 60.0
    if args.test:
        init_ros_node(args, "vins_test")
        try:
            run_standalone_test(args)
        finally:
            shutdown_ros_node()
        return
    if not args.param:
        raise RuntimeError("-param is required in automatic tuning mode; use -test for standalone measurement")
    args.config = expand_path(args.config)
    args.bag = expand_path(args.bag)
    args.csv = expand_path(args.csv)

    # Backward compatibility for older flags. New default: roslaunch output ON, rosbag output OFF.
    if args.show_process_output is True:
        args.show_roslaunch_output = True
        args.show_rosbag_output = True
    elif args.show_process_output is False:
        args.show_roslaunch_output = False
        args.show_rosbag_output = False

    if args.bag_cwd is None:
        args.bag_cwd = os.path.dirname(args.bag) or None

    if not os.path.exists(args.config):
        raise RuntimeError(f"Config file not found: {args.config}")
    if not os.path.exists(args.bag):
        raise RuntimeError(f"Bag file not found: {args.bag}")

    settings = load_search_settings(args.test_json, args.param)
    fixpos_settings = load_fixpos_settings(args.test_json)
    test_settings = load_test_settings(args.test_json)
    args.param = settings.param
    args.test_json = expand_path(args.test_json)
    args.repeats = test_settings.repeats
    args.aggregation = test_settings.aggregation

    # Apply CLI overrides for FIXPOS.
    args.fixpos_enabled = fixpos_settings.enabled if args.fixpos_enabled is None else args.fixpos_enabled
    args.fixpos_time = fixpos_settings.time if args.fixpos_time is None else args.fixpos_time
    args.fixpos_eps = fixpos_settings.eps if args.fixpos_eps is None else args.fixpos_eps
    args.fixpos_max_wait = fixpos_settings.max_wait if args.fixpos_max_wait is None else args.fixpos_max_wait

    original_text = read_file(args.config)
    original_value = find_yaml_param(original_text, args.param)

    if args.cam_topic is None:
        args.cam_topic = find_yaml_string_param(original_text, ["image_topic", "image0_topic"], "/cam0/image_raw")
    if args.imu_topic is None:
        args.imu_topic = find_yaml_string_param(original_text, ["imu_topic"], "/imu0")

    backup_path = args.config + f".backup_vins_tune_{time.strftime('%Y%m%d_%H%M%S')}"
    write_file(backup_path, original_text)

    print(f"Config: {args.config}")
    print(f"Backup: {backup_path}")
    print(f"Bag:    {args.bag}")
    print(f"JSON:   {args.test_json}")
    print(f"Param:  {args.param} = {original_value:.9f}")
    args.objective_name = settings.objective
    print(f"Goal:   minimize objective score: {settings.objective}")
    print(f"Method: probe both sides around current best, then reduce step")
    print(f"Search: step={settings.step:.9f}, min_step={settings.min_step:.9f}, "
          f"max_fails={settings.max_fails}, max_trials={settings.max_trials}, "
          f"max_step_reductions={settings.max_step_reductions}, "
          f"step_reduce_factor={settings.step_reduce_factor:.3f}")
    print(f"Objective weights: delta_xy={settings.delta_xy_weight:.3f}, delta_xyz={settings.delta_xyz_weight:.3f}, "
          f"distance_xy={settings.distance_xy_weight:.3f}, expected_distance_xy={settings.expected_distance_xy}")
    print(f"Launch: {args.launch_cmd}")
    print(f"ROS env: {expand_path(args.ros_setup)}")
    print(f"VINS env: {expand_path(args.vins_setup) if args.vins_setup else 'inherited'}")
    print(f"Clock:  {args.clock}")
    print(f"Cam:    {args.cam_topic}")
    print(f"IMU:    {args.imu_topic}")
    print(f"Odom:   {args.odom_topic}")
    print(f"FIXPOS: enabled={args.fixpos_enabled} time={args.fixpos_time:.1f}s eps={args.fixpos_eps:.4f}m max_wait={args.fixpos_max_wait:.1f}s")
    print(f"Repeats: {args.repeats} per parameter value, aggregation={args.aggregation}")
    print(f"Logs:   roslaunch={'on' if args.show_roslaunch_output else 'off'} ({args.roslaunch_log_mode}), rosbag={'on' if args.show_rosbag_output else 'off'}")
    print(f"Stop:   touch {expand_path(args.stop_file)}")

    print_ros_command_check(args)
    print_bag_info(args)

    restore_original = {"needed": True}

    def restore():
        if restore_original["needed"]:
            try:
                write_file(args.config, original_text)
                print(f"Restored original config: {args.param} = {original_value:.9f}")
            except Exception as e:
                print(f"WARN: failed to restore original config: {e}")

    atexit.register(stop_all_processes)
    atexit.register(shutdown_ros_node)
    atexit.register(restore)

    if args.dry_run:
        print()
        print("========== DRY RUN ==========")
        print(f"1. First parameter value: {args.param}={original_value:.9f} (repeats={args.repeats}, aggregation={args.aggregation})")
        print(f"2. Check nearest value: {args.param}={original_value + settings.step:.9f}")
        print(f"3. If no improvement, check opposite: {args.param}={original_value - settings.step:.9f}")
        print("4. Probe both sides around the current best; reduce step when neither side improves.")
        return

    init_ros_node(args, "vins_param_tuner_4_0_0")

    results: List[TrialResult] = []
    tried = set()
    trial_index = 0
    best: Optional[TrialResult] = None
    baseline_score: Optional[float] = None
    current_value = round(original_value, 9)
    step = settings.step
    direction: Optional[int] = None
    fails_at_current_step = 0
    step_reductions = 0

    def already_tried(v: float) -> bool:
        return round(v, 9) in tried

    def run_candidate(value: float, label: str) -> Optional[TrialResult]:
        nonlocal trial_index, best, baseline_score
        value = round(value, 9)
        if stop_requested:
            return None
        if trial_index >= settings.max_trials:
            return None
        if already_tried(value):
            print(f"[DBG] Skip already tested {args.param}={value:.9f}")
            return None
        tried.add(value)
        trial_index += 1
        best_score_before = result_score(best) if best is not None else None
        print()
        print(f"[SEARCH] {label}")
        print(f"[SEARCH] testing {args.param}={value:.9f} with {args.repeats} repeat(s), aggregation={args.aggregation}")

        runs: List[TrialResult] = []
        for repeat_index in range(1, args.repeats + 1):
            if stop_requested:
                break
            rr = run_trial(args, trial_index, value, original_text, repeat_index=repeat_index, repeats=args.repeats)
            rr.objective_score = calculate_objective_score(rr, settings)
            runs.append(rr)
            if args.repeats > 1:
                if rr.ok and rr.objective_score is not None:
                    print(f"[REPEAT] {repeat_index}/{args.repeats}: score={rr.objective_score:.6f} DeltaXY={rr.score_delta_xy:.6f} DeltaXYZ={rr.delta_xyz:.6f}")
                else:
                    print(f"[REPEAT] {repeat_index}/{args.repeats}: FAIL | {rr.reason}")

        r = aggregate_trial_results(trial_index, value, runs, args.aggregation)
        results.append(r)
        print_trial_result(
            r, args, label, step, direction, settings.max_trials,
            step_reductions, settings.max_step_reductions,
            fails_at_current_step, settings.max_fails,
            baseline_score, best_score_before, settings.improvement_epsilon,
        )
        save_csv(args.csv, results)

        if r.ok and r.objective_score is not None:
            if baseline_score is None:
                baseline_score = r.objective_score

            best_score_now = result_score(best)
            if best is None or r.objective_score < (best_score_now - settings.improvement_epsilon):
                best = r
                print(f"[SEARCH] accepted as best: {args.param}={r.value:.9f}, score={r.objective_score:.6f}, DeltaXY={r.score_delta_xy:.6f} m, DeltaXYZ={r.delta_xyz:.6f} m, std={r.score_stddev:.6f}")
        return r

    print()
    print("========== DIRECTIONAL SEARCH ==========")

    base_result = run_candidate(current_value, "initial value from YAML")
    if best is None:
        print("No valid baseline result; stopping search.")
    else:
        while trial_index < settings.max_trials and not stop_requested:
            if check_stop_file(args):
                print("[SEARCH] stop requested")
                break
            if step < settings.min_step:
                print(f"[SEARCH] stop: step {step:.9f} < min_step {settings.min_step:.9f}")
                break
            if step_reductions > settings.max_step_reductions:
                print(f"[SEARCH] stop: step reductions {step_reductions} > max_step_reductions {settings.max_step_reductions}")
                break

            best_before_score = result_score(best)
            print()
            print(f"[SEARCH] state: best={best.value:.9f} score={best_before_score:.6f} "
                  f"DeltaXY={best.score_delta_xy:.6f} DeltaXYZ={best.delta_xyz:.6f} | "
                  f"step={step:.9f} | fails={fails_at_current_step}/{settings.max_fails} | "
                  f"reductions={step_reductions}/{settings.max_step_reductions} | "
                  f"trials={trial_index}/{settings.max_trials}")

            candidates = [round(best.value + step, 9), round(best.value - step, 9)]
            improved_this_round = False
            ran_any = False
            best_at_round_start = best

            for cand, label in ((candidates[0], "probe +step from best"), (candidates[1], "probe -step from best")):
                if trial_index >= settings.max_trials or stop_requested:
                    break
                if already_tried(cand):
                    print(f"[DBG] Skip already tested {args.param}={cand:.9f}")
                    continue
                ran_any = True
                before = result_score(best)
                r = run_candidate(cand, label)
                after = result_score(best)
                if r is not None and after is not None and before is not None and after < before - settings.improvement_epsilon:
                    improved_this_round = True

            if improved_this_round:
                fails_at_current_step = 0
                print(f"[SEARCH] improvement found; continue around new best {args.param}={best.value:.9f}")
                continue

            if not ran_any:
                # Both sides were already tested. Reduce step immediately.
                print("[SEARCH] both sides already tested at this step")

            fails_at_current_step += 1
            print(f"[SEARCH] no improvement around best; fail {fails_at_current_step}/{settings.max_fails}")

            if fails_at_current_step > settings.max_fails:
                if step_reductions >= settings.max_step_reductions:
                    print(f"[SEARCH] stop: no improvement and max step reductions reached ({settings.max_step_reductions})")
                    break
                old_step = step
                step = step / settings.step_reduce_factor
                step_reductions += 1
                fails_at_current_step = 0
                direction = None
                print(f"[SEARCH] reduce step: {old_step:.9f} -> {step:.9f}")
    print()
    print("========== FINAL REPORT ==========")
    print(f"Results CSV: {args.csv}")

    valid = [r for r in results if r.ok and r.objective_score is not None]
    valid_sorted = sorted(valid, key=lambda x: x.objective_score)
    for r in valid_sorted[:10]:
        dist_info = "" if r.distance_xy is None else f"  DistXY={r.distance_xy:.6f} m"
        std_info = "" if r.score_stddev is None else f"  std={r.score_stddev:.6f}"
        rep_info = f"  repeats={r.valid_repeats}/{r.repeat_count}" if getattr(r, "repeat_count", 1) > 1 else ""
        print(f"{args.param}={r.value:.9f}  score={r.objective_score:.6f}{std_info}{rep_info}  DeltaXY={r.score_delta_xy:.6f} m  DeltaXYZ={r.delta_xyz:.6f} m{dist_info}  messages={r.odom_messages}")

    if best is None:
        print("BEST: not found")
        return

    print()
    print(f"BEST {args.param}: {best.value:.9f}")
    print(f"BEST objective score: {best.objective_score:.6f} ({settings.objective})")
    if best.score_stddev is not None and getattr(best, "repeat_count", 1) > 1:
        print(f"BEST score stddev: {best.score_stddev:.6f} from {best.valid_repeats}/{best.repeat_count} valid repeats")
    print(f"BEST Delta XY: {best.score_delta_xy:.6f} m")
    print(f"BEST Delta XYZ: {best.delta_xyz:.6f} m")
    if best.distance_xy is not None:
        print(f"BEST Distance XY: {best.distance_xy:.6f} m")
        print(f"BEST Distance XYZ: {best.distance_xyz:.6f} m")
    if baseline_score is not None:
        info = describe_improvement(best.objective_score, baseline_score, None)
        if info:
            print(f"BEST improvement {info}")

    if args.keep_best:
        best_text = set_yaml_param(original_text, args.param, best.value)
        write_file(args.config, best_text)
        restore_original["needed"] = False
        print(f"Config left with best value: {args.param} = {best.value:.9f}")
    else:
        print("Original config will be restored. Use --keep-best to keep the best value.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping active rosbag/roslaunch processes...")
        stop_all_processes()
    except Exception as e:
        print("ERROR:", e)
        sys.exit(1)
