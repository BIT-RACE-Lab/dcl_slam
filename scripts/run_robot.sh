#!/usr/bin/env bash
# Copyright 2026 lwb <wb.lv@qq.com>
# SPDX-License-Identifier: Apache-2.0

# =============================================================================
# Start the complete DCL-SLAM runtime for one robot.
# 启动单台机器人的完整 DCL-SLAM 运行环境。
#
# The script starts these long-running components in order:
#   1. The ROS 1 master (or reuses an existing local master)
#   2. The ROS 1 <-> ROS 2 parameter bridge
#   3. The Livox MID360 driver
#   4. The DCL-SLAM single-robot launch file
# 脚本按顺序启动以下常驻组件：
#   1. ROS 1 master（或复用本机已有 master）
#   2. ROS 1 <-> ROS 2 参数桥
#   3. Livox MID360 驱动
#   4. DCL-SLAM 单机器人 launch 文件
#
# Inherited ROS variables are cleared so every run starts from a predictable
# environment. Background PIDs are tracked and stopped together. If any managed
# process exits, the script stops the remaining managed processes.
# 脚本会清理继承的 ROS 变量，使每次运行都从可预测的环境开始；所有后台进程
# 的 PID 都会被记录和统一停止，任一受管进程退出时，其余进程也会随之停止。
#
# Usage: ./scripts/run_robot.sh <a|b> [options]
# 用法：./scripts/run_robot.sh <a|b> [选项]
# Examples:
# 示例：
#   ./scripts/run_robot.sh a
#   ./scripts/run_robot.sh b --ros-ip 192.168.31.12 --domain-id 31 --robots 3
#
# ROS setup scripts may reference undefined variables, so nounset (-u) is not
# enabled.
# ROS setup 脚本可能引用未定义变量，因此不启用 nounset（-u）。
# =============================================================================
set -eo pipefail

# ---------------- Paths / 路径 ----------------
# Derive workspace paths from this file so it works from any CWD.
# 根据脚本自身位置推导工作区路径，使其可从任意当前目录调用。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DCL_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS1_WS="${DCL_ROOT}/ros1_ws"
ROS2_WS="${DCL_ROOT}/ros2_ws"
# Allow deployments with a bridge checkout elsewhere to override this path.
# 允许通过环境变量指定位于其他目录的 bridge 工作区。
ROS1_BRIDGE_ROOT="${ROS1_BRIDGE_ROOT:-$(cd -- "${DCL_ROOT}/.." && pwd)/ros1_bridge}"

# ---------------- Defaults / 默认值 ----------------
# ROS distribution and domain defaults may be overridden through the environment.
# 可通过环境变量覆盖默认的 ROS 发行版和 DDS Domain ID。
ROS1_DISTRO="${ROS1_DISTRO:-noetic}"
ROS2_DISTRO="${ROS2_DISTRO:-foxy}"
DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
NUMBER_OF_ROBOTS=2
ROBOT_PREFIX=""
ROBOT_IP=""

usage() {
    cat <<'EOF'
Usage: ./scripts/run_robot.sh <a|b> [options]

Options:
  --ros-ip IP       Override the robot Wi-Fi address
  --domain-id ID    ROS 2 domain ID (default: 30 or $ROS_DOMAIN_ID)
  --robots N        Number of robots (default: 2)
  -h, --help        Show this help

Default Wi-Fi addresses:
  robot a: 192.168.31.11
  robot b: 192.168.31.12
EOF
}

if [[ "$#" -lt 1 ]]; then
    usage >&2
    exit 2
fi
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    usage
    exit 0
fi

# ---------------- Arguments / 参数 ----------------
# The first argument selects the robot and its default Wi-Fi address. Options may
# override that address, the DDS domain, and the team size.
# 第一个参数选择机器人及其默认 Wi-Fi 地址；后续选项可覆盖该地址、DDS 域和
# 机器人数量。
ROBOT_PREFIX="$1"
shift
case "${ROBOT_PREFIX}" in
    a) ROBOT_IP="192.168.31.11" ;;
    b) ROBOT_IP="192.168.31.12" ;;
    *)
        echo "Error: robot prefix must be a or b; got: ${ROBOT_PREFIX}" >&2
        usage >&2
        exit 2
        ;;
esac

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --ros-ip)
            [[ "$#" -ge 2 ]] || { echo "Error: --ros-ip requires a value" >&2; exit 2; }
            ROBOT_IP="$2"
            shift 2
            ;;
        --domain-id)
            [[ "$#" -ge 2 ]] || { echo "Error: --domain-id requires a value" >&2; exit 2; }
            DOMAIN_ID="$2"
            shift 2
            ;;
        --robots)
            [[ "$#" -ge 2 ]] || { echo "Error: --robots requires a value" >&2; exit 2; }
            NUMBER_OF_ROBOTS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Error: unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

# Validate values before starting any processes.
# 在启动任何进程前校验参数值。
if ! [[ "${DOMAIN_ID}" =~ ^[0-9]+$ ]]; then
    echo "Error: --domain-id must be a non-negative integer" >&2
    exit 2
fi
if ! [[ "${NUMBER_OF_ROBOTS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: --robots must be a positive integer" >&2
    exit 2
fi

# ---------------- Setup files / 环境文件 ----------------
ROS1_SETUP="/opt/ros/${ROS1_DISTRO}/setup.bash"
ROS2_SETUP="/opt/ros/${ROS2_DISTRO}/setup.bash"
ROS1_WS_SETUP="${ROS1_WS}/devel/setup.bash"
ROS2_WS_SETUP="${ROS2_WS}/install/local_setup.bash"
BRIDGE_SETUP="${ROS1_BRIDGE_ROOT}/install/local_setup.bash"
FAST_DDS_PROFILE="${DCL_ROOT}/config/fastdds_wifi.xml"

# Fail before startup if a required environment or configuration is missing.
# 若缺少必需的环境文件或配置，则在启动前立即退出。
for required_file in \
    "${ROS1_SETUP}" \
    "${ROS2_SETUP}" \
    "${ROS1_WS_SETUP}" \
    "${ROS2_WS_SETUP}" \
    "${BRIDGE_SETUP}" \
    "${FAST_DDS_PROFILE}"; do
    if [[ ! -f "${required_file}" ]]; then
        echo "Error: required file not found: ${required_file}" >&2
        echo "Build DCL-SLAM and ros1_bridge first; see README.md." >&2
        exit 1
    fi
done

# ---------------- Clean ROS environment / 清理 ROS 环境 ----------------
# Clear inherited ROS and colcon state before sourcing the required environments.
# These changes affect only this script process, not the caller's shell.
# 在加载所需环境前清除继承的 ROS 和 colcon 状态；这些修改只影响当前脚本
# 进程，不会改变调用者的 shell 环境。
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ROOT
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH

# Load ROS 1 and overlay the DCL workspace. SC1090 is suppressed because these
# setup paths are intentionally resolved at runtime.
# 加载 ROS 1 并叠加 DCL 工作区。setup 路径在运行时动态解析，因此忽略
# ShellCheck 的 SC1090 提示。
# shellcheck disable=SC1090
source "${ROS1_SETUP}"
# shellcheck disable=SC1090
source "${ROS1_WS_SETUP}" --extend

# ---------------- Network environment / 网络环境 ----------------
# Each robot uses its own local ROS 1 master and advertises its Wi-Fi address.
# Both robots must share the DDS domain and Fast DDS profile.
# 每台机器人使用各自的本机 ROS 1 master，并广播自身 Wi-Fi 地址；两台机器人
# 必须使用相同的 DDS 域和 Fast DDS 配置。
export ROS_MASTER_URI="http://127.0.0.1:11311"
export ROS_IP="${ROBOT_IP}"
unset ROS_HOSTNAME
export ROS_DOMAIN_ID="${DOMAIN_ID}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FAST_DDS_PROFILE}"

# DCL writes glog output here by default.
# DCL 默认将 glog 日志写入此目录。
mkdir -p "${HOME}/log"

# ---------------- Process lifecycle / 进程生命周期 ----------------
# Track only processes started here. Cleanup sends SIGINT, waits for them, and
# leaves a pre-existing ROS master untouched.
# 只记录本脚本启动的进程。清理函数向其发送 SIGINT 并等待退出，同时保留
# 脚本启动前已经存在的 ROS master。
child_pids=()

cleanup() {
    local exit_status=$?
    trap - EXIT INT TERM
    if [[ "${#child_pids[@]}" -gt 0 ]]; then
        echo
        echo "Stopping robot ${ROBOT_PREFIX} processes..."
        local pid
        for pid in "${child_pids[@]}"; do
            if kill -0 "${pid}" 2>/dev/null; then
                kill -INT "${pid}" 2>/dev/null || true
            fi
        done
        wait "${child_pids[@]}" 2>/dev/null || true
    fi
    exit "${exit_status}"
}
trap cleanup EXIT INT TERM

# ---------------- Component 1: ROS 1 master / 组件 1：ROS 1 master ----------------
# Reuse an available local master; otherwise start one and poll until it is ready.
# 复用可用的本机 master；如果不存在，则启动一个并轮询等待其就绪。
if rosparam get /rosversion >/dev/null 2>&1; then
    echo "[ROS1] Reusing ROS master at ${ROS_MASTER_URI}"
else
    echo "[ROS1] Starting local roscore"
    roscore &
    roscore_pid=$!
    child_pids+=("${roscore_pid}")

    master_ready=false
    for _ in $(seq 1 50); do
        if rosparam get /rosversion >/dev/null 2>&1; then
            master_ready=true
            break
        fi
        sleep 0.1
    done
    if [[ "${master_ready}" != true ]]; then
        echo "Error: local ROS master did not become ready" >&2
        exit 1
    fi
fi

echo "[Bridge] Loading DCL two-robot topic configuration"
roslaunch dcl_slam bridge_dcl_two_robots.launch

# ---------------- Component 2: ROS 1/ROS 2 bridge / 组件 2：ROS 1/ROS 2 bridge ----------------
# Isolate ROS 2 setup in a subshell to avoid mixing distribution variables in the
# outer ROS 1 environment. exec makes the tracked PID the actual bridge process.
# 在子 shell 中隔离 ROS 2 环境，避免与外层 ROS 1 发行版变量混合；使用 exec
# 确保记录的 PID 对应实际 bridge 进程。
echo "[Bridge] Starting ROS1/ROS2 parameter bridge (domain ${ROS_DOMAIN_ID})"
(
    unset ROS_DISTRO
    # shellcheck disable=SC1090
    source "${ROS2_SETUP}"
    # shellcheck disable=SC1090
    source "${ROS2_WS_SETUP}"
    # shellcheck disable=SC1090
    source "${BRIDGE_SETUP}"
    ros2 daemon stop >/dev/null 2>&1 || true
    exec ros2 run ros1_bridge parameter_bridge
) &
bridge_pid=$!
child_pids+=("${bridge_pid}")

sleep 2
if ! kill -0 "${bridge_pid}" 2>/dev/null; then
    echo "Error: parameter_bridge exited during startup" >&2
    exit 1
fi

# ---------------- Component 3: Livox MID360 driver / 组件 3：Livox MID360 驱动 ----------------
echo "[Livox] Starting MID360 driver for robot ${ROBOT_PREFIX}"
roslaunch livox_ros_driver2 msg_MID360.launch robotPrefix:="${ROBOT_PREFIX}" &
livox_pid=$!
child_pids+=("${livox_pid}")

# Continue after a topic-registration timeout so the driver can recover if the
# sensor is temporarily unavailable.
# 话题注册超时后仍继续运行，使驱动能在雷达暂时不可用时自行恢复。
lidar_topic="/${ROBOT_PREFIX}/livox/lidar"
lidar_ready=false
for _ in $(seq 1 100); do
    if rostopic info "${lidar_topic}" >/dev/null 2>&1; then
        lidar_ready=true
        break
    fi
    if ! kill -0 "${livox_pid}" 2>/dev/null; then
        echo "Error: Livox launch exited during startup" >&2
        exit 1
    fi
    sleep 0.1
done
if [[ "${lidar_ready}" != true ]]; then
    echo "Warning: ${lidar_topic} is not registered yet; DCL-SLAM will wait for data" >&2
fi

# ---------------- Component 4: DCL-SLAM / 组件 4：DCL-SLAM ----------------
# robotPrefix selects the topic namespace; number_of_robots sets the team size.
# robotPrefix 选择话题命名空间，number_of_robots 设置机器人团队规模。
echo "[DCL] Starting robot ${ROBOT_PREFIX}; team size=${NUMBER_OF_ROBOTS}"
roslaunch dcl_slam single_ugv.launch \
    robotPrefix:="${ROBOT_PREFIX}" \
    number_of_robots:="${NUMBER_OF_ROBOTS}" &
dcl_pid=$!
child_pids+=("${dcl_pid}")

echo
echo "Robot ${ROBOT_PREFIX} is running. ROS_IP=${ROS_IP}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Press Ctrl-C to stop the processes started by this script."

# ---------------- Stop when any managed process exits / 任一受管进程退出时停止 ----------------
# Capture a nonzero child status without allowing errexit to bypass the message.
# 捕获子进程的非零退出状态，同时避免 errexit 跳过退出提示。
exit_status=0
wait -n "${child_pids[@]}" || exit_status=$?
echo "A managed process exited with status ${exit_status}; stopping the remaining processes." >&2
exit "${exit_status}"
