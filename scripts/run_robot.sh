#!/usr/bin/env bash
# =============================================================================
# run_robot.sh —— 启动一台机器人完整的 DCL-SLAM 运行环境。
#
# 依次拉起以下组件（它们都是阻塞型的长驻进程，脚本用 '&' 把它们放到后台
# 并行运行，让脚本本身不被卡住）：
#   1. ROS1 主节点 roscore           —— 若远端已有 master 则直接复用
#   2. ROS1 <-> ROS2 参数桥 parameter_bridge
#   3. Livox MID360 激光雷达驱动
#   4. DCL-SLAM 单机节点 single_ugv.launch
#
# 设计要点：
#   - 全程在"子进程"里运行、并显式 unset 继承来的 ROS 环境，因此脚本结束
#     后不会污染调用方终端的环境变量，多次运行也能保持环境干净、可复现；
#   - 所有后台子进程的 PID 被记录进 child_pids 数组，由 cleanup() 统一回收，
#     避免脚本退出后留下孤儿进程；
#   - 结尾用 wait -n 实现"任一子进程退出，立即整体退出并清理"。
#
# 用法: ./scripts/run_robot.sh <a|b> [options]
# 例  : ./scripts/run_robot.sh a
#        ./scripts/run_robot.sh b --ros-ip 192.168.31.12 --domain-id 31 --robots 3
#
# 注意: ROS 的 setup 脚本会引用一些未定义的变量，因此不要开启 nounset(-u)。
# =============================================================================
set -eo pipefail

# ---------------- 目录路径 ----------------
# 依据脚本自身位置推导各工作区根目录，使脚本可从任意 CWD 被调用。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DCL_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS1_WS="${DCL_ROOT}/ros1_ws"
ROS2_WS="${DCL_ROOT}/ros2_ws"
# ros1_bridge 的路径可由环境变量覆盖（例如在不同容器中构建时）。
ROS1_BRIDGE_ROOT="${ROS1_BRIDGE_ROOT:-$(cd -- "${DCL_ROOT}/.." && pwd)/ros1_bridge}"

# ---------------- 默认参数 ----------------
# 允许通过同名环境变量覆盖默认值（${VAR:-default} 语法：未设置时取默认）。
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

# ---------------- 参数解析 ----------------
# 第 1 个位置参数必须是机器人代号 a|b，据此查表得到默认 Wi-Fi IP；
# 后续长选项（--ros-ip / --domain-id / --robots）可覆盖这些默认值。
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

# 数值合法性校验：DOMAIN_ID 为非负整数，NUMBER_OF_ROBOTS 为正整数。
if ! [[ "${DOMAIN_ID}" =~ ^[0-9]+$ ]]; then
    echo "Error: --domain-id must be a non-negative integer" >&2
    exit 2
fi
if ! [[ "${NUMBER_OF_ROBOTS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: --robots must be a positive integer" >&2
    exit 2
fi

# ---------------- 环境 setup 路径 ----------------
# 汇总需要在 source 之前确认存在的各 setup 脚本与配置文件路径。
ROS1_SETUP="/opt/ros/${ROS1_DISTRO}/setup.bash"
ROS2_SETUP="/opt/ros/${ROS2_DISTRO}/setup.bash"
ROS1_WS_SETUP="${ROS1_WS}/devel/setup.bash"
ROS2_WS_SETUP="${ROS2_WS}/install/local_setup.bash"
BRIDGE_SETUP="${ROS1_BRIDGE_ROOT}/install/local_setup.bash"
FAST_DDS_PROFILE="${DCL_ROOT}/config/fastdds_wifi.xml"

# 启动前先做"依赖自检"：任一必需文件缺失立即报错退出，
# 这比运行到一半因缺文件失败更容易定位问题。
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

# ---------------- 构建干净、可复现的 ROS 环境 ----------------
# 关键设计：本脚本在子进程中运行，以下所有 unset/source/export 只影响本
# 脚本自身，绝不会改动调用方终端。这保证：
#   1) 每次运行都从"空白"开始，不依赖调用者终端里残留的 ROS 变量；
#   2) 变量是"任务环境"而非"个人偏好"，不该写进 ~/.bashrc 污染其他终端。
# 先彻底丢弃可能继承进来的 ROS1/ROS2/colcon 相关变量，避免与后面 source
# 的 setup 脚本冲突。
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ROOT
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH

# source ROS1 发行版与 ROS1 工作区的 setup，得到可用的 roslaunch/rostopic 等；
# --extend 表示在已有环境上叠加而非覆盖。SC1090 是 shellcheck 对动态
# source 路径的告警，此处路径确为运行时生成，故忽略。
# shellcheck disable=SC1090
source "${ROS1_SETUP}"
# shellcheck disable=SC1090
source "${ROS1_WS_SETUP}" --extend

# ---------------- 导出本次任务所需的联网/通信变量 ----------------
# ROS_MASTER_URI : 指向本机 ROS1 master（roscore 或复用已有 master）；
# ROS_IP         : 本机对外通告的 IP（机器人 a/b 不同），用于 Wi-Fi 组网；
# ROS_HOSTNAME   : 与 ROS_IP 二选一，这里统一用 IP 故将其清空，防止残留
#                  hostname 使 ROS_IP 失效；
# ROS_DOMAIN_ID  : ROS2 的通信域，配合 FAST DDS 配置在多机器人间隔离/互联；
# RMW_IMPLEMENTATION / FASTRTPS_DEFAULT_PROFILES_FILE: 固定 ROS2 的 DDS 实现
#                  与 QoS 配置文件（多机器人 Wi-Fi 场景必需）。
export ROS_MASTER_URI="http://127.0.0.1:11311"
export ROS_IP="${ROBOT_IP}"
unset ROS_HOSTNAME
export ROS_DOMAIN_ID="${DOMAIN_ID}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FAST_DDS_PROFILE}"

# DCL 的 glog 日志默认写到 $HOME/log，确保目录存在。
mkdir -p "${HOME}/log"

# ---------------- 子进程生命周期管理 ----------------
# child_pids 记录本脚本启动的所有后台长驻进程 PID，是 cleanup 统一回收的
# 依据。cleanup 在每个子进程退出 / Ctrl-C(INT) / TERM / EXIT 时触发：先对
# 仍存活的子进程逐个发送 SIGINT（等价于在它们终端里按 Ctrl-C），再 wait
# 回收，避免留下孤儿进程。
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

# ---------------- 组件 1: ROS1 master (roscore) ----------------
# rosparam 能连通即说明已有 master（可能是被上一台 robot 或其他会话拉起），
# 直接复用；否则后台启动 roscore，并用"轮询 + sleep 0.1"而非固定 sleep
# 的方式等待 master 就绪，超时仍没就绪则报错退出。
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

# ---------------- 组件 2: ROS1/ROS2 参数桥 ----------------
# 关键技巧：bridge 需要 ROS2 环境，但外层已经 source 了 ROS1 的 setup，二者
# 混在同一环境会冲突。于是用"(...)"子 shell 隔离：在里面先 unset ROS_DISTRO
# 再 source ROS2 相关 setup，从而不影响外层 ROS1 环境。
# 子 shell 最后用 exec 让 ros2 run 直接"取代"当前子 shell 进程，保证 $!
# 拿到的 bridge_pid 就是真实的 bridge 进程 PID（而不是中间多包的那层 bash）。
# 整体以 '&' 放后台，配合后面的 sleep + kill -0 做一次粗略的启动存活检查。
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

# ---------------- 组件 3: Livox MID360 驱动 ----------------
# 同样是阻塞型节点，放后台并把 PID 记入数组；随后轮询其是否已在 ROS1
# master 上注册好话题，作为"启动成功"的判断依据。
echo "[Livox] Starting MID360 driver for robot ${ROBOT_PREFIX}"
roslaunch livox_ros_driver2 msg_MID360.launch robotPrefix:="${ROBOT_PREFIX}" &
livox_pid=$!
child_pids+=("${livox_pid}")

# 话题注册通常不到 1 秒即可完成。即使超时也继续往下走（仅告警），让驱动
# 仍有机会在雷达暂时不可用后自行恢复。
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

# ---------------- 组件 4: DCL-SLAM 主节点 ----------------
# 最后一个长驻进程，同样放后台。robotPrefix 决定话题命名空间，
# number_of_robots 告诉 SLAM 本团队有几台机器人。
echo "[DCL] Starting robot ${ROBOT_PREFIX}; team size=${NUMBER_OF_ROBOTS}"
roslaunch dcl_slam single_ugv.launch \
    robotPrefix:="${ROBOT_PREFIX}" \
    number_of_robots:="${NUMBER_OF_ROBOTS}" &
dcl_pid=$!
child_pids+=("${dcl_pid}")

echo
echo "Robot ${ROBOT_PREFIX} is running. ROS_IP=${ROS_IP}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Press Ctrl-C to stop the processes started by this script."

# ---------------- 收尾: 任一进程退出即整体退出 ----------------
# 普通 wait 会等"所有"后台进程结束才返回，而 wait -n 只要"任意一个"结束
# 就立即返回。因此一旦 roscore/bridge/livox/dcl 中任何一个意外退出，脚本
# 立刻结束并走 EXIT trap -> cleanup，把其余仍在运行的进程一并停掉，实现
# "整条 launch 同生共死"。此处命令替换/echo 期间 wait -n 可能返回非零，
# 需在 set -e 下显式承接退出码。
wait -n "${child_pids[@]}"
exit_status=$?
echo "A managed process exited with status ${exit_status}; stopping the remaining processes." >&2
exit "${exit_status}"
