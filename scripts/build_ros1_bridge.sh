#!/usr/bin/env bash
# =============================================================================
# build_ros1_bridge.sh —— 编译定制版 ros1_bridge（ROS1 <-> ROS2 动态桥）。
#
# 为什么需要单独编译 ros1_bridge
#   ros1_bridge 的转换工厂在编译时按"当前可见的 ROS1/ROS2 自定义消息"生成，
#   因此必须先由 ./scripts/build.sh 生成 DCL 的 ROS1(ros1_ws/devel) 与
#   ROS2(ros2_ws/install) 消息，再让两边消息环境同时可见来编译本桥。
#   只有在以下情况才需要（重新）执行本脚本：
#     1) 首次构建 ros1_bridge；
#     2) 修改了 DCL 的 ROS1/ROS2 `.msg` 并重新运行 ./scripts/build.sh；
#     3) 修改了 ros1_bridge 的 mapping_rules.yaml；
#     4) 更换了 ROS1/ROS2 发行版。
#   普通 DCL 算法、launch 或运行参数修改不需要重编 bridge。
#
# 前置条件
#   - 已运行 ./scripts/build.sh，生成 DCL 的 ROS1/ROS2 消息；
#   - 定制版 ros1_bridge 已克隆到 dcl_slam 的同级目录，或用环境变量
#     ROS1_BRIDGE_ROOT 指定其它位置；
#   - 已安装 ROS1/ROS2 与 colcon。
#
# 用法: ./scripts/build_ros1_bridge.sh [选项]
# 例  : ./scripts/build_ros1_bridge.sh
#        ./scripts/build_ros1_bridge.sh -j 4
#        ROS1_DISTRO=noetic ROS2_DISTRO=foxy ./scripts/build_ros1_bridge.sh
#
# 注意: ROS 的 setup.bash 会读取尚未定义的环境变量，不能使用 nounset（-u）。
# =============================================================================
set -eo pipefail

# ---------------- 目录路径 ----------------
# 依据脚本自身位置推导各工作区根目录，使脚本可从任意 CWD 被调用。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DCL_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS1_WS="${DCL_ROOT}/ros1_ws"
ROS2_WS="${DCL_ROOT}/ros2_ws"
# ros1_bridge 默认从 dcl_slam 的同级目录查找，路径可用环境变量覆盖
# （与 run_robot.sh 的约定一致）。
ROS1_BRIDGE_ROOT="${ROS1_BRIDGE_ROOT:-$(cd -- "${DCL_ROOT}/.." && pwd)/ros1_bridge}"

# ---------------- 默认参数 ----------------
ROS1_DISTRO="${ROS1_DISTRO:-noetic}"
ROS2_DISTRO="${ROS2_DISTRO:-foxy}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS=""

usage() {
    cat <<'EOF'
用法: ./scripts/build_ros1_bridge.sh [选项]

前置条件：先运行 ./scripts/build.sh 生成 DCL 的 ROS1/ROS2 消息；定制版
ros1_bridge 位于 dcl_slam 的同级目录（可用 ROS1_BRIDGE_ROOT 覆盖）。
本脚本自动加载两边消息环境并执行 colcon build。

选项:
  -j, --jobs N  并行编译任务数（不指定时使用 colcon 默认值）
  -h, --help    显示帮助
EOF
}

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -j|--jobs)
            if [[ "$#" -lt 2 ]]; then
                echo "错误：选项 $1 需要一个数字参数" >&2
                usage >&2
                exit 2
            fi
            JOBS="$2"
            shift
            ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; exit 2 ;;
    esac
    shift
done

if [[ -n "${JOBS}" ]] && ! [[ "${JOBS}" =~ ^[0-9]+$ ]]; then
    echo "错误：-j 参数必须是正整数，实际为：${JOBS}" >&2
    exit 2
fi

# ---------------- 路径自检 ----------------
# 编译前先确认关键文件存在，比编译到一半因缺文件失败更容易定位问题。
ROS1_SETUP="/opt/ros/${ROS1_DISTRO}/setup.bash"
ROS2_SETUP="/opt/ros/${ROS2_DISTRO}/setup.bash"
ROS1_WS_SETUP="${ROS1_WS}/devel/setup.bash"
ROS2_WS_SETUP="${ROS2_WS}/install/local_setup.bash"
BRIDGE_SRC="${ROS1_BRIDGE_ROOT}/src/ros1_bridge"

for setup_path in "${ROS1_SETUP}" "${ROS2_SETUP}" \
    "${ROS1_WS_SETUP}" "${ROS2_WS_SETUP}"; do
    if [[ ! -f "${setup_path}" ]]; then
        echo "错误：未找到所需文件：${setup_path}" >&2
        case "${setup_path}" in
            *ros1_ws/devel/setup.bash|*ros2_ws/install/local_setup.bash)
                echo "请先运行 ./scripts/build.sh 生成 DCL 的 ROS1/ROS2 消息。" >&2
                ;;
        esac
        exit 1
    fi
done

if [[ ! -d "${BRIDGE_SRC}" ]]; then
    echo "错误：未找到 ros1_bridge 源码：${BRIDGE_SRC}" >&2
    echo "请将定制版 ros1_bridge 克隆到 dcl_slam 的同级目录，" >&2
    echo "或用 ROS1_BRIDGE_ROOT 指定其它位置。" >&2
    exit 1
fi
if ! command -v colcon >/dev/null 2>&1; then
    echo "错误：未找到 colcon，请安装 python3-colcon-common-extensions。" >&2
    exit 1
fi

# ---------------- 构建干净、可复现的 ROS 环境 ----------------
# 脚本在子进程内执行，以下 unset/source 只影响本脚本自身，不会污染调用方
# 终端。先彻底丢弃可能继承进来的 ROS1/ROS2/colcon 相关变量，保证从空白开始。
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ROOT
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH

# 加载 ROS1 发行版与 DCL 的 ROS1 消息工作区（--extend 在已有环境上叠加，
# 而非覆盖）。
# shellcheck disable=SC1090
source "${ROS1_SETUP}"
# shellcheck disable=SC1090
source "${ROS1_WS_SETUP}" --extend

# bridge 需要 ROS1 与 ROS2 环境共存。ROS2 与 ROS1 的发行版变量冲突，
# 先清掉 ROS_DISTRO 再 source ROS2，随后叠加 DCL 的 ROS2 消息镜像工作区。
unset ROS_DISTRO
# shellcheck disable=SC1090
source "${ROS2_SETUP}"
# shellcheck disable=SC1090
source "${ROS2_WS_SETUP}"

echo "[Bridge] 编译 ros1_bridge：${ROS1_BRIDGE_ROOT}"
cd "${ROS1_BRIDGE_ROOT}"
# --cmake-force-configure 强制重新配置，确保转换工厂按当前消息重新生成；
# colcon 本身没有 -j 参数（单包场景），通过 MAKEFLAGS 限制内部 make 的
# 编译并行度。
if [[ -n "${JOBS}" ]]; then
    MAKEFLAGS="-j${JOBS}" colcon build \
        --symlink-install \
        --packages-select ros1_bridge \
        --cmake-force-configure \
        --cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
else
    colcon build \
        --symlink-install \
        --packages-select ros1_bridge \
        --cmake-force-configure \
        --cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
fi

echo "ros1_bridge 构建完成。"
