#!/usr/bin/env bash
# ROS 的 setup.bash 会读取尚未定义的环境变量，不能使用 nounset（-u）。
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DCL_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS1_WS="${DCL_ROOT}/ros1_ws"
ROS2_WS="${DCL_ROOT}/ros2_ws"
ROS1_DISTRO="${ROS1_DISTRO:-noetic}"
ROS2_DISTRO="${ROS2_DISTRO:-foxy}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

build_ros1=true
build_ros2=true
JOBS=""

usage() {
    cat <<'EOF'
用法: ./scripts/build.sh [选项]

选项可组合使用，例如：./scripts/build.sh --ros1-only -j 4
默认依次构建 DCL-SLAM ROS1 工作区和 ROS2 消息镜像。

选项:
  --ros1-only  只构建 ROS1 工作区
  --ros2-only  只构建 ROS2 消息镜像
  -j, --jobs N 并行编译任务数（不指定时使用构建工具默认值）
  -h, --help   显示帮助
EOF
}

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --ros1-only) build_ros2=false ;;
        --ros2-only) build_ros1=false ;;
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

if ! "${build_ros1}" && ! "${build_ros2}"; then
    echo "错误：--ros1-only 与 --ros2-only 不能同时使用" >&2
    usage >&2
    exit 2
fi

reset_ros_environment() {
    # 脚本在子进程内执行，清理这些变量不会污染调用者的终端。
    unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ROOT
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
    unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
}

if "${build_ros1}"; then
    ROS1_SETUP="/opt/ros/${ROS1_DISTRO}/setup.bash"
    if [[ ! -f "${ROS1_SETUP}" ]]; then
        echo "错误：未找到 ${ROS1_SETUP}，请安装 ROS1 ${ROS1_DISTRO}。" >&2
        exit 1
    fi

    reset_ros_environment
    source "${ROS1_SETUP}"

    if ! command -v catkin >/dev/null 2>&1; then
        echo "错误：未找到 catkin，请安装 python3-catkin-tools。" >&2
        exit 1
    fi

    echo "[ROS1] 构建工作区：${ROS1_WS}"
    cd "${ROS1_WS}"
    if [[ ! -d .catkin_tools ]]; then
        catkin init
    fi
    catkin config --merge-devel
    catkin config --cmake-args \
        -DROS_EDITION=ROS1 \
        "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"

    catkin_args=()
    if [[ -n "${JOBS}" ]]; then
        catkin_args+=("-j" "${JOBS}")
    fi
    catkin build "${catkin_args[@]}" dcl_slam dcl_fast_lio livox_ros_driver2
fi

if "${build_ros2}"; then
    ROS2_SETUP="/opt/ros/${ROS2_DISTRO}/setup.bash"
    if [[ ! -f "${ROS2_SETUP}" ]]; then
        echo "错误：未找到 ${ROS2_SETUP}，请安装 ROS2 ${ROS2_DISTRO}。" >&2
        exit 1
    fi

    reset_ros_environment
    source "${ROS2_SETUP}"

    if ! command -v colcon >/dev/null 2>&1; then
        echo "错误：未找到 colcon，请安装 python3-colcon-common-extensions。" >&2
        exit 1
    fi

    echo "[ROS2] 构建消息镜像：${ROS2_WS}"
    cd "${ROS2_WS}"
    # colcon 本身没有 -j 参数（--parallel-workers 控制包级并行，此处仅一个包），
    # 通过 MAKEFLAGS 限制其内部 make 的编译并行度。
    if [[ -n "${JOBS}" ]]; then
        MAKEFLAGS="-j${JOBS}" colcon build \
            --symlink-install \
            --packages-select dcl_slam_msgs \
            --cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
    else
        colcon build \
            --symlink-install \
            --packages-select dcl_slam_msgs \
            --cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
    fi
fi

echo "DCL-SLAM 构建完成。"
