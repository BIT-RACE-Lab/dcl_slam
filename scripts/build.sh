#!/usr/bin/env bash
# Copyright 2026 lwb <wb.lv@qq.com>
# SPDX-License-Identifier: Apache-2.0

# ROS setup files may reference undefined variables, so nounset (-u) is disabled.
# ROS setup 脚本可能引用未定义变量，因此不启用 nounset（-u）。
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
Usage: ./scripts/build.sh [options]

Options may be combined, for example: ./scripts/build.sh --ros1-only -j 4
By default, the script builds the ROS 1 workspace and ROS 2 message mirrors.

Options:
  --ros1-only  Build only the ROS 1 workspace
  --ros2-only  Build only the ROS 2 message mirrors
  -j, --jobs N Number of parallel build jobs (default: tool-defined)
  -h, --help   Show this help
EOF
}

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --ros1-only) build_ros2=false ;;
        --ros2-only) build_ros1=false ;;
        -j|--jobs)
            if [[ "$#" -lt 2 ]]; then
                echo "Error: option $1 requires a numeric argument" >&2
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

if [[ -n "${JOBS}" ]] && ! [[ "${JOBS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: -j must be a positive integer; got: ${JOBS}" >&2
    exit 2
fi

if ! "${build_ros1}" && ! "${build_ros2}"; then
    echo "Error: --ros1-only and --ros2-only cannot be used together" >&2
    usage >&2
    exit 2
fi

reset_ros_environment() {
    # These changes affect only this script process, not the caller's shell.
    # 这些修改只影响当前脚本进程，不会改变调用者的 shell 环境。
    unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ROOT
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
    unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
}

if "${build_ros1}"; then
    ROS1_SETUP="/opt/ros/${ROS1_DISTRO}/setup.bash"
    if [[ ! -f "${ROS1_SETUP}" ]]; then
        echo "Error: ${ROS1_SETUP} not found; install ROS 1 ${ROS1_DISTRO}." >&2
        exit 1
    fi

    reset_ros_environment
    source "${ROS1_SETUP}"

    if ! command -v catkin >/dev/null 2>&1; then
        echo "Error: catkin not found; install python3-catkin-tools." >&2
        exit 1
    fi

    echo "[ROS1] Building workspace: ${ROS1_WS}"
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
        echo "Error: ${ROS2_SETUP} not found; install ROS 2 ${ROS2_DISTRO}." >&2
        exit 1
    fi

    reset_ros_environment
    source "${ROS2_SETUP}"

    if ! command -v colcon >/dev/null 2>&1; then
        echo "Error: colcon not found; install python3-colcon-common-extensions." >&2
        exit 1
    fi

    echo "[ROS2] Building message mirrors: ${ROS2_WS}"
    cd "${ROS2_WS}"
    # Only one package is selected, so use MAKEFLAGS to control its internal
    # make parallelism instead of colcon's package-level --parallel-workers.
    # 此处只构建一个包，因此使用 MAKEFLAGS 控制包内 make 的并行度，而不使用
    # colcon 在包级生效的 --parallel-workers。
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

echo "DCL-SLAM build completed."
