#!/usr/bin/env bash
# Copyright 2026 lwb <wb.lv@qq.com>
# SPDX-License-Identifier: Apache-2.0

# =============================================================================
# Build the customized ROS 1 <-> ROS 2 dynamic bridge.
# 构建项目定制的 ROS 1 <-> ROS 2 动态桥。
#
# ros1_bridge generates conversion factories at build time from the ROS 1 and
# ROS 2 message types currently visible. Run ./scripts/build.sh first, then use
# this script for the initial bridge build and after changing message files,
# mapping_rules.yaml, or either ROS distribution. Algorithm, launch-file, and
# runtime-parameter changes do not require rebuilding the bridge.
# ros1_bridge 会在编译时根据当前可见的 ROS 1 和 ROS 2 消息类型生成转换工厂。
# 请先运行 ./scripts/build.sh；首次构建 bridge，以及消息文件、
# mapping_rules.yaml 或 ROS 发行版发生变化后，再运行本脚本。仅修改算法、
# launch 文件或运行参数时，无需重新构建 bridge。
#
# Prerequisites:
#   - DCL ROS 1 and ROS 2 messages have been built with ./scripts/build.sh.
#   - The customized ros1_bridge is next to dcl_slam, or ROS1_BRIDGE_ROOT points
#     to another checkout.
#   - ROS 1, ROS 2, and colcon are installed.
# 前置条件：
#   - 已通过 ./scripts/build.sh 构建 DCL 的 ROS 1 和 ROS 2 消息。
#   - 定制版 ros1_bridge 位于 dcl_slam 同级目录，或 ROS1_BRIDGE_ROOT 指向
#     其他检出目录。
#   - 已安装 ROS 1、ROS 2 和 colcon。
#
# Usage: ./scripts/build_ros1_bridge.sh [options]
# 用法：./scripts/build_ros1_bridge.sh [选项]
# Examples:
# 示例：
#        ./scripts/build_ros1_bridge.sh
#        ./scripts/build_ros1_bridge.sh -j 4
#        ROS1_DISTRO=noetic ROS2_DISTRO=foxy ./scripts/build_ros1_bridge.sh
#
# ROS setup files may reference undefined variables, so nounset (-u) is disabled.
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
# Match run_robot.sh: use the sibling checkout unless the path is overridden.
# 与 run_robot.sh 保持一致：默认使用同级目录中的 bridge，除非通过环境变量覆盖。
ROS1_BRIDGE_ROOT="${ROS1_BRIDGE_ROOT:-$(cd -- "${DCL_ROOT}/.." && pwd)/ros1_bridge}"

# ---------------- Defaults / 默认值 ----------------
ROS1_DISTRO="${ROS1_DISTRO:-noetic}"
ROS2_DISTRO="${ROS2_DISTRO:-foxy}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS=""

usage() {
    cat <<'EOF'
Usage: ./scripts/build_ros1_bridge.sh [options]

Build the DCL ROS 1 and ROS 2 messages with ./scripts/build.sh first.
The customized ros1_bridge must be next to dcl_slam unless ROS1_BRIDGE_ROOT
points to another checkout. This script loads both environments and runs colcon.

Options:
  -j, --jobs N  Number of parallel build jobs (default: tool-defined)
  -h, --help    Show this help
EOF
}

while [[ "$#" -gt 0 ]]; do
    case "$1" in
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

# ---------------- Prerequisite checks / 前置检查 ----------------
# Fail before the build if a required environment is missing.
# 若缺少必需的环境文件，则在构建开始前立即退出。
ROS1_SETUP="/opt/ros/${ROS1_DISTRO}/setup.bash"
ROS2_SETUP="/opt/ros/${ROS2_DISTRO}/setup.bash"
ROS1_WS_SETUP="${ROS1_WS}/devel/setup.bash"
ROS2_WS_SETUP="${ROS2_WS}/install/local_setup.bash"
BRIDGE_SRC="${ROS1_BRIDGE_ROOT}/src/ros1_bridge"

for setup_path in "${ROS1_SETUP}" "${ROS2_SETUP}" \
    "${ROS1_WS_SETUP}" "${ROS2_WS_SETUP}"; do
    if [[ ! -f "${setup_path}" ]]; then
        echo "Error: required file not found: ${setup_path}" >&2
        case "${setup_path}" in
            *ros1_ws/devel/setup.bash|*ros2_ws/install/local_setup.bash)
                echo "Run ./scripts/build.sh first to generate the DCL ROS 1/ROS 2 messages." >&2
                ;;
        esac
        exit 1
    fi
done

if [[ ! -d "${BRIDGE_SRC}" ]]; then
    echo "Error: ros1_bridge source directory not found: ${BRIDGE_SRC}" >&2
    echo "Clone the customized ros1_bridge next to dcl_slam or set ROS1_BRIDGE_ROOT." >&2
    exit 1
fi
if ! command -v colcon >/dev/null 2>&1; then
    echo "Error: colcon not found; install python3-colcon-common-extensions." >&2
    exit 1
fi

# ---------------- Clean ROS environment / 清理 ROS 环境 ----------------
# Clear inherited ROS and colcon state before loading the required environments.
# These changes affect only this script process, not the caller's shell.
# 在加载所需环境前清除继承的 ROS 和 colcon 状态；这些修改只影响当前脚本
# 进程，不会改变调用者的 shell 环境。
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ROOT
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH

# Load ROS 1 and overlay the DCL ROS 1 workspace.
# 加载 ROS 1 并叠加 DCL ROS 1 工作区。
# shellcheck disable=SC1090
source "${ROS1_SETUP}"
# shellcheck disable=SC1090
source "${ROS1_WS_SETUP}" --extend

# Add ROS 2 and its DCL message overlay after clearing the conflicting
# ROS_DISTRO value set by ROS 1.
# 清除 ROS 1 设置的冲突 ROS_DISTRO 值后，再加载 ROS 2 及其 DCL 消息工作区。
unset ROS_DISTRO
# shellcheck disable=SC1090
source "${ROS2_SETUP}"
# shellcheck disable=SC1090
source "${ROS2_WS_SETUP}"

echo "[Bridge] Building ros1_bridge: ${ROS1_BRIDGE_ROOT}"
cd "${ROS1_BRIDGE_ROOT}"
# Force CMake to regenerate conversion factories. Only one package is selected,
# so MAKEFLAGS controls its internal make parallelism.
# 强制 CMake 重新生成转换工厂。由于只构建一个包，使用 MAKEFLAGS 控制包内
# make 的并行度。
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

echo "ros1_bridge build completed."
