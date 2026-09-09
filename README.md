# DCL-SLAM

<p align="center">
  <strong>English</strong> | <a href="README.zh-CN.md">简体中文</a>
</p>

DCL-SLAM is a distributed collaborative LiDAR SLAM project that combines a ROS 1 algorithm workspace with a ROS 2 message-mirror workspace. The ROS 1 workspace contains DCL-SLAM, DCL-FAST-LIO, Livox ROS Driver 2, and the source code of build dependencies such as GTSAM, glog, and libnabo. The ROS 2 workspace provides mirrored custom messages for inter-robot communication over DDS.

## Repository Layout

| Path | Description |
| --- | --- |
| `ros1_ws/src/DCL-SLAM` | Distributed collaborative LiDAR SLAM, loop closure, and visualization nodes |
| `ros1_ws/src/DCL-FAST-LIO` | FAST-LIO front-end integration |
| `ros1_ws/src/livox_ros_driver2` | Livox LiDAR ROS driver |
| `ros1_ws/src/distributed_mapper` | Distributed pose-graph optimization and consistency filtering |
| `ros1_ws/src/*_catkin`, `ros1_ws/src/libnabo` | Bundled third-party sources required to build the ROS 1 workspace |
| `ros2_ws/src/dcl_slam_msgs` | ROS 2 mirrors of DCL custom messages and `ros1_bridge` mapping rules |
| `config/fastdds_wifi.xml` | Fast DDS interface allowlist and static peer configuration for two physical robots |
| `scripts/build.sh` | Builds the ROS 1 algorithm workspace and ROS 2 message mirrors |
| `scripts/build_ros1_bridge.sh` | Builds the customized `ros1_bridge` after message or mapping changes |
| `scripts/run_robot.sh` | Starts the local ROS master, DDS bridge, Livox driver, and DCL-SLAM for one robot |

## Requirements

- Ubuntu 20.04
- ROS 1 Noetic
- ROS 2 Foxy and `colcon` when DDS transport between robots is required
- The project-specific [ros1_bridge](https://github.com/BIT-Jiang-Group/ros1_bridge) when ROS 1/ROS 2 bridging is required; clone it next to this repository
- CMake, Git, and `catkin_tools`
- Boost, PCL, Eigen, OpenCV, and Python development libraries
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2), installed separately according to its upstream instructions. The bundled ROS 1 driver looks for `liblivox_lidar_sdk_static.a` in `/usr/local/lib`.

Install the commonly required build packages:

```bash
sudo apt update
sudo apt install cmake git python3-catkin-tools libboost-all-dev \
  libpcl-dev libeigen3-dev libopencv-dev python3-dev
```

> Install [ROS Noetic](https://wiki.ros.org/noetic/Installation) and [ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation.html) using the instructions for Ubuntu 20.04. Livox-SDK2 is not bundled with this repository and must be installed on every computer connected to a MID360.

## Clone and Build

### DCL-SLAM

```bash
git clone https://github.com/BIT-Jiang-Group/dcl_slam.git
cd dcl_slam
./scripts/build.sh
```

By default, the script builds:

- ROS 1 packages: `dcl_slam`, `dcl_fast_lio`, and `livox_ros_driver2`
- ROS 2 package: `dcl_slam_msgs`

The ROS 1 workspace uses a merged `devel` space and a Release build. The build tools choose their default parallelism unless `-j` is specified:

```bash
./scripts/build.sh -j 4
./scripts/build.sh --ros1-only -j 2
```

### Customized ros1_bridge

The standard system installation of `ros1_bridge` does not include conversions for DCL custom messages. It also lacks the parameterized topic list, ROS 2 `transient_local` to ROS 1 latch handling, and bidirectional bridge-loop suppression used by this project. Use the customized repository and place it next to `dcl_slam`:

```text
├── dcl_slam/
└── ros1_bridge/
```

Build it after DCL-SLAM so that both ROS 1 and ROS 2 custom messages are available:

```bash
git clone https://github.com/BIT-Jiang-Group/ros1_bridge.git

cd dcl_slam
./scripts/build_ros1_bridge.sh
```

The script loads both message environments and runs a Release `colcon build` with `--symlink-install` and `--cmake-force-configure`. Limit build parallelism when needed:

```bash
./scripts/build_ros1_bridge.sh -j 4
```

By default, the script looks for `ros1_bridge` next to this repository. Override the location with `ROS1_BRIDGE_ROOT`:

```bash
ROS1_BRIDGE_ROOT=/path/to/ros1_bridge ./scripts/build_ros1_bridge.sh
```

Rebuild the bridge after the first build, after changing ROS 1 or ROS 2 `.msg` files, after changing `mapping_rules.yaml`, or after switching ROS distributions. Changes limited to DCL algorithms, launch files, or runtime parameters do not require a bridge rebuild.

Verify that DCL message conversions were generated:

```bash
source ../ros1_bridge/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep dcl_slam
```

## Two-Robot Network and MID360 Setup

The two robot computers must be able to reach each other over Wi-Fi. Each MID360 communicates with its robot computer over a wired interface. The default configuration uses the following example addresses:

| Robot | MID360 IP | Wired host IP | Wi-Fi host IP |
| :---: | :---: | :---: | :---: |
| a | `192.168.2.167` | `192.168.2.166` | `192.168.31.11` |
| b | `192.168.2.167` | `192.168.2.166` | `192.168.31.12` |

> The MID360 and wired host addresses may be identical on both robots only when each sensor is directly connected to its own computer on an isolated wired network. Assign unique device and host addresses if both sensors share the same wired network.

Run an independent ROS 1 master on each robot and use the ROS 1/ROS 2 bridge only for inter-robot DCL messages. The included two-robot bridge configuration keeps local point clouds, IMU data, TF, maps, and localization output off the inter-robot network.

The launch script exports the required ROS and DDS network variables. If you change the Wi-Fi addresses, update both the command-line arguments and `config/fastdds_wifi.xml`.

## Run on Physical Robots

### One-command launch

After building DCL-SLAM and `ros1_bridge` and configuring the MID360 and `fastdds_wifi.xml`, run one command on each robot. The script loads the ROS 1 and ROS 2 environments, starts or reuses a local `roscore`, loads the inter-robot topic list, and starts `parameter_bridge`, the Livox driver, and DCL-SLAM in order.

Robot a:

```bash
cd ~/mtare/dcl_slam
./scripts/run_robot.sh a
```

Robot b:

```bash
cd ~/mtare/dcl_slam
./scripts/run_robot.sh b
```

The default Wi-Fi addresses are `192.168.31.11` and `192.168.31.12`, and the default ROS 2 domain ID is `30`. Override them when required:

```bash
./scripts/run_robot.sh a --ros-ip 192.168.31.21 --domain-id 30
./scripts/run_robot.sh b --ros-ip 192.168.31.22 --domain-id 30
```

Update the interface allowlist and static peers in `config/fastdds_wifi.xml` whenever these addresses change. Press Ctrl-C to stop the bridge, Livox driver, and DCL-SLAM processes started by the script. If a local `roscore` is already available, the script reuses it and leaves it running when the script exits.

Use a bridge checkout in another location by setting `ROS1_BRIDGE_ROOT`:

```bash
ROS1_BRIDGE_ROOT=/path/to/ros1_bridge ./scripts/run_robot.sh a
```

### Component-level debugging

The combined ROS 1 and ROS 2 environment is sensitive to setup order. Use `scripts/run_robot.sh` as the reference for environment setup and process order when launching components in separate terminals. If component-level launches are needed frequently, add explicit script options such as `--skip-bridge` or `--skip-livox` instead of commenting out sections of the script.

## Citation and Acknowledgements

This repository builds on DCL-SLAM, FAST-LIO2, LIO-SAM, DOOR-SLAM, and other open-source projects. See [`ros1_ws/src/DCL-SLAM/README.md`](ros1_ws/src/DCL-SLAM/README.md) for the DCL-SLAM paper citation and upstream acknowledgements. Each bundled component remains subject to the license and attribution files in its own source directory; see [NOTICE](NOTICE) for repository-level attribution.

## License

The repository-level additions are available under the [Apache License 2.0](LICENSE). Bundled third-party components retain their original licenses; consult the license files in their respective source directories.
