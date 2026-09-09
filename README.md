# DCL-SLAM

同时包含 ROS1 算法工作区和 ROS2 消息镜像工作区的分布式协同激光 SLAM 项目。ROS1 工作区包含 DCL-SLAM、DCL-FAST-LIO、Livox ROS Driver 2，以及 GTSAM、glog、libnabo 等构建依赖的完整源码；ROS2 工作区提供跨车 DDS 通信所需的消息镜像。

## 目录说明

| 路径 | 说明 |
| --- | --- |
| `ros1_ws/src/DCL-SLAM` | 分布式协同 LiDAR SLAM、回环与可视化相关节点 |
| `ros1_ws/src/DCL-FAST-LIO` | FAST-LIO 前端适配包 |
| `ros1_ws/src/livox_ros_driver2` | Livox 激光雷达 ROS 驱动 |
| `ros1_ws/src/distributed_mapper` | 分布式图优化与一致性筛选 |
| `ros1_ws/src/*_catkin`、`ros1_ws/src/libnabo` | ROS1 构建所需的第三方依赖源码 |
| `ros2_ws/src/dcl_slam_msgs` | DCL 自定义消息的 ROS2 镜像及 ros1_bridge 映射规则 |
| `config/fastdds_wifi.xml` | 实车双机 Fast DDS 无线网卡白名单与固定节点发现配置 |
| `scripts/build.sh` | 统一构建 ROS1 算法工作区和 ROS2 消息镜像 |
| `scripts/build_ros1_bridge.sh` | 编译定制版 ros1_bridge（自定义消息或映射规则变更时执行） |
| `scripts/run_robot.sh` | 一键启动单车 ROS master、DDS bridge、Livox 驱动和 DCL-SLAM |

## 环境要求

- Ubuntu 20.04
- ROS 1 Noetic
- ROS 2 Foxy、`colcon`（只在使用 DDS 跨车传输时需要）
- [ros1_bridge](https://github.com/BIT-Jiang-Group/ros1_bridge) 项目定制版（只在使用 ROS1/ROS2 bridge 时需要），请将它克隆到 `dcl_slam` 的同级目录
- CMake、Git、`catkin_tools`
- Boost、PCL、Eigen、OpenCV、Python 开发库
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)：需按官方说明单独安装。当前 ROS1 驱动会从 `/usr/local/lib` 查找 `liblivox_lidar_sdk_static.a`。

安装常用构建依赖：

```bash
sudo apt update
sudo apt install cmake git python3-catkin-tools libboost-all-dev \
  libpcl-dev libeigen3-dev libopencv-dev python3-dev
```

> ROS 的安装方式与系统版本相关，请先完成 [ROS Melodic](https://wiki.ros.org/melodic/Installation) 或 [ROS Noetic](https://wiki.ros.org/noetic/Installation) 安装。Livox-SDK2 不包含在本仓库内，必须在每台连接 MID360 的机器上单独安装。

## 获取与构建

### 本体

```bash
git clone git@github.com:BIT-Jiang-Group/dcl_slam.git
cd dcl_slam
./scripts/build.sh
```

脚本默认依次构建以下内容：

- ROS1：`dcl_slam`、`dcl_fast_lio`、`livox_ros_driver2`；
- ROS2：`dcl_slam_msgs`。

ROS1 工作区使用 Release 模式和合并式 `devel` 空间。构建默认使用工具自带的并行度；如需限制或指定并行编译任务数（例如内存受限的机器），可在命令后加 `-j <数量>`：

```bash
./scripts/build.sh -j 4
./scripts/build.sh --ros1-only -j 2
```

### ros1_bridge 依赖

系统安装的原版 `ros1_bridge` 不包含 DCL 自定义消息转换，也不包含本项目所需的参数化话题列表、`transient_local`/ROS1 latch 和双向回环抑制。请使用项目定制仓库，并让它与 `dcl_slam` 保持同级目录：

```text
├── dcl_slam/
└── ros1_bridge/
```

首次获取与构建（**要求`dcl_slam`以编译完成**）：

```bash
git clone https://github.com/BIT-Jiang-Group/ros1_bridge.git

cd dcl_slam

# ros1_bridge 必须在两边自定义消息环境都可见时编译
./scripts/build_ros1_bridge.sh
```

`build_ros1_bridge.sh` 会自动加载 ROS1/ROS2 环境并执行 `colcon build`（`--symlink-install`、`--cmake-force-configure`、Release）。如需限制并行编译任务数，可加 `-j <数量>`：

```bash
./scripts/build_ros1_bridge.sh -j 4
```

ros1_bridge 默认从 `dcl_slam` 的同级目录查找，也可以通过 `ROS1_BRIDGE_ROOT` 指定其它位置：

```bash
ROS1_BRIDGE_ROOT=/path/to/ros1_bridge ./scripts/build_ros1_bridge.sh
```

只有在首次构建、修改 ROS1/ROS2 `.msg`、修改 `mapping_rules.yaml` 或更换 ROS 版本后，才需要重新编译 `ros1_bridge`。普通 DCL 算法、launch 或运行参数修改不需要重编 bridge。

构建完成后检查 DCL 转换是否已经生成：

```bash
source ../ros1_bridge/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep dcl_slam
```

## 双车网络与 MID360 配置

两台车须能通过无线网络互相访问；每台车的 MID360 使用本机有线网卡通信。以下为示例地址：

| 车辆 | MID360 IP | 本机有线网卡 IP | 本机无线网卡 IP |
| :---: | :---: | :---: | :---: |
| 1 | `192.168.2.167` | `192.168.2.166` | `192.168.31.11` |
| 2 | `192.168.2.167` | `192.168.2.166` | `192.168.31.12` |

> 若两台 MID360 分别直连各自的车辆，它们处于独立的有线网段时可使用相同地址；若接入同一有线网络，必须改用不同的设备和主机 IP。

推荐每辆车运行独立 ROS1 Master，并通过 ROS1/ROS2 bridge 只转发 DCL 跨车消息。DCL 自带双车 bridge 参数配置；本车的点云、IMU、TF、地图和定位输出不会进入跨车网络。

> 网络相关环境变量已写入启动脚本中

## 实机运行

### 一键启动（推荐）

完成 DCL-SLAM 和 `ros1_bridge` 构建、MID360 以及 `fastdds_wifi.xml` 配置后，每辆车只需运行一个脚本。脚本会自动加载 ROS1/ROS2 环境，启动或复用本机 `roscore`，加载跨车话题列表，并按顺序启动 `parameter_bridge`、Livox 驱动和 DCL-SLAM。

**1 号车：**

```bash
cd ~/mtare/dcl_slam
./scripts/run_robot.sh a
```

**2 号车：**

```bash
cd ~/mtare/dcl_slam
./scripts/run_robot.sh b
```

默认无线地址分别为 `192.168.31.11` 和 `192.168.31.12`，ROS 2 Domain ID 为 `30`。地址不同时可在命令行覆盖：

```bash
./scripts/run_robot.sh a --ros-ip 192.168.31.21 --domain-id 30
./scripts/run_robot.sh b --ros-ip 192.168.31.22 --domain-id 30
```

此时也必须同步修改 `config/fastdds_wifi.xml` 中的网卡白名单和固定节点地址。按 `Ctrl-C` 会关闭脚本启动的 bridge、Livox 和 DCL-SLAM；如果脚本检测到 `roscore` 已经运行，则复用它且退出时不会关闭该 ROS master。`ros1_bridge` 默认从 `dcl_slam` 的同级目录查找，也可以通过 `ROS1_BRIDGE_ROOT` 指定其他位置：

```bash
ROS1_BRIDGE_ROOT=/path/to/ros1_bridge ./scripts/run_robot.sh a
```

### 分终端启动

由于项目同时包含`ros1`和`ros2`，环境变量较为复杂，建议通过对脚本的部分注释实现单独功能调试

## 致谢

DCL-SLAM 基于 DCL-SLAM、FAST-LIO2、LIO-SAM 和 DOOR-SLAM 等开源工作开展；各组件的许可证与引用信息以其源码目录内的说明文件为准。
