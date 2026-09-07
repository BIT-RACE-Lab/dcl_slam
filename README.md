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
| `scripts/build.sh` | 统一构建 ROS1 算法工作区和 ROS2 消息镜像 |
| `ros1_ws/{build,devel,logs}` | catkin 自动生成内容，不纳入 Git |
| `ros2_ws/{build,install,log}` | colcon 自动生成内容，不纳入 Git |

## 环境要求

- Ubuntu 20.04
- ROS 1 Noetic
- ROS 2 Foxy、`colcon`（只在使用 DDS 跨车传输时需要）
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

## 双车网络与 MID360 配置

两台车须能通过无线网络互相访问；每台车的 MID360 使用本机有线网卡通信。以下为示例地址：

| 车辆 | MID360 IP | 本机有线网卡 IP | 本机无线网卡 IP |
| :---: | :---: | :---: | :---: |
| 1（中心节点） | `192.168.2.167` | `192.168.2.166` | `192.168.31.11` |
| 2 | `192.168.2.167` | `192.168.2.166` | `192.168.31.12` |

> 若两台 MID360 分别直连各自的车辆，它们处于独立的有线网段时可使用相同地址；若接入同一有线网络，必须改用不同的设备和主机 IP。

推荐每辆车运行独立 ROS1 Master，并通过 ROS1/ROS2 bridge 只转发 DCL 跨车消息。DCL 自带双车 bridge 参数配置；本车的点云、IMU、TF、地图和定位输出不会进入跨车网络。

**1 号车：**

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.31.11
export ROS_DOMAIN_ID=30
unset ROS_HOSTNAME
```

**2 号车：**

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.31.12
export ROS_DOMAIN_ID=30
unset ROS_HOSTNAME
```

在每台车上修改 `ros1_ws/src/livox_ros_driver2/config/MID360_config.json`：

- 将 `cmd_data_ip`、`push_msg_ip`、`point_data_ip`、`imu_data_ip` 全部设为**本机有线网卡 IP**；
- 将 `lidar_configs[0].ip` 设为该车连接的 **MID360 IP**；
- 端口保持默认值，除非网络环境已有端口冲突。

配置完成后，请确认两台车能互相 ping 通无线网卡 IP；再确认每台车能 ping 通本机连接的 MID360 IP。

## 实机运行

以下步骤以两辆车、FAST-LIO 前端为例。每个新终端都需先加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/mtare/dcl_slam/ros1_ws/devel/setup.bash --extend
```

先在每辆车上启动本机 ROS master。bridge 必须先于 DCL-SLAM 启动，避免错过启动阶段的全局描述子：

```bash
roscore
```

另开 ROS1 终端加载 DCL 自带的 14 个跨车话题配置；该 launch 加载完参数后自动退出属于正常现象：

```bash
source /opt/ros/noetic/setup.bash
source ~/mtare/dcl_slam/ros1_ws/devel/setup.bash --extend
roslaunch dcl_slam bridge_dcl_two_robots.launch
```

再开一个 bridge 终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/mtare/dcl_slam/ros1_ws/devel/setup.bash --extend
source /opt/ros/foxy/setup.bash
source ~/mtare/dcl_slam/ros2_ws/install/local_setup.bash
source ~/mtare/ros1_bridge/install/local_setup.bash
ros2 run ros1_bridge parameter_bridge
```

然后分别在各车启动 Livox 驱动和本车 LIO 前端。`robotPrefix` 必须与车辆对应，且不可重复。

**1 号车（前缀 `a`）：**

```bash
roslaunch livox_ros_driver2 msg_MID360.launch robotPrefix:=a
roslaunch dcl_slam single_ugv.launch robotPrefix:=a number_of_robots:=2
```

**2 号车（前缀 `b`）：**

```bash
roslaunch livox_ros_driver2 msg_MID360.launch robotPrefix:=b
roslaunch dcl_slam single_ugv.launch robotPrefix:=b number_of_robots:=2
```

`single_ugv.launch` 默认使用 `lioType:=2`，即 FAST-LIO；它启动的是当前车辆命名空间下的 LIO 前端。

### 独立验证 DDS 跨车传输

两车启动 bridge 和 DCL 后，在车 a 检查车 b 的描述子：

```bash
ros2 topic list | grep distributedMapping
rostopic info /b/distributedMapping/globalDescriptors
rostopic echo -n 1 /b/distributedMapping/globalDescriptors
```

预期 ROS1 信息中 `/ros_bridge` 是发布者、`/a/laserMapping` 是订阅者；DCL 日志应出现 `Received global descriptor: robot=1`。车 b 反向检查 `/a/distributedMapping/globalDescriptors`。以下本地高带宽或控制话题不应出现在 ROS2 topic 列表中：

```text
/a|b/livox/lidar
/a|b/livox/imu
/a|b/cloud_registered_global
/a|b/odometry_global
/cmd_vel
```

## 致谢

DCL-SLAM 基于 DCL-SLAM、FAST-LIO2、LIO-SAM 和 DOOR-SLAM 等开源工作开展；各组件的许可证与引用信息以其源码目录内的说明文件为准。
