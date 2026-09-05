# cslam_ws

基于 ROS 1/catkin 的分布式协同激光 SLAM 工作区。仓库包含 DCL-SLAM、DCL-FAST-LIO、Livox ROS Driver 2，以及 GTSAM、glog、libnabo 等构建依赖的完整源码；克隆一次即可获得所有工作区内的软件包。

## 目录说明

| 路径 | 说明 |
| --- | --- |
| `src/DCL-SLAM` | 分布式协同 LiDAR SLAM、回环与可视化相关节点 |
| `src/DCL-FAST-LIO` | FAST-LIO 前端适配包 |
| `src/livox_ros_driver2` | Livox 激光雷达 ROS 驱动 |
| `src/distributed_mapper` | 分布式图优化与一致性筛选 |
| `src/*_catkin`、`src/libnabo` | 构建所需的第三方依赖源码 |
| `build/`、`devel/`、`logs/` | catkin 自动生成内容，不纳入 Git |

## 环境要求

- Ubuntu 20.04
- ROS 1 Noetic
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

source /opt/ros/noetic/setup.bash
catkin init
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1
catkin build
source devel/setup.bash
```

如只需构建主功能包：

```bash
catkin build dcl_slam dcl_fast_lio
source devel/setup.bash
```

工作区默认使用 Release 模式、合并式 `devel` 空间和 16 个并行任务。可通过 `catkin config -j <数量>` 按机器资源调整并行度。

## 双车网络与 MID360 配置

两台车须能通过无线网络互相访问；每台车的 MID360 使用本机有线网卡通信。以下为示例地址：

| 车辆 | MID360 IP | 本机有线网卡 IP | 本机无线网卡 IP |
| :---: | :---: | :---: | :---: |
| 1（中心节点） | `192.168.2.167` | `192.168.2.166` | `192.168.31.11` |
| 2 | `192.168.2.167` | `192.168.2.166` | `192.168.31.12` |

> 若两台 MID360 分别直连各自的车辆，它们处于独立的有线网段时可使用相同地址；若接入同一有线网络，必须改用不同的设备和主机 IP。

以 1 号车为 ROS 中心节点，在两台车的 `~/.bashrc` 中分别添加以下配置，之后重新打开终端或执行 `source ~/.bashrc`。

**1 号车：**

```bash
export ROS_MASTER_URI=http://192.168.31.11:11311
export ROS_IP=192.168.31.11
unset ROS_HOSTNAME
```

**2 号车：**

```bash
export ROS_MASTER_URI=http://192.168.31.11:11311
export ROS_IP=192.168.31.12
unset ROS_HOSTNAME
```

在每台车上修改 `src/livox_ros_driver2/config/MID360_config.json`：

- 将 `cmd_data_ip`、`push_msg_ip`、`point_data_ip`、`imu_data_ip` 全部设为**本机有线网卡 IP**；
- 将 `lidar_configs[0].ip` 设为该车连接的 **MID360 IP**；
- 端口保持默认值，除非网络环境已有端口冲突。

配置完成后，请确认两台车能互相 ping 通无线网卡 IP；再确认每台车能 ping 通本机连接的 MID360 IP。

## 实机运行

以下步骤以两辆车、FAST-LIO 前端为例。每个新终端都需先加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
```

先在 1 号车确保 ROS master 已启动：

```bash
roscore
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

## 致谢

DCL-SLAM 基于 DCL-SLAM、FAST-LIO2、LIO-SAM 和 DOOR-SLAM 等开源工作开展；各组件的许可证与引用信息以其源码目录内的说明文件为准。
