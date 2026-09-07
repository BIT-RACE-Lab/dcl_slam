# 本地 GTSAM 源码

为避免 `gtsam_catkin` 在构建时访问外部网络，请将 GTSAM 源码置于本目录：

```text
third_party/gtsam/
```

必须使用以下固定提交，以保证 `gtsam_catkin` 中的补丁与项目 API 兼容：

```text
687ae3d2511b9c296af08ec2f2e717b0627a8d68
```

源码还必须已应用 `../../use_catkinized_metis.patch` 与
`../../fix_warnings.patch`。本工作区中的 `gtsam` 目录已处于该补丁状态；
不要在构建时重复应用补丁。

可在能联网的机器下载：

```bash
git clone https://bitbucket.org/gtborg/gtsam.git gtsam
cd gtsam
git checkout 687ae3d2511b9c296af08ec2f2e717b0627a8d68
patch -p1 < ../../use_catkinized_metis.patch
patch -p1 < ../../fix_warnings.patch
```

复制 `gtsam` 目录到本目录后，删除其内部 `.git` 目录，再将源码作为本仓库的一部分提交。这样新机器克隆本仓库后无需再下载 GTSAM。

如需将源码放在其他本地位置，可在构建前指定：

```bash
catkin config --cmake-args -DGTSAM_SOURCE_DIR=/absolute/path/to/gtsam
```
