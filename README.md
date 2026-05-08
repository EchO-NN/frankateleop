# Franka Teleop

这是一个基于 GELLO/teleop + Polymetis 的 Franka FR3 遥操作项目。当前运行链路不依赖 ROS2，也不依赖 `franky`；真实机械臂控制走的是：

```text
teleop/GELLO -> Polymetis RobotInterface/GripperInterface -> franka C++ client -> libfranka -> FR3
```

## 目录结构

```text
frankateleop/
  environment.mamba.yml      # mamba/conda 环境定义
  MAMBA_SETUP.md             # 更详细的环境构建说明
  left_franka/               # 左臂启动脚本
  right_franka/              # 右臂启动脚本
  polymetis/                 # Polymetis 控制层，包含 Franka C++ client
  teleop/                    # GELLO/teleop Python 逻辑
  scripts/                   # 通用辅助脚本
```

## 环境构建

推荐使用 mamba。环境名保持为 `polymetis`，因为启动脚本里写的是 `conda activate polymetis`。

```bash
cd frankateleop
mamba env create -f environment.mamba.yml
mamba activate polymetis
```

如果环境已经存在：

```bash
mamba env update -n polymetis -f environment.mamba.yml --prune
mamba activate polymetis
```

安装本地 Python 包：

```bash
pip install -e teleop
pip install -e teleop/third_party/DynamixelSDK/python
pip install -e polymetis/polymetis
```

更多细节见 [MAMBA_SETUP.md](MAMBA_SETUP.md)。

## 编译 Polymetis / libfranka

真实 FR3 需要 Polymetis 的 C++ client 和 `libfranka`。
`libfranka` 应该放在 Polymetis 默认查找的位置：

```text
polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka
```

如果这个目录不存在，可以直接克隆进去：

```bash
git clone https://github.com/frankaemika/libfranka \
  polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka
```

```bash
cd frankateleop/polymetis
./scripts/build_libfranka.sh

cd polymetis  #frankateleop/polymetis/polimetis
rm -rf build
mkdir build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_FRANKA=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_ALLEGRO=OFF
make -j"$(nproc)"
```

如果 `libfranka` 放在项目外部，编译时指定：

```bash
-DFranka_DIR=/absolute/path/to/libfranka/build
```

## 启动左臂

建议 4 个终端按顺序启动：

```bash
cd frankateleop/left_franka
./1_launch_robot.sh
```

```bash
cd frankateleop/left_franka
./2_launch_gripper.sh
```

```bash
cd frankateleop/left_franka
./3_launch_node.sh
```

```bash
cd frankateleop/left_franka
./4_run_env.sh
```

右臂同理使用 `right_franka/` 下的脚本。

## 运行前检查

确认 Python 包可导入：

```bash
conda activate polymetis
python -c "import polymetis, teleop; print('ok')"
```

确认 Franka client 的动态库都能找到：

```bash
ldd polymetis/polymetis/build/franka_panda_client | grep "not found" || true
ldd polymetis/polymetis/build/franka_hand_client | grep "not found" || true
```

真实机器人建议使用实时内核：

```bash
uname -a
cat /sys/kernel/realtime
```

期望 `uname -a` 里包含 `PREEMPT_RT`，并且 `/sys/kernel/realtime` 输出 `1`。

## 网络与端口

常用配置：

- 控制机连接 Franka Control 的网口：`172.16.0.1/24`
- 默认/右臂机器人 IP：`172.16.0.2`
- 左臂机器人 IP：`172.16.0.3`
- 左臂 robot server：`50052`
- 左臂 gripper server：`50054`
- 左臂 teleop server：`6002`

这些值主要在 `polymetis/polymetis/python/polymetis/conf/` 和 `left_franka/`、`right_franka/` 的启动脚本中配置。

## 常见问题

如果 `launch_robot.py` 或 `launch_gripper.py` 找不到：

```bash
conda activate polymetis
pip install -e polymetis/polymetis
which launch_robot.py
```

如果 `ldd` 显示 `libfranka.so` 或 conda 动态库 `not found`，通常需要重新编译 Polymetis，并确保 CMake 指向当前机器上的 `libfranka/build`。

如果真实机器人报 `communication_constraints_violation`，优先检查实时内核、CPU 频率缩放、网线直连和机器人网络配置。
