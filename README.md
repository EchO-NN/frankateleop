# Franka Teleop

Franka Teleop is a GELLO/teleop + Polymetis based single-arm teleoperation stack for a Franka FR3 robot.

The current runtime path does not depend on ROS 2 or `franky`. Real robot control flows through:

```text
teleop/GELLO -> Polymetis RobotInterface/GripperInterface -> Franka C++ client -> libfranka -> FR3
```

## Repository Layout

```text
frankateleop/
  environment.mamba.yml      # mamba/conda environment definition
  MAMBA_SETUP.md             # detailed environment setup notes
  right_franka/              # supported single-arm launch scripts
  1_launch_robot.sh          # legacy root-level script; not the recommended entrypoint
  2_launch_gripper.sh        # legacy root-level script; not the recommended entrypoint
  3_launch_node.sh           # legacy root-level script; not the recommended entrypoint
  4_run_env.sh               # legacy root-level script; not the recommended entrypoint
  left_franka/               # legacy left-arm scripts; not used by this single-arm setup
  polymetis/                 # Polymetis control layer and Franka C++ client
  teleop/                    # GELLO/teleop Python code
  scripts/                   # helper scripts
```

## Environment Setup

Use `mamba` if possible. The environment name should stay `polymetis` because the launch scripts call `conda activate polymetis`.

```bash
cd frankateleop
mamba env create -f environment.mamba.yml
mamba activate polymetis
```

If the environment already exists:

```bash
mamba env update -n polymetis -f environment.mamba.yml --prune
mamba activate polymetis
```

Install the local Python packages:

```bash
pip install -e teleop
pip install -e teleop/third_party/DynamixelSDK/python
pip install -e polymetis/polymetis
```

See [MAMBA_SETUP.md](MAMBA_SETUP.md) for more detailed setup notes.

## Build Polymetis and libfranka

Real FR3 control requires the Polymetis C++ clients and `libfranka`.

Polymetis expects `libfranka` at:

```text
polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka
```

If the directory does not exist, clone `libfranka` there:

```bash
git clone https://github.com/frankaemika/libfranka \
  polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka
```

Build `libfranka` and Polymetis:

```bash
cd frankateleop/polymetis
./scripts/build_libfranka.sh

cd polymetis
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

If `libfranka` is outside this repository, pass its CMake package path:

```bash
-DFranka_DIR=/absolute/path/to/libfranka/build
```

## Launch the Single Arm

This repository uses the `right_franka/` configuration as the single supported runtime path. The root-level scripts and `left_franka/` scripts are kept only as legacy/reference files.

Use four terminals and start the scripts in order:

```bash
cd frankateleop/right_franka
./1_launch_robot.sh
```

```bash
cd frankateleop/right_franka
./2_launch_gripper.sh
```

```bash
cd frankateleop/right_franka
./3_launch_node.sh
```

```bash
cd frankateleop/right_franka
./4_run_env.sh
```

To enable data collection, add `--use_save_interface` to the `run_env.py` command in `right_franka/4_run_env.sh`.

The supported single-arm scripts use these defaults:

| Target | Robot server | Gripper server | Teleop server | Robot config |
| --- | ---: | ---: | ---: | --- |
| Single arm, `right_franka/` | 50051 | 50053 | 6001 | `launch_right_robot` |

The `robot_ip` argument in `3_launch_node.sh` is the IP address of the machine running the Polymetis robot and gripper servers. If everything runs on the same machine, `127.0.0.1` is correct.

## Network and Robot Configuration

Common FR3 network settings used by this repository:

- Control PC interface connected to Franka Control: `172.16.0.1/24`
- Robot IP: `172.16.0.2`
- Robot server: `50051`
- Gripper server: `50053`
- Teleop server: `6001`

Most of these values are configured in:

```text
polymetis/polymetis/python/polymetis/conf/
right_franka/
```

## Pre-Run Checks

Confirm that the Python packages import correctly:

```bash
conda activate polymetis
python -c "import polymetis, teleop; print('ok')"
```

Confirm that the Franka client binaries can find their shared libraries:

```bash
ldd polymetis/polymetis/build/franka_panda_client | grep "not found" || true
ldd polymetis/polymetis/build/franka_hand_client | grep "not found" || true
```

For real robot control, use a real-time kernel:

```bash
uname -a
cat /sys/kernel/realtime
```

Expected result:

- `uname -a` includes `PREEMPT_RT`
- `/sys/kernel/realtime` prints `1`

## Troubleshooting

If `launch_robot.py` or `launch_gripper.py` cannot be found:

```bash
conda activate polymetis
pip install -e polymetis/polymetis
which launch_robot.py
which launch_gripper.py
```

If `ldd` reports `libfranka.so` or conda shared libraries as `not found`, rebuild Polymetis and make sure CMake points to the `libfranka/build` directory on the current machine.

If the real robot reports `communication_constraints_violation`, first check:

- Real-time kernel is enabled
- CPU frequency scaling is not causing latency spikes
- The robot is connected directly through the expected network interface
- Franka Control network settings match the configured robot IP
- No stale robot or gripper server is still listening on the configured ports

The launch scripts clean common occupied ports automatically, but they may require `sudo` and `lsof`.
