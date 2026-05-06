# Mamba Environment Setup

This repository uses one Python environment for both the Franka/Polymetis layer
and the teleop/GELLO layer. The environment name is `polymetis` because the
launch scripts call `conda activate polymetis`.

Run these commands from the `frankateleop/` repository root:

```bash
mamba env create -f environment.mamba.yml
mamba activate polymetis
```

If the environment already exists, update it instead:

```bash
mamba env update -n polymetis -f environment.mamba.yml --prune
mamba activate polymetis
```

Install the local Python packages after the environment is created:

```bash
pip install -e teleop
pip install -e teleop/third_party/DynamixelSDK/python
pip install -e polymetis/polymetis
```

Build the Franka C++ clients used by `launch_robot.py` and `launch_gripper.py`:

`libfranka` should live at Polymetis' default third-party path:

```text
polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka
```

If that directory is missing, clone it there first:

```bash
git clone https://github.com/frankaemika/libfranka \
  polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka
```

```bash
cd polymetis
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

If you keep `libfranka` outside the Polymetis submodule, point CMake at it:

```bash
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_FRANKA=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_ALLEGRO=OFF \
  -DFranka_DIR=/absolute/path/to/libfranka/build
```

Quick checks:

```bash
which launch_robot.py
python -c "import polymetis, teleop; print('ok')"
ldd polymetis/polymetis/build/franka_panda_client | grep "not found" || true
```

For real FR3 hardware, run on a real-time Linux kernel and verify:

```bash
uname -a
cat /sys/kernel/realtime
```

The kernel should report `PREEMPT_RT`, and `/sys/kernel/realtime` should print
`1`.
