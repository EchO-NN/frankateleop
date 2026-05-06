#!/bin/bash

# 双夹爪启动脚本
echo "========================================"
echo "启动双夹爪服务器脚本"
echo "========================================"

# 切换到脚本所在目录
cd "$(dirname "$0")"
echo "当前工作目录: $(pwd)"

sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1

source ~/.bashrc  # 加载bash配置

eval "$(conda shell.bash hook)"  # 初始化conda
conda activate polymetis-local   # 激活环境

unset LD_LIBRARY_PATH
unset PYTHONPATH

# 检查必要的文件是否存在
if [ ! -f "launch_gripper.py" ]; then
    echo "错误: launch_gripper.py 文件不存在!"
    exit 1
fi

if [ ! -f "launch_dual_gripper_server.py" ]; then
    echo "错误: launch_dual_gripper_server.py 文件不存在!"
    exit 1
fi

# 设置环境变量
export PYTHONNOUSERSITE=1
echo "环境变量设置完成"

# 启动第一个夹爪
echo "----------------------------------------"
echo "启动第一个夹爪服务器..."
# 在新终端中启动第一个夹爪服务器
if command -v gnome-terminal > /dev/null; then
    gnome-terminal -- bash -c "cd $(pwd); export PYTHONNOUSERSITE=1; python launch_gripper.py; exec bash"
    echo "第一个夹爪服务器已在新终端启动"
else
    echo "警告: 未找到支持的终端模拟器，将在当前终端启动第一个夹爪服务器"
    python launch_gripper.py &
    echo "第一个夹爪服务器已在后台启动"
fi

# 等待一段时间确保服务启动
sleep 2

# 启动第二个夹爪
echo "----------------------------------------"
echo "启动第二个夹爪服务器..."
# 在新终端中启动第二个夹爪服务器
if command -v gnome-terminal > /dev/null; then
    gnome-terminal -- bash -c "cd $(pwd); export PYTHONNOUSERSITE=1; python launch_gripper.py --config-name=launch_gripper2; exec bash"
    echo "第二个夹爪服务器已在新终端启动"
else
    echo "警告: 未找到支持的终端模拟器，将在当前终端启动第二个夹爪服务器"
    python launch_gripper.py --config-name=launch_gripper2 &
    echo "第二个夹爪服务器已在后台启动"
fi

# 等待一段时间确保服务启动
sleep 2

# 启动双夹爪控制服务器
echo "----------------------------------------"
echo "启动双夹爪控制服务器..."
# 在新终端中启动双夹爪控制服务器
if command -v gnome-terminal > /dev/null; then
    gnome-terminal -- bash -c "cd $(pwd); export PYTHONNOUSERSITE=1; python launch_dual_gripper_server.py; exec bash"
    echo "双夹爪控制服务器已在新终端启动"
else
    echo "警告: 未找到支持的终端模拟器，将在当前终端启动双夹爪控制服务器"
    python launch_dual_gripper_server.py &
    echo "双夹爪控制服务器已在后台启动"
fi

echo "========================================"
echo "双夹爪服务器启动脚本执行完成"
echo "========================================"