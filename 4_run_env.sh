#!/bin/bash
set -e

echo ">>> 激活 conda 环境 polymetis ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis || { echo "❌ 激活失败，请确认 polymetis 环境存在"; exit 1; }

echo ">>> 在当前目录及子目录中查找 teleop ..."
POLY_ROOT=$(find . -type d -name "teleop" | head -n 1)

if [[ -z "$POLY_ROOT" ]]; then
    echo "❌ 当前目录下未找到 teleop 文件夹"
    exit 1
fi

SCRIPT_PATH="$POLY_ROOT/experiments/run_env.py"
if [[ ! -f "$SCRIPT_PATH" ]]; then
    echo "❌ 未找到脚本：$SCRIPT_PATH"
    exit 1
fi

echo ">>> 启动Gripper 客户端 ..."
python3 "$SCRIPT_PATH" --agent=teleop 
#如果要启用采集数据，需要在后面增加“--use_save_interface”
