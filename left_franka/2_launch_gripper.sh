#!/bin/bash
set -e

echo ">>> 激活 conda 环境 polymetis ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis || { echo "❌ 激活失败"; exit 1; }

echo ">>> 清理占用50054端口的 gripper 进程 ..."
if ! command -v lsof &> /dev/null; then
    echo "❌ 错误：未找到lsof命令，请先安装（例如：sudo apt install lsof）"
    exit 1
fi

PIDS=$(sudo lsof -t -iTCP:50054 -sTCP:LISTEN || true)
if [[ -n "$PIDS" ]]; then
    echo "发现占用50054端口的进程，正在逐个终止..."
    while IFS= read -r PID; do
        [[ -z "$PID" ]] && continue
        echo "正在终止 PID: $PID"
        sudo kill -9 "$PID" 2>/dev/null || echo "⚠️ PID $PID 已退出或终止失败，继续处理"
    done <<< "$PIDS"
else
    echo "⚠️ 未发现占用50054端口的进程，无需清理"
fi

POLY_ROOT="../polymetis"

WORK_DIR="$POLY_ROOT/polymetis/python/polymetis"
if [[ ! -d "$WORK_DIR/conf" ]]; then
    echo "❌ 配置目录 $WORK_DIR/conf 不存在"
    exit 1
fi

echo ">>> 启动 Franka 客户端 ..."
cd "$WORK_DIR"
launch_gripper.py --config-name=launch_left_gripper # gripper=franka_hand  #1. franka_hand 2.none
