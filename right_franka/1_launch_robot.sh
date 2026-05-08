#!/bin/bash
set -e

echo ">>> 激活 conda 环境 polymetis ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis || { echo "❌ 激活失败"; exit 1; }
echo "✅ conda环境激活成功"

echo ">>> 清理占用50051端口的进程 ..."

# 检查lsof是否安装
if ! command -v lsof &> /dev/null; then
    echo "❌ 错误：未找到lsof命令，请先安装（例如：sudo apt install lsof）"
    exit 1
fi

echo "🔍 正在查找占用50051端口的进程..."
PIDS=$(sudo lsof -t -iTCP:50051 -sTCP:LISTEN || true)
echo "📝 调试信息：查找到的PID=$PIDS"

if [[ -n "$PIDS" ]]; then
    echo "发现占用50051端口的进程，正在逐个终止..."
    while IFS= read -r PID; do
        [[ -z "$PID" ]] && continue
        echo "正在终止 PID: $PID"
        sudo kill -9 "$PID" 2>/dev/null || echo "⚠️ PID $PID 已退出或终止失败，继续处理"
    done <<< "$PIDS"
    echo "✅ 端口50051相关进程已处理完成"
else
    echo "⚠️ 未发现占用50051端口的进程，无需清理"
fi

echo "✅ 端口清理步骤完成"

POLY_ROOT="../polymetis"
echo "📝 调试信息：POLY_ROOT=$POLY_ROOT"

WORK_DIR="$POLY_ROOT/polymetis/python/polymetis"
echo "📝 调试信息：WORK_DIR=$WORK_DIR"

if [[ ! -d "$WORK_DIR/conf" ]]; then
    echo "❌ 配置目录 $WORK_DIR/conf 不存在"
    exit 1
fi
echo "✅ 配置目录检查通过"

echo ">>> 启动Franka 客户端 ..."
cd "$WORK_DIR" || { echo "❌ 无法进入工作目录 $WORK_DIR"; exit 1; }
echo "📝 调试信息：当前工作目录=$(pwd)"

launch_robot.py --config-name=launch_right_robot
