#!/usr/bin/env bash
#
# CI 步骤 1/4：单元测试
#
# 本脚本在“工具链容器”内直接执行（CI 的 stage 用 builder-base 镜像运行本脚本），
# 因此不再嵌套 docker。容器需含 python3 —— builder-base 已满足。
#
# 容器内执行 scripts/run_unit_tests_report.sh：
#   - 测试只用 Python 标准库，无需 pip / 外网
#   - 产出 test-reports/unit/summary.json（挂载回工作区）
#   - 上传报告平台需要能访问 SERVER_URL（内网）
#
# 环境变量：
#   SERVER_URL / CATEGORY_NAME / PARENT_MODULE_NAME / MODULE_NAME / NO_UPLOAD
#
set -euo pipefail
source "$(dirname "$0")/_env.sh"
cd "$ROOT"

command -v python3 >/dev/null 2>&1 \
    || { echo "[ERROR] 当前环境没有 python3；本步骤需在含 python3 的容器内运行（如 builder-base）" >&2; exit 1; }

exec bash "$ROOT/scripts/run_unit_tests_report.sh"
