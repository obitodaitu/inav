#!/usr/bin/env bash
#
# CI 步骤 1/4：单元测试
#   CI 机器只有 docker，所以测试放进容器里跑；镜像用 Harbor 上带 python3 的
#   builder-base（编译步骤本来也要用它，复用不额外拉镜像）。
#
#   容器内执行 scripts/run_unit_tests_report.sh：
#     - 测试只用 Python 标准库，无需 pip / 外网
#     - 产出 test-reports/unit/summary.json（流水线约定产物，已挂载回工作区）
#     - 上传报告平台需要容器能访问 SERVER_URL（内网）
#
# 环境变量：
#   TEST_IMAGE           跑测试的镜像（默认 builder-base；也可用 library/python:3.10-slim）
#   DOCKER_NET=host      报告平台只在宿主机可达时用（默认 bridge，一般够用）
#   SERVER_URL / CATEGORY_NAME / PARENT_MODULE_NAME / MODULE_NAME / NO_UPLOAD
#
set -euo pipefail
source "$(dirname "$0")/_env.sh"
cd "$ROOT"

TEST_IMAGE="${TEST_IMAGE:-$BUILDER_BASE_IMAGE}"
SERVER_URL="${SERVER_URL:-http://192.168.3.3:31840}"
CATEGORY_NAME="${CATEGORY_NAME:-固定要地防护}"
PARENT_MODULE_NAME="${PARENT_MODULE_NAME:-Vega-Sim}"
MODULE_NAME="${MODULE_NAME:-INAV SITL}"

# 只有显式给 DOCKER_NET 时才加 --network，避免默认行为被改
NET_ARGS=()
if [ -n "${DOCKER_NET:-}" ]; then
    NET_ARGS+=(--network "$DOCKER_NET")
fi

echo "*** 单元测试（镜像 ${TEST_IMAGE}）"
docker run --rm \
    "${NET_ARGS[@]}" \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -e SERVER_URL="$SERVER_URL" \
    -e CATEGORY_NAME="$CATEGORY_NAME" \
    -e PARENT_MODULE_NAME="$PARENT_MODULE_NAME" \
    -e MODULE_NAME="$MODULE_NAME" \
    -e NO_UPLOAD="${NO_UPLOAD:-0}" \
    -v "$ROOT":/src \
    -w /src \
    "$TEST_IMAGE" \
    bash scripts/run_unit_tests_report.sh

echo "*** 测试结果产物: ${ROOT}/test-reports/unit/summary.json"
