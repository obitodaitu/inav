#!/usr/bin/env bash
#
# CI 步骤 3/4：制作运行时镜像
#   把上一步 dist/ 里已编译好的二进制打包成 inav-sitl 镜像（不重新编译）。
#   只依赖 Harbor 的 runtime-base 基础镜像，不访问外网。
#
# 分两段（编译 / 打包）的原因：编译只做一次，且编译产物可作为流水线产物留档。
#
set -euo pipefail
source "$(dirname "$0")/_env.sh"
cd "$ROOT"

ls "$DIST_DIR"/inav_*_SITL >/dev/null 2>&1 \
    || { echo "[ERROR] 缺少编译产物 ${DIST_DIR}/inav_*_SITL，请先运行 02_compile.sh" >&2; exit 1; }

echo "*** 制作镜像 ${IMAGE_REF}"
docker build \
    --file "$ROOT/Dockerfile.sitl.package" \
    --build-arg RUNTIME_BASE_IMAGE="$RUNTIME_BASE_IMAGE" \
    --tag "$IMAGE_REF" \
    "$ROOT"

echo "*** 完成:"
docker images "$IMAGE_REF" --format '{{.Repository}}:{{.Tag}}  {{.Size}}'
