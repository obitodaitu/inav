#!/usr/bin/env bash
#
# CI 步骤 2/4：编译 SITL 二进制
#   在 Harbor 的 builder-base 容器里、禁用网络编译；产物落到 dist/inav_*_SITL，
#   作为下一步制作镜像的输入（全流程只编译一次）。
#
# 内网说明：
#   - 容器加 --network none，编译全程不使用网络。
#   - 工具链来自 builder-base 镜像（CI 可在此之前 docker pull 一次）。
#   - 挂载源码时带 .git，cmake 能取到真实 revision（版本串不再是 GITDIR-N）。
#
set -euo pipefail
source "$(dirname "$0")/_env.sh"
cd "$ROOT"

command -v docker >/dev/null 2>&1 || { echo "[ERROR] 找不到 docker" >&2; exit 1; }

echo "*** 编译 SITL（容器 ${BUILDER_BASE_IMAGE}，禁用网络）"
rm -rf "$ROOT/build_SITL"
docker run --rm --network none \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -v "$ROOT":/src -w /src \
    "$BUILDER_BASE_IMAGE" \
    bash -c 'cmake -S . -B build_SITL -GNinja -DSITL=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo && ninja -C build_SITL'

mkdir -p "$DIST_DIR"
rm -f "$DIST_DIR"/inav_*_SITL
cp "$ROOT"/build_SITL/inav_*_SITL "$DIST_DIR"/
echo "*** 编译产物:"
ls -l "$DIST_DIR"/inav_*_SITL
