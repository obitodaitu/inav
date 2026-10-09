#!/usr/bin/env bash
#
# CI 步骤 2/4：编译 SITL
#
# 在“工具链容器”内直接编译（CI 的 stage 用 builder-base 镜像运行本脚本），
# 不再嵌套 docker。容器需含 cmake/ninja/gcc —— builder-base 已满足。
#
# 产物: dist/inav_*_SITL（供步骤 3 制作镜像使用；全程只编译一次）。
# 内网说明：编译不使用网络。
#
set -euo pipefail
source "$(dirname "$0")/_env.sh"
cd "$ROOT"

for c in cmake ninja gcc; do
    command -v "$c" >/dev/null 2>&1 \
        || { echo "[ERROR] 当前环境缺少 $c；本步骤需在含工具链的容器内运行（如 builder-base）" >&2; exit 1; }
done

echo "*** 编译 SITL"
rm -rf "$ROOT/build_SITL"
cmake -S "$ROOT" -B "$ROOT/build_SITL" -GNinja -DSITL=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
ninja -C "$ROOT/build_SITL"

mkdir -p "$DIST_DIR"
rm -f "$DIST_DIR"/inav_*_SITL
cp "$ROOT"/build_SITL/inav_*_SITL "$DIST_DIR"/
echo "*** 编译产物:"
ls -l "$DIST_DIR"/inav_*_SITL
