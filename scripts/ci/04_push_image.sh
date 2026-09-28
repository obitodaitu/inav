#!/usr/bin/env bash
#
# CI 步骤 4/4：推送镜像到内网 Harbor
#
# 前置：已 docker login "$REGISTRY"（CI 用流水线凭据注入，本脚本不处理登录）。
# 可选：PUSH_LATEST=1 时同时打并推送 :latest。
#
set -euo pipefail
source "$(dirname "$0")/_env.sh"
cd "$ROOT"

docker image inspect "$IMAGE_REF" >/dev/null 2>&1 \
    || { echo "[ERROR] 本地没有镜像 ${IMAGE_REF}，请先运行 03_build_image.sh" >&2; exit 1; }

echo "*** 推送 ${IMAGE_REF}"
docker push "$IMAGE_REF"

if [ "${PUSH_LATEST:-0}" = "1" ]; then
    LATEST_REF="${REGISTRY}/${IMAGE_NAME}:latest"
    echo "*** 推送 ${LATEST_REF}"
    docker tag "$IMAGE_REF" "$LATEST_REF"
    docker push "$LATEST_REF"
fi
