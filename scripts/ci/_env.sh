#!/usr/bin/env bash
#
# CI 各步骤共用的环境变量。步骤脚本用 `source` 引入，不要直接执行。
#
# 所有值都可用环境变量覆盖，默认面向内网：
#   - Harbor: registry.ghostcloud.cn/integration
#   - 基础镜像已在 Harbor（见 Dockerfile.sitl-base）
#
set -euo pipefail

CI_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${CI_DIR}/../.." && pwd)"

# --- 镜像坐标 ---------------------------------------------------------------
REGISTRY="${REGISTRY:-registry.ghostcloud.cn/integration}"
IMAGE_NAME="${IMAGE_NAME:-inav-sitl}"
# tag 优先取显式 IMAGE_TAG；否则用 git 描述（CI 里 checkout 后即 commit/tag），再退化 latest
IMAGE_TAG="${IMAGE_TAG:-$(git -C "$ROOT" describe --tags --always --dirty 2>/dev/null || echo latest)}"
IMAGE_REF="${IMAGE_REF:-${REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}}"

# --- 预装基础镜像（Harbor，构建期唯一外部依赖）------------------------------
BUILDER_BASE_IMAGE="${BUILDER_BASE_IMAGE:-${REGISTRY}/inav-sitl-builder-base:20.04}"
RUNTIME_BASE_IMAGE="${RUNTIME_BASE_IMAGE:-${REGISTRY}/inav-sitl-runtime-base:22.04}"

# --- 编译产物目录（只放最终二进制，作为制作镜像的构建上下文）----------------
DIST_DIR="${DIST_DIR:-${ROOT}/dist}"

export REGISTRY IMAGE_NAME IMAGE_TAG IMAGE_REF
export BUILDER_BASE_IMAGE RUNTIME_BASE_IMAGE DIST_DIR ROOT
