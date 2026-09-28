#!/usr/bin/env bash
#
# CI 步骤：制作镜像 + 推送 Harbor（把 03、04 合成一步，供“同一步/同 daemon”使用）。
#   1) 用 dist/ 里已编译好的二进制构建 inav-sitl
#   2) 推送到 Harbor
#
# 必须和 02_compile.sh 在同一工作区；构建与推送在同一 docker daemon。
#
# 用法:
#   bash scripts/ci/03_build_and_push.sh            # 构建 + 推送
#   bash scripts/ci/03_build_and_push.sh --no-push  # 只构建
#   PUSH_LATEST=1 bash scripts/ci/03_build_and_push.sh
#
# 登录:
#   - 默认假定已 docker login $REGISTRY；
#   - 或设置 REG_USER / REG_PASS，由本脚本登录。
#
set -euo pipefail
source "$(dirname "$0")/_env.sh"
cd "$ROOT"

PUSH=1
for arg in "$@"; do
    case "$arg" in
        --no-push|--skip-push) PUSH=0 ;;
        -h|--help) sed -n '2,19p' "$0" | sed 's/^#\{1,\} \{0,1\}//'; exit 0 ;;
        *) echo "[ERROR] 未知参数: $arg" >&2; exit 2 ;;
    esac
done

# 有账号才登录，否则视为已在流水线前置步骤登录
if [ -n "${REG_USER:-}" ] && [ -n "${REG_PASS:-}" ]; then
    echo "*** docker login ${REGISTRY}"
    printf '%s' "$REG_PASS" | docker login "$REGISTRY" -u "$REG_USER" --password-stdin
fi

bash "$CI_DIR/03_build_image.sh"

if [ "$PUSH" = 0 ]; then
    echo "*** 已构建，跳过推送 (--no-push)"
    exit 0
fi

bash "$CI_DIR/04_push_image.sh"
