#!/usr/bin/env bash
#
# Build the INAV SITL image in two phases so the second one can run air-gapped.
#
#   base   builds the two pre-baked dependency images (Dockerfile.sitl-base).
#          Needs apt/network, so it runs once on an internet-connected machine
#          and the results are pushed to, or saved for, the internal registry.
#   image  builds inav-sitl from those bases (Dockerfile.sitl). Touches no
#          network, which is the whole point on the internal CI.
#
# Usage:
#   docker/build_sitl_image.sh base     # online side, once
#   docker/build_sitl_image.sh image    # offline CI, every build
#   docker/build_sitl_image.sh all      # both (online only)
#
# Environment:
#   BASE_SOURCE          upstream image for the bases   (default ubuntu:22.04)
#   BUILDER_BASE_IMAGE   ref used by the builder stage  (default registry.ghostcloud.cn/integration/inav-sitl-builder-base:22.04)
#   RUNTIME_BASE_IMAGE   ref used by the runtime stage  (default registry.ghostcloud.cn/integration/inav-sitl-runtime-base:22.04)
#   IMAGE                tag of the runtime image       (default inav-sitl:9.1)
#   SAVE                 if set, docker save IMAGE to this .tar.gz after building
#
# NOTE: BUILDER_BASE_IMAGE / RUNTIME_BASE_IMAGE default to registry.ghostcloud.cn.
# If the build nodes resolve the Harbor under another name (e.g. the in-cluster
# registry.cluster.local:30443), override either of them here.
#
# Air-gapped example (defaults already point at the internal Harbor, so `image`
# alone is enough; override only the final tag):
#   IMAGE=registry.ghostcloud.cn/integration/inav-sitl:9.1 \
#   SAVE=inav-sitl-9.1.tar.gz \
#       docker/build_sitl_image.sh image
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BASE_SOURCE="${BASE_SOURCE:-ubuntu:22.04}"
BUILDER_BASE_IMAGE="${BUILDER_BASE_IMAGE:-registry.ghostcloud.cn/integration/inav-sitl-builder-base:22.04}"
RUNTIME_BASE_IMAGE="${RUNTIME_BASE_IMAGE:-registry.ghostcloud.cn/integration/inav-sitl-runtime-base:22.04}"
IMAGE="${IMAGE:-inav-sitl:9.1}"
SAVE="${SAVE:-}"

usage() {
    sed -n '2,32p' "${BASH_SOURCE[0]}" | sed 's/^#\{1,\} \{0,1\}//'
}

build_base() {
    echo "*** Building builder base '${BUILDER_BASE_IMAGE}' from '${BASE_SOURCE}'"
    docker build \
        --file "${REPO_ROOT}/Dockerfile.sitl-base" \
        --target builder-base \
        --build-arg BASE_SOURCE="${BASE_SOURCE}" \
        --tag "${BUILDER_BASE_IMAGE}" \
        "${REPO_ROOT}"

    echo "*** Building runtime base '${RUNTIME_BASE_IMAGE}' from '${BASE_SOURCE}'"
    docker build \
        --file "${REPO_ROOT}/Dockerfile.sitl-base" \
        --target runtime-base \
        --build-arg BASE_SOURCE="${BASE_SOURCE}" \
        --tag "${RUNTIME_BASE_IMAGE}" \
        "${REPO_ROOT}"
}

build_image() {
    echo "*** Building SITL image '${IMAGE}'"
    echo "    builder base: ${BUILDER_BASE_IMAGE}"
    echo "    runtime base: ${RUNTIME_BASE_IMAGE}"
    docker build \
        --file "${REPO_ROOT}/Dockerfile.sitl" \
        --build-arg BUILDER_BASE_IMAGE="${BUILDER_BASE_IMAGE}" \
        --build-arg RUNTIME_BASE_IMAGE="${RUNTIME_BASE_IMAGE}" \
        --tag "${IMAGE}" \
        "${REPO_ROOT}"

    if [ -n "${SAVE}" ]; then
        echo "*** Saving '${IMAGE}' to '${SAVE}'"
        docker save "${IMAGE}" | gzip > "${SAVE}"
    fi
}

case "${1:-}" in
    base)  build_base ;;
    image) build_image ;;
    all)   build_base; build_image ;;
    -h|--help|help|"") usage; [ -n "${1:-}" ] || exit 1 ;;
    *)     echo "Unknown command: $1" >&2; usage >&2; exit 1 ;;
esac
