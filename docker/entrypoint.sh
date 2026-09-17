#!/bin/bash
#
# Entry point for the INAV SITL runtime image.
#
# Environment:
#   SIM         simulator backend to connect to (rf | xp | ue). Unset means
#               configurator-only mode: the SITL just listens on 5760.
#   SIM_IP      simulator host address. Defaults to the default gateway, which
#               is the Windows host when running under WSL2 --network host.
#   SIM_PORT    simulator host port (49000 for the X-Plane/UE UDP protocol).
#   EEPROM_PATH where the eeprom.bin lives (default /data/eeprom.bin).
#
# Any further arguments are passed through to the SITL verbatim, so
#   docker run ... inav-sitl:9.1 --sim=ue --chanmap=M01-01,...
# works as well.

set -e

BIN=$(find /opt/inav -maxdepth 1 -name '*_SITL' -type f | head -n 1)
if [ -z "$BIN" ]; then
    echo "entrypoint: no *_SITL binary found in /opt/inav" >&2
    exit 1
fi

EEPROM_PATH="${EEPROM_PATH:-/data/eeprom.bin}"

args=("--path=${EEPROM_PATH}")

if [ -n "${SIM:-}" ]; then
    if [ -z "${SIM_IP:-}" ]; then
        SIM_IP=$(ip route show default 2>/dev/null | awk '{print $3; exit}')
    fi
    args+=("--sim=${SIM}")
    if [ -n "${SIM_IP:-}" ]; then
        args+=("--simip=${SIM_IP}")
    fi
    if [ -n "${SIM_PORT:-}" ]; then
        args+=("--simport=${SIM_PORT}")
    fi
    echo "entrypoint: ${BIN} ${args[*]} (simulator: ${SIM} @ ${SIM_IP:-?}:${SIM_PORT:-default})"
else
    echo "entrypoint: ${BIN} ${args[*]} (configurator-only)"
fi

exec "${BIN}" "${args[@]}" "$@"
