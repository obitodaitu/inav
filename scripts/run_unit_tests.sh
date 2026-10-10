#!/usr/bin/env bash
#
# Run the Python unit tests for the docker/ tooling
# (make_msp_eeprom.py, msp_rc_client.py).
#
# Stdlib unittest only -- no pytest to install, which is what makes it usable
# on an air-gapped CI. This is the local/console entry; the CI entry that also
# reports and uploads is scripts/run_unit_tests_report.sh.
#
#   scripts/run_unit_tests.sh
#   scripts/run_unit_tests.sh -v
#   scripts/run_unit_tests.sh -k VehicleIndex
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

PYTHON="${PYTHON:-python3}"
exec "${PYTHON}" "${SCRIPT_DIR}/unit_tests.py" "$@"
