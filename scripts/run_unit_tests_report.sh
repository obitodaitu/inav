#!/usr/bin/env bash
#
# 跑 Python 单元测试、产出流水线约定的 summary.json，并上传报告平台。
#
# 产物口径（产品线测试体系约定，见 result-file-taxonomy）：
#   流水线测试结果文件必须是名为 summary.json 的文件，字段固定为
#   total_tests / passed / failed / skipped / pass_rate / duration_seconds / coverage。
#   日检查按 basename 精确匹配 summary.json，故本脚本以它为核心交付物；
#   JUnit XML 只是上传报告平台用的中间产物。
#
# 参照 SimulatorSystem/Scripts/Linux/RunAutomationTests.sh，但产物定位不同：
#   - 测试结果: <ARTIFACT_DIR>/summary.json     ← 流水线必须归档的就是这个
#   - 中间 JUnit: <ARTIFACT_DIR>/unit_tests.xml
#   - 上传回执: <ARTIFACT_DIR>/upload_result.json
#
# 用法:
#   scripts/run_unit_tests_report.sh
#   NO_UPLOAD=1 scripts/run_unit_tests_report.sh     # 只产 summary.json，不上传
#
# 环境变量（都有默认值，按需覆盖）:
#   ARTIFACT_DIR        产物目录（默认 test-reports/unit，即 summary.json 所在）
#   SERVER_URL          报告平台地址（默认 http://192.168.3.3:31840）
#   UPLOAD_PY           上传脚本（默认 scripts/upload_report.py）
#   CATEGORY_NAME       报告分类（默认 固定要地防护）
#   PARENT_MODULE_NAME  父模块（默认 Vega-Sim）
#   MODULE_NAME         模块（默认 INAV SITL）
#   NO_UPLOAD=1         只生成 summary.json，不上传
#
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

SERVER_URL="${SERVER_URL:-http://192.168.3.3:31840}"
UPLOAD_PY="${UPLOAD_PY:-$ROOT/scripts/upload_report.py}"

# upload_report.py 会把产物写到 <report-dir>/<test-type>/<base>.*，
# 取 test-type=unit 时 <report-dir>/unit 即 summary.json 的落点。
ARTIFACT_DIR="${ARTIFACT_DIR:-$ROOT/test-reports/unit}"
REPORT_DIR="$(dirname "$ARTIFACT_DIR")"

JUNIT_XML="$ARTIFACT_DIR/unit_tests.xml"
SUMMARY_JSON="$ARTIFACT_DIR/summary.json"
RESULT_JSON="$ARTIFACT_DIR/upload_result.json"

CATEGORY_NAME="${CATEGORY_NAME:-固定要地防护}"
PARENT_MODULE_NAME="${PARENT_MODULE_NAME:-Vega-Sim}"
MODULE_NAME="${MODULE_NAME:-INAV SITL}"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

[ -f "$UPLOAD_PY" ] || fail "找不到上传脚本 $UPLOAD_PY（可用 UPLOAD_PY=... 指定）"
mkdir -p "$ARTIFACT_DIR"

echo "[1/3] 运行单元测试 ..."
python3 "$ROOT/scripts/unit_tests.py" --xml "$JUNIT_XML"
TEST_EXITCODE=$?
echo
if [ "$TEST_EXITCODE" = "0" ]; then
    echo "[OK] 单元测试通过。"
else
    echo "[WARN] 单元测试失败，退出码 $TEST_EXITCODE（报告照常产出）。"
fi

echo
echo "[2/3] 生成 summary.json ..."
UPLOAD_ARGS=(
    --url "$SERVER_URL"
    --file "$JUNIT_XML"
    --out-json "$RESULT_JSON"
    --report-dir "$REPORT_DIR"
    --test-type unit
    --report-base summary
    --category-name "$CATEGORY_NAME"
    --parent-module-name "$PARENT_MODULE_NAME"
    --module-name "$MODULE_NAME"
)
if [ "${NO_UPLOAD:-0}" = "1" ]; then
    UPLOAD_ARGS+=(--no-upload)
fi
python3 "$UPLOAD_PY" "${UPLOAD_ARGS[@]}"
UPLOAD_EXITCODE=$?
echo

if [ ! -f "$SUMMARY_JSON" ]; then
    echo "[FAIL] 未生成 $SUMMARY_JSON"
    exit 1
fi

echo "[3/3] 测试结果产物: $SUMMARY_JSON"
python3 - "$SUMMARY_JSON" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
print("      total_tests={total_tests} passed={passed} failed={failed} "
      "skipped={skipped} pass_rate={pass_rate} duration_seconds={duration_seconds} "
      "coverage={coverage} test_type={test_type}".format(**d))
PY

if [ "${NO_UPLOAD:-0}" = "1" ]; then
    echo "[OK] 未上传（NO_UPLOAD=1）。"
    exit "$TEST_EXITCODE"
fi

if [ "$UPLOAD_EXITCODE" = "0" ]; then
    echo "[OK] 报告上传成功。回执: $RESULT_JSON"
else
    echo "[FAIL] 报告上传失败，退出码 $UPLOAD_EXITCODE。回执: $RESULT_JSON"
fi
exit "$UPLOAD_EXITCODE"
