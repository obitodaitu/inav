#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Upload a JUnit XML report to the report platform.
# Uses only the Python standard library, so `requests` is not required.
#
# Sourced from SimulatorSystem/tests/upload_report.py so this repository's CI
# is self-contained. Keep it in sync with upstream; override the path at run
# time with UPLOAD_PY=... if you would rather use the sibling repo's copy.
#
# Besides uploading, this script also writes local report artifacts under
# REPORT_DIR/<test_dir> (unit or interface). Old artifacts in that directory
# are removed before writing. Callers can provide --report-base to use a
# stable local name such as summary; otherwise the upload name is reused.

import argparse
import copy
import datetime
import html
import json
import mimetypes
import os
import shutil
import sys
import tempfile
import urllib.request
import uuid
import xml.etree.ElementTree as ElementTree


def build_multipart(fields, file_path, upload_filename=None):
    boundary = uuid.uuid4().hex
    lines = []
    for key, value in fields.items():
        part = (
            "--{boundary}\r\n"
            "Content-Disposition: form-data; name=\"{key}\"\r\n"
            "\r\n"
            "{value}\r\n"
        ).format(boundary=boundary, key=key, value=value).encode("utf-8")
        lines.append(part)

    filename = upload_filename or os.path.basename(file_path)
    mime_type, _ = mimetypes.guess_type(file_path)
    if not mime_type:
        mime_type = "application/octet-stream"

    with open(file_path, "rb") as handle:
        file_data = handle.read()

    header = (
        "--{boundary}\r\n"
        "Content-Disposition: form-data; name=\"file\"; filename=\"{filename}\"\r\n"
        "Content-Type: {mime_type}\r\n"
        "\r\n"
    ).format(boundary=boundary, filename=filename, mime_type=mime_type).encode("utf-8")
    lines.append(header)
    lines.append(file_data)
    lines.append(("\r\n--{boundary}--\r\n").format(boundary=boundary).encode("utf-8"))
    return boundary, b"".join(lines)


def _test_label(test_type):
    return {"unit": "单元测试", "interface": "接口测试"}.get(test_type, test_type)


def _test_dir(test_type):
    return {"unit": "unit", "interface": "interface"}.get(test_type, test_type)


def _safe_component(value):
    return "".join(ch for ch in value if ch not in "/\\:*?\"<>|")


def _parse_junit(file_path):
    summary = {"tests": 0, "failures": 0, "errors": 0, "skipped": 0, "time": 0.0}
    try:
        root = ElementTree.parse(file_path).getroot()
        for key in ("tests", "failures", "errors", "skipped"):
            summary[key] = int(root.get(key, 0))
        summary["time"] = float(root.get("time", 0.0))
    except Exception:
        pass
    return summary


def _merge_junit(paths):
    """把多个 JUnit XML 合并为单个 testsuite(合并全部 testcase), 返回 (bytes, summary)。"""
    root = ElementTree.Element("testsuite", {"name": "SimulatorSystem Interface (merged)"})
    total = failures = errors = skipped = 0
    total_time = 0.0
    for path in paths:
        if not path or not os.path.isfile(path):
            continue
        try:
            parsed = ElementTree.parse(path).getroot()
        except Exception:  # noqa: BLE001
            continue
        for case in parsed.iter("testcase"):
            root.append(copy.deepcopy(case))
            total += 1
            if case.find("failure") is not None:
                failures += 1
            elif case.find("error") is not None:
                errors += 1
            elif case.find("skipped") is not None:
                skipped += 1
            try:
                total_time += float(case.get("time", 0.0))
            except (TypeError, ValueError):
                pass
    root.set("tests", str(total))
    root.set("failures", str(failures))
    root.set("errors", str(errors))
    root.set("skipped", str(skipped))
    root.set("time", "{0:.3f}".format(total_time))
    data = ElementTree.tostring(root, encoding="utf-8", xml_declaration=True)
    summary = {"tests": total, "failures": failures, "errors": errors, "skipped": skipped, "time": total_time}
    return data, summary


def _clean_report_dir(report_dir):
    if os.path.isdir(report_dir):
        for entry in os.listdir(report_dir):
            entry_path = os.path.join(report_dir, entry)
            if os.path.isfile(entry_path) or os.path.islink(entry_path):
                os.remove(entry_path)
            elif os.path.isdir(entry_path):
                shutil.rmtree(entry_path, ignore_errors=True)


def write_report_artifacts(args, upload_payload, summary, report_base=None):
    category = _safe_component(args.category_name)
    parent = _safe_component(args.parent_module_name)
    module = _safe_component(args.module_name)
    label = _test_label(args.test_type)
    timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    report_dir = os.path.join(args.report_dir, _test_dir(args.test_type))
    # Read the source report before writing: the source file may live inside
    # report_dir (e.g. RunAirSimApiTest.sh writes its raw report there).
    with open(args.file, "rb") as handle:
        source_data = handle.read()
    os.makedirs(report_dir, exist_ok=True)

    base = report_base or "{0}-{1}-{2}-{3}-{4}".format(
        category, parent, module, label, timestamp
    )
    if not report_base:
        timestamp = base.rsplit("-", 1)[-1]
    # 仅清理同名旧产物, 保留同目录下其它用例集(如 summary 与 summary-airsim)的产物,
    # 避免接口测试的多个脚本互相清空 test-reports/<type>。
    for _ext in (".xml", ".json", ".html"):
        _stale = os.path.join(report_dir, base + _ext)
        if os.path.isfile(_stale):
            os.remove(_stale)
    xml_path = os.path.join(report_dir, base + ".xml")
    json_path = os.path.join(report_dir, base + ".json")
    html_path = os.path.join(report_dir, base + ".html")

    with open(xml_path, "wb") as handle:
        handle.write(source_data)

    total = summary["tests"]
    failed = summary["failures"] + summary["errors"]
    skipped = summary["skipped"]
    passed = max(total - failed - skipped, 0)
    duration = summary["time"]
    pass_rate = round(passed / total * 100, 2) if total > 0 else 0.0

    artifact = {
        "total_tests": total,
        "passed": passed,
        "failed": failed,
        "skipped": skipped,
        "pass_rate": pass_rate,
        "duration_seconds": duration,
        "coverage": None,
        "category_name": args.category_name,
        "parent_module_name": args.parent_module_name,
        "module_name": args.module_name,
        "test_type": args.test_type,
        "test_label": label,
        "timestamp": timestamp,
        "xml_file": os.path.basename(xml_path),
        "html_file": os.path.basename(html_path),
        "junit": summary,
        "upload": upload_payload,
    }
    with open(json_path, "w", encoding="utf-8") as handle:
        json.dump(artifact, handle, ensure_ascii=False, indent=2)

    total = summary["tests"]
    passed = max(total - summary["failures"] - summary["errors"] - summary["skipped"], 0)
    state = "pass" if summary["failures"] == 0 and summary["errors"] == 0 else "fail"
    rows = ""
    try:
        root = ElementTree.fromstring(source_data)
        case_no = 0
        for suite in root.iter("testsuite"):
            for case in suite.iter("testcase"):
                case_no += 1
                name = case.get("name", "")
                cls = case.get("classname", "")
                status = "pass"
                if case.find("failure") is not None:
                    status = "fail"
                elif case.find("error") is not None:
                    status = "error"
                elif case.find("skipped") is not None:
                    status = "skipped"
                rows += (
                    "<tr><td>{0}</td><td>{1}</td><td class=\"{2}\">{2}</td></tr>\n"
                ).format(case_no, html.escape(cls + "." + name), status)
    except Exception:
        pass

    body = (
        "<!DOCTYPE html>\n"
        "<html lang=\"zh-CN\"><head><meta charset=\"utf-8\">\n"
        "<title>{title}</title>\n"
        "<style>\n"
        "body{{font-family:Microsoft YaHei,Arial,sans-serif;margin:24px;color:#222;}}\n"
        "h1{{font-size:18px;}} .summary{{margin:12px 0;padding:12px;border:1px solid #ddd;border-radius:4px;background:#fafafa;}}\n"
        ".summary .item{{display:inline-block;margin-right:24px;font-size:15px;}}\n"
        ".pass{{color:#2e7d32;}} .fail,.error{{color:#c62828;}} .skipped{{color:#f9a825;}}\n"
        "table{{border-collapse:collapse;width:100%;margin-top:12px;}} "
        "th,td{{border:1px solid #ddd;padding:6px 10px;font-size:14px;text-align:left;}}\n"
        "</style></head><body>\n"
        "<h1>{title}</h1>\n"
        "<div class=\"summary\">"
        "<span class=\"item\">总数 <b>{total}</b></span>"
        "<span class=\"item\">通过 <b class=\"pass\">{passed}</b></span>"
        "<span class=\"item\">失败 <b class=\"fail\">{failed}</b></span>"
        "<span class=\"item\">错误 <b class=\"error\">{errors}</b></span>"
        "<span class=\"item\">跳过 <b class=\"skipped\">{skipped}</b></span>"
        "<span class=\"item\">耗时 <b>{time}s</b></span>"
        "</div>\n"
        "<table><thead><tr><th style=\"width:40px\">#</th><th>用例</th><th style=\"width:90px\">结果</th></tr></thead>\n"
        "<tbody>\n{rows}</tbody></table>\n"
        "</body></html>\n"
    ).format(
        title="{0} {1} {2} - {3}".format(args.category_name, args.parent_module_name, args.module_name, label),
        total=total,
        passed=passed,
        failed=summary["failures"],
        errors=summary["errors"],
        skipped=summary["skipped"],
        time=summary["time"],
        rows=rows,
    )
    with open(html_path, "w", encoding="utf-8") as handle:
        handle.write(body)

    return {"state": state, "xml": xml_path, "json": json_path, "html": html_path}


def main():
    parser = argparse.ArgumentParser(description="Upload a test report")
    parser.add_argument("--url", default="http://192.168.3.3:31840")
    parser.add_argument("--file", required=True)
    parser.add_argument("--out-json", required=True)
    parser.add_argument("--report-dir", default="test-reports",
                        help="root directory for local XML/JSON/HTML artifacts, "
                             "grouped into a subdirectory per test type (default: test-reports)")
    parser.add_argument("--test-type", default="unit")
    parser.add_argument("--category-name", default="固定要地防护")
    parser.add_argument("--parent-module-name", default="Vega-Sim")
    parser.add_argument("--module-name", default="UE仿真")
    parser.add_argument("--title", default="")
    parser.add_argument("--report-base", default="",
                        help="base name for local report artifacts, without extension")
    parser.add_argument("--no-upload", action="store_true",
                        help="skip upload, only generate local artifacts")
    parser.add_argument("--merge", action="append", default=[],
                        help="额外 JUnit XML 合并进本报告(可重复); 用于把多个测试集并入同一 summary.*")
    args = parser.parse_args()

    if not os.path.isfile(args.file):
        sys.stderr.write("report file not found: {0}\n".format(args.file))
        return 2

    extra = [p for p in (args.merge or []) if p and os.path.isfile(p)]
    if extra:
        merged_data, summary = _merge_junit([args.file] + extra)
        tmp = tempfile.NamedTemporaryFile(prefix="merged-", suffix=".xml", delete=False)
        tmp.write(merged_data)
        tmp.close()
        args.file = tmp.name
        print("merged {0} report(s) into one (total {1} cases)".format(1 + len(extra), summary["tests"]))
    else:
        summary = _parse_junit(args.file)

    upload_base = "{0}-{1}-{2}-{3}-{4}".format(
        _safe_component(args.category_name),
        _safe_component(args.parent_module_name),
        _safe_component(args.module_name),
        _test_label(args.test_type),
        datetime.datetime.now().strftime("%Y%m%d%H%M%S"),
    )

    if args.no_upload:
        try:
            artifacts = write_report_artifacts(
                args, {"ok": True, "message": "local only"}, summary,
                args.report_base or upload_base
            )
            print("local artifacts: state={0}, xml={1}".format(artifacts["state"], artifacts["xml"]))
            print("  json: {0}".format(artifacts["json"]))
            print("  html: {0}".format(artifacts["html"]))
        except Exception as error:  # noqa: BLE001
            sys.stderr.write("write report artifacts failed: {0}\n".format(error))
            return 1
        return 0

    upload_filename = upload_base + ".xml"

    fields = {
        "test_type": args.test_type,
        "category_name": args.category_name,
        "parent_module_name": args.parent_module_name,
        "module_name": args.module_name,
    }
    fields["title"] = args.title or upload_base

    boundary, body = build_multipart(fields, args.file, upload_filename=upload_filename)
    url = args.url.rstrip("/") + "/api/upload"
    request = urllib.request.Request(url, data=body, method="POST")
    request.add_header("Content-Type", "multipart/form-data; boundary={0}".format(boundary))
    request.add_header("Accept", "application/json")

    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            raw = response.read().decode("utf-8", errors="replace")
            status = response.status
    except urllib.error.HTTPError as error:
        raw = error.read().decode("utf-8", errors="replace")
        status = error.code
    except Exception as error:  # noqa: BLE001
        sys.stderr.write("upload failed: {0}\n".format(error))
        try:
            write_report_artifacts(
                args, {"ok": False, "error": str(error)}, summary,
                args.report_base or upload_base
            )
        except Exception as artifact_error:  # noqa: BLE001
            sys.stderr.write("write report artifacts failed: {0}\n".format(artifact_error))
        return 1

    try:
        payload = json.loads(raw)
    except ValueError:
        payload = {"raw": raw}

    payload["http_status"] = status
    with open(args.out_json, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)

    try:
        artifacts = write_report_artifacts(args, payload, summary, args.report_base or upload_base)
        print("local artifacts: state={0}, xml={1}".format(artifacts["state"], artifacts["xml"]))
        print("  json: {0}".format(artifacts["json"]))
        print("  html: {0}".format(artifacts["html"]))
    except Exception as error:  # noqa: BLE001
        sys.stderr.write("write report artifacts failed: {0}\n".format(error))

    if status == 200 and payload.get("ok"):
        print("upload ok: report_id={0}, message={1}".format(
            payload.get("report_id"), payload.get("message")))
        return 0

    print("upload failed: status={0}, detail={1}".format(status, payload))
    return 1


if __name__ == "__main__":
    sys.exit(main())
