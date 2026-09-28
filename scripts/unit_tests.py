#!/usr/bin/env python3
"""Run the INAV SITL Python unit tests and emit a JUnit XML report.

Stdlib only. unittest prints to a terminal but does not serialise JUnit XML,
which is the format the report platform (scripts/upload_report.py) consumes.
The small reporter below bridges the two: it subclasses unittest's result
object to capture each case's outcome and writes a <testsuites> document whose
shape matches what the uploader parses.

Usage:
    scripts/unit_tests.py                        # console summary only
    scripts/unit_tests.py --xml unit_tests.xml   # also write JUnit XML
    scripts/unit_tests.py -k BuildRequest -v     # filter + verbose
"""

import argparse
import os
import sys
import time
import traceback
import unittest
import xml.etree.ElementTree as ET

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _short_message(err):
    exctype, value = err[0], err[1]
    text = str(value)
    return "{0}: {1}".format(exctype.__name__, text) if text else exctype.__name__


def _text(err):
    return "".join(traceback.format_exception(*err))


class JUnitResult(unittest.TextTestResult):
    """A TextTestResult that also records each case for JUnit serialisation."""

    def __init__(self, stream, descriptions, verbosity):
        super().__init__(stream, descriptions, verbosity)
        self.records = []

    def startTest(self, test):
        self._started = time.perf_counter()
        super().startTest(test)

    def _elapsed(self):
        return max(time.perf_counter() - self._started, 0.0)

    def _record(self, test, kind, err=None, reason=None):
        classname, _, name = test.id().rpartition(".")
        self.records.append({
            "classname": classname,
            "name": name,
            "time": self._elapsed(),
            "kind": kind,
            "message": reason if reason is not None else (_short_message(err) if err else ""),
            "text": _text(err) if err else "",
        })

    def addSuccess(self, test):
        super().addSuccess(test)
        self._record(test, "pass")

    def addFailure(self, test, err):
        super().addFailure(test, err)
        self._record(test, "failure", err)

    def addError(self, test, err):
        super().addError(test, err)
        self._record(test, "error", err)

    def addSkip(self, test, reason):
        super().addSkip(test, reason)
        self._record(test, "skipped", reason=reason)

    def addExpectedFailure(self, test, err):
        super().addExpectedFailure(test, err)
        self._record(test, "skipped", reason="expected failure")

    def addUnexpectedSuccess(self, test):
        super().addUnexpectedSuccess(test)
        self._record(test, "failure", reason="unexpected success")


def _iter_tests(suite):
    for item in suite:
        if isinstance(item, unittest.TestSuite):
            yield from _iter_tests(item)
        else:
            yield item


def build_xml(records, suite_name):
    total = len(records)
    failures = sum(1 for r in records if r["kind"] == "failure")
    errors = sum(1 for r in records if r["kind"] == "error")
    skipped = sum(1 for r in records if r["kind"] == "skipped")
    duration = sum(r["time"] for r in records)

    attrs = {
        "name": suite_name,
        "tests": str(total),
        "failures": str(failures),
        "errors": str(errors),
        "skipped": str(skipped),
        "time": "{0:.6f}".format(duration),
    }
    testsuites = ET.Element("testsuites", attrs)
    testsuite = ET.SubElement(testsuites, "testsuite", attrs)

    for record in records:
        case = ET.SubElement(testsuite, "testcase", {
            "classname": record["classname"],
            "name": record["name"],
            "time": "{0:.6f}".format(record["time"]),
        })
        if record["kind"] == "failure":
            ET.SubElement(case, "failure", {"message": record["message"]}).text = record["text"]
        elif record["kind"] == "error":
            ET.SubElement(case, "error", {"message": record["message"]}).text = record["text"]
        elif record["kind"] == "skipped":
            ET.SubElement(case, "skipped", {"message": record["message"]})

    return ET.ElementTree(testsuites)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--xml", help="write the JUnit XML report to this path")
    parser.add_argument("--start-dir", default=SCRIPT_DIR,
                        help="directory to discover tests in (default: %(default)s)")
    parser.add_argument("--pattern", default="test_*.py")
    parser.add_argument("--suite-name", default="INAV SITL.Unit")
    parser.add_argument("-k", dest="name_filter", default="",
                        help="only run tests whose id contains this substring")
    parser.add_argument("-v", "--verbose", action="count", default=0)
    args = parser.parse_args()

    loader = unittest.TestLoader()
    discovered = loader.discover(
        start_dir=args.start_dir, pattern=args.pattern, top_level_dir=args.start_dir
    )
    tests = list(_iter_tests(discovered))
    if args.name_filter:
        tests = [t for t in tests if args.name_filter in t.id()]
    suite = unittest.TestSuite(tests)

    print("running {0} test(s) from {1}".format(len(tests), args.start_dir))
    runner = unittest.TextTestRunner(
        stream=sys.stdout, verbosity=1 + args.verbose, resultclass=JUnitResult
    )
    result = runner.run(suite)

    summary = "tests={0} failures={1} errors={2} skipped={3} time={4:.3f}s".format(
        result.testsRun, len(result.failures), len(result.errors),
        len(result.skipped), sum(r["time"] for r in result.records),
    )
    print(summary)

    if args.xml:
        build_xml(result.records, args.suite_name).write(
            args.xml, encoding="utf-8", xml_declaration=True
        )
        print("JUnit XML written to {0}".format(args.xml))

    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
