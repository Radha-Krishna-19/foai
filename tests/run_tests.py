"""
Runs the whole test suite and writes results/test_report.json (used by the testing document).
    python tests/run_tests.py
"""

import json
import os
import sys
import time
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
from tests._record import ACTUALS  # noqa: E402


class Recorder(unittest.TextTestResult):
    def startTest(self, test):
        self._t0 = time.perf_counter()
        super().startTest(test)

    def _add(self, test, status, detail=""):
        doc = (test.shortDescription() or test.id()).strip()
        tid, _, desc = doc.partition(":")
        self.rows.append({"id": tid.split()[0], "suite": test.__class__.__module__.split(".")[-1],
                          "description": desc.strip(), "status": status,
                          "actual": ACTUALS.get(tid.split()[0], ""), "seconds": round(time.perf_counter() - self._t0, 2),
                          "detail": detail})

    def addSuccess(self, test):
        super().addSuccess(test)
        self._add(test, "PASS")

    def addFailure(self, test, err):
        super().addFailure(test, err)
        self._add(test, "FAIL", self._exc_info_to_string(err, test)[-400:])

    def addError(self, test, err):
        super().addError(test, err)
        self._add(test, "ERROR", self._exc_info_to_string(err, test)[-400:])


def main():
    Recorder.rows = []
    suite = unittest.defaultTestLoader.discover(os.path.join(ROOT, "tests"), top_level_dir=ROOT)
    t0 = time.time()
    res = unittest.TextTestRunner(verbosity=2, resultclass=Recorder).run(suite)
    report = {"total": res.testsRun, "passed": sum(r["status"] == "PASS" for r in Recorder.rows),
              "failed": len(res.failures), "errors": len(res.errors), "seconds": round(time.time() - t0, 1),
              "python": sys.version.split()[0], "rows": Recorder.rows}
    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    with open(os.path.join(ROOT, "results", "test_report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=1)
    print(f"\n{report['passed']}/{report['total']} passed in {report['seconds']} s -> results/test_report.json")
    return 0 if res.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
