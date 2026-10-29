"""Run the CrossLedger evaluation suite and write CSV results."""

from __future__ import annotations

import argparse
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from crossledger.evaluation import run_all, summary_text, write_csv  # noqa: E402
from crossledger.geometry import build_path, closest_approach, remaining_polyline  # noqa: E402
from crossledger.security import MessageAuthenticator  # noqa: E402


class GeometryTests(unittest.TestCase):
    def test_right_turns_do_not_intersect(self):
        a = build_path("south", "right", 50.0)
        b = build_path("west", "right", 50.0)
        d, *_ = closest_approach(remaining_polyline(a.waypoints, 0.0), remaining_polyline(b.waypoints, 0.0))
        self.assertGreater(d, 4.5)

    def test_straights_do_intersect(self):
        a = build_path("south", "straight", 50.0)
        b = build_path("west", "straight", 50.0)
        d, *_ = closest_approach(remaining_polyline(a.waypoints, 0.0), remaining_polyline(b.waypoints, 0.0))
        self.assertLess(d, 4.5)

    def test_stopped_eta_no_zero_division(self):
        from crossledger.config import SimConfig
        from crossledger.vehicles import Vehicle

        v = Vehicle("A", "A", "south", "straight", 1, 0.0, 40.0, "#000")
        eta = v.eta_to_intersection(SimConfig())
        self.assertTrue(eta is None or eta == 0.0)


class SecurityTests(unittest.TestCase):
    def test_hmac_and_replay(self):
        auth = MessageAuthenticator(b"unit-test-key")
        payload = {
            "vehicle_id": "EX-101",
            "seq": 1,
            "speed_mps": 4.0,
        }
        env = auth.wrap(payload)
        self.assertTrue(auth.authenticate(env).accepted)
        self.assertFalse(auth.authenticate(env).accepted)
        env2 = auth.wrap({**payload, "seq": 2})
        env2["payload"]["speed_mps"] = 99.0
        result = auth.authenticate(env2)
        self.assertFalse(result.accepted)
        self.assertTrue(result.invalid_mac)


_CACHED_RESULTS = None


class ScenarioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        global _CACHED_RESULTS
        if _CACHED_RESULTS is None:
            _CACHED_RESULTS = run_all()
        cls.results = {r.name: r for r in _CACHED_RESULTS}

    def test_all_scenarios_pass(self):
        failed = [r.name for r in self.results.values() if not r.passed]
        self.assertEqual(failed, [], msg=str({k: v.detail for k, v in self.results.items() if not v.passed}))


def main() -> int:
    parser = argparse.ArgumentParser(description="CrossLedger evaluation suite")
    parser.add_argument(
        "--csv",
        default=os.path.join(ROOT, "results", "evaluation.csv"),
        help="CSV output path",
    )
    parser.add_argument("--skip-unittest", action="store_true")
    args = parser.parse_args()

    global _CACHED_RESULTS
    results = run_all()
    _CACHED_RESULTS = results
    path = write_csv(results, args.csv)
    print(summary_text(results), flush=True)
    print(f"\nWrote {path}", flush=True)
    print("Latency source: simulated in-process V2V channel (not a physical radio).", flush=True)

    if not args.skip_unittest:
        print("\nRunning unit tests...")
        suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
        runner = unittest.TextTestRunner(verbosity=2)
        outcome = runner.run(suite)
        if not outcome.wasSuccessful():
            return 1
    if any(not r.passed for r in results):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
