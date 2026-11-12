# SPDX-License-Identifier: Unlicense
"""msm-frame-stats: frame statistics from the msm display driver's trace events."""

import importlib.machinery
import importlib.util
import json
import unittest
from collections.abc import Iterable
from pathlib import Path

HERE = Path(__file__).resolve().parent
PATH = HERE / "msm-frame-stats"
_loader = importlib.machinery.SourceFileLoader("msm_frame_stats", str(PATH))
_spec = importlib.util.spec_from_loader("msm_frame_stats", _loader)
assert _spec is not None
stats = importlib.util.module_from_spec(_spec)
_loader.exec_module(stats)
PERF = HERE / "session-performance"
_ploader = importlib.machinery.SourceFileLoader("session_performance_msm", str(PERF))
_pspec = importlib.util.spec_from_loader("session_performance_msm", _ploader)
assert _pspec is not None
perf = importlib.util.module_from_spec(_pspec)
_ploader.exec_module(perf)

REFRESH = 1 / 90


def trace(flip_times: Iterable[float], vblank_count: int, jobs: Iterable[tuple[int, float]]) -> list[str]:
    """Return tracefs lines as msm's events print them (formats from the 7.2 kernel)."""
    lines = [
        f"          <idle>-0       [000] d.h1.  {100 + i * REFRESH:.6f}: drm_vblank_event: "
        f"crtc=0, seq={i}, time={i}, high-prec=true"
        for i in range(vblank_count)
    ]
    lines.extend(f"     kworker/0:1-42      [000] d.h1.  {t:.6f}: dpu_crtc_complete_flip: id=95" for t in flip_times)
    lines.extend(
        f"    RenderThread-{pid}    [001] d..1.  101.000000: msm_gpu_submit_retired: id=1 pid={pid} "
        f"ring=0:5 elapsed={int(elapsed_ms * 1e6)} ns mhz=587 start=1 end=2"
        for pid, elapsed_ms in jobs
    )
    return lines


class MsmFrameStatsTests(unittest.TestCase):
    def test_job_margins_are_taken_before_each_vblank(self):
        vblanks = [100 + i * REFRESH for i in range(200)]
        ends = [v - 0.007 for v in vblanks[1:]]
        ends[50] = vblanks[51] - 0.0005  # one job that ends just in time
        del ends[100]  # one vblank without a job
        margins = stats.job_margins(ends, vblanks, REFRESH)
        self.assertEqual(margins["frames"], 199)
        self.assertEqual(margins["without_a_job"], 1)
        self.assertAlmostEqual(margins["p50"], 7.0, places=1)
        self.assertLess(margins["p99"], 1.0)

    def test_a_little_margin_fails_the_check_and_the_web_helpers_share_is_reported(self):
        flips = [100 + i * REFRESH for i in range(901)]
        end = stats.parse(trace(flips, 901, [(700, 5.9)]), {700})
        end["job_margin_ms"] = {"p50": 5.5, "p99": 0.0, "frames": 900, "without_a_job": 12}
        end["web_helper_gpu_percent"] = 12.5
        v = perf.verdict(end, 60.0)
        self.assertEqual(v["status"], "failed")
        self.assertIn("below 1.0 ms", v["failures"][0])
        self.assertEqual(v["web_helper_gpu_percent"], 12.5)
        end["job_margin_ms"]["p99"] = 6.3
        self.assertEqual(perf.verdict(end, 60.0)["status"], "passed")

    def test_a_window_without_a_panel_passes_with_a_warning(self):
        flips = [100 + i * REFRESH for i in range(901)]
        end = stats.parse(trace(flips, 901, [(700, 5.9)]), {700})
        end["job_margin_ms"] = {"p50": 7.1, "p99": 6.3, "frames": 900, "without_a_job": 0}
        end["web_helper_gpu_percent"] = 0.0
        v = perf.verdict(end, 60.0)
        self.assertEqual(v["status"], "passed")
        self.assertIn("no Steam panel was tested", v["warnings"][0])
        end["web_helper_gpu_percent"] = 12.5
        self.assertEqual(perf.verdict(end, 60.0)["warnings"], [])

    def test_the_web_helpers_threads_count_apart_from_the_compositors(self):
        flips = [100 + i * REFRESH for i in range(91)]
        lines = trace(flips, 91, [(700, 5.9), (800, 500.0)])
        s = stats.parse(lines, {700}, {800})
        self.assertEqual(s["gpu_jobs"], 1)
        self.assertAlmostEqual(s["web_helper_gpu_percent"], 50.0, places=0)

    def test_flips_vblanks_and_the_compositors_gpu_time(self):
        flips = [100 + i * REFRESH for i in range(91)]
        flips[50:] = [t + REFRESH for t in flips[50:]]  # one late flip (two refreshes)
        lines = trace(flips, 181, [(700, 5.9), (702, 6.1), (701, 12.0)])
        s = stats.parse(lines, {700, 702})
        self.assertEqual(s["target"], "msm")
        self.assertEqual(s["commits"], 91)
        self.assertEqual(s["late"], 1)
        self.assertEqual(s["stalls"], 0)
        self.assertAlmostEqual(s["vblank_hz"], 90.0, places=1)
        self.assertAlmostEqual(s["max_interval_ms"], 2 * REFRESH * 1e3, places=2)
        self.assertEqual(s["gpu_jobs"], 2)  # thread 701 belongs to another process
        self.assertAlmostEqual(s["gpu_ms"]["mean"], 6.0, places=3)

    def test_performance_judges_the_recording(self):
        flips = [100 + i * REFRESH for i in range(900)]
        end = stats.parse(trace(flips, 900, [(700, 6.0)]), {700})
        v = perf.verdict(end, 60.0)
        self.assertEqual(v["status"], "passed", v["failures"])
        self.assertAlmostEqual(v["commit_hz"], 90.0, delta=0.2)
        self.assertEqual(v["gpu_ms"]["mean"], 6.0)
        json.dumps(v)

    def test_stalls_and_no_frames(self):
        s = stats.parse(trace([100.0, 100.2], 3, []), {700})
        self.assertEqual(s["stalls"], 1)
        self.assertNotIn("gpu_ms", s)
        self.assertEqual(stats.parse([], set())["commits"], 0)


def full_trace(frames: int, late: int) -> list[str]:
    """Return a trace command's lines: one compositor job a refresh, the one of refresh LATE submitted late.

    The compositor's RenderThread (700) wakes 1 ms after each vblank on core 5, submits 0.2 ms later,
    and its job runs 6 ms on the GPU after a 0.1 ms flush. In refresh LATE a web helper (800) holds
    core 5 for 6 ms first, so the job ends after the vblank; a web helper job (801) retires in it too.
    """
    lines = ["# compositor: 700 701", "# helpers: 800 801"]
    idle, render = ("<idle>", 0), ("RenderThread", 700)

    def at(t: float, who: tuple[str, int], cpu: int, event: str, fields: str) -> None:
        lines.append(f"{who[0]:>16}-{who[1]:<7} [{cpu:03d}] d..2.  {t:.6f}: {event}: {fields}")

    def switch(t: float, out: tuple[str, int], into: tuple[str, int]) -> None:
        fields = f"prev_comm={out[0]} prev_pid={out[1]} prev_prio=120 prev_state=R"
        at(t, out, 5, "sched_switch", f"{fields} ==> next_comm={into[0]} next_pid={into[1]} next_prio=0")

    def retired(t: float, ident: int, pid: int, ns: int) -> None:
        at(t, idle, 0, "msm_gpu_submit_retired", f"id={ident} pid={pid} ring=0:1 elapsed={ns} ns mhz=587 start=1 end=2")

    for n in range(frames + 1):
        at(100 + n * REFRESH, idle, 0, "drm_vblank_event", f"crtc=0, seq={n}, time={n}, high-prec=true")
    for n in range(frames):
        v = 100 + n * REFRESH
        wake = v + 0.001
        held = 0.006 if n == late else 0.0
        at(wake, idle, 5, "sched_wakeup", "comm=RenderThread pid=700 prio=0 target_cpu=005")
        helper = ("steamwebhelper", 800)
        if held:
            switch(wake, ("swapper/5", 0), helper)
        switch(wake + held, helper if held else ("swapper/5", 0), render)
        sub = wake + held + 0.0002
        at(sub, render, 5, "msm_gpu_submit", f"id={n} pid=700 ring=0 bos=3 cmds=1")
        at(sub + 0.0001, ("kworker/u50:0", 41), 6, "msm_gpu_submit_flush", f"id={n} pid=700 ring=0:{n} ticks=1")
        retired(sub + 0.0001 + 0.006, n, 700, 6000000)
        if n == late:
            retired(v + 0.004, 9999, 801, 2000000)
    return lines


class MissesTests(unittest.TestCase):
    def test_a_late_submit_names_who_held_the_core(self):
        result = stats.misses(full_trace(60, 30))
        self.assertEqual(result["misses"], 1)
        self.assertEqual(result["refreshes"], 60)  # the vblanks after the first job's
        miss = result["miss_list"][0]
        self.assertGreater(miss["late_ms"], 0)
        self.assertAlmostEqual(miss["submit_after_vblank_ms"], 7.2, places=2)
        self.assertAlmostEqual(miss["submit_wait_ms"], 0.1, places=2)
        self.assertAlmostEqual(miss["gpu_ms"], 6.0, places=2)
        self.assertAlmostEqual(miss["cpu_held_by_ms"]["steamwebhelper"], 6.0, places=2)
        self.assertEqual(miss["other_gpu_ms"], {"801": 2.0})
        self.assertAlmostEqual(result["good_medians"]["submit_after_vblank_ms"], 1.2, places=2)
        self.assertAlmostEqual(result["cpu_held_by_ms_total"]["steamwebhelper"], 6.0, places=2)
        json.dumps(result)

    def test_no_compositor_jobs_no_misses(self):
        self.assertEqual(stats.misses(["# compositor: 1"])["misses"], 0)


if __name__ == "__main__":
    unittest.main()
