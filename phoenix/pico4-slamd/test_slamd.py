# SPDX-License-Identifier: Unlicense
"""pico4-slamd, PICO OS's head tracker's host in its bionic root: its build, cameras, rooms, depth mesh and floor."""

import math
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def read(path: Path) -> str:
    """Return a file's text."""
    return path.read_text(encoding="utf-8")


class FastRpcBridgeTests(unittest.TestCase):
    def test_the_fastrpc_bridge_is_built_and_tested(self):
        # The stock libraries' vendor FastRPC and ION calls, carried over to the upstream driver.
        build = read(ROOT / "build.sh")
        # Its translations are tested natively before the bionic library is built.
        self.assertLess(build.index('"$work/test_fastrpc_bridge"'), build.index('libpico4-fastrpc-bridge.so" \\'))
        bridge = read(ROOT / "fastrpc-bridge.c")
        for symbol in ("\nopen(", "\n__open_2(", "\nopenat(", "\n__openat_2(", "\nioctl(", "\nclose("):
            self.assertIn(symbol, bridge)
        # The bridge reaches the kernel by system calls, never through the functions it replaces.
        self.assertNotIn("dlsym", bridge)

    def test_build_script_stays_lf(self):
        self.assertNotIn(b"\r", (ROOT / "build.sh").read_bytes())


class SlamdThreadTests(unittest.TestCase):
    def test_images_on_their_own_thread(self):
        # As PICO OS: frames (checkIsSync) apart from the IMU (threadReader), so a set never holds
        # up the samples and their poses.
        slamd = read(ROOT / "pico4-slamd.c")
        frame = slamd.index("frame_thread(void *arg)")
        self.assertIn("c->image(c->h, 0, &frames);", slamd[frame : slamd.index("\n}\n", frame)])
        self.assertIn("pthread_create(&thread, 0, frame_thread, &frames)", slamd)
        main = slamd.index("main(int argc")
        self.assertNotIn("image(h,", slamd[main:])
        self.assertIn("__atomic_store_n(&frames.imu_fed_ts, e.ts_ns, __ATOMIC_RELEASE);", slamd)


class TemperatureTests(unittest.TestCase):
    """The IMU's die temperature with every sample, as PICO OS's sensor DSP inserts it."""

    def test_slamd_passes_only_what_libslam_accepts(self):
        slamd = read(ROOT / "pico4-slamd.c")
        self.assertIn("#define TEMPERATURE_MIN -30.0", slamd)
        self.assertIn("#define TEMPERATURE_MAX 100.0", slamd)
        self.assertIn("sample[6] = known ? e.temperature : DEFAULT_TEMPERATURE;", slamd)


class CameraOtpTests(unittest.TestCase):
    """The modules' ids as PICO OS's station service gets them on a fresh install."""

    def test_saved_on_the_data_partition_where_pico_os_keeps_them(self):
        slamd = read(ROOT / "pico4-slamd.c")
        self.assertIn('#define OTP_PATH "/data/stationservice/camera_otp.json"', slamd)
        # Saved only when PICO OS would keep them (all 8 or 9), and the saved ones offered first.
        self.assertIn("isp_otp_valid(ids) ? open(OTP_PATH", slamd)
        # Once saved, a driver started again while this pico4-slamd runs reads no OTP.
        self.assertIn("memcpy(feed->otp_saved, ids, sizeof(ids));", slamd)
        self.assertLess(
            slamd.index("memcpy(feed->otp_saved, saved, sizeof(saved));"),
            slamd.index("load_tables(camera_vendor(feed, generation))"),
        )
        self.assertIn("isp_lsc_file(vendor)", slamd)


class AutoExposureTests(unittest.TestCase):
    """PICO OS's auto exposure in pico4-slamd, the station commands in the driver."""

    def test_slamd_calls_it_as_pico_os(self):
        slamd = read(ROOT / "pico4-slamd.c")
        self.assertIn('dlopen("libExposureImageProcess.so", RTLD_NOW)', slamd)
        self.assertIn('dlsym(lib, "ExposurePredict")', slamd)
        # Images after black level and lens shading, before gamma; 16-bit at the 50 Hz rate.
        self.assertIn("to_ae ? g_linear[k] : 0", slamd)
        self.assertIn("#define AE_FPS 50", slamd)
        self.assertIn("#define AE_BIT_WIDTH 16", slamd)
        # Nothing sent when it is what the set was taken with, or what was sent last.
        self.assertIn("if (!own && !same)", slamd)
        self.assertIn("__atomic_fetch_add(&feed->ae_written, 1, __ATOMIC_RELEASE);", slamd)
        self.assertIn("__atomic_store_n(&feed->ae_generation, 0, __ATOMIC_RELEASE);", slamd)
        android = read(ROOT / "android.c")
        for name in ("allocate", "lock", "unlock", "release", "sendHandleToUnixSocket", "recvHandleFromUnixSocket"):
            self.assertIn(f"AHardwareBuffer_{name}(", android)


class RoomTests(unittest.TestCase):
    """PICO OS's map persistence and a SteamVR universe per room."""

    def test_slamd_saves_every_map_with_its_rooms_shape(self):
        slamd = read(ROOT / "pico4-slamd.c")
        self.assertIn("room_decode((const double(*)[3])g_seen, 4, &id, sqrt) == 0", slamd)
        self.assertIn("room_fit((const double(*)[3])g_seen, (const double(*)[3])shape, 4,", slamd)
        self.assertIn("write_boundary(SESSION_BOUNDARY, (const double(*)[3])shape, 4);", slamd)
        # The universe comes from the room's id, not from the tracker's map name.
        self.assertIn("room_identity(id, feed->room_uuid, sizeof(feed->room_uuid));", slamd)
        self.assertIn("room_table_add(id, map);", slamd)
        # A copy left from an earlier session must not count as this one's relocalisation.
        self.assertLess(slamd.index("unlink(RELOCALISED_BOUNDARY);"), slamd.index('void *h = t->create("");'))
        # The clean stop that saves the map, as PICO OS's service stops the tracker.
        stop = slamd.index("t->prestop(h);")
        self.assertLess(stop, slamd.index('t->stop(h, "non_sleep");'))
        self.assertLess(slamd.index('t->stop(h, "non_sleep");'), slamd.index("t->destroy(h);"))

    def test_slamd_restarts_a_lost_tracker_as_pico_os(self):
        # PICO OS's tracking service restarts a tracker whose confidence fell from 1 to 0
        # and stayed 0 for more than restart_tracking_sec (10).
        slamd = read(ROOT / "pico4-slamd.c")
        self.assertIn("#define RESTART_DEFAULT_S 10", slamd)
        self.assertIn("if (p->confidence == 0 && last_confidence == 1) {", slamd)
        self.assertIn("lost_since = now_ns(CLOCK_BOOTTIME);", slamd)
        restart = slamd.index("restarting the tracker")
        self.assertLess(slamd.index("pthread_join(thread, 0);", restart), slamd.index("tracker_stop(&t, h);", restart))
        self.assertLess(
            slamd.index("tracker_stop(&t, h);", restart), slamd.index("h = tracker_start(&t, feed);", restart)
        )

    def test_slamd_takes_a_late_relocalisation(self):
        # The room settles when the tracker writes its copy of the saved room's boundary, also after
        # a new room was decided. libSlam's callback for this does not come here.
        slamd = read(ROOT / "pico4-slamd.c")
        self.assertNotIn('dlsym(lib, "RelocResultCallbackRegister")', slamd)
        start = slamd.index("tracker_start(const struct tracker *t")
        self.assertLess(slamd.index("unlink(RELOCALISED_BOUNDARY);", start), slamd.index("t->create(", start))
        loop = slamd.index("relocalised in a saved map")
        self.assertLess(
            slamd.index("if (!relocalised && read_boundary(RELOCALISED_BOUNDARY, g_seen) == 4) {", loop - 400), loop
        )
        self.assertLess(
            loop, slamd.index("if (!room_settled && input_since != 0 && now - input_since > relocalise_ns) {")
        )
        settle = slamd.index("settle_room(struct pico4_slam_feed *feed")
        self.assertLess(
            slamd.index("__atomic_store_n(&feed->room_state, 0, __ATOMIC_RELEASE);", settle),
            slamd.index("__atomic_store_n(&feed->room_state, state, __ATOMIC_RELEASE);", settle),
        )

    def test_the_time_to_relocalise_runs_from_the_trackers_first_frames(self):
        # Cameras that start 10 s late after a failed sync must not decide a new room unseen.
        slamd = read(ROOT / "pico4-slamd.c")
        self.assertIn("#define RELOCALISE_DEFAULT_S 10", slamd)
        self.assertIn(
            "if (input_since == 0 && __atomic_load_n(&frames.sets_fed, __ATOMIC_RELAXED) != sets_at_start) {", slamd
        )
        # A restarted tracker starts over: its copy is removed and its first frames count again.
        restart = slamd.index("restarting the tracker")
        for reset in ("relocalised = false;", "sets_at_start = __atomic_load_n(&frames.sets_fed", "input_since = 0;"):
            self.assertLess(restart, slamd.index(reset, restart))


class FloorTests(unittest.TestCase):
    """The floor libSlam detects, to SteamVR's room setup as the Frame's driver_cv sends its own."""

    def test_slamd_passes_the_ground_with_each_pose(self):
        slamd = read(ROOT / "pico4-slamd.c")
        self.assertIn("p->floor_status = (uint32_t)plane_status;", slamd)
        self.assertIn("p->floor_height = plane_height;", slamd)
        self.assertLess(
            slamd.index("p->floor_height = plane_height;"),
            slamd.index("__atomic_store_n(&feed->poses_written, w + 1, __ATOMIC_RELEASE);"),
        )


class DepthMeshTests(unittest.TestCase):
    """PICO OS's see-through depth mesh (libphotoid) in pico4-slamd, fed as its see-through service."""

    def test_head_and_points_go_in_y_up(self):
        # libphotoid takes the head and the map points in libSlam's world turned (-y, x, z): fed
        # unchanged, a head turned right put the hemisphere below the horizon.
        vst = read(ROOT / "vst.c")
        turn = vst[vst.index("\nto_y_up(") :]
        turn = turn[: turn.index("\n}\n")]
        self.assertIn("v[0] = -v[1];\n\tv[1] = x;", turn)
        feed = vst[vst.index("\nvst_feed(") :]
        feed = feed[: feed.index("\n}\n")]
        self.assertIn("to_y_up(position);\n\tto_y_up(orientation + 1);", feed)
        self.assertIn("f->head = head;", feed)
        points = vst[vst.index("\nvst_points(") :]
        points = points[: points.index("\n}\n")]
        self.assertIn("p[0] = -p[1];\n\t\t\tp[1] = x;", points)
        self.assertIn("g_vst.points(&turned);", points)
        # The turn keeps a quaternion's rotation consistent: q' = (w, C v) is C R C^T.
        w, x, y, z = 0.82, -0.56, 0.05, -0.02
        n = math.sqrt(w * w + x * x + y * y + z * z)
        w, x, y, z = w / n, x / n, y / n, z / n

        def rot(w: float, x: float, y: float, z: float) -> list[list[float]]:
            return [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]

        c = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        r = rot(w, x, y, z)
        c_r_ct = [
            [sum(c[i][a] * r[a][b] * c[j][b] for a in range(3) for b in range(3)) for j in range(3)] for i in range(3)
        ]
        turned = rot(w, -y, x, z)
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(turned[i][j], c_r_ct[i][j])

    def test_cvp_session_answered_without_the_hardware(self):
        # vst_releaseA opens a CVP session and takes its depth from FLNet on the Compute DSP.
        stubs = read(ROOT / "stubs.c")
        for name in (
            "cvpCreateSession",
            "cvpStartSession",
            "cvpStopSession",
            "cvpDeleteSession",
            "cvpInitDfs",
            "cvpDeInitDfs",
            "cvpDfs_Sync",
            "cvpMemAlloc",
            "cvpMemFree",
        ):
            self.assertIn(f"{name}(", stubs, name)
        # Its image sizes stay libcvp2's own calculation.
        self.assertNotIn("cvpQueryImageInfo(", stubs)
        # A run of the depth from stereo fails loudly: nothing here computes it.
        sync = stubs[stubs.index("cvpDfs_Sync(") :]
        self.assertIn("return -1;", sync[: sync.index("cvpMemAlloc(")])
        self.assertNotIn("cvp_restart_related", read(ROOT / "vst.c"))

    def test_slamd_feeds_it_as_pico_os(self):
        slamd = read(ROOT / "pico4-slamd.c")
        vst = read(ROOT / "vst.c")
        # libSlam's ground detection: the floor in every result, which libphotoid takes as its ground.
        self.assertIn('t->set(h, "plane", "1");', slamd)
        # The forward pair (cam3 left, cam2 right) with the tracker's result at the pair's time.
        self.assertIn(
            "vst_feed(g_images[3], g_images[2], PICO4_SLAM_FEED_W, PICO4_SLAM_FEED_H, ts, exposure_ns, head);", slamd
        )
        # The map points only while the tracker tracks (type 1, confidence 1).
        self.assertIn("head[0x40] == 1 && head[0x41] == 1", slamd)
        self.assertIn('dlsym(lib, "AlgorithmPointCallbackRegister")', slamd)
        # Only while the driver asks for meshes.
        self.assertIn("__atomic_load_n(&feed->mesh_wanted_ns, __ATOMIC_ACQUIRE) <", slamd)
        # The buffers before the start: the pipeline writes its first mesh without a check.
        self.assertLess(vst.index("mesh_buffer(g_vst.vertices"), vst.index("int ok = start();"))
        self.assertIn('"$src/crt.c" "$src/pico4-slamd.c" "$src/vst.c"', read(ROOT / "build.sh"))


class BuildTests(unittest.TestCase):
    def test_the_tracker_links_against_bionic_stand_ins_not_pico_os_s_libraries(self):
        build = read(ROOT / "build.sh")
        self.assertIn('libs=(-L"$stubs")', build)
        self.assertNotIn("system/lib64", build)
        stubs = read(ROOT / "bionic-stubs.c")
        for lib in ("LIBC", "LIBM", "LIBDL"):
            self.assertIn(f"defined(STUB_{lib})", stubs)


class StationRateTests(unittest.TestCase):
    """The cameras' rate: 50 Hz as stock, or PICO OS's 60 Hz for a faster room view."""

    def test_auto_exposure_takes_the_measured_rate(self):
        slamd = (ROOT / "pico4-slamd.c").read_text(encoding="utf-8")
        self.assertIn("ts - c->last_set_ts < 36700000 ? 60 : AE_FPS", slamd)
        self.assertIn("&unused, __atomic_load_n(&c->fps, __ATOMIC_RELAXED),", slamd)


if __name__ == "__main__":
    unittest.main()
