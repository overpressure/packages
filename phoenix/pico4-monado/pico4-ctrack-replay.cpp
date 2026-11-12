// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-ctrack-replay: run the controller tracker over recordings of pico4-cam
 * capture --record-leds / --record-imu and print what it did (JSON).
 *
 *   pico4-ctrack-replay [--calib DIR] [--slots 0,2,1,3] [--head-up X,Y,Z | --estimate-head-up]
 *                       [--trace FILE] [--max-imu-gap S] LEDS.bin IMU.bin
 *
 * --calib holds the factory camera calibration (pvr/mav0/cam<k>/sensor.yaml) and the LED
 * models (controllers/led-models.json); default /usr/share/qcom/pico4/calibration. The head
 * is taken as still for the whole recording: its pose is the rotation that turns its up
 * direction in the headset body frame into the tracking world's +x. That direction is the mean
 * of the head accelerometer in the IMU recording, or --head-up. --estimate-head-up measures it
 * instead from frames where a controller lies still (for recordings without the head IMU): its
 * accelerometer gives up in its model frame, and an identification-
 * table solve over five or more LEDs gives its rotation in the headset frame. --trace writes one
 * CSV line per frame set and hand. --max-imu-gap replaces stock's 35 ms limit between controller IMU
 * samples (beyond it tracking resets), to separate the tracker from gaps in the radio's samples.
 * --delay left|right S drops that hand's IMU samples of the first S seconds, so that the other
 * hand is tracked first (acquisition next to a tracked hand). --skip S drops the LED frame sets
 * of the first S seconds while the IMU runs, as when the driver starts the cameras after the IMU.
 *
 * Runs on the headset and on a development host alike.
 */

#include "pico4_blemcu.h"
#include "pico4_ctrack_calib.hpp"
#include "pico4_ctrack_engine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace pico4::ctrack;

namespace {

constexpr double kQTimerHz = 19200000.0;
constexpr size_t kLedHeader = 48;

struct LedFrame
{
	int slot = 0;
	int64_t mono_ns = 0;
	int width = 0, height = 0;
	std::vector<uint8_t> pixels;
};

struct FrameSet
{
	int64_t t_ns = 0;
	std::array<const LedFrame *, kCameras> cam{};
};

bool
read_file(const std::string &path, std::vector<uint8_t> &out)
{
	std::ifstream f(path, std::ios::binary);
	if (!f) {
		return false;
	}
	out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
	return true;
}

template <typename T>
T
le(const uint8_t *p)
{
	T v;
	memcpy(&v, p, sizeof(v));
	return v;
}

bool
read_led_frames(const std::string &path, std::vector<LedFrame> &frames)
{
	std::vector<uint8_t> data;
	if (!read_file(path, data)) {
		return false;
	}
	for (size_t pos = 0; pos + kLedHeader <= data.size();) {
		const uint8_t *h = &data[pos];
		if (memcmp(h, "P4LF", 4) != 0 || h[4] != 1) {
			fprintf(stderr, "%s: no P4LF v1 record at offset %zu\n", path.c_str(), pos);
			return false;
		}
		LedFrame f;
		f.slot = h[5];
		f.mono_ns = (int64_t)le<uint64_t>(h + 24);
		f.width = le<uint16_t>(h + 36);
		f.height = le<uint16_t>(h + 38);
		size_t n = (size_t)f.width * f.height;
		if (pos + kLedHeader + n > data.size()) {
			break;
		}
		f.pixels.assign(h + kLedHeader, h + kLedHeader + n);
		frames.push_back(std::move(f));
		pos += kLedHeader + n;
	}
	return true;
}

//! Frame sets: one frame of each slot within 2 ms (one station pulse).
std::vector<FrameSet>
assemble(const std::vector<LedFrame> &frames, const int slots[kCameras])
{
	std::vector<const LedFrame *> sorted;
	for (const LedFrame &f : frames) {
		if (f.width == kImageWidth && f.height == kImageHeight) {
			sorted.push_back(&f);
		}
	}
	std::sort(sorted.begin(), sorted.end(), [](const LedFrame *a, const LedFrame *b) { return a->mono_ns < b->mono_ns; });
	std::vector<FrameSet> sets;
	for (size_t i = 0; i < sorted.size();) {
		size_t j = i;
		std::map<int, const LedFrame *> by_slot;
		while (j < sorted.size() && sorted[j]->mono_ns - sorted[i]->mono_ns < 2000000) {
			by_slot.emplace(sorted[j]->slot, sorted[j]);
			j++;
		}
		FrameSet s;
		bool complete = true;
		for (int k = 0; k < kCameras; k++) {
			auto it = by_slot.find(slots[k]);
			complete = complete && it != by_slot.end();
			s.cam[k] = complete ? it->second : nullptr;
		}
		if (complete) {
			s.t_ns = sorted[i]->mono_ns;
			sets.push_back(s);
		}
		i = j;
	}
	return sets;
}

/*!
 * The controllers' IMU samples on CLOCK_MONOTONIC, per hand, in time order, and the sum and
 * count of the head accelerometer samples (headset body frame), if recorded.
 */
bool
read_imu(const std::string &path, std::array<std::vector<ImuSample>, 2> &out, Vec3 &head_accel_sum, int &head_accel_n)
{
	head_accel_sum.setZero();
	head_accel_n = 0;
	std::vector<uint8_t> data;
	if (!read_file(path, data)) {
		return false;
	}
	std::vector<double> offsets; // CLOCK_MONOTONIC minus QTimer time, ns
	std::vector<std::pair<uint64_t, std::vector<uint8_t>>> packets;
	for (size_t pos = 0; pos + 8 <= data.size();) {
		const uint8_t *h = &data[pos];
		if (memcmp(h, "P4L", 3) != 0) {
			fprintf(stderr, "%s: no record at offset %zu\n", path.c_str(), pos);
			return false;
		}
		uint16_t n = le<uint16_t>(h + 6);
		if (pos + 8 + n > data.size()) {
			break;
		}
		if (h[3] == 'C' && n == 16) {
			offsets.push_back((double)le<uint64_t>(h + 8) - (double)le<uint64_t>(h + 16) * 1e9 / kQTimerHz);
		} else if (h[3] == 'I' && n > 8) {
			packets.emplace_back(le<uint64_t>(h + 8), std::vector<uint8_t>(h + 16, h + 8 + n));
		} else if (h[3] == 'H' && n == 21 && h[16] == 0) {
			head_accel_sum += Vec3(le<float>(h + 17), le<float>(h + 21), le<float>(h + 25));
			head_accel_n++;
		}
		pos += 8 + n;
	}
	if (offsets.empty()) {
		fprintf(stderr, "%s: no clock records\n", path.c_str());
		return false;
	}
	std::nth_element(offsets.begin(), offsets.begin() + offsets.size() / 2, offsets.end());
	double offset = offsets[offsets.size() / 2];
	for (auto &[ticks, raw] : packets) {
		pico4_blemcu_packet p;
		if (!pico4_blemcu_parse(raw.data(), raw.size(), &p)) {
			continue;
		}
		double event_ns = (double)ticks * 1e9 / kQTimerHz + offset;
		for (int i = 0; i < p.imu_count; i++) {
			const pico4_blemcu_imu &m = p.imu[i];
			if (m.device != PICO4_BLEMCU_LEFT && m.device != PICO4_BLEMCU_RIGHT) {
				continue;
			}
			ImuSample s;
			s.t_ns = (int64_t)(event_ns - m.offset_us * 1000.0);
			for (int a = 0; a < 3; a++) {
				s.gyro[a] = m.gyro[a] / PICO4_BLEMCU_GYRO_LSB_PER_DPS * M_PI / 180.0;
				s.accel[a] = m.accel[a] / PICO4_BLEMCU_ACCEL_LSB_PER_G * 9.80665;
			}
			s.temperature = m.temperature / 132.48 + 25.0;
			out[m.device == PICO4_BLEMCU_LEFT ? kLeft : kRight].push_back(s);
		}
	}
	for (auto &v : out) {
		std::stable_sort(v.begin(), v.end(), [](const ImuSample &a, const ImuSample &b) { return a.t_ns < b.t_ns; });
		v.erase(std::unique(v.begin(), v.end(), [](const ImuSample &a, const ImuSample &b) { return a.t_ns == b.t_ns; }),
		        v.end());
	}
	return true;
}

ImageView
view(const LedFrame *f)
{
	return ImageView{f->pixels.data(), f->width, f->height, f->width};
}

//! The head's rotation for an up direction in the headset frame: up -> W +x.
Pose
head_pose(const Vec3 &up_H)
{
	return Pose(Quat::FromTwoVectors(up_H.normalized(), Vec3::UnitX()), Vec3::Zero());
}

/*!
 * Up in the headset frame, measured: in frame sets where a controller is still (IMU around the
 * frame: gyro < 0.05 rad/s, |accel| within 0.3 m/s^2 of g), its accelerometer gives up in M,
 * and the best identification-table hypothesis (EPnP over five blobs, mean error < 0.5 px)
 * gives R_HM. The mean of R_HM up_M over all such frames.
 */
bool
estimate_head_up(const std::array<Camera, kCameras> &cams, const LedModel models[2],
                 const std::vector<FrameSet> &sets, const std::array<std::vector<ImuSample>, 2> &imu, Vec3 &up_H,
                 int &used)
{
	IdTable table;
	table.generate(models[0]);
	Vec3 sum = Vec3::Zero();
	used = 0;
	for (const FrameSet &s : sets) {
		for (int hand = 0; hand < 2; hand++) {
			Vec3 acc = Vec3::Zero();
			int n = 0;
			bool still = true;
			for (const ImuSample &m : imu[hand]) {
				if (std::llabs(m.t_ns - s.t_ns) > 50000000) {
					continue;
				}
				still = still && m.gyro.norm() < 0.05 && std::abs(m.accel.norm() - 9.80665) < 0.3;
				acc += m.accel;
				n++;
			}
			if (n < 3 || !still) {
				continue;
			}
			Vec3 up_M = sensor_to_model(acc / n).normalized();
			double best = 0.5;
			Quat q_HM;
			bool found = false;
			for (int k = 0; k < kCameras; k++) {
				ImageView img = view(s.cam[k]);
				InitInput in;
				in.hand = hand;
				InitBlobs prep = prepare_init_clusters(img, detect_init_blobs(img, scene_is_bright(view(s.cam[3]))), in);
				for (const InitCluster &cl : prep.clusters) {
					if (cl.blobs.size() < 5) {
						continue;
					}
					std::vector<BlobPoint> five(cl.blobs.begin(), cl.blobs.begin() + 5);
					std::vector<Vec3> bearings;
					std::vector<Vec2> uv;
					for (const BlobPoint &b : five) {
						Vec3 f = cams[k].cam2world(Vec2(b.x, b.y));
						bearings.push_back(f);
						uv.emplace_back(f.x() / f.z(), f.y() / f.z());
					}
					for (const auto &h : table.lookup(bearings, hand == kRight)) {
						std::vector<Vec2> u;
						std::vector<Vec3> P;
						for (auto [blob, led] : h) {
							u.push_back(uv[blob]);
							P.push_back(models[hand].position[led]);
						}
						Pose T_CM;
						if (!epnp(u, P, T_CM)) {
							continue;
						}
						double e = 0;
						for (auto [blob, led] : h) {
							Vec2 q;
							cams[k].world2cam(T_CM * models[hand].position[led], q);
							e += (q - Vec2(five[blob].x, five[blob].y)).norm();
						}
						e /= h.size();
						if (e < best) {
							best = e;
							q_HM = cams[k].T_HC.q * T_CM.q;
							found = true;
						}
					}
				}
			}
			if (found) {
				sum += q_HM * up_M;
				used++;
			}
		}
	}
	if (used == 0) {
		return false;
	}
	up_H = sum.normalized();
	return true;
}

struct Summary
{
	int frames_with_body = 0;
	int tracked = 0;
	int acquisitions = 0;
	int became_tracked = 0;
	int lost = 0;
	std::map<int, int> methods;
	int accepted = 0;
	std::vector<double> residuals;
};

double
median(std::vector<double> v)
{
	if (v.empty()) {
		return NAN;
	}
	std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
	return v[v.size() / 2];
}

int
usage()
{
	fprintf(stderr, "usage: pico4-ctrack-replay [--calib DIR] [--slots a,b,c,d] [--head-up X,Y,Z | --estimate-head-up]\n"
	                "                           [--trace FILE] [--max-imu-gap S] [--delay left|right S] [--skip S]\n"
	                "                           LEDS.bin IMU.bin\n");
	return 2;
}

} // namespace

int
main(int argc, char **argv)
{
	std::string calib = "/usr/share/qcom/pico4/calibration", trace_path;
	int slots[kCameras] = {0, 2, 1, 3};
	Vec3 up_H = Vec3::Zero();
	bool estimate = false;
	FusionParams params;
	int delay_hand = -1;
	double delay_s = 0, skip_s = 0;
	std::vector<std::string> files;
	for (int i = 1; i < argc; i++) {
		std::string a = argv[i];
		if (a == "--calib" && i + 1 < argc) {
			calib = argv[++i];
		} else if (a == "--slots" && i + 1 < argc) {
			if (sscanf(argv[++i], "%d,%d,%d,%d", &slots[0], &slots[1], &slots[2], &slots[3]) != 4) {
				return usage();
			}
		} else if (a == "--head-up" && i + 1 < argc) {
			if (sscanf(argv[++i], "%lf,%lf,%lf", &up_H.x(), &up_H.y(), &up_H.z()) != 3 || up_H.norm() == 0) {
				return usage();
			}
		} else if (a == "--estimate-head-up") {
			estimate = true;
		} else if (a == "--max-imu-gap" && i + 1 < argc) {
			params.max_imu_gap = atof(argv[++i]);
			if (!(params.max_imu_gap > 0)) {
				return usage();
			}
		} else if (a == "--delay" && i + 2 < argc) {
			std::string h = argv[++i];
			delay_hand = h == "left" ? kLeft : h == "right" ? kRight : -1;
			delay_s = atof(argv[++i]);
			if (delay_hand < 0 || !(delay_s > 0)) {
				return usage();
			}
		} else if (a == "--skip" && i + 1 < argc) {
			skip_s = atof(argv[++i]);
			if (!(skip_s > 0)) {
				return usage();
			}
		} else if (a == "--trace" && i + 1 < argc) {
			trace_path = argv[++i];
		} else if (!a.empty() && a[0] != '-') {
			files.push_back(a);
		} else {
			return usage();
		}
	}
	if (files.size() != 2 || (up_H.norm() > 0 && estimate)) {
		return usage();
	}

	std::array<Camera, kCameras> cams;
	LedModel models[2];
	std::vector<LedFrame> frames;
	std::array<std::vector<ImuSample>, 2> imu;
	std::string error;
	if (!load_cameras(calib, cams, error) ||
	    !load_led_models(calib + "/controllers/led-models.json", models[0], models[1], error)) {
		fprintf(stderr, "%s\n", error.c_str());
		return 1;
	}
	Vec3 head_accel;
	int head_accel_n;
	if (!read_led_frames(files[0], frames) || !read_imu(files[1], imu, head_accel, head_accel_n)) {
		return 1;
	}
	std::vector<FrameSet> sets = assemble(frames, slots);
	if (skip_s > 0 && !sets.empty()) {
		int64_t until = sets.front().t_ns + (int64_t)(skip_s * 1e9);
		sets.erase(std::remove_if(sets.begin(), sets.end(), [until](const FrameSet &s) { return s.t_ns < until; }),
		           sets.end());
	}
	if (delay_hand >= 0 && !sets.empty()) {
		int64_t until = sets.front().t_ns + (int64_t)(delay_s * 1e9);
		auto &v = imu[delay_hand];
		v.erase(std::remove_if(v.begin(), v.end(), [until](const ImuSample &s) { return s.t_ns < until; }), v.end());
	}
	int head_frames = 0;
	if (estimate && !estimate_head_up(cams, models, sets, imu, up_H, head_frames)) {
		fprintf(stderr, "no still controller with an identified LED pattern: pass --head-up\n");
		return 1;
	}
	if (up_H.norm() == 0) {
		if (head_accel_n == 0) {
			fprintf(stderr, "no head accelerometer in %s: pass --head-up or --estimate-head-up\n", files[1].c_str());
			return 1;
		}
		up_H = head_accel / head_accel_n; // the reaction to gravity points up
	}
	Pose T_WH = head_pose(up_H);

	FILE *trace = nullptr;
	if (!trace_path.empty()) {
		trace = fopen(trace_path.c_str(), "w");
		if (trace == nullptr) {
			perror(trace_path.c_str());
			return 1;
		}
		fprintf(trace, "t_ns,hand,body,tracked_before,init,method,camera,tracking_ok,few,near,features,inliers,"
		               "residual_mean,accepted,tracked_after,x,y,z,qw,qx,qy,qz,out_x,out_y,out_z,prior_x,prior_y,"
		               "prior_z\n");
	}

	Tracker tracker(cams, models[0], models[1], params);
	std::array<size_t, 2> next{0, 0};
	Summary sum[2];
	for (const FrameSet &s : sets) {
		for (int hand = 0; hand < 2; hand++) {
			while (next[hand] < imu[hand].size() && imu[hand][next[hand]].t_ns <= s.t_ns) {
				tracker.add_imu((Hand)hand, imu[hand][next[hand]++], &T_WH);
			}
		}
		std::array<ImageView, kCameras> images;
		for (int k = 0; k < kCameras; k++) {
			images[k] = view(s.cam[k]);
		}
		tracker.process_frame(s.t_ns, T_WH, images);
		for (int hand = 0; hand < 2; hand++) {
			const HandFrameStats &st = tracker.stats((Hand)hand);
			Summary &m = sum[hand];
			m.frames_with_body += st.has_body;
			m.tracked += st.tracked_after;
			m.acquisitions += st.init && st.method >= 0;
			m.became_tracked += !st.tracked_before && st.tracked_after;
			m.lost += st.tracked_before && !st.tracked_after;
			if (st.method >= 0) {
				m.methods[st.method]++;
			}
			m.accepted += st.accepted;
			if (st.tracking_ok) {
				m.residuals.push_back(st.residual_mean);
			}
			if (trace != nullptr) {
				OutputPose out = tracker.pose_at((Hand)hand, s.t_ns);
				Vec3 h = (T_WH.inverse() * st.T_WM).t, o = (T_WH.inverse() * out.T_WM).t;
				Vec3 pr = (T_WH.inverse() * st.prior).t;
				Quat q = st.T_WM.q;
				fprintf(trace, "%lld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%d,%d,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,"
				               "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
				        (long long)s.t_ns, hand, st.has_body, st.tracked_before, st.init, st.method, st.init_camera,
				        st.tracking_ok, st.few_points, st.near, st.features, st.inliers, st.residual_mean,
				        st.accepted, st.tracked_after, h.x(), h.y(), h.z(), q.w(), q.x(), q.y(), q.z(),
				        out.valid ? o.x() : NAN, out.valid ? o.y() : NAN, out.valid ? o.z() : NAN,
				        st.tracked_before ? pr.x() : NAN, st.tracked_before ? pr.y() : NAN,
				        st.tracked_before ? pr.z() : NAN);
			}
		}
	}
	if (trace != nullptr) {
		fclose(trace);
	}

	printf("{\n  \"frame_sets\": %zu,\n  \"head_up\": [%.4f, %.4f, %.4f],\n  \"head_up_frames\": %d,\n  \"hands\": {",
	       sets.size(), up_H.x(), up_H.y(), up_H.z(), head_frames);
	for (int hand = 0; hand < 2; hand++) {
		const Summary &m = sum[hand];
		printf("%s\n    \"%s\": {\"imu_samples\": %zu, \"frames_with_imu\": %d, \"tracked_frames\": %d, "
		       "\"acquisitions\": %d, \"became_tracked\": %d, \"lost\": %d, \"accepted\": %d, "
		       "\"median_residual_px\": %.3f, \"methods\": {",
		       hand ? "," : "", hand ? "right" : "left", imu[hand].size(), m.frames_with_body, m.tracked,
		       m.acquisitions, m.became_tracked, m.lost, m.accepted, median(m.residuals));
		bool first = true;
		for (auto [k, v] : m.methods) {
			printf("%s\"%d\": %d", first ? "" : ", ", k, v);
			first = false;
		}
		printf("}}");
	}
	printf("\n  }\n}\n");
	return 0;
}
