// Tier-2 recorder: drives a paused Gazebo Harmonic world over gz-transport and
// records range data of kinematic sensor rigs (docs/LOG.md, experiments/gazebo/README.md).
//
// Input  (plain text, written by experiments/gazebo/record.py):
//   line 1: <world_name> <n_keyframes> <n_rigs> <n_sensors>
//   then n_sensors lines: <topic> <kind: scan|depth> <n_values>
//   then n_rigs lines:    <model_name>
//   then n_keyframes * n_rigs lines: <x> <y> <z> <yaw>   (world ENU, m / rad)
// Output: one binary file per sensor, <out_dir>/<index>.f32, n_keyframes * n_values
//   float32 (little endian). Scan: gz LaserScan ranges (row-major: vertical, horizontal);
//   depth: R_FLOAT32 image (row-major). +inf = no return.
//
// For each keyframe: set every rig pose (/world/<w>/set_pose), step two physics
// iterations (/world/<w>/control), and keep, for every sensor, the first message
// whose sim-time stamp reaches the second iteration. The control service replies
// before the steps are executed, so completion is detected from the stamps (a
// message counter is not enough: late renders of the previous keyframe would
// be taken for the new one; docs/LOG.md L28).
//
// Build: see experiments/gazebo/Makefile. No ROS dependency (AGENTS.md §3).

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/transport/Node.hh>

namespace {

struct Sensor {
  std::string topic;
  std::string kind;
  size_t n_values = 0;
  std::mutex mu;
  uint64_t count = 0;
  int64_t stamp_ns = -1;
  std::vector<float> data;
};

int64_t StampNs(const gz::msgs::Header& h) {
  return static_cast<int64_t>(h.stamp().sec()) * 1000000000LL + h.stamp().nsec();
}

bool Step(gz::transport::Node& node, const std::string& world, unsigned int steps) {
  gz::msgs::WorldControl req;
  req.set_pause(true);
  req.set_multi_step(steps);
  gz::msgs::Boolean rep;
  bool result = false;
  const bool ok = node.Request("/world/" + world + "/control", req, 5000, rep, result);
  return ok && result && rep.data();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: gz_recorder <plan.txt> <out_dir> <timeout_s>\n";
    return 2;
  }
  std::ifstream plan(argv[1]);
  const std::string out_dir = argv[2];
  const double timeout_s = std::stod(argv[3]);
  std::string world;
  size_t n_kf = 0, n_rigs = 0, n_sensors = 0;
  plan >> world >> n_kf >> n_rigs >> n_sensors;
  std::vector<std::unique_ptr<Sensor>> sensors;
  for (size_t i = 0; i < n_sensors; ++i) {
    auto s = std::make_unique<Sensor>();
    plan >> s->topic >> s->kind >> s->n_values;
    sensors.push_back(std::move(s));
  }
  std::vector<std::string> rigs(n_rigs);
  for (auto& r : rigs) plan >> r;
  std::vector<double> poses(n_kf * n_rigs * 4);
  for (auto& v : poses) plan >> v;
  if (!plan) {
    std::cerr << "gz_recorder: malformed plan file\n";
    return 2;
  }

  gz::transport::Node node;
  for (auto& sp : sensors) {
    Sensor* s = sp.get();
    bool ok = false;
    if (s->kind == "scan") {
      ok = node.Subscribe<gz::msgs::LaserScan>(
          s->topic, [s](const gz::msgs::LaserScan& m) {
            std::lock_guard<std::mutex> lk(s->mu);
            if (StampNs(m.header()) < s->stamp_ns) return;  // stale, out-of-order render
            s->data.assign(m.ranges().begin(), m.ranges().end());
            s->stamp_ns = StampNs(m.header());
            ++s->count;
          });
    } else {
      ok = node.Subscribe<gz::msgs::Image>(s->topic, [s](const gz::msgs::Image& m) {
        std::lock_guard<std::mutex> lk(s->mu);
        if (StampNs(m.header()) < s->stamp_ns) return;  // stale, out-of-order render
        const auto& raw = m.data();
        s->data.resize(raw.size() / sizeof(float));
        std::memcpy(s->data.data(), raw.data(), s->data.size() * sizeof(float));
        s->stamp_ns = StampNs(m.header());
        ++s->count;
      });
    }
    if (!ok) {
      std::cerr << "gz_recorder: cannot subscribe to " << s->topic << "\n";
      return 1;
    }
  }

  // Wait for the world and a first message from every sensor.
  const auto t_start = std::chrono::steady_clock::now();
  auto elapsed = [&]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  };
  for (;;) {
    Step(node, world, 1);
    bool all = true;
    for (auto& s : sensors) {
      std::lock_guard<std::mutex> lk(s->mu);
      all = all && s->count > 0;
    }
    if (all) break;
    if (elapsed() > 180.0) {
      std::cerr << "gz_recorder: sensors silent after 180 s\n";
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Let pending renders arrive, then take the current sim time from the stamps.
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  int64_t sim_ns = 0;
  for (auto& s : sensors) {
    std::lock_guard<std::mutex> lk(s->mu);
    sim_ns = std::max(sim_ns, s->stamp_ns);
  }
  const int64_t step_ns = 1000000;  // physics step of the generated world (1 ms)

  std::vector<std::FILE*> files;
  for (size_t i = 0; i < n_sensors; ++i) {
    files.push_back(std::fopen((out_dir + "/" + std::to_string(i) + ".f32").c_str(), "wb"));
    if (!files.back()) {
      std::cerr << "gz_recorder: cannot open output\n";
      return 1;
    }
  }
  const auto t_rec = std::chrono::steady_clock::now();
  for (size_t k = 0; k < n_kf; ++k) {
    for (size_t r = 0; r < n_rigs; ++r) {
      const double* p = &poses[(k * n_rigs + r) * 4];
      gz::msgs::Pose req;
      req.set_name(rigs[r]);
      req.mutable_position()->set_x(p[0]);
      req.mutable_position()->set_y(p[1]);
      req.mutable_position()->set_z(p[2]);
      req.mutable_orientation()->set_x(0.0);
      req.mutable_orientation()->set_y(0.0);
      req.mutable_orientation()->set_z(std::sin(0.5 * p[3]));
      req.mutable_orientation()->set_w(std::cos(0.5 * p[3]));
      gz::msgs::Boolean rep;
      bool result = false;
      if (!node.Request("/world/" + world + "/set_pose", req, 5000, rep, result) || !result ||
          !rep.data()) {
        std::cerr << "gz_recorder: set_pose failed for " << rigs[r] << " at keyframe " << k << "\n";
        return 1;
      }
    }
    const int64_t target_ns = sim_ns + 2 * step_ns;
    if (!Step(node, world, 2)) {
      std::cerr << "gz_recorder: step failed at keyframe " << k << "\n";
      return 1;
    }
    // The render of the second iteration is strictly after the pose update.
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    for (;;) {
      bool all = true;
      for (size_t i = 0; i < n_sensors; ++i) {
        std::lock_guard<std::mutex> lk(sensors[i]->mu);
        all = all && sensors[i]->stamp_ns >= target_ns;
      }
      if (all) break;
      if (std::chrono::steady_clock::now() > deadline) {
        std::cerr << "gz_recorder: sensor timeout at keyframe " << k << "\n";
        return 1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    sim_ns = target_ns;
    for (size_t i = 0; i < n_sensors; ++i) {
      std::lock_guard<std::mutex> lk(sensors[i]->mu);
      if (sensors[i]->data.size() != sensors[i]->n_values) {
        std::cerr << "gz_recorder: " << sensors[i]->topic << " has " << sensors[i]->data.size()
                  << " values, expected " << sensors[i]->n_values << "\n";
        return 1;
      }
      std::fwrite(sensors[i]->data.data(), sizeof(float), sensors[i]->data.size(), files[i]);
    }
    if (k % 50 == 0) {
      const double el =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t_rec).count();
      std::printf("[gz_recorder] keyframe %zu/%zu  %.0f s\n", k, n_kf, el);
      std::fflush(stdout);
    }
  }
  for (auto* f : files) std::fclose(f);
  std::printf("[gz_recorder] done\n");
  return 0;
}
