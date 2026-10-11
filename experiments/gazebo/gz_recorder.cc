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
// Stream mode (T-S3-03, a ROS 2 node drives it): gz_recorder --stream <plan.txt> <timeout_s>.
// The plan has n_keyframes = 0 and no poses; once the world is quiescent the recorder writes
// "RDY\n" to stdout, then for every n_rigs pose lines read from stdin it steps one keyframe and
// writes the sensors' frames to stdout (float32, sensors in plan order). Ends at end of input.
// Messages go to stderr.
//
// For each keyframe: set every rig pose (/world/<w>/set_pose), step kStepsPerKeyframe
// physics iterations (/world/<w>/control), and keep, for every sensor, the first message
// whose sim-time stamp reaches the last iteration. The control service replies
// before the steps are executed, so completion is detected from the stamps (a
// message counter is not enough: late renders of the previous keyframe would
// be taken for the new one; docs/LOG.md L28). A render can show the scene of an
// earlier iteration than its stamp; with two iterations a stray step left whole
// recordings one keyframe late with on-schedule stamps (LOG L39), so four are stepped.
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
#include <cstdint>
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

constexpr unsigned int kStepsPerKeyframe = 4;  // see the header comment (LOG L39)

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

const int64_t kStepNs = 1000000;  // physics step of the generated world (1 ms)

// One keyframe: set the rig poses (n_rigs x [x, y, z, yaw]), step, and wait until every sensor
// has rendered the last iteration; sim_ns advances. False (with a message) on any failure.
bool StepKeyframe(gz::transport::Node& node, const std::string& world,
                  const std::vector<std::string>& rigs, const double* poses,
                  std::vector<std::unique_ptr<Sensor>>& sensors, int64_t& sim_ns,
                  double timeout_s, size_t k) {
  // Every sensor must sit at the expected sim time before the keyframe's steps. If the world
  // ran ahead (a step still queued), the renders of the previous keyframe would already meet
  // the target below and be stored as this keyframe's (LOG L39).
  for (auto& s : sensors) {
    std::lock_guard<std::mutex> lk(s->mu);
    if (s->stamp_ns != sim_ns) {
      std::cerr << "gz_recorder: " << s->topic << " stamped " << s->stamp_ns
                << " ns before keyframe " << k << ", expected " << sim_ns << "\n";
      return false;
    }
  }
  for (size_t r = 0; r < rigs.size(); ++r) {
    const double* p = &poses[r * 4];
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
      return false;
    }
  }
  const int64_t target_ns = sim_ns + kStepsPerKeyframe * kStepNs;
  if (!Step(node, world, kStepsPerKeyframe)) {
    std::cerr << "gz_recorder: step failed at keyframe " << k << "\n";
    return false;
  }
  // The render of the last iteration is after the pose update, with margin.
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  for (;;) {
    bool all = true;
    for (auto& s : sensors) {
      std::lock_guard<std::mutex> lk(s->mu);
      all = all && s->stamp_ns >= target_ns;
    }
    if (all) break;
    if (std::chrono::steady_clock::now() > deadline) {
      std::cerr << "gz_recorder: sensor timeout at keyframe " << k << "\n";
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  sim_ns = target_ns;
  for (auto& s : sensors) {
    std::lock_guard<std::mutex> lk(s->mu);
    if (s->stamp_ns != target_ns) {  // the world ran ahead of the plan
      std::cerr << "gz_recorder: " << s->topic << " stamped " << s->stamp_ns << " ns at keyframe "
                << k << ", expected " << target_ns << "\n";
      return false;
    }
    if (s->data.size() != s->n_values) {
      std::cerr << "gz_recorder: " << s->topic << " has " << s->data.size()
                << " values, expected " << s->n_values << "\n";
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  const bool stream = argc == 4 && std::string(argv[1]) == "--stream";
  if (argc != 4) {
    std::cerr << "usage: gz_recorder <plan.txt> <out_dir> <timeout_s>\n"
                 "       gz_recorder --stream <plan.txt> <timeout_s>\n";
    return 2;
  }
  std::ifstream plan(stream ? argv[2] : argv[1]);
  const std::string out_dir = stream ? "" : argv[2];
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

  // Take the sim time only once the world is quiescent: every sensor at the same stamp,
  // unchanged for a second. Steps requested above may still be queued (the control service
  // replies first), and a sim time read while they run is too early.
  int64_t sim_ns = -1;
  for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    int64_t lo = INT64_MAX, hi = -1;
    for (auto& s : sensors) {
      std::lock_guard<std::mutex> lk(s->mu);
      lo = std::min(lo, s->stamp_ns);
      hi = std::max(hi, s->stamp_ns);
    }
    if (lo == hi && hi == sim_ns) break;
    sim_ns = hi;
    if (elapsed() > 180.0) {
      std::cerr << "gz_recorder: world not quiescent after 180 s\n";
      return 1;
    }
  }
  if (stream) {
    std::fwrite("RDY\n", 1, 4, stdout);
    std::fflush(stdout);
    std::vector<double> pose(n_rigs * 4);
    for (size_t k = 0;; ++k) {
      for (auto& v : pose) {
        if (!(std::cin >> v)) return 0;  // end of input: the run is over
      }
      if (!StepKeyframe(node, world, rigs, pose.data(), sensors, sim_ns, timeout_s, k)) return 1;
      for (auto& s : sensors) {
        std::lock_guard<std::mutex> lk(s->mu);
        std::fwrite(s->data.data(), sizeof(float), s->data.size(), stdout);
      }
      std::fflush(stdout);
    }
  }

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
    if (!StepKeyframe(node, world, rigs, &poses[k * n_rigs * 4], sensors, sim_ns, timeout_s, k))
      return 1;
    for (size_t i = 0; i < n_sensors; ++i) {
      std::lock_guard<std::mutex> lk(sensors[i]->mu);
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
